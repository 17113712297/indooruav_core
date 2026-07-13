#!/usr/bin/env python3
"""
mode_manager.py — ROS 模式管理器节点

处理来自 Android/MSDK 端的模式指令，管理对应进程生命周期和配置文件修改。
通过 /indooruav_core/mode_manager/command 服务响应指令。

指令列表:
  建图模式:  mapping_set_name, mapping_start, mapping_save_map, mapping_stop
  采点模式:  collect_set_map, collect_set_wp_name, collect_start,
            collect_gen_2d, collect_gen_pixel, collect_stop
  巡航模式:  cruise_set_map, cruise_set_wp, cruise_start
  文件查询:  list_maps, list_waypoints
"""

import glob
import os
import re
import signal
import subprocess
import time
import traceback

import rospy
from indooruav_msgs.srv import ModeCommand, ModeCommandResponse

# ============================================================
# Configuration — must match ros_launcher_gui.py paths
# ============================================================

WORKSPACE = os.path.expanduser("~/Project/IndoorUavInspection2/catkin_ws")
THREE_D_WORKSPACE = os.path.expanduser("~/Project/3D/catkin_ws")
SHELL_DIR = os.path.join(WORKSPACE, "src", "shell")
SETUP_BASH = os.path.join(WORKSPACE, "devel", "setup.bash")
THREE_D_SETUP_BASH = os.path.join(THREE_D_WORKSPACE, "devel", "setup.bash")

MAP_DIR = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "map3d")
WAYPOINT_DIR = os.path.join(WORKSPACE, "src", "indooruav_waypoint", "waypoints")
LOCALIZE_YAML = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "config", "localize.yaml")
WAYPOINT_YAML = os.path.join(WORKSPACE, "src", "indooruav_waypoint", "config", "config.yaml")


class ModeManager:
    def __init__(self):
        # Track launched processes: {launch_key: Popen}
        self.processes = {}
        self._env_cache = None

        # Service server
        self.service = rospy.Service(
            "/indooruav_core/mode_manager/command",
            ModeCommand,
            self.handle_command
        )

        # ensure log dirs exist
        for d in ["log/bringup_mapping", "log/bringup_localize",
                  "log/bringup_waypoint_recorder"]:
            os.makedirs(os.path.join(WORKSPACE, d), exist_ok=True)

        rospy.loginfo("[ModeManager] ready")

    # ── env sourcing (same as ros_launcher_gui.py) ────────────
    def _get_env(self):
        if self._env_cache is not None:
            return dict(self._env_cache)
        try:
            cmd = (f'source "{THREE_D_SETUP_BASH}" && '
                   f'source "{SETUP_BASH}" --extend && env')
            output = subprocess.check_output(
                cmd, shell=True, executable="/bin/bash",
                stderr=subprocess.PIPE, timeout=30)
            env = dict(os.environ)
            for line in output.decode("utf-8").splitlines():
                if "=" in line:
                    k, _, v = line.partition("=")
                    env[k] = v
            env["ROSCONSOLE_STDOUT_LINE_BUFFERED"] = "1"
            env["PYTHONUNBUFFERED"] = "1"
            self._env_cache = env
            return dict(env)
        except subprocess.TimeoutExpired:
            return os.environ.copy()
        except Exception:
            traceback.print_exc()
            return os.environ.copy()

    # ── process lifecycle helpers ─────────────────────────────
    def _launch_bash(self, key, script_path, args=None, timeout=3.0):
        """Launch a shell script via bash, tracking it by key."""
        if key in self.processes and self.processes[key].poll() is None:
            rospy.logwarn("[ModeManager] '%s' already running, skip", key)
            return True
        cmd = ["bash", script_path]
        if args:
            cmd.extend(args)
        try:
            env = self._get_env()
            proc = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.processes[key] = proc
            time.sleep(0.3)
            if proc.poll() is not None:
                rospy.logerr("[ModeManager] '%s' exited immediately (rc=%d)", key, proc.returncode)
                self.processes.pop(key, None)
                return False
            rospy.loginfo("[ModeManager] '%s' started (pid=%d)", key, proc.pid)
            return True
        except Exception as e:
            rospy.logerr("[ModeManager] failed to start '%s': %s", key, e)
            return False

    def _launch_roslaunch(self, key, pkg, launch_file, extra_args=None):
        """Launch a roslaunch directly, tracking by key."""
        if key in self.processes and self.processes[key].poll() is None:
            return True
        env = self._get_env()
        cmd = ["roslaunch", "--screen", pkg, launch_file]
        if extra_args:
            cmd.extend(extra_args)
        try:
            proc = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.processes[key] = proc
            time.sleep(1.0)
            if proc.poll() is not None:
                rospy.logerr("[ModeManager] roslaunch '%s' exited immediately (rc=%d)", key, proc.returncode)
                self.processes.pop(key, None)
                return False
            rospy.loginfo("[ModeManager] roslaunch '%s' started (pid=%d)", key, proc.pid)
            return True
        except Exception as e:
            rospy.logerr("[ModeManager] failed to roslaunch '%s': %s", key, e)
            return False

    def _kill_process(self, key):
        """Stop a tracked process by key."""
        proc = self.processes.get(key)
        if proc and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    proc.wait()
                except ProcessLookupError:
                    pass
            except ProcessLookupError:
                pass
            self.processes.pop(key, None)
            rospy.loginfo("[ModeManager] killed '%s'", key)
            return True
        self.processes.pop(key, None)
        return False

    def _kill_group(self, key_prefix):
        """Stop all processes whose key starts with prefix."""
        for key in list(self.processes.keys()):
            if key.startswith(key_prefix):
                self._kill_process(key)

    def _run_rosservice(self, service_name, args="{}", timeout_s=15):
        """Run rosservice call."""
        try:
            env = self._get_env()
            result = subprocess.run(
                ["rosservice", "call", service_name, args],
                env=env, capture_output=True, text=True, timeout=timeout_s)
            out = (result.stdout + result.stderr).strip()
            success = (
                "success: True" in out or "success: true" in out
                or result.returncode == 0)
            rospy.loginfo("[ModeManager] rosservice %s → %s", service_name, out[:200])
            return success, out
        except subprocess.TimeoutExpired:
            rospy.logerr("[ModeManager] rosservice %s timed out", service_name)
            return False, "timeout"
        except Exception as e:
            rospy.logerr("[ModeManager] rosservice %s failed: %s", service_name, e)
            return False, str(e)

    def _modify_yaml(self, yaml_path, pattern, replacement):
        """Edit a YAML file's first matching line."""
        try:
            with open(yaml_path, "r") as f:
                content = f.read()
            new_content = re.sub(pattern, replacement, content, count=1)
            if new_content == content:
                return f"no match found for {pattern}"
            with open(yaml_path, "w") as f:
                f.write(new_content)
            rospy.loginfo("[ModeManager] updated %s", yaml_path)
            return ""
        except Exception as e:
            return str(e)

    def _scan_pcd_files(self):
        """Return sorted list of PCD filenames."""
        try:
            def sort_key(f):
                name = os.path.splitext(os.path.basename(f))[0]
                return (0, int(name)) if name.isdigit() else (1, name)
            files = sorted(glob.glob(os.path.join(MAP_DIR, "*.pcd")), key=sort_key)
            return [os.path.basename(f) for f in files]
        except Exception as e:
            rospy.logerr("[ModeManager] scan map dir: %s", e)
            return []

    def _scan_waypoint_files(self):
        """Return sorted list of waypoints*.yaml filenames."""
        try:
            files = sorted(glob.glob(os.path.join(WAYPOINT_DIR, "waypoints*.yaml")))
            return [os.path.basename(f) for f in files]
        except Exception as e:
            rospy.logerr("[ModeManager] scan waypoint dir: %s", e)
            return []

    # ── command handlers ──────────────────────────────────────
    def _handle_mapping_set_name(self, payload):
        """Store map name — no YAML change needed for mapping."""
        self._current_map_name = payload.strip() or "my_map"
        return True, "ok"

    def _handle_mapping_start(self, payload):
        """Start MID360 LiDAR + mapping node."""
        self._kill_group("mapping_")
        ok1 = self._launch_roslaunch("mapping_lidar", "livox_ros_driver2",
                                     "msg_MID360.launch")
        ok2 = self._launch_bash("mapping_node", os.path.join(SHELL_DIR, "bringup_mapping.sh"))
        if ok1 or ok2:
            return True, "mapping started"
        return False, "failed to start mapping"

    def _handle_mapping_save_map(self, payload):
        """Save map via /save_map service."""
        map_name = payload.strip() or getattr(self, "_current_map_name", "my_map")
        if not map_name.endswith(".pcd"):
            map_name += ".pcd"
        success, out = self._run_rosservice("/save_map", f"save_path: '{map_name}'")
        return success, out

    def _handle_mapping_stop(self, payload):
        """Stop LiDAR + mapping."""
        self._kill_group("mapping_")
        return True, "mapping stopped"

    def _handle_list_maps(self, payload):
        """Return comma-separated list of PCD files."""
        files = self._scan_pcd_files()
        return True, ",".join(files)

    def _handle_list_waypoints(self, payload):
        """Return comma-separated list of waypoint files."""
        files = self._scan_waypoint_files()
        return True, ",".join(files)

    # ── 采点模式 handlers ───────────────────────────────────
    def _handle_collect_set_map(self, payload):
        """Set localization map PCD file by modifying localize.yaml."""
        selected = payload.strip()
        if not selected:
            return False, "地图名为空"
        err = self._modify_yaml(
            LOCALIZE_YAML,
            r'(pcd_name:\s*)(?:"[^"]*"|\S+)',
            r'\g<1>"{}"'.format(selected)
        )
        if err:
            return False, err
        return True, f"定位地图已设置为: {selected}"

    def _handle_collect_set_wp_name(self, payload):
        """Set waypoint recorder filename by modifying config.yaml."""
        filename = payload.strip()
        if not filename:
            return False, "文件名为空"
        err = self._modify_yaml(
            WAYPOINT_YAML,
            r'(?s)(waypoint_recorder:.*?waypoints_file_path:\s*)[^\n]+',
            r'\g<1>waypoints/{}'.format(filename),
        )
        if err:
            return False, err
        return True, f"航点文件名已设置为: waypoints/{filename}"

    def _handle_collect_start(self, payload):
        """Start LiDAR + localization + waypoint recorder."""
        self._kill_group("collect_")
        ok1 = self._launch_roslaunch("collect_lidar", "livox_ros_driver2",
                                     "msg_MID360.launch")
        ok2 = self._launch_bash("collect_localize", os.path.join(SHELL_DIR, "bringup_localize.sh"))
        ok3 = self._launch_bash("collect_recorder", os.path.join(SHELL_DIR, "bringup_waypoint_recorder.sh"),
                                ["direct"])
        if ok1 or ok2 or ok3:
            return True, "采点模式已启动"
        return False, "启动采点模式失败"

    def _handle_collect_gen_2d(self, payload):
        """Generate 2D map from PCD (pcd_to_2d.py)."""
        script = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "scripts", "pcd_to_2d.py")
        try:
            env = self._get_env()
            proc = subprocess.Popen(
                ["python3", "-u", script],
                env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.processes["collect_pcd2d"] = proc
            rospy.loginfo("[ModeManager] pcd_to_2d.py started (pid=%d)", proc.pid)
            return True, "2D地图生成已启动"
        except Exception as e:
            return False, f"启动2D地图生成失败: {e}"

    def _handle_collect_gen_pixel(self, payload):
        """Generate pixel coordinates (odometry_to_pixel_offline.py) from current waypoints file."""
        script = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "scripts",
                              "odometry_to_pixel_offline.py")
        try:
            # 从 config.yaml 读取当前 waypoints 文件路径
            import yaml
            with open(WAYPOINT_YAML, "r") as f:
                cfg = yaml.safe_load(f)
            rel_path = cfg.get("indooruav_waypoint", {}).get("waypoint_recorder", {}).get(
                "waypoints_file_path", "waypoints/waypoints.yaml")
            wp_path = os.path.join(WORKSPACE, "src", "indooruav_waypoint", rel_path)
            if not os.path.isfile(wp_path):
                rospy.logwarn("[ModeManager] pixel: waypoints file not found: %s", wp_path)
                return False, f"航点文件不存在: {wp_path}"

            env = self._get_env()
            proc = subprocess.Popen(
                ["python3", "-u", script, wp_path],
                env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.processes["collect_pixel"] = proc
            rospy.loginfo("[ModeManager] odometry_to_pixel_offline.py started (pid=%d) file=%s",
                          proc.pid, wp_path)
            return True, "像素坐标生成已启动"
        except Exception as e:
            return False, f"启动像素坐标生成失败: {e}"

    def _handle_collect_stop(self, payload):
        """Stop all collect-related processes."""
        self._kill_group("collect_")
        return True, "采点模式已停止"

    # ── 巡航模式 handlers ───────────────────────────────────
    def _handle_cruise_set_map(self, payload):
        """Set localization map by modifying localize.yaml."""
        selected = payload.strip()
        if not selected:
            return False, "地图名为空"
        err = self._modify_yaml(
            LOCALIZE_YAML,
            r'(pcd_name:\s*)(?:"[^"]*"|\S+)',
            r'\g<1>"{}"'.format(selected)
        )
        if err:
            return False, err
        return True, f"定位地图已设置为: {selected}"

    def _handle_cruise_set_wp(self, payload):
        """Set tracker waypoint file by modifying config.yaml (first waypoints_file_path under waypoint_tracker)."""
        filename = payload.strip()
        if not filename:
            return False, "文件名为空"
        # 匹配 waypoint_tracker 下的 waypoints_file_path（第一个）
        err = self._modify_yaml(
            WAYPOINT_YAML,
            r'(waypoint_tracker:.*?waypoints_file_path:\s*)[^\n]+',
            r'\g<1>waypoints/{}'.format(filename),
        )
        if err:
            return False, err
        return True, f"航线文件已设置为: waypoints/{filename}"

    def _handle_cruise_start(self, payload):
        """Send takeoff command via HTTP to trigger state machine."""
        import urllib.request
        import urllib.parse
        try:
            url = "http://localhost:20000/sendTakeoffState"
            params = urllib.parse.urlencode({
                "siteId": 11,
                "deviceId": 1,
                "takeoffState": 1
            })
            full_url = f"{url}?{params}"
            resp = urllib.request.urlopen(full_url, timeout=10)
            body = resp.read().decode("utf-8")
            rospy.loginfo("[ModeManager] cruise_start HTTP response: %s", body)
            if '"resultCode": 0' in body or '"resultCode":0' in body:
                return True, "起飞指令已发送"
            else:
                return False, f"HTTP 返回值异常: {body}"
        except Exception as e:
            rospy.logerr("[ModeManager] cruise_start HTTP failed: %s", e)
            return False, f"HTTP 请求失败: {e}"

    # ── main dispatcher ──────────────────────────────────────
    def handle_command(self, req):
        command = req.command
        payload = req.payload

        rospy.loginfo("[ModeManager] command='%s' payload='%s'", command, payload)

        handlers = {
            "mapping_set_name": self._handle_mapping_set_name,
            "mapping_start": self._handle_mapping_start,
            "mapping_save_map": self._handle_mapping_save_map,
            "mapping_stop": self._handle_mapping_stop,
            "list_maps": self._handle_list_maps,
            "list_waypoints": self._handle_list_waypoints,
            # 采点模式
            "collect_set_map": self._handle_collect_set_map,
            "collect_set_wp_name": self._handle_collect_set_wp_name,
            "collect_start": self._handle_collect_start,
            "collect_gen_2d": self._handle_collect_gen_2d,
            "collect_gen_pixel": self._handle_collect_gen_pixel,
            "collect_stop": self._handle_collect_stop,
            # 巡航模式
            "cruise_set_map": self._handle_cruise_set_map,
            "cruise_set_wp": self._handle_cruise_set_wp,
            "cruise_start": self._handle_cruise_start,
        }

        handler = handlers.get(command)
        if handler is None:
            return ModeCommandResponse(False, f"unknown command: {command}")

        try:
            success, message = handler(payload)
            return ModeCommandResponse(success, message)
        except Exception as e:
            rospy.logerr("[ModeManager] command '%s' exception: %s", command, e)
            traceback.print_exc()
            return ModeCommandResponse(False, str(e))


if __name__ == "__main__":
    rospy.init_node("mode_manager_node", anonymous=True)
    mm = ModeManager()
    rospy.spin()