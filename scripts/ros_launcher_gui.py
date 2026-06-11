#!/usr/bin/env python3
import re
import subprocess
import os
import sys
import signal
import time
import traceback
import threading
import glob
import tkinter as tk
from tkinter import ttk, scrolledtext
from tkinter import font as tkfont

import rospy
from geometry_msgs.msg import Twist

# ============================================================
# Configuration — add entries here to extend
# ============================================================

WORKSPACE = os.path.expanduser("~/Project/IndoorUavInspection2/catkin_ws")
THREE_D_WORKSPACE = os.path.expanduser("~/Project/3D/catkin_ws")
SHELL_DIR = os.path.join(WORKSPACE, "src", "shell")
SETUP_BASH = os.path.join(WORKSPACE, "devel", "setup.bash")
THREE_D_SETUP_BASH = os.path.join(THREE_D_WORKSPACE, "devel", "setup.bash")

MAP_DIR = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "map")
VEL_TOPIC = "indooruav_controller/waypoint_tracker/cmd_vel"
VEL_RATE_HZ = 10.0
VEL_STEP = 0.1  # m/s per click

COMMANDS = {
    "Core": [
        {"label": "启动状态机",   "cmd": ["bash", os.path.join(SHELL_DIR, "bringup_indooruav_core.sh")]},
    ],
    "Controller": [
        {"label": "启动仿真控制", "cmd": ["bash", os.path.join(SHELL_DIR, "bringup_controller_simulate.sh")]},
        {"label": "启动实物控制", "cmd": ["bash", "-c", 'echo "888888" | sudo -S chmod 666 /dev/i2c-7 && echo "888888" | sudo -S chmod 777 /dev/ttyTHS0 && bash ' + os.path.join(SHELL_DIR, "bringup_controller_hardware.sh")]},
    ],
    "SLAM": [
        {"label": "启动MID360",   "cmd": ["roslaunch", os.path.join(THREE_D_WORKSPACE, "src", "livox_ros_driver2", "launch_ROS1", "msg_MID360.launch")]},
        {"label": "启动建图",     "cmd": ["bash", os.path.join(SHELL_DIR, "bringup_mapping.sh")]},
        {"label": "启动定位",     "cmd": ["bash", os.path.join(SHELL_DIR, "bringup_localize.sh")]},
    ],
    "Waypoint": [
        {"label": "启动航线追踪",     "cmd": ["bash", os.path.join(SHELL_DIR, "bringup_waypoint_tracker.sh")]},
        {"label": "启动航线记录(direct)",   "cmd": ["bash", os.path.join(SHELL_DIR, "bringup_waypoint_recorder.sh"), "direct"]},
        {"label": "启动航线记录(path)",     "cmd": ["bash", os.path.join(SHELL_DIR, "bringup_waypoint_recorder.sh"), "path_relay"]},
    ],
    "Test Tools": [
        {"label": "状态机测试",   "cmd": ["bash", os.path.join(SHELL_DIR, "test_state_machine.sh")],               "stdin": True},
        {"label": "仿真控制测试", "cmd": ["bash", os.path.join(SHELL_DIR, "test_controller_simulate.sh")],         "stdin": True},
        {"label": "实物控制测试", "cmd": ["bash", os.path.join(SHELL_DIR, "test_controller_hardware.sh")],         "stdin": True},
        {"label": "里程计旋转",   "cmd": ["bash", os.path.join(SHELL_DIR, "odometry_frame_rotator.sh")]},
        {"label": "航点记录按钮", "cmd": ["bash", os.path.join(SHELL_DIR, "waypoint_record_button.sh")]},
        {"label": "里程计记录",   "cmd": ["python", os.path.join(WORKSPACE, "src", "indooruav_core", "scripts", "odometry_recorder.py")]},
    ],
}

# ============================================================

# ANSI escape sequence patterns (same logic as the sed filter in shell scripts)
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]|\x1b\][^\x07]*\x07")

def _strip_ansi(text):
    return _ANSI_RE.sub("", text)

# ============================================================


class RosLauncher:
    # CJK fonts to try, in order of preference
    _CJK_FONTS = [
        "Microsoft YaHei", "Noto Sans CJK SC", "Noto Sans SC",
        "WenQuanYi Micro Hei", "WenQuanYi Zen Hei",
        "SimHei", "SimSun", "AR PL UMing CN",
    ]

    @classmethod
    def _detect_cjk_font(cls):
        available = set(tkfont.families())
        for name in cls._CJK_FONTS:
            if name in available:
                return name
        return "TkDefaultFont"

    def __init__(self, root):
        self.root = root
        self.root.title("ROS Launch Manager")
        self.processes = {}       # label -> Popen
        self._env_cache = None
        self.outputs = {}         # label -> ScrolledText (in notebook tab)
        self.stdin_entries = {}   # label -> (Entry, Button) for interactive processes
        self.roscore_proc = None  # managed roscore process

        # Velocity control state
        self.vel_enabled = False
        self.vel_pub = None
        self.vel_values = [0.0, 0.0, 0.0, 0.0]  # vx, vy, vz, yaw_rate
        self.vel_thread = None
        self.vel_running = False

        cjk_font = self._detect_cjk_font()
        self._cjk_font = cjk_font
        default_font = (cjk_font, 10)
        bold_font = (cjk_font, 10, "bold")
        self.root.option_add("*Font", default_font)
        style = ttk.Style()
        style.configure(".", font=default_font)
        style.configure("Bold.TLabel", font=bold_font)

        self._build_ui()
        self._start_roscore()

        # Ensure roscore is cleaned up on exit
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _start_roscore(self):
        """Check if roscore is already running; start it if not."""
        # Quick check: try connecting to the master
        try:
            import socket
            master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
            host = master_uri.split("://")[1].split(":")[0]
            port = int(master_uri.split(":")[-1].rstrip("/"))
            s = socket.create_connection((host, port), timeout=1)
            s.close()
            self._dbg("[DEBUG] ROS master already running, skip roscore")
            return
        except Exception:
            pass

        self._dbg("[DEBUG] Starting roscore ...")
        env = os.environ.copy()
        try:
            self.roscore_proc = subprocess.Popen(
                ["roscore"],
                env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                start_new_session=True,
            )
            # Wait briefly for roscore to come up
            time.sleep(2)
            if self.roscore_proc.poll() is not None:
                self._dbg(f"[DEBUG] roscore failed to start, rc={self.roscore_proc.returncode}")
                self.roscore_proc = None
            else:
                self._dbg(f"[DEBUG] roscore started, pid={self.roscore_proc.pid}")
        except Exception as e:
            self._dbg(f"[DEBUG] Failed to start roscore: {e}")
            self.roscore_proc = None

    def _update_roscore_status(self):
        if self.roscore_proc and self.roscore_proc.poll() is None:
            self.roscore_status.config(text="● running", foreground="green")
        else:
            self.roscore_status.config(text="○ not running", foreground="gray")

    def _on_close(self):
        """Clean up all processes, gazebo, and roscore on window close."""
        # Stop velocity control
        self.vel_running = False
        if self.vel_pub:
            self.vel_pub.publish(Twist())

        self._kill_all()
        self._kill_gazebo()
        if self.roscore_proc and self.roscore_proc.poll() is None:
            self._dbg("[DEBUG] Stopping roscore ...")
            try:
                os.killpg(os.getpgid(self.roscore_proc.pid), signal.SIGINT)
                self.roscore_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.roscore_proc.pid), signal.SIGKILL)
                self.roscore_proc.wait()
            except ProcessLookupError:
                pass
        self.root.destroy()

    def _dbg(self, msg):
        """Log to both stderr (terminal) and GUI output widget."""
        print(msg, file=sys.stderr, flush=True)
        self._log(msg)

    def _get_output(self, label, with_stdin=False):
        """Return the ScrolledText for a given process label, creating a tab if needed."""
        if label not in self.outputs:
            frame = ttk.Frame(self.notebook)

            # Pack stdin bar first so it claims space before the expanded text widget
            if with_stdin:
                stdin_frame = ttk.Frame(frame, relief="sunken", borderwidth=1)
                stdin_frame.pack(fill="x", side="bottom", padx=2, pady=(2, 2))
                ttk.Label(stdin_frame, text=" >").pack(side="left")
                entry = ttk.Entry(stdin_frame)
                entry.pack(side="left", fill="x", expand=True, padx=2)
                send_btn = ttk.Button(
                    stdin_frame, text="Send", width=5,
                    command=lambda: self._send_stdin(label),
                )
                send_btn.pack(side="left", padx=2)
                entry.bind("<Return>", lambda e: self._send_stdin(label))
                self.stdin_entries[label] = (entry, send_btn)
                self._dbg(f"[DEBUG] Stdin bar added for '{label}'")

            text = scrolledtext.ScrolledText(frame, state="normal", wrap="word",
                                              font=(self._cjk_font, 10))
            # Block editing (printable chars, backspace, delete, enter) but allow Ctrl+C/Ins/etc
            def _block_edit(e):
                if e.keysym and len(e.keysym) == 1 and e.state == 0:
                    return "break"
                if e.keysym in ("BackSpace", "Delete", "Return", "Tab", "space"):
                    return "break"
            text.bind("<Key>", _block_edit)
            text.pack(fill="both", expand=True)

            self.notebook.add(frame, text=label)
            self.outputs[label] = text
            self.notebook.select(frame)

        elif with_stdin and label not in self.stdin_entries:
            # Tab was created without stdin (e.g. by _log) — add it now
            self._dbg(f"[DEBUG] Late-adding stdin bar for '{label}'")
            text = self.outputs[label]
            frame = text.master  # parent of ScrolledText is the tab frame
            stdin_frame = ttk.Frame(frame, relief="sunken", borderwidth=1)
            stdin_frame.pack(fill="x", side="bottom", padx=2, pady=(2, 2), before=text)
            ttk.Label(stdin_frame, text=" >").pack(side="left")
            entry = ttk.Entry(stdin_frame)
            entry.pack(side="left", fill="x", expand=True, padx=2)
            send_btn = ttk.Button(
                stdin_frame, text="Send", width=5,
                command=lambda: self._send_stdin(label),
            )
            send_btn.pack(side="left", padx=2)
            entry.bind("<Return>", lambda e: self._send_stdin(label))
            self.stdin_entries[label] = (entry, send_btn)

        return self.outputs[label]

    def _get_env(self):
        """Source setup.bash once, cache the merged environment."""
        if self._env_cache is not None:
            return dict(self._env_cache)

        self._dbg(f"[DEBUG] Sourcing {THREE_D_SETUP_BASH} + {SETUP_BASH} ...")
        try:
            cmd = f'source "{THREE_D_SETUP_BASH}" && source "{SETUP_BASH}" --extend && env'
            output = subprocess.check_output(
                cmd, shell=True, executable="/bin/bash",
                stderr=subprocess.PIPE, timeout=30,
            )
            env = dict(os.environ)
            for line in output.decode("utf-8").splitlines():
                if "=" in line:
                    k, _, v = line.partition("=")
                    env[k] = v
            # Force line-buffered output when stdout is a pipe
            env["ROSCONSOLE_STDOUT_LINE_BUFFERED"] = "1"
            env["PYTHONUNBUFFERED"] = "1"
            self._env_cache = env
            self._dbg("[DEBUG] Source ok, env cached")
            return dict(env)
        except subprocess.TimeoutExpired as e:
            self._dbg(f"[DEBUG] Timeout sourcing setup.bash: {e}")
            return os.environ.copy()
        except Exception as e:
            self._dbg(f"[DEBUG] Failed to source setup.bash: {e}")
            traceback.print_exc(file=sys.stderr)
            return os.environ.copy()

    def _find_entry(self, label):
        for items in COMMANDS.values():
            for e in items:
                if e["label"] == label:
                    return e
        return None

    def _launch(self, label):
        self._dbg(f"[DEBUG] _launch('{label}') called")
        entry = self._find_entry(label)
        if not entry:
            self._dbg(f"[DEBUG] _launch: label '{label}' not found in COMMANDS")
            return

        self._dbg(f"[DEBUG] _launch: cmd={entry['cmd']}")

        if label in self.processes and self.processes[label].poll() is None:
            self._dbg(f"[DEBUG] _launch: '{label}' already running (pid={self.processes[label].pid})")
            self._log(f"[WARN] Already running", label=label)
            return

        cmd = entry["cmd"].copy()
        if cmd[0] == "roslaunch" and "--screen" not in cmd:
            cmd.insert(1, "--screen")
        needs_stdin = entry.get("stdin", False)

        # Create output tab (with stdin input if needed) before logging
        self._get_output(label, with_stdin=needs_stdin)

        self._dbg(f"[DEBUG] _launch: launching {' '.join(cmd)}")
        self._log(f"[RUN] {' '.join(cmd)}", label=label)
        entry["_status"].config(text="◉", foreground="green")
        self.root.update_idletasks()

        try:
            env = self._get_env()
            self._dbg(f"[DEBUG] Env acquired, Popen...")
            popen_kwargs = dict(
                env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                start_new_session=True,  # makes the process a session leader, so os.killpg kills all children
            )
            if needs_stdin:
                popen_kwargs["stdin"] = subprocess.PIPE
            proc = subprocess.Popen(cmd, **popen_kwargs)
            self._dbg(f"[DEBUG] Process started, pid={proc.pid}")
            self.processes[label] = proc
            self._stream_output(label, proc, entry)

            # Check if process died immediately
            time.sleep(0.5)
            rc = proc.poll()
            if rc is not None:
                self._dbg(f"[DEBUG] Process {label} exited immediately, rc={rc}")
                # Drain any remaining output
                remaining = proc.stdout.read()
                if remaining:
                    self._dbg(f"[DEBUG] Remaining stdout: {remaining[:500]}")
                entry["_status"].config(text="✗", foreground="red")
                self.processes.pop(label, None)
            else:
                self._dbg(f"[DEBUG] Process {label} still alive after 0.5s")
        except Exception as e:
            self._dbg(f"[DEBUG] Exception in _launch: {e}")
            traceback.print_exc(file=sys.stderr)
            self._log(f"[ERR] Failed to start: {e}", label=label)
            entry["_status"].config(text="✗", foreground="red")

    def _kill(self, label):
        self._dbg(f"[DEBUG] _kill('{label}') called")
        entry = self._find_entry(label)
        proc = self.processes.get(label)

        if proc and proc.poll() is None:
            self._dbg(f"[DEBUG] _kill: sending SIGINT to pgid={os.getpgid(proc.pid)}")
            self._log(f"[KILL] Stopping...", label=label)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._dbg(f"[DEBUG] _kill: timeout, sending SIGKILL")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait()
            del self.processes[label]
            self._dbg(f"[DEBUG] _kill: process ended, rc={proc.returncode}")
        else:
            self._dbg(f"[DEBUG] _kill: proc is None or already dead")

        if label == "启动仿真控制":
            self._kill_gazebo()

        if entry:
            entry["_status"].config(text="○", foreground="black")

    def _kill_gazebo(self):
        """Kill stray gzserver / gzclient processes that roslaunch doesn't manage."""
        for name in ["gzserver", "gzclient"]:
            try:
                subprocess.run(["pkill", "-SIGTERM", name], timeout=3)
            except Exception:
                pass

    def _save_map(self):
        map_name = self.map_name_entry.get().strip()
        if not map_name:
            self._log("[WARN] 地图名称为空！", label="System")
            return
        if not map_name.endswith(".pcd"):
            map_name += ".pcd"
        try:
            env = self._get_env()
            result = subprocess.run(
                ["rosservice", "call", "/save_map", f"save_path: '{map_name}'"],
                env=env, capture_output=True, text=True, timeout=30,
            )
            out = (result.stdout + result.stderr).strip()
            if out:
                self._log(f"[SAVE] {out}", label="System")
            else:
                self._log(f"[SAVE] 地图已保存: {map_name}", label="System")
        except subprocess.TimeoutExpired:
            self._log("[ERR] 保存地图超时！", label="System")
        except Exception as e:
            self._log(f"[ERR] 保存地图失败: {e}", label="System")

    # ------------------------------------------------------------------
    # Map selection
    # ------------------------------------------------------------------

    def _load_map_list(self):
        """Scan MAP_DIR for .pcd files and populate the combobox."""
        try:
            def _sort_key(f):
                name = os.path.splitext(os.path.basename(f))[0]
                return (0, int(name)) if name.isdigit() else (1, name)
            pcd_files = sorted(glob.glob(os.path.join(MAP_DIR, "*.pcd")), key=_sort_key)
            names = [os.path.basename(f) for f in pcd_files]
            self.map_combo["values"] = names
            if names:
                self.map_combo.set(names[0])
            print(f"[DEBUG] Found {len(names)} PCD files in {MAP_DIR}", file=sys.stderr)
        except Exception as e:
            print(f"[DEBUG] Failed to scan map dir: {e}", file=sys.stderr)

    def _apply_map(self):
        """Set the selected map PCD name via rosparam."""
        selected = self.map_combo.get()
        if not selected:
            self._log("[WARN] 未选择地图！", label="System")
            return
        try:
            env = self._get_env()
            param_name = "localizer/auto_reloc/pcd_name"
            subprocess.run(
                ["rosparam", "set", param_name, selected],
                env=env, capture_output=True, text=True, timeout=5,
            )
            self._log(f"[MAP] 已设置定位地图: {selected}", label="System")
            self._log(f"[MAP] 请重启定位节点使地图生效", label="System")
        except Exception as e:
            self._log(f"[ERR] 设置地图失败: {e}", label="System")

    # ------------------------------------------------------------------
    # Velocity control
    # ------------------------------------------------------------------

    def _init_ros_vel(self):
        """Initialize ROS node and velocity publisher if not already done."""
        if self.vel_pub is not None:
            return True
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("ros_launcher_gui", anonymous=True, disable_signals=True)
            self.vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
            self._dbg(f"[DEBUG] Velocity publisher ready on {VEL_TOPIC}")
            return True
        except Exception as e:
            self._dbg(f"[DEBUG] Failed to init ROS vel: {e}")
            return False

    def _toggle_vel_control(self):
        """Enable or disable velocity control mode."""
        if not self.vel_enabled:
            if not self._init_ros_vel():
                self._log("[ERR] 无法初始化 ROS 速度发布", label="System")
                return
            self.vel_enabled = True
            self.vel_running = True
            self.vel_btn.config(text="关闭控制")
            self._log("[VEL] 速度控制已开启", label="System")

            # Start publisher thread
            self.vel_thread = threading.Thread(target=self._vel_pub_loop, daemon=True)
            self.vel_thread.start()
        else:
            self.vel_enabled = False
            self.vel_running = False
            self.vel_btn.config(text="开启控制")
            # Send zero velocity
            if self.vel_pub:
                self.vel_pub.publish(Twist())
            self._zero_vel()
            self._log("[VEL] 速度控制已关闭", label="System")

    def _vel_pub_loop(self):
        """Publish velocity at 10Hz while enabled."""
        rate = rospy.Rate(VEL_RATE_HZ)
        while self.vel_running and not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = self.vel_values[0]
            twist.linear.y = self.vel_values[1]
            twist.linear.z = self.vel_values[2]
            twist.angular.z = self.vel_values[3]
            if self.vel_pub:
                self.vel_pub.publish(twist)
            rate.sleep()

    def _adjust_vel(self, axis, direction):
        """Adjust velocity for given axis by VEL_STEP * direction."""
        if not self.vel_enabled:
            self._log("[WARN] 请先开启速度控制", label="System")
            return
        self.vel_values[axis] += VEL_STEP * direction
        self.vel_values[axis] = round(self.vel_values[axis], 2)
        self._update_vel_display()

    def _zero_vel(self):
        """Set all velocities to zero."""
        self.vel_values = [0.0, 0.0, 0.0, 0.0]
        self._update_vel_display()

    def _update_vel_display(self):
        """Update velocity display labels."""
        for i, lbl in enumerate(self.vel_labels):
            lbl.config(text=f"{self.vel_values[i]:+.2f}")

    def _kill_all(self):
        for label in list(self.processes.keys()):
            self._kill(label)

    def _stream_output(self, label, proc, entry):
        def reader():
            self._dbg(f"[DEBUG] Reader thread started for '{label}'")
            for line in proc.stdout:
                if line:
                    self._log(_strip_ansi(line.rstrip()), label=label)
            self._dbg(f"[DEBUG] Reader thread for '{label}': stdout closed, rc={proc.returncode}")
            entry["_status"].config(text="○", foreground="black")
            self.processes.pop(label, None)
            self._log(f"[DONE] (exit {proc.returncode})", label=label)

        t = threading.Thread(target=reader, daemon=True)
        t.start()

    def _send_stdin(self, label):
        """Send text from the stdin entry to the process."""
        entry, _ = self.stdin_entries.get(label, (None, None))
        if not entry:
            return
        text = entry.get()
        if not text:
            return
        entry.delete(0, "end")
        proc = self.processes.get(label)
        if proc and proc.poll() is None:
            try:
                proc.stdin.write(text + "\n")
                proc.stdin.flush()
                self._log(f"> {text}", label=label)
            except Exception as e:
                self._log(f"[ERR] stdin write failed: {e}", label=label)
        else:
            self._log(f"[WARN] Process not running, can't send input", label=label)

    def _log(self, msg, label=None):
        """Write a message to a process tab, or to the System tab if label is None."""
        output = self._get_output(label or "System")
        output.insert("end", msg + "\n")
        output.see("end")

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")

        F = self._cjk_font
        row = 0

        # ── 顶部状态栏 ──────────────────────────────────────────────
        top_bar = ttk.Frame(main)
        top_bar.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(top_bar, text="ROS Launch Manager", font=(F, 12, "bold")).pack(side="left")
        self.roscore_status = ttk.Label(top_bar, text="", font=(F, 9))
        self.roscore_status.pack(side="left", padx=(12, 0))
        self._update_roscore_status()
        ttk.Button(top_bar, text="全部停止", command=self._kill_all).pack(side="right")
        row += 1

        ttk.Separator(main, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(0, 4))
        row += 1

        # ── 启动按钮区域（按行分组）────────────────────────────────
        # 第一行: Core, Controller, SLAM
        row1_categories = ["Core", "Controller", "SLAM"]
        btn_row1 = ttk.Frame(main)
        btn_row1.grid(row=row, column=0, sticky="ew", pady=2)
        self._build_button_row(btn_row1, row1_categories, F)
        row += 1

        # 第二行: Waypoint, Test Tools
        row2_categories = ["Waypoint", "Test Tools"]
        btn_row2 = ttk.Frame(main)
        btn_row2.grid(row=row, column=0, sticky="ew", pady=2)
        self._build_button_row(btn_row2, row2_categories, F)
        row += 1

        ttk.Separator(main, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(8, 4))
        row += 1

        # ── 下方区域：左配置 + 右日志 ──────────────────────────────
        bottom = ttk.Frame(main)
        bottom.grid(row=row, column=0, sticky="nsew")

        # 左侧配置面板（35%）
        left_panel = ttk.Frame(bottom)
        left_panel.pack(side="left", fill="both", padx=(0, 6))

        # 右侧输出日志（65%）
        right_panel = ttk.Frame(bottom)
        right_panel.pack(side="left", fill="both", expand=True)

        # ── 左侧：地图选择 ────────────────────────────────────────
        ttk.Label(left_panel, text="定位地图", font=(F, 10, "bold")).pack(anchor="w", pady=(0, 4))

        map_sel = ttk.Frame(left_panel)
        map_sel.pack(fill="x", pady=1)
        ttk.Label(map_sel, text="选择地图:", font=(F, 9)).pack(side="left")
        self.map_combo = ttk.Combobox(map_sel, width=16, state="readonly")
        self.map_combo.pack(side="left", padx=(4, 4))
        self._load_map_list()
        ttk.Button(map_sel, text="应用", command=self._apply_map).pack(side="left")

        map_save = ttk.Frame(left_panel)
        map_save.pack(fill="x", pady=1)
        ttk.Label(map_save, text="地图名称:", font=(F, 9)).pack(side="left")
        self.map_name_entry = ttk.Entry(map_save, width=16)
        self.map_name_entry.pack(side="left", padx=(4, 4))
        self.map_name_entry.insert(0, "my_map")
        ttk.Button(map_save, text="保存", command=self._save_map).pack(side="left")

        ttk.Separator(left_panel, orient="horizontal").pack(fill="x", pady=8)

        # ── 左侧：速度控制 ────────────────────────────────────────
        ttk.Label(left_panel, text="速度控制", font=(F, 10, "bold")).pack(anchor="w", pady=(0, 4))

        self.vel_btn = ttk.Button(left_panel, text="开启控制", width=10,
                                  command=self._toggle_vel_control)
        self.vel_btn.pack(anchor="w", pady=(0, 8))

        # 速度显示
        vel_display = ttk.Frame(left_panel)
        vel_display.pack(fill="x", pady=(0, 8))
        self.vel_labels = []
        vel_names = ["X:", "Y:", "Z:", "Yaw:"]
        for i, name in enumerate(vel_names):
            col = ttk.Frame(vel_display)
            col.pack(side="left", expand=True)
            ttk.Label(col, text=name, font=(F, 8)).pack()
            lbl = ttk.Label(col, text="0.00", font=(F, 10, "bold"))
            lbl.pack()
            self.vel_labels.append(lbl)

        # 速度按钮（2列4行网格）
        vel_btns = ttk.Frame(left_panel)
        vel_btns.pack(fill="x")
        vel_axes = [
            ("X+", 0, 1), ("X-", 0, -1),
            ("Y+", 1, 1), ("Y-", 1, -1),
            ("Z+", 2, 1), ("Z-", 2, -1),
            ("Yaw+", 3, 1), ("Yaw-", 3, -1),
        ]
        for i, (text, axis, direction) in enumerate(vel_axes):
            r, c = divmod(i, 2)
            ttk.Button(vel_btns, text=text, width=6,
                       command=lambda a=axis, d=direction: self._adjust_vel(a, d)).grid(
                row=r, column=c, padx=1, pady=1, sticky="ew")
        vel_btns.grid_columnconfigure(0, weight=1)
        vel_btns.grid_columnconfigure(1, weight=1)

        ttk.Button(left_panel, text="归零", command=self._zero_vel).pack(fill="x", pady=(4, 0))

        # ── 右侧：输出日志 ────────────────────────────────────────
        ttk.Label(right_panel, text="Output", font=(F, 10, "bold")).pack(anchor="w", pady=(0, 4))
        self.notebook = ttk.Notebook(right_panel)
        self.notebook.pack(fill="both", expand=True)

        # ── 布局权重 ──────────────────────────────────────────────
        self.root.grid_rowconfigure(0, weight=1)
        main.grid_rowconfigure(row, weight=1)  # bottom 行可扩展
        main.grid_columnconfigure(0, weight=1)
        bottom.pack_propagate(True)

    def _build_button_row(self, parent, categories, font):
        """在 parent 中横向渲染指定分类的启动按钮。"""
        first = True
        for category in categories:
            items = COMMANDS.get(category, [])
            if not items:
                continue
            if not first:
                ttk.Separator(parent, orient="vertical").pack(side="left", fill="y", padx=6)
            first = False
            ttk.Label(parent, text=category, font=(font, 9, "bold")).pack(side="left", padx=(0, 4))
            for entry in items:
                label = entry["label"]
                btn_frame = ttk.Frame(parent)
                btn_frame.pack(side="left", padx=1)
                ttk.Button(btn_frame, text=label, width=14,
                           command=lambda l=label: self._launch(l)).pack(side="left")
                status = ttk.Label(btn_frame, text="○", width=1)
                status.pack(side="left")
                kill_btn = ttk.Button(btn_frame, text="✕", width=2,
                                      command=lambda l=label: self._kill(l))
                kill_btn.pack(side="left")
                entry["_status"] = status



if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("680x720")
    app = RosLauncher(root)
    root.mainloop()
