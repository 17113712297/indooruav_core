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
from std_msgs.msg import String

# ============================================================
# Configuration — add entries here to extend
# ============================================================

WORKSPACE = os.path.expanduser("~/Project/IndoorUavInspection2/catkin_ws")
THREE_D_WORKSPACE = os.path.expanduser("~/Project/3D/catkin_ws")
SHELL_DIR = os.path.join(WORKSPACE, "src", "shell")
SETUP_BASH = os.path.join(WORKSPACE, "devel", "setup.bash")
THREE_D_SETUP_BASH = os.path.join(THREE_D_WORKSPACE, "devel", "setup.bash")

MAP_DIR = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "map3d")
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
        {"label": "航点记录按钮", "cmd": ["bash", os.path.join(SHELL_DIR, "waypoint_record_button.sh")]},
        {"label": "里程计记录",   "cmd": ["python", os.path.join(WORKSPACE, "src", "indooruav_core", "scripts", "odometry_recorder.py")]},
        {"label": "像素坐标发布", "cmd": ["python3", "-u", os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "scripts", "odometry_to_pixel.py")]},
    ],
    "Services": [
        {"label": "启动HTTP服务", "cmd": ["roslaunch", "indooruav_http", "bringup_indooruav_http.launch"]},
        {"label": "启动降落", "cmd": ["roslaunch", "indooruav_mission", "bringup_mission.launch"]},
    ],
}

# ============================================================
# Visual theme — colors are display-only and do not affect logic
# ============================================================


class Palette:
    BG         = "#eef1f6"   # window background
    SURFACE    = "#f7f9fc"   # light surfaces
    BORDER     = "#d7dde6"
    TEXT       = "#2c3e50"
    SUBTLE     = "#7a8794"
    ACCENT     = "#3a7bd5"
    ACCENT_D   = "#2c5fa8"
    SUCCESS    = "#27ae60"
    SUCCESS_D  = "#1e8b4d"
    DANGER     = "#e74c3c"
    DANGER_D   = "#c0392b"
    WARNING    = "#e67e22"
    IDLE       = "#a9b3bf"
    CHIP_BG    = "#e5eaf1"
    CONSOLE_BG = "#fbfcfe"
    CONSOLE_FG = "#2c3e50"


# Status glyphs / colors for the per-command indicator (display only)
STATUS_IDLE    = ("●", Palette.IDLE)
STATUS_RUNNING = ("●", Palette.SUCCESS)
STATUS_ERROR   = ("●", Palette.DANGER)

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
        self._setup_styles(default_font, bold_font)
        self.root.configure(bg=Palette.BG)

        self._start_roscore()
        self._build_ui()

        # Ensure roscore is cleaned up on exit
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # Theming (visual only)
    # ------------------------------------------------------------------

    def _setup_styles(self, default_font, bold_font):
        """Configure ttk styles for a clean, modern look. Display-only."""
        P = Palette
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # Combobox popdown list colors
        self.root.option_add("*TCombobox*Listbox.background", P.CONSOLE_BG)
        self.root.option_add("*TCombobox*Listbox.foreground", P.TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", P.ACCENT)
        self.root.option_add("*TCombobox*Listbox.selectForeground", "white")

        style.configure(".", background=P.BG, foreground=P.TEXT, font=default_font)
        style.configure("TFrame", background=P.BG)
        style.configure("TLabel", background=P.BG, foreground=P.TEXT)
        style.configure("Subtle.TLabel", background=P.BG, foreground=P.SUBTLE)
        style.configure("Header.TLabel", background=P.BG, foreground=P.ACCENT, font=bold_font)
        style.configure("Title.TLabel", background=P.BG, foreground=P.TEXT)
        style.configure("Bold.TLabel", font=bold_font)
        style.configure("TSeparator", background=P.BORDER)

        # Inputs
        style.configure("TEntry", fieldbackground="white", bordercolor=P.BORDER,
                        relief="flat", padding=3)
        style.configure("TCombobox", fieldbackground="white", background="white",
                        bordercolor=P.BORDER, arrowcolor=P.TEXT, padding=3)
        style.map("TCombobox", fieldbackground=[("readonly", "white")])

        # ---- 3D buttons ----------------------------------------------
        # clam draws a bevel from lightcolor (top-left) + darkcolor (bottom-right);
        # raised relief + a 3px border gives depth, and on press we invert the
        # bevel and sink the relief so the button visibly depresses.
        def _bevel(name, base, light, dark, border, fg="white",
                   pad=(11, 6), bw=3):
            style.configure(name, padding=pad, relief="raised", borderwidth=bw,
                            background=base, foreground=fg,
                            bordercolor=border, lightcolor=light, darkcolor=dark,
                            focuscolor=base)
            style.map(
                name,
                background=[("pressed", dark), ("active", light),
                            ("disabled", "#d9dee6")],
                foreground=[("disabled", P.SUBTLE)],
                relief=[("pressed", "sunken"), ("!pressed", "raised")],
                lightcolor=[("pressed", dark)],
                darkcolor=[("pressed", light)],
            )

        # Base neutral button
        _bevel("TButton", "#e3e8f0", "#ffffff", "#aeb8c8", "#aeb8c8", fg=P.TEXT)
        # Accent button
        _bevel("Accent.TButton", P.ACCENT, "#6aa1ec", "#23538f", P.ACCENT_D)
        # Success button
        _bevel("Success.TButton", P.SUCCESS, "#45d07f", "#1b7a45", P.SUCCESS_D)
        # Danger button
        _bevel("Danger.TButton", P.DANGER, "#f2715f", "#b32a1d", P.DANGER_D)
        # Small kill button (compact)
        _bevel("Kill.TButton", "#efd1cc", "#ffffff", "#d6aaa3",
               "#d6aaa3", fg=P.DANGER_D, pad=(2, 2), bw=2)

        # Stdin bar frame
        style.configure("Stdin.TFrame", background=P.SURFACE)

        # Notebook
        style.configure("TNotebook", background=P.BG, borderwidth=0, tabmargins=(2, 4, 2, 0))
        style.configure("TNotebook.Tab", padding=(14, 6), background=P.CHIP_BG,
                        foreground=P.SUBTLE, borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", P.CONSOLE_BG)],
                  foreground=[("selected", P.TEXT)],
                  expand=[("selected", (1, 1, 1, 0))])

    def _section_header(self, parent, text):
        """A small accent bar + bold title used to head each config block."""
        head = ttk.Frame(parent)
        head.pack(fill="x", pady=(0, 4))
        bar = tk.Frame(head, width=4, height=16, bg=Palette.ACCENT)
        bar.pack(side="left", padx=(0, 6))
        bar.pack_propagate(False)
        ttk.Label(head, text=text, style="Header.TLabel").pack(side="left")
        return head

    # ------------------------------------------------------------------

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
            self._update_roscore_status()
            return
        except Exception:
            pass

        self._dbg("[DEBUG] Starting roscore ...")
        env = self._get_env()
        try:
            self.roscore_proc = subprocess.Popen(
                ["roscore"],
                env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                start_new_session=True,
            )
            # Wait for roscore to come up (poll the master port)
            for _ in range(15):
                time.sleep(1)
                if self.roscore_proc.poll() is not None:
                    self._dbg(f"[DEBUG] roscore failed to start, rc={self.roscore_proc.returncode}")
                    self.roscore_proc = None
                    break
                try:
                    s = socket.create_connection(("localhost", 11311), timeout=1)
                    s.close()
                    self._dbg(f"[DEBUG] roscore started, pid={self.roscore_proc.pid}")
                    break
                except Exception:
                    pass
            else:
                self._dbg("[DEBUG] roscore still not ready after 15s, giving up")
                self.roscore_proc.kill()
                self.roscore_proc = None
        except Exception as e:
            self._dbg(f"[DEBUG] Failed to start roscore: {e}")
            self.roscore_proc = None
        self._update_roscore_status()

    def _update_roscore_status(self):
        if not hasattr(self, "roscore_status"):
            return
        if self.roscore_proc and self.roscore_proc.poll() is None:
            self.roscore_status.config(text="● running", foreground=Palette.SUCCESS)
        else:
            self.roscore_status.config(text="○ not running", foreground=Palette.SUBTLE)

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
        if hasattr(self, "notebook"):
            self._log(msg)

    def _style_console(self, text):
        """Apply console colors and log-level tags to a ScrolledText (display only)."""
        try:
            text.configure(background=Palette.CONSOLE_BG, foreground=Palette.CONSOLE_FG,
                           insertbackground=Palette.TEXT, borderwidth=0,
                           highlightthickness=0, padx=8, pady=6,
                           selectbackground=Palette.ACCENT, selectforeground="white")
        except tk.TclError:
            pass
        text.tag_config("err", foreground=Palette.DANGER)
        text.tag_config("warn", foreground=Palette.WARNING)
        text.tag_config("ok", foreground=Palette.SUCCESS)
        text.tag_config("info", foreground=Palette.ACCENT)
        text.tag_config("muted", foreground=Palette.SUBTLE)

    @staticmethod
    def _pick_log_tag(msg):
        """Choose a color tag for a log line based on its prefix (display only)."""
        if "[ERR" in msg:
            return "err"
        if "[WARN" in msg or "[KILL]" in msg:
            return "warn"
        if "[DEBUG]" in msg:
            return "muted"
        if "[RUN]" in msg or "[DONE]" in msg:
            return "ok"
        if msg.startswith(">"):
            return "info"
        if any(k in msg for k in ("[SAVE]", "[MAP]", "[WP]", "[REC]", "[GIMBAL]", "[VEL]")):
            return "info"
        return ""

    def _create_stdin_bar(self, frame, label, before=None):
        """Create the interactive stdin input bar inside a tab frame."""
        stdin_frame = ttk.Frame(frame, style="Stdin.TFrame", padding=(4, 3))
        pack_opts = dict(fill="x", side="bottom", padx=2, pady=(2, 2))
        if before is not None:
            pack_opts["before"] = before
        stdin_frame.pack(**pack_opts)
        ttk.Label(stdin_frame, text=" ›", style="Subtle.TLabel").pack(side="left")
        entry = ttk.Entry(stdin_frame)
        entry.pack(side="left", fill="x", expand=True, padx=4)
        send_btn = ttk.Button(
            stdin_frame, text="Send", width=6, style="Accent.TButton",
            command=lambda: self._send_stdin(label),
        )
        send_btn.pack(side="left", padx=2)
        entry.bind("<Return>", lambda e: self._send_stdin(label))
        self.stdin_entries[label] = (entry, send_btn)

    def _get_output(self, label, with_stdin=False):
        """Return the ScrolledText for a given process label, creating a tab if needed."""
        if label not in self.outputs:
            frame = ttk.Frame(self.notebook)

            # Pack stdin bar first so it claims space before the expanded text widget
            if with_stdin:
                self._create_stdin_bar(frame, label)
                self._dbg(f"[DEBUG] Stdin bar added for '{label}'")

            text = scrolledtext.ScrolledText(frame, state="normal", wrap="word",
                                              font=(self._cjk_font, 10))
            self._style_console(text)
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
            self._create_stdin_bar(frame, label, before=text)

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

    def _set_status(self, entry, status):
        """Update a command's status indicator (glyph, color)."""
        if entry and "_status" in entry:
            glyph, color = status
            entry["_status"].config(text=glyph, foreground=color)

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
        self._set_status(entry, STATUS_RUNNING)
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
                self._set_status(entry, STATUS_ERROR)
                self.processes.pop(label, None)
            else:
                self._dbg(f"[DEBUG] Process {label} still alive after 0.5s")
        except Exception as e:
            self._dbg(f"[DEBUG] Exception in _launch: {e}")
            traceback.print_exc(file=sys.stderr)
            self._log(f"[ERR] Failed to start: {e}", label=label)
            self._set_status(entry, STATUS_ERROR)

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

        self._set_status(entry, STATUS_IDLE)

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
            current = self.map_combo.get()
            self.map_combo["values"] = names
            if current and current in names:
                self.map_combo.set(current)
            elif names:
                self.map_combo.set(names[0])
            print(f"[DEBUG] Found {len(names)} PCD files in {MAP_DIR}", file=sys.stderr)
        except Exception as e:
            print(f"[DEBUG] Failed to scan map dir: {e}", file=sys.stderr)

    def _apply_map(self):
        """Set the selected map PCD name by modifying localize.yaml directly."""
        selected = self.map_combo.get()
        if not selected:
            self._log("[WARN] 未选择地图！", label="System")
            return
        yaml_path = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "config", "localize.yaml")
        try:
            with open(yaml_path, "r") as f:
                content = f.read()
            # Replace pcd_name value (supports both quoted "xxx.pcd" and unquoted xxx.pcd)
            new_content = re.sub(
                r'(pcd_name:\s*)(?:"[^"]*"|\S+)',
                r'\1"{}"'.format(selected),
                content,
                count=1,
            )
            with open(yaml_path, "w") as f:
                f.write(new_content)
            self._log(f"[MAP] 已设置定位地图: {selected}", label="System")
            self._log(f"[MAP] 请重启定位节点使地图生效", label="System")
        except Exception as e:
            self._log(f"[ERR] 设置地图失败: {e}", label="System")

    # ------------------------------------------------------------------
    # Waypoint selection
    # ------------------------------------------------------------------

    WAYPOINT_DIR = os.path.join(WORKSPACE, "src", "indooruav_waypoint", "waypoints")
    WAYPOINT_YAML = os.path.join(WORKSPACE, "src", "indooruav_waypoint", "config", "config.yaml")

    def _load_waypoint_list(self):
        """Scan WAYPOINT_DIR for waypoints*.yaml files and populate the combobox."""
        try:
            yaml_files = sorted(glob.glob(os.path.join(self.WAYPOINT_DIR, "waypoints*.yaml")))
            names = [os.path.basename(f) for f in yaml_files]
            current = self.wp_combo.get()
            self.wp_combo["values"] = names
            if current and current in names:
                self.wp_combo.set(current)
            elif names:
                self.wp_combo.set(names[0])
        except Exception as e:
            print(f"[DEBUG] Failed to scan waypoint dir: {e}", file=sys.stderr)

    def _apply_waypoint(self):
        """Set the selected waypoint file by modifying config.yaml directly."""
        selected = self.wp_combo.get()
        if not selected:
            self._log("[WARN] 未选择航线！", label="System")
            return
        try:
            with open(self.WAYPOINT_YAML, "r") as f:
                content = f.read()
            # Replace waypoints_file_path value (first occurrence = tracker)
            new_content = re.sub(
                r'(waypoints_file_path:\s*)[^\n]+',
                r'\g<1>waypoints/{}'.format(selected),
                content,
                count=1,
            )
            with open(self.WAYPOINT_YAML, "w") as f:
                f.write(new_content)
            self._log(f"[WP] 已设置航线文件: {selected}", label="System")
            self._log(f"[WP] 请重启航线追踪节点使航线生效", label="System")
        except Exception as e:
            self._log(f"[ERR] 设置航线失败: {e}", label="System")

    # ------------------------------------------------------------------
    # Waypoint recording
    # ------------------------------------------------------------------

    def _get_current_waypoint_file(self):
        """Read the recorder's waypoint file path from config.yaml."""
        try:
            with open(self.WAYPOINT_YAML, "r") as f:
                content = f.read()
            # Find the second waypoints_file_path (recorder section)
            matches = re.findall(r'waypoints_file_path:\s*([^\n]+)', content)
            if len(matches) >= 2:
                wp_rel = matches[1]  # e.g. "config/waypoints823.yaml"
                # Build absolute path: indooruav_waypoint/<rel>
                return os.path.join(WORKSPACE, "src", "indooruav_waypoint", wp_rel)
        except Exception:
            pass
        return None

    def _call_trigger_srv(self, srv_name, action_name):
        """Call a std_srvs/Trigger service."""
        try:
            env = self._get_env()
            result = subprocess.run(
                ["rosservice", "call", srv_name, "{}"],
                env=env, capture_output=True, text=True, timeout=5,
            )
            out = (result.stdout + result.stderr).strip()
            if out:
                self._log(f"[REC] {action_name}: {out}", label="System")
            else:
                self._log(f"[REC] {action_name} 完成", label="System")
        except Exception as e:
            self._log(f"[ERR] {action_name} 失败: {e}", label="System")

    def _call_trigger_srv_with_check(self, srv_name, action_name, btn, check_file=True):
        """Call service, then verify the waypoint file was actually modified."""
        wp_file = self._get_current_waypoint_file() if check_file else None
        mtime_before = os.path.getmtime(wp_file) if wp_file and os.path.exists(wp_file) else 0

        try:
            env = self._get_env()
            result = subprocess.run(
                ["rosservice", "call", srv_name, "{}"],
                env=env, capture_output=True, text=True, timeout=5,
            )
            out = (result.stdout + result.stderr).strip()
        except Exception as e:
            self._log(f"[ERR] {action_name} 失败: {e}", label="System")
            self._flash_btn(btn, success=False)
            return

        # Check service response for success field
        srv_ok = "success: True" in out or "success: true" in out

        if not srv_ok:
            self._log(f"[REC] {action_name} 失败: {out}", label="System")
            self._flash_btn(btn, success=False)
            return

        # Service succeeded, now check file if needed
        if check_file and wp_file:
            time.sleep(0.3)
            mtime_after = os.path.getmtime(wp_file) if os.path.exists(wp_file) else 0
            if mtime_after > mtime_before:
                self._log(f"[REC] {action_name} 成功，文件已更新", label="System")
                self._flash_btn(btn, success=True)
            else:
                self._log(f"[REC] {action_name} 完成，但文件未变化", label="System")
                self._flash_btn(btn, success=False)
        else:
            self._log(f"[REC] {action_name} 成功", label="System")
            self._flash_btn(btn, success=True)

    def _flash_btn(self, btn, success=True):
        """Flash button green on success, red on failure, then revert."""
        style = "Success.TButton" if success else "Danger.TButton"
        btn.config(style=style)
        self.root.after(1500, lambda: btn.config(style="TButton"))

    def _wp_record(self):
        """Record current position as a waypoint."""
        self._call_trigger_srv_with_check(
            "indooruav_controller/waypoint_recorder/record", "记录航点",
            self.wp_record_btn, check_file=False)

    def _wp_clear(self):
        """Clear all recorded waypoints from memory."""
        self._call_trigger_srv_with_check(
            "indooruav_controller/waypoint_recorder/clear", "清除航点",
            self.wp_clear_btn, check_file=False)

    def _apply_rec_filename(self):
        """Set recorder save filename by modifying config.yaml directly."""
        filename = self.wp_rec_name_entry.get().strip()
        if not filename:
            self._log("[WARN] 文件名为空！", label="System")
            return
        try:
            with open(self.WAYPOINT_YAML, "r") as f:
                content = f.read()
            # Update the recorder's waypoints_file_path (only under waypoint_recorder:)
            new_content = re.sub(
                r'(waypoint_recorder:.*?waypoints_file_path:\s*)[^\n]+',
                r'\g<1>waypoints/{}'.format(filename),
                content,
                flags=re.DOTALL,
            )
            with open(self.WAYPOINT_YAML, "w") as f:
                f.write(new_content)
            self._log(f"[REC] 已设置保存文件名: {filename}", label="System")
            self._log(f"[REC] 请重启航线记录节点使配置生效", label="System")
        except Exception as e:
            self._log(f"[ERR] 设置文件名失败: {e}", label="System")

    def _wp_save(self):
        """Save recorded waypoints to file."""
        self._call_trigger_srv_with_check(
            "indooruav_controller/waypoint_recorder/save", "保存航点",
            self.wp_save_btn, check_file=True)

    def _run_pcd_to_2d(self):
        """Run pcd_to_2d.py in background, highlight button while running."""
        if self.pcd2d_running:
            return
        self.pcd2d_running = True
        self.pcd2d_btn.config(text="生成中...", style="Accent.TButton")
        self._log("[2D] 开始生成2D地图...", label="System")

        def worker():
            try:
                env = self._get_env()
                script = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "scripts", "pcd_to_2d.py")
                proc = subprocess.Popen(
                    ["python3", "-u", script],
                    env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1,
                )
                lines = []
                for line in proc.stdout:
                    line = line.rstrip()
                    lines.append(line)
                    print(f"[2D] {line}", file=sys.stderr, flush=True)
                proc.wait()
                out = "\n".join(lines)
                if proc.returncode == 0:
                    self._log(f"[2D] 生成完成", label="System")
                else:
                    self._log(f"[2D] 生成失败 (exit {proc.returncode})", label="System")
            except subprocess.TimeoutExpired:
                self._log("[2D] 生成超时！", label="System")
            except Exception as e:
                self._log(f"[2D] 生成异常: {e}", label="System")
            finally:
                self.pcd2d_running = False
                self.pcd2d_btn.config(text="生成2D地图", style="TButton")

        threading.Thread(target=worker, daemon=True).start()

    def _run_odometry_to_pixel(self):
        """Run odometry_to_pixel_offline.py in background."""
        if self.pixel_running:
            return
        self.pixel_running = True
        self.pixel_btn.config(text="生成中...", style="Accent.TButton")
        self._log("[PIXEL] 开始生成像素坐标...", label="System")

        def worker():
            try:
                env = self._get_env()
                script = os.path.join(WORKSPACE, "src", "FASTLIO2_SAM_LC", "scripts",
                                      "odometry_to_pixel_offline.py")
                proc = subprocess.Popen(
                    ["python3", "-u", script],
                    env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1,
                )
                lines = []
                for line in proc.stdout:
                    line = line.rstrip()
                    lines.append(line)
                    print(f"[PIXEL] {line}", file=sys.stderr, flush=True)
                proc.wait()
                if proc.returncode == 0:
                    self._log(f"[PIXEL] 像素坐标生成完成", label="System")
                else:
                    self._log(f"[PIXEL] 生成失败 (exit {proc.returncode})", label="System")
            except Exception as e:
                self._log(f"[PIXEL] 生成异常: {e}", label="System")
            finally:
                self.pixel_running = False
                self.pixel_btn.config(text="生成像素坐标", style="TButton")

        threading.Thread(target=worker, daemon=True).start()

    def _wp_auto_start(self):
        """Start auto recording waypoints."""
        try:
            env = self._get_env()
            result = subprocess.run(
                ["rosservice", "call",
                 "indooruav_controller/waypoint_recorder/auto_record_start", "{}"],
                env=env, capture_output=True, text=True, timeout=5,
            )
            out = (result.stdout + result.stderr).strip()
            if "success: True" in out or "success: true" in out:
                self._log("[REC] 自动录制启动成功", label="System")
                self.auto_recording = True
                self.wp_auto_start_btn.config(text="录制中...", style="Success.TButton")
            else:
                self._log(f"[REC] 自动录制启动失败: {out}", label="System")
        except Exception as e:
            self._log(f"[ERR] 自动录制启动失败: {e}", label="System")

    def _wp_auto_stop(self):
        """Stop auto recording waypoints."""
        try:
            env = self._get_env()
            result = subprocess.run(
                ["rosservice", "call",
                 "indooruav_controller/waypoint_recorder/auto_record_stop", "{}"],
                env=env, capture_output=True, text=True, timeout=5,
            )
            out = (result.stdout + result.stderr).strip()
            if "success: True" in out or "success: true" in out:
                self._log("[REC] 自动录制已停止", label="System")
            else:
                self._log(f"[REC] 停止录制失败: {out}", label="System")
        except Exception as e:
            self._log(f"[ERR] 停止录制失败: {e}", label="System")
        self.auto_recording = False
        self.wp_auto_start_btn.config(text="自动录制", style="TButton")

    def _apply_auto_interval(self):
        """Set auto record interval by modifying config.yaml."""
        try:
            interval = float(self.wp_auto_interval_entry.get())
        except ValueError:
            self._log("[WARN] 间隔值无效，请输入数字", label="System")
            return
        if interval < 0.5:
            self._log("[WARN] 间隔不能小于 0.5 秒", label="System")
            return
        try:
            with open(self.WAYPOINT_YAML, "r") as f:
                content = f.read()
            # Update auto_record_interval_s in recorder section
            new_content = re.sub(
                r'(auto_record_interval_s:\s*)[\d.]+',
                r'\g<1>{}'.format(interval),
                content,
                count=1,
            )
            with open(self.WAYPOINT_YAML, "w") as f:
                f.write(new_content)
            self._log(f"[REC] 已设置自动录制间隔: {interval} 秒", label="System")
            self._log(f"[REC] 请重启航线记录节点使配置生效", label="System")
        except Exception as e:
            self._log(f"[ERR] 设置间隔失败: {e}", label="System")

    # ------------------------------------------------------------------
    # Gimbal pitch
    # ------------------------------------------------------------------

    def _apply_gimbal_pitch(self):
        """Set gimbal pitch angle by modifying gimbal_angle_after_takeoff.yaml."""
        try:
            pitch = float(self.gimbal_pitch_entry.get())
        except ValueError:
            self._log("[WARN] Pitch 值无效，请输入数字", label="System")
            return
        yaml_path = os.path.join(WORKSPACE, "src", "indooruav_core", "config",
                                  "gimbal_angle_after_takeoff.yaml")
        try:
            with open(yaml_path, "r") as f:
                content = f.read()
            new_content = re.sub(
                r'(pitch:\s*)-?[\d.]+',
                r'\g<1>{}'.format(pitch),
                content,
                count=1,
            )
            with open(yaml_path, "w") as f:
                f.write(new_content)
            self._log(f"[GIMBAL] 已设置起飞后云台 Pitch: {pitch}°", label="System")
            self._log(f"[GIMBAL] 请重启状态机节点使配置生效", label="System")
        except Exception as e:
            self._log(f"[ERR] 设置 Pitch 失败: {e}", label="System")

    # ------------------------------------------------------------------
    # State display
    # ------------------------------------------------------------------

    def _init_state_subscriber(self):
        """Subscribe to state machine state topic."""
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("ros_launcher_gui", anonymous=True, disable_signals=True)
            rospy.Subscriber("/indooruav_core/state_machine/state",
                             String, self._state_callback, queue_size=1)
            print("[DEBUG] State subscriber ready", file=sys.stderr)
        except Exception as e:
            print(f"[DEBUG] Failed to init state subscriber: {e}", file=sys.stderr)

    def _state_callback(self, msg):
        """Update state display when state changes."""
        current_state = msg.data
        for name, lbl in self.state_labels.items():
            if name == current_state:
                lbl.config(foreground="white", background=Palette.SUCCESS,
                           font=(self._cjk_font, 9, "bold"))
            else:
                lbl.config(foreground=Palette.SUBTLE, background=Palette.CHIP_BG,
                           font=(self._cjk_font, 9))

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
            self.vel_btn.config(text="关闭控制", style="Danger.TButton")
            self._log("[VEL] 速度控制已开启", label="System")

            # Start publisher thread
            self.vel_thread = threading.Thread(target=self._vel_pub_loop, daemon=True)
            self.vel_thread.start()
        else:
            self.vel_enabled = False
            self._zero_vel()  # Set vel_values to zero first
            # Keep publishing zero for a short time before stopping thread
            time.sleep(0.5)
            self.vel_running = False
            self.vel_btn.config(text="开启控制", style="Success.TButton")
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
            val = self.vel_values[i]
            color = Palette.SUCCESS if val > 0 else (Palette.DANGER if val < 0 else Palette.TEXT)
            lbl.config(text=f"{val:+.2f}", foreground=color)

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
            self._set_status(entry, STATUS_IDLE)
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
        # Check if user is at the bottom before inserting
        view = output.yview()
        at_bottom = view[1] >= 0.99
        output.insert("end", msg + "\n", self._pick_log_tag(msg))
        if at_bottom:
            output.see("end")

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")

        F = self._cjk_font
        row = 0

        # ── 顶部状态栏 ──────────────────────────────────────────────
        top_bar = ttk.Frame(main)
        top_bar.grid(row=row, column=0, sticky="ew", pady=(0, 10))
        accent_bar = tk.Frame(top_bar, width=5, height=22, bg=Palette.ACCENT)
        accent_bar.pack(side="left", padx=(0, 8))
        accent_bar.pack_propagate(False)
        ttk.Label(top_bar, text="ROS Launch Manager", style="Title.TLabel",
                  font=(F, 14, "bold")).pack(side="left")
        self.roscore_status = ttk.Label(top_bar, text="", font=(F, 9))
        self.roscore_status.pack(side="left", padx=(14, 0))
        self._update_roscore_status()
        ttk.Button(top_bar, text="全部停止", style="Danger.TButton",
                   command=self._kill_all).pack(side="right")
        row += 1

        ttk.Separator(main, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(0, 6))
        row += 1

        # ── 启动按钮区域（按行分组）────────────────────────────────
        # 第一行: Core, Controller, SLAM
        row1_categories = ["Core", "Controller", "SLAM", "Services"]
        btn_row1 = ttk.Frame(main)
        btn_row1.grid(row=row, column=0, sticky="ew", pady=3)
        self._build_button_row(btn_row1, row1_categories, F)
        row += 1

        # 第二行: Waypoint, Test Tools
        row2_categories = ["Waypoint", "Test Tools"]
        btn_row2 = ttk.Frame(main)
        btn_row2.grid(row=row, column=0, sticky="ew", pady=3)
        self._build_button_row(btn_row2, row2_categories, F)
        row += 1

        ttk.Separator(main, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(10, 6))
        row += 1

        # ── 下方区域：左配置 + 右日志 ──────────────────────────────
        bottom = ttk.Frame(main)
        bottom.grid(row=row, column=0, sticky="nsew")

        # 左侧配置面板（35%）
        left_panel = ttk.Frame(bottom)
        left_panel.pack(side="left", fill="both", padx=(0, 10))

        # 右侧输出日志（65%）
        right_panel = ttk.Frame(bottom)
        right_panel.pack(side="left", fill="both", expand=True)

        # ── 左侧：地图选择 ────────────────────────────────────────
        self._section_header(left_panel, "定位地图")

        map_sel = ttk.Frame(left_panel)
        map_sel.pack(fill="x", pady=2)
        ttk.Label(map_sel, text="选择地图:", style="Subtle.TLabel", font=(F, 9)).pack(side="left")
        self.map_combo = ttk.Combobox(map_sel, width=16, state="readonly")
        self.map_combo.pack(side="left", padx=(4, 4))
        self._load_map_list()
        ttk.Button(map_sel, text="刷新", width=4,
                   command=self._load_map_list).pack(side="left", padx=(0, 2))
        ttk.Button(map_sel, text="应用", style="Accent.TButton",
                   command=self._apply_map).pack(side="left")

        map_save = ttk.Frame(left_panel)
        map_save.pack(fill="x", pady=2)
        ttk.Label(map_save, text="地图名称:", style="Subtle.TLabel", font=(F, 9)).pack(side="left")
        self.map_name_entry = ttk.Entry(map_save, width=16)
        self.map_name_entry.pack(side="left", padx=(4, 4))
        self.map_name_entry.insert(0, "my_map")
        ttk.Button(map_save, text="保存", style="Accent.TButton",
                   command=self._save_map).pack(side="left")

        ttk.Separator(left_panel, orient="horizontal").pack(fill="x", pady=10)

        # ── 左侧：航线选择 ────────────────────────────────────────
        self._section_header(left_panel, "航线选择")

        wp_sel = ttk.Frame(left_panel)
        wp_sel.pack(fill="x", pady=2)
        ttk.Label(wp_sel, text="选择航线:", style="Subtle.TLabel", font=(F, 9)).pack(side="left")
        self.wp_combo = ttk.Combobox(wp_sel, width=16, state="readonly")
        self.wp_combo.pack(side="left", padx=(4, 4))
        self._load_waypoint_list()
        ttk.Button(wp_sel, text="刷新", width=4,
                   command=self._load_waypoint_list).pack(side="left", padx=(0, 2))
        ttk.Button(wp_sel, text="应用", style="Accent.TButton",
                   command=self._apply_waypoint).pack(side="left")

        ttk.Separator(left_panel, orient="horizontal").pack(fill="x", pady=10)

        # ── 左侧：航线记录 ────────────────────────────────────────
        self._section_header(left_panel, "航线记录")

        wp_rec_name = ttk.Frame(left_panel)
        wp_rec_name.pack(fill="x", pady=2)
        ttk.Label(wp_rec_name, text="文件名称:", style="Subtle.TLabel", font=(F, 9)).pack(side="left")
        self.wp_rec_name_entry = ttk.Entry(wp_rec_name, width=12)
        self.wp_rec_name_entry.pack(side="left", padx=(4, 4))
        self.wp_rec_name_entry.insert(0, "waypoints.yaml")
        ttk.Button(wp_rec_name, text="应用", style="Accent.TButton",
                   command=self._apply_rec_filename).pack(side="left")

        wp_rec_btns = ttk.Frame(left_panel)
        wp_rec_btns.pack(fill="x", pady=3)
        self.wp_record_btn = ttk.Button(wp_rec_btns, text="记录", width=6,
                   command=self._wp_record)
        self.wp_record_btn.pack(side="left", padx=2)
        self.wp_clear_btn = ttk.Button(wp_rec_btns, text="清除", width=6, style="Danger.TButton",
                   command=self._wp_clear)
        self.wp_clear_btn.pack(side="left", padx=2)
        self.wp_save_btn = ttk.Button(wp_rec_btns, text="保存", width=6, style="TButton",
                   command=self._wp_save)
        self.wp_save_btn.pack(side="left", padx=2)

        # 自动录制
        self.auto_recording = False
        wp_auto_btns = ttk.Frame(left_panel)
        wp_auto_btns.pack(fill="x", pady=3)
        self.wp_auto_start_btn = ttk.Button(wp_auto_btns, text="自动录制", width=8, style="Success.TButton",
                   command=self._wp_auto_start)
        self.wp_auto_start_btn.pack(side="left", padx=2)
        ttk.Button(wp_auto_btns, text="停止录制", width=8, style="Danger.TButton",
                   command=self._wp_auto_stop).pack(side="left", padx=2)

        # 自动录制间隔
        wp_auto_interval = ttk.Frame(left_panel)
        wp_auto_interval.pack(fill="x", pady=2)
        ttk.Label(wp_auto_interval, text="录制间隔:", style="Subtle.TLabel", font=(F, 9)).pack(side="left")
        self.wp_auto_interval_entry = ttk.Entry(wp_auto_interval, width=6)
        self.wp_auto_interval_entry.pack(side="left", padx=(4, 4))
        self.wp_auto_interval_entry.insert(0, "3.0")
        ttk.Label(wp_auto_interval, text="秒", style="Subtle.TLabel", font=(F, 9)).pack(side="left")
        ttk.Button(wp_auto_interval, text="应用", style="Accent.TButton",
                   command=self._apply_auto_interval).pack(side="left", padx=(4, 0))

        # 生成2D地图 + 生成像素坐标 按钮
        self.pcd2d_running = False
        self.pixel_running = False
        wp_2d = ttk.Frame(left_panel)
        wp_2d.pack(fill="x", pady=3)
        self.pcd2d_btn = ttk.Button(wp_2d, text="生成2D地图", style="TButton",
                                     command=self._run_pcd_to_2d)
        self.pcd2d_btn.pack(side="left", padx=2)
        self.pixel_btn = ttk.Button(wp_2d, text="生成像素坐标", style="TButton",
                                     command=self._run_odometry_to_pixel)
        self.pixel_btn.pack(side="right", padx=2)

        ttk.Separator(left_panel, orient="horizontal").pack(fill="x", pady=10)

        # ── 左侧：速度控制 ────────────────────────────────────────
        self._section_header(left_panel, "速度控制")

        # 开启控制 + 速度显示 + 归零（一行）
        vel_top = ttk.Frame(left_panel)
        vel_top.pack(fill="x", pady=(0, 4))
        self.vel_btn = ttk.Button(vel_top, text="开启", width=5,
                                  style="Success.TButton",
                                  command=self._toggle_vel_control)
        self.vel_btn.pack(side="left", padx=(0, 4))
        self.vel_labels = []
        vel_names = ["X", "Y", "Z", "Yaw"]
        for i, name in enumerate(vel_names):
            ttk.Label(vel_top, text=f"{name}:", style="Subtle.TLabel", font=(F, 8)).pack(side="left")
            lbl = ttk.Label(vel_top, text="0.00", font=(F, 9, "bold"), width=5)
            lbl.pack(side="left", padx=(0, 6))
            self.vel_labels.append(lbl)
        ttk.Button(vel_top, text="归零", width=4, style="Accent.TButton",
                   command=self._zero_vel).pack(side="left")

        # 速度按钮（4列2行紧凑网格）
        vel_btns = ttk.Frame(left_panel)
        vel_btns.pack(fill="x", pady=(0, 4))
        vel_axes = [
            ("X+", 0, 1), ("X-", 0, -1), ("Y+", 1, 1), ("Y-", 1, -1),
            ("Z+", 2, 1), ("Z-", 2, -1), ("Yaw+", 3, 1), ("Yaw-", 3, -1),
        ]
        for i, (text, axis, direction) in enumerate(vel_axes):
            r, c = divmod(i, 4)
            ttk.Button(vel_btns, text=text, width=4,
                       command=lambda a=axis, d=direction: self._adjust_vel(a, d)).grid(
                row=r, column=c, padx=1, pady=1, sticky="ew")
        for c in range(4):
            vel_btns.grid_columnconfigure(c, weight=1)

        ttk.Separator(left_panel, orient="horizontal").pack(fill="x", pady=10)

        # ── 左侧：云台 Pitch 设置 ────────────────────────────────
        self._section_header(left_panel, "云台 Pitch")

        gimbal_frame = ttk.Frame(left_panel)
        gimbal_frame.pack(fill="x", pady=2)
        ttk.Label(gimbal_frame, text="角度:", style="Subtle.TLabel", font=(F, 9)).pack(side="left")
        self.gimbal_pitch_entry = ttk.Entry(gimbal_frame, width=8)
        self.gimbal_pitch_entry.pack(side="left", padx=(4, 4))
        self.gimbal_pitch_entry.insert(0, "-60")
        ttk.Button(gimbal_frame, text="应用", style="Accent.TButton",
                   command=self._apply_gimbal_pitch).pack(side="left")
        ttk.Label(gimbal_frame, text="°", style="Subtle.TLabel", font=(F, 9)).pack(side="left")

        # ── 右侧：状态显示 ────────────────────────────────────────
        self._section_header(right_panel, "状态机")

        state_frame = ttk.Frame(right_panel)
        state_frame.pack(fill="x", pady=(0, 8))

        self.state_labels = {}
        states = [
            ("Await", "待机"),
            ("CheckBeforeTakeOff", "自检"),
            ("TakeOff", "起飞"),
            ("Cruise", "巡航"),
            ("DataCollection", "采集"),
            ("Land", "降落"),
            ("Charge", "充电"),
        ]
        for state_name, display_name in states:
            lbl = ttk.Label(state_frame, text=display_name, font=(F, 9),
                            foreground=Palette.SUBTLE, background=Palette.CHIP_BG,
                            relief="flat", padding=(6, 5), anchor="center")
            lbl.pack(side="left", padx=3, fill="x", expand=True)
            self.state_labels[state_name] = lbl

        # Initialize state subscriber
        self._init_state_subscriber()

        ttk.Separator(right_panel, orient="horizontal").pack(fill="x", pady=6)

        # ── 右侧：输出日志 ────────────────────────────────────────
        self._section_header(right_panel, "Output")
        self.notebook = ttk.Notebook(right_panel)
        self.notebook.pack(fill="both", expand=True)

        # ── 布局权重 ──────────────────────────────────────────────
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
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
                ttk.Separator(parent, orient="vertical").pack(side="left", fill="y", padx=8)
            first = False
            ttk.Label(parent, text=category, style="Header.TLabel",
                      font=(font, 9, "bold")).pack(side="left", padx=(0, 6))
            for entry in items:
                label = entry["label"]
                btn_frame = ttk.Frame(parent)
                btn_frame.pack(side="left", padx=1)
                # 根据标签长度动态调整按钮宽度
                btn_width = max(8, min(14, len(label) + 2))
                ttk.Button(btn_frame, text=label, width=btn_width,
                           command=lambda l=label: self._launch(l)).pack(side="left")
                glyph, color = STATUS_IDLE
                status = ttk.Label(btn_frame, text=glyph, width=2, foreground=color,
                                   font=(font, 10))
                status.pack(side="left", padx=(1, 0))
                kill_btn = ttk.Button(btn_frame, text="✕", width=2, style="Kill.TButton",
                                      command=lambda l=label: self._kill(l))
                kill_btn.pack(side="left")
                entry["_status"] = status



if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("900x780")
    root.minsize(800, 640)
    app = RosLauncher(root)
    root.mainloop()
