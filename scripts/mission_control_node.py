#!/usr/bin/env python3
"""
mission_control_node.py — 人工干预（暂停 / 继续 / 返航）入口节点

设计意图
--------
遥控器侧（MSDK App）经 PSDK 低速通道下发的控制意图，最终只需要变成对
本节点的一个字符串命令即可，传输层（PSDK 数据通道 / HTTP / 手动调试）
完全解耦：

    rostopic pub -1 /mission/command std_msgs/String "data: 'pause'"

支持命令：
    pause    暂停：Cruise → Paused，无人机悬停并保留航点索引
    resume   继续：Paused → Cruise，从未完成的航点继续
    return   返航：Paused → Return，反方向追踪航点回到第一个航点，
             到达后由 tracker 发 return_complete，状态机自动切 Land
    status   查询当前状态（结果发到 /mission/command_result）

约束：
    - return 只在 Paused 状态可用（飞行中会被拒绝）
    - 所有命令都转发给 indooruav_core 状态机，由状态机决定是否接受；
      本节点只做"入口 + 合法性问题预检"，不直接调 tracker
"""

import json

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty

CMD_PAUSE = "pause"
CMD_RESUME = "resume"
CMD_RETURN = "return"
CMD_STATUS = "status"

# 与 indooruav_core 状态机的 State 名称保持一致
STATE_PAUSED = "Paused"
STATE_CRUISE = "Cruise"
STATE_RETURN = "Return"


class MissionControl:
    def __init__(self):
        self.state = "Unknown"

        # 状态机状态（latched，启动即可拿到当前值）
        self.state_sub = rospy.Subscriber(
            "/indooruav_core/state_machine/state", String, self._on_state, queue_size=1)

        self.result_pub = rospy.Publisher(
            "/mission/command_result", String, queue_size=5, latch=True)

        # tracker 状态（JSON 字符串），仅用于结果回显
        self.tracker_status = "{}"
        self.tracker_sub = rospy.Subscriber(
            "/indooruav_controller/waypoint_tracker/status", String,
            self._on_tracker_status, queue_size=1)

        self._services = {}
        for name, srv in (
            ("pause", "/indooruav_core/state_machine_event/pause"),
            ("resume", "/indooruav_core/state_machine_event/resume"),
            ("return", "/indooruav_core/state_machine_event/return"),
        ):
            try:
                rospy.wait_for_service(srv, timeout=5.0)
                self._services[name] = rospy.ServiceProxy(srv, Empty)
                rospy.loginfo("[MissionControl] service ready: %s", srv)
            except rospy.ROSException:
                rospy.logerr("[MissionControl] service NOT available: %s", srv)

        # 等待 latched 状态到位，避免开机后第一帧判断失效
        rospy.sleep(0.5)

        self.cmd_sub = rospy.Subscriber(
            "/mission/command", String, self._on_command, queue_size=5)

        rospy.loginfo("[MissionControl] ready (state=%s)", self.state)

    # ── 回调 ──────────────────────────────────────────────
    def _on_state(self, msg):
        if msg.data != self.state:
            rospy.loginfo("[MissionControl] state: %s -> %s", self.state, msg.data)
        self.state = msg.data

    def _on_tracker_status(self, msg):
        self.tracker_status = msg.data

    def _on_command(self, msg):
        cmd = msg.data.strip().lower()
        rospy.loginfo("[MissionControl] command='%s' (state=%s)", cmd, self.state)

        if cmd == CMD_STATUS:
            self._reply(True, "status", self.tracker_status)
            return

        handler = {
            CMD_PAUSE: self._handle_pause,
            CMD_RESUME: self._handle_resume,
            CMD_RETURN: self._handle_return,
        }.get(cmd)

        if handler is None:
            self._reply(False, "unknown command: %s" % msg.data)
            return

        handler()

    # ── 命令处理 ──────────────────────────────────────────
    def _handle_pause(self):
        # 只有巡航中才能暂停；已是 Paused 视为幂等成功
        if self.state == STATE_PAUSED:
            self._reply(True, "already paused")
            return
        if self.state != STATE_CRUISE:
            self._reply(False, "pause rejected: state=%s (only Cruise can pause)" % self.state)
            return
        self._call("pause")

    def _handle_resume(self):
        if self.state == STATE_CRUISE:
            self._reply(True, "already cruising")
            return
        if self.state != STATE_PAUSED:
            self._reply(False, "resume rejected: state=%s (only Paused can resume)" % self.state)
            return
        self._call("resume")

    def _handle_return(self):
        # ★ 核心约束：飞行中（Cruise/RETURNING）不允许直接返航，必须先暂停
        if self.state == STATE_RETURN:
            self._reply(True, "already returning")
            return
        if self.state != STATE_PAUSED:
            self._reply(False, "return rejected: state=%s (must pause first)" % self.state)
            return
        self._call("return")

    # ── 工具 ──────────────────────────────────────────────
    def _call(self, name):
        srv = self._services.get(name)
        if srv is None:
            self._reply(False, "%s rejected: service unavailable" % name)
            return
        try:
            srv()
            self._reply(True, "%s accepted" % name)
        except rospy.ServiceException as e:
            self._reply(False, "%s failed: %s" % (name, e))

    def _reply(self, ok, message, extra=None):
        payload = {
            "ok": bool(ok),
            "message": message,
            "state": self.state,
        }
        if extra is not None:
            try:
                payload["tracker"] = json.loads(extra)
            except (ValueError, TypeError):
                payload["tracker"] = extra

        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(out)
        if ok:
            rospy.loginfo("[MissionControl] %s", out.data)
        else:
            rospy.logwarn("[MissionControl] %s", out.data)


if __name__ == "__main__":
    rospy.init_node("mission_control_node")
    MissionControl()
    rospy.spin()
