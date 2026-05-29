#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import os
import rospy
import yaml
from datetime import datetime
from nav_msgs.msg import Odometry

class _OrderedDumper(yaml.Dumper):
    pass

def _dict_representer(dumper, data):
    return dumper.represent_mapping("tag:yaml.org,2002:map", data.items())

_OrderedDumper.add_representer(dict, _dict_representer)


OUTPUT_DIR = os.path.join(
    os.path.expanduser("~"),
    "Project/IndoorUavInspection2/catkin_ws/src/FASTLIO2_SAM_LC/log"
)


class OdometryRecorder(object):
    def __init__(self):
        self.odom_records = []
        self.global_odom_records = []

        self.latest_odom = None
        self.latest_global_odom = None

        self.odom_sub = rospy.Subscriber(
            "/Odometry", Odometry, self._odom_cb, queue_size=10
        )
        self.global_odom_sub = rospy.Subscriber(
            "/Odometry_global", Odometry, self._global_odom_cb, queue_size=10
        )

        self.record_timer = rospy.Timer(rospy.Duration(0.1), self._record_cb)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("OdometryRecorder started (10 Hz record rate)")

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _global_odom_cb(self, msg):
        self.latest_global_odom = msg

    def _record_cb(self, event):
        if self.latest_odom is not None:
            self.odom_records.append(self._extract(self.latest_odom))
        if self.latest_global_odom is not None:
            self.global_odom_records.append(self._extract(self.latest_global_odom))

    @staticmethod
    def _extract(msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "z": round(msg.pose.pose.position.z, 3),
            "yaw_deg": round(yaw_deg, 1),
        }

    def _on_shutdown(self):
        if not self.odom_records and not self.global_odom_records:
            rospy.loginfo("No odometry data recorded, skipping save")
            return

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

        if self.odom_records:
            path = os.path.join(OUTPUT_DIR, "odometry_{}.yaml".format(ts))
            with open(path, "w") as f:
                yaml.dump(self.odom_records, f, Dumper=_OrderedDumper, default_flow_style=None, width=120)
            rospy.loginfo("Saved {} Odometry records -> {}".format(
                len(self.odom_records), path))

        if self.global_odom_records:
            path = os.path.join(OUTPUT_DIR, "odometry_global_{}.yaml".format(ts))
            with open(path, "w") as f:
                yaml.dump(self.global_odom_records, f, Dumper=_OrderedDumper, default_flow_style=None, width=120)
            rospy.loginfo("Saved {} Odometry_global records -> {}".format(
                len(self.global_odom_records), path))


def main():
    rospy.init_node("odometry_recorder", anonymous=False)
    OdometryRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
