#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import yaml
from datetime import datetime
from nav_msgs.msg import Odometry

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
        return {
            "timestamp": msg.header.stamp.to_sec(),
            "position": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            },
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w,
            },
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
                yaml.dump(self.odom_records, f, default_flow_style=False)
            rospy.loginfo("Saved {} Odometry records -> {}".format(
                len(self.odom_records), path))

        if self.global_odom_records:
            path = os.path.join(OUTPUT_DIR, "odometry_global_{}.yaml".format(ts))
            with open(path, "w") as f:
                yaml.dump(self.global_odom_records, f, default_flow_style=False)
            rospy.loginfo("Saved {} Odometry_global records -> {}".format(
                len(self.global_odom_records), path))


def main():
    rospy.init_node("odometry_recorder", anonymous=False)
    OdometryRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
