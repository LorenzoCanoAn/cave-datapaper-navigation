#! /bin/python3
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import rospy
import numpy as np
import matplotlib.pyplot as plt

angle_max = np.deg2rad(40)
threshold = 1


class ObstacleDetectionNode:
    def __init__(self):
        rospy.init_node("obstacle_detection_node")
        self.scan_sub = rospy.Subscriber(
            "/scan", sensor_msgs.LaserScan, callback=self.scan_callback
        )
        self.obstacle_pub = rospy.Publisher(
            "obstacle_detected", std_msgs.Bool, queue_size=3
        )
        self.width_of_obstacle_detection = 1
        self.dist_to_obstacle = 1.5

    def scan_callback(self, msg: sensor_msgs.LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        to_keep = np.where(np.abs(y) < self.width_of_obstacle_detection / 2)
        x = x[to_keep]
        y = y[to_keep]
        x = x[np.where(x > 0)]
        obstacle_detected = np.any(x < self.dist_to_obstacle)
        self.obstacle_pub.publish(std_msgs.Bool(obstacle_detected))


def main():
    obstacle_detection_node = ObstacleDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
