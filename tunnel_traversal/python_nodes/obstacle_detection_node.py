#! /bin/python3
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import rospy
import numpy as np

angle_max = np.deg2rad(40)
threshold = 1


class ObstacleDetectionNode:
    def __init__(self):
        rospy.init_node("obstacle_detection_node")
        self.scan_sub = rospy.Subscriber(
            "/scan", sensor_msgs.LaserScan, callback=self.scan_callback
        )
        self.obstacle_pub = rospy.Publisher(
            "~obstacle_detected", std_msgs.Bool, queue_size=3
        )

    def scan_callback(self, msg: sensor_msgs.LaserScan):
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle = msg.angle_min
        for i in range(len(ranges)):
            if abs(angle) < angle_max:
                if ranges[i] < threshold:
                    self.obstacle_pub.publish(std_msgs.Bool(True))
                    return
            angle += angle_increment
        self.obstacle_pub.publish(std_msgs.Bool(False))


def main():
    obstacle_detection_node = ObstacleDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
