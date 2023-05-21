#!/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, Float32
import numpy as np
import cv2
from time import time_ns


def warp_2pi_rad(angle_rad):
    while angle_rad < 0:
        angle_rad += 2 * np.pi
    return angle_rad % (2 * np.pi)


def warp_360_deg(angle_deg):
    while angle_deg < 0:
        angle_deg += 360
    return angle_deg % 360


def time():
    return time_ns() * 1e-9


class TunnelTraversalNavigator:
    def __init__(self):
        self.paused = True
        self.time_elapsed = None
        rospy.init_node("tunnel_traversal_navigator")
        command_topic = rospy.get_param("~command_topic")
        image_topic = rospy.get_param("~image_topic")
        angle_topic = rospy.get_param("~angle_topic")
        self.min_dist_from_walls = 2
        self._frontal_angles_range = rospy.get_param(
            "~frontal_angles_range_deg", (-40, 40)
        )
        self._publish_vector = rospy.get_param("~config/publish_vector", default=True)
        self.command_subscriber = rospy.Subscriber(
            command_topic, String, callback=self.command_callback
        )
        self.ptcl_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback
        )
        self.angle_pub = rospy.Publisher(angle_topic, Float32, queue_size=2)
        if self._publish_vector:
            self._vector_publisher = rospy.Publisher(
                "internals/vector", Float32MultiArray, queue_size=2
            )

    def image_callback(self, ptcl_msg: Image):
        dtype = np.dtype(np.float32)
        channels = 1
        dtype = dtype.newbyteorder(">" if ptcl_msg.is_bigendian else "<")
        shape = (ptcl_msg.height, ptcl_msg.width, channels)
        img = np.frombuffer(ptcl_msg.data, dtype=dtype).reshape(shape)
        img.strides = (ptcl_msg.step, dtype.itemsize * channels, dtype.itemsize)
        img = np.array(img)
        # Apply gaussian blurr to the whole image
        conv_img = cv2.GaussianBlur(img, (15, 31), cv2.BORDER_WRAP)
        # Get the two middle rows and do the average of the two
        vector_at_medium_height = np.reshape((conv_img[7, :] + conv_img[8, :]) / 2, -1)
        if self._publish_vector:
            vector_msg = Float32MultiArray()
            vector_msg.data = vector_at_medium_height
            self._vector_publisher.publish(vector_msg)
        idxi = int(self._frontal_angles_range[0])
        idxj = int(self._frontal_angles_range[1])
        idxk = idxi + np.argmax(vector_at_medium_height[range(idxi, idxj)])
        angle_rad_to_advance = np.deg2rad(float(idxk))
        # To correct the angle so that the robot is centered
        distances_at_the_right = img[8, :][range(-100, -80)]
        distances_at_the_left = img[8, :][range(80, 100)]
        min_dist_right = np.min(distances_at_the_right)
        min_dist_left = np.min(distances_at_the_left)
        if (
            min_dist_left < self.min_dist_from_walls
            and min_dist_right > self.min_dist_from_walls
        ):
            angle_rad_to_advance -= np.deg2rad(2)
        elif (
            min_dist_left > self.min_dist_from_walls
            and min_dist_right < self.min_dist_from_walls
        ):
            angle_rad_to_advance += np.deg2rad(2)
        elif (
            min_dist_left < self.min_dist_from_walls
            and min_dist_right < self.min_dist_from_walls
        ):
            if min_dist_left < min_dist_right:
                angle_rad_to_advance += np.deg2rad(2)
            else:
                angle_rad_to_advance -= np.deg2rad(2)
        if self.paused:
            pass
        elif not self.time_elapsed is None:
            self.angle_pub.publish(Float32(angle_rad_to_advance))
            self.time_elapsed += time() - self.prev_time
            print(
                f"Time left: {self.time_limit-self.time_elapsed:10.4f}",
                end="\r",
                flush=True,
            )
            if self.time_elapsed >= self.time_limit:
                self.command_callback(String("stop"))
        self.prev_time = time()

    def command_callback(self, msg: String):
        print(f"Command recieved: {msg}")
        if msg.data.lower() == "start":
            self.time_limit = rospy.get_param("~time_limit", default=20)
            self.paused = False
            self.time_elapsed = 0
        elif msg.data.lower() == "stop":
            self.paused = True
            self.time_elapsed = None
        elif msg.data.lower() == "pause":
            self.paused = True
        elif msg.data.lower() == "unpause":
            self.paused = False

    def run(self):
        rospy.spin()


def main():
    node = TunnelTraversalNavigator()
    node.run()


if __name__ == "__main__":
    main()
