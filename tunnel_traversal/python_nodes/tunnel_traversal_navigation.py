#!/bin/python3
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.point_cloud2 import read_points
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension
import numpy as np
from scipy.ndimage import gaussian_filter
import cv2


class TunnelTraversalNavigator:
    def __init__(self):
        rospy.init_node("tunnel_traversal_navigator")
        toggle_topic = rospy.get_param("~toggle_topic")
        image_topic = rospy.get_param("~image_topic")
        velocity_topic = rospy.get_param("~velocity_topic")
        self._publish_vector = rospy.get_param("~config/publish_vector", default=True)
        self.toggle_subscriber = rospy.Subscriber(
            toggle_topic, Bool, callback=self.toggle_callback
        )
        self.ptcl_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback
        )
        self.vel_pub = rospy.Publisher(velocity_topic, Twist, queue_size=2)
        if self._publish_vector:
            self._vector_publisher = rospy.Publisher(
                "~internals/vector", Float32MultiArray, queue_size=2
            )
        self._publish_vel = False

    def image_callback(self, ptcl_msg: Image):
        if not self._publish_vel:
            return
        dtype = np.dtype(np.float32)
        channels = 1
        dtype = dtype.newbyteorder(">" if ptcl_msg.is_bigendian else "<")
        shape = (ptcl_msg.height, ptcl_msg.width, channels)
        img = np.frombuffer(ptcl_msg.data, dtype=dtype).reshape(shape)
        img.strides = (ptcl_msg.step, dtype.itemsize * channels, dtype.itemsize)
        conv_img = cv2.GaussianBlur(img, (15, 15), cv2.BORDER_WRAP)
        conv_vector = np.reshape((conv_img[7, :] + conv_img[8, :]) / 2, -1)
        print(conv_vector)
        if self._publish_vector:
            vector_msg = Float32MultiArray()
            vector_msg.data = conv_vector
            self._vector_publisher.publish(vector_msg)

    def toggle_callback(self, msg: Bool):
        self._publish_vel = msg.data

    def run(self):
        rospy.spin()


def main():
    node = TunnelTraversalNavigator()
    node.run()


if __name__ == "__main__":
    main()
