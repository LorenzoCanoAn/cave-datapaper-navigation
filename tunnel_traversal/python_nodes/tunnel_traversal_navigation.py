#!/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, Float32
import numpy as np
import cv2


def warp_2pi_rad(angle_rad):
    while angle_rad < 0:
        angle_rad += 2 * np.pi
    return angle_rad % (2 * np.pi)


def warp_360_deg(angle_deg):
    while angle_deg < 0:
        angle_deg += 360
    return angle_deg % 360


class TunnelTraversalNavigator:
    def __init__(self):
        rospy.init_node("tunnel_traversal_navigator")
        toggle_topic = rospy.get_param("~toggle_topic")
        image_topic = rospy.get_param("~image_topic")
        angle_topic = rospy.get_param("~angle_topic")
        self._frontal_angles_range = rospy.get_param(
            "~frontal_angles_range_deg", (-40, 40)
        )
        self._publish_vector = rospy.get_param("~config/publish_vector", default=True)
        self.toggle_subscriber = rospy.Subscriber(
            toggle_topic, Bool, callback=self.toggle_callback
        )
        self.ptcl_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback
        )
        self.angle_pub = rospy.Publisher(angle_topic, Float32, queue_size=2)
        if self._publish_vector:
            self._vector_publisher = rospy.Publisher(
                "internals/vector", Float32MultiArray, queue_size=2
            )
        self._publish_vel = False

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
        distances_at_the_right = vector_at_medium_height[range(-100, -80)]
        distances_at_the_left = vector_at_medium_height[range(80, 100)]
        avg_dist_right = np.mean(distances_at_the_right)
        avg_dist_left = np.mean(distances_at_the_left)
        print(f"left: {avg_dist_left}, right: {avg_dist_right}")
        if avg_dist_left > avg_dist_right:
            angle_rad_to_advance += np.deg2rad(2)
        else:
            angle_rad_to_advance -= np.deg2rad(2)
        if self._publish_vel:
            self.angle_pub.publish(Float32(angle_rad_to_advance))

    def toggle_callback(self, msg: Bool):
        print("Toggle callback recieved")
        self._publish_vel = msg.data

    def run(self):
        rospy.spin()


def main():
    node = TunnelTraversalNavigator()
    node.run()


if __name__ == "__main__":
    main()
