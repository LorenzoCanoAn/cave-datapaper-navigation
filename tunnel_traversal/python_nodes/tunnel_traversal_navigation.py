#!/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, Float32
import numpy as np
import cv2


def angle_rad_to_index(angle):
    angle_deg = np.rad2deg(angle)
    return angle_deg_to_index(angle)


def angle_deg_to_index(angle):
    angle = int(angle)
    index = (-angle + 180) % 360
    return index


def index_to_angle_deg(index):
    angle = (-index + 360) % 360
    angle = (angle - 180) % 360
    return angle


def index_to_angle_rad(index):
    return np.deg2rad(index_to_angle_deg(index))


def warp_rad_angle_pi(angle):
    if angle > np.pi:
        return angle - 2 * np.pi
    return angle


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
        self._publish_vel = True

    def image_callback(self, ptcl_msg: Image):
        dtype = np.dtype(np.float32)
        channels = 1
        dtype = dtype.newbyteorder(">" if ptcl_msg.is_bigendian else "<")
        shape = (ptcl_msg.height, ptcl_msg.width, channels)
        img = np.frombuffer(ptcl_msg.data, dtype=dtype).reshape(shape)
        img.strides = (ptcl_msg.step, dtype.itemsize * channels, dtype.itemsize)
        img = np.array(img)
        # img[np.where(img == 0.0)] = 1
        # To detect an obstacle, get the elements of the image in the middle
        conv_img = cv2.GaussianBlur(img, (15, 31), cv2.BORDER_WRAP)
        conv_vector = np.reshape((conv_img[7, :] + conv_img[8, :]) / 2, -1)
        if self._publish_vector:
            vector_msg = Float32MultiArray()
            vector_msg.data = conv_vector
            self._vector_publisher.publish(vector_msg)
        idxi = angle_deg_to_index(self._frontal_angles_range[0])
        idxj = angle_deg_to_index(self._frontal_angles_range[1])
        idxk = idxj + np.argmax(conv_vector[idxj:idxi])
        angle_rad_to_advance = warp_rad_angle_pi(index_to_angle_rad(idxk))
        # To correct the angle so that the robot is centered
        vector_at_medium_height = img[7, :]
        distances_at_the_right = vector_at_medium_height[
            angle_deg_to_index(-80) : angle_deg_to_index(-100)
        ]
        distances_at_the_left = vector_at_medium_height[
            angle_deg_to_index(100) : angle_deg_to_index(80)
        ]
        avg_dist_right = np.average(distances_at_the_right)
        avg_dist_left = np.average(distances_at_the_left)
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
