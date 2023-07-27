import rospy
import gazebo_msgs.srv as gazebo_srv
import math
import numpy as np
import std_msgs.msg as std_msgs


def quaternion_to_euler(qx, qy, qz, qw):
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(2 * (qw * qy - qx * qz))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    # Convert angles to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    return roll, pitch, yaw


class RobotPoseRecorder:
    def __init__(self, robot_name, frequency):
        self.robot_name = robot_name
        self.frequency = frequency
        rospy.init_node("Robot pose recorder")
        self.gazebo_get_model_state_proxy = rospy.ServiceProxy(
            "/gazebo/get_model_state", gazebo_srv.GetModelState, persistent=True
        )
        rospy.Subscriber(
            "~enable_recording", std_msgs.Boold, self.enable_recording_callback, 1
        )
        self.enable_recording = False
        self.data = np.zeros(0, 6)

    def enable_recording_callback(self, msg: std_msgs.Bool):
        self.enable_recording = msg.data

    def run(self):
        while not rospy.is_shutdown():
            if self.enable_recording:
                response = self.gazebo_get_model_state_proxy(
                    gazebo_srv.GetModelStateRequest(self.robot_name, "")
                )
                assert isinstance(response, gazebo_srv.GetModelStateResponse)
                x = response.pose.position.x
                y = response.pose.position.y
                z = response.pose.position.z
                qx = response.pose.orientation.x
                qy = response.pose.orientation.y
                qz = response.pose.orientation.z
                qw = response.pose.orientation.w
                r, p, yw = quaternion_to_euler(qx, qy, qz, qw)
                pose_vector = np.array(x, y, z, r, p, yw)
                self.data = np.vstack([self.data, pose_vector])
