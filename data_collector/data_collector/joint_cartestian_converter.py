import time

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np
import pandas as pd

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import rclpy.task
from sensor_msgs.msg import JointState

from moveit_msgs.msg import (
    RobotState,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetPositionFK



class JointCartesianConverter(Node):

    joint_data_file_path = ""
    
    move_group_name_ = "arm"
    namespace_ = "lbr"

    joint_state_topic_ = "joint_states"
    lbr_state_topic_ = "state"
    pose_topic_ = "pose"
    plan_srv_name_ = "plan_kinematic_path"
    ik_srv_name_ = "compute_ik"
    fk_srv_name_ = "compute_fk"
    execute_action_name_ = "execute_trajectory"
    fri_execute_action_name_ = "joint_trajectory_controller/follow_joint_trajectory"

    base_ = "link_0"
    end_effector_ = "link_ee"

    def __init__(self):
        super().__init__("joint_cartesian_converter")

        self.fk_client_callback = MutuallyExclusiveCallbackGroup()
        self.fk_client_ = self.create_client(GetPositionFK, self.fk_srv_name_, callback_group=self.fk_client_callback)
        if not self.fk_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error("FK service not available.")
            exit(1)

        self.joint_data = pd.read_csv(self.joint_data_file_path)

    def quat_2_euler(self, quat):
        """calculates and returns: yaw, pitch, roll from given quaternion"""
        return R.from_quat(quat).as_euler("xyz")

    def convert(self):
        
        # Convert each row in the dataframe to cartesian coordinates and save in the dataframe
        for index, row in self.joint_data.iterrows():
            
            joint_state = JointState()
            joint_state.name = ""
            joint_state.position = [row["P1"], row["P2"], row["P3"], row["P4"], row["P5"], row["P7"], row["P6"]]
            joint_state.velocity = [row["V1"], row["V2"], row["V3"], row["V4"], row["V5"], row["V7"], row["V6"]]
            joint_state.effort = [row["E1"], row["E2"], row["E3"], row["E4"], row["E5"], row["E7"], row["E6"]]
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = f"{self.namespace_}/{self.base_}"
            current_robot_state = RobotState()
            current_robot_state.joint_state = joint_state

            request = GetPositionFK.Request()

            request.header.frame_id = f"{self.namespace_}/{self.base_}"
            request.header.stamp = self.get_clock().now().to_msg()

            request.fk_link_names.append(self.end_effector_)
            request.robot_state = current_robot_state

            future = self.fk_client_.call_async(request)

            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.get_logger().error("Failed to get FK solution")
                return None
            
            response = future.result()
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"Failed to get FK solution: {response.error_code.val}"
                )
                return None
            
            pose = response.pose_stamped[0].pose
            euler_pose = self.quat_2_euler(pose.orientation)
            self.joint_data.at[index, "X"] = pose.position.x
            self.joint_data.at[index, "Y"] = pose.position.y
            self.joint_data.at[index, "Z"] = pose.position.z
            self.joint_data.at[index, "QX"] = pose.orientation.x
            self.joint_data.at[index, "QY"] = pose.orientation.y
            self.joint_data.at[index, "QZ"] = pose.orientation.z
            self.joint_data.at[index, "QW"] = pose.orientation.w
            self.joint_data.at[index, "R"] = euler_pose[0]
            self.joint_data.at[index, "P"] = euler_pose[1]
            self.joint_data.at[index, "Y"] = euler_pose[2]

            # Compute the end_effector velocity

        self.joint_data.to_csv(self.joint_data_file_path, index=False)

def main(args=None):
    rclpy.init(args=args)

    joint_cartesian_converter = JointCartesianConverter()
    joint_cartesian_converter.convert()

    rclpy.shutdown()