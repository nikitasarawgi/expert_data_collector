import rclpy
import os
import pandas as pd
import csv
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import WrenchStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer

class KukaDataRecorder(Node):
    def __init__(self):
        super().__init__("kuka_data_recorder")

        self.data_folder = ""
        self.joint_states_file = ""
        # Create two folders for the images
        self.imageA_folder = os.path.join(self.data_folder, "imageA")
        self.imageB_folder = os.path.join(self.data_folder, "imageB")
        os.makedirs(self.imageA_folder, exist_ok=True)
        os.makedirs(self.imageB_folder, exist_ok=True)

        # Initialize everything for trajectory replaying
        self.feedback = None
        self._action_client = ActionClient(self, FollowJointTrajectory, '/lbr/joint_trajectory_controller/follow_joint_trajectory')
        self.joint_names = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
        self.joint_trajectories = self.readJoinStatesFromCsv(os.path.join(self.data_folder, "processed", self.joint_states_file))
        # This can be done through goal sending
        self.initializeRobotPose()

        # Create a synchronized callback for all the information
        self.imageA_topic = ""
        self.imageB_topic = ""
        self.joint_state_topic = ""
        self.wrench_topic = ""

        self.imageA_sub = Subscriber(self, Image, self.imageA_topic)
        self.imageB_sub = Subscriber(self, Image, self.imageB_topic)
        self.joint_state_sub = Subscriber(self, JointState, self.joint_state_topic)
        self.wrench_sub = Subscriber(self, WrenchStamped, self.wrench_topic)

        self.synchronizer =  ApproximateTimeSynchronizer(
            [self.imageA_sub, self.imageB_sub, self.joint_state_sub, self.wrench_sub],
            queue_size=100,
            slop=0.01,
        )

        self.synchronizer.registerCallback(self.recordSyncData)
        self.get_logger().info("Data Synchronizer Node Initialized")

        self.dataframe = pd.DataFrame(columns=[
            'timestamp', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7',
            'V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7',
            'E1', 'E2', 'E3', 'E4', 'E5', 'E6', 'E7', 'Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz'
        ])

        # This needs to be done through moveit planner since the points need to be interpolated to a trajectory
        self.executeTrajectory()


    def recordSyncData(self, imageA_msg, imageB_msg, joint_state_msg, wrench_msg):
        # The timestamp of imageA_msg will be used to name the columns
        timestamp = imageA_msg.header.stamp

        self.get_logger().info(f"Recording data at timestamp {timestamp}")

        # Save the joint state
        data_row = {'timestamp': timestamp}

        for i in range(7):
            # imp imp: Remember that kuka publishes A1, A2, A3, A4, A5, A7, A6 (NOTE THE ORDER)
            j = i
            if i == 5:
                j = 6
            if i == 6: 
                j = 5
            data_row[f'P{j+1}'] = joint_state_msg.position[j]
            data_row[f'V{j+1}'] = joint_state_msg.velocity[j]
            data_row[f'E{j+1}'] = joint_state_msg.effort[j]

        # Save the wrench
        data_row['Fx'] = wrench_msg.wrench.force.x
        data_row['Fy'] = wrench_msg.wrench.force.y
        data_row['Fz'] = wrench_msg.wrench.force.z
        data_row['Tx'] = wrench_msg.wrench.torque.x
        data_row['Ty'] = wrench_msg.wrench.torque.y
        data_row['Tz'] = wrench_msg.wrench.torque.z

        self.dataframe = self.dataframe.append(data_row, ignore_index=True)

        # Save the images
        imageA_filename = os.path.join(self.imageA_folder, f"{timestamp}_A.jpg")
        imageB_filename = os.path.join(self.imageB_folder, f"{timestamp}_B.jpg")
        with open(imageA_filename, "wb") as f:
            f.write(imageA_msg.data)
        with open(imageB_filename, "wb") as f:
            f.write(imageB_msg.data)

    def readJoinStatesFromCsv(self, file_path):
        joint_states = []
        with open(file_path, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                joint_states.append([math.radians(float(row['A1'])), math.radians(float(row['A2'])), math.radians(float(row['A3'])),
                                    math.radians(float(row['A4'])), math.radians(float(row['A5'])), math.radians(float(row['A6'])), math.radians(float(row['A7']))])
        return joint_states

    def initializeRobotPose(self):
        # The robot needs to be initialized to the first position of the trajectory 
        # to ensure that there is no sudden movement when the trajectory is replayed
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        self.get_logger().info(f"Processing trajectory point init")

        point = JointTrajectoryPoint()
        point.positions = self.joint_trajectories[0]
        point.time_from_start.sec = 8  # Set the seconds part to 0

        trajectory_msg.points.append(point)
        goal_msg.trajectory = trajectory_msg
        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info(f"Goal for init point was rejected")
            return
        
        self.get_logger().info(f"Goal for init point was accepted")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        self.get_logger().info(f"Result : {result}, Initial position reached")

    def executeTrajectory(self):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        for i, joint_values in enumerate(self.joint_trajectories):
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.time_from_start.sec = 1 * i
            point.time_from_start.nanosec = 0
            trajectory_msg.points.append(point)
        goal_msg.trajectory = trajectory_msg
        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info(f"Goal for trajectory was rejected")
            return
        
        self.get_logger().info(f"Goal for trajectory was accepted")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        self.get_logger().info(f"Result : {result}, Trajectory executed?")

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback
