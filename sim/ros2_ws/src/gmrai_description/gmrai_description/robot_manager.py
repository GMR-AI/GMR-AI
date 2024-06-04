from enum import Enum
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool
from custom_interfaces.msg import StringStamped
from custom_interfaces.srv import AskModelPath, AskHomePose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import os

from ament_index_python.packages import get_package_share_directory

from gmrai_description.robot_client import *

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEDED = 1
    CANCELED = 2
    FAILED = 3

class RobotManager(Node):
    def __init__(self):
        super().__init__('gmr_manager')
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.behavior_tree = os.path.join(get_package_share_directory('gmrai_description'), 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
        self.model_path = ''
        self.reconstruction_active = False
        self.home_position = [0.0, 0.0, 0.0]

        # Callback groups
        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()
        group3 = ReentrantCallbackGroup()

        # Action Clients
        self.navigation_aclient = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=group1)

        # Clients
        self.reconstruction_client = self.create_client(AskModelPath, 'reconstruction/services/path', callback_group=group2)
        self._reconstruction_request = AskModelPath.Request()
        self.home_client = self.create_client(AskHomePose, 'gmr/services/home_pose', callback_group=group3)
        self._home_request = AskHomePose.Request()

        # Publishers
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.reconstruction_switch_publisher = self.create_publisher(Bool, 'robot_manager/publishers/switch_reconstruction', qos_profile=qos, callback_group=group2)

        # Subscribers
        self.reconstruction_path_subscriber = self.create_subscription(StringStamped, 'reconstruction/publishers/path', self.reconstruction_callback, qos_profile=qos, callback_group=group3)

    # Getters
    def get_reconstruction(self):
        if self.reconstruction_active:
            return self.model_path            
        return self.send_reconstruction_request()

    def get_feedback(self):
        return self.feedback

    # Reconstruction Request and Start
    def send_reconstruction_request(self):
        """AskModelPath.srv (from custom_interfaces)
        # Request
        ---
        # Response
        path: http://docs.ros.org/en/api/std_msgs/html/msg/String.html

        returns: path to file without extension (they are the same with different extensions .ply .obj)
        """
        while not self.reconstruction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reconstruction service not available, waiting again...')

        self.get_logger().info("Requesting reconstruction...")
        future = self.reconstruction_client.call_async(self._reconstruction_request)
        self.executor.spin_until_future_complete(future)
        self.get_logger().info("Got reconstruction path!")
        return future.result().path


    def publish_reconstruction_switch(self, switch):
        msg = Bool()
        msg.data = switch
        self.reconstruction_switch_publisher.publish(msg)


    def reconstruction_callback(self, msg):
        """StringStamped.msg (from custom_interfaces)
        header: http://docs.ros.org/en/api/std_msgs/html/msg/Header.html
        data: http://docs.ros.org/en/api/std_msgs/html/msg/String.html
        """

        self.reconstruction_active = True
        self.model_path = msg.data


    # Home position
    def send_home_pose_request(self):
        """AskHomePosition.srv (from custom_interfaces)
        # Request
        ---
        # Response
        pose: http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
        """
        while not self.home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Home position not available, waiting again...")
        
        self.get_logger().info(f"Requesting home position...")
        future = self.reconstruction_client.call_async(self._home_request)
        self.executor.spin_until_future_complete(future)
        self.get_logger().info("Got home pose!")
        return future.result().pose

    # Navigation Callbacks and Request
    def send_navigation_goal(self, position):
        self.get_logger().info(f"Preparing goal to {position}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.behavior_tree = self.behavior_tree
        
        goal_msg.pose.header.stamp # Fill with sim time
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = position[0]
        goal_msg.pose.pose.position.y = position[1]
        goal_msg.pose.pose.position.z = position[2]
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        self.get_logger().info(f"Prepared goal to {position}!")

        self.get_logger().info("Sending goal to navigation server...")
        future = self.navigation_aclient.send_goal_async(goal_msg)
        self.executor.spin_until_future_complete(future)
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info(f"Action rejected :<")
            return False
        
        self.get_logger().info(f"Sent goal to navigation server!")
        self.result_future = self.goal_handle.get_result_async()
        return True

    def start_navigation(self, position):
        accepted = self.send_navigation_goal(position)
        return accepted
    
    def is_navigation_finished(self):
        if not self.result_future:
            return True
        
        self.executor.spin_until_future_complete(self.result_future, timeout_sec=0.100)
        if self.result_future.result():
            status = self.result_future.result().result
            if status != 0:
                self.get_logger().info(f'Task failed with status code: {status}')
                return True
            else:
                self.get_logger().info(f"Task finished succesfully! {status}")
                return True
        else:
            return False

    def cancel_navigation(self):
        cancel_goal_future = self.goal_handle.cancel_goal_async()
        self.executor.spin_until_future_complete(cancel_goal_future)
        self.go_home()

    def go_home(self):
        self.send_navigation_goal()

    def startup(self, node_name = 'bt_navigator'):
        # Waits for the node within the tester namespace to become active
        self.get_logger().info(f'Waiting for {node_name} to become active...')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{node_service} service not available, waiting...')
        
        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.get_logger().info(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            self.executor.spin_until_future_complete(future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().info(f'Result of get_state: {state}')
            time.sleep(2)
        return


def test_new_job(robot_manager: RobotManager):
    executor = MultiThreadedExecutor()
    executor.add_node(robot_manager)

    robot_manager.get_logger().info(f'Starting first reconstruction...')
    path = robot_manager.get_reconstruction()
    robot_manager.get_logger().info(f'Reconstruction path: {path}')
    robot_manager.get_logger().info(f'Starting second reconstruction...')
    path = robot_manager.get_reconstruction()
    robot_manager.get_logger().info(f'Reconstruction path: {path}')


def test_start_job(robot_manager: RobotManager):

    executor = MultiThreadedExecutor()
    executor.add_node(robot_manager)

    robot_manager.startup()

    # Example position
    position = [0.0, 3.0, 0.0]

    robot_manager.publish_reconstruction_switch(True)
    robot_manager.start_navigation(position)

    while not robot_manager.is_navigation_finished():
        # To give feedback
        robot_manager.get_logger().info(f"Feedback: {robot_manager.get_feedback()}")
        time.sleep(1)
        continue
    
    robot_manager.publish_reconstruction_switch(False)
    robot_manager.get_logger().info(f"Finished navigation to {position}")

def test_cancel_job(robot_manager: RobotManager):
    executor = MultiThreadedExecutor()
    executor.add_node(robot_manager)

    robot_manager.startup()

    # Example position
    position = [0.0, 3.0, 0.0]

    robot_manager.publish_reconstruction_switch(True)
    robot_manager.start_navigation(position)

    while not robot_manager.is_navigation_finished():
        # To give feedback
        robot_manager.get_logger().info(f"Feedback: {robot_manager.get_feedback()}")
        time.sleep(1)
        continue

def main():
    rclpy.init()

    manager = RobotManager()
    
    # test_new_job(manager)
    test_start_job(manager)
    # test_cancel_job(manager)

    rclpy.shutdown()
