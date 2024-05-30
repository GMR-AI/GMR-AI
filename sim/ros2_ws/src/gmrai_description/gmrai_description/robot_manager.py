from enum import Enum
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from custom_interfaces.msg import StringStamped
from custom_interfaces.srv import AskModelPath
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
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
        self.status = None
        self.feedback = None
        self.behavior_tree = os.path.join(get_package_share_directory('gmrai_description'), 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
        self.model_path = ''
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Clients
        self.reconstruction_client = self.create_client(AskModelPath, 'reconstruction/services/path')
        while not self.reconstruction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reconstruction service not available, waiting again...')
        self._reconstruction_request = AskModelPath.Request()


        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.model_subscriber = self.create_subscription(StringStamped, 'model/path', self.model_callback, qos_profile=qos)
        # self.model_path_client = self.create_client(AskModelPath, 'reconstruction/ask')


    def get_reconstruction(self):
        return self.send_reconstruction_request()


    # Reconstruction request
    def send_reconstruction_request(self):
        """AskModelPath.srv (from custom_interfaces)
        # Request
        ---
        # Response
        header: http://docs.ros.org/en/api/std_msgs/html/msg/Header.html
        path: http://docs.ros.org/en/api/std_msgs/html/msg/String.html

        returns: path to file without extension (they are the same with different extensions .ply .obj)
        """
        self.get_logger().info("Requesting reconstruction...")
        self.future = self.reconstruction_client.call_async(self._reconstruction_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Got reconstruction path!")
        return self.future().result().path


    def navigate(self, position):
        """Send a NavigateToPose action request."""
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = position[0]
        goal_msg.pose.pose.position.y = position[1]
        goal_msg.pose.pose.position.z = position[2]
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        goal_msg.behavior_tree = self.behavior_tree
        
        self.get_logger().info('Navigating to ' + str(position))
        send_goal_future = self.nav_client.send_goal_async(goal_msg, self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Navigate to pose request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancel_navigation(self):
        cancel_goal_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_goal_future)

    def is_task_completed(self):
        """Check if the task request of any type is completed yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.100)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Task failed with status code: {self.status}')
                return True
        else:
            # Timeout, still processing, not completed yet
            return False
        
        self.get_logger().info('Task succeded!')
        return True
    
    def _feedback_callback(self, msg):
        self.feedback = msg.feedback
        return
    
    def get_feedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def get_result(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN
    
    def get_model_path(self):
        rclpy.spin_once(self, timeout_sec=1)
        return self.model_path

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
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().info(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def model_callback(self, msg):
        self.model_path = msg.data


def test_new_job(robot_manager: RobotManager):
    path = robot_manager.get_reconstruction()
    robot_manager.get_logger().info(f'Reconstruction path: {path}')

def main():
    rclpy.init()

    manager = RobotManager()
    test_new_job(manager)
    # manager.startup()

    #     # Example position
    # position = [0.0, 3.0, 0.0]

    # manager.navigate(position)

    # i = 0
    # while not manager.is_task_completed():
    #     # Do something with the feedback
    #     i += 1
    #     feedback = manager.get_feedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #             Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #             + ' seconds.')
    #     time.sleep(1)

    # # Do something depending on the return code
    # result = manager.get_result()
    # if result == TaskResult.SUCCEDED:
    #     print('Goal succeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')
