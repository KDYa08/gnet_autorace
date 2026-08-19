import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from nav2_msgs.action import NavigateToPose

import math
import signal
import subprocess

class Nav2Manager(LifecycleNode):

    def __init__(self):
        super().__init__('tunnel')
        self.get_logger().info('Lifecycle node created.')

    def on_configure(self, state):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)

        self.callback_group = ReentrantCallbackGroup()
        self.nav2_process = None
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.goal_x = 2.0
        self.goal_y = 1.0
        self.goal_yaw = 0.0
        
        self.activate = False

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', self.qos_profile)
        self.goal_pub = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.timer = self.create_timer(0.5, self.check_and_play)
        
        self.get_logger().info('Configuring Complete')
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state):
        self.get_logger().info('Activating...')
        self.start_nav2()

        self.get_logger().info('Activating Complete')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        twistStamped = TwistStamped()
        self.cmd_vel_publisher.publish(twistStamped)

        self.get_logger().info('Deactivating Complete')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('CleanUp...')
        self.stop_nav2()
        
        self.get_logger().info('CleanUp Complete')
        return super().on_cleanup(state)


    def on_shutdown(self, state):
        self.get_logger().info('Shutdowning...')

        self.stop_nav2()
        
        self.get_logger().info('Shutdowning Complete')
        return super().on_shutdown(state)

    def check_and_play(self):
        self.initial_pose_check = self.count_subscribers('/initialpose')
        if self.initial_pose_check == 1:
            self.start_nav2()
            self.send_goal()
            self.timer.cancel()

    # ==========================================================
    # Nav2 launch 실행
    # ==========================================================

    def start_nav2(self):
        map_path = '/home/kdya08/map.yaml'

        try:

            self.nav2_process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'turtlebot3_navigation2',
                    'navigation2.launch.py',
                    'use_sim_time:=True',
                    f'map:={map_path}',
                ]
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to launch Nav2: {e}'
            )

            return False

        self.get_logger().info(
            f'Nav2 started. PID={self.nav2_process.pid}'
        )

    def send_initial_pose(self):

        msg = PoseWithCovarianceStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        # ------------------------------------------------------
        # Position
        # ------------------------------------------------------

        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0

        # ------------------------------------------------------
        # Quaternion
        # ------------------------------------------------------

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0

        msg.pose.pose.orientation.z = math.sin(
            self.initial_yaw / 2.0
        )

        msg.pose.pose.orientation.w = math.cos(
            self.initial_yaw / 2.0
        )

        # ------------------------------------------------------
        # Covariance
        # ------------------------------------------------------

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        # ------------------------------------------------------
        # Publish
        # ------------------------------------------------------

        self.initial_pose_pub.publish(msg)

        self.initial_pose_sent = True

        self.get_logger().info(
            'Initial pose published'
        )

        self.get_logger().info(
            f'x={self.initial_x}, '
            f'y={self.initial_y}, '
            f'yaw={self.initial_yaw}'
        )

    def send_goal(self):

        if self.goal_sent:

            return

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        # ------------------------------------------------------
        # Position
        # ------------------------------------------------------

        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = 0.0

        # ------------------------------------------------------
        # Quaternion
        # ------------------------------------------------------

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0

        msg.pose.orientation.z = math.sin(
            self.goal_yaw / 2.0
        )

        msg.pose.orientation.w = math.cos(
            self.goal_yaw / 2.0
        )

        # ------------------------------------------------------
        # Publish
        # ------------------------------------------------------

        self.goal_pub.send_goal(msg)

        self.goal_sent = True

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Goal published'
        )

        self.get_logger().info(
            f'x={self.goal_x}, '
            f'y={self.goal_y}, '
            f'yaw={self.goal_yaw}'
        )

        self.get_logger().info(
            '========================================'
        )

    # ==========================================================
    # Nav2 종료
    # ==========================================================

    def stop_nav2(self):

        if self.nav2_process is None:

            return

        if self.nav2_process.poll() is not None:

            self.nav2_process = None

            return

        self.get_logger().info(
            'Stopping Nav2...'
        )

        try:

            self.nav2_process.send_signal(
                signal.SIGINT
            )

            self.nav2_process.wait(
                timeout=5
            )

        except subprocess.TimeoutExpired:

            self.get_logger().warn(
                'Nav2 did not terminate. Killing process.'
            )

            self.nav2_process.kill()

        except Exception as e:

            self.get_logger().error(
                f'Failed to stop Nav2: {e}'
            )

        self.nav2_process = None


# ==============================================================
# Main
# ==============================================================

def main(args=None):

    rclpy.init(args=args)

    node = Nav2Manager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if rclpy.ok():
            try:
                node.trigger_shutdown()
                node.nav2_process.kill()
            except Exception:
                pass
        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
