import rclpy

from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

import math
import signal
import subprocess
import os
import time


class Nav2Manager(LifecycleNode):

    def __init__(self):
        super().__init__('tunnel')
        self.get_logger().info('Lifecycle node created.')

    def on_configure(self, state):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        # callback 함수를 병렬로 실행하기 위해 사용
        self.callback_group = ReentrantCallbackGroup()

        self.mission_state_publisher = self.create_lifecycle_publisher(String, '/mission_state', self.qos_profile)
        self.line_motor_publisher = self.create_lifecycle_publisher(String, '/line_motor_state', self.qos_profile)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', self.qos_profile)
        self.goal_client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.callback_group)
        self.state_client = self.create_client(GetState, '/bt_navigator/get_state', callback_group=self.callback_group)

        # 메시지 송신 여부
        self.initial_pose_sent = False
        self.goal_sent = False

        # 동작 활성화 여부
        self.nav2_active = False
        self.shutdown_started = False

        # 초기 위치
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # 목표 위치
        self.goal_x = 2.0
        self.goal_y = 1.0
        self.goal_yaw = 0.0

        # nav2 상태
        self.nav2_state = 'unknown'
        self.nav2_process = None
        
        self.timer = self.create_timer(0.5, self.check_nav2_status, callback_group=self.callback_group)

        self.get_logger().info('Configuring Complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating...')
        self.mission_state_publisher.on_activate(state)
        self.line_motor_publisher.on_activate(state)
        self.start_nav2()

        self.get_logger().info('Activating Complete')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        self.mission_state_publisher.on_deactivate(state)
        self.line_motor_publisher.on_deactivate(state)
        self.cancel_goal()
        self.stop_nav2()

        self.get_logger().info('Deactivating Complete')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('CleanUp...')
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        self.destroy_publisher(self.initial_pose_publisher)
        self.stop_nav2()

        if self.timer is not None:
            self.timer.cancel()

        self.get_logger().info('CleanUp Complete')
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        self.get_logger().info('Shutdowning...')
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        self.cancel_goal()
        self.stop_nav2()

        self.get_logger().info('Shutdowning Complete')
        return super().on_shutdown(state)

    def start_nav2(self):
        # nav2가 이미 실행중인 상태
        if self.nav2_process is not None:
            if self.nav2_process.poll() is None:
                self.get_logger().warn('Nav2 is already running')
                return True

        # map 저장 경로
        map_path = '/home/{user}/map.yaml'

        self.get_logger().info('Starting Nav2...')

        try:
            # nav2 실행
            self.nav2_process = subprocess.Popen(
                [
                    'ros2',
                    'launch',
                    'turtlebot3_navigation2',
                    'navigation2.launch.py',
                    'use_sim_time:=True',
                    f'map:={map_path}',
                ],

                start_new_session=True
            )

        except Exception as e:
            self.get_logger().error(f'Failed to start Nav2: {e}')
            self.nav2_process = None
            return False

        self.get_logger().info(f'Nav2 started. PID={self.nav2_process.pid}')
        return True

    def check_nav2_status(self):
        # 목표 지점을 보냈을 시 생략
        if self.goal_sent:
            return

        # nav2가 활성화 되이지 않을 때
        if not self.state_client.service_is_ready():
            self.nav2_state = 'unknown'
            self.get_logger().info('Nav2 lifecycle service is not ready')
            return

        # nav2 상태 확인 요청
        request = GetState.Request()
        future = self.state_client.call_async(request)
        future.add_done_callback(self.nav2_state_callback)

    def nav2_state_callback(self, future):
        try:
            response = future.result()

        except Exception as e:
            self.get_logger().error(f'Failed to get Nav2 state: {e}')
            self.nav2_state = 'unknown'
            return

        # nav2 srv 상태를 저장
        state_id = response.current_state.id
        state_label = response.current_state.label

        # 상태가 변경되면 변경된 상태를 저장하고 출력
        if self.nav2_state != state_label:
            self.nav2_state = state_label
            self.get_logger().warn(f'Nav2 state changed: {state_label} ({state_id})')

        # nav2 상태가 configure라면 Activate로 변경
        if state_id == State.PRIMARY_STATE_INACTIVE:
            if not self.nav2_active:
                self.get_logger().info('Nav2 is ACTIVE')
                self.nav2_active = True
                self.on_nav2_active()

    def on_nav2_active(self):
        if self.goal_sent:
            return

        # Initial pose
        if not self.initial_pose_sent:
            self.send_initial_pose()
            self.create_timer(1.0, self.send_goal, callback_group=self.callback_group)

    def send_initial_pose(self):
        if self.initial_pose_sent:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = (self.get_clock().now().to_msg())
        msg.header.frame_id = 'map'

        # Position
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0

        # Quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)

        # Publish
        self.initial_pose_publisher.publish(msg)
        self.initial_pose_sent = True
        self.get_logger().info('Initial pose published')
        self.get_logger().info(f'x={self.initial_x}, y={self.initial_y}, yaw={self.initial_yaw}')

    def send_goal(self):
        if self.goal_sent:
            return

        # nav2 active 확인
        if not self.nav2_active:
            self.get_logger().warn('Nav2 is not ACTIVE. Goal not sent.')
            return

        # Action server 확인
        if not self.goal_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('/navigate_to_pose action server is not ready')
            return

        # Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = (self.get_clock().now().to_msg())
        goal_msg.pose.header.frame_id = 'map'

        # Position
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0

        # Quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)

        self.get_logger().info('Sending NavigateToPose goal')
        self.get_logger().info(f'x={self.goal_x}, y={self.goal_y}, yaw={self.goal_yaw}')

        # Send async
        future = self.goal_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()

        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')
            return

        # Goal rejected
        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal REJECTED')
            return

        # Goal accepted
        self.goal_sent = True
        self.goal_handle = goal_handle
        self.get_logger().info('NavigateToPose goal ACCEPTED')

        # Result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result()

        except Exception as e:
            self.get_logger().error(f'Failed to receive goal result: {e}')
            return

        # Action 수행 결과 출력
        status = result.status
        self.get_logger().info(f'Navigation finished. status={status}')
        
        # 목표 지점 이동 성공 시 status=4
        if status == 4:
            msg = String()
            msg.data = 'play'
            self.line_motor_publisher.publish(msg)
            msg.data = 'tunnel'
            self.mission_state_publisher.publish(msg)
            

    def cancel_goal(self):
        # self.goal_handle 변수가 존재하는지 확인
        # 목표 지점으로 이동 중인 상태
        if not hasattr(self, 'goal_handle'):
            return

        if self.goal_handle is None:
            return

        try:
            self.get_logger().info('Canceling NavigateToPose goal...')
            self.goal_handle.cancel_goal_async()

        except Exception as e:
            self.get_logger().warn(f'Failed to cancel goal: {e}')

        self.goal_handle = None

    def stop_nav2(self):
        # 중복 종료 방지
        if self.shutdown_started:
            return

        self.shutdown_started = True

        process = self.nav2_process
        if process is None:
            return

        self.get_logger().info('Stopping Nav2...')

        # 이미 종료된 경우
        if process.poll() is not None:
            self.get_logger().info('Nav2 process already terminated')
            self.nav2_process = None
            return

        try:
            # 새로운 process group 전체에 SIGINT
            pgid = os.getpgid(process.pid)

            self.get_logger().info(f'Sending SIGINT to process group {pgid}')
            # terminal 종료 신호
            os.killpg(pgid, signal.SIGINT)

            # 정상 종료 대기
            try:
                process.wait(timeout=8.0)
                self.get_logger().info('Nav2 terminated normally')
                return

            except subprocess.TimeoutExpired:
                self.get_logger().warn('Nav2 did not terminate after SIGINT')

            # SIGTERM
            self.get_logger().warn('Sending SIGTERM to Nav2 process group')

            try:
                # 종료 요청 송신
                os.killpg(pgid, signal.SIGTERM)

            except ProcessLookupError:
                pass

            try:
                process.wait(timeout=5.0)
                self.get_logger().info('Nav2 terminated after SIGTERM')
                return

            except subprocess.TimeoutExpired:
                self.get_logger().warn('Nav2 still running')


            # 최종 SIGKILL
            self.get_logger().error('Force killing Nav2 process group')

            try:
                # 강제 종료 명령
                os.killpg(pgid, signal.SIGKILL)

            except ProcessLookupError:
                pass

            try:
                process.wait(timeout=3.0)

            except subprocess.TimeoutExpired:
                self.get_logger().error('Failed to terminate Nav2 process')

        except ProcessLookupError:
            self.get_logger().info('Nav2 process group already gone')

        except Exception as e:
            self.get_logger().error(f'Error while stopping Nav2: {e}')

        finally:
            self.nav2_process = None
            self.get_logger().info('Nav2 stop procedure complete')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Manager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received')

    finally:
        # 실행 중인 프로그램 종료 신호 송신
        try:
            node.cancel_goal()
        except Exception as e:
            node.get_logger().warn(
                f'Goal cancel error: {e}'
            )

        try:
            node.stop_nav2()
        except Exception as e:
            node.get_logger().error(
                f'Nav2 shutdown error: {e}'
            )

        try:
            node.destroy_node()
        except Exception as e:
            print(
                f'Failed to destroy node: {e}'
            )

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
