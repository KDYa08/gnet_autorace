import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

import numpy as np
import cv2
import math

class Parking(LifecycleNode):
    def __init__(self):
        super().__init__('parking')
        self.get_logger().info('Lifecycle node created.')
        self.activate = False 
        
    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
        
        self.mission_state_publisher = self.create_lifecycle_publisher(String, '/mission_state', self.qos_profile)
        self.odom_sbuscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, self.qos_profile)
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.img_callback, self.qos_profile)
            self.get_logger().info("sim node mode")
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.img_callback, self.qos_profile)
            self.get_logger().info("real node mode")
        self.create_subscription(String, '/parking_action', self.check_action, self.qos_profile)
        self.direction_publisher = self.create_lifecycle_publisher(String, '/direction', self.qos_profile)
        self.line_motor_publisher = self.create_lifecycle_publisher(String, '/line_motor_state', self.qos_profile)
        self.cmd_vel_publisher = self.create_lifecycle_publisher(TwistStamped, '/cmd_vel', self.qos_profile)
        
        self.checkpoint = False
        self.checkpoint_num = 0
        self.s_x, self.s_y = 0, 0
        self.s_roll, self.s_pitch, self.s_yaw = 0, 0, 0
        self.left_parking = True
        
        self.cv_bridge = CvBridge()
        
        self.get_logger().info('Configuring Complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')
        self.activate = True
        self.mission_state_publisher.on_activate(state)
        self.direction_publisher.on_activate(state)
        self.line_motor_publisher.on_activate(state)
        self.cmd_vel_publisher.on_activate(state)
        
        self.get_logger().info('Activating Complete')
        return super().on_activate(state)

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating...')
        self.activate = False
        self.mission_state_publisher.on_deactivate(state)
        self.direction_publisher.on_deactivate(state)
        self.line_motor_publisher.on_deactivate(state)
        self.cmd_vel_publisher.on_deactivate(state)
        cv2.destroyAllWindows()
        
        self.get_logger().info('Deactivating Complete')
        return super().on_deactivate(state)
        
    def on_cleanup(self, state: State):
        self.get_logger().info('CleanUp...')
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.direction_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        self.destroy_publisher(self.cmd_vel_publisher)
        
        self.get_logger().info('CleanUp Complete')
        return super().on_cleanup(state)
    
    def on_shutdown(self, state: State):
        self.get_logger().info('Shutdowning...')
        self.activate = False
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.direction_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        self.destroy_publisher(self.cmd_vel_publisher)
        
        cv2.destroyAllWindows()
        self.get_logger().info('Shutdowning Complete')
        return super().on_shutdown(state)
    
    def img_callback(self, img):
        if self.activate == True:
            src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")
            dst = cv2.GaussianBlur(src, (0, 0), 2)
            
            gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

            # x, y 외곽선 검출후 영상 병합
            sobelx = cv2.Sobel(gray, cv2.CV_8U, 2, 0, 5)
            sobely = cv2.Sobel(gray, cv2.CV_8U, 0, 2, 5)
            sobelxy = cv2.bitwise_or(sobelx, sobely)
            
            # morpholozy 연산 수행 후 이진화
            kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
            dilate = cv2.dilate(sobelxy, kernel, anchor=(-1, -1), iterations=4)
            erode = cv2.erode(dilate, kernel, anchor=(-1, -1), iterations=2)
            _, thresh = cv2.threshold(erode, 20, 255, cv2.THRESH_BINARY)

            self.get_logger().info(f"{np.count_nonzero(thresh)}")
            if self.checkpoint_num == 3:
                if np.count_nonzero(thresh) >= 10000:
                    self.left_parking = False
                else:
                    self.left_parking = True
                self.checkpoint_num += 1

            cv2.imshow('parking_cam', src)
            cv2.imshow('thresh', thresh)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                twistStamped = TwistStamped()
                self.cmd_vel_publisher.publish(twistStamped)
                cv2.destroyAllWindows()
                self.destroy_node()
                rclpy.shutdown()
    
    def odom_callback(self, odom):
        if self.activate == True:
            if self.checkpoint == False:
                self.s_x, self.s_y = odom.pose.pose.position.x, odom.pose.pose.position.y
                orientation_q = odom.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (self.s_roll, self.s_pitch, self.s_yaw) = euler_from_quaternion(orientation_list)
                self.checkpoint = True
            elif self.checkpoint == True:
                orientation_q = odom.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (c_roll, c_pitch, c_yaw) = euler_from_quaternion(orientation_list)
                
                c_x, c_y = odom.pose.pose.position.x, odom.pose.pose.position.y
                distance = math.sqrt((c_x - self.s_x)**2 + (c_y - self.s_y)**2)
                rotation = np.rad2deg(c_yaw - self.s_yaw)
                self.get_logger().info(f'checkpoint : {self.checkpoint_num}')
                self.get_logger().info(f'distance : {distance:.2f}m, rotation : {rotation:.2f}deg')
                self.get_logger().info(f"\nstart_xy : ({self.s_x}, {self.s_y})\ncurrent_xy : ({c_x}, {c_y})\ndiff_xy : ({c_x - self.s_x}, {c_y - self.s_y})")

                # parking_sign을 감지한 시점부터 parking 구역 앞까지 라인트레이싱으로 이동
                if self.checkpoint_num == 0:
                    if distance >= 0.9:
                        self.checkpoint = False
                        msg = String()
                        msg.data = 'parking'
                        self.checkpoint_num += 1
                        self.line_motor_publisher.publish(msg)
                        self.stop()

                # parking 구역 안으로 진입
                elif self.checkpoint_num == 1:
                    if distance >= 0.4:
                        self.checkpoint = False
                        self.checkpoint_num += 1
                        self.stop()
                    else:
                        self.go_straight()

                elif self.checkpoint_num == 2:
                    if rotation <= -90:
                        self.checkpoint = False
                        self.checkpoint_num += 1
                        self.stop()
                    else:
                        self.turn_right()

                # 앞에 차량이 감지되면 뒤로 주차
                # 차량이 없으면 앞으로 주차
                elif self.checkpoint_num == 4:
                    if self.left_parking:
                        if distance >= 0.3:
                            self.checkpoint = False
                            self.checkpoint_num += 1
                            self.stop()
                        else:
                            self.back_straight()
                    else:
                        if distance >= 0.3:
                            self.checkpoint = False
                            self.checkpoint_num += 1
                            self.stop()
                        else:
                            self.go_straight()
                
                elif self.checkpoint_num == 5:
                    if self.left_parking:
                        if distance >= 0.3:
                            self.checkpoint = False
                            self.checkpoint_num += 1
                            self.stop()
                        else:
                            self.go_straight()
                    else:
                        if distance >= 0.3:
                            self.checkpoint = False
                            self.checkpoint_num += 1
                            self.stop()
                        else:
                            self.back_straight()

                elif self.checkpoint_num == 6:
                    if rotation <= -90:
                        self.checkpoint = False
                        self.checkpoint_num += 1
                        self.stop()
                    else:
                        self.turn_right()

                elif self.checkpoint_num == 7:
                    if distance >= 0.25:
                        msg = String()
                        msg.data = 'left'
                        self.direction_publisher.publish(msg)
                        msg.data = 'stop'
                        self.line_motor_publisher.publish(msg)
                        msg.data = 'parking'
                        self.activate = False
                        cv2.destroyAllWindows()
                        self.mission_state_publisher.publish(msg)
                    else:
                        self.go_straight()
                        
    def go_straight(self):
        msg = TwistStamped()
        msg.linear.x = 0.1
        self.cmd_vel_publisher.publish(msg)
        
    def back_straight(self):
        msg = TwistStamped()
        msg.linear.x = -0.1
        self.cmd_vel_publisher.publish(msg)

    def turn_right(self):
        msg = TwistStamped()
        msg.angular.z = -0.5
        self.cmd_vel_publisher.publish(msg)
    
    def turn_left(self):
        msg = TwistStamped()
        msg.angular.z = 0.5
        self.cmd_vel_publisher.publish(msg)
        
    def stop(self):
        msg = TwistStamped()
        self.cmd_vel_publisher.publish(msg)
    
    def check_action(self, msg):
        data = msg.data
        self.checkpoint_num -= 9
        
def main(args=None):
    rclpy.init(args=args)
    node = Parking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

