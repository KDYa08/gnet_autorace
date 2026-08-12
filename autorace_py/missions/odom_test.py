import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

import numpy as np
import cv2
import math

class OdomTest(Node):
    def __init__(self):
        super().__init__('odom_test')       
        self.qos_profile = QoSProfile(depth=10)
        self.odom_sbuscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, self.qos_profile)
        self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.img_callback, self.qos_profile)
        self.Lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, self.qos_profile)
        self.create_subscription(String, '/start_sign', self.start, self.qos_profile)
        
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
        
        self.odom_status = False
        self.s_x, self.s_y = 0, 0
        self.s_roll, self.s_pitch, self.s_yaw = 0, 0, 0
        
        
        self.width, self.height = 550, 550
        self.point_cloud = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.center = (self.width // 2, self.height // 2)
        self.visualization_scope = 200
        
        self.cv_bridge = CvBridge()
    
    def img_callback(self, img):
        src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")

        cv2.imshow('src', src)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            twistStamped = TwistStamped()
            self.cmd_vel_publisher.publish(twistStamped)
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
    
    def odom_callback(self, odom):
        if self.odom_status == False:
            self.s_x, self.s_y = odom.pose.pose.position.x, odom.pose.pose.position.y
            orientation_q = odom.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (self.s_roll, self.s_pitch, self.s_yaw) = euler_from_quaternion(orientation_list)
        elif self.odom_status == True:
            orientation_q = odom.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (c_roll, c_pitch, c_yaw) = euler_from_quaternion(orientation_list)
                
            c_x, c_y = odom.pose.pose.position.x, odom.pose.pose.position.y
            distance = math.sqrt((c_x - self.s_x)**2 + (c_y - self.s_y)**2)
            rotation = np.rad2deg(c_yaw - self.s_yaw)
            self.get_logger().info(f'distance : {distance:.2f}m, rotation : {rotation:.2f}deg')
            self.get_logger().info(f"\nstart_xy : ({self.s_x}, {self.s_y})\ncurrent_xy : ({c_x}, {c_y})\ndiff_xy : ({c_x - self.s_x}, {c_y - self.s_y})")
    
    def scan_callback(self, scan):
        self.point_cloud = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # 라이더 기준 앵글
        angle = scan.angle_min
        
        # 각도별 거리를 이용하여 좌표를 구하고 시각화
        for r in scan.ranges:
            if 0.05 <= r <= 0.5:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                l_x = int(-x * self.visualization_scope + self.center[0])
                l_y = int(y * self.visualization_scope + self.center[1])
                if self.sim:
                    if 0 < angle < np.pi: # 0~180degree
                        # 파란점, 왼쪽
                        cv2.circle(self.point_cloud, (l_x, l_y), 2, (255, 0, 0), -1)
                    elif np.pi < angle < 2*np.pi:
                        # 빨간점, 오른쪽
                        cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 0, 255), -1)
                    if 0 < angle < 0.1*np.pi or 1.9*np.pi < angle < 2*np.pi:
                        # 앞쪽 부분을 초록점으로 표시
                        cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 255, 0), -1)
                else:
                    if 0 < angle < np.pi: # 0~180degree
                        # 파란점, 왼쪽
                        cv2.circle(self.point_cloud, (l_x, l_y), 2, (255, 0, 0), -1)
                    elif -np.pi < angle < 0:
                        # 빨간점, 오른쪽
                        cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 0, 255), -1)
                    if -0.1*np.pi < angle < 0.1*np.pi:
                        # 앞쪽 부분을 초록점으로 표시
                        cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 255, 0), -1)
            angle += scan.angle_increment
        
        cv2.circle(self.point_cloud, self.center, 10, (255, 255, 255), -1)
        M = cv2.getRotationMatrix2D(self.center, -90, 1.0)
        self.point_cloud = cv2.warpAffine(self.point_cloud, M, (self.width, self.height))
        cv2.imshow("LiDAR point cloud", self.point_cloud)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            twistStamped = TwistStamped()
            self.cmd_vel_publisher.publish(twistStamped)
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
    
    def start(self, msg):
        if msg.data == "play":
            self.odom_status = True
        elif msg.data == "stop":
            self.odom_status = False

def main(args=None):
    rclpy.init(args=args)
    node = OdomTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

