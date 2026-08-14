import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

import numpy as np
import cv2

class OdomTest(Node):
    def __init__(self):
        super().__init__('odom_test')       
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
        
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.img_callback, self.qos_profile)
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.img_callback, self.qos_profile)
        
        cv2.namedWindow('controls',2)
        cv2.resizeWindow('controls', 550, 150)

        self.H_low = 0
        self.H_high = 180
        self.S_low = 0
        self.S_high = 255
        self.V_low = 0
        self.V_high = 255

        cv2.createTrackbar('low H','controls',0,180,self.callback)
        cv2.createTrackbar('high H','controls',180,180,self.callback)

        cv2.createTrackbar('low S','controls',0,255,self.callback)
        cv2.createTrackbar('high S','controls',255,255,self.callback)

        cv2.createTrackbar('low V','controls',0,255,self.callback)
        cv2.createTrackbar('high V','controls',255,255,self.callback)
        
        self.cv_bridge = CvBridge()
    
    def img_callback(self, img):
        src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")
        
        dst = src.copy()
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)

        hsv_low = np.array([self.H_low,self.S_low,self.V_low], np.uint8)
        hsv_high = np.array([self.H_high,self.S_high,self.V_high], np.uint8)

        mask = cv2.inRange(hsv, hsv_low, hsv_high)

        cv2.imshow('mask',mask)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
    
    def callback(self, x):
        self.H_low = cv2.getTrackbarPos('low H','controls')
        self.H_high = cv2.getTrackbarPos('high H','controls')
        self.S_low = cv2.getTrackbarPos('low S','controls')
        self.S_high = cv2.getTrackbarPos('high S','controls')
        self.V_low = cv2.getTrackbarPos('low V','controls')
        self.V_high = cv2.getTrackbarPos('high V','controls')

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

