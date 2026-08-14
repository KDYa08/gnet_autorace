import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import numpy as np
import cv2

class TrafficLight(LifecycleNode):
    def __init__(self):
        super().__init__('traffic_light')
        self.get_logger().info('Lifecycle node created.')
        self.activate = False

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value

        # 필요한 토픽을 골라서 활성화
        # self.mission_state_publisher = self.create_lifecycle_publisher(String, '/mission_state', self.qos_profile)
        # self.line_motor_publisher = self.create_lifecycle_publisher(String, '/line_motor_state', self.qos_profile)
        # self.direction_publisher = self.create_lifecycle_publisher(String, '/direction', self.qos_profile)
        
        # csi_cam 영상 구독, csi_cam pkg에서 발행 중
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.greenlight_ready, self.qos_profile)
            self.get_logger().info("sim node mode")
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.greenlight_ready, self.qos_profile)
            self.get_logger().info("real node mode")
        
        # 초록불 hsv 범위
        if self.sim:
            self.green_lower = np.array([0, 0, 0])
            self.green_upper = np.array([180, 255, 255])
        else:
            self.green_lower = np.array([0, 0, 0])
            self.green_upper = np.array([180, 255, 255])
        
        self.cv_bridge = CvBridge()
        
        self.get_logger().info('Configuring Complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')
        self.activate = True
        self.mission_state_publisher.on_activate(state)
        self.line_motor_publisher.on_activate(state)
        
        self.get_logger().info('Activating Complete')
        return super().on_activate(state)

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating...')
        self.activate = False
        self.mission_state_publisher.on_deactivate(state)
        self.line_motor_publisher.on_deactivate(state)
        cv2.destroyAllWindows()
        
        self.get_logger().info('Deactivating Complete')
        return super().on_deactivate(state)
        
    def on_cleanup(self, state: State):
        self.get_logger().info('CleanUp...')
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        
        self.get_logger().info('CleanUp Complete')
        return super().on_cleanup(state)
    
    def on_shutdown(self, state: State):
        self.get_logger().info('Shutdowning...')
        self.activate = False
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        
        cv2.destroyAllWindows()
        self.get_logger().info('Shutdowning Complete')
        return super().on_shutdown(state)
    
    def greenlight_ready(self, img):
        if self.activate == True:
            src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")
            ###################################################
            # 신호등을 감지 할 수 있는 코드 만들기                 # 
            # tip : 실제 대회장에서 초록색 영역은 매우 작게 잡히고   #
            # 노이즈가 많으므로 대처할 수 있는 방법 생각해보기       #
            ###################################################
            
            if "미션 완료 조건 충속 시":
                # 코드 반복 종료
                self.activate = False
                msg = String()
                # 미션 완료 토픽 발행
                msg.data = "traffic_light"
                self.get_logger().info("green light detect!")
                cv2.destroyAllWindows()
                self.mission_state_publisher.publish(msg)

            cv2.imshow('src', src)

            # q누르면 노드 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                self.activate = False
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
