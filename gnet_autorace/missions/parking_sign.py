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

class ParkingSign(LifecycleNode):
    def __init__(self):
        super().__init__('parking_sign')
        self.get_logger().info('Lifecycle node created.')
        self.activate = False

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
        
        self.mission_state_publisher = self.create_lifecycle_publisher(String, '/mission_state', self.qos_profile)
        # 라인트레이싱 할 라인 방향 지정하는 토픽
        self.direction_publisher = self.create_lifecycle_publisher(String, '/direction', self.qos_profile)
        self.line_motor_publisher = self.create_lifecycle_publisher(String, '/line_motor_state', self.qos_profile)
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.parking_sign_detect, self.qos_profile)
            self.get_logger().info("sim node mode")
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.parking_sign_detect, self.qos_profile)
            self.get_logger().info("real node mode")
        
        self.traffic_sign = []
        self.traffic_img = None
        cv2.namedWindow('src')
        cv2.setMouseCallback('src', self.select_roi)
        
        self.cv_bridge = CvBridge()
        
        self.get_logger().info('Configuring Complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')
        self.activate = True
        self.mission_state_publisher.on_activate(state)
        self.direction_publisher.on_activate(state)
        self.line_motor_publisher.on_activate(state)
        
        self.get_logger().info('Activating Complete')
        return super().on_activate(state)

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating...')
        self.activate = False
        self.mission_state_publisher.on_deactivate(state)
        self.direction_publisher.on_deactivate(state)
        self.line_motor_publisher.on_deactivate(state)
        cv2.destroyAllWindows()
        
        self.get_logger().info('Deactivating Complete')
        return super().on_deactivate(state)
        
    def on_cleanup(self, state: State):
        self.get_logger().info('CleanUp...')
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.direction_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        
        self.get_logger().info('CleanUp Complete')
        return super().on_cleanup(state)
    
    def on_shutdown(self, state: State):
        self.get_logger().info('Shutdowning...')
        self.activate = False
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.direction_publisher)
        self.destroy_publisher(self.line_motor_publisher)
        
        cv2.destroyAllWindows()
        self.get_logger().info('Shutdowning Complete')
        return super().on_shutdown(state)
    
    def parking_sign_detect(self, img):
        if self.activate == True:
            # cv_birdge로 영상 변환
            src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")
            #src = cv2.GaussianBlur(src, (3, 3), 3)
            
            # templet기능을 쓰기 위해 회색조 영상 변환
            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            
            # 사전에 촬영한 이미지 불러오기
            try:
                if self.sim:
                    parking_template = cv2.imread('{colcon_ws}/gnet_autorace/gnet_autorace/missions/sim_traffic_signs/parking.jpg', 0)
                else:
                    parking_template = cv2.imread('{colcon_ws}/gnet_autorace/gnet_autorace/missions/traffic_signs/parking.jpg', 0)
                cv2.imshow("parking_sign", parking_template)
                w, h = parking_template.shape[::-1]
                box_loc = self.match_template(gray, parking_template)
                
                # 검출된 박스 시각화 후 방향 판단
                for box in zip(*box_loc[::-1]):
                    sx, sy = box
                    ex, ey = sx+ w, sy + h
                    cv2.rectangle(src, (sx, sy), (ex, ey), (0, 255, 0), 1)
                    msg = String()
                    #msg.data = 'stop'
                    #self.line_motor_publisher.publish(msg)
                    msg.data = 'left'
                    self.direction_publisher.publish(msg)
                    self.get_logger().info("parking sign check!")
                    msg.data = 'parking_sign'
                    self.activate = False
                    cv2.destroyAllWindows()
                    self.mission_state_publisher.publish(msg)
                    return
                    
            except Exception as e:
                parking_template = cv2.imread('{colcon_ws}/gnet_autorace/gnet_autorace/missions/traffic_signs/empty_img.jpg', 0)
                cv2.imshow("parking_sign", parking_template)
                #self.get_logger().info(f"{e}")
            
            try:
                self.traffic_img = src[self.traffic_sign[0][1]:self.traffic_sign[1][1], self.traffic_sign[0][0]:self.traffic_sign[1][0]].copy()
                cv2.rectangle(src, self.traffic_sign[0], self.traffic_sign[1], (0, 255, 0), 2)
                cv2.imshow('parking_sign', self.traffic_img)
            except Exception as e:
                pass
                #self.get_logger().info(f"{e}")
            
            cv2.imshow('src', src)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                self.activate = False
                self.destroy_node()
                rclpy.shutdown()
            
            # 키보드를 이용해 이미지 캡쳐
            elif key == ord('s'):
                if self.sim:
                    filename = "{colcon_ws}/gnet_autorace/gnet_autorace/missions/sim_traffic_signs/parking.jpg"
                else:
                    filename = "{colcon_ws}/gnet_autorace/gnet_autorace/missions/traffic_signs/parking.jpg"
                cv2.imwrite(filename, self.traffic_img)
                self.get_logger().info(f"parking 이미지가 저장되었습니다.")
                
    def match_template(self, gray, template):
        w, h = template.shape[::-1]
                
        # 매칭 결과 저장
        result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
        thresh = 0.8
        # 매칭 결과(인덱스에 유사도가 저장) 중 임계값이 넘는 값들만 저장
        box_loc = np.where(result >= thresh)
        
        return box_loc
                
    def select_roi(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.traffic_sign = [(x, y)]
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.traffic_sign.append((x, y))

def main(args=None):
    rclpy.init(args=args)
    node = ParkingSign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

