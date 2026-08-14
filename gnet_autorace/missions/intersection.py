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

class Intersection(LifecycleNode):
    def __init__(self):
        super().__init__('intersection')
        self.get_logger().info('Lifecycle node created.')
        self.mission_state_publisher = None
        self.activate = False

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
                
        self.line_motor_publisher = self.create_lifecycle_publisher(String, '/line_motor_state', self.qos_profile)                
        # 미션 완료 시 발행할 토픽, core프로그램에서 구독 중
        self.mission_state_publisher = self.create_lifecycle_publisher(String, '/mission_state', self.qos_profile)
        # 라인트레이싱 할 라인 방향 지정하는 토픽
        self.direction_publisher = self.create_lifecycle_publisher(String, '/direction', self.qos_profile)
        # csi_cam 영상 구독, csi_cam pkg에서 발행 중
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.direction_detect, self.qos_profile)
            self.get_logger().info("sim node mode")
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.direction_detect, self.qos_profile)
            self.get_logger().info("real node mode")
        
        # 표지판 이미지의 시작 좌표와 끝좌표를 저장할 리스트
        self.traffic_sign = []
        # 표지판 이미지를 저장할 변수
        self.traffic_img = None
        # 마우스 이벤트 처리를 위한 콜백 지정
        cv2.namedWindow('src')
        cv2.setMouseCallback('src', self.select_roi)
        
        self.cv_bridge = CvBridge()
        
        self.get_logger().info('Configuring Complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')
        self.activate = True
        self.line_motor_publisher.on_activate(state)
        self.mission_state_publisher.on_activate(state)
        self.direction_publisher.on_activate(state)
        
        self.get_logger().info('Activating Complete')
        return super().on_activate(state)

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating...')
        self.activate = False
        self.line_motor_publisher.on_deactivate(state)
        self.mission_state_publisher.on_deactivate(state)
        self.direction_publisher.on_deactivate(state)
        cv2.destroyAllWindows()
        
        self.get_logger().info('Deactivating Complete')
        return super().on_deactivate(state)
        
    def on_cleanup(self, state: State):
        self.get_logger().info('CleanUp...')
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.direction_publisher)
        
        self.get_logger().info('CleanUp Complete')
        return super().on_cleanup(state)
    
    def on_shutdown(self, state: State):
        self.get_logger().info('Shutdowning...')
        self.activate = False
        
        self.destroy_publisher(self.mission_state_publisher)
        self.destroy_publisher(self.direction_publisher)
        
        cv2.destroyAllWindows()
        self.get_logger().info('Shutdowning Complete')
        return super().on_shutdown(state)
    
    def direction_detect(self, img):
        if self.activate == True:
            # cv_birdge로 영상 변환
            src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")
            #src = cv2.GaussianBlur(src, (3, 3), 3)
            
            # templet기능을 쓰기 위해 회색조 영상 변환
            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            
            try:
                # 사전에 촬영한 표지판 이미지 불러오기
                if self.sim:
                    L_template = cv2.imread('/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/sim_traffic_signs/intersection_left.jpg', 0)
                else:
                    L_template = cv2.imread('/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/traffic_signs/intersection_left.jpg', 0)
                cv2.imshow("left_sign", L_template)
                # 매칭 결과 중 임계값이 넘는 값들만 저장
                w, h = L_template.shape[::-1]
                box_loc = self.match_template(gray, L_template)
                
                # 검출된 박스 시각화 후 방향 판단
                for box in zip(*box_loc[::-1]):
                    # 시작 좌표
                    sx, sy = box
                    # 끝 좌표
                    ex, ey = sx+ w, sy + h
                    # 표지판 감지 영역 시각화
                    cv2.rectangle(src, (sx, sy), (ex, ey), (0, 255, 0), 1)
                    # 왼쪽 표지판을 감지 했기에 왼쪽 라인 인식 토픽 발행
                    msg = String()
                    #msg.data = 'stop'
                    #self.line_motor_publisher.publish(msg)
                    msg.data = 'left'
                    self.direction_publisher.publish(msg)
                    self.get_logger().info("left check!")
                    # 미션 완료 토픽 발행
                    msg.data = 'intersection'
                    self.activate = False
                    cv2.destroyAllWindows()
                    self.mission_state_publisher.publish(msg)
                    return
            
            # telmplete 이미지가 존재하지 않을 때 예외 처리
            except Exception as e:
                L_template = cv2.imread('/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/traffic_signs/empty_img.jpg', 0)
                cv2.imshow("left_sign", L_template)
                #self.get_logger().info(f"{e}")
            
            try:   
                if self.sim:
                    R_template = cv2.imread('/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/sim_traffic_signs/intersection_right.jpg', 0)
                else: 
                    R_template = cv2.imread('/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/traffic_signs/intersection_right.jpg', 0)
                cv2.imshow("right_sign", R_template)
                w, h = R_template.shape[::-1]
                box_loc = self.match_template(gray, R_template)
                
                for box in zip(*box_loc[::-1]):
                    sx, sy = box
                    ex, ey = sx+ w, sy + h
                    cv2.rectangle(src, (sx, sy), (ex, ey), (0, 0, 255), 1)
                    msg = String()
                    #msg.data = 'stop'
                    #self.line_motor_publisher.publish(msg)
                    msg.data = 'right'
                    self.direction_publisher.publish(msg)
                    self.get_logger().info("right check!")
                    msg.data = 'intersection'
                    self.activate = False
                    cv2.destroyAllWindows()
                    self.mission_state_publisher.publish(msg)
                    return
            
            except Exception as e:
                R_template = cv2.imread('/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/traffic_signs/empty_img.jpg', 0)
                cv2.imshow("right_sign", R_template)
                self.get_logger().info(f"{e}")
            
            try:
	            # 표지판 좌표로 표지판 roi지정
                self.traffic_img = src[self.traffic_sign[0][1]:self.traffic_sign[1][1], self.traffic_sign[0][0]:self.traffic_sign[1][0]].copy()
                # 표지판 roi 시각화
                cv2.rectangle(src, self.traffic_sign[0], self.traffic_sign[1], (0, 255, 0), 2)
                cv2.imshow('traffic_sign', self.traffic_img)
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
            elif key == ord('a'):
                if self.sim:
                    filename = "/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/sim_traffic_signs/intersection_left.jpg"
                else:
                    filename = "/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/traffic_signs/intersection_left.jpg"
                cv2.imwrite(filename, self.traffic_img)
                self.get_logger().info("left 이미지가 저장되었습니다.")
            
            elif key == ord('d'):
                if self.sim:
                    filename = "/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/sim_traffic_signs/intersection_right.jpg"
                else:
                    filename = "/home/kdya08/turtlebot3_ws/src/autorace_py/autorace_py/missions/traffic_signs/intersection_right.jpg"
                cv2.imwrite(filename, self.traffic_img)
                self.get_logger().info("right 이미지가 저장되었습니다.")
    
    def match_template(self, gray, template):
        w, h = template.shape[::-1]
                
        # 매칭 결과 저장
        result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
        thresh = 0.8
        # 매칭 결과(인덱스에 유사도가 저장) 중 임계값이 넘는 값들만 저장
        box_loc = np.where(result >= thresh)
        
        return box_loc

    def select_roi(self, event, x, y, flags, param):
	    # 좌클릭 시 표지판 이미지 시작 좌표 저장
        if event == cv2.EVENT_LBUTTONDOWN:
            self.traffic_sign = [(x, y)]
        
        # 드래그 후에 표지판 이미지 끝 좌표 저장
        elif event == cv2.EVENT_LBUTTONUP:
            self.traffic_sign.append((x, y))

def main(args=None):
    rclpy.init(args=args)
    node = Intersection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
