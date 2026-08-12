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
import time 

class BlockBar(LifecycleNode):
    def __init__(self):
        super().__init__('blockbar')
        self.get_logger().info('Lifecycle node created.')
        self.activate = False

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
        
        self.mission_state_publisher = self.create_lifecycle_publisher(String, '/mission_state', self.qos_profile)
        self.line_motor_publisher = self.create_lifecycle_publisher(String, '/line_motor_state', self.qos_profile)
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.blockbar_detect, self.qos_profile)
            self.get_logger().info("sim node mode")
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.blockbar_detect, self.qos_profile)
            self.get_logger().info("real node mode")
        
        if self.sim:
            self.red_lower = np.array([0, 179, 164])
            self.red_upper = np.array([25, 255, 255])
        else:
            self.red_lower = np.array([0, 188, 163])
            self.red_upper = np.array([15, 255, 223])
            
        self.traffic_sign = []
        self.traffic_img = None
        cv2.namedWindow('src')
        cv2.setMouseCallback('src', self.select_roi)
                
        self.cv_bridge = CvBridge()
        
        self.block_detect = False
        
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
    
    def blockbar_detect(self, img):
        if self.activate == True:
            src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")
            hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
            
            mask_red = cv2.inRange(hsv, self.red_lower, self.red_upper)
            h, w = mask_red.shape[:2]
            
            if self.sim:
                mask_red[0:60, 0:w] = 0
                mask_red[122:h, 0:w] = 0
            else:
                mask_red[0:72, 0:w] = 0
                mask_red[133:h, 0:w] = 0
            
            cv2.rectangle(src, (0, 66), (w, 122), (0, 255, 0), 2)
            
            # 차단바 중심의 빨간색과 중심가 가장 가까운 빨간색 중심의 거리를 이용해서 정지 거리 조정
            # 외곽선 검출
            contours = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            
            h, w = mask_red.shape[:2]
            
            # 외과선의 중점 좌표와 좌표간의 차이 저장
            cx = []; cy = []; x_diff = []
            for i in contours:
                
                M = cv2.moments(i)
                # moment 영역이 10 이상일 때 중점 좌표 기록 후 시각화
                if int(M['m00']) >= 10:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])
                    
                    cx.append(cX)
                    cy.append(cY)
                    cv2.circle(src, (cX, cY), 3, (0, 0, 255), -1)
            
            try:
                cx.sort()
                
                # 화면 중심과 중심점의 거리를 저장
                for i in range(len(cx) - 1):
                    x_diff.append(abs(w//2 - cx[i]))
                x_diff_sort = sorted(x_diff)
                # 거리가 가장 짧은 점과 두 번 째로 짧은 점의 좌표 저장
                pos1, pos2 = cx[x_diff.index(x_diff_sort[0])], cx[x_diff.index(x_diff_sort[1])]
                
                # 시각화
                cv2.circle(src, (pos1, int(sum(cy)/len(cy))), 3, (0, 255, 0), -1)
                cv2.circle(src, (pos2, int(sum(cy)/len(cy))), 3, (0, 255, 0), -1)
                
                self.get_logger().info(f"{pos1}, {pos2}, diff:{abs(pos1 - pos2)}")
                
	            # 중심점들의 거리차가 40px 이상이면
                if abs(pos1 - pos2) >= 40:
                    self.get_logger().info("blockbar detect")
                    msg = String()
                    # 라인트레이싱 모터 비활성화
                    msg.data = 'stop'
                    self.line_motor_publisher.publish(msg)
                    self.block_detect = True
                else:
                    if self.block_detect and abs(pos1 - pose) <= 30:
                        self.get_logger().info("Let's go")
                        self.activate = False
                        msg = String()
                        # 라인트레이싱 모터 활성화
                        msg.data = 'play'
                        self.line_motor_publisher.publish(msg)
                        # 미션 완료 신호 발행
                        msg.data = "blockbar"
                        cv2.destroyAllWindows()
                        self.mission_state_publisher.publish(msg)
            
            # 중심점을 찾을 수 없을 때(차단바가 올라 갔을 때)
            except Exception as e:
                #self.get_logger().info(f'{e}')
                
                # 차단바가 이전에 내려왔는지 확인
                if self.block_detect:
                    self.get_logger().info("Let's go")
                    self.activate = False
                    msg = String()
                    # 라인트레이싱 모터 활성화
                    msg.data = 'play'
                    self.line_motor_publisher.publish(msg)
                    # 미션 완료 신호 발행
                    msg.data = "blockbar"
                    cv2.destroyAllWindows()
                    self.mission_state_publisher.publish(msg)
                pass
            
            # 화면에 감지되는 빨간색 픽셀 수로 정지 거리 조정(대회 전용)
            '''
            red_pixels = cv2.countNonZero(mask_red)

            if red_pixels > 10000:                 # 기준 픽셀 수
                self.get_logger().info("🚧 차단바 감지 → STOP")
                msg = String()
                msg.data = "stop"
                self.get_logger().info("blockbar detect!")
                self.line_motor_publisher.publish(msg)
                if red_pixels < 10000:
                    self.activate = False
                    msg.data = "blockbar"
                    self.get_logger().info("GO!")
                    cv2.destroyAllWindows()
                    self.mission_state_publisher.publish(msg)
                    msg.data = "play"
                    self.line_motor_publisher.publish(msg)
            '''
            try:
                cv2.rectangle(src, self.traffic_sign[0], self.traffic_sign[1], (0, 255, 0), 2)
            except:
                pass
            cv2.imshow("red_detect", mask_red)
            cv2.imshow('src', src)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                self.activate = False
                self.destroy_node()
                rclpy.shutdown()

    def select_roi(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.traffic_sign = [(x, y)]
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.traffic_sign.append((x, y))
            self.get_logger().info(f"{self.traffic_sign}")

def main(args=None):
    rclpy.init(args=args)
    node = BlockBar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
