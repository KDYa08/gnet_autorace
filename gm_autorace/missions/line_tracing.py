import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped

import numpy as np
import cv2

class Line_Tracing(LifecycleNode):
    def __init__(self):
        super().__init__('line_tracing')
        self.get_logger().info('Lifecycle node created.')
        self.activate = False

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
        
        self.line_motor_subscriber = self.create_subscription(String, '/line_motor_state', self.line_motor_state, self.qos_profile)
        # 라인트레이싱 할 라인 방향 지정하는 토픽
        self.direction_subscriber = self.create_subscription(String, '/direction', self.direction_decide, self.qos_profile)
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.line_tracing_callback, self.qos_profile)
            self.get_logger().info("sim node mode")
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.line_tracing_callback, self.qos_profile)
            self.get_logger().info("real node mode")
        self.cmd_vel_publisher = self.create_lifecycle_publisher(TwistStamped, '/cmd_vel', self.qos_profile)       
        self.cv_bridge = CvBridge()
        
        # stop : 라인트레이싱 모터 비활성화
        # play : 라인트레이싱 모터 활성화
        self.line_motor_state = 'stop'
        # 라인트레이싱 라인 방향 지정
        self.direction = "left"
        
        self.get_logger().info('Configuring Complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating...')
        self.activate = True
        self.cmd_vel_publisher.on_activate(state)
        
        self.get_logger().info('Activating Complete')
        return super().on_cleanup(state)

    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating...')
        self.activate = False
        twistStamped = TwistStamped()
        self.cmd_vel_publisher.publish(twistStamped)
        self.cmd_vel_publisher.on_deactivate(state)
        cv2.destroyAllWindows()
        
        self.get_logger().info('Deactivating Complete')
        return super().on_cleanup(state)
        
    def on_cleanup(self, state: State):
        self.get_logger().info('CleanUp...')
        
        self.destroy_publisher(self.cmd_vel_publisher)
        
        self.get_logger().info('CleanUp Complete')
        return super().on_cleanup(state)
    
    def on_shutdown(self, state: State):
        self.get_logger().info('Shutdowning...')
        
        self.activate = False
        twistStamped = TwistStamped()
        self.cmd_vel_publisher.publish(twistStamped)
        
        self.destroy_publisher(self.cmd_vel_publisher)
        cv2.destroyAllWindows()
        self.get_logger().info('Shutdowning Complete')
        return super().on_cleanup(state)
    
    def line_tracing_callback(self, img):
        if self.activate == True:
            # cvbridge 변환
            src = self.cv_bridge.compressed_imgmsg_to_cv2(img, "bgr8")
            dst = cv2.GaussianBlur(src, (0, 0), 2)
            
            gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape[:2]
            
            # 라인트레이싱 할 영역 높이 설정
            roi_h_L = h*8//9
            roi_h_R = h*8//9
            
            # x, y 외곽선 검출후 영상 병합
            sobelx = cv2.Sobel(gray, cv2.CV_8U, 2, 0, 5)
            sobely = cv2.Sobel(gray, cv2.CV_8U, 0, 2, 5)
            sobelxy = cv2.bitwise_or(sobelx, sobely)
            
            # 라인트레이싱 방향에 따라 좌우 영역 슬라이싱(topic으로 설정 가능)
            if self.direction == 'left':
                # 라인트레이싱 영역 슬라이싱
                sobelxy[0:roi_h_L-10, 0:w] = 0
                sobelxy[roi_h_L+10:h, 0:w] = 0
                sobelxy[0:h, w//2:w] = 0
            elif self.direction == 'right':
                # 라인트레이싱 영역 슬라이싱
                sobelxy[0:roi_h_R-10, 0:w] = 0
                sobelxy[roi_h_R+10:h, 0:w] = 0
                sobelxy[0:h, 0:w//2] = 0
            
            # morpholozy 연산 수행 후 이진화
            kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
            dilate = cv2.dilate(sobelxy, kernel, anchor=(-1, -1), iterations=4)
            erode = cv2.erode(dilate, kernel, anchor=(-1, -1), iterations=3)
            _, thresh = cv2.threshold(erode, 20, 255, cv2.THRESH_BINARY)
            
            # 외곽선 검출
            contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            
            # 외과선의 중점 좌표와 좌표간의 차이 저장
            cx = []
            for i in contours:
                
                M = cv2.moments(i)
                # moment 영역이 10 이상일 때 중점 좌표 기록 후 시각화
                if int(M['m00']) >= 10:
                    cX = int(M['m10'] / M['m00'])
                    
                    cx.append(cX)
                    if self.direction == 'left':
                        cv2.circle(src, (cX, roi_h_L), 3, (0, 0, 255), -1)
                    elif self.direction == 'right':
                        cv2.circle(src, (cX, roi_h_R), 3, (0, 0, 255), -1)
                    
                    '''
                # bounding rect로 라인의 사각형 검출 후 중점 기록(moment보다 느려서 사용 x)
                i_x, i_y, i_w, i_h = cv2.boundingRect(i)
                if cv2.contourArea(i) >= 500:
                    cX = i_x + i_w//2
                    cx.append(cX)
                    cv2.circle(src, (cX, h*3//4), 3, (0, 0, 255), -1)
                '''
            try:
                # 저장된 중점 좌표들의 평균값을 중점으로 사용(영상에 노이즈가 발생할 시, 라인트레이싱 이탈 확률이 높아 다른 방법 사용)
                x = int(sum(cx)/len(cx))

                if self.direction == 'left':
                    cv2.circle(src, (x, roi_h_L), 3, (0, 255, 0), -1)
                elif self.direction == 'right':
                    cv2.circle(src, (x, roi_h_R), 3, (0, 255, 0), -1)
                
                # 슬라이싱 영역을 기준으로 중심 위치가 라인트레이싱 영역에 맞게 중점 설정
                if self.direction == "left":
                    pos = x - w//4 + 0
                elif self.direction == "right":
                    pos = x - w*3//4 - 0
                
                # 라인 위치 최대 최솟값 설정
                pos = max(min(pos, 60), -60)
                
                # 선형 보간 공식 사용(pos value transform zpos)
                # output = (x - xmin)/(xmax - xmin) * (ymax - ymin) + ymin
                z_pos = (pos + 60)/(60 + 60) * (1.4 + 1.4) - 1.4
                
                # 보정값
                if self.sim:
                    z_pos = z_pos - 0.48 if self.direction == "right" else z_pos + 0.48
                else:
                    z_pos = z_pos if self.direction == "right" else z_pos
                
                # |z_pos|가 0.02면 직진으로 판단
                if -0.02 <= z_pos <= 0.02:
                    z_pos = 0
                
                # 회전 속도에 반비례하여 직진 속도 조절 최소 직진 속도 설정
                if self.sim:
                    x_linear = max(0.08 - abs(z_pos/9), 0.02)
                else:
                    x_linear = max(0.18 - abs(z_pos/7), 0.02)
                
                if self.line_motor_state == "play" or self.line_motor_state == "stop":
                    self.get_logger().info(f'z:{z_pos:.2f}, x:{x_linear:.2f}, pos:{pos}')
    
                
                twistStamped = TwistStamped()
                # motor_state가 play일 때만 움직임(topic으로 설정 가능)
                # 상태에 따라 라인트레이싱 화면에 상태 표시
                if self.line_motor_state == "play":
                    twistStamped.linear.x = float(x_linear)
                    twistStamped.angular.z = float(-z_pos)
                    self.cmd_vel_publisher.publish(twistStamped)
                    cv2.putText(src, self.line_motor_state, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                elif self.line_motor_state == "stop":
                    self.cmd_vel_publisher.publish(twistStamped)
                    cv2.putText(src, self.line_motor_state, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                elif self.line_motor_state == 'construction' or self.line_motor_state == 'tunnel' or self.line_motor_state == 'parking':
                    cv2.putText(src, self.line_motor_state, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2, cv2.LINE_AA)

            except Exception as e:
                self.get_logger().info(f'{e}')
            
            cv2.imshow("thresh", thresh)
            cv2.imshow("src", src)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                twistStamped = TwistStamped()
                self.cmd_vel_publisher.publish(twistStamped)
                cv2.destroyAllWindows()
                self.activate = False
                self.destroy_node()
                rclpy.shutdown()
            
    def line_motor_state(self, data):
        self.line_motor_state = data.data
    
    def direction_decide(self, data):
        self.direction = data.data

def main(args=None):
    rclpy.init(args=args)
    node = Line_Tracing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

