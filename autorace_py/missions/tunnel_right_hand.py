import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import math

# 라이더로 감지한 벽을 일차함수로 변환하여 평행 이동하여 특정 거리 앞의 교점으로 이동
# 라이더로 오른벽만 따라가는 방법

class Tunnel(LifecycleNode):
    def __init__(self):
        super().__init__('tunnel')
        self.get_logger().info('Lifecycle node created.')
        self.activate = False

    def on_configure(self, state: State):
        self.get_logger().info('Configuring...')
        self.qos_profile = QoSProfile(depth=10)
        self.declare_parameter('sim_bool', False)
        self.sim = self.get_parameter('sim_bool').get_parameter_value().bool_value
        
        self.mission_state_publisher = self.create_lifecycle_publisher(String, '/mission_state', self.qos_profile)
        # tunnel 모터 상태 활성화/비활성화 topic
        self.motor_subscriber = self.create_subscription(String, '/tunnel_motor_state', self.tunnel_motor_state_callback, self.qos_profile)
        # consturciton 탈출 후 따라갈 라인 지정
        self.direction_publisher = self.create_lifecycle_publisher(String, '/direction', self.qos_profile)
        # 라인 트레이싱 모터 활성화/비활성화 topic, tunnel 모터 명령과 라인트레이싱 모터 명령이 겹치지 않도록 하기 위함
        self.line_motor_publisher = self.create_lifecycle_publisher(String, '/line_motor_state', self.qos_profile)
        # 미션 진행도를 파악하기 위해 odom topic 구독
        self.odom_sbuscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, self.qos_profile)
        self.Lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, self.qos_profile)
        if self.sim:
            self.CompressedImage = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.img_callback, self.qos_profile)
            self.get_logger().info("sim node mode")
        else:
            self.CompressedImage = self.create_subscription(CompressedImage, '/csi_camera1/compressed', self.img_callback, self.qos_profile)
            self.get_logger().info("real node mode")
        self.cmd_vel_publisher = self.create_lifecycle_publisher(TwistStamped, '/cmd_vel', self.qos_profile)
        
        # ====== 파라미터 ======
        # cloudpoint 시각화 배율
        self.visualization_scope = 200
        if self.sim:
            # 선속도
            self.v = 0.05
            # lookahead 거리, 값이 크면 부드럽게, 작으면 빠르게 움직임
            self.lookahead = 0.1
            # 벽과 유지할 안전거리(m)
            self.d_safe = 0.15
        else:
            self.v = 0.1
            self.lookahead = 0.1
            self.d_safe = 0.18
        self.tunnel_motor_state = 'play'
        self.follow_mode = False
        
        self.s_x, self.s_y = 0, 0
        
        # opencv로 시각화를 위해 창 크기 설정
        self.width, self.height = 550, 550
        self.point_cloud = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.center = (self.width // 2, self.height // 2)
        
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
        twistStamped = TwistStamped()
        self.cmd_vel_publisher.publish(twistStamped)
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
        twistStamped = TwistStamped()
        self.cmd_vel_publisher.publish(twistStamped)
        
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

            cv2.imshow('tunnel_cam', src)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                twistStamped = TwistStamped()
                self.cmd_vel_publisher.publish(twistStamped)
                cv2.destroyAllWindows()
                self.destroy_node()
                rclpy.shutdown()
    
    def odom_callback(self, odom):
        if self.activate == True:
            # tunnel 미션 시작 전까지 시작 위치 업데이트
            if self.follow_mode == False:
                self.s_x, self.s_y = odom.pose.pose.position.x, odom.pose.pose.position.y
            elif self.follow_mode == True:
                # tunnel 미션 시작 후 시작 위치와 현재 위치의 변위를 게산
                c_x, c_y = odom.pose.pose.position.x, odom.pose.pose.position.y
                distance = math.sqrt((c_x - self.s_x)**2 + (c_y - self.s_y)**2)
                self.get_logger().info(f'distance : {distance:.2f}m')
                
                # 변위가 2.37m일 때(미션 완료)
                if distance >= 2.4:
                    # tunnel 모터 비활성화
                    self.tunnel_motor_state = 'stop'
                    self.activate = False
                    cv2.destroyAllWindows()
                    msg = String()
                    # 라인트레이싱 방향 지정
                    msg.data = 'right'
                    self.direction_publisher.publish(msg)
                    # 라인트레이싱 모터 활성화                    
                    msg.data = 'play'
                    self.line_motor_publisher.publish(msg)
                    # 미션 완료 토픽 발행
                    msg.data = 'tunnel'
                    self.mission_state_publisher.publish(msg)

    def scan_callback(self, scan):
        if self.activate == True:        
            self.point_cloud = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            # 라이더 기준 앵글
            angle = scan.angle_min
            points = []
            
            # 각도별 거리를 이용하여 좌표를 구하고 시각화
            for r in scan.ranges:
                if 0.05 <= r <= 0.5:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    
                    # cloudpoint 화면에 맞게 비율과 좌표 변경
                    l_x = int(-x * self.visualization_scope + self.center[0])
                    l_y = int(y * self.visualization_scope + self.center[1])
                    
                    # tunnel 미션 진행 중일 때
                    if self.follow_mode == True:
                        if np.pi < angle < 2*np.pi:
                            # 오른쪽 부분을 빨간점으로 표시
                            points.append((x,y,r))
                            cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 0, 255), -1)
                        else:
                            cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 0, 150), -1)
                    # tunnel 미션을 시작하지 않았을 때
                    else:
                        if np.pi*5/4 < angle < np.pi*7/4:
                            # 오른쪽 부분을 초록점으로 표시
                            points.append((x,y,r))
                            cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 255, 0), -1)
                        else:
                            cv2.circle(self.point_cloud, (l_x, l_y), 2, (0, 150, 0), -1)
                angle += scan.angle_increment
            
            try:
                # 좌표를 numpy array로 저장
                points = np.array(points)
                # 감지된 점들중 최솟값을 저장
                right_distance = min(points[:, 2])
                #self.get_logger().info(f"{right_distance}")
                
                # 최솟값이 0.3m이하이고 미션 시작 전이면 미션 시작
                if self.follow_mode == False:
                    if right_distance <= 0.3:
                        self.follow_mode = True
                    else:
                        twistStamped = TwistStamped()
                        twistStamped.linear.x = self.v
                        self.cmd_vel_publisher.publish(twistStamped)
            except:
                pass
            
            # tunnel 미션 진행 중일 때
            if self.follow_mode == True:
                # 안정성을 위해 감지된 점이 5개 미만이면 함수 조기 종료
                if len(points) < 5:
                    return
                # 감지된 좌표들을 그룹화하여 x, y좌표를 분리하여 저장
                x, y = self.object_grouping(points)
                # 좌표들을 최소자승법을 이용해 직선을 구하하고 안전거리 만큼 평행이동하여 벽과 나란히 이동하는 경로 생성
                a, b_line, c_offset = self.find_object_line(x, y)
                # 경로 위에 로봇이 목표 지점으로 정할 좌표 계산
                x_L, y_L = self.find_goal_coordinate(a, b_line, c_offset)
                # 목표 좌표까지 이동하기 위해 필요한 회전값 게산  
                omega = self.tranform_coordinate_to_vector(x_L, y_L)
                
                twistStamped = TwistStamped()
                # tunnel 모터가 활성화 상태일 때
                if self.tunnel_motor_state == 'play':
                    twistStamped.linear.x = self.v
                    twistStamped.angular.z = omega
                    self.cmd_vel_publisher.publish(twistStamped)
                # tunnel 모터가 비활성화 상태일 때
                elif self.tunnel_motor_state == 'stop':
                    self.cmd_vel_publisher.publish(twistStamped)
            
            # 중심에 흰색점으로 로봇 표시
            cv2.circle(self.point_cloud, self.center, 10, (255, 255, 255), -1)
            # 실제 로봇 방향에 맞춰 창 회전
            M = cv2.getRotationMatrix2D(self.center, -90, 1.0)
            self.point_cloud = cv2.warpAffine(self.point_cloud, M, (self.width, self.height))
            cv2.imshow("LiDAR point cloud", self.point_cloud)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                twistStamped = TwistStamped()
                self.cmd_vel_publisher.publish(twistStamped)
                cv2.destroyAllWindows()
                self.activate = False
                self.destroy_node()
                rclpy.shutdown()
    
    def object_grouping(self, points):
        # ====== 직선 피팅 (y = mx + b 형태) ======
        X = []
        Y = []
        
        # ====== min distance object grouping ======
        # 감지된 점들의 거리 저장
        R = points[:, 2]
        # 거리중 가장 작은 값을 최솟 값으로 지정
        min_distance = min(R)
        for i, point in enumerate(points):
            # 좌표와 거리로 분해
            x, y, d = point
            # 최소 거리값과 감지된 점의 거리 차이가 0.1m이하이면 그룹화
            if d - min_distance <= 0.1:
                X.append(x)
                Y.append(y)
                cv2.circle(self.point_cloud, (int(-x * self.visualization_scope + self.center[0]), int(y * self.visualization_scope + self.center[1])), 2, (0, 255, 0), -1)
        return X, Y
    
    def find_object_line(self, X, Y):
        '''
        y1 = mx1 + b
        y2 = mx2 + b
        y3 = mx3 + b
                 위 식들을 행렬로 나타냈을 때
        A = [xi, 1]
        A^T @ [m, b] = xi*m + 1*b = Y
        
        .T는 전치 명령임
        '''
        A = np.vstack([X, np.ones(len(X))]).T
        
        '''
        A @ β = Y
        β = [m, b]
                  이때 β를 구하여 m, b를 구함 
        '''
        m, b = np.linalg.lstsq(A, Y, rcond=None)[0]

        # 직선: y = mx + b
        # ax + by + c = 0 형태로 변환
        # mx - y + b = 0
        # a는 x계수
        # b는 y계수
        # c는 상수 계수
        a = m
        b_line = -1
        c = b
        
        '''
                 법선 벡터의 길이를 구하여 norm에 저장
        ax + by + c = 0 에서 법선 벡터는 (a,b)임
        (ax1 + by1 + c) - (ax2 + by2 + c) = 0
        a(x1 - x2) + b(y1 - y2) = 0
                 이를 벡터로 표현하면
        (a, b)⋅(x1 - x2, y1 - y2) = 0
        (x1 - x2, y1 - y2)는 직선 벡터이고 내적이 0이므로 (a, b)는 법선 벡터임
        (a^2 + b^2)^1/2는 법선 벡터의 길이임
        '''
        norm = math.sqrt(a*a + b_line*b_line)
        
        # 직선의 방정식을 법선의 벡터의 길이에 안전 거리를 곱하여 평행이동
        c_offset = c + self.d_safe * norm
        
        return a, b_line, c_offset
    
    def find_goal_coordinate(self, a, b_line, c_offset):
        # ====== Lookahead Point 계산 ======
        # 로봇은 x좌표계를 앞으로 인식하고 lookahead로 얼마나 멀리 볼지 설정함
        x_L = self.lookahead

        # ax + by + c_offset = 0
        # a*x_L + b_line*y_L + c_offset = 0
        # y_L = -(a*x_L + c_offset)/b_line
        
        # x좌표의 0.2(lookahead)에서 직선의 방정식의 교점인 y좌표를 구함
        y_L = -(a * x_L + c_offset) / b_line
        
        cv2.arrowedLine(self.point_cloud, self.center, (int(-x_L * self.visualization_scope + self.center[0]), int(y_L * self.visualization_scope + self.center[1])), (255, 255, 255), 2)
        
        return x_L, y_L 
    
    def tranform_coordinate_to_vector(self, x_L, y_L):
        # ====== Pure Pursuit 제어 ======
            # arc tan를 이용하여 lookahead 벡터의 각도를 구함
        alpha = math.atan2(y_L, x_L)
        
        # 원의 반지름을  R이라고 하면 곡률은 κ = 1/R
        # 원의 중심은 (0, R), lookahead점이 원의 방정식 위에 있음을 식으로 나타내면
        #     x_L^2 + (y_L - R)^2 = R^2
        #     x_L^2 + y_L^2 = 2 y_L R
        # L_d^2 = x_L^2 + y_L^2 이므로
        #     L_d^2 = 2 y_L R
        #     R = L_d^2 / (2 y_L)
        # 곡률 κ = 1/R 이므로
        #     κ = 2 y_L / L_d^2
        # sin(alpha) = y_L / L_d 이므로
        #     κ = 2 sin(alpha) / L_d
        # 평면 운동에서 각속도 ω = v κ 이므로
        #     ω = v * (2 sin(alpha) / L_d)
        omega = 2.0 * self.v * math.sin(alpha) / self.lookahead

        # 안정성 제한
        omega = max(min(omega, 1.5), -1.5)
        
        return omega
                
    def tunnel_motor_state_callback(self, msg):
        self.tunnel_motor_state = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = Tunnel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

