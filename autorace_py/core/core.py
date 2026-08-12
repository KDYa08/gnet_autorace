import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class MissionsCore(Node):
    def __init__(self):
        super().__init__('missions_core')

        qos_profile = QoSProfile(depth=10)

        self.create_subscription(String, '/mission_state', self.mission_done, qos_profile)
        self.create_subscription(String, '/mission_change', self.mission_change, qos_profile)
        
        # on sign
        '''
        self.missions = [
        'line_tracing', 
        'traffic_light', 
        'intersection', 
        'construction_sign', 
        'construction', 
        'parking_sign', 
        'parking', 
        'blockbar', 
        'tunnel_sign', 
        'tunnel']
        '''
        
        # no sign
        
        self.missions = [
        'line_tracing', 
        'traffic_light', 
        'intersection',  
        'construction', 
        'parking', 
        'blockbar',  
        'tunnel']
        
        # rtree test
        self.missions = [
        'line_tracing', 
        'traffic_light', 
        'intersection',
        'construction_sign',  
        'construction',
        'parking_sign', 
        'parking']
        
        # skip
        '''
        self.missions = [
        'line_tracing', 
        'construction', 
        'parking_sign', 
        'parking', 
        'blockbar',  
        'tunnel']
        '''
        
        self.mission_clients = {name: self.create_client(ChangeState, f'/{name}/change_state') for name in self.missions}

        self.current_index = 0

        self.get_logger().info('🚀 Missions Core Ready')
        self.wait_for_services()
        
        self.start_current_mission()
        self.current_index += 1
        self.start_current_mission()

    def wait_for_services(self):
        for name, client in self.mission_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} lifecycle service...')

    def change_state(self, client, transition_id):
        req = ChangeState.Request()
        req.transition.id = int(transition_id)
        future = client.call_async(req)
        future.add_done_callback(self.lifecycle_response_callback)

    def lifecycle_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Lifecycle transition success")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def start_current_mission(self):
        mission = self.missions[self.current_index]
        self.get_logger().info(f'▶ Starting {mission}')

        client = self.mission_clients[mission]
        self.change_state(client, Transition.TRANSITION_CONFIGURE)
        self.change_state(client, Transition.TRANSITION_ACTIVATE)        
        self.get_logger().info("check")

    def stop_current_mission(self):
        mission = self.missions[self.current_index]
        self.get_logger().info(f'⏹ Stopping {mission}')

        client = self.mission_clients[mission]
        self.change_state(client, Transition.TRANSITION_DEACTIVATE)
        self.change_state(client, Transition.TRANSITION_CLEANUP)

    def mission_done(self, msg: String):
        self.get_logger().info(f'✅ {msg.data} mission done')

        if msg.data != self.missions[self.current_index]:
            self.get_logger().warn(f'⚠ Mission order mismatch(sub:{msg.data}, matach:{self.missions[self.current_index]}')
            return
        self.stop_current_mission()
        self.current_index += 1

        if self.current_index < len(self.missions):
            self.start_current_mission()
        else:
            self.get_logger().info('🎉 All missions completed')

    def mission_change(self, msg: String):
        self.get_logger().info(f'✅ {msg.data} mission change')

        self.stop_current_mission()
        self.current_index = self.missions.index(msg.data)        
        self.start_current_mission()

def main(args=None):
    rclpy.init(args=args)
    node = MissionsCore()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
