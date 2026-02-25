#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import State

import time

class PX4RebootNode(Node):
    def __init__(self):
        super().__init__('px4_reboot_node')
        self.cli = self.create_client(CommandLong, '/mavros/cmd/command')
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

    def state_callback(self, msg):
        if msg.connected:
            self.get_logger().info('MAVROS connected to PX4')
            self.send_reboot_command()

    def send_reboot_command(self):
        req = CommandLong.Request()
        req.command = 246  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        req.param1 = 1.0   # reboot autopilot (use float)
        req.param2 = 0.0   # shutdown companion computer (0 = no)
        req.param3 = 0.0   # reserved
        req.param4 = 0.0   # reserved
        req.param5 = 0.0   # reserved
        req.param6 = 0.0   # reserved
        req.param7 = 0.0   # reserved
        req.confirmation = 0
        
        self.get_logger().info('Sending reboot command to PX4...')
        future = self.cli.call_async(req)
        future.add_done_callback(self.reboot_response_callback)

    def reboot_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Reboot response: success={response.success}, result={response.result}')
            
            if response.success:
                self.get_logger().info('Reboot command accepted, waiting for PX4 to reboot...')
                # Give some time for the command to be processed
                time.sleep(2.0)
            else:
                self.get_logger().error(f'Reboot command failed with result: {response.result}')
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
        # Shutdown the node after processing
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PX4RebootNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()