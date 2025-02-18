#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from px4_msgs.srv import VehicleCommand
from px4_msgs.msg import TrajectorySetpoint
import threading
import time

class Arming(Node):
    def __init__(self):
        super().__init__('arming')

        self.declare_parameter('nb_drones', 1)
        self.nb_drones = self.get_parameter('nb_drones').get_parameter_value().integer_value
        
        self.arm_check_clients = []
        self.is_armed = [False] * self.nb_drones
        self.is_offboard = [False] * self.nb_drones
        self.is_taken_off = [False] * self.nb_drones  # Track takeoff status

        # Publishers for velocity control
        self.velocity_publishers = []
        
        for i in range(self.nb_drones):
            service = f'/px4_{i + 1}/fmu/vehicle_command'
            client = self.create_client(VehicleCommand, service)
            self.arm_check_clients.append(client)

            topic = f'/px4_{i + 1}/fmu/trajectory_setpoint/out'
            pub = self.create_publisher(TrajectorySetpoint, topic, 10)
            self.velocity_publishers.append(pub)

            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service for drone {i + 1} not available')
            else:
                self.get_logger().info(f'Vehicle command service for drone {i + 1} available')

        self.timer = self.create_timer(1.0, self.control_loop)
    
    def control_loop(self):
        for i in range(self.nb_drones):
            if not self.is_armed[i]:
                self.arm(i)
            elif not self.is_offboard[i]:
                self.set_offboard_mode(i)
            elif not self.is_taken_off[i]:
                self.takeoff(i)
            else:
                self.hold_position(i)

    def arm(self, idx):
        self.send_vehicle_command(400, idx, param1=1.0)
    
    def set_offboard_mode(self, idx):
        self.send_vehicle_command(176, idx, param1=1.0, param2=6.0)
    
    def takeoff(self, idx):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -5.0]  # Adjusted altitude
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.velocity_publishers[idx].publish(msg)
        self.is_taken_off[idx] = True
        self.get_logger().info(f"Takeoff command sent to drone {idx + 1}.")
        #self.send_vehicle_command(22, idx, param7=3.0)  # Takeoff to 3m
        
    def send_vehicle_command(self, command, idx, param1=0.0, param2=0.0, param7=0.0):
        request = VehicleCommand.Request()
        request.request.command = command
        request.request.param1 = param1
        request.request.param2 = param2
        request.request.param7 = param7
        request.request.target_component = 1
        request.request.source_system = 1
        request.request.source_component = 1
        request.request.from_external = True
        request.request.timestamp = self.get_clock().now().nanoseconds // 1000
        
        future = self.arm_check_clients[idx].call_async(request)
        future.add_done_callback(lambda f: self.command_response(idx, command, f))
    
    def command_response(self, idx, command, future):
        try:
            response = future.result()
            if response and response.reply:
                if command == 400:
                    self.is_armed[idx] = True
                    self.get_logger().info(f'Drone {idx + 1} armed!')
                elif command == 176:
                    self.is_offboard[idx] = True
                    self.get_logger().info(f'Drone {idx + 1} in OFFBOARD mode!')
                elif command == 22:
                    self.is_taken_off[idx] = True
                    self.get_logger().info(f'Drone {idx + 1} took off to 3m!')
        except Exception as e:
            self.get_logger().error(f'Command failed for Drone {idx + 1}: {e}')
    
    def hold_position(self, idx):
        self.get_logger().info(f"Drone {idx + 1} Attempting to hold position")
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -3.0]  # Hold altitude at 3m
        msg.yaw = 0.0
        self.velocity_publishers[idx].publish(msg)

def main(args=None):
    print('Starting arming drones...')
    rclpy.init(args=args)
    node = Arming()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''


        
    
    
    
'''