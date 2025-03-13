#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.srv import VehicleCommand
from px4_msgs.msg import VehicleCommand as vc
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool

class MultiOffboardControl(Node):
    def __init__(self, drone_namespace):
        super().__init__('hardcode_sequence')

        self.drone_namespace = drone_namespace
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Create subscriptions
        self.status_sub   = self.create_subscription(VehicleStatus, f'{drone_namespace}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.attitude_sub = self.create_subscription(VehicleAttitude, f'/{drone_namespace}/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile)
        self.my_bool_sub  = self.create_subscription(Bool, '/arm_message', self.arm_message_callback, qos_profile)

        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'/{drone_namspace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity      = self.create_publisher(Twist, f'/{drone_namspace}/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory    = self.create_publisher(TrajectorySetpoint, f'/{drone_namspace}/fmu/in/trajectory_setpoint', qos_profile)

        service = f'/{drone_namspace}/fmu/vehicle_command'
        client  = self.create_client(VehicleCommand, service)
        arm_check_clients[drone_namspace] = client

        arm_timer_period = .1 # seconds
        self.arm_timer_  = self.create_timer(arm_timer_period, self.arm_timer_callback)

        timer_period = 0.02  # seconds
        self.timer   = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state     = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state     = VehicleStatus.ARMING_STATE_ARMED
        self.velocity      = Vector3()
        self.yaw           = 0.0  #yaw value we send as command
        self.trueYaw       = 0.0  #current yaw value of drone
        self.offboardMode  = False
        self.flightCheck   = False
        self.myCnt         = 0
        self.arm_message   = False
        self.failsafe      = False
        self.current_state = "IDLE"
        self.last_state    = self.current_state

        self.current_position = Vector3()


    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")


    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")
            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1


    def arm(self):
        request = VehicleCommand.Request()
        request.request.command = 400
        request.request.param1 = 1.0
        request.request.param2 = 0.0
        request.request.target_component = 1
        request.request.source_system = 1
        request.request.source_component = 1
        request.request.from_external = True
        request.request.timestamp = self.get_clock().now().nanoseconds // 1000
        is_armed[self.drone_namespace] = True
          
        future = arm_check_clients[self.drone_namespace].call_async(request)
        future.add_done_callback(lambda f: self.command_response(400, f))
    
    def state_offboard(self):
        self.myCnt = 0
        request = VehicleCommand.Request()
        request.request.command = vc.VEHICLE_CMD_DO_SET_MODE
        request.request.param1 = 1.0
        request.request.param2 = 6.0
        request.request.target_component = 1
        request.request.source_system = 1
        request.request.source_component = 1
        request.request.from_external = True
        request.request.timestamp = self.get_clock().now().nanoseconds // 1000
        is_offboard[self.drone_namespace] = True   
        future = arm_check_clients[self.drone_namespace].call_async(request)
        future.add_done_callback(lambda f: self.command_response(176, f))

    def take_off(self):
        request = VehicleCommand.Request()
        request.request.command = vc.VEHICLE_CMD_NAV_TAKEOFF
        request.request.param1 = 1.0
        request.request.param2 = 0.0
        request.request.param7 = 1.0
        request.request.target_component = 1
        request.request.source_system = 1
        request.request.source_component = 1
        request.request.from_external = True
        request.request.timestamp = self.get_clock().now().nanoseconds // 1000
        is_taken_off[self.drone_namespace] = True
        future = arm_check_clients[self.drone_namespace].call_async(request)
        future.add_done_callback(lambda f: self.command_response(22, f))

    def command_response(self, command, future):
        try:
            response = future.result()
            if response and response.reply:
                if command == 400:
                    is_armed[self.drone_namespace] = True
                    #self.get_logger().info(f'Drone {self.drone_namespace} armed!')
                elif command == 176:
                    is_offboard[self.drone_namespace] = True
                    self.get_logger().info(f'Drone {self.drone_namespace} in OFFBOARD mode!')
                elif command == 22:
                    is_taken_off[self.drone_namespace] = True
                    # self.get_logger().info(f'Drone {self.drone_namespace} took off to 3m!')
        except Exception as e:
            self.get_logger().error(f'Command failed for Drone {self.drone_namespace}: {e}')

    

    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass


    def attitude_callback(self, msg):
        #self.get_logger().info(f"attitude velocity callback for {self.drone_namespace}")
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))  

    def cmdloop_callback(self):
        c = 0
        for drone in drones:
            if(is_armed[self.drone_namespace] == True and is_taken_off[self.drone_namespace] == True and is_offboard[self.drone_namespace] == True):
                c += 1
        if c == 5:
            self.timer.cancel()
            if self.drone_namespace == '/px4_1':
                self.way_points = [[0.5, -6.5], [0.5, -5.5], [0.5, -4.5], [0.5, -3.5], [0.5, -2.5], 
                                    [0.5, -1.5], [0.5, -0.5], [0.5, 0.5], [0.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-1.5, 1.5], [-2.5, 1.5], [-3.5, 1.5], [-4.5, 1.5],
                                    [-5.5, 1.5], [-6.5, 1.5], [-7.5, 1.5], [-8.5, 1.5], [-9.5, 1.5],
                                    [-9.5, 1.5], [-8.5, 1.5], [-7.5, 1.5], [-6.5, 1.5], [-5.5, 1.5],
                                    [-4.5, 1.5], [-3.5, 1.5], [-2.5, 1.5], [-1.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5],
                                    [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5], [-0.5, 1.5],
                                    [0.5, 1.5], [0.5, 0.5], [0.5, -0.5], [0.5, -1.5], [0.5, -2.5],
                                    [-0.5, -3.5], [-0.5, -4.5], [-0.5, -5.5], [-0.5, -6.5], [-0.5, -7.5]
                                ]
            elif self.drone_namespace == '/px4_2':
                self.way_points = [[0.5, -7.5], [0.5, -6.5], [0.5, -5.5], [0.5, -4.5], [0.5, -3.5],
                                    [0.5, -2.5], [0.5, -1.5], [0.5, -0.5], [0.5, 0.5], [0.5, 1.5],
                                    [0.5, 2.5], [0.5, 3.5], [0.5, 4.5], [0.5, 5.5], [0.5, 6.5],
                                    [-0.5, 6.5], [-1.5, 6.5], [-2.5, 6.5], [-3.5, 6.5], [-4.5, 6.5],
                                    [-5.5, 6.5], [-6.5, 6.5], [-7.5, 6.5], [-8.5, 6.5], [-9.5, 6.5],
                                    [-9.5, 6.5], [-8.5, 6.5], [-7.5, 6.5], [-6.5, 6.5], [-5.5, 6.5],
                                    [-4.5, 6.5], [-3.5, 6.5], [-2.5, 6.5], [-1.5, 6.5], [-0.5, 6.5],
                                    [0.5, 6.5], [1.5, 6.5], [2.5, 6.5], [3.5, 6.5], [4.5, 6.5],
                                    [5.5, 6.5], [6.5, 6.5], [7.5, 6.5], [8.5, 6.5], [9.5, 6.5],
                                    [9.5, 6.5], [8.5, 6.5], [7.5, 6.5], [6.5, 6.5], [5.5, 6.5],
                                    [4.5, 6.5], [3.5, 6.5], [2.5, 6.5], [1.5, 6.5], [0.5, 6.5],
                                    [0.5, 5.5], [0.5, 4.5], [0.5, 3.5], [0.5, 2.5], [0.5, 1.5], 
                                    [0.5, 0.5], [0.5, -0.5], [0.5, -1.5], [0.5, -2.5], [0.5, -3.5], 
                                    [0.5, -4.5], [0.5, -5.5], [0.5, -6.5], [0.5, -7.5], [0.5, -8.5]
                                ]
            elif self.drone_namespace == '/px4_3':
                self.way_points = [[0.5, -8.5], [0.5, -7.5], [0.5, -6.5], [0.5, -5.5], [0.5, -4.5],
                                    [0.5, -3.5], [0.5, -2.5], [0.5, -1.5], [0.5, -0.5], [0.5, 0.5], 
                                    [0.5, 1.5], [1.5, 1.5], [2.5, 1.5], [3.5, 1.5], [4.5, 1.5],
                                    [5.5, 1.5], [6.5, 1.5], [7.5, 1.5], [8.5, 1.5], [9.5, 1.5],
                                    [9.5, 1.5], [8.5, 1.5], [7.5, 1.5], [6.5, 1.5], [5.5, 1.5],
                                    [4.5, 1.5], [3.5, 1.5], [2.5, 1.5], [1.5, 1.5], [0.5, 1.5],
                                    [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], 
                                    [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], 
                                    [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], 
                                    [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], 
                                    [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], 
                                    [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 1.5], [0.5, 0.5], 
                                    [0.5, -0.5], [0.5, -1.5], [0.5, -2.5], [0.5, -3.5], [0.5, -4.5], 
                                    [0.5, -5.5], [0.5, -6.5], [0.5, -7.5], [0.5, -8.5], [0.5, -9.5]               
                                ]
            elif self.drone_namespace == '/px4_4':
                self.way_points = [[-0.5, -7.5], [-0.5, -6.5], [-0.5, -5.5], [-0.5, -4.5], [-0.5, -3.5],
                                    [-1.5, -3.5], [-2.5, -3.5], [-3.5, -3.5], [-4.5, -3.5], [-5.5, -3.5], 
                                    [-6.5, -3.5], [-7.5, -3.5], [-8.5, -3.5], [-9.5, -3.5],
                                    [-8.5, -3.5], [-7.5, -3.5], [-6.5, -3.5], [-5.5, -3.5],
                                    [-4.5, -3.5], [-3.5, -3.5], [-2.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5], [-1.5, -3.5],
                                    [-1.5, -3.5], [-0.5, -3.5],
                                    [-0.5, -4.5], [-0.5, -5.5], [-0.5, -6.5], [-0.5, -7.5], [-0.5, -8.5]
                                ]
            elif self.drone_namespace == '/px4_5':
                self.way_points = [[1.5, -7.5], [1.5, -6.5], [1.5, -5.5], [1.5, -4.5], [1.5, -3.5],
                                    [1.5, -3.5], [2.5, -3.5], [3.5, -3.5], [4.5, -3.5], [-5.5, -3.5], 
                                    [6.5, -3.5], [7.5, -3.5], [8.5, -3.5], [9.5, -3.5],
                                    [8.5, -3.5], [7.5, -3.5], [6.5, -3.5], [5.5, -3.5],
                                    [4.5, -3.5], [3.5, -3.5], [2.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -3.5], [1.5, -3.5],
                                    [1.5, -4.5], [1.5, -5.5], [1.5, -6.5], [1.5, -7.5], [1.5, -8.5]
                                ]
        
            self.cmdloop()
    
    def cmdloop(self):
        for i in range(len(self.way_points)):
            if(is_offboard[self.drone_namespace] == True):
                offboard_msg = OffboardControlMode()
                offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.position = True
                offboard_msg.velocity = False
                offboard_msg.acceleration = False
                self.publisher_offboard_mode.publish(offboard_msg)            
                
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                trajectory_msg.velocity[0] = float('nan')
                trajectory_msg.velocity[1] = float('nan')
                trajectory_msg.velocity[2] = float('nan')
                trajectory_msg.position[0] = self.way_points[i][0]
                trajectory_msg.position[1] = self.way_points[i][1]
                trajectory_msg.position[2] = 5.0
                trajectory_msg.acceleration[0] = float('nan')
                trajectory_msg.acceleration[1] = float('nan')
                trajectory_msg.acceleration[2] = float('nan')
                trajectory_msg.yaw = float('nan')
                trajectory_msg.yawspeed = float('nan')

                self.publisher_trajectory.publish(trajectory_msg)

drones = ['/px4_1', '/px4_2', '/px4_3', '/px4_4', '/px4_5']

nb_drones = 5
arm_check_clients = {}
        
is_armed = {'px4_1':False, 'px4_2':False, 'px4_3':False, 'px4_4':False, 'px4_5':False}
is_offboard = {'px4_1':False, 'px4_2':False, 'px4_3':False, 'px_4':False, 'px4_5':False}
is_taken_off = {'px4_1':False, 'px4_2':False, 'px4_3':False, 'px_4':False, 'px4_5':False}

def main(args=None):
    rclpy.init(args=args)
    
    nodes = [OffboardControl(drone) for drone in drones]

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()