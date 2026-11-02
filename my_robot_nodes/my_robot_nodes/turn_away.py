#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed
from std_srvs.srv import SetBool

class TurnAway(Node):
    def __init__(self):
        super().__init__('turn_away')
        
        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)
        
        self.enabled = False
        # Publishers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher_ = self.create_publisher(RobotStatus, '/robot_status', 10)
        
        # Subscriber to /obstacle
        self.subscription = self.create_subscription(
            ObstacleInfo,
            '/obstacle',
            self.obstacle_callback,
            10)
        
        #Service
        self.enable_service = self.create_service(
            SetBool,
            '/turn_away_enable',
            self.enable_callback)
        self.speed_service = self.create_service(
            SetTurnSpeed,
            '/set_turn_speed',
            self.set_turn_speed_callback)
        self.get_logger().info('TurnAway node initialized...')
    def enable_callback(self, request, response):
        was_enabled = self.enabled
        self.enabled = request.data
        
        # If switched off, publish one zero velocity command
        if was_enabled and not self.enabled:
            twist = Twist() # defaults to 0.0
            self.publisher_.publish(twist)
            response.message = "TurnAway node disabled."
        else:
            response.message = "TurnAway node enabled." if self.enabled else "TurnAway node remains disabled."
            
        response.success = True
        self.get_logger().info(response.message)
        return response

    def set_turn_speed_callback(self, request, response):
        max_speed = self.get_parameter('max_turn_speed').get_parameter_value().double_value
        
        if request.turn_speed <= 0.0:
            response.success = False
            response.message = "Rejected: turn_speed must be greater than 0."
        elif request.turn_speed > max_speed:
            response.success = False
            response.message = f"Rejected: turn_speed exceeds max_turn_speed ({max_speed})."
        else:
            # Store accepted speed as parameter
            self.set_parameters([
                rclpy.parameter.Parameter('turn_speed', rclpy.Parameter.Type.DOUBLE, request.turn_speed)
            ])
            response.success = True
            response.message = f"Successfully updated turn_speed to {request.turn_speed} rad/s."
            self.get_logger().info(response.message)
            
        return response

    def obstacle_callback(self, msg: ObstacleInfo):
        forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        
        twist = Twist()
        status = RobotStatus()
        
        status.obstacle_distance = getattr(msg, 'distance', 0.0)
        
        if not self.enabled:
            status.state = "stopped"
            self.status_publisher_.publish(status)
            return
        
        if not msg.too_close:
            # Drive forward when clear
            twist.linear.x = forward_speed
            twist.angular.z = 0.0
            status.state = "forward"
        else:
            # Stop moving forward when too close
            twist.linear.x = 0.0
            # Obstacle is to the left if angle > 0 -> turn right (negative angular.z)
            # Otherwise turn left (positive angular.z)
            if msg.angle > 0.0:
                twist.angular.z = -turn_speed
            else:
                twist.angular.z = turn_speed
            status.state = "turning"
                
        self.publisher_.publish(twist)
        self.status_publisher_.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = TurnAway()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()