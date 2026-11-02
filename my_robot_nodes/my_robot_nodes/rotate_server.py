#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import Rotate

def yaw_from_quaternion(orientation):
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_difference(angle1, angle2):
    diff = angle1 - angle2
    return math.atan2(math.sin(diff), math.cos(diff))

class RotateServer(Node):
    def __init__(self):
        super().__init__('rotate_server')
        
        self.declare_parameter('turn_speed', 0.5)
        
        # Action Server
        self._action_server = ActionServer(
            self,
            Rotate,
            '/rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Publisher & Subscriber with Sensor Data QoS for reliable odom reception
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        self.current_yaw = 0.0
        self.last_yaw = 0.0
        self.accumulated_turned = 0.0
        self.odom_received = False
        
        self.get_logger().info('Rotate server initialized.')

    def odom_callback(self, msg: Odometry):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.odom_received:
            self.current_yaw = yaw
            self.last_yaw = yaw
            self.odom_received = True
        else:
            self.current_yaw = yaw
            delta = angle_difference(self.current_yaw, self.last_yaw)
            self.accumulated_turned += delta
            self.last_yaw = self.current_yaw

    def goal_callback(self, goal_request):
        if goal_request.angle == 0.0:
            self.get_logger().warn('Rejected goal of zero angle.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel rotation goal.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal_angle = goal_handle.request.angle
        turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        
        self.get_logger().info(f'Executing rotate goal: target {goal_angle:.2f} rad at speed {turn_speed}')
        
        start_accumulated = self.accumulated_turned
        twist = Twist()
        direction = 1.0 if goal_angle > 0.0 else -1.0
        twist.angular.z = direction * turn_speed
        
        feedback_msg = Rotate.Feedback()
        result = Rotate.Result()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                goal_handle.canceled()
                result.turned = self.accumulated_turned - start_accumulated
                return result
            
            turned_so_far = self.accumulated_turned - start_accumulated
            feedback_msg.turned = turned_so_far
            goal_handle.publish_feedback(feedback_msg)
            
            # Check completion based on direction
            if direction > 0.0 and turned_so_far >= goal_angle:
                break
            elif direction < 0.0 and turned_so_far <= goal_angle:
                break
            
            self.publisher_.publish(twist)
            time.sleep(0.05) # 20 Hz non-blocking loop
            
        # Stop the robot when finished
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        
        goal_handle.succeed()
        result.turned = self.accumulated_turned - start_accumulated
        self.get_logger().info(f'Rotate goal succeeded! Turned: {result.turned:.2f} rad')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = RotateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()