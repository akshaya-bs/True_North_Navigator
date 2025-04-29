#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from true_north_interfaces.srv import TrueNorthCalculation
from geometry_msgs.msg import Twist

class TrueNorthSystemManager(Node):
    def __init__(self):
        super().__init__('true_north_system_manager')
        # Existing services
        self.create_service(Trigger, '/activate_true_north', self.activate_callback)
        
        # Controller client
        self.controller_client = self.create_client(
            Trigger,  # Using Empty service type for activation
            '/activate_controller'
        )
    def activate_callback(self, request, response):
        """Handle /activate_true_north service call"""
        self.get_logger().info('Starting True North system...')
    
    # Start controller node
        if not self.start_controller():
            self.get_logger().error('Controller activation failed')
            response.success = False  # Explicitly set response
            return response
        
    # Wait for calculation service
        if not self.wait_for_service('/calculate_true_north', timeout_sec=5.0):
            self.get_logger().error('Calculation service not available')
            response.success = False  # Explicitly set response
            return response
        
        response.success = True  # Explicitly set success
        return response

    def start_controller(self):
        """Activate controller through service call"""
        req = Trigger.Request()
        try:
            future = self.controller_client.call_async(req)
            while not future.done():
                rclpy.spin_once(self)
                return future.result() is not None
        except Exception as e:
            self.get_logger().error(f'Controller activation error: {str(e)}')
            return False
'''   
    def activate_callback(self, request, response):
        """Handle /activate_true_north service call"""
        self.get_logger().info('Starting True North system...')
        
        # Start controller node
        if not self.start_controller():
            self.get_logger().error('Controller activation failed')
            return response
            
        # Wait for calculation service
        if not self.wait_for_service('/calculate_true_north', timeout_sec=5.0):
            self.get_logger().error('Calculation service not available')
            return response
            
        return response

    def start_controller(self):
        """Activate controller through service call"""
        req = Empty.Request()
        try:
            future = self.controller_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            #rclpy.spin_once(self, timeout_sec=3.0)
            return future.result() is not None
        except Exception as e:
            self.get_logger().error(f'Controller activation error: {str(e)}')
            return False
     
'''
def main(args=None):
    rclpy.init(args=args)
    manager = TrueNorthSystemManager()
    rclpy.spin(manager)
    rclpy.shutdown()

