
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import UInt32, Float64
from geometry_msgs.msg import Twist
from true_north_interfaces.srv import TrueNorthCalculation
import math

class TrueNorthController(Node):
    def __init__(self):
        super().__init__('true_north_controller')
        self.get_logger().info('Controller initialized')

        # Current state
        self.current_heading = 0.0  # Degrees
        self.declination = 0.0      # Degrees
        self.target_heading = 0.0   # Degrees
        self.compass_ready = False
        self.rotation_active = False
        self.rotation_threshold =0.5  # Degrees
        self.control_timer = None
        
        # State machine
        # States: IDLE, ALIGNING_TO_MAGNETIC_NORTH, CALCULATING_DECLINATION, ALIGNING_TO_TRUE_NORTH
        self.state = "IDLE"

        # Setup publishers
        self.control_mode_pub = self.create_publisher(UInt32, '/command/setControlMode', 10)
        self.twist_pub = self.create_publisher(
            Twist, '/mcu/command/manual_twist',
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))

        # Compass subscriber (changed from magnetometer)
        self.create_subscription(Float64, '/gx5/compass', self.compass_callback, 10)

        # Services
        self.create_service(Trigger, '/activate_controller', self.activate_callback)
        self.true_north_client = self.create_client(TrueNorthCalculation, 'calculate_true_north')

    def compass_callback(self, msg):
        """Process compass heading data"""
        self.current_heading = msg.data  # Direct heading in degrees from compass
        
        # Emergency stop check when actively rotating
        if self.rotation_active:
            if self.state == "ALIGNING_TO_MAGNETIC_NORTH":
                # Check if we're close to magnetic north (0°)
                if abs(self.current_heading) <= self.rotation_threshold:
                    self._stop_rotation()
                    self.get_logger().info(f'Aligned to Magnetic North: {self.current_heading:.2f}°')
                    self.state = "CALCULATING_DECLINATION"
                    self._calculate_declination()
            
            elif self.state == "ALIGNING_TO_TRUE_NORTH":
                # Check if we're close to the target heading (true north)
                current_diff = self._calculate_heading_difference(self.target_heading, self.current_heading)
                if abs(current_diff) <= self.rotation_threshold:
                    self._stop_rotation()
                    self.get_logger().info(f'Aligned to True North. Final heading: {self.current_heading:.2f}°')
                    self.state = "IDLE"

        if not self.compass_ready:
            self.compass_ready = True
            self.get_logger().info(f'Compass initialized. Current heading: {self.current_heading:.2f}°')

    def _calculate_heading_difference(self, target, current):
        """Calculate the shortest rotation path between two headings"""
        # Handle the -180 to 180 range of compass data
        diff = target - current
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff

    def activate_callback(self, request, response):
        """Handle controller activation"""
        self.get_logger().info('Starting alignment process...')

        if not self.compass_ready:
            self.get_logger().error('Compass not initialized')
            response.success = False
            response.message = "Compass not initialized"
            return response

        self.initialize_control()
        
        # Begin the state machine by aligning to magnetic north
        self.state = "ALIGNING_TO_MAGNETIC_NORTH"
        self._start_magnetic_north_alignment()
        
        response.success = True
        response.message = "Alignment started"
        return response

    def initialize_control(self):
        """Set control mode to manual"""
        mode_msg = UInt32(data=170)
        self.control_mode_pub.publish(mode_msg)
        self.get_logger().info('Control mode set to AUTOMATIC')

    def _start_magnetic_north_alignment(self):
        """Begin alignment to magnetic north (0°)"""
        if abs(self.current_heading) <= self.rotation_threshold:
            self.get_logger().info('Already aligned with Magnetic North')
            self.state = "CALCULATING_DECLINATION"
            self._calculate_declination()
            return
            
        self.rotation_active = True
        self.control_timer = self.create_timer(0.1, self.control_callback)
        self.get_logger().info(f'Turning to Magnetic North from {self.current_heading:.2f}°')

    def _calculate_declination(self):
        """Request magnetic declination calculation"""
        self.get_logger().info('Calculating magnetic declination...')
        
        if not self.true_north_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('True North service unavailable')
            self.state = "IDLE"
            return
            
        future = self.true_north_client.call_async(TrueNorthCalculation.Request())
        future.add_done_callback(self._declination_callback)

    def _declination_callback(self, future):
        """Handle declination service response and start true north alignment"""
        try:
            response = future.result()
            if response.declination_direction == "Error":
                raise Exception(response.true_north_direction)

            # Apply correct declination sign based on direction
            self.declination = abs(response.declination) * (1 if response.declination_direction == "East" else -1)
            
            # Calculate target heading for true north
            # True north = magnetic north - declination
            self.target_heading = -self.declination  # Since magnetic north is 0°
            
            # Ensure target is in the -180 to 180 range
            if self.target_heading > 180:
                self.target_heading -= 360
            elif self.target_heading < -180:
                self.target_heading += 360

            self.get_logger().info(f'''
            Current Heading: {self.current_heading:.2f}°
            Declination: {self.declination:.2f}° ({response.declination_direction})
            Target Heading (True North): {self.target_heading:.2f}°''')

            # Check if already aligned with true north
            current_diff = self._calculate_heading_difference(self.target_heading, self.current_heading)
            if abs(current_diff) < self.rotation_threshold:
                self.get_logger().info('Already aligned with True North')
                self.state = "IDLE"
                return

            # Begin alignment to true north
            self.state = "ALIGNING_TO_TRUE_NORTH"
            self.rotation_active = True
            self.control_timer = self.create_timer(0.1, self.control_callback)

        except Exception as e:
            self.get_logger().error(f'Service failed: {str(e)}')
            self.state = "IDLE"
            self.rotation_active = False

    def control_callback(self):
        """Closed-loop heading control with optimal rotation direction"""
        if not self.rotation_active:
            return

        # Determine target based on current state
        target = 0.0 if self.state == "ALIGNING_TO_MAGNETIC_NORTH" else self.target_heading
        
        # Calculate current difference using the shortest path
        current_diff = self._calculate_heading_difference(target, self.current_heading)
        
        # Check alignment
        if abs(current_diff) <= self.rotation_threshold:
            self._stop_rotation()
            if self.state == "ALIGNING_TO_MAGNETIC_NORTH":
                self.get_logger().info(f'Aligned to Magnetic North: {self.current_heading:.2f}°')
                self.state = "CALCULATING_DECLINATION"
                self._calculate_declination()
            elif self.state == "ALIGNING_TO_TRUE_NORTH":
                self.get_logger().info(f'Aligned to True North. Final heading: {self.current_heading:.2f}°')
                self.state = "IDLE"
            return

        # Determine rotation direction with speed limit
        # Negative diff means we need to turn right (positive angular.z)
        # Positive diff means we need to turn left (negative angular.z)
        angular_z = -0.3 if current_diff > 0 else 0.3

        # Reduce speed when close to target
        if abs(current_diff) < 5:
            angular_z *= 0.5
            
        # Publish command
        twist = Twist()
        twist.angular.z = angular_z
        self.twist_pub.publish(twist)
        self.get_logger().debug(f'Adjusting: {self.current_heading:.2f}° → {target:.2f}°, diff={current_diff:.2f}°')

    def _stop_rotation(self):
        """Stop rotation and clean up resources"""
        twist = Twist()
        self.twist_pub.publish(twist)  # Send zero velocity command
        self.rotation_active = False
        if self.control_timer:
            self.control_timer.cancel()
        self.get_logger().info('Rotation stopped')

def main(args=None):
    rclpy.init(args=args)
    controller = TrueNorthController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

