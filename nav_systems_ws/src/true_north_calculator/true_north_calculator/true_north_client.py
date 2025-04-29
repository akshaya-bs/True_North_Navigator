
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from true_north_interfaces.srv import TrueNorthCalculation
from std_srvs.srv import Trigger 

class TrueNorthClient(Node):
    def __init__(self):
        super().__init__('true_north_client')
        self.client = self.create_client(
            TrueNorthCalculation, 
            'calculate_true_north'
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service unavailable, waiting...')

    def get_declination(self):
        return self.client.call_async(TrueNorthCalculation.Request())

def main(args=None):
    rclpy.init(args=args)
    client = TrueNorthClient()
    
    future = client.get_declination()
    
    # Process while waiting for service
    while rclpy.ok():
        rclpy.spin_once(client)
        if future.done():
            try:
                response = future.result()
                print(f'''
                Magnetic Declination: {response.declination:.3f}°
                Direction: {response.declination_direction}
                True North Position: {response.true_north_direction}
                '''.strip())
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
                break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

