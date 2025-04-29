
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from true_north_interfaces.srv import TrueNorthCalculation
from sensor_msgs.msg import NavSatFix
from .time_utils import TimeConverter
from .core import GeoMag
import math
from datetime import datetime


class TrueNorthService(Node):
    def __init__(self):
        super().__init__('true_north_service')

        # Initialize service
        self.srv = self.create_service(
            TrueNorthCalculation,
            'calculate_true_north',
            self.calculate_true_north_callback
        )

        # Initialize GNSS subscriber
        self.gnss_sub = self.create_subscription(
            NavSatFix,
            '/gx5/gnss1/fix',
            self.gnss_callback,
            10
        )

        # Store latest GNSS data
        self.latest_gnss = None
        self.gnss_data_received = False

        self.get_logger().info('True North Service initialized with GNSS subscription')

    def gnss_callback(self, msg):
        """Store latest GNSS data only once"""
        if not self.gnss_data_received:
            self.latest_gnss = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude / 1000.0  # Convert meters to kilometers
            }
            self.gnss_data_received = True  # Set flag to indicate GNSS data is ready
            self.get_logger().info(
                f'GNSS data received: Latitude={msg.latitude}, Longitude={msg.longitude}, Altitude={msg.altitude / 1000.0} km'
            )

    def calculate_true_north_callback(self, request, response):
        """Service callback using GNSS data"""
        if not self.gnss_data_received or self.latest_gnss is None:
            self.get_logger().error('No GNSS data available')
            response.declination = 0.0
            response.declination_direction = "Error"
            response.true_north_direction = "No GNSS data"
            return response

        try:
            # Get coordinates from stored GNSS data
            lat = self.latest_gnss['latitude']
            lon = self.latest_gnss['longitude']
            alt = self.latest_gnss['altitude']

            # Perform calculation
            geo_mag = GeoMag()
            decimal_year = TimeConverter.decimal_year_from_date(datetime.now())

            result = geo_mag.calculate(
                glat=lat,
                glon=lon,
                alt=alt,
                time=decimal_year
            )

            # Determine directions
            if result.d > 0:
                decl_dir = "East"
                true_north_dir = "left"
            elif result.d < 0:
                decl_dir = "West"
                true_north_dir = "right"
            else:
                decl_dir = "Zero"
                true_north_dir = "aligned"

            # Set response
            response.declination = math.degrees(result.d)
            response.declination_direction = decl_dir
            response.true_north_direction = true_north_dir

            # Log the calculated True North information
            self.get_logger().info(f'Calculated True North: Declination={response.declination:.3f}° {decl_dir}')

        except Exception as e:
            self.get_logger().error(f'Calculation error: {str(e)}')
            response.declination = 0.0
            response.declination_direction = "Error"
            response.true_north_direction = "Calculation failed"

        return response


def main(args=None):
    rclpy.init(args=args)
    service = TrueNorthService()
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Shutting down service')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

 
