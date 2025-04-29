from setuptools import setup
import os
from glob import glob
package_name = 'true_north_calculator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('true_north_calculator/*.COF')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),


        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rsrch-349',
    maintainer_email='balasubramaniakshaya@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'true_north_service = true_north_calculator.true_north_service:main',
        'true_north_client = true_north_calculator.true_north_client:main',
        'true_north_controller = true_north_calculator.true_north_controller:main',
        'true_north_manager = true_north_calculator.true_north_manager:main',
        
        ],
    },
)
