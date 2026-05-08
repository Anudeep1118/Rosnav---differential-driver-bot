from setuptools import setup, find_packages

package_name = 'rosnav_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RosNav',
    description='RosNav driver nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = rosnav_driver.serial_bridge:main',
            'waypoint_nav = rosnav_driver.waypoint_nav:main',
        ],
    },
)
