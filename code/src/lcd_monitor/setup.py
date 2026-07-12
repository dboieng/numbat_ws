from setuptools import setup, find_packages

package_name = 'lcd_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lcd_monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike@todo.todo',
    description='LCD monitor node for Spot Micro (ROS 2)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lcd_monitor_node = lcd_monitor.sm_lcd_driver:main',
        ],
    },
)
