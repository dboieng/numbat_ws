from setuptools import setup, find_packages

package_name = 'spot_micro_keyboard_command'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/keyboard_command.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike@todo.todo',
    description='Keyboard command node for Spot Micro (ROS 2)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_micro_keyboard_command_node = spot_micro_keyboard_command.spotMicroKeyboardMove:main',
        ],
    },
)