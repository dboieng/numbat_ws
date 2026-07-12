from setuptools import setup, find_packages

package_name = 'servo_move_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/keyboard_move.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mike@todo.todo',
    description='Package to move servos via keyboard inputs',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_move_keyboard_node = servo_move_keyboard.servoMoveKeyboard:main',
        ],
    },
)