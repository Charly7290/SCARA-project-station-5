from setuptools import setup

package_name = 'scara_teleop'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/teleop.launch.py',
            'launch/controller_teleop.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='Charly',
    maintainer_email='carlosdanielmurillomena@gmail.com',
    description='Keyboard + controller teach-pendant teleop for Station 5 (SCARA) over scara_cmd / Micro-ROS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = scara_teleop.keyboard_teleop_node:main',
            'controller_teleop = scara_teleop.controller_teleop_node:main',
        ],
    },
)
