from setuptools import setup

package_name = 'scara_twin'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/twin.launch.py']),
        ('share/' + package_name + '/config', ['config/twin_params.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/twin.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charly',
    maintainer_email='carlosdanielmurillomena@gmail.com',
    description='Digital twin of the SCARA (Station 5) in RViz.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joint_bridge = scara_twin.joint_bridge:main',
            'target_ghost = scara_twin.target_ghost:main',
        ],
    },
)
