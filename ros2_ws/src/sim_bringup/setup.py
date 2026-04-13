from setuptools import setup

package_name = 'sim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/sim_bringup.launch.py', 'launch/hw_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_toolbox_online_async.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/pool_sim.rviz']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO Maintainer',
    maintainer_email='todo@example.com',
    description='Launch and configuration files for the simulation-first stack.',
    license='MIT',
)
