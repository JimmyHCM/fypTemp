from setuptools import setup

package_name = 'local_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO Maintainer',
    maintainer_email='todo@example.com',
    description='Local planner implementing a simple pure pursuit strategy.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_planner_node = local_planner.local_planner_node:main',
        ],
    },
)
