from setuptools import setup

package_name = 'polar_to_grid_mapper'

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
    description='Laser scan to occupancy grid mapper.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polar_to_grid_mapper_node = polar_to_grid_mapper.mapper_node:main',
        ],
    },
)
