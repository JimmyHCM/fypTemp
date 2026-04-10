from setuptools import setup

package_name = 'sim_scan_publisher'

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
    description='Synthetic laser scan publisher for pool simulation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_scan_publisher_node = sim_scan_publisher.scan_publisher_node:main',
        ],
    },
)
