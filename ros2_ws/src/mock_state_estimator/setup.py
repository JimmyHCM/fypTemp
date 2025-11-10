from setuptools import setup

package_name = 'mock_state_estimator'

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
    description='Simple motion model with noise to provide simulated odometry.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_state_estimator_node = mock_state_estimator.state_estimator_node:main',
        ],
    },
)
