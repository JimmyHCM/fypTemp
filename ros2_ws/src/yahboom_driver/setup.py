from setuptools import setup

package_name = 'yahboom_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='TODO Maintainer',
    maintainer_email='todo@example.com',
    description='Yahboom YB-ERF01-V3.0 ESC driver for differential drive.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yahboom_driver_node = yahboom_driver.yahboom_driver_node:main',
        ],
    },
)
