from setuptools import setup, find_packages

package_name = 'rover_serial'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='rover@example.com',
    description='USB serial communication node between Raspberry Pi and Arduino.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = rover_serial.serial_node:main',
        ],
    },
)
