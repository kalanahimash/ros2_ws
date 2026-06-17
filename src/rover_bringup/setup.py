from setuptools import setup
import os

package_name = 'rover_bringup'


def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    for root, dirs, files in os.walk('launch'):
        for f in files:
            data_files.append(('share/' + package_name + '/' + root, [os.path.join(root, f)]))
    for root, dirs, files in os.walk('config'):
        for f in files:
            data_files.append(('share/' + package_name + '/' + root, [os.path.join(root, f)]))
    return data_files


setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='rover@example.com',
    description='Launch files for the autonomous rover.',
    license='MIT',
    entry_points={},
)
