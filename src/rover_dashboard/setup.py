from setuptools import setup, find_packages
import os

package_name = 'rover_dashboard'


def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    # Include templates and static files
    for root, dirs, files in os.walk('rover_dashboard/templates'):
        for f in files:
            path = os.path.join(root, f)
            install_dir = os.path.join('share', package_name, root)
            data_files.append((install_dir, [path]))
    for root, dirs, files in os.walk('rover_dashboard/static'):
        for f in files:
            path = os.path.join(root, f)
            install_dir = os.path.join('share', package_name, root)
            data_files.append((install_dir, [path]))
    return data_files


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools', 'flask', 'flask-socketio'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='rover@example.com',
    description='Flask web dashboard with live stream.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = rover_dashboard.dashboard_node:main',
        ],
    },
)
