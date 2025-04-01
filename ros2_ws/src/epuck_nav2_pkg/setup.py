from setuptools import setup
import os
from glob import glob

package_name = 'epuck_nav2_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # Empty since there's no epuck_nav2_pkg/ module directory
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install the script to share/epuck_nav2_pkg/scripts/
        (os.path.join('share', package_name, 'scripts'), ['scripts/epuck_controller_nav2.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='E-puck navigation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Optional: 'epuck_controller_nav2 = epuck_nav2_pkg.scripts.epuck_controller_nav2:main',
        ],
    },
)