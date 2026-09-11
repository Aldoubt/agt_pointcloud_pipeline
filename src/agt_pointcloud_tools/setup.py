from setuptools import find_packages, setup

package_name = 'agt_pointcloud_tools'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Xuan Yang',
    maintainer_email='xuanyang.robotics@gmail.com',
    description='Offline rosbag analysis and visual tuning tools for AGT point cloud pipeline.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'agt-lidar-analyze = agt_pointcloud_tools.cli:main',
            'agt-lidar-tuner = agt_pointcloud_tools.tuner_gui:main',
        ],
    },
)
