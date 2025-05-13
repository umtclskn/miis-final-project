from setuptools import find_packages, setup

package_name = 'robot_path_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/robot1_static_path.yml',
            'config/robot2_static_path.yml',
            'config/robot3_static_path.yml',
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umut',
    maintainer_email='umtclskn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'path_tracker = robot_path_tracker.path_tracker_node:main',
        ],
    },
)
