
from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
        ('share/' + package_name + '/config', [
            'config/amcl_config.yaml',
            'config/global_costmap.yaml',
            'config/local_costmap.yaml',
            'config/nav2_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_nav_goal_publisher = robot_navigation.simple_nav_goal_publisher:main',
        ],
    },
)
