from setuptools import find_packages, setup

package_name = 'state_robot'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name, ['launch/state_robot.launch.py']),
        #('share/' + urdf_test, ['launch/robot_pub_urdf_launch.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auv',
    maintainer_email='auv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = state_robot.state_publisher:main'
        ],
    },
)
