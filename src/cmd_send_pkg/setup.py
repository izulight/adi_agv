from setuptools import find_packages, setup

package_name = 'cmd_send_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auv',
    maintainer_email='takao131103@outlook.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clear_iiexceeded_node = cmd_send_pkg.clear_iiexceeded_node:main',
            'brake_deactivate_node = cmd_send_pkg.brake_deactivate_node:main',
            'brake_activate_node = cmd_send_pkg.brake_activate_node:main',
            #新規作成コマンドをここに追記
        ],
    },
)
