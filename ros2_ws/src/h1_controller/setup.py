from setuptools import find_packages, setup

package_name = 'h1_controller'

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
    maintainer='Vijay Yadav',
    maintainer_email='yvijay19071995@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = h1_controller.teleop_node:main',
            'waypoint_controller = h1_controller.waypoint_controller:main',
        ],
    },
)
