from setuptools import find_packages, setup

package_name = 'kitti_pointcloud_publisher'

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
    maintainer='samir',
    maintainer_email='samir.abdallah@telecomnancy.eu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'kitti_pointcloud_publisher = kitti_pointcloud_publisher.kitti_pointcloud_publisher:main'
        ],
    },
)
