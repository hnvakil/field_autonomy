from setuptools import setup

package_name = 'arkit_data_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch_data_servers.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali1',
    maintainer_email='allisonli18710@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_server = arkit_data_streamer.pose_server:main',
            'image_server = arkit_data_streamer.image_server:main'
        ],
    },
)
