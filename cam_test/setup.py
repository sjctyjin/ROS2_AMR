from setuptools import find_packages, setup

package_name = 'cam_test'

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
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'node_object_webcams    = cam_test.cam_test:main',
        'topic_pub 		= cam_test.topic_helloworld_pub:main',
        'topic_sub 		= cam_test.topic_helloworld_sub:main',
        'topic_webcam_pub       = cam_test.topic_webcam_pub:main',
        'topic_webcam_sub       = cam_test.topic_webacm_sub:main',
        'service_object_server  = cam_test.service_object_client:main',
        'topic_odom_sub         = cam_test.topic_odom_sub:main',
        'topic_webcam_sub_compress = cam_test.topic_webcam_sub_compress:main',
        'topic_goal_pose        = cam_test.topic_goal_pose:main'
        ],
    },
)
