from setuptools import setup

package_name = 'opencv_course'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huanyu-pc',
    maintainer_email='huanyu-pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'Imread = opencv_course.Imread:main',
            'Videoread = opencv_course.Videoread:main',
            'Img_filter = opencv_course.Img_filter:main',
            'image_edge = opencv_course.image_edge:main',
            'Im_morphology = opencv_course.Im_morphology:main',
            'Im_threshold = opencv_course.Im_threshold:main',
            'convolutions = opencv_course.convolutions:main',
        ],
    },
)
