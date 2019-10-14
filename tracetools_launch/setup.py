import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'tracetools_launch'

setup(
    name=package_name,
    version='0.2.8',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    maintainer=(
        'Christophe Bedard, '
        'Ingo Lütkebohle'
    ),
    maintainer_email=(
        'bedard.christophe@gmail.com, '
        'ingo.luetkebohle@de.bosch.com'
    ),
    author='Christophe Bedard',
    author_email='fixed-term.christophe.bourquebedard@de.bosch.com',
    url='https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing',
    keywords=[],
    description='Launch integration for tracing.',
    long_description=(
        'This package provides a trace action to '
        'launch tracing through a launch file.'
    ),
    license='Apache 2.0',
    tests_require=['pytest'],
)
