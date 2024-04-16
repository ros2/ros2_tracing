from setuptools import find_packages
from setuptools import setup

package_name = 'test_ros2trace'

setup(
    name=package_name,
    version='8.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christophe Bedard',
    maintainer_email='bedard.christophe@gmail.com',
    author='Christophe Bedard',
    author_email='bedard.christophe@gmail.com',
    url='https://github.com/ros2/ros2_tracing',
    keywords=[],
    description='Tests for the ros2trace package.',
    license='Apache 2.0',
    tests_require=['pytest'],
)
