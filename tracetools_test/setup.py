from setuptools import find_packages
from setuptools import setup

package_name = 'tracetools_test'

setup(
    name=package_name,
    version='4.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    maintainer=(
        'Christophe Bedard, '
        'Ingo Luetkebohle'
    ),
    maintainer_email=(
        'bedard.christophe@gmail.com, '
        'ingo.luetkebohle@de.bosch.com'
    ),
    author='Christophe Bedard',
    author_email='fixed-term.christophe.bourquebedard@de.bosch.com',
    url='https://github.com/ros2/ros2_tracing',
    keywords=[],
    description='Utilities for tracing-related tests.',
    license='Apache 2.0',
    tests_require=['pytest'],
)
