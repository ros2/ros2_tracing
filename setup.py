from setuptools import find_packages
from setuptools import setup

package_name = 'tracetools_trace'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    keywords=['ROS'],
    description='Tools for setting up tracing sessions',
    entry_points={
        'console_scripts': [
            f'trace = {package_name}.trace:main',
        ],
    },
    tests_require=['pytest'],
)
