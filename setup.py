from setuptools import setup

package_name = 'tracetools_trace'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
)
