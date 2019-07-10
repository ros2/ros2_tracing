from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2trace',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    maintainer=(
        'Christophe Bedard, '
        'Ingo Lütkebohle'
    ),
    maintainer_email=(
        'fixed-term.christophe.bourquebedard@de.bosch.com, '
        'ingo.luetkebohle@de.bosch.com'
    ),
    author='Christophe Bedard',
    author_email='fixed-term.christophe.bourquebedard@de.bosch.com',
    # url=',
    keywords=[],
    description='The run command for ROS 2 command line tools.',
    long_description="""\
The package provides the trace command for the ROS 2 command line tools.""",
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'trace = ros2trace.command.trace:TraceCommand',
        ],
    }
)
