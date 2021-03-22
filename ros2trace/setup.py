from ament_package.generate_setuptools_dict import generate_setuptools_dict
from setuptools import find_packages
from setuptools import setup

package_name = 'ros2trace'
package_info = generate_setuptools_dict(
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    keywords=[],
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            f'trace = {package_name}.command.trace:TraceCommand',
        ],
    }
)
setup(**package_info)
