import glob

from ament_package.generate_setuptools_dict import generate_setuptools_dict
from setuptools import find_packages
from setuptools import setup

package_name = 'tracetools_launch'
package_info = generate_setuptools_dict(
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    keywords=[],
    tests_require=['pytest'],
)
setup(**package_info)
