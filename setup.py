from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob(os.path.join('launch', '*_launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zin',
    maintainer_email='zinlinhtun34@gmail.com',
    description='Controlling the robot via a controlled environment, i.e. a terminal or a server',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control=control_package.control:main'
        ],
    },
)
