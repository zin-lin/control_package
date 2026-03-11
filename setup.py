from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_package'
launch = 'launch'
imgs = 'control_package/imgs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zin',
    maintainer_email='zinlinhtun34@gmail.com',
    description='Control Vehicle through terminal or server',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terminal_control=control_package.control:main',
            'app=control_package.gui_control:main',
            'rolle=control_package.rolle_gui_control:main',
            'bev=control_package.bev:main',
            'bevpo=control_package.bevpo:main',
            'l2=control_package.l2:main',
            'rec_depth=control_package.rec_depth:main',
            'rec_rgb=control_package.rec_rgb:main',
            'rec_pc2=control_package.rec_pc2:main',
        ],
    },
)
