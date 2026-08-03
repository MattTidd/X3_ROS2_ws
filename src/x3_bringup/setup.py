from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'x3_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include directories:
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.py"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Tidd',
    maintainer_email='mtidd2@unb.ca',
    description='Package for performing bringup on real X3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mcnamu_driver = x3_bringup.mcnamu_driver:main"
        ],
    },
)
