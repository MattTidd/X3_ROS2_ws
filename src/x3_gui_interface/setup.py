from setuptools import find_packages, setup

package_name = 'x3_gui_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Tidd',
    maintainer_email='mtidd2@unb.ca',
    description='Contains the GUI interface for interfacing with the MRS',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'x3_gui_interface = x3_gui_interface.x3_gui_interface:main',
        ],
    },
)
