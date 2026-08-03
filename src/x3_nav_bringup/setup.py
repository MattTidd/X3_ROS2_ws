from setuptools import find_packages, setup

package_name = 'x3_nav_bringup'

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
    description='Package containing the goal client for the X3 DRL implementation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "goal_client = x3_nav_bringup.goal_client:main"
        ],
    },
)
