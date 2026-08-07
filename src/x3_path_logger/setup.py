from setuptools import find_packages, setup

package_name = 'x3_path_logger'

setup(
    name=package_name,
    version='0.0.0',
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
    description='This is a dedicated node for collating odometry from agents and saving the paths that they have travelled.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_logger_node = x3_path_logger.path_logger:main'
        ],
    },
)
