from setuptools import find_packages, setup

package_name = 'x3_covariance_filter'

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
    description='Attaches tunable covariance to specified topics',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "covariance_filter_node = x3_covariance_filter.covariance_filter:main",
        ],
    },
)
