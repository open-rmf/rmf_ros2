from setuptools import setup, find_packages

package_name = 'rmf_charging_schedule'

setup(
    name=package_name,
    version='2.7.2',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grey',
    maintainer_email='mxgrey@intrinsic.ai',
    description='A node that manages a fixed schedule for robot charger usage',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'charging_schedule=rmf_charging_schedule.main:main',
        ],
    },
)
