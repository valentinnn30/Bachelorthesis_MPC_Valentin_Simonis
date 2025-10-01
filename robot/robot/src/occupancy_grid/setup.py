from setuptools import find_packages, setup

package_name = 'occupancy_grid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),  # Include the launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reefranger',
    maintainer_email='klaus.moeri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={  # Use this for optional test dependencies
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'occupancy_grid = occupancy_grid.occupancy_grid:main',
            'own_grid = occupancy_grid.own_grid:main',
            'reset_cloudpoints = occupancy_grid.reset_cloudpoints:main',
        ],
    },
)
