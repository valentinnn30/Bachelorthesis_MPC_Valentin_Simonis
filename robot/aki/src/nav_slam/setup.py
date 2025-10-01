from setuptools import find_packages, setup

package_name = 'nav_slam'

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
    maintainer='reefranger',
    maintainer_email='48968873+Zundii@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid = nav_slam.pid:main',
            'mpc_tvp = nav_slam.mpc_tvp:main',
            'mpc = nav_slam.mpc:main',
            'path = nav_slam.path:main',
            'grid = nav_slam.grid:main',
            'reset = nav_slam.reset:main',
            'pid_test = nav_slam.pid_test:main',
            'changepath = nav_slam.changepath:main',
            'feeding = nav_slam.feeding:main',
            'image_filter = nav_slam.image_filter:main',
            'image_filter2 = nav_slam.image_filter2:main',
            'image_filter3 = nav_slam.image_filter3:main',
            'point_filter = nav_slam.point_filter:main',
        ],
    },
)
