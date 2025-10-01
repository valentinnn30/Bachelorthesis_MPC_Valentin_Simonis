from setuptools import find_packages, setup

package_name = 'nav_tools'

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
            'thruster = nav_tools.thruster:main',
            'ultrasonic = nav_tools.ultrasonic:main',
            'imu = nav_tools.imu:main',
            'statemachine = nav_tools.statemachine:main',
            'pressure = nav_tools.pressure:main',
            'STMcom = nav_tools.STMcom:main',
            'STMcom_mpc = nav_tools.STMcom_mpc:main',
            'orientation_pid = nav_tools.orientation_pid:main',
            'thruster_torque = nav_tools.thruster_torque:main',
            'pid_spin = nav_tools.pid_spin:main',
            'bcu_pid_test = nav_tools.bcu_pid_test:main',
            'bcu_thruster = nav_tools.bcu_thruster:main',
            'distance_bcu_pid = nav_tools.distance_bcu_pid:main',
            'pressure_bcu_pid = nav_tools.pressure_bcu_pid:main'
        ],
    },
)
