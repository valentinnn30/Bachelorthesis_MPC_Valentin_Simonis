from setuptools import find_packages, setup

package_name = 'nav_yolo'

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
            'pid = nav_yolo.pid:main',
            'mpc = nav_yolo.mpc:main',
            'tables = nav_yolo.tables:main',
            'pid_nothresholdtimer = nav_yolo.pid_nothresholdtimer:main',
        ],
    },
)
