from setuptools import find_packages, setup

package_name = 'nav_locator'

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
            'pid = nav_locator.pid:main',
            'waterlinked = nav_locator.waterlinked:main',
            'worldframe = nav_locator.worldframe:main'
        ],
    },
)
