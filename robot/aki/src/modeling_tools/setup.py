from setuptools import find_packages, setup

package_name = 'modeling_tools'

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
    maintainer_email='klaus.moeri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multiple_inverse = modeling_tools.multiple_inverse:main',
            'savetocsv = modeling_tools.savetocsv:main',
            'robotpos = modeling_tools.robotpos:main',
        ],
    },
)
