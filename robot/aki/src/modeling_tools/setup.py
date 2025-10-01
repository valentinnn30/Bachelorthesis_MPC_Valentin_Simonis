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
            'transform_inverse = modeling_tools.transform_inverse:main',
            'multiple_inverse = modeling_tools.multiple_inverse:main',
            'tag12transforms = modeling_tools.tag12transforms:main',
            'savetocsv = modeling_tools.savetocsv:main',
            'robotpos = modeling_tools.robotpos:main',
            'robotpos2 = modeling_tools.robotpos2:main',

        ],
    },
)
