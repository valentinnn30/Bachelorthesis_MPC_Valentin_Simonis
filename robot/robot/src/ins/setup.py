from setuptools import find_packages, setup

package_name = 'ins'

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
    license='Apache-2.0',
    extras_require={  # Use this for optional test dependencies
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ins = ins.ins:main'
        ],
    },
)
