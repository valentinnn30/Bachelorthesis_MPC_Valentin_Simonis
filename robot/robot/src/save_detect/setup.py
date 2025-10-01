from setuptools import find_packages, setup

package_name = 'save_detect'

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
    maintainer_email='reefranger@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={  # Use this for optional test dependencies
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        	'save_detect = save_detect.save_detect:main'
        #'subscriber_detect = save_detect.subscriber_detect:main'
        	#'publisher_detect = save_detect.publisher_detect:main'
        ],
    },
)
