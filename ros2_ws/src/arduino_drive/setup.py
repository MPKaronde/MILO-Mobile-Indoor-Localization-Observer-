from setuptools import find_packages, setup

package_name = 'arduino_drive'

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
    maintainer='milo',
    maintainer_email='karonde.manav@gmail.com',
    description='Action server for drive commands',
    license='TODO: License declaration',
    test_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={ 
        'console_scripts': [ 
        'drive_action_server = arduino_drive.drive_action_server:main', 
        'drive_action_client = arduino_drive.drive_action_client:main', 
        ], 
    },
)
