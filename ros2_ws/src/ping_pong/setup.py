from setuptools import setup

package_name = 'ping_pong'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',  # replace with your name
    maintainer_email='YOUR_EMAIL',  # replace with your email
    description='A ping-pong ROS2 package using std_srvs/Trigger',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dt_control = ping_pong.dt_control:main',
            'main_control = ping_pong.main_control:main',
        ],
    },

)
