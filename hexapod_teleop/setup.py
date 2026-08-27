from setuptools import setup, find_packages

package_name = 'hexapod_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/keyboard_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tia_parekh',
    maintainer_email='tiaparekh9@gmail.com',
    description='Manual /cmd_vel input sources for the hexapod',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = hexapod_teleop.keyboard_teleop:main',
        ],
    },
)
