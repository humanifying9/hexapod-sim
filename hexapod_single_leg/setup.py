from setuptools import setup, find_packages

package_name = 'hexapod_single_leg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.lib', package_name + '.lib.*']),
    py_modules=[],
    data_files=[
        # Install the URDF and launch files so Gazebo can find them
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf',
            ['urdf/hexapod_leg.urdf.xacro']),
        ('share/' + package_name + '/launch',
            ['launch/gazebo.launch.py']),
        ('share/' + package_name + '/config',
            ['config/hexapod_single_leg.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tia_parekh',
    maintainer_email='tiaparekh9@gmail.com',
    description='ROS 2 package for simulating and controlling a single hexapod leg in Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This maps "leg_controller" command → hexapod_single_leg.controller:main
            'leg_controller = hexapod_single_leg.controller:main',
        ],
    },
)
