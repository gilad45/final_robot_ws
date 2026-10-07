from setuptools import find_packages, setup

package_name = 'robot_logic'

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
    maintainer='gilad',
    maintainer_email='giladberkove10@gmail.com',
    description='this pkg is responsible for the robot logic',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'StateMachineNode = robot_logic.state_machine:main',
        ],
    },
)
