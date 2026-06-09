from setuptools import setup

package_name = 'deltaflex_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='IK/FK mathematics and service node for DeltaFlex robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ik_service_node = deltaflex_kinematics.ik_service_node:main',
        ],
    },
)
