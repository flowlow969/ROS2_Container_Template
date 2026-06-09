from setuptools import setup

package_name = 'deltaflex_controllers'

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
    description='Cartesian-space control node for DeltaFlex robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cartesian_controller = deltaflex_controllers.cartesian_controller_node:main',
        ],
    },
)
