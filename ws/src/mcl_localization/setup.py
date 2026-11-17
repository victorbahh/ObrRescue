from setuptools import setup

package_name = 'mcl_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mcl_launch.py', 'launch/localization.launch.py',
                              'launch/rescue.launch.py']),
        ('share/' + package_name + '/config', ['config/slam.yaml', 'config/navigation.yaml',
                              'config/navigate.xml']),
        ('share/' + package_name + '/maps', ['maps/obr_world.yaml', 'maps/obr_world.pgm']),
        ('share/' + package_name + '/rviz', ['rviz/localization.rviz']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='OBR',
    maintainer_email='user@todo.todo',
    description='Monte Carlo Localization for HeRo in OBR rescue room.',
    license='CC-BY-NC-SA-4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcl_node = mcl_localization.mcl_node:main',
            'victim_node = mcl_localization.victim_node:main',
            'mission_node = mcl_localization.mission_node:main',
            'world_map = mcl_localization.world_map:main',
        ],
    },
)
