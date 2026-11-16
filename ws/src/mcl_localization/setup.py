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
        ('share/' + package_name + '/launch', ['launch/mcl_launch.py']),
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
        ],
    },
)
