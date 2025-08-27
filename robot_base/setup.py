from setuptools import setup
import os
from glob import glob

package_name = 'robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': ''},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/base.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego',
    maintainer_email='diego@todo.todo',
    description='Nodo Python que controla la base diferencial y publica odometr\EDa + TF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = robot_base.base_driver:main',
        ],
    },
)

