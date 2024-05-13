import os
from setuptools import setup
from glob import glob

package_name = 'robot_navigation'
submodules = 'robot_navigation/model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),     
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')), # Path to the launch file 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomi',
    maintainer_email='tomi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train = robot_navigation.train:main',
            'test = robot_navigation.test:main'
        ],
    },
)