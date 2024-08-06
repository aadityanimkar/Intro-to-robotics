from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'simmulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'meshes/iiwa7/collision'), glob('meshes/iiwa7/collision/*.stl')),
        (os.path.join('share', package_name, 'meshes/iiwa7/visual'), glob('meshes/iiwa7/visual/*.stl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaditya',
    maintainer_email='aaditya@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
