from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'cr10_ik_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rawad',
    maintainer_email='rawad@todo.todo',
    description='CR10 IK Project',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'straight_line_ik = cr10_ik_project.straight_line_ik:main',
            'circular_ik = cr10_ik_project.circular_ik:main',
        ],
    },
)
