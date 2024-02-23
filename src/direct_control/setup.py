from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'direct_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rivaldi',
    maintainer_email='rivaldi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'actuator = direct_control.actuator:main',
                'sensor = direct_control.sensor:main',

                'asdasd = direct_control.asdasd:main',
                'motor = direct_control.motor:main',
        ],
    },
)
