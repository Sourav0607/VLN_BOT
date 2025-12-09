from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'vision_language_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Worlds
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.*')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.*')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.*')),
        # Maps
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.*')),
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sourav',
    maintainer_email='sourav.hawaldar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'person_detector = vision_language_nav.person_detector:main',
            'person_navigator = vision_language_nav.person_navigator:main',
            'llam_parser = vision_language_nav.llm_command_parser:main',
        ],
    },
)
