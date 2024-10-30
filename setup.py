from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'elderly_activity_visualization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Include configuration files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anhar',
    maintainer_email='anhrisn@gmail.com',
    description='Elderly activity visualization for NCGG',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_fetcher=elderly_activity_visualization.data_fetcher:main',
            'activity_processor=elderly_activity_visualization.activity_processor:main',
            'visualization=elderly_activity_visualization.visualization:main',
        ],
    },
)
