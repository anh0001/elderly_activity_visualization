from setuptools import setup
import os
from glob import glob

package_name = 'elderly_activity_visualization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Elderly activity visualization using radar charts',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_fetcher = elderly_activity_visualization.data_fetcher:main',
            'activity_processor = elderly_activity_visualization.activity_processor:main',
            'visualization = elderly_activity_visualization.visualization:main',
        ],
    },
)