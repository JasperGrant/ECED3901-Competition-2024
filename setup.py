from setuptools import setup
import os
from glob import glob

package_name = 'eced3901_competition_2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jasper Grant',
    maintainer_email='jasper.grant@dal.ca',
    description='A ROS2 package to be used to connect both teams to the UI in the ECED3901 2024 competition',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'competition_publisher = eced3901_competition_2024.competition_publisher:main',
                'pose_listener = eced3901_competition_2024.pose_listener:main',
                'test_student = eced3901_competition_2024.test_student:main',
        ],
},
)
