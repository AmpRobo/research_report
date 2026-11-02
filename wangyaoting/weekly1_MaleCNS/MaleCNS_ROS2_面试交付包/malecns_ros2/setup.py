from glob import glob
from setuptools import setup

package_name = 'malecns_robot'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    package_data={package_name: ['data/*.npz', 'data/*.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('malecns_robot/launch/*.py')),
        ('share/' + package_name + '/worlds', glob('malecns_robot/worlds/*')),
        ('share/' + package_name + '/models', glob('malecns_robot/models/*')),
        ('share/' + package_name + '/rviz', glob('malecns_robot/rviz/*')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='Local demo', maintainer_email='maintainer@example.com',
    description='MaleCNS connectome based mobile robot controller', license='MIT',
    entry_points={'console_scripts': ['controller = malecns_robot.controller:main']},
)
