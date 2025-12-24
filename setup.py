from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros_vlm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ken2',
    maintainer_email='mohamedissaoui2468@gmail.com',
    description='A generic ROS2 VLM agent for robot interaction',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = ros_vlm.agent_node:main',
            'cam_publisher = ros_vlm.cam:main',
            'speech_to_text = ros_vlm.speech_to_text:main',
            'tts = ros_vlm.tts:main',
        ],
    },
)
