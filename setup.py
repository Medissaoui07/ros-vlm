from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros_vlm'      # Python module name (matches src/ros_vlm/ros_vlm/)
ros_package_name = 'ros_vlm'  # ROS 2 package name (matches package.xml <name> and src/ros_vlm/)

setup(
    name=ros_package_name, # Use 'ros_vlm' here
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']), # This finds the 'ros_vlm' Python module
     data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + ros_package_name]),
        ('share/' + ros_package_name, ['package.xml']),
        (os.path.join('share', ros_package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
        # Updated config file installation
        (os.path.join('share', ros_package_name, 'config'),
         glob('config/*.yaml')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ken2',
    maintainer_email='mohamedissaoui2468@gmail.com',
    description='A ROS 2 camera application for VLM.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_publisher = ros_vlm.cam:main',        
            'image_viewer = ros_vlm.image_viewer:main', 
            'vlm_node = ros_vlm.vlm_node:main',
            'speech_to_text = ros_vlm.speech_to_text:main',
            'agent_node = ros_vlm.agent_node:main',
            'tts = ros_vlm.tts:main',
            
        ],
    },
)
