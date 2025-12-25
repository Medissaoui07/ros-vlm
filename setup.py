from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros_vlm'

setup(
    name=package_name,
    version='0.2.0',
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
    description='A ROS2 agent using MCP protocol for tool management',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcp_agent = ros_vlm.mcp_agent_node:main',
            'mcp_server = ros_vlm.mcp_server:main',
        ],
    },
)
