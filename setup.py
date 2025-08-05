from setuptools import setup
import os
# import glob
from glob import glob
from setuptools import find_packages





package_name = 'youbot'
resource_file = os.path.join('resource', package_name)

# Ensure the resource marker file exists
if not os.path.exists(resource_file):
    os.makedirs('resource', exist_ok=True)
    open(resource_file, 'a').close()

data_files = [
    # ROS 2 package index
    ('share/ament_index/resource_index/packages', [resource_file]),
    ('share/' + package_name, ['package.xml']),
    # ('share/' + package_name +'/srv/', ['/srv/SrvString.srv']),
    # ('share/' + package_name +'/srv/', glob('srv/*.srv')),
    # ('share/' + package_name, glob('srv/*.srv')),
    # (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
]

# Dynamically include all files from launch/, description/, config/, meshes/
for folder in ['launch', 'description', 'config','worlds','views','src']:
    files = glob(f'{folder}/**', recursive=True)
    files = [f for f in files if os.path.isfile(f)]  # Only include files, not directories
    if files:
        data_files.append((f'share/{package_name}/{folder}', files))

for folder in ['meshes/sensors','meshes/youbot_arm','meshes/youbot_base','meshes/youbot_gripper','meshes/youbot_plate',]:
    files = glob(f'{folder}/**', recursive=True)
    files = [f for f in files if os.path.isfile(f)]  # Only include files, not directories
    if files:
        data_files.append((f'share/{package_name}/{folder}', files))




setup(
    name=package_name,
    version='0.1.0',
    # packages=[package_name],
    packages=find_packages(include=[package_name]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Python-based ROS 2 package for YouBot robot platform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blinker_node = src.blinker:main',
            'relay_control_node = src.RelayControl:main',
            'log_node = src.MessageLogger:main',
            'simreal_selector = src.SimRealSelector:main',
            'opcua_client_node = src.OpcUaClient:main',
            'ro_bridge = src.ROBridge:main',
            'heart_beat_node = src.HeartBeater:main',
            'statemachine = src.StateController:main',
            'astolfi = src.AstolfiController:main',
            'lcddisplay = src.LCDDisplay:main',
            'vmeter = src.VoltageSensor:main'
            

        ],
    
    }
)
