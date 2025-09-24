# Import necessary libraries
import os
from glob import glob
from setuptools import setup

# Define the name of your ROS 2 package
package_name = 'obs_detection'

setup(
    name=package_name,
    version='0.0.1',
    
    # This is a list of where to install non-Python files
    data_files=[
        # Boilerplate to make the package discoverable
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # *** IMPORTANT ***
        # Find and install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Find and install all world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    
    # This is where you define your Python executables
    entry_points={
        'console_scripts': [
            # Creates an executable named 'scanner' that runs the main function
            # from the 'scanner.py' file inside the 'obs_detection' module folder.
            'scanner = obs_detection.scanner:main',
        ],
    },
)