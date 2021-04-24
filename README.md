# ASMR

Our ASMR project aims to deliver a robot capable of reconaissance, and heat-sesnitive target elimination
by firing a payload at high speed to the target. The core software in this repository eschews the main
features of our product. It is written in Python 3 due to its simplicity in syntax, while still providing
significant customizability to the functionalities of our product.

The programs and features created for the ASMR are done within the Robot Operating Software 2 foxy environment. Before use of the ASMR, ensure that you have a fundamental understanding of the ROS2 environment. For more information, visit the ROS2 documentation here: https://docs.ros.org/en/foxy/index.html.

## Installation

Please ensure that you have Python3 installed, if not, please do so.
This repository requires the use of the scipy, numpy and matplotlib libraries.
If you do not have them, refer to the guides below for installation:

scipy: https://www.scipy.org/install.html

numpy: https://numpy.org/install/

matplotlib: https://pypi.org/project/matplotlib/


This repository consists of the programs and files required to operate the ASMR robot using the ROS2 environment.

To set up the ROS master:

1. Create a new ROS2 workspace by following this guide: https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html, but name the workspace asmr_ws.

2. Create a new package by folowing this guide: https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html, but name the package auto_nav.

3. Navigate to the asmr_ws/src/auto_nav directory, and replace the setup.py and package.xml files with the ones in the repository.

4. Navigate to the asmr_ws/src/auto_nav/auto_nav directory, and place all program files except nav_launcher.py, recon_launcher.py, and setup_Rpi.py into the folder.

5. Navigate to your home directory and run 
'''colcon build''' 
to complete the set up process on the ROS master.


To set up the Raspberry Pi on the ASMR:

1. Create a new package within the turltebot_3 ws by folowing this guide: https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html, but name the package launcher.

2. Navigate to the turlebot3_ws/src/launcher directory, and replace the setup.py file with the setup_Rpi.py file in the repository.

3. Navigate to the asmr_ws/src/auto_nav/auto_nav directory, and place the files nav_launcher.py, recon_launcher.py, and setup_Rpi.py into the folder.

4. Navigate to your home directory and run 
'''colcon build''' 
to complete the set up process on the Raspberry Pi.

## Usage

The ASMR comes with 2 main modes of operation, Autonomous Navigation and Mapping as well as Reconnaissance and Firing mode. We have included other basic functionalities as well. 

For a complete guide on each of the 2 modes and how to use them, please refer to the ASMR Hardware Design documentation. 

## Calibration
Given the flexibility presented by the code, calibration and optimization will depend on
the conditions of the user. Sensitive variables are highlighted in the documentation of the
relevant code.

## Contributions
Please centre your contributions, if possible, along the following areas.
1. Best calibrated values for highest power efficiency
2. Improved optimizations for mapping algorithm
3. Smoother overall motion(Motion in an arc rather than straight line and rotate motion in the current model)
Do avoid pull requests for cosmetic fixes


## License
[Apache-2.0] Apache License Version 2.0






