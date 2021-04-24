# ASMR

Our ASMR project aims to deliver a robot capable of reconaissance, and heat-sesnitive target elimination
by firing a payload at high speed to the target. The core software in this repository eschews the main
features of our product. It is written in Python 3 due to its simplicity in syntax, while still providing
significant customizability to the functionalities of our product.

## Installation

Please ensure that you have Python3 installed, if not, please do so.
This repository requires the use of the scipy, numpy and matplotlib libraries.
If you have not, refer to the links below to install them:

scipy: https://www.scipy.org/install.html

numpy: https://numpy.org/install/

matplotlib: https://pypi.org/project/matplotlib/


This repository consists of the programs and files required to operate the ASMR robot using the ROS2 environment.

To install the programs:

1. Create a new ROS2 workspace by following this guide: https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html, but name the workspace asmr_ws.

2. Create a new package by folowing this guide: https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html, but name the package auto_nav.

3. Navigate to the asmr_ws/src/auto_nav directory, and replace the setup.py and package.xml files with the ones in the repository.

4. Navigate to the asmr_ws/src/auto_nav/auto_nav directory, and place all program files except nav_launcher.py, recon_launcher.py, and setup_Rpi.py into the folder.

5. Navigate to your home directory and run 
'''colcon build''' 
to complete the set up process.

## Usage

Open 4 terminal windows.

In the first window,
'''ssh ubuntu@ ''' OR '''ssh ubuntu@'''(Backup Raspberry Pi)
'''rosbu'''

In the second window,
'''colcon build'''(only necessary if you have made changes to any code)
'''cd'''
'''rslam'''

In the third window,
'''ros2 run pypubsub talker'''

In the fourth window,
'''ros2 run auto_nav (name of python file) '''

## Calibration
Given the flexibility presented by the code, calibration and optimization will depend on
the conditions of the user. Sensitive variables are highlighted in the documentation of the
relevant code.

## Contributions
Please centre your contributions, if possible, along the following areas.
1. Best calibrated values for highest power efficiency
2. Improved optimizations for mapping algorithm
3. Smoother overall motion(Continuous motion rather than stop-start motion in current model)
Do avoid pull requests for cosmetic fixes


## License
[BSD] 3 clause licence






