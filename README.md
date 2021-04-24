# ASMR

Our ASMR project aims to deliver a robot capable of reconaissance, and heat-sesnitive target elimination
by firing a payload at high speed to the target. The core software in this repository eschews the main
features of our product. It is written in Python 3 due to its simplicity in syntax, while still providing
significant customizability to the functionalities of our product.

## Installation

This repository requires the use of the scipy, numpy and matplotlib libraries.
If you have not, refer to the links below to install them:

scipy: https://www.scipy.org/install.html

numpy: https://numpy.org/install/

matplotlib: https://pypi.org/project/matplotlib/

Clone this repository using the following steps:

1.

2.

3.

4.

5.

FInally, Install Python 3 using the command 

After which, you are ready to go!

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






