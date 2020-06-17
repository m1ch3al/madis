"""

##     ##        ###        ########
###   ###       ## ##       ##     ##
#### ####      ##   ##      ##     ##
## ### ##     ##     ##     ##     ##
##     ##     #########     ##     ##
##     ## ### ##     ## ### ##     ## ###
##     ## ### ##     ## ### ########  ###

MAD - M1ch3al Autonomous Drone
This class belong to MADIS (M1ch3al Autonomous Drone Internal System)

       Author: SIROLA RENATO
Creation Date: 2020-06-02
       E-mail: renato.sirola@gmail.com

Content:
Main application entry point (inside the vehicle)

"""

from madis.utils.conf_utils import *
import time
from madis.utils.blackboard import BlackBoard
import subprocess
import os.path


def main():
    homedir = os.path.expanduser("~")
    mad_configuration = read_mad_configuration("{}/.madics/mad-configuration.yaml".format(homedir))
    # Creates ROS publishers
    preload()
    create_ros_nodes("/home/{}/.madics/mad-configuration.yaml".format(homedir))
    while True:
        time.sleep(10)


def create_ros_nodes(mad_configuration_filepath):
    blackboard = BlackBoard()
    mad_initial_configuration = blackboard.get_value("initial_configuration")
    node_configuration = read_ros_node_configuration()
    for single_ros_node in node_configuration:
        script_name = node_configuration[single_ros_node]["script-name"]
        subprocess.Popen(["python", script_name, single_ros_node, mad_configuration_filepath])


def preload():
    blackboard = BlackBoard()
    initial_configuration = blackboard.get_value("initial_configuration")
    if initial_configuration["preload"]["wait-at-boot"] is True:
        time.sleep(initial_configuration["preload"]["seconds-to-wait"])


if __name__ == "__main__":
    main()
