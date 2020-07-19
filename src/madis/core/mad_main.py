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
from madis.utils.logger_utils import create_logger, get_logger
import subprocess
import os.path


def main():
    homedir = os.path.expanduser("~")
    logger_filepath = "{}/.madics/logger-configuration.yaml".format(homedir)
    logger = create_logger("mad_main.main", logger_filepath)
    logger.info("Start MADIS System")
    mad_configuration = read_mad_configuration("{}/.madics/mad-configuration.yaml".format(homedir))
    # Creates ROS publishers
    preload()
    logger.info("I'm creating the ROS-nodes")
    create_ros_nodes("{}/.madics/mad-configuration.yaml".format(homedir))
    while True:
        time.sleep(10)


def create_ros_nodes(mad_configuration_filepath):
    logger = get_logger("mad_main.create_ros_nodes")
    blackboard = BlackBoard()
    mad_initial_configuration = blackboard.get_value("initial_configuration")
    node_configuration = read_ros_node_configuration()
    for single_ros_node in node_configuration:
        logger.debug("I'm creating ROS node: {}".format(single_ros_node))
        script_name = node_configuration[single_ros_node]["script-name"]
        subprocess.Popen(["python", script_name, single_ros_node, mad_configuration_filepath])
        logger.debug("ROS node launched: {}".format(single_ros_node))


def preload():
    blackboard = BlackBoard()
    initial_configuration = blackboard.get_value("initial_configuration")

    command_at_first = initial_configuration["preload"]["command-to-exec-before"]
    if command_at_first is not None:
        splitted_command = command_at_first.split(" ")
        subprocess.Popen(splitted_command)

    if initial_configuration["preload"]["wait-at-boot"] is True:
        time.sleep(initial_configuration["preload"]["seconds-to-wait"])

    command_at_end = initial_configuration["preload"]["command-to-exec-after"]
    if command_at_end is not None:
        splitted_command = command_at_end.split(" ")
        subprocess.Popen(splitted_command)


if __name__ == "__main__":
    main()
