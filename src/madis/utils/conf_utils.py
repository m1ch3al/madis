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
A configuration system for MAD

"""

import yaml
import os
from madis.utils.blackboard import BlackBoard
from madis.utils.logger_utils import get_logger


def read_mad_configuration(main_mad_configuration_file):
    logger = get_logger("conf_utils.read_mad_configuration")
    mad_configuration = {}
    logger.debug("reading configuration [{}]".format(main_mad_configuration_file))
    with open(main_mad_configuration_file, "r") as yaml_file_descriptor:
        initial_configuration = yaml.safe_load(yaml_file_descriptor)
    yaml_file_descriptor.close()
    blackboard = BlackBoard()
    blackboard.insert_value("initial_configuration", initial_configuration, 0)
    sensors_configuration = read_sensors_configuration(main_mad_configuration_file, initial_configuration)
    mad_configuration["sensors"] = sensors_configuration
    ros_publishers = read_ros_node_configuration(initial_configuration)
    logger.debug("I'm finished to read the drone configuration")
    return mad_configuration


def read_sensors_configuration(main_mad_configuration_file, initial_configuration):
    logger = get_logger("conf_utils.read_sensors_configuration")
    mad_configuration = {}
    logger.debug("Loading SENSOR(S) configuration")
    for element in initial_configuration["sensors-configuration"]:
        full_path_sensor_configuration = os.path.join(os.path.dirname(main_mad_configuration_file), initial_configuration["sensors-configuration"][element])
        with open(full_path_sensor_configuration, "r") as yaml_file_descriptor:
            sensor_configuration = yaml.safe_load(yaml_file_descriptor)
            mad_configuration[sensor_configuration["configuration-name"]] = sensor_configuration
            logger.debug("Reading sensor configuration -> type: {} - driver: {}".format(sensor_configuration["configuration-name"], sensor_configuration["driver-class"]))
    yaml_file_descriptor.close()
    return mad_configuration


def read_ros_node_configuration(initial_configuration=None):
    if initial_configuration is None:
        blackboard = BlackBoard()
        initial_configuration = blackboard.get_value("initial_configuration")
    return initial_configuration["ros-nodes"]
