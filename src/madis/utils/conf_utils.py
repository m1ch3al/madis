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


def read_mad_configuration(main_mad_configuration_file):
    mad_configuration = {}
    with open(main_mad_configuration_file, "r") as yaml_file_descriptor:
        initial_configuration = yaml.safe_load(yaml_file_descriptor)
    yaml_file_descriptor.close()
    blackboard = BlackBoard()
    blackboard.insert_value("initial_configuration", initial_configuration, 0)
    sensors_configuration = read_sensors_configuration(main_mad_configuration_file, initial_configuration)
    mad_configuration["sensors"] = sensors_configuration
    ros_publishers = read_ros_node_configuration(initial_configuration)
    return mad_configuration


def read_sensors_configuration(main_mad_configuration_file, initial_configuration):
    mad_configuration = {}
    for element in initial_configuration["sensors-configuration"]:
        full_path_sensor_configuration = os.path.join(os.path.dirname(main_mad_configuration_file), initial_configuration["sensors-configuration"][element])
        with open(full_path_sensor_configuration, "r") as yaml_file_descriptor:
            sensor_configuration = yaml.safe_load(yaml_file_descriptor)
            mad_configuration[sensor_configuration["configuration-name"]] = sensor_configuration
    yaml_file_descriptor.close()
    return mad_configuration


def read_ros_node_configuration(initial_configuration=None):
    if initial_configuration is None:
        blackboard = BlackBoard()
        initial_configuration = blackboard.get_value("initial_configuration")
    return initial_configuration["ros-nodes"]
