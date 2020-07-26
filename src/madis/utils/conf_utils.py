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

    # Read sensors configuration and copy it inside the mad_configuration dict
    sensors_configuration = read_sensors_configuration(main_mad_configuration_file, initial_configuration)
    mad_configuration["sensors"] = sensors_configuration

    # Read motors configuration and copy it inside the mad_configuration dict
    motors_configuration = read_motors_configuration(main_mad_configuration_file, initial_configuration)
    mad_configuration["motors"] = motors_configuration

    network_configuration = read_network_configuration(initial_configuration)
    mad_configuration["network"] = network_configuration

    logger.debug("I'm finished to read the drone configuration")
    return mad_configuration


def read_network_configuration(initial_configuration):
    network_configuration = dict()
    data = initial_configuration["network-configuration"]
    for element in data:
        sensor_id = element["udp-server-configuration"]["sensor"]
        port = element["udp-server-configuration"]["port"]
        interval = element["udp-server-configuration"]["interval"]
        network_configuration[sensor_id] = dict()
        network_configuration[sensor_id]["bind_port"] = port
        network_configuration[sensor_id]["frequency"] = round((1/interval), 3)
    return network_configuration


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


def read_motors_configuration(main_mad_configuration_file, initial_configuration):
    logger = get_logger("conf_utils.read_motors_configuration")
    mad_configuration = {}
    logger.debug("Loading SENSOR(S) configuration")
