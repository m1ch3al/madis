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
import threading
from madis.core.publishers import *
import os


def main():
    homedir = os.path.expanduser("~")
    logger_filepath = "{}/.madics/logger-configuration.yaml".format(homedir)
    logger = create_logger("mad_main.main", logger_filepath)
    logger.info("Start MADIS System")
    mad_configuration = read_mad_configuration("{}/.madics/mad-configuration.yaml".format(homedir))
    # Creates ROS publishers
    preload()
    logger.info("I'm creating the ROS-nodes")
    SHARED_DATA = create_shared_data()
    create_sensors_reader(mad_configuration, SHARED_DATA)
    time.sleep(3)
    while True:
        print_values(SHARED_DATA)
        time.sleep(0.5)


def create_shared_data():
    SHARED_DATA = dict()
    SHARED_DATA["gps"] = None
    SHARED_DATA["environmental"] = None
    SHARED_DATA["inclinometer"] = None
    return SHARED_DATA


def create_sensors_reader(mad_configuration, SHARED_DATA):
    logger = get_logger("mad_main.create_ros_nodes")
    for single_sensor_configuration in mad_configuration["sensors"]:
        # Sensor : GPS
        if single_sensor_configuration == "gps":
            required_sensor_configuration = mad_configuration["sensors"][single_sensor_configuration]
            logger.info("GPS sensor configuration : LOADED")
            gps_publisher_thread = threading.Thread(target=start_gps_publisher, args=(required_sensor_configuration, SHARED_DATA, ))
            gps_publisher_thread.setDaemon(True)
            gps_publisher_thread.start()
            logger.info("GPS publisher thread : STARTED")

        # Sensor : INCLINOMETER (Gyro+Accelerometer)
        if single_sensor_configuration == "inclinometer":
            required_sensor_configuration = mad_configuration["sensors"][single_sensor_configuration]
            logger.info("GYRO+ACCELEROMETER sensor(s) configuration : LOADED")
            gps_publisher_thread = threading.Thread(target=start_stabilizer_publisher, args=(required_sensor_configuration, SHARED_DATA, ))
            gps_publisher_thread.setDaemon(True)
            gps_publisher_thread.start()
            logger.info("STABILIZER publisher thread : STARTED")

        # Sensor : ENVIRONMENTAL
        if single_sensor_configuration == "environmental":
            required_sensor_configuration = mad_configuration["sensors"][single_sensor_configuration]
            logger.info("ENVIRONMENTAL sensor(s) configuration : LOADED")
            gps_publisher_thread = threading.Thread(target=start_environmental_publisher, args=(required_sensor_configuration, SHARED_DATA, ))
            gps_publisher_thread.setDaemon(True)
            gps_publisher_thread.start()
            logger.info("ENVIRONMENTAL publisher thread : STARTED")


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
