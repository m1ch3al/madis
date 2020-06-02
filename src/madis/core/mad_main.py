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
import importlib
import _thread
import time
from madis.ros.pub_ros_navigation import navigation_node


def main():
    mad_configuration = read_mad_configuration("/home/micheal/.madics/mad-configuration.yaml")

    # Creates sensors readers
    sensor_readers = create_sensors_readers(mad_configuration)

    # Creates ROS publisher
    navigation_node(sensor_readers["gps"])
    while True:
        time.sleep(10)


def create_sensors_readers(mad_configuration):
    sensor_readers = {}
    for element in mad_configuration["sensors"]:
        module_name = mad_configuration["sensors"][element]["driver-module"]
        class_name = mad_configuration["sensors"][element]["driver-class"]
        sensor_reader = instantiate_class(module_name, class_name)
        sensor_reader.initialize_sensor()
        sensor_readers[mad_configuration["sensors"][element]["configuration-name"]] = sensor_reader
    return sensor_readers


def instantiate_class(module_name, class_name, constructor_parameters=[]):
    try:
        module = importlib.import_module(module_name)
        class_ = getattr(module, class_name)
        class_instantiated = class_(*constructor_parameters)
        return class_instantiated
    except Exception as ex:
        message = "Introspection Exception : try to create this class [{}] in this module : [{}]".format(class_name, module_name)
        message += "Cannot instantiate the requested object class, cause: {}".format(str(ex))
        raise Exception(message)


if __name__ == "__main__":
    main()

