import os
import time

from madis.utils.introspection import instantiate_class
from terminaltables import AsciiTable


def start_gps_publisher(sensor_configuration, shared_data):
    module_name = sensor_configuration["driver-module"]
    class_name = sensor_configuration["driver-class"]
    sensor_reader = instantiate_class(module_name, class_name)
    sensor_reader.initialize_sensor()
    interval = 1/sensor_configuration["frequency"]
    while True:
        data = sensor_reader.read_from_sensor()
        shared_data["gps"] = data
        time.sleep(interval)


def start_stabilizer_publisher(sensor_configuration, shared_data):
    module_name = sensor_configuration["driver-module"]
    class_name = sensor_configuration["driver-class"]
    sensor_reader = instantiate_class(module_name, class_name)
    sensor_reader.initialize_sensor()
    interval = 1/sensor_configuration["frequency"]
    while True:
        data = sensor_reader.read_from_sensor()
        shared_data["inclinometer"] = data
        time.sleep(interval)


def start_environmental_publisher(sensor_configuration, shared_data):
    module_name = sensor_configuration["driver-module"]
    class_name = sensor_configuration["driver-class"]
    sensor_reader = instantiate_class(module_name, class_name)
    sensor_reader.initialize_sensor()
    interval = 1/sensor_configuration["frequency"]
    while True:
        data = sensor_reader.read_from_sensor()
        shared_data["environmental"] = data
        time.sleep(interval)


def print_values(shared_data):
    os.system("clear")
    gps_data = [
        ["Time", "Latitude", "Longitude", "Altitude", "Heading"],
        [shared_data["gps"]["time"],
         shared_data["gps"]["latitude"],
         shared_data["gps"]["longitude"],
         shared_data["gps"]["altitude"],
         shared_data["gps"]["heading"]]
    ]
    gps_table = AsciiTable(gps_data)
    gps_table.title = "GPS data from sensor"
    gps_table.padding_left = 3
    gps_table.outer_border = 2
    print("{}\n".format(gps_table.table))

    environmental_data = [
        ["Time", "altitude", "pressure", "temperature"],
        [shared_data["environmental"]["time"],
         shared_data["environmental"]["altitude"],
         shared_data["environmental"]["pressure"],
         shared_data["environmental"]["temperature"]]
    ]
    env_table = AsciiTable(environmental_data)
    env_table.title = "ENVIRONMENTAL data from sensor"
    env_table.padding_left = 3
    env_table.outer_border = 2
    print("{}\n".format(env_table.table))

    stabilization_data = [
        ["Time", "acc(X)", "acc(Y)", "acc(Z)"],
        [shared_data["inclinometer"]["time"],
         shared_data["inclinometer"]["acceleration_x"],
         shared_data["inclinometer"]["acceleration_y"],
         shared_data["inclinometer"]["acceleration_z"]]
    ]
    stabilization_table = AsciiTable(stabilization_data)
    stabilization_table.title = "GYRO/ACCELERATION data from sensor"
    stabilization_table.padding_left = 3
    stabilization_table.outer_border = 2
    print("{}\n".format(stabilization_table.table))
