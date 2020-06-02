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
A dummy set of sensors for simulate data inside the DRONE.

"""

from madis.sensors.sensor import Sensor
import json


class DummySensorGPS(Sensor):
    def __init__(self):
        Sensor.__init__(self)
        self._altitude = None
        self._latitude = None
        self._longitude = None
        self._heading = None
        self._data = {"latitude": None, "longitude": None, "altitude": None, "heading": None}

    def initialize_sensor(self):
        self._altitude = 2.5            # meters
        self._latitude = 44.078855      # wgs84
        self._longitude = 10.012132     # wgs84
        self._heading = 23.4            # degrees

    def read_from_sensor(self):
        self._prepare_dummy_data()
        json_data = json.dumps(self._data)
        return json_data

    def _prepare_dummy_data(self):
        self._data["latitude"] = self._latitude
        self._data["longitude"] = self._longitude
        self._data["altitude"] = self._altitude
        self._data["heading"] = self._heading


class DummySensorTempPressureAltitude(Sensor):
    def __init__(self):
        Sensor.__init__(self)
        self._altitude = None
        self._pressure = None
        self._temperature = None
        self._data = {"altitude": None, "pressure": None, "temperature": None}

    def initialize_sensor(self):
        self._altitude = 2.3           # meters
        self._pressure = 1000          # hPa (hectopascals)
        self._temperature = 27.4       # degrees

    def read_from_sensor(self):
        self._prepare_dummy_data()
        json_data = json.dumps(self._data)
        return json_data

    def _prepare_dummy_data(self):
        self._data["altitude"] = self._altitude
        self._data["pressure"] = self._pressure
        self._data["temperature"] = self._temperature


class DummySensorSonar(Sensor):
    def __init__(self):
        Sensor.__init__(self)
        self._distance = None
        self._data = {"distance": None}

    def initialize_sensor(self):
        self._distance = 150           # centimeters

    def read_from_sensor(self):
        self._prepare_dummy_data()
        json_data = json.dumps(self._data)
        return json_data

    def _prepare_dummy_data(self):
        self._data["distance"] = self._distance


class DummySensorAccelGyro(Sensor):
    def __init__(self):
        Sensor.__init__(self)
        self._acc_x = None
        self._acc_y = None
        self._acc_z = None
        self._gyro_x = None
        self._gyro_y = None
        self._gyro_z = None
        self._data = {"acceleration_x": None, "acceleration_y": None, "acceleration_z": None,
                      "gyroscope_x": None, "gyroscope_y": None, "gyroscope_z": None}

    def initialize_sensor(self):
        self._acc_x = 0.34234
        self._acc_y = 0.57432
        self._acc_z = 0.35123
        self._gyro_x = 0.3214132
        self._gyro_y = 0.52352
        self._gyro_z = 0.023423

    def read_from_sensor(self):
        self._prepare_dummy_data()
        json_data = json.dumps(self._data)
        return json_data

    def _prepare_dummy_data(self):
        self._data["acceleration_x"] = self._acc_x
        self._data["acceleration_y"] = self._acc_y
        self._data["acceleration_z"] = self._acc_z
        self._data["gyroscope_x"] = self._gyro_x
        self._data["gyroscope_y"] = self._gyro_y
        self._data["gyroscope_z"] = self._gyro_z
