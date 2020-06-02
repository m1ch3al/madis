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
