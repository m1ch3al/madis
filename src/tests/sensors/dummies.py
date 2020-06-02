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
Tests of dummy sensors.

"""

import unittest
from madis.sensors.dummy import DummySensorGPS
import json


class TestGPSDummyData(unittest.TestCase):
    def test_data(self):
        dummy_gps = DummySensorGPS()
        dummy_gps.initialize_sensor()
        json_data = dummy_gps.read_from_sensor()
        gps_data = json.loads(json_data)
        print(gps_data)
        self.assertEqual(gps_data["latitude"], 44.078855)
        self.assertEqual(gps_data["longitude"], 10.012132)
        self.assertEqual(gps_data["heading"], 23.4)
        self.assertEqual(gps_data["altitude"], 2.5)

