from madis.sensors.sensor_bn880 import BN880Sensor
import unittest
import json
import os
import time


class TestBN880Sensor(unittest.TestCase):
    def test_data(self):
        cycles = 1000
        counter = 0
        bn880_sensor = BN880Sensor()
        bn880_sensor.initialize_sensor()
        while counter < cycles:
            os.system("clear")
            json_data = bn880_sensor.read_from_sensor()
            gps_data = json.loads(json_data)
            print(gps_data)
            counter += 1
            time.sleep(0.5)


if __name__ == '__main__':
    unittest.main()
