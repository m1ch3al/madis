from madis.sensors.sensor_mma845x import MMA845XSensor
import unittest
import json
import os
import time


class TestMMA845X(unittest.TestCase):
    def test_data(self):
        cycles = 1000
        counter = 0
        mma845x_sensor = MMA845XSensor()
        mma845x_sensor.initialize_sensor()
        while counter < cycles:
            os.system("clear")
            json_data = mma845x_sensor.read_from_sensor()
            accelerometer_data = json.loads(json_data)
            print(accelerometer_data)
            counter += 1
            time.sleep(0.1)


if __name__ == '__main__':
    unittest.main()
