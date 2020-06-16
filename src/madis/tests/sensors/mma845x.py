from madis.sensors.sensor_mma845x import MMA845XSensor
import unittest
import json


class TestMMA845X(unittest.TestCase):
    def test_data(self):
        mma845x_sensor = MMA845XSensor()
        mma845x_sensor.initialize_sensor()
        json_data = mma845x_sensor.read_from_sensor()
        accelerometer_data = json.loads(json_data)
        print(accelerometer_data)


if __name__ == '__main__':
    unittest.main()
