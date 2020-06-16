from madis.sensors.sensor_bmp280 import BME280Sensor
import unittest
import json


class TestBME280(unittest.TestCase):
    def test_data(self):
        bme280_sensor = BME280Sensor()
        bme280_sensor.initialize_sensor()
        json_data = bme280_sensor.read_from_sensor()
        gps_data = json.loads(json_data)
        print(gps_data)

