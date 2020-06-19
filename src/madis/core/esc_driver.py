import pigpio
import time


class ESCDriver(object):
    def __init__(self, gpio_pin, max_value, min_value):
        self._gpio_pin = gpio_pin
        self._min_value = min_value
        self._max_value = max_value
        self._pi = pigpio.pi()
        self._requested_speed = 0
        self.set_speed(0)

    def calibrate_esc(self):
        self.set_speed(0)
        print("Disconnect the battery and press Enter")
        inp = input()
        if inp == '':
            self.set_speed(self._max_value)
            print("Connect the battery NOW.. you will here two beeps, then wait for a gradual falling tone then press Enter")
            inp = input()
            if inp == '':
                self.set_speed(self._min_value)
                print("Now you should have heard a special tone....12 seconds from now")
                for i in reversed(range(12, 1)):
                    time.sleep(1)
                    print("{} - ".format(i))
                print("done")
                print("Setting speed to zero.....waiting 2 seconds")
                self.set_speed(0)
                time.sleep(2)
                print("Arming ESC now...")
                self.set_speed(self._min_value)
                time.sleep(1)
                print("ESC is now calibrated correctly")

    def arm_esc(self):
        self.set_speed(0)
        time.sleep(1)
        self.set_speed(self._max_value)
        time.sleep(1)
        self.set_speed(self._min_value)
        time.sleep(1)

    def stop_esc(self):
        self.set_speed(0)
        self._pi.stop()

    def set_speed(self, requested_speed):
        self._requested_speed = requested_speed
        self._pi.set_servo_pulsewidth(self._gpio_pin, self._requested_speed)


import os


def main():
    os.system("clear")
    print("Starting <pigpiod>....")
    os.system("sudo pigpiod")
    time.sleep(1)
    print("done")
    esc_driver = ESCDriver(gpio_pin=16, max_value=1700, min_value=500)
    esc_driver.calibrate_esc()
    esc_driver.arm_esc()
    esc_driver.set_speed(800)
    time.sleep(3)
    esc_driver.stop_esc()


if __name__ == "__main__":
    main()


