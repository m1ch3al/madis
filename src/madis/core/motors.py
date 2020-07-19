
from madis.core.esc_driver import ESCDriver


class Motor(object):
    def __init__(self, motor_id, gpio_pin, min_value=1200, max_value=1700):
        self._motor_id = motor_id
        self._min_value = min_value
        self._max_value = max_value
        self._esc_driver = ESCDriver(gpio_pin=gpio_pin, max_value=max_value, min_value=min_value)
        self._current_speed_percentage = None

    def get_motor_id(self):
        return self._motor_id

    def get_motor_gpio_number(self):
        return self._esc_driver.get_gpio_pin()

    def set_speed(self, percentage):
        if percentage < 0:
            raise ValueError("Speed percentage is NEGATIVE - use value between 0 and 100")
        if percentage > 100:
            raise ValueError("Speed percentage is TOO HIGH - use value between 0 and 100")
        value_to_set = ((self._max_value - self._min_value) * (percentage/100)) + self._min_value
        self._esc_driver.set_speed(int(value_to_set))
        self._current_speed_percentage = percentage
        print("value_to_set : {}".format(value_to_set))
        print("current percentage speed : {}%".format(self._current_speed_percentage))

    def get_current_speed(self):
        esc_speed = self._esc_driver.get_current_speed()
        self._current_speed_percentage = round(((esc_speed - self._min_value)/(self._max_value - self._min_value)), 1) * 100
        return self._current_speed_percentage

    def stop_motor(self):
        self._esc_driver.set_speed(0)

    def increase_speed(self, percentage_step=None):
        self.get_current_speed()
        if percentage_step is None:
            self.set_speed(self._current_speed_percentage + 1)
        else:
            self.set_speed(self._current_speed_percentage + percentage_step)

    def decrease_speed(self, percentage_step=None):
        self.get_current_speed()
        if percentage_step is None:
            self.set_speed(self._current_speed_percentage - 1)
        else:
            self.set_speed(self._current_speed_percentage - percentage_step)

    def calibrate(self):
        self._esc_driver.calibrate_esc()


import time

def main():
    print("Start motor test...")
    motor = Motor(motor_id=1, gpio_pin=20, max_value=1700, min_value=1100)
    print("Do you want to calibrate the motor ? (y/n) : ")
    inp = input()
    if inp.lower() == "y":
        motor.calibrate()
    else:
        print("skipping calibration...")
        motor._esc_driver.arm_esc()
    stop = False
    while not stop:
        print("Insert speed in percentage (0-100) or x for exit: ")
        speed_requested = input()
        if speed_requested.lower() == "x":
            stop = True
        else:
            if speed_requested == "+":
                motor.increase_speed()
                print("increasing speed - Motor speed : {} %".format(motor.get_current_speed()))
            if speed_requested == "-":
                motor.decrease_speed()
                print("decrease speed - Motor speed : {} %".format(motor.get_current_speed()))
            try:
                motor.set_speed(float(speed_requested))
                print("Set speed - Motor speed : {} %".format(motor.get_current_speed()))
            except Exception as ex:
                pass
    motor.stop_motor()


if __name__ == "__main__":
    main()


