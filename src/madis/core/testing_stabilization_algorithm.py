import time
from madis.core.motors import Motor
import threading
import rospy
from std_msgs.msg import String
import yaml
import os


class StabilizerWith2Motors(object):
    def __init__(self, gpio_motor_A, gpio_motor_B, min_value=1100, max_value=1700):
        self._gpio_motor_A = gpio_motor_A
        self._gpio_motor_B = gpio_motor_B
        self._min_value = min_value
        self._max_value = max_value

        self._motor_A = None
        self._motor_B = None

    def initialize_motors(self):
        self._motor_A = Motor(1, self._gpio_motor_A, min_value=self._min_value, max_value=self._max_value)
        self._motor_B = Motor(2, self._gpio_motor_B, min_value=self._min_value, max_value=self._max_value)

        calibration_thread_motor_A = threading.Thread(target=self._motor_A.calibrate, args=())
        calibration_thread_motor_B = threading.Thread(target=self._motor_B.calibrate, args=())
        calibration_thread_motor_A.setDaemon(True)
        calibration_thread_motor_B.setDaemon(True)
        calibration_thread_motor_A.start()
        calibration_thread_motor_B.start()

    def subscribe_into_MAD_system(self):
        rospy.init_node('stabilizer_algorightm', anonymous=True)
        rospy.Subscriber("stability", String, self._regulate_motors_speed)

    def _regulate_motors_speed(self, ros_data):
        stabilizer_data = yaml.safe_load(ros_data.data)
        accel_x = stabilizer_data["acceleration_x"]
        accel_y = stabilizer_data["acceleration_y"]
        if accel_x < 0:
            if self._motor_A.get_current_speed() <= 30:
                self._motor_A.increase_speed(1)
            self._motor_B.decrease_speed(0.6)
        else:
            if self._motor_B.get_current_speed() <= 30:
                self._motor_B.increase_speed()
            self._motor_A.decrease_speed(0.6)
        os.system("clear")
        print("         Acceleration X : {}".format(accel_x))
        print("         Acceleration Y : {}".format(accel_y))
        print(" MOTOR A (left) speed % : {}".format(self._motor_A.get_current_speed()))
        print("MOTOR B (right) speed % : {}".format(self._motor_B.get_current_speed()))


    def stop_all(self):
        self._motor_A.stop_motor()
        self._motor_B.stop_motor()


def main():
    stabilizer = StabilizerWith2Motors(gpio_motor_A=16, gpio_motor_B=20)
    print("Stabilizer created")
    print("Motors calibration")
    stabilizer.initialize_motors()
    time.sleep(50)
    stabilizer._motor_A.set_speed(10)
    stabilizer._motor_B.set_speed(10)
    print("MOTOR STATUS : ")
    print(" MOTOR A (left) speed % : {}".format(stabilizer._motor_A.get_current_speed()))
    print("MOTOR B (right) speed % : {}".format(stabilizer._motor_B.get_current_speed()))
    print("Calibrations done...start stabilization PID")
    time.sleep(5)
    stabilizer.subscribe_into_MAD_system()
    counter = 0
    try:
        while True:
            counter += 1
            time.sleep(1)
            if counter == 60:
                stabilizer.stop_all()
                exit(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

