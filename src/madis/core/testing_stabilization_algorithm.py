import time
from madis.core.motors import Motor
import threading
import rospy
from std_msgs.msg import String
import yaml


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
        self._motor_A.set_speed(15)
        self._motor_A.set_speed(15)


def main():
    stabilizer = StabilizerWith2Motors(gpio_motor_A=16, gpio_motor_B=20)
    stabilizer.subscribe_into_MAD_system()
    stabilizer.initialize_motors()
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()


