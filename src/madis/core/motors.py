
class Motor(object):
    def __init__(self, motor_id):
        self._motor_id = motor_id
        self.esc_driver = None

    def get_motor_id(self):
        return self._motor_id

    def set_speed(self, rpm_value):
        pass

    def get_current_speed(self):
        pass

    def increase_speed(self, percentage):
        pass

    def decrease_speed(self, percentage):
        pass


