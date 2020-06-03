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
Abstract class for represent various sensor inside the DRONE.

"""

from abc import ABC, abstractmethod
from madis.utils.time_date import from_str, date2epoch


class Sensor(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def initialize_sensor(self):
        pass

    @abstractmethod
    def read_from_sensor(self):
        pass

    def get_utc_time(self):
        return date2epoch(from_str("now", is_utc=True))
