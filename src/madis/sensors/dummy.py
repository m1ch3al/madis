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
A dummy set of sensors for simulate data inside the DRONE.

"""

from abc import ABC, abstractmethod


class Sensor(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def initialize_sensor(self):
        pass

    @abstractmethod
    def read_from_sensor(self):
        pass
