import math
import numpy as np

from controllers.params import Controller

class Goal:
    __slots__ = 'x', 'y'

    def __init__(self, x, y):
        self.x = x
        self.y = y


class Hold(Controller):

    def __init__(self):
        Controller.__init__(self, 'hold')
        self.test = "nothing"

    def execute(self, state, robot, sensor_distances, dt):
        return 0.0, 0.0

    def restart(self):
        pass