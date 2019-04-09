import math
import numpy as np
from controllers.params import Controller

class Goal:
    __slots__ = 'x', 'y'

    def __init__(self, x, y):
        self.x = x
        self.y = y


class GoToGoal(Controller):
    __slots__ = 'params', 'e_old', 'E', 'goal'

    def __init__(self, params, goal):
        Controller.__init__(self, 'gtg')
        self.params = params
        self.goal = goal
        self.e_old = 0
        self.E = 0

    def restart(self):
        self.e_old = 0
        self.E = 0

    def get_heading_angle(self, state):
        theta = math.atan2(self.goal.y - state.y, self.goal.x - state.x) - state.phi
        return math.atan2(math.sin(theta), math.cos(theta))

    def execute(self, state, robot, sensor_distances, dt):
        heading_angle = self.get_heading_angle(state)

        e_p = heading_angle
        e_i = self.E + e_p * dt
        e_d = (e_p - self.e_old) / dt

        w = self.params.k_p*e_p + self.params.k_i*e_i + self.params.k_d*e_d
        v = robot.specs.v_max

        self.e_old = e_p
        self.E = e_i

        return v, w

