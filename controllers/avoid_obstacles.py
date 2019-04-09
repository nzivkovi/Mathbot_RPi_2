import math
import numpy as np

from robots.mathbot import Mathbot
from controllers.params import Controller
class AvoidObstacles(Controller):
    __slots__ = 'params', 'e_old', 'E', 'robot', 'sensor_states_r', 'sensor_weights', 'vectors', 'direction'

    def __init__(self, params, robot):
        Controller.__init__(self, 'ao')
        self.params = params
        self.robot = robot
        self.e_old = 0
        self.E = 0
        self.direction = 1

        self.sensor_states_r = [sensor.state_r for sensor in self.robot.sensors]
        self.sensor_weights = [(math.cos(state.phi + 1.5)) for state in self.sensor_states_r]
        ws = sum(self.sensor_weights)
        #self.sensor_weights = [w / ws for w in self.sensor_weights]
        self.sensor_weights = [1, 1, 0.5, 1, 1]
        ##self.sensor_weights = [0.5, 0.75, 1.0, 0.75, 0.5]
        self.vectors = None

    def restart(self):
        self.e_old = 0
        self.E = 0

    def get_heading_angle(self, state, sensor_distances):
        self.vectors = np.array([np.dot(p.get_transformation(), np.array([d, 0, 1])) for d, p
                                 in zip(sensor_distances, self.sensor_states_r)])

        heading = np.dot(self.vectors.transpose(), self.sensor_weights)
##        print(heading)
        heading_angle = math.atan2(-heading[1], -heading[0])
        return heading_angle
        # heading_angle = math.atan2(-heading[1], -heading[0]) - state.phi
        # return math.atan2(math.sin(heading_angle), math.cos(heading_angle))

    def execute(self, state, robot, sensor_distances, dt):
        heading_angle = self.get_heading_angle(state, sensor_distances)

        e_p = heading_angle
        e_i = self.E + e_p * dt
        e_d = (e_p - self.e_old) / dt

        w = self.params.k_p*e_p + self.params.k_i*e_i + self.params.k_d*e_d
        v = robot.specs.v_max

        self.e_old = e_p
        self.E = e_i
        
##        print(v,w)

        return v, -w

