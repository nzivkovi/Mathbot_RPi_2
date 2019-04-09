import math
import numpy as np

from robots.mathbot import Mathbot
from controllers.params import Controller


class FollowWall(Controller):
    __slots__ = 'params', 'e_old', 'E', 'robot', 'sensor_states_r', 'sensor_weights',\
                'vectors', 'initial_distance', 'direction', 'normals'

    def __init__(self, params, robot):
        Controller.__init__(self, 'fw')
        self.params = params
        self.robot = robot
        self.e_old = 0
        self.E = 0
        self.initial_distance = 0
        self.direction = 1

        self.sensor_states_r = [sensor.state_r for sensor in self.robot.sensors]
        self.sensor_weights = [(math.cos(state.phi + 1.5)) for state in self.sensor_states_r]
        self.sensor_weights = [1, 1, 0.5, 1, 1]
        self.vectors = None
        self.normals = None

    def restart(self, distance):
        self.e_old = 0
        self.E = 0
        self.initial_distance = distance

    def get_heading_angle(self, state, sensor_distances):
        heading = self.get_heading(state, sensor_distances)
##        print(heading)
        heading_angle = math.atan2(heading[1], heading[0])
        return heading_angle

    def get_heading(self, state, sensor_distances):
        sensors = [(i, d, p) for i, d, p in zip(range(len(sensor_distances)), sensor_distances, self.sensor_states_r)
                   if p.phi * self.direction < 0 and d < self.robot.sensors[0].specs.range_max]

        if len(sensors) == 0:
            return np.array([1, -self.direction, 0])
        self.vectors = np.array([(np.dot(p.get_transformation(), np.array([d, 0, 1]))) for i, d, p
                                 in sensors])

        self.normals = np.array([np.array([self.direction * v[0], -self.direction * v[1]]) for v in self.vectors])

        heading = np.dot(self.normals.transpose(), [w for w, i in zip(self.sensor_weights,
                                                                      range(len(self.sensor_weights)))
                                                    if i in [j for j, d, p in sensors]])

        return heading

    def execute(self, state, robot, sensor_distances, dt):
        heading_angle = self.get_heading_angle(state, sensor_distances)
##        print(heading_angle)

        e_p = heading_angle
        e_i = self.E + e_p * dt
        e_d = (e_p - self.e_old) / dt

        w = self.params.k_p*e_p + self.params.k_i*e_i + self.params.k_d*e_d
        v = robot.specs.v_max

        self.e_old = e_p
        self.E = e_i
        
##        print(v,w)

        return v, -w
