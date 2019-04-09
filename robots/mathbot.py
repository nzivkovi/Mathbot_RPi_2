import math

from helpers.body import State, Body
from helpers.spi import SpiMaster

from robots.sensor import Sensor

import numpy as np

class Specs:
    __slots__ = 'wheel_radius', 'base_length', 'cpr', 'v_max', 'min_rpm', 'max_rpm', 'angular_wheel_min', \
                'angular_wheel_max'

    def __init__(self, wheel_radius, base_length, cpr, v_max, min_rpm, max_rpm):
        self.wheel_radius = wheel_radius
        self.base_length = base_length
        self.cpr = cpr
        self.v_max = v_max
        self.min_rpm = min_rpm
        self.max_rpm = max_rpm
        self.angular_wheel_min = self.min_rpm * 2 * math.pi / 60
        self.angular_wheel_max = self.max_rpm * 2 * math.pi / 60


class Wheel:
    __slots__ = 'nr_of_ticks', 'nr_of_revolutions', 'cpr', 'wheel_radius'

    def __init__(self, wheel_radius, cpr):
        self.nr_of_ticks = 0
        self.nr_of_revolutions = 0
        self.wheel_radius = wheel_radius
        self.cpr = cpr

    # def update_ticks(self, v, dt):
    #     self.nr_of_revolutions += v*dt/2/math.pi
    #     self.nr_of_ticks = int(self.nr_of_revolutions*self.cpr)


class Mathbot(Body):
    __slots__ = 'specs', 'left_wheel', 'right_wheel', 'chassis_body', 'chassis', 'envelope', 'wheel_points',\
                'sensors', 'left_wheel_body', 'right_wheel_body', 'spi'

    def __init__(self, state, specs, spi):
        envelope = [(-0.05, -0.06), (0.05, -0.06), (0.07, -0.035), (0.07, 0.035),\
                          (0.05, 0.06), (-0.05, 0.06)]
        Body.__init__(self, state, envelope)
        self.specs = specs
        self.spi = spi
        self.left_wheel = Wheel(specs.wheel_radius, specs.cpr)
        self.right_wheel = Wheel(specs.wheel_radius, specs.cpr)

        # self.chassis = [(-0.04, -0.05), (0.04, -0.05), (0.06, -0.025), (0.06, 0.025), (0.04, 0.05), (-0.04, 0.05)]
        # self.chassis_body = Body(self.state, self.chassis)

        # left_wheel_points = [(-0.02, -0.06), (0.02, -0.06), (0.02, -0.05), (-0.02, -0.05)]
        # right_wheel_points = [(-0.02, 0.06), (0.02, 0.06), (0.02, 0.05), (-0.02, 0.05)]
        # self.left_wheel_body = Body(self.state, left_wheel_points)
        # self.right_wheel_body = Body(self.state, right_wheel_points)

        # self.wheel_points = [(0, -0.05), (0, 0.05)]

        """
        sensors_states_r = [
            State(-0.038, 0.048, np.radians(128)),
            State(0.019, 0.064, np.radians(75)),
            State(0.050, 0.050, np.radians(42)),
            State(0.070, 0.017, np.radians(13)),
            State(0.070, -0.017, np.radians(-13)),
            State(0.050, -0.050, np.radians(-42)),
            State(0.019, -0.064, np.radians(-75)),
            State(-0.038, -0.048, np.radians(-128)),
            State(-0.048, 0.000, np.radians(180))
        ]"""

        sensors_states_r = [
            State(0.01, 0.035, np.radians(90)),
            State(0.015, 0.02, np.radians(45)),
            State(0.0175, 0.00, np.radians(0)),
            State(0.015, -0.02, np.radians(-45)),
            State(0.01, -0.035, np.radians(-90))
        ]

        self.sensors = []

        for sensor_state in sensors_states_r:
            self.sensors.append(Sensor(sensor_state, self))


    def update_ticks(self):
        self.left_wheel.nr_of_ticks, self.right_wheel.nr_of_ticks = self.spi.get_ticks()

    def diff_to_uni(self, angular_left, angular_right):
        transl_speed = (angular_left + angular_right) * self.specs.wheel_radius / 2
        angular_speed = (angular_right - angular_left) * self.specs.wheel_radius / self.specs.base_length
        return transl_speed, angular_speed

    def uni_to_diff(self, transl_speed, angular_speed):
        summ = 2 * transl_speed / self.specs.wheel_radius
        diff = self.specs.base_length * angular_speed / self.specs.wheel_radius
        
##        print(summ, diff)

        angular_left = (summ - diff) / 2
        angular_right = (summ + diff) / 2

        return angular_left, angular_right

    # def update_state(self, transl_speed, angular_speed, dt):
    #     if angular_speed == 0:
    #         self.state.x += transl_speed * math.cos(self.state.phi) * dt
    #         self.state.y += transl_speed * math.sin(self.state.phi) * dt
    #     else:
    #         phi_dt = angular_speed * dt
    #         self.state.x += 2 * transl_speed / angular_speed * math.cos(self.state.phi + phi_dt / 2) * math.sin(phi_dt / 2)
    #         self.state.y += 2 * transl_speed / angular_speed * math.sin(self.state.phi + phi_dt / 2) * math.sin(phi_dt / 2)
    #         self.state.phi += phi_dt

    def move(self, angular_left, angular_right, dt):
        # transl_speed, angular_speed = self.diff_to_uni(angular_left, angular_right)
        # self.update_state(transl_speed, angular_speed, dt)
        # self.left_wheel.update_ticks(angular_left, dt)
        # self.right_wheel.update_ticks(angular_right, dt)
        ##angular_left = 255 * angular_left / self.specs.angular_wheel_max
        ##angular_right = 255 * angular_right / self.specs.angular_wheel_max
        self.spi.send_velocities(angular_left, angular_right)

    # def get_envelope(self):
    #     return self.envelope

    # def update_body(self):
    #     self.get_envelope_i(True)
    #     self.chassis_body.get_envelope_i(True, False)
    #     self.left_wheel_body.get_envelope_i(True, False)
    #     self.right_wheel_body.get_envelope_i(True, False)
    #     for sensor in self.sensors:
    #         sensor.get_envelope_i(True)
    #         sensor.set_state()
