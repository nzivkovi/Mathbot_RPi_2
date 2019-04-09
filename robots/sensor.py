from helpers.body import Body, State
import numpy as np
from math import cos, sin, sqrt, pow, exp
from helpers.polygon import Polygon
class SensorSpecs:
    __slots__ = 'range_min', 'range_max', 'alpha'

    def __init__(self, range_min, range_max, alpha):
        self.range_max = range_max
        self.range_min = range_min
        self.alpha = alpha

class Sensor(Body):
    __slots__ = 'specs', 'robot', 'full_cone', '_distance', 'points', 'state_r'

    def __init__(self, state, robot):
        self.robot = robot
        self.specs = SensorSpecs(0.1, 0.8, np.radians(20))
        self.state_r = State(state.x, state.y, state.phi)
        # self.full_cone = [(0, 0),\
        #                   (self.specs.range_max * cos(self.specs.alpha/2), self.specs.range_max * sin(self.specs.alpha/2)),
        #                   (self.specs.range_max, 0),
        #                   (self.specs.range_max * cos(self.specs.alpha / 2),
        #                    -self.specs.range_max * sin(self.specs.alpha / 2)),
        #                   ]
        # Body.__init__(self, state, self.full_cone)
        # self._distance = 65536
        # self.points = self.get_cone(self.specs.range_max)

    # def get_envelope(self):
    #     return self.full_cone

    # def get_cone(self, dst):
    #     return [
    #         (self.specs.range_min * cos(self.specs.alpha / 2),\
    #          self.specs.range_min * sin(self.specs.alpha / 2)),\
    #         (dst * cos(self.specs.alpha / 2),\
    #          dst * sin(self.specs.alpha / 2)),\
    #         (dst, 0),
    #         (dst * cos(self.specs.alpha / 2), \
    #          -dst * sin(self.specs.alpha / 2)),
    #         (self.specs.range_min * cos(self.specs.alpha / 2), \
    #          -self.specs.range_min * sin(self.specs.alpha / 2))
    #         ]

    # def get_distance_to(self, other_object):
    #     x = self.state.x
    #     y = self.state.y
    #     min_distance = None
    #     for p_x, p_y in self.get_contact_points(other_object):
    #         distance = sqrt(pow(x-p_x, 2)+pow(y-p_y, 2))
    #         if min_distance is not None:
    #             if distance < min_distance:
    #                 min_distance = distance
    #         else:
    #             min_distance = distance
    #     return min_distance

    # def update_distance(self, other_object=None):
    #     if other_object is None:
    #         self._distance = 65536
    #         self.points = self.get_cone(self.specs.range_max)
    #         return True

    #     else:
    #         distance_to_obj = self.get_distance_to(other_object)
    #         if distance_to_obj:
    #             if self._distance > distance_to_obj:
    #                 self.points = self.get_cone(distance_to_obj)
    #                 self._distance = distance_to_obj
    #                 return True
    #     return False

    def set_state(self):
        x = self.state_r.x
        y = self.state_r.y
        phi = self.state_r.phi

        r_x = self.robot.state.x
        r_y = self.robot.state.y
        r_phi = self.robot.state.phi

        self.state.x = r_x + x * cos(r_phi) - y * sin(r_phi)
        self.state.y = r_y + x * sin(r_phi) + y * cos(r_phi)
        self.state.phi = r_phi + phi

    # def get_envelope_i(self, recalculate=False):
    #     if self.envelope_i is None or recalculate:
    #         self.envelope_i = [(self.state.x \
    #                             + x * cos(self.state.phi)\
    #                             - y * sin(self.state.phi),\
    #                             self.state.y + x * sin(self.state.phi) \
    #                             + y * cos(self.state.phi)

    #                             ) for x, y in self.full_cone]
    #         self.poly = Polygon(self.envelope_i)
    #     return self.envelope_i

    # def get_envelope_plot(self):
    #     return [(self.state.x \
    #       + x * cos(self.state.phi) \
    #       - y * sin(self.state.phi), \
    #       self.state.y + x * sin(self.state.phi) \
    #       + y * cos(self.state.phi)

    #       ) for x, y in self.points]


    # def distance_to_value(self, distance):
    #     if distance < self.specs.range_min:
    #         return 3960
    #     else:
    #         return 3960 * exp(-30*(distance - self.specs.range_min))

    # def distance(self):
    #     return self._distance

    # def read_distance(self):
    #     return self.distance_to_value(self.distance())
