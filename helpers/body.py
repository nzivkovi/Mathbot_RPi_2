import math
from helpers.polygon import Polygon
import numpy as np
class State:
    __slots__ = 'x', 'y', 'phi'

    def __init__(self, x=0, y=0, phi=0):
        self.x = x
        self.y = y
        self.phi = phi

    def get_transformation(self):
        T = np.array([
            [np.cos(self.phi), -np.sin(self.phi), self.x],
            [np.sin(self.phi), np.cos(self.phi), self.y],
            [0, 0, 1.0]
        ])

        return T


class Body:
    __slots__ = "state", "envelope_i", "envelope", "poly"

    def __init__(self, state, envelope=None):
        self.state = state
        self.envelope = envelope
        self.envelope_i = None
        self.poly = None

    def get_envelope(self):
        return self.envelope

    def get_envelope_i(self, recalculate=False, calculate_poly=True):
        if self.envelope_i is None or recalculate:
            self.envelope_i = [(self.state.x + x * math.cos(self.state.phi) - y * math.sin(self.state.phi),
                           self.state.y + x * math.sin(self.state.phi) + y * math.cos(self.state.phi))
                          for x, y in self.get_envelope()]
            if calculate_poly:
                self.poly = Polygon(self.envelope_i)
        return self.envelope_i

    def get_bounds(self):
        xs, ys = zip(*self.get_envelope_i())
        return min(xs), min(ys), max(xs), max(ys)

    def get_bounding_rectangle(self):
        x_min, y_min, x_max, y_max = self.get_bounds()
        return x_min, y_min, x_max - x_min, y_max - y_min

    def get_contact_points(self, other):
        if other is None:
            return []
        else:
            return self.poly.intersection_points(other.poly)

    def has_collision(self, other_body):
        collision = self.poly.collidepoly(other_body.poly)
        if isinstance(collision, bool):
            if not collision:
                return False
        return True
