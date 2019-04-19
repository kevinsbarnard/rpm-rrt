# cartesian.py
# Cartesian data structures

import operator
from random import uniform

import math


class Point:
    """ Cartesian point of arbitrary dimension """

    coordinates = dict()
    dim = 0

    def __init__(self, **coordinates):
        self.coordinates = coordinates
        self.dim = len(coordinates.keys())

    def __repr__(self):
        s = '('
        s += ', '.join([k + '=' + str(v) for k, v in sorted(self.coordinates.items(), key=lambda n: n[0])])
        s += ')'
        return s

    def unary_el_op(self, op):
        new_coordinates = dict()
        for c, v in self.coordinates.items():
            new_coordinates[c] = op(v)
        return Point(**new_coordinates)

    def binary_el_op(self, other, op):
        new_coordinates = dict()
        for c, v in self.coordinates.items():
            new_coordinates[c] = op(v, other.coordinates[c])
        return Point(**new_coordinates)

    def __neg__(self):
        return self.unary_el_op(operator.neg)

    def __add__(self, other):
        return self.binary_el_op(other, operator.add)

    def __sub__(self, other):
        return self.binary_el_op(other, operator.sub)

    def __mul__(self, other: float):
        mul_coordinates = dict()
        for c, v in self.coordinates.items():
            mul_coordinates[c] = v * other
        return Point(**mul_coordinates)

    def __truediv__(self, other: float):
        div_coordinates = dict()
        for c, v in self.coordinates.items():
            div_coordinates[c] = v / other
        return Point(**div_coordinates)

    def dist(self, other, p=2):
        """ Returns the Cartesian distance to another point """
        d = 0.0
        for c in self.coordinates.keys():
            d += math.fabs(self.coordinates[c] - other.coordinates[c]) ** p
        return d ** (1.0 / p)

    def norm(self, p=2):
        return self.dist(Point(**dict(zip(
            self.coordinates.keys(),
            [0.0 for _ in range(len(self.coordinates.keys()))]
        ))))


class Space:
    """ Cartesian space of arbitrary dimension """

    bounds = None
    dim = 0

    def __init__(self, **bounds):
        self.bounds = bounds
        self.dim = len(bounds.keys())

    def sample(self, n=1):
        """
        Returns a uniformly random sampled Point (if n==1) or a list of points (if n>1)
        :param n:
        :return:
        """

        if not n > 0:
            raise ValueError('n ({}) must be > 0'.format(n))

        pts = []
        for _ in range(n):
            coordinate_dict = dict()
            for coordinate, bound in self.bounds.items():
                coordinate_dict[coordinate] = uniform(bound[0], bound[1])
            pts.append(Point(**coordinate_dict))
        if n == 1:
            return pts[0]
        return pts

    def within(self, pt: Point):
        return all([self.bounds[c][0] <= pt.coordinates[c] <= self.bounds[c][1] for c in pt.coordinates.keys()])
