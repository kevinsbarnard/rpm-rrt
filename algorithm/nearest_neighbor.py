# nearest_neighbor.py
# Nearest neighbor algorithm

from structs.cartesian import Point


def nearest_neighbor(ref_point: Point, points, key=None):
    dists = []
    if key:
        dists = [ref_point.dist(key(point)) for point in points]
    else:
        dists = [ref_point.dist(point) for point in points]
    return min(zip(points, dists), key=lambda pair: pair[1])[0]


def within_distance(ref_point: Point, points, distance: float, key=None):
    dists = []
    if key:
        dists = [ref_point.dist(key(point)) for point in points]
    else:
        dists = [ref_point.dist(point) for point in points]
    return [el[0] for el in filter(lambda pair: pair[1] < distance, zip(points, dists))]
