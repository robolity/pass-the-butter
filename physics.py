# -*- coding: utf-8 -*-

from math import sin, cos, atan2
from utilities import LinearAlgebra

#**********CLASSES IN MODULE**********************

#Physics
#Polygon
#LineSegment
#Pose
#Geometrics
#CollisionException

#*************************************************

# bring in linear algebra functions
linalg = LinearAlgebra()


#*************************************************
class Physics(object):

    def __init__(self, world):
        # the world this physics engine acts on
        self.world = world

        self.geometrics = Geometrics()

    def apply_physics(self):
        self._detect_collisions()
        self._update_proximity_sensors()

    # test the world for existing collisions with solids
    # raises a CollisionException if one occurs
    def _detect_collisions(self):
        colliders = self.world.colliders()
        solids = self.world.solids()

        for collider in colliders:
            polygon1 = collider.global_geometry     # polygon1

            for solid in solids:
                if solid is not collider:  # don't test an object against itself
                    polygon2 = solid.global_geometry    # polygon2

                    # don't bother testing objects that are not near each other
                    if self.geometrics.check_nearness(polygon1, polygon2):
                        if self.geometrics.convex_polygon_intersect_test(
                                polygon1, polygon2):
                            raise CollisionException()

    # update any proximity sensors that are in range of solid objects
    def _update_proximity_sensors(self):
        robots = self.world.robots
        solids = self.world.solids()

        for robot in robots:
            sensors = robot.ir_sensors

            for sensor in sensors:
                dmin = float('inf')
                detector_line = sensor.detector_line

                for solid in solids:

                    # assume that the sensor does not detect it's own robot
                    if solid is not robot:
                        solid_polygon = solid.global_geometry

                        # don't bother testing objects that are not near each
                        #    other
                        if self.geometrics.check_nearness(detector_line,
                                solid_polygon):
                            intersection_exists, intersection, d = (
                                (self.geometrics.
                                    directed_line_segment_polygon_intersection(
                                        detector_line, solid_polygon)))

                            if intersection_exists and d < dmin:
                                dmin = d

                # if there is an intersection, update the sensor with the
                #    new delta value
                if dmin != float('inf'):
                    sensor.detect(dmin)
                else:
                    sensor.detect(None)


#*************************************************
class Geometry(object):

    def __init__(self, vertexes):
        self.vertexes = vertexes  # a list of 2-dimensional vectors

        # define the centerpoint and radius of a circle containing this polygon
        # value is a tuple of the form ( [ cx, cy ], r )
        # NOTE: this may not be the "minimum bounding circle"
        self.bounding_circle = self._bounding_circle()

    def get_transformation_to_pose(self, pose):
        raise NotImplementedError()


#*******************************************************
class Polygon(Geometry):

    # return a copy of this polygon transformed to the given pose
    def get_transformation_to_pose(self, pose):
        p_pos, p_theta = pose.vunpack()
        return Polygon(linalg.rotate_and_translate_vectors(self.vertexes,
            p_theta, p_pos))

    # get a list of of this polygon's edges as vertex pairs
    def edges(self):
        vertexes = self.vertexes

        edges = []
        n = len(vertexes)
        for i in range(n):
            edges.append([vertexes[i], vertexes[(i + 1) % n]])

        return edges

    # get the number of edges of this polygon
    def numedges(self):
        return len(self.vertexes)

    # get the centerpoint and radius for a circle that completely contains
    #    this polygon
    def _bounding_circle(self):
        # NOTE: this method is meant to give a quick bounding circle
        #   the circle calculated may not be the "minimum bounding circle"

        c = self._centroidish()
        r = 0.0
        for v in self.vertexes:
            d = linalg.distance(c, v)
            if d > r:
                r = d

        return c, r

    # approximate the centroid of this polygon
    def _centroidish(self):
        # NOTE: this method is meant to give a quick and dirty approximation
        #       of center of the polygon
        # - it returns the average of the vertexes
        # - the actual centroid may not be equivalent

        n = len(self.vertexes)
        x = 0.0
        y = 0.0
        for v in self.vertexes:
            x += v[0]
            y += v[1]
        x /= n
        y /= n

        return [x, y]


#**********************************************************
class LineSegment(Geometry):

    # return a copy of this line segment transformed to the given pose
    def get_transformation_to_pose(self, pose):
        p_pos, p_theta = pose.vunpack()
        return LineSegment(linalg.rotate_and_translate_vectors(self.vertexes,
            p_theta, p_pos))

    # get the centerpoint and radius of a circle that contains this line segment
    def _bounding_circle(self):
        v = self._as_vector()
        vhalf = linalg.scale(v, 0.5)

        c = linalg.add(self.vertexes[0], vhalf)
        r = linalg.mag(v) * 0.5

        return c, r

    # get the vector from the beginning point to the end point of this
    #    line segment
    def _as_vector(self):
        return linalg.sub(self.vertexes[1], self.vertexes[0])


#****************************************************
class Pose(object):
    def __init__(self, *args):
        if len(args) == 2:  # initialize using a vector (vect, theta)
            vect = args[0]
            theta = args[1]

            self.x = vect[0]
            self.y = vect[1]
            self.theta = self.normalize_angle(theta)
        elif len(args) == 3:  # initialize using scalars (x, y, theta)
            x = args[0]
            y = args[1]
            theta = args[2]

            self.x = x
            self.y = y
            self.theta = self.normalize_angle(theta)
        else:
            raise TypeError("Wrong number of arguments. Pose requires 2 "
                "or 3 arguments to initialize")

    # get a new pose given by this pose transformed to a given reference pose
    def transform_to(self, reference_pose):
        # elements of this pose (the relative pose)
        rel_vect, rel_theta = self.vunpack()

        # elements of the reference pose
        ref_vect, ref_theta = reference_pose.vunpack()

        # construct the elements of the transformed pose
        result_vect_d = linalg.rotate_vector(rel_vect, ref_theta)
        result_vect = linalg.add(ref_vect, result_vect_d)
        result_theta = ref_theta + rel_theta

        return Pose(result_vect, result_theta)

    # get a new pose given by inverting this pose, e.g. return the pose
    #    of the "world" relative to this pose
    def inverse(self):
        result_theta = -self.theta
        result_pos = linalg.rotate_vector([-self.x, -self.y], result_theta)

        return Pose(result_pos, result_theta)

    # update pose using a vector
    def vupdate(self, vect, theta):
        self.x = vect[0]
        self.y = vect[1]
        self.theta = self.normalize_angle(theta)

    # update pose using scalars
    def supdate(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = self.normalize_angle(theta)

    # return the constituents of this pose with location as a vector
    def vunpack(self):
        return [self.x, self.y], self.theta

    # return the constituents of this pose as all scalars
    def sunpack(self):
        return self.x, self.y, self.theta

    # return the position component of this pose as a vector
    def vposition(self):
        return [self.x, self.y]

    # map the given angle to the equivalent angle in [ -pi, pi ]
    def normalize_angle(self, theta):
        return atan2(sin(theta), cos(theta))


#**********************************************
class Geometrics(object):

    def __init__(self):
        pass

    # a fast test to determine if two geometries might be touching
    def check_nearness(self, geometry1, geometry2):
        c1, r1 = geometry1.bounding_circle
        c2, r2 = geometry2.bounding_circle
        return linalg.distance(c1, c2) <= r1 + r2

    # determine if two convex polygons intersect
    def convex_polygon_intersect_test(self, polygon1, polygon2):
        # assign polygons according to which has fewest sides - we will
        #    test against the polygon with fewer sides first
        if polygon1.numedges() <= polygon2.numedges():
            polygonA = polygon1
            polygonB = polygon2
        else:
            polygonA = polygon2
            polygonB = polygon1

        # perform Seperating Axis Test
        intersect = True
        edge_index = 0
        edges = polygonA.edges() + polygonB.edges()
        # loop through the edges of polygonA searching for a separating axis
        while intersect is True and edge_index < len(edges):
            # get an axis normal to the current edge
            edge = edges[edge_index]
            edge_vector = linalg.sub(edge[1], edge[0])
            projection_axis = linalg.lnormal(edge_vector)

            # get the projection ranges for each polygon onto the projection
            #    axis
            minA, maxA = self.range_project_polygon(projection_axis, polygonA)
            minB, maxB = self.range_project_polygon(projection_axis, polygonB)

            # test if projections overlap
            if minA > maxB or maxA < minB:
                intersect = False
            edge_index += 1

        return intersect

    # get the min and max dot-products of a projection axis and the vertexes
    #    of a polygon - this is sufficient for overlap comparison
    def range_project_polygon(self, axis_vector, polygon):
        vertexes = polygon.vertexes

        c = linalg.dot(axis_vector, vertexes[0])
        minc = c
        maxc = c
        for i in range(1, len(vertexes)):
            c = linalg.dot(axis_vector, vertexes[i])

            if c < minc:
                minc = c
            elif c > maxc:
                maxc = c

        return minc, maxc

    # test two line segments for intersection
    # takes raw line segments, i.e. a pair of vectors
    # returns
    #   intersection_exists - boolean
    #        - value indicating whether an intersection was found
    #   intersection - vector
    #        - the intersection point, or None if none was found
    #   d - float in [0,1]
    #        - distance along line1 at which the intersection occurs
    def line_segment_intersection(self, line1, line2):
        # see http://stackoverflow.com/questions/563198
        nointersect_symbol = (False, None, None)

        p1, r1 = line1[0], linalg.sub(line1[1], line1[0])
        p2, r2 = line2[0], linalg.sub(line2[1], line2[0])

        r1xr2 = float(linalg.cross(r1, r2))
        if r1xr2 == 0.0:
            return nointersect_symbol

        p2subp1 = linalg.sub(p2, p1)

        d1 = linalg.cross(p2subp1, r2) / r1xr2
        d2 = linalg.cross(p2subp1, r1) / r1xr2

        if d1 >= 0.0 and d1 <= 1.0 and d2 >= 0.0 and d2 <= 1.0:
            return True, linalg.add(p1, linalg.scale(r1, d1)), d1
        else:
            return nointersect_symbol

    # test a line segment and a polygon for intersection
    # returns:
    #   intersection_exists - boolean
    #        - value indicating whether an intersection was found
    #   intersection - vector
    #        - the intersection point, or None if none was found
    #   d - float in [0, 1]
    #        - distance along the thest line at which the intersection occurs
    def directed_line_segment_polygon_intersection(self, line_segment,
            test_polygon):
        test_line = line_segment.vertexes  # get the raw line segment
        dmin = float('inf')
        intersection = None

        # a dumb algorithm that tests every edge of the polygon
        for edge in test_polygon.edges():
            intersection_exists, _intersection, d = (
                self.line_segment_intersection(test_line, edge))

            if intersection_exists and d < dmin:
                dmin = d
                intersection = _intersection

        if dmin != float('inf'):
            return True, intersection, dmin
        else:
            return False, None, None


#**************************************************************
class CollisionException(Exception):
    pass