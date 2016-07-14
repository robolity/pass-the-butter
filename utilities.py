# -*- coding: utf-8 -*-

# Zumo Simulator - Controller and simulator for Pololu Zumo 32u4 robot
# Changes from forked project are copyright (C) 2016 Justin D. Clarke
#
#
# Forked from:
#    https://github.com/nmccrea/sobot-rimulator
#    Sobot Rimulator - A Robot Programming Tool
#    Copyright (C) 2013-2014 Nicholas S. D. McCrea
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Email robolity@gmail.com for questions, comments, or to report bugs.

from math import sqrt, sin, cos

#**********CLASSES IN MODULE**********************

#LinearAlgebra

#*************************************************


class LinearAlgebra(object):
    """Class to access linear algebra functions.

    Attributes:
        none

    Methods:
        __init__()
        add(a, b)
        sub(a, b)
        scale(a, s)
        dot(a, b)
        cross(a, b)
        mag(a)
        unit(a)
        rnormal(a)
        runormal(a)
        lnormal(a)
        lunormal(a)
        proj(a, b)
        distance(a, b)
        rotate_vector(a, theta)
        rotate_vectors(vects, theta)
        rotate_and_translate_vector(a, theta, tvect)
        rotate_and_translate_vectors(vects, theta, tvect)
    """
    def __init__(self):
        pass

    def add(self, a, b):
        """Return the sum of two vectors a & b."""
        return [a[0] + b[0], a[1] + b[1]]

    def sub(self, a, b):
        """Return the difference of two vectors (a - b)."""
        return [a[0] - b[0], a[1] - b[1]]

    def scale(self, a, s):
        """Return multiple of vector a by a scalar s."""
        return [s * a[0], s * a[1]]

    def dot(self, a, b):
        """Return the dot-product of two vectors a & b."""
        return a[0] * b[0] + a[1] * b[1]

    def cross(self, a, b):
        """Return the cross-product of two vectors (a x b)."""
        return a[0] * b[1] - a[1] * b[0]

    def mag(self, a):
        """Return the magnitude of vector a."""
        return sqrt(a[0] ** 2 + a[1] ** 2)

    def unit(self, a):
        """Return the unit vector of vector a."""
        m = self.mag(a)
        return [a[0] / m, a[1] / m]

    def rnormal(self, a):
        """Return the right-hand normal of a vector a."""
        return [a[1], -a[0]]

    def runormal(self, a):
        """Return the right-hand unit normal of a vector."""
        return self.unit([a[1], -a[0]])

    def lnormal(self, a):
        """Return the left-hand normal of a vector."""
        return [-a[1], a[0]]

    def lunormal(self, a):
        """Return the left-hand unit normal of a vector."""
        return self.unit([-a[1], a[0]])

    def proj(self, a, b):
        """Return the projection of vector a onto vector b."""
        scale = float(self.dot(a, b)) / (b[0] ** 2 + b[1] ** 2)
        return [scale * b[0], scale * b[1]]

    def distance(self, a, b):
        """Return the length of the difference of two vectors a and b."""
        return self.mag(self.sub(a, b))

    def rotate_vector(self, a, theta):
        """Return the result of rotating a vector by theta radians."""
        sin_theta = sin(theta)
        cos_theta = cos(theta)

        a0 = a[0] * cos_theta - a[1] * sin_theta
        a1 = a[0] * sin_theta + a[1] * cos_theta

        return [a0, a1]

    def rotate_vectors(self, vects, theta):
        """Return the result of rotating a set of vectors by theta radians."""
        sin_theta = sin(theta)
        cos_theta = cos(theta)

        rotvects = []
        for a in vects:
            a0 = a[0] * cos_theta - a[1] * sin_theta
            a1 = a[0] * sin_theta + a[1] * cos_theta
            rotvects.append([a0, a1])

        return rotvects

    def rotate_and_translate_vector(self, a, theta, tvect):
        """Return the result of rotating and translating a vector."""
        return self.add(self.rotate_vector(a, theta), tvect)

    def rotate_and_translate_vectors(self, vects, theta, tvect):
        """Return the result of rotating and translating a set of vectors."""
        rtvects = []
        for a in self.rotate_vectors(vects, theta):
            rtvects.append(self.add(a, tvect))

        return rtvects

    def determine_side_of_line(self, lpoint1, lpoint2, tpoint):
        """Determine which side of a line a point lies on.

        returns  1 if the point is to the left of the line
        returns -1 if the point is to the right of the line
        returns  0 if the point is on the line
        the directionality of the line is taken to be lpoint1 -> lpoint2
        """
        tx = tpoint[0]
        ty = tpoint[1]
        l1x = lpoint1[0]
        l1y = lpoint1[1]
        l2x = lpoint2[0]
        l2y = lpoint2[1]

        d = (ty - l1y) * (l2x - l1x) - (tx - l1x) * (l2y - l1y)

        return d if d == 0 else int(d / abs(d))