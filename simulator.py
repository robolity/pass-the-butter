#!/usr/bin/env python

# Sobot Rimulator - A Robot Programming Tool
# Copyright (C) 2013-2014 Nicholas S. D. McCrea
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
# Email mccrea.engineering@gmail.com for questions, comments, or to report bugs.


from environment import World

REFRESH_RATE = 2.0  # hertz


class Simulator(object):

    def __init__(self):
        # timing control (period = ideal time period of each step)
        self.period = 1.0 / REFRESH_RATE  # seconds

        # create the world
        self.world = World(self.period)


# RUN THE SIM:
if __name__ == "__main__":
    Simulator()
