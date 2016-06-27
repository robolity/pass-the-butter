# -*- coding: utf-8 -*-

from random import random, randrange
from math import pi, cos, sin
import time

import pickle

from gui import Viewer
from physics import Physics, Polygon, Pose, Geometrics
from robot import Robot, RobotView

#**********CLASSES IN MODULE**********************

#World
#WorldView
#MapManger
#Obstacle
#ObstacleView
#RectangularObstacle(Obstacle)
#Goal

#*************************************************

# generate random map or custom map
RANDOM = False

# default environment parameters
DEFAULT_ZOOM = 200          # pixels per meter
DEFAULT_WORLD_M_W = 4.0     # meters
DEFAULT_WORLD_M_H = 4.0     # meters

# random environment parameters
OBS_MIN_DIM = 0.1           # meters
OBS_MAX_DIM = 2.5           # meters
OBS_MAX_COMBINED_DIM = 2.6  # meters
OBS_MIN_COUNT = 10
OBS_MAX_COUNT = 50
OBS_MIN_DIST = 0.4          # meters
OBS_MAX_DIST = 6.0          # meters
GOAL_MIN_DIST = 0.5         # meters
GOAL_MAX_DIST = 2.0         # meters
MIN_GOAL_CLEARANCE = 0.2    # meters

#custom environment paramters
GOAL_X_DIST = 0
GOAL_Y_DIST = -.5

#gridline spacing for WorldView
MAJOR_GRIDLINE_INTERVAL = 1.0  # meters
MAJOR_GRIDLINE_SUBDIVISIONS = 5  # minor gridlines for every major gridline


class World(object):

    def __init__(self, period=0.05):
        # bind the world step rate
        # - seconds, passed to gui loop
        self.period = period
        # - ideal step length, but procees time will vary
        self.current_dt_target = period

        # setup the size of the world
        self.width = DEFAULT_WORLD_M_W
        self.height = DEFAULT_WORLD_M_H

        # initialize physics engine
        self.physics = Physics(self)

        # create the map manager
        self.map_manager = MapManager()

        # create the GUI
        self.viewer = Viewer(self, self.period, DEFAULT_ZOOM, RANDOM)
        self.viewer.initialise_viewer()

    def initialise_world(self, random=False, loading=False):
        # initialize world time
        self.world_time = 0.0  # seconds
        self.prev_absolute_time = time.time()

        # initialize lists of world objects
        self.supervisors = []
        self.robots = []
        self.obstacles = []

        # create the robot
        robot_1 = Robot(1)
        #robot_2 = Robot(2,-1.0,3.5,270)
        self.add_robot(robot_1)
        #self.add_robot(robot_2)

        # generate a random environment
        if random:
            self.map_manager.random_map(self)
            print("Random Map")
        elif loading:
            self.map_manager.apply_to_world(self)
            print("Loading Map")
        else:
            self.map_manager.custom_map(self)
            print("Custom Map")

        # create the world view
        self.world_view = WorldView(self, self.viewer)

        # render the initial world
        self.draw_world()

    def draw_world(self):
        self.viewer.new_frame()                 # start a fresh frame
        self.world_view.draw_world_to_frame()   # draw the world onto the frame
        self.viewer.draw_frame()                # render the frame

    # step the simulation through one time interval
    def step(self):
        # calculate time since last step iteration
        self.current_dt = time.time() - self.prev_absolute_time

        # update world_time to the current time
        self.prev_absolute_time = time.time()

        # increment world time
        self.world_time += self.current_dt

        # Flag if current_dt is lagging by more than 5%
        if (self.current_dt > (self.current_dt_target * 1.05) or
                self.current_dt < (self.current_dt_target * 0.95)):
            # raise Exception("Simulation lagging")
            print("Simulation lagging: {:.4f}").format(self.current_dt)

        # step all the robots
        for robot in self.robots:
            # step robot motion
            robot.step_motion(self.current_dt)

        # apply physics interactions
        self.physics.apply_physics()

        # NOTE: the supervisors must run last to ensure they are observing
        #       the "current" world step all of the supervisors
        for supervisor in self.supervisors:
            supervisor.step(self.current_dt)

    def add_robot(self, robot):
        self.robots.append(robot)
        self.supervisors.append(robot.supervisor)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    # return all objects in the world that might collide with other objects
    #    in the world during simulation
    def colliders(self):
        # moving objects only
        return self.robots  # as obstacles are static we should not test them
                            #    against each other

    # return all solids in the world
    def solids(self):
        return self.robots + self.obstacles

    def stop_robots(self):
        # stop all the robots
        for robot in self.robots:
            # stop robot motion
            robot.stop_motion()


#******************************************************************
class WorldView(object):

    def __init__(self, world, viewer):
        # bind the viewer
        self.viewer = viewer

        # initialize views for world objects
        self.robot_views = []
        for robot in world.robots:
            self.add_robot(robot)

        self.obstacle_views = []
        for obstacle in world.obstacles:
            self.add_obstacle(obstacle)

    def add_robot(self, robot):
        robot_view = RobotView(self.viewer, robot)
        self.robot_views.append(robot_view)

    def add_obstacle(self, obstacle):
        obstacle_view = ObstacleView(self.viewer, obstacle)
        self.obstacle_views.append(obstacle_view)

    def draw_world_to_frame(self):
        # draw the grid
        self._draw_grid_to_frame()

        # draw all the robots
        for robot_view in self.robot_views:
            robot_view.draw_robot_to_frame()

        # draw all the obstacles
        for obstacle_view in self.obstacle_views:
            obstacle_view.draw_obstacle_to_frame()

    def _draw_grid_to_frame(self):
        # NOTE: THIS FORMULA ASSUMES THE FOLLOWING:
        # - Window size never changes
        # - Window is always centered at (0, 0)

        # calculate minor gridline interval
        minor_gridline_interval = (
            MAJOR_GRIDLINE_INTERVAL / MAJOR_GRIDLINE_SUBDIVISIONS)

        # determine world space to draw grid upon
        meters_per_pixel = 1.0 / self.viewer.pixels_per_meter
        width = meters_per_pixel * self.viewer.view_width_pixels
        height = meters_per_pixel * self.viewer.view_height_pixels
        x_halfwidth = width * 0.5
        y_halfwidth = height * 0.5

        x_max = int(x_halfwidth / minor_gridline_interval)
        y_max = int(y_halfwidth / minor_gridline_interval)

        # build the gridlines
        major_lines_accum = []  # accumulator for major gridlines
        minor_lines_accum = []  # accumulator for minor gridlines

        for i in range(x_max + 1):  # build the vertical gridlines
            x = i * minor_gridline_interval

            if x % MAJOR_GRIDLINE_INTERVAL == 0:  # sort major from minor
                accum = major_lines_accum
            else:
                accum = minor_lines_accum

            # positive-side gridline
            accum.append([[x, -y_halfwidth], [x, y_halfwidth]])
            # negative-side gridline
            accum.append([[-x, -y_halfwidth], [-x, y_halfwidth]])

        for j in range(y_max + 1):  # build the horizontal gridlines
            y = j * minor_gridline_interval

            if y % MAJOR_GRIDLINE_INTERVAL == 0:  # sort major from minor
                accum = major_lines_accum
            else:
                accum = minor_lines_accum

            # positive-side gridline
            accum.append([[-x_halfwidth, y], [x_halfwidth, y]])
            # negative-side gridline
            accum.append([[-x_halfwidth, -y], [x_halfwidth, -y]])

        # draw the gridlines
        self.viewer.current_frame.add_lines(
            major_lines_accum,               # draw major gridlines
            linewidth=meters_per_pixel,      # roughly 1 pixel
            color="black",
            alpha=0.2)
        self.viewer.current_frame.add_lines(
            minor_lines_accum,               # draw minor gridlines
            linewidth=meters_per_pixel,      # roughly 1 pixel
            color="black",
            alpha=0.1)


#*************************************************
class MapManager(object):

    def __init__(self):
        self.current_obstacles = []
        self.current_goal = None
        self.geometrics = Geometrics()

    def random_map(self, world):
        # OBSTACLE PARAMS
        obs_min_dim = OBS_MIN_DIM
        obs_max_dim = OBS_MAX_DIM
        obs_max_combined_dim = OBS_MAX_COMBINED_DIM
        obs_min_count = OBS_MIN_COUNT
        obs_max_count = OBS_MAX_COUNT
        obs_min_dist = OBS_MIN_DIST
        obs_max_dist = OBS_MAX_DIST

        # GOAL PARAMS
        goal_min_dist = GOAL_MIN_DIST
        goal_max_dist = GOAL_MAX_DIST

        # BUILD RANDOM ELEMENTS
        # generate the goal
        goal_dist_range = goal_max_dist - goal_min_dist
        dist = goal_min_dist + (random() * goal_dist_range)
        phi = -pi + (random() * 2 * pi)
        x = dist * sin(phi)
        y = dist * cos(phi)
        goal = [x, y]

        # generate a proximity test geometry for the goal
        r = MIN_GOAL_CLEARANCE
        n = 6
        _goal_test_geometry = []
        for i in range(n):
            _goal_test_geometry.append([x + r * cos(i * 2 * pi / n),
                y + r * sin(i * 2 * pi / n)])
            goal_test_geometry = Polygon(_goal_test_geometry)

        # generate the obstacles
        obstacles = []
        obs_dim_range = obs_max_dim - obs_min_dim
        obs_dist_range = obs_max_dist - obs_min_dist
        num_obstacles = randrange(obs_min_count, obs_max_count + 1)

        test_geometries = [r.global_geometry for r in world.robots] + (
            [goal_test_geometry])
        while len(obstacles) < num_obstacles:

            # generate dimensions
            width = obs_min_dim + (random() * obs_dim_range)
            height = obs_min_dim + (random() * obs_dim_range)
            while width + height > obs_max_combined_dim:
                height = obs_min_dim + (random() * obs_dim_range)

            # generate position
            dist = obs_min_dist + (random() * obs_dist_range)
            phi = -pi + (random() * 2 * pi)
            x = dist * sin(phi)
            y = dist * cos(phi)

            # generate orientation
            theta = -pi + (random() * 2 * pi)

            # test if the obstacle overlaps the robots or the goal
            obstacle = RectangleObstacle(width, height, Pose(x, y, theta))
            intersects = False
            for test_geometry in test_geometries:
                intersects |= self.geometrics.convex_polygon_intersect_test(
                    test_geometry, obstacle.global_geometry)
            if intersects is False:
                obstacles.append(obstacle)

        # update the current obstacles and goal
        self.current_obstacles = obstacles
        self.current_goal = goal

        # apply the new obstacles and goal to the world
        self.apply_to_world(world)

    def custom_map(self, world):
        # OBSTACLE PARAMS
        obs_params = [{'w': 0.5, 'h': 1.2, 'x': 2.0, 'y': 0.0, 'deg': 0},
                      {'w': 0.3, 'h': 0.6, 'x': 1.0, 'y': 1.0, 'deg': 25}]

        # BUILD CUSTOM ELEMENTS
        # generate the goal
        x = GOAL_X_DIST
        y = GOAL_Y_DIST
        goal = [x, y]

        # generate a proximity test geometry for the goal
        r = MIN_GOAL_CLEARANCE
        n = 6
        _goal_test_geometry = []
        for i in range(n):
            _goal_test_geometry.append([x + r * cos(i * 2 * pi / n),
                y + r * sin(i * 2 * pi / n)])
            goal_test_geometry = Polygon(_goal_test_geometry)

        # generate the obstacles
        obstacles = []
        obs_index = 0

        # collect together robot and goal geometries to check obstacles
        #    are not on top of them
        test_geometries = [r.global_geometry for r in world.robots] + (
            [goal_test_geometry])

        while obs_index < len(obs_params):
            # convert orientation to radians
            theta = -pi + ((obs_params[obs_index]['deg'] / 360.0) * 2 * pi)

            # create the obstacle
            obstacle = RectangleObstacle(obs_params[obs_index]['w'],
                obs_params[obs_index]['h'], Pose(obs_params[obs_index]['x'],
                    obs_params[obs_index]['y'], theta))

            # test if the obstacle overlaps the robots or the goal
            intersects = False
            for test_geometry in test_geometries:
                intersects |= self.geometrics.convex_polygon_intersect_test(
                    test_geometry, obstacle.global_geometry)
            if intersects is False:
                obstacles.append(obstacle)
            else:
                print("Obstacle {} collides with robots or goal").format(
                    obs_index)

            obs_index += 1

        # update the current obstacles and goal
        self.current_obstacles = obstacles
        self.current_goal = goal

        # apply the new obstacles and goal to the world
        self.apply_to_world(world)

    def save_map(self, filename):
        with open(filename, 'wb') as _file:
            pickle.dump(self.current_obstacles, _file)
            pickle.dump(self.current_goal, _file)

    def load_map(self, filename, world):
        with open(filename, 'rb') as _file:
            self.current_obstacles = pickle.load(_file)
            self.current_goal = pickle.load(_file)

        # apply the loaded obstacles and goal to the world
        world.initialise_world(False, True)

    def apply_to_world(self, world):
        # add the current obstacles
        for obstacle in self.current_obstacles:
            world.add_obstacle(obstacle)

        # program the robot supervisors
        for robot in world.robots:
            robot.supervisor.goal = self.current_goal[:]


#****************************************************
class Obstacle(object):

    def __init__(self):
        pass


#****************************************************
class ObstacleView(object):

    def __init__(self, viewer, obstacle):
        self.viewer = viewer
        self.obstacle = obstacle

    def draw_obstacle_to_frame(self):
        obstacle = self.obstacle

        # grab the obstacle pose
        obstacle_pos, obstacle_theta = obstacle.pose.vunpack()

        # draw the obstacle to the frame
        obstacle_poly = obstacle.global_geometry.vertexes
        self.viewer.current_frame.add_polygons([obstacle_poly],
                                                color="dark red",
                                                alpha=0.4)

        # === FOR DEBUGGING: ===
        # self._draw_bounding_circle_to_frame()

    def _draw_bounding_circle_to_frame(self):
        c, r = self.obstacle.global_geometry.bounding_circle
        self.viewer.current_frame.add_circle(pos=c,
                                             radius=r,
                                             color="black",
                                             alpha=0.2)


#****************************************************
class RectangleObstacle(Obstacle):

    def __init__(self, width, height, pose):
        self.pose = pose
        self.width = width
        self.height = height

        # define the geometry
        halfwidth_x = width * 0.5
        halfwidth_y = height * 0.5
        vertexes = [[halfwidth_x, halfwidth_y],
                    [halfwidth_x, -halfwidth_y],
                    [-halfwidth_x, -halfwidth_y],
                    [-halfwidth_x, halfwidth_y]]
        self.geometry = Polygon(vertexes)
        self.global_geometry = Polygon(vertexes).get_transformation_to_pose(
            self.pose)


#****************************************************
class Goal(object):

    def __init__(self):
        pass


#**************************************************************
class ObstaclePositionException(Exception):
    pass