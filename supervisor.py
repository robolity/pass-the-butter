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

from math import radians, log, pi, sin, cos


from physics import Pose
from utilities import LinearAlgebra
from controllers import AvoidObstaclesController, AvoidObstaclesControllerView
from controllers import FollowWallController, FollowWallControllerView
from controllers import GoToAngleController
from controllers import GoToGoalController, GoToGoalControllerView
from controllers import GTGAndAOController, GTGAndAOControllerView

#**********CLASSES IN MODULE**********************

#Supervisor
#SupervisorView
#RobotSupervisorInterface
#SupervisorControllerInterface
#ControlState
#SupervisorStateMachine
#GoalReachedException

#*************************************************

# bring in linear algebra functions
linalg = LinearAlgebra()

#**********************************************************
# control parameters
R_TRANS_VEL_LIMIT = 0.3148     # m/s
R_ANG_VEL_LIMIT = 2.2763       # rad/s

#PID values
gtg_kP = 5.0
gtg_kI = 0.0
gtg_kD = 0.0

ao_kP = 10.0
ao_kI = 0.0
ao_kD = 0.0

fw_kP = 10.0
fw_kI = 0.0
fw_kD = 0.0

gta_kP = 5.0
gta_kI = 0.0
gta_kD = 0.0

gtgao_kP = 10.0
gtgao_kI = 0.0
gtgao_kD = 0.0


#**********************************************************
class Supervisor(object):

    def __init__(self,
                robot_interface,  # the interface through which this supervisor
                                  # will interact with the robot
                wheel_radius,     # the radius of a drive wheel on the robot
                wheel_base_length,  # the robot's wheel base
                wheel_encoder_ticks_per_rev,   # the number of wheel encoder
                                               # ticks per revolution of a
                                               # drive wheel
                sensor_placements,  # placement pose of the sensors on the
                                    # robot body
                sensor_range,       # max detection range of the sensors
                initial_pose=Pose(0.0, 0.0, 0.0),  # the pose the robot will
                                                   # have when control begins
                goal=[0.0, 0.0]):  # the goal to which this supervisor will
                                   # guide the robot

        # internal clock time in seconds
        self.supervisor_time = 0.0

        # robot representation
        # NOTE: the supervisor does NOT have access to the physical robot,
        #       only the robot's interface
        self.robot = robot_interface

        # proximity sensor information
        self.proximity_sensor_placements = [Pose(rawpose[0], rawpose[1],
            radians(rawpose[2])) for rawpose in sensor_placements]
        self.proximity_sensor_max_range = sensor_range

        # odometry information
        self.robot_wheel_radius = wheel_radius
        self.robot_wheel_base_length = wheel_base_length
        self.wheel_encoder_ticks_per_revolution = wheel_encoder_ticks_per_rev
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        # controllers
        controller_interface = SupervisorControllerInterface(self)
        self.go_to_angle_controller = GoToAngleController(controller_interface,
            gta_kP, gta_kI, gta_kD)
        self.go_to_goal_controller = GoToGoalController(controller_interface,
             gtg_kP, gtg_kI, gtg_kD)
        self.avoid_obstacles_controller = (
            AvoidObstaclesController(controller_interface, ao_kP, ao_kI, ao_kD))
        self.gtg_and_ao_controller = GTGAndAOController(controller_interface,
             gtgao_kP, gtgao_kI, gtgao_kD)
        self.follow_wall_controller = FollowWallController(controller_interface,
             fw_kP, fw_kI, fw_kD)

        # state machine
        self.state_machine = SupervisorStateMachine(self)

        # state
        # - sensor distances
        self.proximity_sensor_distances = [0.0, 0.0] * len(sensor_placements)
        # estimated pose
        self.estimated_pose = initial_pose
        # current controller
        self.current_controller = self.go_to_goal_controller

        # goal
        self.goal = goal

        # control bounds
        self.v_max = self.robot.trans_vel_limit
        self.omega_max = self.robot.ang_vel_limit

        # CONTROL OUTPUTS - UNICYCLE
        self.v_output = 0.0
        self.omega_output = 0.0

    # simulate this supervisor running for one time increment
    def step(self, dt):
        # increment the internal clock time
        self.supervisor_time += dt
        # print("super time:{}").format(self.supervisor_time)

        # NOTE: for simplicity, we assume that the onboard computer executes
        #       exactly one control loop for every simulation time increment
        #       although technically this is not likely to be realistic, it
        #       is a good simplificiation
        # execute one full control loop
        self.execute()

    # execute one control loop
    def execute(self):
        self._update_state()               # update state
        self.current_controller.execute()  # apply the current controller

        # output the generated control signals to the robot
        self._send_robot_commands()

    # update the estimated robot state and the control state
    def _update_state(self):
        # update estimated robot state from sensor readings
        self._update_proximity_sensor_distances()
        self._update_odometry()

        # calculate new heading vectors for each controller
        self._update_controller_headings()

        # update the control state
        self.state_machine.update_state()

    # calculate updated heading vectors for the active controllers
    def _update_controller_headings(self):
        self.go_to_goal_controller.update_heading()
        self.avoid_obstacles_controller.update_heading()
        self.gtg_and_ao_controller.update_heading()
        self.follow_wall_controller.update_heading()

    # update the distances indicated by the proximity sensors
    def _update_proximity_sensor_distances(self):
        self.proximity_sensor_distances = [0.02 - (log(readval / 3960.0)) / 30.0
            for readval in self.robot.read_proximity_sensors()]

    # update the estimated position of the robot using it's wheel encoder
    #    readings
    def _update_odometry(self):
        R = self.robot_wheel_radius
        N = float(self.wheel_encoder_ticks_per_revolution)

        # read the wheel encoder values
        ticks_left, ticks_right = self.robot.read_wheel_encoders()

        # get the difference in ticks since the last iteration
        d_ticks_left = ticks_left - self.prev_ticks_left
        d_ticks_right = ticks_right - self.prev_ticks_right

        # estimate the wheel movements
        d_left_wheel = 2 * pi * R * (d_ticks_left / N)
        d_right_wheel = 2 * pi * R * (d_ticks_right / N)
        d_center = 0.5 * (d_left_wheel + d_right_wheel)

        # calculate the new pose
        prev_x, prev_y, prev_theta = self.estimated_pose.sunpack()
        new_x = prev_x + (d_center * cos(prev_theta))
        new_y = prev_y + (d_center * sin(prev_theta))
        new_theta = prev_theta + ((d_right_wheel - d_left_wheel)
            / self.robot_wheel_base_length)

        # update the pose estimate with the new values
        self.estimated_pose.supdate(new_x, new_y, new_theta)

        # save the current tick count for the next iteration
        self.prev_ticks_left = ticks_left
        self.prev_ticks_right = ticks_right

    # generate and send the correct commands to the robot
    def _send_robot_commands(self):
        # limit the speeds:
        v = max(min(self.v_output, self.v_max), -self.v_max)
        omega = max(min(self.omega_output, self.omega_max), -self.omega_max)

        # send the drive commands to the robot
        v_l, v_r = self._uni_to_diff(v, omega)

        self.robot.set_wheel_drive_rates(v_l, v_r)

    def _uni_to_diff(self, v, omega):
        # v = translational velocity (m/s)
        # omega = angular velocity (rad/s)

        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v_l = ((2.0 * v) - (omega * L)) / (2.0 * R)
        v_r = ((2.0 * v) + (omega * L)) / (2.0 * R)

        return v_l, v_r

    def _diff_to_uni(self, v_l, v_r):
        # v_l = left-wheel angular velocity (rad/s)
        # v_r = right-wheel angular velocity (rad/s)

        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v = (R / 2.0) * (v_r + v_l)
        omega = (R / L) * (v_r - v_l)

        return v, omega


#**********************************************************
class SupervisorView(object):

    def __init__(self, viewer, supervisor, robot_geometry):
        self.viewer = viewer
        self.supervisor = supervisor
        self.supervisor_state_machine = supervisor.state_machine

        # controller views
        self.go_to_goal_controller_view = (
            GoToGoalControllerView(viewer, supervisor))
        self.avoid_obstacles_controller_view = (
            AvoidObstaclesControllerView(viewer, supervisor))
        self.gtg_and_ao_controller_view = (
            GTGAndAOControllerView(viewer, supervisor))
        self.follow_wall_controller_view = (
            FollowWallControllerView(viewer, supervisor))

        # additional information for rendering
        # - robot geometry
        self.robot_geometry = robot_geometry
        # - path taken by robot's internal image
        self.robot_estimated_traverse_path = []

    # draw a representation of the supervisor's internal state to the frame
    def draw_supervisor_to_frame(self):
        # update the estimated robot traverse path
        self.robot_estimated_traverse_path.append(
            self.supervisor.estimated_pose.vposition())

        # draw the goal to frame
        self._draw_goal_to_frame()

        # draw the supervisor-generated data to frame if indicated
        if self.viewer.draw_invisibles:
            self._draw_robot_state_estimate_to_frame()
            self._draw_current_controller_to_frame()

        # === FOR DEBUGGING ===
        # self._draw_all_controllers_to_frame()

    def _draw_goal_to_frame(self):
        goal = self.supervisor.goal
        self.viewer.current_frame.add_circle(pos=goal,
                                             radius=0.05,
                                             color="dark green",
                                             alpha=0.65)
        self.viewer.current_frame.add_circle(pos=goal,
                                             radius=0.01,
                                             color="black",
                                             alpha=0.5)

    def _draw_robot_state_estimate_to_frame(self):
        # draw the estimated position of the robot
        vertexes = self.robot_geometry.vertexes[:]
        vertexes.append(vertexes[0])  # close the drawn polygon
        self.viewer.current_frame.add_lines([vertexes],
                                            color="black",
                                            linewidth=0.0075,
                                            alpha=0.5)

        # draw the estimated traverse path of the robot
        self.viewer.current_frame.add_lines(
            [self.robot_estimated_traverse_path],
            linewidth=0.005,
            color="red",
            alpha=0.5)

    # draw the current controller's state to the frame
    def _draw_current_controller_to_frame(self):
        current_state = self.supervisor_state_machine.current_state
        if current_state == ControlState.GO_TO_GOAL:
            (self.go_to_goal_controller_view.
                draw_go_to_goal_controller_to_frame())
        elif current_state == ControlState.AVOID_OBSTACLES:
            (self.avoid_obstacles_controller_view.
                draw_avoid_obstacles_controller_to_frame())
        elif current_state == ControlState.GTG_AND_AO:
            (self.gtg_and_ao_controller_view.
                draw_gtg_and_ao_controller_to_frame())
        elif current_state in [ControlState.SLIDE_LEFT,
                ControlState.SLIDE_RIGHT]:
            (self.follow_wall_controller_view.
                draw_active_follow_wall_controller_to_frame())

    # draw all of the controllers's to the frame
    def _draw_all_controllers_to_frame(self):
        self.go_to_goal_controller_view.draw_go_to_goal_controller_to_frame()
        (self.avoid_obstacles_controller_view.
            draw_avoid_obstacles_controller_to_frame())
        # self.gtg_and_ao_controller_view.draw_gtg_and_ao_controller_to_frame()
        (self.follow_wall_controller_view.
            draw_complete_follow_wall_controller_to_frame())


#**************************************************************
# a class representing the available interactions a supervisor may have
#    with a robot
class RobotSupervisorInterface(object):

    def __init__(self, robot):
        self.robot = robot
        self.trans_vel_limit = self.robot.trans_vel_limit
        self.ang_vel_limit = self.robot.ang_vel_limit

    # read the proximity sensors
    def read_proximity_sensors(self):
        return [s.read() for s in self.robot.ir_sensors]

    # read the wheel encoders
    def read_wheel_encoders(self):
        return [e.read() for e in self.robot.wheel_encoders]

    # apply wheel drive command
    def set_wheel_drive_rates(self, v_l, v_r):
        self.robot.set_wheel_drive_rates(v_l, v_r)


#**********************************************************
# an interfacing allowing a controller to interact with its supervisor
class SupervisorControllerInterface(object):

    def __init__(self, supervisor):
        self.supervisor = supervisor

    # get the current control state
    def current_state(self):
        return self.supervisor.state_machine.current_state

    # get the supervisor's internal pose estimation
    def estimated_pose(self):
        return self.supervisor.estimated_pose

    # get the placement poses of the robot's sensors
    def proximity_sensor_placements(self):
        return self.supervisor.proximity_sensor_placements

    # get the robot's proximity sensor read values converted to real
    #    distances in meters
    def proximity_sensor_distances(self):
        return self.supervisor.proximity_sensor_distances

    # get true/false indicators for which sensors are actually detecting
    #    obstacles
    def proximity_sensor_positive_detections(self):
        sensor_range = self.supervisor.proximity_sensor_max_range
        return [d < sensor_range - 0.001
            for d in self.proximity_sensor_distances()]

    # get the velocity limit of the supervisor
    def v_max(self):
        return self.supervisor.v_max

    # get the supervisor's goal
    def goal(self):
        return self.supervisor.goal

    # get the supervisor's internal clock time
    def time(self):
        return self.supervisor.supervisor_time

    # set the outputs of the supervisor
    def set_outputs(self, v, omega):
        self.supervisor.v_output = v
        self.supervisor.omega_output = omega


#**********************************************************
# a simple enumeration of control states
class ControlState(object):
    AT_GOAL = 0
    GO_TO_GOAL = 1
    AVOID_OBSTACLES = 2
    GTG_AND_AO = 3
    SLIDE_LEFT = 4
    SLIDE_RIGHT = 5


#**********************************************************
# event parameters
D_STOP = 0.05     # meters from goal
D_CAUTION = 0.15  # meters from obstacle
D_DANGER = 0.04   # meters from obstacle

# progress margin
PROGRESS_EPSILON = 0.05


class SupervisorStateMachine(object):

    def __init__(self, supervisor):
        self.supervisor = supervisor

        # initialize state
        self.transition_to_state_go_to_goal()

        # progress tracking
        self.best_distance_to_goal = float("inf")

    def update_state(self):
        #check what the previous state is, execute it to test if should
        #    transition to a different state
        if self.current_state == ControlState.GO_TO_GOAL:
            self.execute_state_go_to_goal()
        elif self.current_state == ControlState.AVOID_OBSTACLES:
            self.execute_state_avoid_obstacles()
        elif self.current_state == ControlState.SLIDE_LEFT:
            self.execute_state_slide_left()
        elif self.current_state == ControlState.SLIDE_RIGHT:
            self.execute_state_slide_right()
        else:
            raise Exception("undefined supervisor state or behavior")

    # === STATE PROCEDURES ===
    def execute_state_go_to_goal(self):
        if self.condition_at_goal():
            self.transition_to_state_at_goal()
        elif self.condition_danger():
            self.transition_to_state_avoid_obstacles()
        elif self.condition_at_obstacle():
            sl = self.condition_slide_left()
            sr = self.condition_slide_right()
            if sl and not sr:
                self.transition_to_state_slide_left()
            elif sr and not sl:
                self.transition_to_state_slide_right()
            # elif sl and sr: raise Exception("cannot determine
            #    slide direction")

    def execute_state_avoid_obstacles(self):
        if self.condition_at_goal():
            self.transition_to_state_at_goal()
        elif not self.condition_danger():
            sl = self.condition_slide_left()
            sr = self.condition_slide_right()
            if sl and not sr:
                self.transition_to_state_slide_left()
            elif sr and not sl:
                self.transition_to_state_slide_right()
            elif not sr and not sl:
                self.transition_to_state_go_to_goal()
            # else: raise Exception( "cannot determine slide direction" )

    def execute_state_slide_left(self):
        if self.condition_at_goal():
            self.transition_to_state_at_goal()
        elif self.condition_danger():
            self.transition_to_state_avoid_obstacles()
        elif self.condition_progress_made() and not self.condition_slide_left():
            self.transition_to_state_go_to_goal()

    def execute_state_slide_right(self):
        if self.condition_at_goal():
            self.transistion_to_state_at_goal()
        elif self.condition_danger():
            self.transition_to_state_avoid_obstacles()
        elif (self.condition_progress_made() and
                not self.condition_slide_right()):
            self.transition_to_state_go_to_goal()

    # def execute_state_gtg_and_ao( self ):
    #   if self.condition_at_goal():
    #        self.transition_to_state_at_goal()
    #   elif self.condition_danger():
    #        self.transition_to_state_avoid_obstacles()
    #   elif self.condition_no_obstacle():
    #        self.transition_to_state_go_to_goal()

    # === STATE TRANSITIONS ===
    def transition_to_state_at_goal(self):
        # set linear & angular velocity to zero to stop robot at goal
        self.supervisor.v_output = 0
        self.supervisor.omega_output = 0

        self.current_state = ControlState.AT_GOAL
        raise GoalReachedException("Goal!")

    def transition_to_state_avoid_obstacles(self):
        self.current_state = ControlState.AVOID_OBSTACLES
        self.supervisor.current_controller = (
            self.supervisor.avoid_obstacles_controller)

    def transition_to_state_go_to_goal(self):
        self.current_state = ControlState.GO_TO_GOAL
        self.supervisor.current_controller = (
            self.supervisor.go_to_goal_controller)

    def transition_to_state_slide_left(self):
        self.current_state = ControlState.SLIDE_LEFT
        self._update_best_distance_to_goal()
        self.supervisor.current_controller = (
            self.supervisor.follow_wall_controller)

    def transition_to_state_slide_right(self):
        self.current_state = ControlState.SLIDE_RIGHT
        self._update_best_distance_to_goal()
        self.supervisor.current_controller = (
            self.supervisor.follow_wall_controller)

    def transition_to_state_gtg_and_ao(self):
        self.current_state = ControlState.GTG_AND_AO
        self.supervisor.current_controller = (
            self.supervisor.gtg_and_ao_controller)

    # === CONDITIONS ===
    def condition_at_goal(self):
        return linalg.distance(self.supervisor.estimated_pose.vposition(),
            self.supervisor.goal) < D_STOP

    def condition_at_obstacle(self):
        for d in self._forward_sensor_distances():
            if d < D_CAUTION:
                return True
        return False

    def condition_danger(self):
        for d in self._forward_sensor_distances():
            if d < D_DANGER:
                return True
        return False

    def condition_no_obstacle(self):
        for d in self._forward_sensor_distances():
            if d < D_CAUTION:
                return False
        return True

    def condition_progress_made(self):
        return (self._distance_to_goal() <
            self.best_distance_to_goal - PROGRESS_EPSILON)

    def condition_slide_left(self):
        heading_gtg = self.supervisor.go_to_goal_controller.gtg_heading_vector
        heading_ao = (self.supervisor.avoid_obstacles_controller.
                            ao_heading_vector)
        heading_fwl = self.supervisor.follow_wall_controller.l_fw_heading_vector

        ao_cross_fwl = linalg.cross(heading_ao, heading_fwl)
        fwl_cross_gtg = linalg.cross(heading_fwl, heading_gtg)
        ao_cross_gtg = linalg.cross(heading_ao, heading_gtg)

        return((ao_cross_gtg > 0.0 and ao_cross_fwl > 0.0 and
                    fwl_cross_gtg > 0.0) or
                (ao_cross_gtg <= 0.0 and ao_cross_fwl <= 0.0 and
                    fwl_cross_gtg <= 0.0))

    def condition_slide_right(self):
        heading_gtg = self.supervisor.go_to_goal_controller.gtg_heading_vector
        heading_ao = (self.supervisor.avoid_obstacles_controller.
                            ao_heading_vector)
        heading_fwr = self.supervisor.follow_wall_controller.r_fw_heading_vector

        ao_cross_fwr = linalg.cross(heading_ao, heading_fwr)
        fwr_cross_gtg = linalg.cross(heading_fwr, heading_gtg)
        ao_cross_gtg = linalg.cross(heading_ao, heading_gtg)

        return((ao_cross_gtg > 0.0 and ao_cross_fwr > 0.0 and
                    fwr_cross_gtg > 0.0) or
                (ao_cross_gtg <= 0.0 and ao_cross_fwr <= 0.0 and
                    fwr_cross_gtg <= 0.0))

    # === helper methods ===
    def _forward_sensor_distances(self):
        return self.supervisor.proximity_sensor_distances[1:7]

    def _distance_to_goal(self):
        return linalg.distance(self.supervisor.estimated_pose.vposition(),
            self.supervisor.goal)

    def _update_best_distance_to_goal(self):
        self.best_distance_to_goal = min(self.best_distance_to_goal,
            self._distance_to_goal())

    # === FOR DEBUGGING ===
    def _print_debug_info(self):
        print ("\n ======== \n")
        print ("STATE: " + str(["At Goal", "Go to Goal", "Avoid Obstacles",
            "Blended", "Slide Left", "Slide Right"][self.current_state]))
        print ("")
        print ("CONDITIONS:")
        print ("At Obstacle: " + str(self.condition_at_obstacle()))
        print ("Danger: " + str(self.condition_danger()))
        print ("No Obstacle: " + str(self.condition_no_obstacle()))
        print ("Progress Made: " + str(self.condition_progress_made())
            + " ( Best Dist: " + str(round(self.best_distance_to_goal, 3))
            + ", Current Dist: " + str(round(self._distance_to_goal(), 3))
            + " )")
        print ("Slide Left: " + str(self.condition_slide_left()))
        print ("Slide Right: " + str(self.condition_slide_right()))


#**************************************************************
class GoalReachedException(Exception):
    pass