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
    """Class to act as communication interface between Robot and Controllers

    Attributes:
        supervisor_time -> float
        robot -> RobotSupervisorInterface object
        proximity_sensor_placements -> list of Pose objects
        proximity_sensor_max_range -> float
        robot_wheel_radius -> float
        robot_wheel_base_length -> float
        wheel_encoder_ticks_per_revolution -> float
        prev_ticks_left -> int
        prev_ticks_right -> int
        go_to_angle_controller -> GoToAngleController object
        go_to_goal_controller -> GoToGoalController object
        avoid_obstacles_controller -> AvoidObstaclesController object
        gtg_and_ao_controller -> GTGAndAOController object
        follow_wall_controller -> FollowWallController object
        state_machine -> SupervisorStateMachine object
        proximity_sensor_distances -> list
        estimated_pose -> Pose object
        current_controller -> Controller object
        goal -> list (2D location)
        v_max -> float
        omega_max -> float
        v_output -> float
        omega_output -> float

    Methods:
        __init__(robot_interface, wheel_radius, wheel_base_length,
            wheel_encoder_ticks_per_rev, sensor_placements, sensor_range,
            initial_pose=Pose(0.0, 0.0, 0.0), goal=[0.0, 0.0])
        step(dt)
        execute()
        _update_state()
        _update_controller_headings
        _update_proximity_sensor_distances()
        _update_odometry()
        _send_robot_commands()
        _uni_to_diff(v, omega)
        _diff_to_uni(v_l, v_r)
    """
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
        """Sets up a supervisor to interface between robot and controllers."""
        # internal clock time in seconds
        self.supervisor_time = 0.0

        # robot representation
        # NOTE: the supervisor does NOT have access to the physical robot,
        #       only the robot's interface
        self.robot_interface = robot_interface

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
        self.encoder_values = [0, 0]

        # estimated pose
        self.estimated_pose = initial_pose

        # current controller
        self.current_controller = self.go_to_goal_controller

        # goal
        self.goal = goal

        # control bounds
        self.v_max = self.robot_interface.trans_vel_limit
        self.omega_max = self.robot_interface.ang_vel_limit

        # CONTROL OUTPUTS - UNICYCLE
        self.v_output = 0.0
        self.omega_output = 0.0

    def step(self, dt):
        """Simulate this supervisor running for one time increment."""
        # increment the internal clock time
        self.supervisor_time += dt
        print("super time:{}").format(self.supervisor_time)

        # NOTE: for simplicity, we assume that the onboard computer executes
        #       exactly one control loop for every simulation time increment
        #       although technically this is not likely to be realistic, it
        #       is a good simplificiation
        # execute one full control loop
        self.execute()

    def execute(self):
        """Execute one control loop."""
        self._update_state()               # update state
        self.current_controller.execute()  # apply the current controller

        # output the generated control signals to the robot
        self._send_robot_commands()

    def _update_state(self):
        """update the estimated robot state and the control state."""
        # update estimated robot state from sensor readings
        self._update_proximity_sensor_distances()
        self._update_odometry()

        # calculate new heading vectors for each controller
        self._update_controller_headings()

        # update the control state
        self.state_machine.update_state()

        # show current state on the GUI
        self.robot_interface.robot.viewer._label_controller_state.set_text(
            str(self.state_machine.current_state))

    def _update_controller_headings(self):
        """Calculate updated heading vectors for the active controllers."""
        self.go_to_goal_controller.update_heading()
        self.avoid_obstacles_controller.update_heading()
        self.gtg_and_ao_controller.update_heading()
        self.follow_wall_controller.update_heading()

    def _update_proximity_sensor_distances(self):
        """Update the distances indicated by the proximity sensors."""
        self.proximity_sensor_distances = [0.02 - (log(readval / 3960.0)) / 30.0
            for readval in self.robot_interface.read_proximity_sensors()]

    def update_encoder_values(self):
        self.encoder_values = [readval
            for readval in self.robot_interface.read_wheel_encoders()]

    def _update_odometry(self):
        """Update estimated position of robot using wheel encoder readings."""
        R = self.robot_wheel_radius
        N = float(self.wheel_encoder_ticks_per_revolution)

        # read the wheel encoder values
        self.update_encoder_values()
        ticks_left, ticks_right = self.encoder_values

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

    def _send_robot_commands(self):
        """Generate and send the correct commands to the robot."""
        # limit the speeds:
        v = max(min(self.v_output, self.v_max), -self.v_max)
        omega = max(min(self.omega_output, self.omega_max), -self.omega_max)

        # send the drive commands to the robot
        v_l, v_r = self._uni_to_diff(v, omega)

        self.robot_interface.set_wheel_drive_rates(v_l, v_r)

    def _uni_to_diff(self, v, omega):
        """Convert translation vel and angular vel to left and right wheel vel.

        v = translational velocity (m/s)
        omega = angular velocity (rad/s)
        """
        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v_l = ((2.0 * v) - (omega * L)) / (2.0 * R)
        v_r = ((2.0 * v) + (omega * L)) / (2.0 * R)

        return v_l, v_r

    def _diff_to_uni(self, v_l, v_r):
        """Convert left and right wheel vel to translation vel and angular vel.

        v_l = left-wheel angular velocity (rad/s)
        v_r = right-wheel angular velocity (rad/s)
        """
        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v = (R / 2.0) * (v_r + v_l)
        omega = (R / L) * (v_r - v_l)

        return v, omega


#**********************************************************
class SupervisorView(object):
    """Draws the robot's heading vectors and avoided obstacle boundaries

    Attributes:
        viewer -> Viewer object
        supervisor -> Supervisor object
        supervisor_state_machine -> SupervisorStateMachine object
        go_to_goal_controller_view -> GoToGoalController object
        avoid_obstacles_controller_view -> AvoidObstaclesController object
        gtg_and_ao_controller_view -> GTGAndAOController object
        follow_wall_controller_view -> FollowWallController object
        robot_geometry -> Polygon object
        robot_estimated_traverse_path -> list

    Methods:
        __init__(viewer, supervisor, robot_geometry)
        draw_go_to_goal_controller_to_frame()
    """
    def __init__(self, viewer, supervisor, robot_geometry):
        """Bind viewer, supervisor and robot_geometry and setup controller views

        Keywords:
            viewer -> Viewer
            supervisor -> Supervisor object
            robot_geometry -> Geometry object
        """
        self.viewer = viewer
        self.supervisor = supervisor
        self.supervisor_state_machine = supervisor.state_machine

        # controller views
        self.go_to_goal_controller_view = GoToGoalControllerView(
            viewer, supervisor)
        self.avoid_obstacles_controller_view = AvoidObstaclesControllerView(
            viewer, supervisor)
        self.gtg_and_ao_controller_view = GTGAndAOControllerView(
            viewer, supervisor)
        self.follow_wall_controller_view = FollowWallControllerView(
            viewer, supervisor)

        # additional information for rendering
        # - robot geometry
        self.robot_geometry = robot_geometry
        # - path taken by robot's internal image
        self.robot_estimated_traverse_path = []

    # draw a representation of the supervisor's internal state to the frame
    def draw_supervisor_to_frame(self):
        """Draws goal and optionally traversed path and current controller."""
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
        """Adds shapes to the viewer to represent the goal."""
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
        """Adds shapes to viewer to represent the robot and traversed path."""
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

    def _draw_current_controller_to_frame(self):
        """Draws the current controller's state to the frame."""
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

    def _draw_all_controllers_to_frame(self):
        """Draws the state of all the controllers to the frame."""
        self.go_to_goal_controller_view.draw_go_to_goal_controller_to_frame()
        (self.avoid_obstacles_controller_view.
            draw_avoid_obstacles_controller_to_frame())
        # self.gtg_and_ao_controller_view.draw_gtg_and_ao_controller_to_frame()
        (self.follow_wall_controller_view.
            draw_complete_follow_wall_controller_to_frame())


#**************************************************************t
class RobotSupervisorInterface(object):
    """Represents available interactions a supervisor may have with a robot.

    Attributes:
        robot -> Robot object
        trans_vel_limit -> float
        ang_vel_limit -> float

    Methods:
        __init__(robot)
        read_proximity_sensors()
        read_wheel_encoders()
        set_wheel_drive_rates()
    """
    def __init__(self, robot):
        """Binds the robot and sets it's speed limits."""
        self.robot = robot
        self.trans_vel_limit = self.robot.trans_vel_limit
        self.ang_vel_limit = self.robot.ang_vel_limit

    def read_proximity_sensors(self):
        """Read the proximity sensors."""
        return [s.read() for s in self.robot.proximity_sensors]

    def read_wheel_encoders(self):
        """Read the wheel encoders."""
        self.robot.physical_robot.read_wheel_encoders()
        return [e.read() for e in self.robot.wheel_encoders]

    def set_wheel_drive_rates(self, v_l, v_r):
        """Apply wheel drive command."""
        self.robot.set_wheel_drive_rates(v_l, v_r)


#**********************************************************
class SupervisorControllerInterface(object):
    """An interfacing allowing a controller to interact with its supervisor.

    Attributes:
        supervisor -> Supervisor object
        trans_vel_limit -> float
        ang_vel_limit -> float

    Methods:
        __init__(robot)
        read_proximity_sensors()
        read_wheel_encoders()
        set_wheel_drive_rates()
    """
    def __init__(self, supervisor):
        """Bind the supervisor."""
        self.supervisor = supervisor

    def current_state(self):
        """Get the current control state."""
        return self.supervisor.state_machine.current_state

    def estimated_pose(self):
        """Get the supervisor's internal pose estimation."""
        return self.supervisor.estimated_pose

    def proximity_sensor_placements(self):
        """Get the placement poses of the robot's sensors."""
        return self.supervisor.proximity_sensor_placements

    def proximity_sensor_distances(self):
        """Get the robot's proximity sensor read values.

        These are returned as real distances in meters.
        """
        return self.supervisor.proximity_sensor_distances

    def proximity_sensor_positive_detections(self):
        """Get true/false indicate for which sensors are detecting obstacles."""
        sensor_range = self.supervisor.proximity_sensor_max_range
        return [d < sensor_range - 0.001
            for d in self.proximity_sensor_distances()]

    def v_max(self):
        """Get the velocity limit of the supervisor."""
        return self.supervisor.v_max

    def goal(self):
        """Get the supervisor's goal."""
        return self.supervisor.goal

    def time(self):
        """Get the supervisor's internal clock time."""
        return self.supervisor.supervisor_time

    def set_outputs(self, v, omega):
        """Set the outputs of the supervisor."""
        self.supervisor.v_output = v
        self.supervisor.omega_output = omega


#**********************************************************
class ControlState(object):
    """A simple enumeration of control states."""
    AT_GOAL = 0
    GO_TO_GOAL = 1
    AVOID_OBSTACLES = 2
    GTG_AND_AO = 3
    SLIDE_LEFT = 4
    SLIDE_RIGHT = 5


#**********************************************************
# event parameters
D_STOP = 0.05     # meters from goal
D_CAUTION = 0.18  # meters from obstacle
D_DANGER = 0.08   # meters from obstacle

# progress margin
PROGRESS_EPSILON = 0.05


class SupervisorStateMachine(object):
    """A state machine to handle when and how to tranisition between controllers

    Attributes:
        supervisor -> Supervisor object
        best_distance_to_goal -> float
        current_state -> ControlState object

    Methods:
        __init__(robot)
        read_proximity_sensors()
        read_wheel_encoders()
        set_wheel_drive_rates()
    """
    def __init__(self, supervisor):
        """Bind the supervisor, and set initial state to Go to Goal.

        Keywords:
            supervisor -> Supervisor object
        """
        self.supervisor = supervisor

        # initialize state
        self.transition_to_state_go_to_goal()

        # progress tracking
        self.best_distance_to_goal = float("inf")

    def update_state(self):
        """Based on current state, test if should transition to a new state."""
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
        """Test if still clear of obstacles, but change state if obstacles."""
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
        """Test if now clear of obstacles, and if so change to GTG state."""
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
        """Test if now clear of obstacles, or keep sliding left if obstacle."""
        if self.condition_at_goal():
            self.transition_to_state_at_goal()
        elif self.condition_danger():
            self.transition_to_state_avoid_obstacles()
        elif self.condition_progress_made() and not self.condition_slide_left():
            self.transition_to_state_go_to_goal()

    def execute_state_slide_right(self):
        """Test if now clear of obstacles, or keep sliding right if obstacle."""
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
        """Sets linear & angular velocity to zero to stop robot at goal."""
        self.supervisor.v_output = 0
        self.supervisor.omega_output = 0

        self.current_state = ControlState.AT_GOAL
        raise GoalReachedException("Goal!")

    def transition_to_state_avoid_obstacles(self):
        """Change state and controller to avoid obstacles."""
        self.current_state = ControlState.AVOID_OBSTACLES
        self.supervisor.current_controller = (
            self.supervisor.avoid_obstacles_controller)

    def transition_to_state_go_to_goal(self):
        """Change state and controller to go to goal."""
        self.current_state = ControlState.GO_TO_GOAL
        self.supervisor.current_controller = (
            self.supervisor.go_to_goal_controller)

    def transition_to_state_slide_left(self):
        """Change state to slide left and controller to follow wall."""
        self.current_state = ControlState.SLIDE_LEFT
        self._update_best_distance_to_goal()
        self.supervisor.current_controller = (
            self.supervisor.follow_wall_controller)

    def transition_to_state_slide_right(self):
        """Change state to slide right and controller to follow wall."""
        self.current_state = ControlState.SLIDE_RIGHT
        self._update_best_distance_to_goal()
        self.supervisor.current_controller = (
            self.supervisor.follow_wall_controller)

    def transition_to_state_gtg_and_ao(self):
        """Change state and controller to GTG and AO hybrid."""
        self.current_state = ControlState.GTG_AND_AO
        self.supervisor.current_controller = (
            self.supervisor.gtg_and_ao_controller)

    # === CONDITIONS ===
    def condition_at_goal(self):
        """Returns True if robot has reached the goal."""
        return linalg.distance(self.supervisor.estimated_pose.vposition(),
            self.supervisor.goal) < D_STOP

    def condition_at_obstacle(self):
        """Returns True if the proximity sensors can just see an obstacle."""
        for d in self._forward_sensor_distances():
            if d < D_CAUTION:
                return True
        return False

    def condition_danger(self):
        """Returns True if the proximity sensors are close to an obstacle."""
        for d in self._forward_sensor_distances():
            if d < D_DANGER:
                return True
        return False

    def condition_progress_made(self):
        """Returns True if the robot is getting closer to the goal."""
        return (self._distance_to_goal() <
            self.best_distance_to_goal - PROGRESS_EPSILON)

    def condition_slide_left(self):
        """When at wall, decide if should go left if closer to goal."""
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
        """When at wall, decide if should go left if closer to goal."""
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
        """Returns the distance readings of each of the proximity sensors."""
        return self.supervisor.proximity_sensor_distances[1:7]

    def _distance_to_goal(self):
        """Returns the straight line distance to the goal."""
        return linalg.distance(self.supervisor.estimated_pose.vposition(),
            self.supervisor.goal)

    def _update_best_distance_to_goal(self):
        """Updates best distance to goal as current distance to goal changes."""
        self.best_distance_to_goal = min(self.best_distance_to_goal,
            self._distance_to_goal())

    # === FOR DEBUGGING ===
    def _print_debug_info(self):
        """Prints degub readout of current state of robot."""
        print ("\n ======== \n")
        print ("STATE: " + str(["At Goal", "Go to Goal", "Avoid Obstacles",
            "Blended", "Slide Left", "Slide Right"][self.current_state]))
        print ("")
        print ("CONDITIONS:")
        print ("At Obstacle: " + str(self.condition_at_obstacle()))
        print ("Danger: " + str(self.condition_danger()))
        print ("No Obstacle: " + str(not(self.condition_at_obstacle())))
        print ("Progress Made: " + str(self.condition_progress_made())
            + " ( Best Dist: " + str(round(self.best_distance_to_goal, 3))
            + ", Current Dist: " + str(round(self._distance_to_goal(), 3))
            + " )")
        print ("Slide Left: " + str(self.condition_slide_left()))
        print ("Slide Right: " + str(self.condition_slide_right()))


#**************************************************************
class GoalReachedException(Exception):
    """Exception when the goal is reached, no actions taken in class."""
    pass