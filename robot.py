# -*- coding: utf-8 -*-import time

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

import serial
import serial.tools.list_ports
import threading
import time
from math import pi, radians, cos, sin, ceil, e, fabs

from physics import Polygon, LineSegment, Pose
from utilities import LinearAlgebra
from supervisor import Supervisor, SupervisorView, RobotSupervisorInterface

#**********CLASSES IN MODULE**********************

#Robot
#RobotView
#RobotPhysicalInterface
#RobotComm
#Sensor
#ProximitySensor
#ProximitySensorView
#WheelEncoder
#DifferentialDriveDynamics

#*************************************************

# bring in linear algebra functions
linalg = LinearAlgebra()

#*************************************************

debug = True
use_serial = False

# Robot's Physical Properties
R_WHEEL_RADIUS = 0.0194        # meters
R_WHEEL_BASE_LENGTH = 0.0885   # meters
R_WHEEL_TICKS_PER_REV = 909.7
R_MAX_WHEEL_DRIVE_RATE = 100  # rpm

# Robot's Physical Dimensions
R_BOTTOM_PLATE = ([[-0.024, 0.064],
                  [0.033, 0.064],
                  [0.057, 0.043],
                  [0.074, 0.010],
                  [0.074, -0.010],
                  [0.057, -0.043],
                  [0.033, -0.064],
                 [-0.025, -0.064],
                 [-0.042, -0.043],
                 [-0.048, -0.010],
                 [-0.048, 0.010],
                 [-0.042, 0.043]])

R_TOP_PLATE = ([[-0.031, 0.043],
                [-0.031, -0.043],
                 [0.033, -0.043],
                 [0.052, -0.021],
                 [0.057, 0.000],
                 [0.052, 0.021],
                 [0.033, 0.043]])

# Sensor read value limits
R_SENSOR_MIN_READ_VALUE = 18
R_SENSOR_MAX_READ_VALUE = 3960

R_SENSOR_MIN_RANGE = 0.02
R_SENSOR_MAX_RANGE = 0.2

R_SENSOR_POSES = ([[-0.038, 0.048, 128],  # x, y, theta_degrees
                   [0.019, 0.064, 75],
                   [0.050, 0.050, 42],
                   [0.070, 0.017, 13],
                   [0.070, -0.017, -13],
                   [0.050, -0.050, -42],
                   [0.019, -0.064, -75],
                  [-0.038, -0.048, -128],
                  [-0.048, 0.000, 180]])

# Serial connectin baud rate
BAUD_RATE = 9600


#***********************************************************
class Robot(object):  # Robot

    def __init__(self, ID, x=0.0, y=0.0, deg=90.0):
        # robot ID
        self.id = ID

        # wheel arrangement
        self.wheel_radius = R_WHEEL_RADIUS             # meters
        self.wheel_base_length = R_WHEEL_BASE_LENGTH   # meters

        # drive rates
        self.max_speed = R_MAX_WHEEL_DRIVE_RATE * ((2 * pi) / 60)  # rpm 2 rad/s
        self.trans_vel_limit = self.max_speed * self.wheel_radius  # m/s
        self.ang_vel_limit = 0.2 * self.max_speed                 # rad/s

        if debug:
            print("SPEED{:.3f}m/s").format(self.trans_vel_limit)

        # pose
        theta = -pi + ((deg / 360.0) * 2 * pi)
        self.pose = Pose(x, y, theta)

        # geometry
        self.geometry = Polygon(R_BOTTOM_PLATE)
        self.global_geometry = (
            self.geometry.get_transformation_to_pose(self.pose))

        # wheel encoders
        self.left_wheel_encoder = WheelEncoder(R_WHEEL_TICKS_PER_REV)
        self.right_wheel_encoder = WheelEncoder(R_WHEEL_TICKS_PER_REV)
        self.wheel_encoders = (
            [self.left_wheel_encoder, self.right_wheel_encoder])

        # IR sensors
        self.ir_sensors = []
        for _pose in R_SENSOR_POSES:
            ir_pose = Pose(_pose[0], _pose[1], radians(_pose[2]))
            self.ir_sensors.append(
                ProximitySensor(self, ir_pose, R_SENSOR_MIN_RANGE,
                    R_SENSOR_MAX_RANGE, radians(20)))

        # dynamics
        self.dynamics = DifferentialDriveDynamics(self.wheel_radius,
            self.wheel_base_length)

        # supervisor
        self.supervisor = Supervisor(RobotSupervisorInterface(self),
            R_WHEEL_RADIUS, R_WHEEL_BASE_LENGTH, R_WHEEL_TICKS_PER_REV,
            R_SENSOR_POSES, R_SENSOR_MAX_RANGE,
            Pose(self.pose.x, self.pose.y, self.pose.theta))

        # physical robot communication
        self.use_serial = use_serial

        if self.use_serial:
            self.physical_robot = RobotPhysicalInterface(self)

        ## initialize state
        # set wheel drive rates (rad/s)
        self.left_wheel_drive_rate = 0.0
        self.right_wheel_drive_rate = 0.0

    # simulate the robot's motion over the given time interval
    def step_motion(self, dt):
        v_l = self.left_wheel_drive_rate
        v_r = self.right_wheel_drive_rate

        # apply the robot dynamics to moving parts
        self.dynamics.apply_dynamics(v_l, v_r, dt,
                                      self.pose, self.wheel_encoders)

        # update global geometry (blue shell)
        self.global_geometry = (
            self.geometry.get_transformation_to_pose(self.pose))

        # update all of the sensors
        for ir_sensor in self.ir_sensors:
            ir_sensor.update_position()

        # send wheel speeds to physical robot and retrieve sensor values
        if self.use_serial:
            self.physical_robot.step()

    def stop_motion(self):
        # step the physical robot with it's final (True) zero speeds)
        if self.use_serial:
            self.physical_robot.step(True)

    # set the drive rates (angular velocities) for this robot's wheels in rad/s
    def set_wheel_drive_rates(self, v_l, v_r):
        # simulate physical limit on drive motors
        v_l = min(self.max_speed, v_l)
        v_r = min(self.max_speed, v_r)

        # set drive rates
        self.left_wheel_drive_rate = v_l
        self.right_wheel_drive_rate = v_r


#******************************************************
class RobotView(object):

    def __init__(self, viewer, robot):
        self.viewer = viewer
        self.robot = robot

        # add the supervisor views for this robot
        self.supervisor_view = (
            SupervisorView(viewer, robot.supervisor, robot.global_geometry))

        # add the IR sensor views for this robot
        self.ir_sensor_views = []
        for ir_sensor in robot.ir_sensors:
            self.ir_sensor_views.append(ProximitySensorView(viewer, ir_sensor))

        self.traverse_path = []  # this robot's traverse path

    def draw_robot_to_frame(self):
        # update the robot traverse path
        position = self.robot.pose.vposition()
        self.traverse_path.append(position)

        # draw the internal state (supervisor) to the frame
        self.supervisor_view.draw_supervisor_to_frame()

        # draw the IR sensors to the frame if indicated
        if self.viewer.draw_invisibles:
            for ir_sensor_view in self.ir_sensor_views:
                ir_sensor_view.draw_proximity_sensor_to_frame()

        # draw the robot
        robot_bottom = self.robot.global_geometry.vertexes
        (self.viewer.current_frame.add_polygons([robot_bottom],
                                                 color="blue",
                                                 alpha=0.5))
        # add decoration
        robot_pos, robot_theta = self.robot.pose.vunpack()
        robot_top = linalg.rotate_and_translate_vectors(R_TOP_PLATE,
            robot_theta, robot_pos)
        (self.viewer.current_frame.add_polygons([robot_top],
                                                color="black",
                                                alpha=0.5))

        # draw the robot's traverse path if indicated
        if self.viewer.draw_invisibles:
            self._draw_traverse_path_to_frame()

    def _draw_traverse_path_to_frame(self):
        (self.viewer.current_frame.add_lines([self.traverse_path],
                                              color="black",
                                              linewidth=0.01))

    # draws the traverse path as dots weighted according to robot speed
    def _draw_rich_traverse_path_to_frame(self):
        # when robot is moving fast, draw small, opaque dots
        # when robot is moving slow, draw large, transparent dots
        d_min, d_max = 0.0, 0.01574  # possible distances between dots
        r_min, r_max = 0.007, 0.02   # dot radius
        a_min, a_max = 0.3, 0.55     # dot alpha value
        m_r = (r_max - r_min) / (d_min - d_max)
        b_r = r_max - m_r * d_min
        m_a = (a_max - a_min) / (r_min - r_max)
        b_a = a_max - m_a * r_min

        prev_posn = self.traverse_path[0]
        frame = self.viewer.current_frame
        for posn in self.traverse_path[1::1]:
            d = linalg.distance(posn, prev_posn)
            r = (m_r * d) + b_r
            a = (m_a * r) + b_a
            (frame.add_circle(pos=posn,
                              radius=r,
                              color="black",
                              alpha=a))
            prev_posn = posn


#******************************************************************
class RobotPhysicalInterface(object):

    def __init__(self, robot):
        self.robot = robot

        # initialise velocity and direction of wheels
        self.v_l = 0
        self.v_r = 0
        self.dir_l = 'F'
        self.dir_r = 'F'

        # initialise the serial connection to physical robot
        if self.robot.use_serial:
            self.robot_comm = RobotComm(self.robot)  # robot.id,
                                           # "Serial{}".format(robot.id))
            timeout = time.time()

            # 1 second pause to allow serial connection to establish
            while not (self.robot_comm.ser_open or
                    (time.time() - (timeout > 1.0))):
                time.sleep(0.01)

            if self.robot_comm.ser_open is False:
                print("Connection to Zumo failed. "
                    "No robot movement will occur.")
            else:
                print("Now connected to Zumo")

    def step(self, final=False):
        # get the wheel drives rates assigned to the robot by the supervisor
        self.get_wheel_drive_rates()
        if final:
            # set the wheel rates to zero to stop the robot if final step
            self.v_l = 0
            self.v_r = 0

        #the direction values for left
        if (self.v_l < 0):
            self.dir_l = 'B'
            self.v_l = int(fabs(self.v_l))
        else:
            self.dir_l = 'F'

        # the direction values for right
        if (self.v_r < 0):
            self.dir_r = 'B'
            self.v_r = int(fabs(self.v_r))
        else:
            self.dir_r = 'F'

        # command for left wheel - [dir]L###
        to_send_l = "{}L{:03d}".format(self.dir_l, self.v_l)

        # command for right wheel - [dir]R###
        to_send_r = "{}R{:03d}".format(self.dir_r, self.v_r)

        # join these commands together
        to_send = to_send_l + to_send_r

        if self.robot.use_serial and self.robot_comm.connected:
            self.robot_comm.send_lock.acquire()
            self.robot_comm.send_queue.append(to_send)
            #self.robot_comm.send_queue.append(to_send_r)
            self.robot_comm.send_lock.release()

            if debug:
                print("--------------------------")
                print(self.robot_comm.send_queue)
                print("sending command {} and {} to Zumo").format(to_send_l,
                    to_send_r)

            # run serial comms to send send_queue to the physical robot
            self.robot_comm.run()

    # read the proximity sensors of the physical robot
    def read_proximity_sensors(self):
        return [s.read() for s in self.robot.ir_sensors]

    # read the wheel encoders of the physical robot
    def read_wheel_encoders(self):
        return [enc.read() for enc in self.robot.wheel_encoders]

    # apply wheel drive command to the physical robot (convert rad/s to rpm)
    def get_wheel_drive_rates(self):
        self.v_l = int(self.robot.left_wheel_drive_rate * (60 / (2 * pi)))
        self.v_r = int(self.robot.right_wheel_drive_rate * (60 / (2 * pi)))


#******************************************************************
class RobotComm(object):  # threading.Thread):

    def __init__(self, robot):  # , thread_id, name):
        # threading.Thread.__init__(self)

        # self.thread_id = thread_id
        # self.name = name

        # bind the robot this serial comm will be for
        self.robot = robot

        self.ser = None

        self.send_queue = []
        self.send_lock = threading.Lock()

        self.recieve_queue = []
        self.recieve_lock = threading.Lock()

        self.ser_open = False
        self.ser_num = 0

        self.serial_sends = []

        self.connected = False

        #self.start()
        self.connect()

    def connect(self):
        com_list = []
        comports = serial.tools.list_ports.comports()
        for comport in comports:
            for thing in comport:
                # creates list of available com ports
                com_list.append(thing)
        if debug:
            print(com_list)

        # removes duplicates from comList and returns a list
        com_list = list(set(com_list))
        if debug:
            print(com_list)
        print ("Attempting to connect to Zumo")
        for port in com_list:
            try:
                #,parity = serial.PARITY_NONE,stopbits = serial.STOPBITS_ONE,
                    #bytesize = serial.EIGHTBITS)
                ser = serial.Serial(port, baudrate=BAUD_RATE, timeout=None)
                ser.write('V\n')
                result = ser.readline()

                if debug:
                    print (port)
                    print (ser)
                    print ("Result:" + result)
                if "ZUMO" in result:
                    print ("Connect Successful! Connected on port:" + port)
                    self.ser = ser
                    self.ser.flush()
                    self.ser_open = True
                    s = str(port)
                    self.ser_num = int(s[-1:])

                    # start the logging clock
                    self.start_time = time.clock()

                    # connected flag
                    self.connected = True
                    self.robot.use_serial = True

                    break

            except serial.serialutil.SerialException:
                print("No connection available")
                self.connected = False
                self.robot.use_serial = False

    def run(self):
        #self.connect()
        #lastUpdateTime = time.clock()

        #while(self.connected):
            #if ((time.clock() - lastUpdateTime) >= 0.05): # run loop every 50ms
                #lastUpdateTime = time.clock()

        # send waiting messages
        send = False
        if(len(self.send_queue) > 0):
            self.send_lock.acquire()
            to_send = self.send_queue.pop(0)
            self.send_lock.release()
            send = True
        #else:
            # keeps infinite while loop from killing processor
        #    time.sleep(0.01)
        if send:
            send_time = time.clock() - self.start_time
            self.serial_sends.append([float(send_time), str(to_send)])
            time.sleep(0.003)
            if self.ser_open:
                if self.ser.writable:
                    if self.ser_open:
                        self.ser.write(str(to_send))
            if debug:
                print(("sent '{}' to COM{}").format(
                    str(to_send).strip('\r'), self.ser_num))


#******************************************************
# AN ABSTRACT SENSOR CLASS
class Sensor(object):

    def read(self):
        raise NotImplementedError()


#******************************************************
class ProximitySensor(Sensor):

    def __init__(self, robot,  # robot this sensor is attached to
            placement_pose,    # pose of this sensor relative to the robot
            min_range,         # min sensor range (meters)
            max_range,         # max sensor range (meters)
            phi_view):   # view angle of this sensor (rad from front of robot)

        # (NOTE: pose normalized on robot located at origin and with theta 0,
        # i.e. facing east)

        # bind the robot
        self.robot = robot

        # pose attributes
        #  - pose of this sensor relative to the robot
        self.placement_pose = placement_pose
        #  - global pose of this sensor
        self.pose = Pose(0.0, 0.0, 0.0)

        # detector line
        self.detector_line_source = LineSegment([[0.0, 0.0], [max_range, 0.0]])
        self.detector_line = LineSegment([[0.0, 0.0], [max_range, 0.0]])

        # pose and detector_line are incorrect until:
        # set initial position
        self.update_position()

        # sensitivity attributes
        self.min_range = min_range
        self.max_range = max_range
        self.phi_view = phi_view

        # physical distance detected to target as a proportion of max_range
        #( must be in range [0, 1] or None )
        self.target_delta = None

        # sensor output
        self.read_value = R_SENSOR_MIN_READ_VALUE

    # set this proximity sensor to detect an object at
    # distance ( delta * max_range )
    def detect(self, delta):
        if delta is not None and (delta < 0.0 or delta > 1.0):
            raise Exception("delta out of bounds - must be in range [0.0, 1.0]")

        if delta is None:
            self.target_delta = None
            self.read_value = R_SENSOR_MIN_READ_VALUE
        else:
            max_range = self.max_range
            min_range = self.min_range

            d = max_range * delta   # d is the real distance in meters
            if d <= min_range:        # d in [0.00, 0.02]
                self.target_delta = min_range / max_range
                self.read_value = R_SENSOR_MAX_READ_VALUE
            else:                     # d in (0.02, 0.20]
                self.target_delta = delta
                self.read_value = max(R_SENSOR_MIN_READ_VALUE,
                    int(ceil(R_SENSOR_MAX_READ_VALUE * e ** (-30 * (d - 0.02))))
                    )

    # get this sensor's output
    def read(self):
        return self.read_value

    # update the global position of this sensor
    def update_position(self):
        # update global pose
        self._update_pose()

        # update detector line
        self.detector_line = (
            self.detector_line_source.get_transformation_to_pose(self.pose))

    # update this sensor's pose
    def _update_pose(self):
        self.pose = self.placement_pose.transform_to(self.robot.pose)


#*******************************************************
class ProximitySensorView(object):

    def __init__(self, viewer, proximity_sensor):
        self.viewer = viewer
        self.proximity_sensor = proximity_sensor

    def draw_proximity_sensor_to_frame(self):
        proximity_sensor = self.proximity_sensor

        # grab proximity sensor pose values
        sensor_pos, sensor_theta = proximity_sensor.pose.vunpack()

        # build the sensor cone
        r = proximity_sensor.max_range
        phi = proximity_sensor.phi_view
        sensor_cone_poly = [[0.0, 0.0],
                            [r * cos(-phi / 2), r * sin(-phi / 2)],
                            [r, 0.0],
                            [r * cos(phi / 2), r * sin(phi / 2)]]
        sensor_cone_poly = linalg.rotate_and_translate_vectors(sensor_cone_poly,
                                                               sensor_theta,
                                                               sensor_pos)

        # shade the sensor cone according to positive detection
        if self.proximity_sensor.target_delta is not None:
            alpha = 0.9 - 0.8 * self.proximity_sensor.target_delta
        else:
            alpha = 0.1

        # add the sensor cone to the frame
        self.viewer.current_frame.add_polygons([sensor_cone_poly],
                                                color="red",
                                                alpha=alpha)

        # === FOR DEBUGGING: ===
        # self._draw_detector_line_to_frame()
        # self._draw_detector_line_origins_to_frame()
        # self._draw_bounding_circle_to_frame()
        # self._draw_detection_to_frame()

    def _draw_detection_to_frame(self):
        target_delta = self.proximity_sensor.target_delta
        if target_delta is not None:
            detector_endpoints = self.proximity_sensor.detector_line.vertexes
            detector_vector = linalg.sub(detector_endpoints[1],
                detector_endpoints[0])
            target_vector = linalg.add(detector_endpoints[0],
                linalg.scale(detector_vector, target_delta))

            self.viewer.current_frame.add_circle(pos=target_vector,
                                                 radius=0.02,
                                                 color="black",
                                                 alpha=0.7)

    def _draw_detector_line_to_frame(self):
        vertexes = self.proximity_sensor.detector_line.vertexes

        self.viewer.current_frame.add_lines([vertexes],
                                            linewidth=0.005,
                                            color="black",
                                            alpha=0.7)

    def _draw_detector_line_origins_to_frame(self):
        origin = self.proximity_sensor.detector_line.vertexes[0]
        self.viewer.current_frame.add_circle(pos=(origin[0], origin[1]),
                                             radius=0.02,
                                             color="black")

    def _draw_bounding_circle_to_frame(self):
        c, r = self.proximity_sensor.detector_line.bounding_circle
        self.viewer.current_frame.add_circle(pos=c,
                                             radius=r,
                                             color="black",
                                             alpha=0.2)
        self.viewer.current_frame.add_circle(pos=c,
                                             radius=0.005,
                                             color="black",
                                             alpha=0.3)


#*******************************************************
class WheelEncoder(Sensor):

    def __init__(self, ticks_per_rev):
        self.ticks_per_rev = ticks_per_rev
        self.real_revs = 0.0
        self.tick_count = 0

    # update the tick count for this wheel encoder
    # takes a float representing the number of forward revolutions made
    def step_revolutions(self, revolutions):
        self.real_revs += revolutions
        self.tick_count = int(self.real_revs * self.ticks_per_rev)

    def read(self):
        return self.tick_count


#***********************************************************
class DifferentialDriveDynamics(object):

    def __init__(self, wheel_radius, wheel_base_length):
        self.wheel_radius = wheel_radius
        self.wheel_base_length = wheel_base_length

    # apply physical dynamics to the given representations of moving parts
    def apply_dynamics(self, v_l, v_r, dt,         # dynamics parameters
                            pose, wheel_encoders):  # the moving parts

        # calculate the change in wheel angle (in radians)
        d_angle_left = dt * v_l
        d_angle_right = dt * v_r

        # calculate the distance traveled
        wheel_meters_per_rad = self.wheel_radius
        d_left_wheel = d_angle_left * wheel_meters_per_rad
        d_right_wheel = d_angle_right * wheel_meters_per_rad
        d_center = (d_left_wheel + d_right_wheel) / 2.0

        # calculate the new pose
        old_x, old_y, old_theta = pose.sunpack()
        new_x = old_x + (d_center * cos(old_theta))
        new_y = old_y + (d_center * sin(old_theta))
        new_theta = old_theta + (
            ((d_right_wheel - d_left_wheel) / self.wheel_base_length))

        # calculate the number of rotations each wheel has made
        revolutions_left = d_angle_left / (2 * pi)
        revolutions_right = d_angle_right / (2 * pi)

        # update the state of the moving parts
        pose.supdate(new_x, new_y, new_theta)
        # print("New y: {}").format(new_y)
        wheel_encoders[0].step_revolutions(revolutions_left)
        wheel_encoders[1].step_revolutions(revolutions_right)