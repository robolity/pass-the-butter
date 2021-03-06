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
import time
import timeit
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
use_serial = True

# Serial connectin baud rate
BAUD_RATE = 115200


#***********************************************************
class Robot(object):  # Robot
    """Class representing a robot simultaneously in the GUI and real world.

    Attributes:
        id -> int
        wheel_radius -> float
        wheel_base_length -> float
        max_speed -> float
        trans_vel_limit -> float
        ang_vel_limit -> float
        pose -> Pose object
        geometry -> Polygon object
        global_geometry -> Polygon object
        left_wheel_encoder -> WheelEncoder object
        right_wheel_encoder -> WheelEncoder object
        wheel_encoders -> list
        proximity_sensors -> list
        dynamics -> DifferentialDriveDynamics object
        supervisor -> Supervisor object
        use_serial -> boolean
        physical_robot -> RobotPhysicalInterface object
        left_wheel_drive_rate -> float
        right_wheel_drive_rate -> float

    Methods:
        __init__(ID, x=0.0, y=0.0, deg=90.0)
        step_motion(dt)
        stop_motion()
        set_wheel_drive_rates(v_l, v_r)
    """
    def __init__(self, ID, viewer, x=0.0, y=0.0, deg=0.0):
        """Bind robot ID and setup robot geometry, location, supervisor & comms.

        Keywords:
            id -> int
            x -> float
            y -> float
            deg -> float
            robot_params -> list
        """

        # robot ID
        self.id = ID

        # bind the viewer of the robot
        self.viewer = viewer

        # bind the initial pose of the robot
        self.x = x
        self.y = y
        self.deg = deg

        # setup the robot
        self.setup_robot(self.x, self.y, self.deg)

        # physical robot communication
        self.use_serial = use_serial

        if self.use_serial:
            self.physical_robot = RobotPhysicalInterface(self)

    def setup_robot(self, x=0.0, y=0.0, deg=0.0):
        """Apply the robot and sensor config parameters"""
        # retrieve robot config parameters
        robot_params = self.viewer.get_robot_parameters()

        self.wheel_radius = float(robot_params[0][0][1])        # metres
        self.wheel_base_length = float(robot_params[0][1][1])   # metres
        self.ticks_per_rev = float(robot_params[0][2][1])       # unitless
        self.max_speed = float(robot_params[0][3][1])           # rpm
        self.body_width = float(robot_params[0][4][1])          # metres
        self.body_length = float(robot_params[0][5][1])         # metres
        self.payload_width = float(robot_params[0][6][1])       # metres
        self.payload_length = float(robot_params[0][7][1])      # metres
        self.payload_offset = float(robot_params[0][8][1])      # metres

        self.sensor_min_value = float(robot_params[1][0][1])    # millivolts
        self.sensor_max_value = float(robot_params[1][1][1])    # millivolts
        self.sensor_min_range = float(robot_params[1][2][1])    # metres
        self.sensor_max_range = float(robot_params[1][3][1])    # metres
        self.sensor_phi_range = float(robot_params[1][4][1])    # metres

        # x, y, theta_degrees
        self.sensor_poses = ([
            [float(robot_params[2][0][1]), float(robot_params[2][1][1]),
                float(robot_params[2][2][1])],
            [float(robot_params[3][0][1]), float(robot_params[3][1][1]),
                float(robot_params[3][2][1])],
            [float(robot_params[4][0][1]), float(robot_params[4][1][1]),
                float(robot_params[4][2][1])]
            ])

        self.robot_body = ([[self.body_length / 2, self.body_width / 2],
            [-self.body_length / 2, self.body_width / 2],
            [-self.body_length / 2, -self.body_width / 2],
            [self.body_length / 2, -self.body_width / 2]])

        self.robot_payload = ([[self.payload_length / 2 + self.payload_offset,
                self.payload_width / 2],
            [-self.payload_length / 2 + self.payload_offset,
                self.payload_width / 2],
            [-self.payload_length / 2 + self.payload_offset,
                -self.payload_width / 2],
            [self.payload_length / 2 + self.payload_offset,
                -self.payload_width / 2]])

        # drive rates
        self.max_speed *= ((2 * pi) / 60)  # rpm 2 rad/s
        self.trans_vel_limit = self.max_speed * self.wheel_radius  # m/s
        self.ang_vel_limit = 0.2 * self.max_speed                 # rad/s

        if debug:
            print("SPEED {:.3f}m/s").format(self.trans_vel_limit)

        # pose
        theta = -pi + ((deg / 360.0) * 2 * pi)
        self.pose = Pose(x, y, theta)

        # geometry
        self.geometry = Polygon(self.robot_body)
        self.global_geometry = (
            self.geometry.get_transformation_to_pose(self.pose))

        # wheel encoders
        self.left_wheel_encoder = WheelEncoder(self.ticks_per_rev)
        self.right_wheel_encoder = WheelEncoder(self.ticks_per_rev)
        self.wheel_encoders = (
            [self.left_wheel_encoder, self.right_wheel_encoder])

        # proximity sensors
        self.proximity_sensors = []
        for _pose in self.sensor_poses:
            sensor_pose = Pose(_pose[0], _pose[1], radians(_pose[2]))
            self.proximity_sensors.append(
                ProximitySensor(self, sensor_pose, self.sensor_min_range,
                    self.sensor_max_range, radians(self.sensor_phi_range)))

        # dynamics
        self.dynamics = DifferentialDriveDynamics(self, self.wheel_radius,
            self.wheel_base_length)

        # supervisor
        self.supervisor = Supervisor(RobotSupervisorInterface(self),
            self.wheel_radius, self.wheel_base_length, self.ticks_per_rev,
            self.sensor_poses, self.sensor_max_range,
            Pose(self.pose.x, self.pose.y, self.pose.theta))

        # set wheel drive rates (rad/s)
        self.left_wheel_drive_rate = 0.0
        self.right_wheel_drive_rate = 0.0

    def step_motion(self, dt):
        """Simulate the robot's motion over the given time interval."""
        # apply the supervisor velocities & encoder values -> determine new pose
        self.dynamics.apply_dynamics(self.left_wheel_drive_rate,
            self.right_wheel_drive_rate, dt, self.pose, self.wheel_encoders)

        # update global geometry (blue shell)
        self.global_geometry = (
            self.geometry.get_transformation_to_pose(self.pose))

        # update the position all of proximity the sensors
        for proximity_sensor in self.proximity_sensors:
            proximity_sensor.update_position()

        # send wheel speeds to physical robot and retrieve sensor values
        if self.physical_robot.robot_comm.connected:
            self.physical_robot.step()

    def stop_motion(self):
        """Stop the physical robot."""
        if self.physical_robot.robot_comm.connected:
            self.physical_robot.stop()

    def set_wheel_drive_rates(self, v_l, v_r):
        """Set the drive rates (angular vel rad/s) for this robot's wheels."""
        # simulate physical limit on drive motors
        self.left_wheel_drive_rate = min(self.max_speed, v_l)
        self.right_wheel_drive_rate = min(self.max_speed, v_r)

        print("left_wheel_drive_rate = {}, right_wheel_drive_rate = {}").format(
            self.left_wheel_drive_rate, self.right_wheel_drive_rate)

    def read_encoders(self):

        self.physical_robot.read_wheel_encoders()


#******************************************************
class RobotView(object):
    """Class to draw the robot, supervisor and ir sensors to the frame.

    Attributes:
        viewer -> Viewer object
        robot -> Robot object
        supervisor_view -> SupervisorView object
        proximity_sensor_views -> list
        traverse_path -> list

    Methods:
        __init__(viewer, robot)
        draw_robot_to_frame()
        _draw_traverse_path_to_frame()
        _draw_rich_traverse_path_to_frame()
    """
    def __init__(self, viewer, robot):
        """Binds the view and robot, sets up the supervisor and ir sensor views

        Keywords:
            viewer -> Viewer object
            robot -> Robot object
        """
        self.viewer = viewer
        self.robot = robot

        # add the supervisor views for this robot
        self.supervisor_view = (
            SupervisorView(viewer, robot.supervisor, robot.global_geometry))

        # add the proximity sensor views for this robot
        self.proximity_sensor_views = []
        for proximity_sensor in robot.proximity_sensors:
            self.proximity_sensor_views.append(ProximitySensorView(
                viewer, proximity_sensor))

        self.traverse_path = []  # this robot's traverse path

    def draw_robot_to_frame(self):
        """Draws the robot and proximity sensors to the viewer frame."""
        # update the robot traverse path
        position = self.robot.pose.vposition()
        self.traverse_path.append(position)

        # draw the internal state (supervisor) to the frame
        self.supervisor_view.draw_supervisor_to_frame()

        # draw the proximity sensors to the frame if indicated
        if self.viewer.draw_invisibles:
            for proximity_sensor_view in self.proximity_sensor_views:
                proximity_sensor_view.draw_proximity_sensor_to_frame()

        # draw the robot
        robot_bottom = self.robot.global_geometry.vertexes
        (self.viewer.current_frame.add_polygons([robot_bottom],
                                                 color="blue",
                                                 alpha=0.5))
        # add decoration
        robot_pos, robot_theta = self.robot.pose.vunpack()
        robot_top = linalg.rotate_and_translate_vectors(
            self.robot.robot_payload, robot_theta, robot_pos)
        (self.viewer.current_frame.add_polygons([robot_top],
                                                color="black",
                                                alpha=0.5))

        # draw the robot's traverse path if indicated
        if self.viewer.draw_invisibles:
            self._draw_traverse_path_to_frame()

    def _draw_traverse_path_to_frame(self):
        """Draws a line representing the already travelled path of the robot."""
        (self.viewer.current_frame.add_lines([self.traverse_path],
                                              color="black",
                                              linewidth=0.01))

    def _draw_rich_traverse_path_to_frame(self):
        """Draws the traverse path as dots weighted according to robot speed."""
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
    """Handles communication to & from physical robot to mimic simulated robot.

    Attributes:
        robot -> Robot object
        v_l -> int
        v_r -> int
        dir_l -> char
        dir_r -> char

    Methods:
        __init__(robot)
        step()self.enc_l = 0
        self.enc_r = 0
        read_proximity_sensors()
        read_wheel_encoders()
        get_wheel_drive_rates()
    """
    def __init__(self, robot):
        """Binds the robot and sets up serial communication to the robot.

        Keywords:
            robot -> Robot object
        """
        self.robot = robot

        # initialise velocity and direction of wheels
        self.v_l = 0
        self.v_r = 0
        self.dir_l = 'F'
        self.dir_r = 'F'

        # intialise encoders
        self.enc_l = 0
        self.enc_r = 0

        self.enc_l_prev = 0
        self.enc_r_prev = 0

        # initialise the serial connection to physical robot
        self.robot_comm = RobotComm(self.robot, '/V', 'butter')  # robot.id

        #timeout = time.time()

        ## 1 second pause to allow serial connection to establish
        #while not (self.robot_comm.ser_open or
                #(time.time() - (timeout > 1.0))):
            #time.sleep(0.01)

        if self.robot_comm.ser_open is False:
            print("Connection to Zumo failed. "
                "No robot movement will occur.")
        else:
            print("Now connected to Zumo")

    def step(self):
        """Get wheel drives rates assigned to the robot by the supervisor."""

        self.get_wheel_drive_rates()

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

        #convert v_l to chars
        char1_l = (self.v_l & 0xFF00) >> 8
        char2_l = self.v_l & 0x00FF

        #convert v_l to chars
        char1_r = (self.v_r & 0xFF00) >> 8
        char2_r = self.v_r & 0x00FF

        # command for left wheel - [dir]L###
        to_send_l = self.dir_l + 'L' + chr(char1_l) + chr(char2_l)

        # command for right wheel - [dir]R###
        to_send_r = self.dir_r + 'R' + chr(char1_r) + chr(char2_r)

        # join these commands together
        to_send = to_send_l + to_send_r

        self.send_command(to_send)

    def stop(self):
        """Set the physical robot wheel speeds to zero to stop the robot."""
        # command for left wheel - [dir]L###
        to_send_l = self.dir_l + 'L' + chr(0) + chr(0)

        # command for right wheel - [dir]R###
        to_send_r = self.dir_r + 'R' + chr(0) + chr(0)

        # join these commands together
        to_send = to_send_l + to_send_r

        self.send_command(to_send)

    def read_proximity_sensors(self):
        """Read the proximity sensors of the physical robot."""
        pass

    def read_wheel_encoders(self):
        """Read the wheel encoders of the physical robot."""
        if self.robot_comm.connected:
            self.send_command('E')
            stream = self.read_command('e')

            if len(stream) is 4:
                # Left encoder value
                left_msb = ord(stream[0]) << 8
                self.enc_l = left_msb + ord(stream[1])

                # Right encoder value
                right_msb = ord(stream[2]) << 8
                self.enc_r = right_msb + ord(stream[3])

                print("Encoders: L {}, R{}").format(self.enc_l, self.enc_r)

                self.robot.left_wheel_encoder.tick_count = self.enc_l
                self.robot.right_wheel_encoder.tick_count = self.enc_r
            else:
                print("No encoder reading")
        else:
            pass # they are estimated in apply_dynamics

    def get_wheel_drive_rates(self):
        """Apply wheel drive command to the physical robot (rad/s to rpm)."""
        self.v_l = int(self.robot.left_wheel_drive_rate * (60 / (2 * pi)))
        self.v_r = int(self.robot.right_wheel_drive_rate * (60 / (2 * pi)))

    def send_command(self, to_send):
        if self.robot_comm.connected:
            self.robot_comm.send_queue.append(to_send)

            if debug:
                print("--------------------------")
                print(self.robot_comm.send_queue)
                print("sending command {} to Zumo").format(to_send)

            # run serial comms to send send_queue to the physical robot
            self.robot_comm.send()
        else:
            print("Serial connection to robot {} lost.").format(self.robot.id)

    def read_command(self, terminator):
        if self.robot_comm.connected:
            stream = []
            # append data stream until encoder terminator reached
            self.robot_comm.receive(terminator)
            while(len(self.robot_comm.receive_queue) > 0):
                received = self.robot_comm.receive_queue.pop(0)
                stream.append(received)  # add char to the output string

            if debug:
                print("Received {} chars of data, stream is: {}".format(
                    len(stream), list(map(hex, bytearray(stream)))))
            return stream
        else:
            print("Serial connection to robot {} lost.").format(self.robot.id)


#******************************************************************
class RobotComm(object):
    """Class to create a serial communication link to the physical robot.

    Attributes:
        robot -> Robot object
        ser -> Serial object
        query -> char
        response -> char
        send_queue -> list
        recieve_queue -> list
        ser_open -> boolean
        ser_num -> int
        serial_sends -> list
        connected -> boolean
        _tic -> float
        _toc -> float

    Methods:
        __init__(robot)
        connect()
        send()
        receive(num_bytes)
        _start_timing()
        _stop_timing()
    """
    def __init__(self, robot, query, response):
        """Binds the robot this serial comm will be for and sets up serial lists

        Keywords:
            robot -> Robot object
            query -> string to get response from correct COM port
            response -> string expected from the queried COM port
        """
        self.robot = robot

        self.ser = None

        self.query = query
        self.response = response

        # serial command queues
        self.send_queue = []
        self.receive_queue = []

        self.ser_open = False
        self.ser_num = 0

        # stores what time commands were sent to the controller
        self.serial_sends = []

        self.connected = False

        self.connect()

    def connect(self):
        """Searches through available serial ports for the robot."""
        com_list = []

        comports = serial.tools.list_ports.comports()
        for comport in comports:
            for thing in comport:
                # creates list of available com ports
                com_list.append(thing)

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

                time.sleep(0.01)  # Give serial connection time to start up

                # send robot present check character
                ser.write(self.query)

                # Give serial connection time to respond
                time.sleep(0.03)
                print("Port: {} {}").format(port, ser.inWaiting())

                # read waiting messages to look for query answer
                result = ''
                while ser.inWaiting() > 0:
                    result += ser.read(1)

                time.sleep(0.5)  # Give serial query time to reset

                if debug:
                    if result != '':
                        print("{} on port {} >> {}".format(ser, port, result))

                if self.response in result:
                    print("Connect Successful! Connected on port: {}".format(
                        port))
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

    def send(self):
        """Sends serial commands to the controller.

        Check if there is anything in the send queue ready to send.
        If there are items in the queue, send them.
        """
        while(len(self.send_queue) > 0):
            to_send = self.send_queue.pop(0)
            if self.ser_open:
                if self.ser.writable:
                    if self.ser_open:
                        self.ser.write(to_send)
            if debug:
                # print using this method to prevent timeout
                print(("sent '{}' to COM{}").format(
                    list(map(hex, bytearray(to_send))),
                    self.ser_num))

    def receive(self, terminator):
        """Receives serial command from the controller."""
        # wait for bytes to be sent from EPOS2 controller
        time.sleep(0.001)

        result = ''
        print("Stream waiting: {}".format(self.ser.inWaiting()))
        while self.ser.inWaiting() > 0:
            result = self.ser.read(1)
            if result is 'e':
                break
            self.receive_queue.append(result)

    def _start_timing(self):
        """Start clock for timing code execution."""
        self._tic = timeit.default_timer()

    def _stop_timing(self):
        """Stop clock for timing code execution and print elapsed time."""
        self._toc = timeit.default_timer()
        print("Code execution time: {:.6f}s").format(self._toc - self._tic)


#******************************************************
class Sensor(object):
    """Abstract sensor class.
    Attributes:
        None

    Methods:
        read()
    """
    def read(self):
        """Placeholder method for parent Sensor class."""
        raise NotImplementedError()


#******************************************************
class ProximitySensor(Sensor):
    """Class representing a proximity sensor mounted to the robot.

    Attributes:
        robot -> Robot object
        placement_pose -> Pose object
        pose -> Pose object
        detector_line_source -> LineSegment object
        detector_line -> LineSegment object
        min_range -> float
        max_range -> float
        phi_view -> float
        target_delta -> float
        read_value -> float

    Methods:
        __init__(robot, placement_pose, min_range, max_range, phi_view)
        detect(delta)
        read()
        update_position()
        _update_pose()
    """
    def __init__(self, robot,  # robot this sensor is attached to
            placement_pose,    # pose of this sensor relative to the robot
            min_range,         # min sensor range (meters)
            max_range,         # max sensor range (meters)
            phi_view):   # view angle of this sensor (rad from front of robot)
        """Binds the robot and sets up the sensor's pose and attributes.

        Keywords:
            robot -> Robot object
            min_range -> float
            max_range -> float
            phi_view -> float

        (NOTE: pose normalized on robot located at origin and with theta 0,
        i.e. facing east)
        """
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
        self.read_value = self.robot.sensor_min_value

    def detect(self, delta):
        """Sets proximity sensor to detect object at dist (delta*max_range)."""
        if delta is not None and (delta < 0.0 or delta > 1.0):
            raise Exception("delta out of bounds - must be in range [0.0, 1.0]")

        if delta is None:
            self.target_delta = None
            self.read_value = self.robot.sensor_min_value
        else:
            max_range = self.max_range
            min_range = self.min_range

            d = max_range * delta   # d is the real distance in meters
            if d <= min_range:        # d in [0.00, 0.02]
                self.target_delta = min_range / max_range
                self.read_value = self.robot.sensor_max_value
            else:                     # d in (0.02, 0.20]
                self.target_delta = delta
                self.read_value = max(self.robot.sensor_min_value,
                    int(ceil(self.robot.sensor_max_value * e ** (-30 * (
                        d - 0.02)))))

    def read(self):
        """Get this sensor's output."""
        return self.read_value

    def update_position(self):
        """Update the global position of this sensor."""
        # update global pose
        self._update_pose()

        # update detector line
        self.detector_line = (
            self.detector_line_source.get_transformation_to_pose(self.pose))

    def _update_pose(self):
        """Update this sensor's pose."""
        self.pose = self.placement_pose.transform_to(self.robot.pose)


#*******************************************************
class ProximitySensorView(object):
    """Class to draw the proximity sensor to the frame.

    Attributes:
        viewer -> Viewer object
        proximity_sensor -> ProximitySensor object

    Methods:
        __init__(viewer, proximity_sensor)
        draw_proximity_sensor_to_frame()
        _draw_detection_to_frame()
        _draw_detector_line_to_frame()
        _draw_detector_line_origins_to_frame()
        _draw_bounding_circle_to_frame()
    """
    def __init__(self, viewer, proximity_sensor):
        """Binds the viewer and proximity_sensor.

        Keywords:
            viewer -> Viewer object
            proximity_sensor -> ProximitySensor object
        """
        self.viewer = viewer
        self.proximity_sensor = proximity_sensor

    def draw_proximity_sensor_to_frame(self):
        """Draws the proximity sensor to the frame."""
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
        """Draws a circle on the view of the sensor where obstacle detected."""
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
        """Draws a line on the view of the sensor showing its range."""
        vertexes = self.proximity_sensor.detector_line.vertexes

        self.viewer.current_frame.add_lines([vertexes],
                                            linewidth=0.005,
                                            color="black",
                                            alpha=0.7)

    def _draw_detector_line_origins_to_frame(self):
        """Draws a circle at the origin of the proximity sensor on the robot."""
        origin = self.proximity_sensor.detector_line.vertexes[0]
        self.viewer.current_frame.add_circle(pos=(origin[0], origin[1]),
                                             radius=0.02,
                                             color="black")

    def _draw_bounding_circle_to_frame(self):
        """Draws a bounding circle around the proximity sensor's range."""
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
    """Class representing a wheel encoder on the robot.

    Attributes:
        ticks_per_rev -> float
        real_revs -> float
        tick_count -> int

    Methods:
        __init__(ticks_per_rev)
        step_revolutions(revolutions)
        read()
    """
    def __init__(self, ticks_per_rev):
        """Initialises the wheel encoder parameters and initial count of 0.

        Keywords:
            robot -> Robot
            ticks_per_rev -> float
        """
        self.ticks_per_rev = ticks_per_rev
        self.real_revs = 0.0
        self.tick_count = 0

    def update_simulated_tick_count(self, revolutions):
        """Update the tick count for this wheel encoder.

        Takes a float representing the number of forward revolutions made.
        """
        self.real_revs += revolutions
        self.tick_count = int(self.real_revs * self.ticks_per_rev)
        print("Tick count: {}").format(self.tick_count)

    def read(self):
        """Returns the current encoder tick count."""
        return self.tick_count


#***********************************************************
class DifferentialDriveDynamics(object):
    """Class for calculating the change in robot position for calculated vels.

    Attributes:
        wheel_radius -> float
        wheel_base_length -> float

    Methods:
        __init(wheel_radius, wheel_base_length)
        apply_dynamics(v_l, v_r, dt, pose, wheel_encoders)
    """
    def __init__(self, robot, wheel_radius, wheel_base_length):
        """Applies the wheel radius and wheel base length of the robot.

        Keywords:
            wheel_radius -> float
            wheel_base_length -> float
        """
        self.robot = robot
        self.wheel_radius = wheel_radius
        self.wheel_base_length = wheel_base_length

    def apply_dynamics(self, v_l, v_r, dt,         # dynamics parameters
                            pose, wheel_encoders):  # the moving parts
        """Apply physical dynamics to the robot's moving parts."""
        # calculate the change in wheel angle (in radians)
        #print("current vel_l: {}").format(v_l)
        d_angle_left = dt * v_l
        #print("change in left wheel angle (radians): {}").format(d_angle_left)
        d_angle_right = dt * v_r

        # calculate the distance traveled
        wheel_meters_per_rad = self.wheel_radius
        d_left_wheel = d_angle_left * wheel_meters_per_rad
        #print("change in left wheel travelled (m): {}").format(d_left_wheel)
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
        #print("New y: {}").format(new_y)

        if not self.robot.physical_robot.robot_comm.connected:
            wheel_encoders[0].update_simulated_tick_count(revolutions_left)
            wheel_encoders[1].update_simulated_tick_count(revolutions_right)