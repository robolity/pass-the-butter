# -*- coding: utf-8 -*-

from math import pi, atan2, sin, cos

from physics import LinearAlgebra

#**********CLASSES IN MODULE**********************

#GoToGoalController
#GoToGoalControllerView
#AvoidObstaclesController
#AvoidObstaclesControllerView
#FollowWallController
#FollowWallControllerView
#GTGAndAOController
#GTGAndAOControllerView
#GoToAngleController


#*************************************************

debug = False

# bring in linear algebra functions
linalg = LinearAlgebra()

# length of heading vector
VECTOR_LEN = 0.75 

# wall-following directions
FWDIR_LEFT = 0
FWDIR_RIGHT = 1

#************************************************************


class Controller(object):
	"""Parent controller class to bind the generic controller inputs & outputs
	
	Attributes:
		supervisor -> SupervisorControllerInterface
        kP -> float
        kI -> float
        kD -> float
        prev_time -> float
        prev_eP -> float
        prev_eI -> float
    
    Methods:
        __init__(supervisor_interface)
        update_heading()
        execute()
        _print_vars(eP, eI, eD, v, omega)
    """
	def __init__(self, supervisor_interface, kP, kI, kD):
        """Bind the supervisor interface and initialise the controller variables
        
        Keywords:
            supervisor_interface -> SupervisorControllerInterface
        """
        # bind the supervisor
        self.supervisor = supervisor

        # bind the gains
        self.kP = kP
        self.kI = kI
        self.kD = kD

        # stored values - for computing next results
        self.prev_time = 0.0
        self.prev_eP = 0.0
        self.prev_eI = 0.0

    def update_heading(self):
        """Generate and store new heading vector"""
        pass

    def execute(self):
        """Calculate PID terms and generate the supervisor's v and omega"""
        pass

    def _print_vars(self, eP, eI, eD, v, omega):
        """Prints out the PID errors, control components and outputs"""
        print "\n\n"
        print "=============="
        print "ERRORS:"
        print "eP: " + str(eP)
        print "eI: " + str(eI)
        print "eD: " + str(eD)
        print ""
        print "CONTROL COMPONENTS:"
        print "kP * eP = " + str(self.kP) + " * " + str(eP)
        print "= " + str(self.kP * eP)
        print "kI * eI = " + str(self.kI) + " * " + str(eI)
        print "= " + str(self.kI * eI)
        print "kD * eD = " + str(self.kD) + " * " + str(eD)
        print "= " + str(self.kD * eD)
        print ""
        print "OUTPUTS:"
        print "omega: " + str(omega)
        print "v    : " + str(v)


#************************************************************		
class GoToGoalController(Controller):
    """PID controller to direct robot towards the goal location
    
    Attributes:
        kP -> float
        kI -> float
        kD -> float
        prev_time -> float
        prev_eP -> float
        prev_eI -> float
        gtg_heading_vector -> 2-dim array
    
    Methods:
        __init__(supervisor)
        update_heading()
        execute()
        calculate_gtg_heading_vector()
        _print_vars(eP, eI, eD, v, omega)
    """
    def __init__(self, supervisor_interface, kP, kI, kD):
        """Bind the supervisor interface and initialise the controller variables
        
        Keywords:
            supervisor_interface -> SupervisorControllerInterface
			kP -> float
			kI -> float
			kD -> float
        """
		super(GoToGoalController, self).__init__(supervisor_interface, 
			kP, kI, kD)
		
        # key vectors and data (initialize to any non-zero vector)
        self.gtg_heading_vector = [1.0, 0.0]

    def update_heading(self):
        """Generate and store new heading vector"""
        self.gtg_heading_vector = self.calculate_gtg_heading_vector()

    def execute(self):
        """Calculate PID terms and generate the supervisor's v and omega"""
        # calculate the time that has passed since the last control iteration
        current_time = self.supervisor.time()
        dt = current_time - self.prev_time

        # calculate the error terms
        theta_d = atan2(self.gtg_heading_vector[1], self.gtg_heading_vector[0])
        eP = theta_d
        eI = self.prev_eI + eP * dt
        eD = (eP - self.prev_eP) / dt

        # calculate angular velocity
        omega = self.kP * eP + self.kI * eI + self.kD * eD

        # calculate translational velocity
        # velocity is v_max when omega is 0,
        # drops rapidly to zero as |omega| rises
        v = self.supervisor.v_max() / (abs(omega) + 1) ** 0.5

        # store values for next control iteration
        self.prev_time = current_time
        self.prev_eP = eP
        self.prev_eI = eI

        self.supervisor.set_outputs(v, omega)

        # === FOR DEBUGGING ===
        if debug:
			self._print_vars(eP, eI, eD, v, omega)

    # return a go-to-goal heading vector in the robot's reference frame
    def calculate_gtg_heading_vector(self):
        """Calculate and return go to goal heading vector
        
		Return a go-to-goal heading vector in the robot's reference frame
		
		Returns:
            goal -> 2-dim array
        """
        # get the inverse of the robot's pose
        robot_inv_pos, robot_inv_theta = (
            self.supervisor.estimated_pose().inverse().vunpack())
        #print(robot_inv_pos)
        # calculate the goal vector in the robot's reference frame
        goal = self.supervisor.goal()
        goal = linalg.rotate_and_translate_vector(goal, robot_inv_theta,
                robot_inv_pos)

        return goal

		
#**********************************************************
class GoToGoalControllerView(object):
    """PID controller to direct robot towards the goal location
    
    Attributes:
        viewer -> Viewer
        supervisor -> SupervisorControllerInterface
        go_to_goal_controller -> GoToGoalContoller
    
    Methods:
        draw_go_to_goal_controller_to_frame()
    """
    def __init__(self, viewer, supervisor):
        """Bind the viewe, supervisor and controller
		
		Keywords:
			viewer -> Viewer
			supervisor -> SupervisorControllerInterface
		"""
        self.viewer = viewer
        self.supervisor = supervisor

        self.go_to_goal_controller = supervisor.go_to_goal_controller

    def draw_go_to_goal_controller_to_frame(self):
        """Draw go-to-goal controller's internal state to the frame"""
        robot_pos, robot_theta = self.supervisor.estimated_pose.vunpack()

        # draw the computed go-to-goal vector
        gtg_heading_vector = linalg.scale(linalg.unit(
            self.go_to_goal_controller.gtg_heading_vector), VECTOR_LEN)
        vector_line = [[0.0, 0.0], gtg_heading_vector]
        vector_line = linalg.rotate_and_translate_vectors(vector_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([vector_line],
                                            linewidth=0.02,
                                            color="dark green",
                                            alpha=1.0)


#********************************************************
class AvoidObstaclesController(Controller):
    """PID controller to direct robot away from obstacles
    
    Attributes:
        supervisor_interface -> SupervisorControllerInterface
        proximity_sensor_placements -> list
        sensor_gains -> list
        kP -> float
        kI -> float
        kD -> float
        prev_time -> float
        prev_eP -> float
        prev_eI -> float
        obstacle_vectors -> list of 2-dim arrays
        ao_heading_vector -> 2-dim array
    
    Methods:
        __init__(supervisor)
        update_heading()
        execute()
        calculate_ao_heading_vector()
        _print_vars(eP, eI, eD, v, omega)
    """
    def __init__(self, supervisor_interface, kP, kI, kD):
        """Bind the supervisor interface and initialise the controller variables
        
        Keywords:
            supervisor_interface -> SupervisorControllerInterface
			kP -> float
			kI -> float
			kD -> float
        """
		super(AvoidObstaclesController, self).__init__(supervisor_interface, 
			kP, kI, kD)

        # sensor placements
        self.proximity_sensor_placements = (
            self.supervisor_interface.proximity_sensor_placements())

        # sensor gains (weights)
        self.sensor_gains = [1.0 + ((0.4 * abs(p.theta)) / pi)
                             for p in self.proximity_sensor_placements]

        # key vectors and data (initialize to any non-zero vector)
        self.obstacle_vectors = [[1.0, 0.0]] * (
            len(self.proximity_sensor_placements))
        self.ao_heading_vector = [1.0, 0.0]

    def update_heading(self):
        """Generate and store new heading and obstacle vectors"""
        self.ao_heading_vector, self.obstacle_vectors = (
            self.calculate_ao_heading_vector())

    def execute(self):
		"""Calculate PID terms and generate the supervisor's v and omega"""
		
        # calculate the time that has passed since the last control iteration
        current_time = self.supervisor.time()
        dt = current_time - self.prev_time

        # calculate the error terms
        theta_d = atan2(self.ao_heading_vector[1], self.ao_heading_vector[0])
        eP = theta_d
        eI = self.prev_eI + eP * dt
        eD = (eP - self.prev_eP) / dt

        # calculate angular velocity
        omega = self.kP * eP + self.kI * eI + self.kD * eD

        # calculate translational velocity
        # velocity is v_max when omega is 0,
        # drops rapidly to zero as |omega| rises
        v = self.supervisor_interface.v_max() / (abs(omega) + 1) ** 2

        # store values for next control iteration
        self.prev_time = current_time
        self.prev_eP = eP
        self.prev_eI = eI

        self.supervisor_interface.set_outputs(v, omega)
		
		# === FOR DEBUGGING ===
        if debug:
			self._print_vars(eP, eI, eD, v, omega)

    # return a obstacle avoidance vector in the robot's reference frame
    # also returns vectors to detected obstacles in the robot's reference frame
    def calculate_ao_heading_vector(self):
		"""Calculate and return avoid obstacle heading vector
		
		Returns an obstacle avoidance vector in the robot's reference frame and 
		returns vectors to detected obstacles in the robot's reference frame
		
		Returns: 
			ao_heading_vector -> 2-dim array
			obstacle_vectors -> list of 2-dim arrays
		"""
        # initialize vector
        obstacle_vectors = [[0.0, 0.0]] * len(self.proximity_sensor_placements)
        ao_heading_vector = [0.0, 0.0]

        # get the distances indicated by the robot's sensor readings
        sensor_distances = (
            self.supervisor_interface.proximity_sensor_distances())

        # calculate the position of detected obstacles and find an
        #    avoidance vector
        robot_pos, robot_theta = (
            self.supervisor_interface.estimated_pose().vunpack())
        for i in range(len(sensor_distances)):
            # calculate the position of the obstacle
            sensor_pos, sensor_theta = (
                self.proximity_sensor_placements[i].vunpack())
            vector = [sensor_distances[i], 0.0]
            vector = linalg.rotate_and_translate_vector(vector, sensor_theta,
                sensor_pos)
            obstacle_vectors[i] = vector   # store the obstacle vectors in
                                           #    the robot's reference frame

            # accumluate the heading vector within the robot's reference frame
            ao_heading_vector = linalg.add(ao_heading_vector,
                linalg.scale(vector, self.sensor_gains[i]))

        return ao_heading_vector, obstacle_vectors


#*********************************************************
class AvoidObstaclesControllerView(object):
	"""PID controller to direct robot towards the goal location
    
    Attributes:
        viewer -> Viewer
        supervisor -> SupervisorControllerInterface
        avoid_obstacles_controller -> AvoidObstaclesController
    
    Methods:
        __init__(viewer, supervisor)
		draw_avoid_obstacles_controller_to_frame()
		draw_go_to_goal_controller_to_frame()
    """
    def __init__(self, viewer, supervisor):
        """Bind the viewe, supervisor and controller
		
		Keywords:
			viewer -> Viewer
			supervisor -> SupervisorControllerInterface
		"""
		self.viewer = viewer
        self.supervisor = supervisor
        self.avoid_obstacles_controller = supervisor.avoid_obstacles_controller

    # draw a representation of the avoid-obstacles controller's internal
    #     state to the frame
    def draw_avoid_obstacles_controller_to_frame(self):
		"""Draw avoid obstacle controller's internal state to the frame"""
        robot_pos, robot_theta = self.supervisor.estimated_pose.vunpack()

        # draw the detected environment boundary (i.e. sensor readings)
        obstacle_vertexes = self.avoid_obstacles_controller.obstacle_vectors[:]

        # close the drawn polygon
        obstacle_vertexes.append(obstacle_vertexes[0])

        obstacle_vertexes = linalg.rotate_and_translate_vectors(
            obstacle_vertexes, robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([obstacle_vertexes],
                                            linewidth=0.005,
                                            color="black",
                                            alpha=1.0)

        # draw the computed avoid-obstacles vector
        ao_heading_vector = linalg.scale(linalg.unit(
            self.avoid_obstacles_controller.ao_heading_vector), VECTOR_LEN)
        vector_line = [[0.0, 0.0], ao_heading_vector]
        vector_line = linalg.rotate_and_translate_vectors(vector_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([vector_line],
                                             linewidth=0.02,
                                             color="red",
                                             alpha=1.0)


#****************************************************
class FollowWallController(Controller):

    def __init__(self, supervisor_interface, kP, kI, kD):
        """Bind the supervisor interface and initialise the controller variables
        
        Keywords:
            supervisor_interface -> SupervisorControllerInterface
			kP -> float
			kI -> float
			kD -> float
        """
		super(FollowWallController, self).__init__(supervisor_interface, 
			kP, kI, kD)

        # sensor placements
        self.proximity_sensor_placements = (
            supervisor.proximity_sensor_placements())

        # wall-follow parameters
        self.follow_distance = 0.15  # meters from the center of the robot
                                     #    to the wall

        # key vectors and data (initialize to any non-zero vector)
        self.l_wall_surface = [[1.0, 0.0], [1.0, 0.0]]  # the followed surface,
                                                        #    in robot space
        self.l_parallel_component = [1.0, 0.0]
        self.l_perpendicular_component = [1.0, 0.0]
        self.l_distance_vector = [1.0, 0.0]
        self.l_fw_heading_vector = [1.0, 0.0]

        self.r_wall_surface = [[1.0, 0.0], [1.0, 0.0]]  # the followed surface,
                                                        #    in robot space
        self.r_parallel_component = [1.0, 0.0]
        self.r_perpendicular_component = [1.0, 0.0]
        self.r_distance_vector = [1.0, 0.0]
        self.r_fw_heading_vector = [1.0, 0.0]

    def update_heading(self):
        """Calculate and return heading vectors for following left & right"""
		
		# generate and store new heading vector and critical points for
        # 	 following to the left
		
        [
          self.l_fw_heading_vector,
          self.l_parallel_component,
          self.l_perpendicular_component,
          self.l_distance_vector,
          self.l_wall_surface
                            ] = self.calculate_fw_heading_vector(FWDIR_LEFT)
        
		# generate and store new heading vector and critical points for
        #    following to the right
        [
          self.r_fw_heading_vector,
          self.r_parallel_component,
          self.r_perpendicular_component,
          self.r_distance_vector,
          self.r_wall_surface
                            ] = self.calculate_fw_heading_vector(FWDIR_RIGHT)

    def execute(self):
		"""Calculate PID terms and generate the supervisor's v and omega"""
		
        # determine which direction to slide in
        current_state = self.supervisor.current_state()
        if current_state == 4:
            heading_vector = self.l_fw_heading_vector
        elif current_state == 5:
            heading_vector = self.r_fw_heading_vector
        else:
            raise Exception("applying follow-wall controller when not in a "
                "sliding state currently not supported")

        # calculate the time that has passed since the last control iteration
        current_time = self.supervisor.time()
        dt = current_time - self.prev_time

        # calculate the error terms
        theta_d = atan2(heading_vector[1], heading_vector[0])
        eP = theta_d
        eI = self.prev_eI + eP * dt
        eD = (eP - self.prev_eP) / dt

        # calculate angular velocity
        omega = self.kP * eP + self.kI * eI + self.kD * eD

        # calculate translational velocity
        # velocity is v_max when omega is 0,
        # drops rapidly to zero as |omega| rises
        v = self.supervisor.v_max() / (abs(omega) + 1) ** 0.7

        # store values for next control iteration
        self.prev_time = current_time
        self.prev_eP = eP
        self.prev_eI = eI

        self.supervisor.set_outputs(v, omega)

        # === FOR DEBUGGING ===
        if debug:
			self._print_vars(eP, eI, eD, v, omega)

    
    def calculate_fw_heading_vector(self, follow_direction):
		"""Calculate and return follow wall heading vector
		
		Return a wall-following vector in the robot's reference frame.
		Also returns the component vectors used to calculate the heading
		and the vectors representing the followed surface in robot-space
		
		Returns:
			l_fw_heading_vector
			l_parallel_component
			l_perpendicular_component
			l_distance_vector
			l_wall_surface
		"""
		
        # get the necessary variables for the working set of sensors
        #   the working set is the sensors on the side we are bearing on,
        #   indexed from rearmost to foremost on the robot
        #   NOTE: uses preexisting knowledge of the how the sensors are stored
        #         and indexed
        if follow_direction == FWDIR_LEFT:
            # if we are following to the left, we bear on the righthand sensors
            sensor_placements = self.proximity_sensor_placements[7:3:-1]
            sensor_distances = (
                self.supervisor.proximity_sensor_distances()[7:3:-1])
            sensor_detections = (
                self.supervisor.proximity_sensor_positive_detections()[7:3:-1])
        elif follow_direction == FWDIR_RIGHT:
            # if we are following to the right, we bear on the lefthand sensors
            sensor_placements = self.proximity_sensor_placements[:4]
            sensor_distances = self.supervisor.proximity_sensor_distances()[:4]
            sensor_detections = (
                self.supervisor.proximity_sensor_positive_detections()[:4])
        else:
            raise Exception("unknown wall-following direction")

        if True not in sensor_detections:
            # if there is no wall to track detected, we default to predefined
            #    reference points
            # NOTE: these points are designed to turn the robot towards the
            #       bearing side, which aids with cornering behavior the
            #       resulting heading vector is also meant to point close to
            #       directly aft of the robot this helps when determining
            #       switching conditions in the supervisor state machine
            p1 = [-0.2, 0.0]
            if follow_direction == FWDIR_LEFT:
                p2 = [-0.2, -0.0001]
            if follow_direction == FWDIR_RIGHT:
                p2 = [-0.2, 0.0001]
        else:
            # sort the sensor distances along with their corresponding indices
            # - this method ensures two different sensors are always used
            sensor_distances, indices = zip(*sorted(zip(
                                          sensor_distances,  # sensor distances
                                          [0, 1, 2, 3]  # corresponding indices
                                        )))
            # get the smallest sensor distances and their corresponding indices
            d1, d2 = sensor_distances[0:2]
            i1, i2 = indices[0:2]

            # calculate the vectors to the obstacle in the robot's
            #    reference frame
            sensor1_pos, sensor1_theta = sensor_placements[i1].vunpack()
            sensor2_pos, sensor2_theta = sensor_placements[i2].vunpack()
            p1, p2 = [d1, 0.0], [d2, 0.0]
            p1 = linalg.rotate_and_translate_vector(p1, sensor1_theta,
                sensor1_pos)
            p2 = linalg.rotate_and_translate_vector(p2, sensor2_theta,
                sensor2_pos)

            # ensure p2 is forward of p1
            if i2 < i1:
                p1, p2 = p2, p1

        # compute the key vectors and auxiliary data
        l_wall_surface = [p2, p1]
        l_parallel_component = linalg.sub(p2, p1)
        l_distance_vector = linalg.sub(p1, linalg.proj(p1,
            l_parallel_component))
        unit_perp = linalg.unit(l_distance_vector)
        distance_desired = linalg.scale(unit_perp, self.follow_distance)
        l_perpendicular_component = linalg.sub(l_distance_vector,
            distance_desired)
        l_fw_heading_vector = linalg.add(l_parallel_component,
            l_perpendicular_component)

        return (l_fw_heading_vector, l_parallel_component,
            l_perpendicular_component, l_distance_vector, l_wall_surface)


#**********************************************************
class FollowWallControllerView(object):

    def __init__(self, viewer, supervisor):
        """Bind the viewe, supervisor and controller
		
		Keywords:
			viewer -> Viewer
			supervisor -> SupervisorControllerInterface
		"""
        self.viewer = viewer
        self.supervisor = supervisor
        self.follow_wall_controller = supervisor.follow_wall_controller

    # draw a representation of the currently-active side of the follow-wall
    #    controller state to the frame
    def draw_active_follow_wall_controller_to_frame(self):
		"""Draw follow wall controller's internal state to the frame
		
		Draw a representation of the currently-active side of the follow-wall
        controller state to the frame
		"""
        # determine which side to renderi
        current_state = self.supervisor.state_machine.current_state
        if current_state == 4:
            self._draw_follow_wall_controller_to_frame_by_side(FWDIR_LEFT)
        elif current_state == 5:
            self._draw_follow_wall_controller_to_frame_by_side(FWDIR_RIGHT)
        else:
            raise Exception("applying follow-wall controller when not in a "
                "sliding state currently not supported")

    def draw_complete_follow_wall_controller_to_frame(self):
		"""Draw a representation of both sides of the follow-wall controller"""
        self._draw_follow_wall_controller_to_frame_by_side(FWDIR_LEFT)
        self._draw_follow_wall_controller_to_frame_by_side(FWDIR_RIGHT)

    def _draw_follow_wall_controller_to_frame_by_side(self, side):
		"""Draw the controller to the frame for the indicated side only"""
        if side == FWDIR_LEFT:
            surface_line = self.follow_wall_controller.l_wall_surface
            distance_vector = self.follow_wall_controller.l_distance_vector
            #perpendicular_component = (
            #    self.follow_wall_controller.l_perpendicular_component)
            #parallel_component = (
            #    self.follow_wall_controller.l_parallel_component)
            fw_heading_vector = self.follow_wall_controller.l_fw_heading_vector
        elif side == FWDIR_RIGHT:
            surface_line = self.follow_wall_controller.r_wall_surface
            distance_vector = self.follow_wall_controller.r_distance_vector
            #perpendicular_component = (
            #    self.follow_wall_controller.r_perpendicular_component)
            #parallel_component = (
            #    self.follow_wall_controller.r_parallel_component)
            fw_heading_vector = self.follow_wall_controller.r_fw_heading_vector
        else:
            raise Exception("unrecognized argument: follow-wall direction "
                "indicator")

        robot_pos, robot_theta = self.supervisor.estimated_pose.vunpack()

        # draw the estimated wall surface
        surface_line = linalg.rotate_and_translate_vectors(surface_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([surface_line],
                                            linewidth=0.01,
                                            color="black",
                                            alpha=1.0)

        # draw the measuring line from the robot to the wall
        range_line = [[0.0, 0.0], distance_vector]
        range_line = linalg.rotate_and_translate_vectors(range_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([range_line],
                                            linewidth=0.005,
                                            color="black",
                                            alpha=1.0)

        # # draw the perpendicular component vector
        # perpendicular_component_line = [[0.0, 0.0], perpendicular_component]
        # perpendicular_component_line = linalg.rotate_and_translate_vectors(
        #    perpendicular_component_line, robot_theta, robot_pos)
        # self.viewer.current_frame.add_lines(  [perpendicular_component_line],
        #                                       linewidth = 0.01,
        #                                       color = "blue",
        #                                       alpha = 0.8)

        # # draw the parallel component vector
        # parallel_component_line = [[0.0, 0.0], parallel_component]
        # parallel_component_line = linalg.rotate_and_translate_vectors(
        #    parallel_component_line, robot_theta, robot_pos)
        # self.viewer.current_frame.add_lines(  [parallel_component_line],
        #                                       linewidth = 0.01,
        #                                       color = "red",
        #                                       alpha = 0.8)

        # draw the computed follow-wall vector
        fw_heading_vector = linalg.scale(linalg.unit(fw_heading_vector),
            VECTOR_LEN)
        vector_line = [[0.0, 0.0], fw_heading_vector]
        vector_line = linalg.rotate_and_translate_vectors(vector_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([vector_line],
                                            linewidth=0.02,
                                            color="orange",
                                            alpha=1.0)


#**********************************************************
class GoToAngleController(Controller):

    def __init__(self, supervisor_interface, kP, kI, kD):
        """Bind the supervisor interface and initialise the controller variables
        
        Keywords:
            supervisor_interface -> SupervisorControllerInterface
			kP -> float
			kI -> float
			kD -> float
        """
		super(GoToAngleController, self).__init__(supervisor_interface, 
			kP, kI, kD)

    def execute(self, theta_d):
		"""Calculate PID terms and generate the supervisor's v and omega"""
		
        theta = self.supervisor.estimated_pose().theta
        e = self.normalize_angle(theta_d - theta)
        omega = self.kP * e

        self.supervisor.set_outputs(1.0, omega)

    def normalize_angle(theta):
		"""Map the given angle to the equivalent angle in [ -pi, pi ]
		
		Keywords:
			theta -> float
		"""
        return atan2(sin(theta), cos(theta))


#**********************************************************
class GTGAndAOController(Controller):

    def __init__(self, supervisor_interface, kP, kI, kD):
        """Bind the supervisor interface and initialise the controller variables
        
        Keywords:
            supervisor_interface -> SupervisorControllerInterface
			kP -> float
			kI -> float
			kD -> float
        """
		super(GTGAndAOController, self).__init__(supervisor_interface, 
			kP, kI, kD)

        # initialize controllers to blend
        self.go_to_goal_controller = GoToGoalController(supervisor_interface, 
			kP, kI, kD)
        self.avoid_obstacles_controller = AvoidObstaclesController(
			supervisor_interface, kP, kI, kD)

        # sensor gains (weights)
        self.avoid_obstacles_controller.sensor_gains = [
          1.0 - ((0.4 * abs(p.theta)) / pi)
          for p in self.avoid_obstacles_controller.proximity_sensor_placements]

        # blending factor
        self.alpha = 0.1  # go-to-goal heading is given this much weight,
                          # avoid-obstacles is given the remaining weight

        # key vectors and data (initialize to any non-zero vector)
        self.obstacle_vectors = ([[1.0, 0.0]]
            * len(self.avoid_obstacles_controller.proximity_sensor_placements))
        self.ao_heading_vector = [1.0, 0.0]
        self.gtg_heading_vector = [1.0, 0.0]
        self.blended_heading_vector = [1.0, 0.0]

    def update_heading(self):
		"""Generate and store vectors generated by the two controller types"""
		
        self.gtg_heading_vector = (
            self.go_to_goal_controller.calculate_gtg_heading_vector())
        self.ao_heading_vector, self.obstacle_vectors = (
            self.avoid_obstacles_controller.calculate_ao_heading_vector())

        # normalize the heading vectors
        self.gtg_heading_vector = linalg.unit(self.gtg_heading_vector)
        self.ao_heading_vector = linalg.unit(self.ao_heading_vector)

        # generate the blended vector
        self.blended_heading_vector = linalg.add(
            linalg.scale(self.gtg_heading_vector, self.alpha),
            linalg.scale(self.ao_heading_vector, 1.0 - self.alpha))

    def execute(self):
		"""Calculate PID terms and generate the supervisor's v and omega"""
		
        # calculate the time that has passed since the last control iteration
        current_time = self.supervisor.time()
        dt = current_time - self.prev_time

        # calculate the error terms
        theta_d = atan2(self.blended_heading_vector[1],
            self.blended_heading_vector[0])
        eP = theta_d
        eI = self.prev_eI + eP * dt
        eD = (eP - self.prev_eP) / dt

        # calculate angular velocity
        omega = self.kP * eP + self.kI * eI + self.kD * eD

        # calculate translational velocity
        # velocity is v_max when omega is 0,
        # drops rapidly to zero as |omega| rises
        v = self.supervisor.v_max() / (abs(omega) + 1) ** 1.5

        # store values for next control iteration
        self.prev_time = current_time
        self.prev_eP = eP
        self.prev_eI = eI

        self.supervisor.set_outputs(v, omega)

        # === FOR DEBUGGING ===
        if debug:
			self._print_vars(eP, eI, eD, v, omega)


#***********************************************************
class GTGAndAOControllerView(object):

    def __init__(self, viewer, supervisor):
        """Bind the viewe, supervisor and controller
		
		Keywords:
			viewer -> Viewer
			supervisor -> SupervisorControllerInterface
		"""
		self.viewer = viewer
        self.supervisor = supervisor
        self.gtg_and_ao_controller = supervisor.gtg_and_ao_controller

    def draw_gtg_and_ao_controller_to_frame(self):
		"""Draw blended gtg & ao controller's internal state to the frame"""
        robot_pos, robot_theta = self.supervisor.estimated_pose.vunpack()

        # draw the detected environment boundary (i.e. sensor readings)
        obstacle_vertexes = self.gtg_and_ao_controller.obstacle_vectors[:]
        obstacle_vertexes.append(obstacle_vertexes[0])  # close the drawn
                                                        #     polygon
        obstacle_vertexes = linalg.rotate_and_translate_vectors(
            obstacle_vertexes, robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([obstacle_vertexes],
                                            linewidth=0.005,
                                            color="black",
                                            alpha=1.0)

        # draw the computed avoid-obstacles vector
        ao_heading_vector = linalg.scale(linalg.unit(
            self.gtg_and_ao_controller.ao_heading_vector), VECTOR_LEN)
        vector_line = [[0.0, 0.0], ao_heading_vector]
        vector_line = linalg.rotate_and_translate_vectors(vector_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([vector_line],
                                            linewidth=0.005,
                                            color="red",
                                            alpha=1.0)

        # draw the computed go-to-goal vector
        gtg_heading_vector = linalg.scale(linalg.unit(
            self.gtg_and_ao_controller.gtg_heading_vector), VECTOR_LEN)
        vector_line = [[0.0, 0.0], gtg_heading_vector]
        vector_line = linalg.rotate_and_translate_vectors(vector_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([vector_line],
                                            linewidth=0.005,
                                            color="dark green",
                                            alpha=1.0)

        # draw the computed blended vector
        blended_heading_vector = linalg.scale(linalg.unit(
            self.gtg_and_ao_controller.blended_heading_vector), VECTOR_LEN)
        vector_line = [[0.0, 0.0], blended_heading_vector]
        vector_line = linalg.rotate_and_translate_vectors(vector_line,
            robot_theta, robot_pos)
        self.viewer.current_frame.add_lines([vector_line],
                                            linewidth=0.02,
                                            color="blue",
                                            alpha=1.0)
