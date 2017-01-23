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

from math import pi
from time import time

import pickle

import pygtk
pygtk.require('2.0')
import gtk
import gobject

from physics import CollisionException
from supervisor import GoalReachedException

#**********CLASSES IN MODULE**********************

#Viewer
#Frame
#Painter
#ColorPalette

#*************************************************

# user response codes for file chooser dialog buttons
LS_DIALOG_RESPONSE_CANCEL = 1
LS_DIALOG_RESPONSE_ACCEPT = 2


#***********************************************************
class Viewer(object):
    """Creates the GTK 2.0 gui for the simulator.

    Attributes:
        world -> World object
        period -> float
        map_type -> boolean
        current_frame -> Frame object
        pixels_per_meter -> int
        view_width_pixels -> int
        view_height_pixels -> int
        window -> GTK window object
        drawing_area -> GTK drawing area object
        painter -> Painter object
        draw_invisibles -> boolean
        alert_box -> GTK label object

    Methods:
        __init__(world, period, zoom, map_type)
        initialise_viewer()
        new_frame()
        draw_frame()
        step_sim()
        """
    def __init__(self, world, period, zoom, map_type):
        """"Generates the gui of the simulation.

        Params:
            world -> World object
            period -> float
            zoom -> int
            map_type -> boolean
        """
        # bind the world and it's parameters
        self.world = world
        self.period = period
        self.map_type = map_type

        # initialize frame
        self.current_frame = Frame()

        # initialize camera parameters
        self.pixels_per_meter = zoom
        self.view_width_pixels = int(self.world.width * self.pixels_per_meter)
        self.view_height_pixels = int(self.world.height * self.pixels_per_meter)

        # initialize the window
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.set_title('Pass the Butter')
        self.window.set_resizable(False)
        self.window.connect('delete_event', self.on_delete)

        # initialize the drawing_area
        self.drawing_area = gtk.DrawingArea()
        self.drawing_area.set_size_request(self.view_width_pixels,
            self.view_height_pixels)
        self.drawing_area.connect('expose_event', self.on_expose)

        # initialize the painter
        self.painter = Painter(self.drawing_area, self.pixels_per_meter)

        # SIMULATOR CONTROL SECTION

        # == initialize the buttons

        # build the play button
        self._button_play = gtk.Button('Play')
        play_image = gtk.Image()
        play_image.set_from_stock(gtk.STOCK_MEDIA_PLAY, gtk.ICON_SIZE_BUTTON)
        self._button_play.set_image(play_image)
        self._button_play.set_image_position(gtk.POS_LEFT)
        self._button_play.connect('clicked', self.on_play)

        # build the pause button
        self._button_pause = gtk.Button('Pause')
        pause_image = gtk.Image()
        pause_image.set_from_stock(gtk.STOCK_MEDIA_STOP, gtk.ICON_SIZE_BUTTON)
        self._button_pause.set_image(pause_image)
        self._button_pause.set_image_position(gtk.POS_LEFT)
        self._button_pause.connect('clicked', self.on_pause)

        # build the step button
        self._button_step = gtk.Button('Step')
        step_image = gtk.Image()
        step_image.set_from_stock(gtk.STOCK_MEDIA_NEXT, gtk.ICON_SIZE_BUTTON)
        self._button_step.set_image(step_image)
        self._button_step.set_image_position(gtk.POS_LEFT)
        self._button_step.connect('clicked', self.on_step)

        # build the reset button
        self._button_reset = gtk.Button('Reset')
        reset_image = gtk.Image()
        reset_image.set_from_stock(gtk.STOCK_MEDIA_REWIND, gtk.ICON_SIZE_BUTTON)
        self._button_reset.set_image(reset_image)
        self._button_reset.set_image_position(gtk.POS_LEFT)
        self._button_reset.connect('clicked', self.on_reset)

        # build the save map button
        self._button_save_map = gtk.Button('Save Map')
        save_map_image = gtk.Image()
        save_map_image.set_from_stock(gtk.STOCK_SAVE, gtk.ICON_SIZE_BUTTON)
        self._button_save_map.set_image(save_map_image)
        self._button_save_map.set_image_position(gtk.POS_LEFT)
        self._button_save_map.connect('clicked', self.on_save_map)

        # build the load map button
        self._button_load_map = gtk.Button('Load Map')
        load_map_image = gtk.Image()
        load_map_image.set_from_stock(gtk.STOCK_OPEN, gtk.ICON_SIZE_BUTTON)
        self._button_load_map.set_image(load_map_image)
        self._button_load_map.set_image_position(gtk.POS_LEFT)
        self._button_load_map.connect('clicked', self.on_load_map)

        # build the random map buttons
        self._button_random_map = gtk.Button('Random Map')
        random_map_image = gtk.Image()
        random_map_image.set_from_stock(gtk.STOCK_REFRESH, gtk.ICON_SIZE_BUTTON)
        self._button_random_map.set_image(random_map_image)
        self._button_random_map.set_image_position(gtk.POS_LEFT)
        self._button_random_map.connect('clicked', self.on_random_map)

        # build the draw-invisibles toggle button
        self.draw_invisibles = False  # controls whether invisible world
                                      #    elements are displayed
        self._button_draw_invisibles = gtk.Button()
        self._decorate_draw_invisibles_button_inactive()
        self._button_draw_invisibles.set_image_position(gtk.POS_LEFT)
        self._button_draw_invisibles.connect('clicked', self.on_draw_invisibles)

        # build the robot parameters toggle button
        self._button_robot_parameters = gtk.Button('Robot Parameters')
        robot_parameters_image = gtk.Image()
        robot_parameters_image.set_from_stock(gtk.STOCK_REFRESH,
            gtk.ICON_SIZE_BUTTON)
        self._button_robot_parameters.set_image(random_map_image)
        self._button_robot_parameters.set_image_position(gtk.POS_LEFT)
        self._button_robot_parameters.connect('clicked',
            self.on_robot_parameters)

        # == lay out the window

        # pack the simulation control buttons
        sim_controls_box = gtk.HBox(spacing=5)
        sim_controls_box.pack_start(self._button_play, False, False)
        sim_controls_box.pack_start(self._button_pause, False, False)
        sim_controls_box.pack_start(self._button_step, False, False)
        sim_controls_box.pack_start(self._button_reset, False, False)

        # pack the map control buttons
        map_controls_box = gtk.HBox(spacing=5)
        map_controls_box.pack_start(self._button_save_map, False, False)
        map_controls_box.pack_start(self._button_load_map, False, False)
        map_controls_box.pack_start(self._button_random_map, False, False)

        # pack the invisibles and robot parameters button
        misc_button_box = gtk.HBox()
        misc_button_box.pack_start(self._button_draw_invisibles, False,
            False)
        misc_button_box.pack_start(self._button_robot_parameters,
            False, False)

        # align the controls
        sim_controls_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        map_controls_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        misc_button_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)

        sim_controls_alignment.add(sim_controls_box)
        map_controls_alignment.add(map_controls_box)
        misc_button_alignment.add(misc_button_box)

        # create the alert box
        self.alert_box = gtk.Label()

        # create the world time box
        self._label_world_time_heading = gtk.Label('World Time: ')
        self._label_world_time = gtk.Label('0.00')
        self._label_world_time_units = gtk.Label('s')

        # pack the world time label
        world_time_box = gtk.HBox()
        world_time_box.pack_start(self._label_world_time_heading, False, False)
        world_time_box.pack_start(self._label_world_time, False, False)
        world_time_box.pack_start(self._label_world_time_units, False, False)
        world_time_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        world_time_alignment.add(world_time_box)

        # build the robot controller state label
        self._label_controller_state_heading = gtk.Label('Controller State: ')
        self._label_controller_state = gtk.Label('')

        # pack the robot controller label
        controller_state_box = gtk.HBox()
        controller_state_box.pack_start(self._label_controller_state_heading,
            False, False)
        controller_state_box.pack_start(self._label_controller_state, False,
            False)
        controller_status_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        controller_status_alignment.add(controller_state_box)

        # lay out the simulation view and all of the controls
        simulator_box = gtk.VBox()
        simulator_box.pack_start(self.drawing_area)
        simulator_box.pack_start(world_time_alignment, False, False, 5)
        simulator_box.pack_start(self.alert_box, False, False, 5)
        simulator_box.pack_start(sim_controls_alignment, False, False, 5)
        simulator_box.pack_start(map_controls_alignment, False, False, 5)
        simulator_box.pack_start(misc_button_alignment, False, False, 5)
        simulator_box.pack_start(controller_status_alignment, False, False, 5)

        # CONTROLLER PARAMETERS SECTION

        # == initialize the controller parameters
        self.control_params = [
            [
                ['Refresh rate', '10.0', 'fps']],
            [
                ['gtg kP', '5.0', ''],
                ['gtg kI', '0.0', ''],
                ['gtg kD', '0.0', '']],
            [
                ['ao kP', '10.0', ''],
                ['ao kI', '0.0', ''],
                ['ao kD', '0.0', '']],
            [
                ['fw kP', '10.0', ''],
                ['fw kI', '0.0', ''],
                ['fw kD', '0.0', '']],
            [
                ['gta kP', '5.0', ''],
                ['gta kI', '0.0', ''],
                ['gta kD', '0.0', '']],
            [
                ['gtgao kP', '10.0', ''],
                ['gtgao kI', '0.0', ''],
                ['gtgao kD', '0.0', '']]
            ]

        # == initialize the buttons

        # build the robot config paramters heading label
        self._label_control_config = gtk.Label('Controller Settings')

        # build the save control config button
        self._button_save_control_config = gtk.Button('Save Config')
        save_control_config_image = gtk.Image()
        save_control_config_image.set_from_stock(gtk.STOCK_SAVE,
            gtk.ICON_SIZE_BUTTON)
        self._button_save_control_config.set_image(save_control_config_image)
        self._button_save_control_config.set_image_position(gtk.POS_LEFT)
        self._button_save_control_config.connect('clicked',
            self.on_save_control_config)

        # build the load control config button
        self._button_load_control_config = gtk.Button('Load Config')
        load_control_config_image = gtk.Image()
        load_control_config_image.set_from_stock(gtk.STOCK_OPEN,
            gtk.ICON_SIZE_BUTTON)
        self._button_load_control_config.set_image(load_control_config_image)
        self._button_load_control_config.set_image_position(gtk.POS_LEFT)
        self._button_load_control_config.connect('clicked',
            self.on_load_control_config)

        # build the apply control config button
        self._button_apply_control_config = gtk.Button('Apply')
        apply_control_config_image = gtk.Image()
        apply_control_config_image.set_from_stock(gtk.STOCK_APPLY,
            gtk.ICON_SIZE_BUTTON)
        self._button_apply_control_config.set_image(apply_control_config_image)
        self._button_apply_control_config.set_image_position(gtk.POS_LEFT)
        self._button_apply_control_config.set_sensitive(False)
        self._button_apply_control_config.connect('clicked',
            self.on_apply_control_config)

        # == initialize the text entry boxes

        # add the parameters to tables
        # The default values below are the robot properties for a Zumo 32U4
        # robot with raspberry pi mounted on top

        # initialise parameter table counter to 0
        self.num_control_param_tables = 0

        self.add_control_parameter_table('Simulation refresh rate:')
        self.add_control_parameter_input(self.control_params[0][0], 0)

        self.add_control_parameter_table('Go to goal PID values:')
        self.add_control_parameter_input(self.control_params[1][0], 1)
        self.add_control_parameter_input(self.control_params[1][1], 1)
        self.add_control_parameter_input(self.control_params[1][2], 1)

        self.add_control_parameter_table('Avoid obstacles PID values:')
        self.add_control_parameter_input(self.control_params[2][0], 2)
        self.add_control_parameter_input(self.control_params[2][1], 2)
        self.add_control_parameter_input(self.control_params[2][2], 2)

        self.add_control_parameter_table('Follow wall PID values:')
        self.add_control_parameter_input(self.control_params[3][0], 3)
        self.add_control_parameter_input(self.control_params[3][1], 3)
        self.add_control_parameter_input(self.control_params[3][2], 3)

        self.add_control_parameter_table('Go to angle PID values:')
        self.add_control_parameter_input(self.control_params[4][0], 4)
        self.add_control_parameter_input(self.control_params[4][1], 4)
        self.add_control_parameter_input(self.control_params[4][2], 4)

        self.add_control_parameter_table(
            'Go to goal and avoid PID values:')
        self.add_control_parameter_input(self.control_params[5][0], 5)
        self.add_control_parameter_input(self.control_params[5][1], 5)
        self.add_control_parameter_input(self.control_params[5][2], 5)

         # == lay out the window

        # pack the robot config heading label # == lay out the window

        # pack the robot config heading label
        control_config_heading_box = gtk.HBox()
        control_config_heading_box.pack_start(self._label_control_config, False,
            False)

        # pack the save and load robot buttons
        load_control_button_box = gtk.HBox()
        load_control_button_box.pack_start(self._button_save_control_config,
            False, False)
        load_control_button_box.pack_start(self._button_load_control_config,
            False, False)

        # pack the apply robot button
        apply_control_params_button_box = gtk.HBox()
        apply_control_params_button_box.pack_start(
            self._button_apply_control_config, False, False)

        # pack the robot parameters
        control_parameters_box = gtk.VBox()
        for i in range(self.num_control_param_tables):
            control_parameters_box.pack_start(getattr(self,
                'control_param_table_heading_' + str(i)), False, False)
            control_parameters_box.pack_start(getattr(self,
                'control_param_table_' + str(i)), False, False)

        # align the controls
        control_config_heading_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        control_config_heading_alignment.add(control_config_heading_box)

        load_control_button_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        load_control_button_alignment.add(load_control_button_box)

        control_parameters_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        control_parameters_alignment.add(control_parameters_box)

        apply_control_params_button_alignment = gtk.Alignment(
            0.5, 0.0, 0.0, 1.0)
        apply_control_params_button_alignment.add(
            apply_control_params_button_box)

        # lay out the parameter inputs for the simulator
        control_param_box = gtk.VBox()
        control_param_box.pack_start(control_config_heading_alignment, False,
            False, 5)
        control_param_box.pack_start(load_control_button_alignment, False,
            False, 5)
        control_param_box.pack_start(control_parameters_alignment, False,
            False, 5)
        control_param_box.pack_start(apply_control_params_button_alignment,
            False, False, 5)

        # pack the simulator and control parameter boxes next to each other
        layout_box = gtk.HBox()
        layout_box.set_spacing(10)
        layout_box.pack_start(simulator_box, False, False)
        layout_box.pack_start(control_param_box, False, False)

        # apply the layout
        self.window.add(layout_box)

        # ROBOT PARAMETERS WINDOW

        # == initialize the robot parameters
        self.robot_params = [
            [
                ['Wheel radius', '0.0194', 'm'],
                ['Wheel base length', '0.0885', 'm'],
                ['Wheel ticks per rev', '300', ''],
                ['Max wheel drive rate', '100.0', 'rpm'],
                ['Robot body width', '0.1', 'm'],
                ['Robot body length', '0.1', 'm'],
                ['Robot payload width', '0.11', 'm'],
                ['Robot payload length', '0.13', 'm'],
                ['Payload centre offset', '-0.025', 'm']],
            [
                ['Sensor min value', '18.0', 'mV'],
                ['Sensor max value', '3960.0', 'mV'],
                ['Sensor min range', '0.01', 'm'],
                ['Sensor max range', '0.3', 'm'],
                ['Sensor view angle', '40.0', 'deg']],
            [
                ['x', '0.045', 'm'],
                ['y', '0.040', 'm'],
                ['angle', '90.0', 'deg']],
            [
                ['x', '0.045', 'm'],
                ['y', '0.000', 'm'],
                ['angle', '0.0', 'deg']],
            [
                ['x', '0.045', 'm'],
                ['y', '-0.040', 'm'],
                ['angle', '-90.0', 'deg']]
            ]

        # show the simulator window
        self.window.show_all()

    ### Setup of control parameters tables

    def add_control_parameter_table(self, heading):
        """Adds a heading above each parameter table."""
        # increment table counter
        self.num_control_param_tables += 1
        table_num = self.num_control_param_tables - 1

        # create param counter for this table
        setattr(self, 'num_control_params_' + str(table_num), 0)

        # create table heading
        _heading = gtk.Label(heading)

        # justify the parameter table heading to the left
        setattr(self, 'control_param_table_heading_' + str(table_num),
            gtk.Alignment(0.0, 0.0, 0.0, 1.0))
        getattr(self, 'control_param_table_heading_' + str(
            table_num)).add(_heading)
        getattr(self, 'control_param_table_heading_' + str(
            table_num)).set_padding(10, 10, 0, 0)

        # create table
        setattr(self, 'control_param_table_' + str(table_num), gtk.Table(
            1, 3, False))

    def add_control_parameter_input(self, param, table):
        """Add a robot config parameter to the GUI.

        NOTE: The text entry box for each parameter is given a name based on
        its row so that if the order of parameters is changed later on the
        parameter names need to be changed when the add_parameter_input()
        function is called."""

        # assign table for parameter to be added to
        _table = getattr(self, 'control_param_table_' + str(table))

        # parameter label
        _label = gtk.Label(param[0])

        # justify the parameter labels to the right
        _label_alignment = gtk.Alignment(1.0, 0.0, 0.0, 1.0)
        _label_alignment.add(_label)

        # increment param counter
        setattr(self, 'num_control_params_' + str(table),
            getattr(self, 'num_control_params_' + str(table)) + 1)
        row = getattr(self, 'num_control_params_' + str(table)) - 1

        # parameter text entry box
        setattr(self, 'control_param_' + str(table) + '_' + str(
            row), gtk.Entry(max=0))
        getattr(self, 'control_param_' + str(table) + '_' + str(
            row)).set_width_chars(8)
        getattr(self, 'control_param_' + str(table) + '_' + str(
            row)).set_alignment(1.0)
        getattr(self, 'control_param_' + str(table) + '_' + str(
            row)).set_text(param[1])
        getattr(self, 'control_param_' + str(table) + '_' + str(
            row)).connect('activate', self.activate_apply_control_config)

        # parameter unit
        _unit = gtk.Label(param[2])
        _unit_alignment = gtk.Alignment(0.0, 0.0, 0.0, 1.0)
        _unit_alignment.add(_unit)

        # add parameter to parameters table
        _table.attach(_label_alignment, 0, 1, row, row + 1)
        _table.attach(getattr(self, 'control_param_' + str(table) + '_' + str(
            row)), 1, 2, row, row + 1)
        _table.attach(_unit_alignment, 2, 3, row, row + 1)

    def activate_apply_control_config(self, widget):
        self._button_apply_control_config.set_sensitive(True)

    def get_control_parameters(self):
        """Create an array of control config parameters from the GUI."""
        return self.control_params

    def set_control_parameters(self, param_list):
        """Load the control config parameters into the GUI text boxes."""
        # save the values from the file into the parameter list
        self.control_params = param_list

        # save the values from the file into the text boxes
        for i in range(self.num_control_param_tables):
            for j in range(getattr(self, 'num_control_params_' + str(i))):
                self.control_params[i][j][1] = getattr(self,
                    'control_param_' + str(i) + '_' + str(j)).set_text(
                        param_list[i][j][1])

    ### Setup of robot parameters window

    def robot_parameter_window(self):
        # ROBOT PARAMETERS SECTION

        # == initialize the buttons

        # build the robot config paramters heading label
        self._label_robot_config = gtk.Label('Robot Configuration')

        # build the save robot config button
        self._button_save_robot_config = gtk.Button('Save Config')
        save_robot_config_image = gtk.Image()
        save_robot_config_image.set_from_stock(gtk.STOCK_SAVE,
            gtk.ICON_SIZE_BUTTON)
        self._button_save_robot_config.set_image(save_robot_config_image)
        self._button_save_robot_config.set_image_position(gtk.POS_LEFT)
        self._button_save_robot_config.connect('clicked',
            self.on_save_robot_config)

        # build the load robot config button
        self._button_load_robot_config = gtk.Button('Load Config')
        load_robot_config_image = gtk.Image()
        load_robot_config_image.set_from_stock(gtk.STOCK_OPEN,
            gtk.ICON_SIZE_BUTTON)
        self._button_load_robot_config.set_image(load_robot_config_image)
        self._button_load_robot_config.set_image_position(gtk.POS_LEFT)
        self._button_load_robot_config.connect('clicked',
            self.on_load_robot_config)

        # build the apply robot config button
        self._button_apply_robot_config = gtk.Button('Apply')
        apply_robot_config_image = gtk.Image()
        apply_robot_config_image.set_from_stock(gtk.STOCK_APPLY,
            gtk.ICON_SIZE_BUTTON)
        self._button_apply_robot_config.set_image(apply_robot_config_image)
        self._button_apply_robot_config.set_image_position(gtk.POS_LEFT)
        self._button_apply_robot_config.set_sensitive(False)
        self._button_apply_robot_config.connect('clicked',
            self.on_apply_robot_config)

        # == initialize the text entry boxes

        # add the parameters to tables
        # The default values below are the robot properties for a Zumo 32U4
        # robot with raspberry pi mounted on top

        # initialise parameter table counter to 0
        self.num_robot_param_tables = 0

        self.add_robot_parameter_table('Physical robot parameters:')
        self.add_robot_parameter_input(self.robot_params[0][0], 0)
        self.add_robot_parameter_input(self.robot_params[0][1], 0)
        self.add_robot_parameter_input(self.robot_params[0][2], 0)
        self.add_robot_parameter_input(self.robot_params[0][3], 0)
        self.add_robot_parameter_input(self.robot_params[0][4], 0)
        self.add_robot_parameter_input(self.robot_params[0][5], 0)
        self.add_robot_parameter_input(self.robot_params[0][6], 0)
        self.add_robot_parameter_input(self.robot_params[0][7], 0)
        self.add_robot_parameter_input(self.robot_params[0][8], 0)

        self.add_robot_parameter_table('Sensor parameters:')
        self.add_robot_parameter_input(self.robot_params[1][0], 1)
        self.add_robot_parameter_input(self.robot_params[1][1], 1)
        self.add_robot_parameter_input(self.robot_params[1][2], 1)
        self.add_robot_parameter_input(self.robot_params[1][3], 1)
        self.add_robot_parameter_input(self.robot_params[1][4], 1)

        self.add_robot_parameter_table('Sensor 1 position:')
        self.add_robot_parameter_input(self.robot_params[2][0], 2)
        self.add_robot_parameter_input(self.robot_params[2][1], 2)
        self.add_robot_parameter_input(self.robot_params[2][2], 2)

        self.add_robot_parameter_table('Sensor 1 position:')
        self.add_robot_parameter_input(self.robot_params[3][0], 3)
        self.add_robot_parameter_input(self.robot_params[3][1], 3)
        self.add_robot_parameter_input(self.robot_params[3][2], 3)

        self.add_robot_parameter_table('Sensor 1 position:')
        self.add_robot_parameter_input(self.robot_params[4][0], 4)
        self.add_robot_parameter_input(self.robot_params[4][1], 4)
        self.add_robot_parameter_input(self.robot_params[4][2], 4)

         # == lay out the window

        # pack the robot config heading label # == lay out the window

        # pack the robot config heading label
        robot_config_heading_box = gtk.HBox()
        robot_config_heading_box.pack_start(self._label_robot_config, False,
            False)

        # pack the save and load robot buttons
        load_robot_button_box = gtk.HBox()
        load_robot_button_box.pack_start(self._button_save_robot_config, False,
            False)
        load_robot_button_box.pack_start(self._button_load_robot_config, False,
            False)

        # pack the apply robot button
        apply_robot_params_button_box = gtk.HBox()
        apply_robot_params_button_box.pack_start(
            self._button_apply_robot_config, False, False)

        # pack the robot parameters
        robot_parameters_box = gtk.VBox()
        for i in range(self.num_robot_param_tables):
            robot_parameters_box.pack_start(getattr(self,
                'robot_param_table_heading_' + str(i)), False, False)
            robot_parameters_box.pack_start(getattr(self,
                'robot_param_table_' + str(i)), False, False)

        # align the controls
        robot_config_heading_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        robot_config_heading_alignment.add(robot_config_heading_box)

        load_robot_button_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        load_robot_button_alignment.add(load_robot_button_box)

        robot_parameters_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        robot_parameters_alignment.add(robot_parameters_box)

        apply_robot_params_button_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        apply_robot_params_button_alignment.add(apply_robot_params_button_box)

        # lay out the parameter inputs for the simulator
        parameters_box = gtk.VBox()
        parameters_box.pack_start(robot_config_heading_alignment, False, False,
            5)
        parameters_box.pack_start(load_robot_button_alignment, False, False, 5)
        parameters_box.pack_start(robot_parameters_alignment, False, False, 5)
        parameters_box.pack_start(apply_robot_params_button_alignment, False,
            False, 5)

        # create robot parameters dialog box
        self.parameters_window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.parameters_window.set_title('Robot Parameters')
        self.parameters_window.add(parameters_box)

        self.parameters_window.show_all()

    def add_robot_parameter_table(self, heading):
        """Adds a heading above each parameter table."""
        # increment table counter
        self.num_robot_param_tables += 1
        table_num = self.num_robot_param_tables - 1

        # create param counter for this table
        setattr(self, 'num_robot_params_' + str(table_num), 0)

        # create table heading
        _heading = gtk.Label(heading)

        # justify the parameter table heading to the left
        setattr(self, 'robot_param_table_heading_' + str(table_num),
            gtk.Alignment(0.0, 0.0, 0.0, 1.0))
        getattr(self, 'robot_param_table_heading_' + str(
            table_num)).add(_heading)
        getattr(self, 'robot_param_table_heading_' + str(
            table_num)).set_padding(10, 10, 0, 0)

        # create table
        setattr(self, 'robot_param_table_' + str(table_num), gtk.Table(
            1, 3, False))

    def add_robot_parameter_input(self, param, table):
        """Add a robot config parameter to the GUI.

        NOTE: The text entry box for each parameter is given a name based on
        its row so that if the order of parameters is changed later on the
        parameter names need to be changed when the add_parameter_input()
        function is called."""

        # assign table for parameter to be added to
        _table = getattr(self, 'robot_param_table_' + str(table))

        # parameter label
        _label = gtk.Label(param[0])

        # justify the parameter labels to the right
        _label_alignment = gtk.Alignment(1.0, 0.0, 0.0, 1.0)
        _label_alignment.add(_label)

        # increment param counter
        setattr(self, 'num_robot_params_' + str(table),
            getattr(self, 'num_robot_params_' + str(table)) + 1)
        row = getattr(self, 'num_robot_params_' + str(table)) - 1

        # parameter text entry box
        setattr(self, 'robot_param_' + str(table) + '_' + str(
            row), gtk.Entry(max=0))
        getattr(self, 'robot_param_' + str(table) + '_' + str(
            row)).set_width_chars(8)
        getattr(self, 'robot_param_' + str(table) + '_' + str(
            row)).set_alignment(1.0)
        getattr(self, 'robot_param_' + str(table) + '_' + str(
            row)).set_text(param[1])
        getattr(self, 'robot_param_' + str(table) + '_' + str(
            row)).connect('activate', self.activate_apply_robot_config)

        # parameter unit
        _unit = gtk.Label(param[2])
        _unit_alignment = gtk.Alignment(0.0, 0.0, 0.0, 1.0)
        _unit_alignment.add(_unit)

        # add parameter to parameters table
        _table.attach(_label_alignment, 0, 1, row, row + 1)
        _table.attach(getattr(self, 'robot_param_' + str(table) + '_' + str(
            row)), 1, 2, row, row + 1)
        _table.attach(_unit_alignment, 2, 3, row, row + 1)

    def activate_apply_robot_config(self, widget):
        self._button_apply_robot_config.set_sensitive(True)

    def get_robot_parameters(self):
        """Create an array of robot config parameters from the GUI."""
        return self.robot_params

    def set_robot_parameters(self, param_list):
        """Load the robot config parameters into the GUI text boxes."""
        self.robot_params = param_list

    ### Setup of Viewer

    def initialise_viewer(self):
        """Initialises the world in the viewer and starts the gui."""
        # gtk simulation event source - for simulation control
        self.sim_event_source = gobject.idle_add(self.world.initialise_world,
            self.map_type)  # we use this opportunity to initialize the world

        # set intial state of buttons
        self._control_panel_state_init()

        # start gtk
        gtk.main()

    def new_frame(self):
        """Creates a new gui frame."""
        self.current_frame = Frame()

    def draw_frame(self):
        """Draws the gui frame to the screen."""
        self.drawing_area.queue_draw_area(0, 0, self.view_width_pixels,
            self.view_height_pixels)

    def _control_panel_state_init(self):
        """Sets the initial gui button states."""
        self.alert_box.set_text('')
        self._button_play.set_sensitive(True)
        self._button_pause.set_sensitive(False)
        self._button_step.set_sensitive(True)
        self._button_reset.set_sensitive(False)

    def _control_panel_state_playing(self):
        """Sets button states if playing simulation."""
        self._button_play.set_sensitive(False)
        self._button_pause.set_sensitive(True)
        self._button_reset.set_sensitive(True)

    def _control_panel_state_paused(self):
        """Sets button states if simulation is paused."""
        self._button_play.set_sensitive(True)
        self._button_pause.set_sensitive(False)
        self._button_reset.set_sensitive(True)

    def _control_panel_state_finished(self, alert_text):
        """Sets button states if simulation is finished.

        Params:
            alert_text -> GTK object
        """
        self.alert_box.set_text(alert_text)
        self._button_play.set_sensitive(False)
        self._button_pause.set_sensitive(False)
        self._button_step.set_sensitive(False)

    def step_sim(self):
        """Increment the simulation one time period."""
        try:
            self.world.prev_time = time()
            self.world.step()
        except GoalReachedException:
            self.end_sim('Goal Reached!')
        except CollisionException:
            self.end_sim('Collision!')

        # draw the resulting world
        self.world.draw_world()

    def _run_sim(self):
        """Continuously loops through the simulation by repeatedly stepping."""
        self.sim_event_source = gobject.timeout_add(int(self.period * 1000),
            self._run_sim)
        self.step_sim()

    def end_sim(self, alert_text=''):
        """Ends simulation if exception met and stops physical robots

        Params:
            alert_text -> GTK object
        """
        self.world.stop_robots()
        gobject.source_remove(self.sim_event_source)
        self._control_panel_state_finished(alert_text)

  # EVENT HANDLERS:
    def on_play(self, widget):
        """Starts the simulation when the play button is pressed."""
        # this ensures multiple calls to play_sim do not speed up
        #    the simulator
        gobject.source_remove(self.sim_event_source)
        self.world.prev_absolute_time = time()  # reset time to now
        self._run_sim()
        self._control_panel_state_playing()

    def on_pause(self, widget):
        """Pause the simulation when the pause button is pressed."""
        self.world.stop_robots()
        gobject.source_remove(self.sim_event_source)
        self._control_panel_state_paused()

    def on_step(self, widget):
        """Simulation steps one time period when the step button is pressed."""
        gobject.source_remove(self.sim_event_source)
        # reset time so that only 1 time period passes per click
        self.world.prev_absolute_time = time() - self.period
        self._control_panel_state_paused()
        self.step_sim()

    def on_reset(self, widget):
        """The simulation world, robots and supervisors are reset."""
        self.world.stop_robots()
        gobject.source_remove(self.sim_event_source)
        self._control_panel_state_init()
        # reset the world in the viewer
        self.world.initialise_world()

    def on_save_map(self, widget):
        """Saves the obstacle, goal and robot positions to an external file."""
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Save Map',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                        gtk.STOCK_SAVE, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_do_overwrite_confirmation(True)
        file_chooser.set_current_folder('Maps')

        # run the file chooser dialog
        response_id = file_chooser.run()

        # handle the user's response
        if response_id == LS_DIALOG_RESPONSE_CANCEL:
            file_chooser.destroy()
        elif response_id == LS_DIALOG_RESPONSE_ACCEPT:
            self.world.map_manager.save_map(file_chooser.get_filename())
            file_chooser.destroy()

    def on_load_map(self, widget):
        """Load the obstacle, goal and robot positions from an external file."""
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Load Map',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_OPEN,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                            gtk.STOCK_OPEN, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_current_folder('Maps')

        # run the file chooser dialog
        response_id = file_chooser.run()

        # handle the user's response
        if response_id == LS_DIALOG_RESPONSE_CANCEL:
            file_chooser.destroy()
        elif response_id == LS_DIALOG_RESPONSE_ACCEPT:
            self.world.map_manager.load_map(file_chooser.get_filename(),
                self.world)
            file_chooser.destroy()

    def on_random_map(self, widget):
        """Generates a random map for the simulation world.

        A new world is generated with a random positioning and number
        of obstacles, a randomly placed goal, and the robot in the centre
        of the world."""
        gobject.source_remove(self.sim_event_source)
        self.world.initialise_world(True)

    def on_draw_invisibles(self, widget):
        """Display on running simulation robot heading and sensors.

        Draws the robot proximity sensors and distance to obstacles,
        robot heading and the path already travelled by the robot in
        the current simulation."""
        # toggle the draw_invisibles state
        self.draw_invisibles = not self.draw_invisibles
        if self.draw_invisibles:
            self._decorate_draw_invisibles_button_active()
        else:
            self._decorate_draw_invisibles_button_inactive()
        self.world.draw_world()

    def on_save_control_config(self, widget):
        """Saves the control config parameters to an external file."""
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Save Control Config',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                        gtk.STOCK_SAVE, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_do_overwrite_confirmation(True)
        file_chooser.set_current_folder('Control')

        # run the file chooser dialog
        response_id = file_chooser.run()

        # handle the user's response
        if response_id == LS_DIALOG_RESPONSE_CANCEL:
            file_chooser.destroy()
        elif response_id == LS_DIALOG_RESPONSE_ACCEPT:
            with open(file_chooser.get_filename(), 'wb') as _file:
                pickle.dump(self.get_control_parameters(), _file)
            file_chooser.destroy()

    def on_load_control_config(self, widget):
        """Load the control config parameters from an external file."""
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Load Control Config',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_OPEN,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                            gtk.STOCK_OPEN, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_current_folder('Control')

        # run the file chooser dialog
        response_id = file_chooser.run()

        # handle the user's response
        if response_id == LS_DIALOG_RESPONSE_CANCEL:
            file_chooser.destroy()
        elif response_id == LS_DIALOG_RESPONSE_ACCEPT:
            with open(file_chooser.get_filename(), 'rb') as _file:
                config_list = pickle.load(_file)
                self.set_control_parameters(config_list)
            file_chooser.destroy()

    def on_apply_control_config(self, widget):
        """The control parameters are applied and robot reset."""
        # save the values in the text boxes to the control parameters list
        for i in range(self.num_control_param_tables):
            for j in range(getattr(self, 'num_control_params_' + str(i))):
                self.control_params[i][j][1] = getattr(self,
                    'control_param_' + str(i) + '_' + str(j)).get_text()

        self._button_apply_control_config.set_sensitive(False)

        # reset the world in the viewer
        self.world.initialise_world()

    def on_robot_parameters(self, widget):
        """The robot parameters dialog is opened and simulation paused."""
        # Pause simulation
        self.world.stop_robots()
        gobject.source_remove(self.sim_event_source)
        self._control_panel_state_paused()

        # open robot parameters dialog
        self.robot_parameter_window()

    def on_save_robot_config(self, widget):
        """Saves the robot config parameters to an external file."""
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Save Robot Config',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                        gtk.STOCK_SAVE, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_do_overwrite_confirmation(True)
        file_chooser.set_current_folder('Robots')

        # run the file chooser dialog
        response_id = file_chooser.run()

        # handle the user's response
        if response_id == LS_DIALOG_RESPONSE_CANCEL:
            file_chooser.destroy()
        elif response_id == LS_DIALOG_RESPONSE_ACCEPT:
            with open(file_chooser.get_filename(), 'wb') as _file:
                pickle.dump(self.get_robot_parameters(), _file)
            file_chooser.destroy()

    def on_load_robot_config(self, widget):
        """Load the robot config parameters from an external file."""
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Load Robot Config',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_OPEN,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                            gtk.STOCK_OPEN, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_current_folder('Robots')

        # run the file chooser dialog
        response_id = file_chooser.run()

        # handle the user's response
        if response_id == LS_DIALOG_RESPONSE_CANCEL:
            file_chooser.destroy()
        elif response_id == LS_DIALOG_RESPONSE_ACCEPT:
            with open(file_chooser.get_filename(), 'rb') as _file:
                config_list = pickle.load(_file)
                self.set_robot_parameters(config_list)
            file_chooser.destroy()

    def on_apply_robot_config(self, widget):
        """The robot parameters dialog is closed and robot reset."""
        # save the values in the text boxes to the robot parameters list
        for i in range(self.num_robot_param_tables):
            for j in range(getattr(self, 'num_robot_params_' + str(i))):
                self.robot_params[i][j][1] = getattr(self,
                    'robot_param_' + str(i) + '_' + str(j)).get_text()

        # close robot parameters dialog
        self.parameters_window.destroy()

        # reset the world in the viewer
        self.world.initialise_world()

    def on_expose(self, widget, event):
        """Exposes the object to the viewer."""
        self.painter.draw_frame(self.current_frame)

    def on_delete(self, widget, event):
        """Deletes the viewer and quits the gtk object."""
        gtk.main_quit()
        return False

    def _decorate_draw_invisibles_button_active(self):
        """Updates draw invisibles button text if active."""
        draw_invisibles_image = gtk.Image()
        draw_invisibles_image.set_from_stock(gtk.STOCK_REMOVE,
            gtk.ICON_SIZE_BUTTON)
        self._button_draw_invisibles.set_image(draw_invisibles_image)
        self._button_draw_invisibles.set_label('Hide Invisibles')

    def _decorate_draw_invisibles_button_inactive(self):
        """Updates draw invisibles button text if inactive."""
        draw_invisibles_image = gtk.Image()
        draw_invisibles_image.set_from_stock(gtk.STOCK_ADD,
            gtk.ICON_SIZE_BUTTON)
        self._button_draw_invisibles.set_image(draw_invisibles_image)
        self._button_draw_invisibles.set_label('Show Invisibles')


#***********************************************************
class Frame(object):
    """Class to create shapes to be drawn into the Viewer object.

    Attributes:
        draw_list -> list

    Methods:
        __init__()
        add_circle(pos, radius, color, [alpha])
        add_polygons(polygons, color, [alpha])
        add_lines(lines, linewidth, color, [alpha])
    """

    def __init__(self):
        """Initialises the list of drawn objects in the frame."""
        self.draw_list = []

    def add_circle(self, pos, radius, color, alpha=None):
        """Adds drawn circles to the list of frame objects."""
        self.draw_list.append({
          'type': 'circle',
          'pos': pos,
          'radius': radius,
          'color': color,
          'alpha': alpha
        })

    def add_polygons(self, polygons, color, alpha=None):
        """Adds drawn polygons to the list of frame objects."""
        self.draw_list.append({
          'type': 'polygons',
          'polygons': polygons,
          'color': color,
          'alpha': alpha
        })

    def add_lines(self, lines, linewidth, color, alpha=None):
        """Adds drawn lines to the list of frame objects."""
        self.draw_list.append({
          'type': 'lines',
          'lines': lines,
          'linewidth': linewidth,
          'color': color,
          'alpha': alpha
        })


#************************************************************
class Painter(object):
    """Class to draw Frame objects into the Viewer object.

    Attributes:
        draw_list -> list
        drawing_area
        pixels_per_meter

    Methods:
        __init__(drawing_area, pixels_per_meter)
        draw_frame(frame)
        draw_circle(pos, radius, color, [alpha])
        draw_polygons(polygons, color, [alpha])
        draw_lines(lines, linewidth, color, [alpha])
        set_color(cairo_context, color_string, alpha)
    """

    def __init__(self, drawing_area, pixels_per_meter):
        """Initialises the drawing area and its size."""
        self.drawing_area = drawing_area
        self.pixels_per_meter = pixels_per_meter

    def draw_frame(self, frame):
        """Draws the frame and all the object it contains into the frame."""
        context = self.drawing_area.window.cairo_create()

        width_pixels = self.drawing_area.allocation.width
        height_pixels = self.drawing_area.allocation.height

        # transform the the view to metric coordinates
        # - move origin to center of window
        context.translate(width_pixels / 2.0, height_pixels / 2.0)
        # - pull view to edges of window ( also flips y-axis )
        context.scale(self.pixels_per_meter, -self.pixels_per_meter)

        # draw the background in white
        self.set_color(context, 'white', 1.0)
        context.paint()

        draw_list = frame.draw_list
        for component in draw_list:
            if component['type'] == 'circle':
                self.draw_circle(context,
                              component['pos'],
                              component['radius'],
                              component['color'],
                              component['alpha'])

            elif component['type'] == 'polygons':
                self.draw_polygons(context,
                                component['polygons'],
                                component['color'],
                                component['alpha'])

            elif component['type'] == 'lines':
                self.draw_lines(context,
                             component['lines'],
                             component['linewidth'],
                             component['color'],
                             component['alpha'])

    def draw_circle(self, context,
                   pos, radius,
                   color, alpha):
        """Draws circle to the frame."""
        self.set_color(context, color, alpha)
        context.arc(pos[0], pos[1], radius, 0, 2.0 * pi)
        context.fill()

    def draw_polygons(self, context,
                     polygons,
                     color, alpha):
        """Draws polygons to the frame."""
        self.set_color(context, color, alpha)
        for polygon in polygons:
            context.new_path()
            context.move_to(*polygon[0])
            for point in polygon[1:]:
                context.line_to(*point)
            context.fill()

    def draw_lines(self, context,
                  lines, linewidth,
                  color, alpha):
        """Draws lines to the frame."""
        self.set_color(context, color, alpha)
        context.set_line_width(linewidth)
        for line in lines:
            context.new_path()
            context.move_to(*line[0])
            for point in line[1:]:
                context.line_to(*point)
            context.stroke()

    def set_color(self, cairo_context, color_string, alpha):
        """Pulls colour value from the ColorPallette object."""
        ColorPalette.dab(cairo_context, color_string, alpha)


#*************************************************************
class ColorPalette(object):
    """Class to set color for Painter objects.

    Attributes:
        none

    Methods:
        dab(cls, cairo_context, color_string, alpha)
    """
    @classmethod
    def dab(cls, cairo_context, color_string, alpha):
        """Returns requested color and alpha values."""
        vals = [c / 255.0 for c in color_table[color_string]]
        if alpha:
            cairo_context.set_source_rgba(vals[0], vals[1], vals[2], alpha)
        else:
            cairo_context.set_source_rgb(vals[0], vals[1], vals[2])


color_table = {
    "white": (255, 255, 255),
    "black": (0, 0, 0),
    "blue": (0, 0, 255),
    "red": (255, 0, 0),
    "dark red": (139, 0, 0),
    "dark green": (0, 100, 0),
    "orange": (255, 165, 0)
}