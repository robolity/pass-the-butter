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

        # pack the invisibles button
        invisibles_button_box = gtk.HBox()
        invisibles_button_box.pack_start(self._button_draw_invisibles, False,
            False)

        # align the controls
        sim_controls_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        map_controls_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        invisibles_button_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        sim_controls_alignment.add(sim_controls_box)
        map_controls_alignment.add(map_controls_box)
        invisibles_button_alignment.add(invisibles_button_box)

        # create the alert box
        self.alert_box = gtk.Label()

        # create the world time box

        # lay out the simulation view and all of the controls
        simulator_box = gtk.VBox()
        simulator_box.pack_start(self.drawing_area)
        simulator_box.pack_start(self.alert_box, False, False, 5)
        simulator_box.pack_start(sim_controls_alignment, False, False, 5)
        simulator_box.pack_start(map_controls_alignment, False, False, 5)
        simulator_box.pack_start(invisibles_button_alignment, False, False, 5)

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

        # == initialize the text entry boxes

        # Create table for paramters (label, text box, units)
        _num_parameters = 4  # number of rows in table to fit all parameters
        self.parameter_table = gtk.Table(_num_parameters, 3, False)

        # initialise the parameter text entry boxes
        self.r_wheel_radius = gtk.Entry(max=0)
        self.r_wheel_base_length = gtk.Entry(max=0)
        self.r_wheel_ticks_per_rev = gtk.Entry(max=0)
        self.r_max_wheel_drive_rate = gtk.Entry(max=0)

        # add the parameters to the table
        self.add_parameter_input('Wheel radius', '0.0194', 'mm', 1,
            self.r_wheel_radius)
        self.add_parameter_input('Wheel base length', '0.0885', 'mm', 2,
            self.r_wheel_base_length)
        self.add_parameter_input('Wheel ticks per rev', '909.7', '', 3,
            self.r_wheel_ticks_per_rev)
        self.add_parameter_input('Max wheel drive rate', '100', 'rpm', 4,
            self.r_max_wheel_drive_rate)

        # == lay out the window

        # pack the robot config heading label
        robot_config_heading_box = gtk.HBox()
        robot_config_heading_box.pack_start(self._label_robot_config, False,
            False)

        # pack the load robot button
        load_robot_button_box = gtk.HBox()
        load_robot_button_box.pack_start(self._button_save_robot_config, False,
            False)
        load_robot_button_box.pack_start(self._button_load_robot_config, False,
            False)

        # pack the robot parameters
        robot_parameters_box = gtk.HBox()
        robot_parameters_box.pack_start(self.parameter_table, False, False)

        # align the controls
        robot_config_heading_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        load_robot_button_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        robot_parameters_alignment = gtk.Alignment(0.5, 0.0, 0.0, 1.0)
        robot_config_heading_alignment.add(robot_config_heading_box)
        load_robot_button_alignment.add(load_robot_button_box)
        robot_parameters_alignment.add(robot_parameters_box)

        # lay out the parameter inputs for the simulator
        parameters_box = gtk.VBox()
        parameters_box.pack_start(robot_config_heading_alignment, False, False,
            5)
        parameters_box.pack_start(load_robot_button_alignment, False, False, 5)
        parameters_box.pack_start(robot_parameters_alignment, False, False, 5)

        # pack the simulator and parameter boxes next to each other
        layout_box = gtk.HBox()
        layout_box.set_spacing(10)
        layout_box.pack_start(simulator_box, False, False)
        layout_box.pack_start(parameters_box, False, False)

        # apply the layout
        self.window.add(layout_box)

        # show the simulator window
        self.window.show_all()

    def add_parameter_input(self, label, default_val, unit, row, param_txt):
        # parameter label
        _label = gtk.Label(label)

        # parameter text entry box
        param_txt.set_width_chars(8)
        param_txt.set_alignment(1.0)
        param_txt.set_text(default_val)

        # parameter unit
        _unit = gtk.Label(unit)

        # justify the parameter labels to the right
        _label_alignment = gtk.Alignment(1.0, 0.0, 0.0, 1.0)
        _label_alignment.add(_label)

        # add parameter to parameters table
        self.parameter_table.attach(_label_alignment, 0, 1, row - 1, row)
        self.parameter_table.attach(param_txt, 1, 2, row - 1, row)
        self.parameter_table.attach(_unit, 2, 3, row - 1, row)

    def get_robot_parameters(self):
        self.robot_parameter = []
        self.robot_parameter.append(float(self.r_wheel_radius.get_text()))
        self.robot_parameter.append(float(self.r_wheel_base_length.get_text()))
        self.robot_parameter.append(
            float(self.r_wheel_ticks_per_rev.get_text()))
        self.robot_parameter.append(
            float(self.r_max_wheel_drive_rate.get_text()))

        return self.robot_parameter

    def set_robot_parameters(self, param_list):
        self.r_wheel_radius.set_text(str(param_list[0]))
        self.r_wheel_base_length.set_text(str(param_list[1]))
        self.r_wheel_ticks_per_rev.set_text(str(param_list[2]))
        self.r_max_wheel_drive_rate.set_text(str(param_list[3]))

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
                print config_list
                self.set_robot_parameters(config_list)
            file_chooser.destroy()

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