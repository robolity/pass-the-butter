# -*- coding: utf-8 -*-
from math import pi
from time import time

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
    def __init__(self, world, period, zoom, map_type):
        # bind the world and it's parameters
        self.world = world
        self.period = period
        self.zoom = zoom
        self.map_type = map_type

        # initialize frame
        self.current_frame = Frame()

        # initialize camera parameters
        self.pixels_per_meter = self.zoom
        self.view_width_pixels = int(self.world.width * self.pixels_per_meter)
        self.view_height_pixels = int(self.world.height * self.pixels_per_meter)

        # initialize the window
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.set_title('Zumo Simulator')
        self.window.set_resizable(False)
        self.window.connect('delete_event', self.on_delete)

        # initialize the drawing_area
        self.drawing_area = gtk.DrawingArea()
        self.drawing_area.set_size_request(self.view_width_pixels,
            self.view_height_pixels)
        self.drawing_area.connect('expose_event', self.on_expose)

        # initialize the painter
        self.painter = Painter(self.drawing_area, self.pixels_per_meter)

        # == initialize the buttons

        # build the play button
        self.button_play = gtk.Button('Play')
        play_image = gtk.Image()
        play_image.set_from_stock(gtk.STOCK_MEDIA_PLAY, gtk.ICON_SIZE_BUTTON)
        self.button_play.set_image(play_image)
        self.button_play.set_image_position(gtk.POS_LEFT)
        self.button_play.connect('clicked', self.on_play)

        # build the stop button
        self.button_stop = gtk.Button('Stop')
        stop_image = gtk.Image()
        stop_image.set_from_stock(gtk.STOCK_MEDIA_STOP, gtk.ICON_SIZE_BUTTON)
        self.button_stop.set_image(stop_image)
        self.button_stop.set_image_position(gtk.POS_LEFT)
        self.button_stop.connect('clicked', self.on_stop)

        # build the step button
        self.button_step = gtk.Button('Step')
        step_image = gtk.Image()
        step_image.set_from_stock(gtk.STOCK_MEDIA_NEXT, gtk.ICON_SIZE_BUTTON)
        self.button_step.set_image(step_image)
        self.button_step.set_image_position(gtk.POS_LEFT)
        self.button_step.connect('clicked', self.on_step)

        # build the reset button
        self.button_reset = gtk.Button('Reset')
        reset_image = gtk.Image()
        reset_image.set_from_stock(gtk.STOCK_MEDIA_REWIND, gtk.ICON_SIZE_BUTTON)
        self.button_reset.set_image(reset_image)
        self.button_reset.set_image_position(gtk.POS_LEFT)
        self.button_reset.connect('clicked', self.on_reset)

        # build the save map button
        self.button_save_map = gtk.Button('Save Map')
        save_map_image = gtk.Image()
        save_map_image.set_from_stock(gtk.STOCK_SAVE, gtk.ICON_SIZE_BUTTON)
        self.button_save_map.set_image(save_map_image)
        self.button_save_map.set_image_position(gtk.POS_LEFT)
        self.button_save_map.connect('clicked', self.on_save_map)

        # build the load map button
        self.button_load_map = gtk.Button('Load Map')
        load_map_image = gtk.Image()
        load_map_image.set_from_stock(gtk.STOCK_OPEN, gtk.ICON_SIZE_BUTTON)
        self.button_load_map.set_image(load_map_image)
        self.button_load_map.set_image_position(gtk.POS_LEFT)
        self.button_load_map.connect('clicked', self.on_load_map)

        # build the random map buttons
        self.button_random_map = gtk.Button('Random Map')
        random_map_image = gtk.Image()
        random_map_image.set_from_stock(gtk.STOCK_REFRESH, gtk.ICON_SIZE_BUTTON)
        self.button_random_map.set_image(random_map_image)
        self.button_random_map.set_image_position(gtk.POS_LEFT)
        self.button_random_map.connect('clicked', self.on_random_map)

        # build the draw-invisibles toggle button
        self.draw_invisibles = False  # controls whether invisible world
                                      #    elements are displayed
        self.button_draw_invisibles = gtk.Button()
        self._decorate_draw_invisibles_button_inactive()
        self.button_draw_invisibles.set_image_position(gtk.POS_LEFT)
        self.button_draw_invisibles.connect('clicked', self.on_draw_invisibles)

        # == lay out the window

        # pack the simulation control buttons
        sim_controls_box = gtk.HBox(spacing=5)
        sim_controls_box.pack_start(self.button_play, False, False)
        sim_controls_box.pack_start(self.button_stop, False, False)
        sim_controls_box.pack_start(self.button_step, False, False)
        sim_controls_box.pack_start(self.button_reset, False, False)

        # pack the map control buttons
        map_controls_box = gtk.HBox(spacing=5)
        map_controls_box.pack_start(self.button_save_map, False, False)
        map_controls_box.pack_start(self.button_load_map, False, False)
        map_controls_box.pack_start(self.button_random_map, False, False)

        # pack the invisibles button
        invisibles_button_box = gtk.HBox()
        invisibles_button_box.pack_start(self.button_draw_invisibles, False,
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
        layout_box = gtk.VBox()
        layout_box.pack_start(self.drawing_area)
        layout_box.pack_start(self.alert_box, False, False, 5)
        layout_box.pack_start(sim_controls_alignment, False, False, 5)
        layout_box.pack_start(map_controls_alignment, False, False, 5)
        layout_box.pack_start(invisibles_button_alignment, False, False, 5)

        # apply the layout
        self.window.add(layout_box)

        # show the simulator window
        self.window.show_all()

    def initialise_viewer(self):

        # timing control
        # self.dt = self.world.dt  # seconds

        # gtk simulation event source - for simulation control
        self.sim_event_source = gobject.idle_add(self.world.initialise_world,
            self.map_type)  # we use this opportunity to initialize the world

        # set intial state of buttons
        self.control_panel_state_init()

        # start gtk
        gtk.main()

    def new_frame(self):
        self.current_frame = Frame()

    def draw_frame(self):
        self.drawing_area.queue_draw_area(0, 0, self.view_width_pixels,
            self.view_height_pixels)

    def control_panel_state_init(self):
        self.alert_box.set_text('')
        self.button_play.set_sensitive(True)
        self.button_stop.set_sensitive(False)
        self.button_step.set_sensitive(True)
        self.button_reset.set_sensitive(False)

    def control_panel_state_playing(self):
        self.button_play.set_sensitive(False)
        self.button_stop.set_sensitive(True)
        self.button_reset.set_sensitive(True)

    def control_panel_state_paused(self):
        self.button_play.set_sensitive(True)
        self.button_stop.set_sensitive(False)
        self.button_reset.set_sensitive(True)

    def control_panel_state_finished(self, alert_text):
        self.alert_box.set_text(alert_text)
        self.button_play.set_sensitive(False)
        self.button_stop.set_sensitive(False)
        self.button_step.set_sensitive(False)

    def _step_sim(self):
        # increment the simulation
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
        # loop this function to continually step through the simulation
        self.sim_event_source = gobject.timeout_add(int(self.period * 1000),
            self._run_sim)
        self._step_sim()

    def end_sim(self, alert_text=''):
        self.world.stop_robots()
        gobject.source_remove(self.sim_event_source)
        self.control_panel_state_finished(alert_text)

  # EVENT HANDLERS:
    def on_play(self, widget):
        # this ensures multiple calls to play_sim do not speed up the simulator
        gobject.source_remove(self.sim_event_source)
        self._run_sim()
        self.control_panel_state_playing()

    def on_stop(self, widget):
        self.world.stop_robots()
        gobject.source_remove(self.sim_event_source)
        self.control_panel_state_paused()

    def on_step(self, widget):
        gobject.source_remove(self.sim_event_source)
        self.control_panel_state_paused()
        self._step_sim()

    def on_reset(self, widget):
        gobject.source_remove(self.sim_event_source)
        self.control_panel_state_init()
        # reset the world in the viewer
        self.world.initialise_world()

    def on_save_map(self, widget):
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Save Map',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_SAVE,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                        gtk.STOCK_SAVE, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_do_overwrite_confirmation(True)
        file_chooser.set_current_folder('maps')

        # run the file chooser dialog
        response_id = file_chooser.run()

        # handle the user's response
        if response_id == LS_DIALOG_RESPONSE_CANCEL:
            file_chooser.destroy()
        elif response_id == LS_DIALOG_RESPONSE_ACCEPT:
            self.world.map_manager.save_map(file_chooser.get_filename())
            file_chooser.destroy()

    def on_load_map(self, widget):
        # create the file chooser
        file_chooser = gtk.FileChooserDialog(
            title='Load Map',
            parent=self.window,
            action=gtk.FILE_CHOOSER_ACTION_OPEN,
            buttons=(gtk.STOCK_CANCEL, LS_DIALOG_RESPONSE_CANCEL,
                            gtk.STOCK_OPEN, LS_DIALOG_RESPONSE_ACCEPT))
        file_chooser.set_current_folder('maps')

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
        gobject.source_remove(self.sim_event_source)
        self.world.initialise_world(True)

    def on_draw_invisibles(self, widget):
        # toggle the draw_invisibles state
        self.draw_invisibles = not self.draw_invisibles
        if self.draw_invisibles:
            self._decorate_draw_invisibles_button_active()
        else:
            self._decorate_draw_invisibles_button_inactive()
        self.world.draw_world()

    def on_expose(self, widget, event):
        self.painter.draw_frame(self.current_frame)

    def on_delete(self, widget, event):
        gtk.main_quit()
        return False

    def _decorate_draw_invisibles_button_active(self):
        draw_invisibles_image = gtk.Image()
        draw_invisibles_image.set_from_stock(gtk.STOCK_REMOVE,
            gtk.ICON_SIZE_BUTTON)
        self.button_draw_invisibles.set_image(draw_invisibles_image)
        self.button_draw_invisibles.set_label('Hide Invisibles')

    def _decorate_draw_invisibles_button_inactive(self):
        draw_invisibles_image = gtk.Image()
        draw_invisibles_image.set_from_stock(gtk.STOCK_ADD,
            gtk.ICON_SIZE_BUTTON)
        self.button_draw_invisibles.set_image(draw_invisibles_image)
        self.button_draw_invisibles.set_label('Show Invisibles')


#***********************************************************
class Frame(object):

    def __init__(self):
        self.draw_list = []

    def add_circle(self, pos, radius, color, alpha=None):
        self.draw_list.append({
          'type': 'circle',
          'pos': pos,
          'radius': radius,
          'color': color,
          'alpha': alpha
        })

    def add_polygons(self, polygons, color, alpha=None):
        self.draw_list.append({
          'type': 'polygons',
          'polygons': polygons,
          'color': color,
          'alpha': alpha
        })

    def add_lines(self, lines, linewidth, color, alpha=None):
        self.draw_list.append({
          'type': 'lines',
          'lines': lines,
          'linewidth': linewidth,
          'color': color,
          'alpha': alpha
        })


#************************************************************
class Painter(object):

    def __init__(self, drawing_area, pixels_per_meter):
        self.drawing_area = drawing_area
        self.pixels_per_meter = pixels_per_meter

    def draw_frame(self, frame):
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
        self.set_color(context, color, alpha)
        context.arc(pos[0], pos[1], radius, 0, 2.0 * pi)
        context.fill()

    def draw_polygons(self, context,
                     polygons,
                     color, alpha):
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
        self.set_color(context, color, alpha)
        context.set_line_width(linewidth)
        for line in lines:
            context.new_path()
            context.move_to(*line[0])
            for point in line[1:]:
                context.line_to(*point)
            context.stroke()

    def set_color(self, cairo_context, color_string, alpha):
        ColorPalette.dab(cairo_context, color_string, alpha)


#*************************************************************
class ColorPalette(object):

    @classmethod
    def dab(cls, cairo_context, color_string, alpha):
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