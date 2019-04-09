import matplotlib.pyplot as plt
import math

import numpy

from robots.mathbot import Mathbot
from supervisor.supervisor import Supervisor

plt.ion()


class DynamicPlot:
    #Suppose we know the x range
    min_x = -1.5
    max_x = 2.0
    min_y = -1.5
    max_y = 2.0

    def __init__(self, mathbot: Mathbot, supervisor: Supervisor, color, set_lims=True):
        self.color = color
        self.mathbot = mathbot
        self.supervisor = supervisor
        self.figure, self.ax = plt.subplots(1, 1)
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        if set_lims:
            self.ax.set_xlim(self.min_x, self.max_x)
            self.ax.set_ylim(self.min_y, self.max_y)
        #Other stuff
        self.ax.grid()
        self.ax.set_axisbelow(True)
        self.ax.set_aspect('equal')
        #mng = plt.get_current_fig_manager()
        #mng.window.showMaximized()
        self.omni_patches: list = []

        #self.envelope = plt.Polygon(mathbot.get_envelope_i(), color="#7FFFD4", alpha=0.5)
        #self.ax.add_patch(self.envelope)

        self.chassis = plt.Polygon(mathbot.chassis_body.get_envelope_i(), color=color, alpha=1)
        self.ax.add_patch(self.chassis)

        self.patch1 = plt.Polygon(self.mathbot.left_wheel_body.get_envelope_i(), color="#778899", alpha=1)
        self.patch2 = plt.Polygon(self.mathbot.right_wheel_body.get_envelope_i(), color="#778899", alpha=1)

        self.ax.add_patch(self.patch1)
        self.ax.add_patch(self.patch2)

        self.sensors = []
        for sensor in mathbot.sensors:
            sensor_patch = plt.Polygon(sensor.get_envelope_plot(), color="#7FFFD4", alpha=0.5)
            self.sensors.append(sensor_patch)
            self.ax.add_patch(sensor_patch)

        self.obstacles = []
        for obstacle in self.supervisor.obstacles:
            obstacle_patch = plt.Polygon(obstacle.get_envelope_i(), color="#800080", alpha=0.1)
            self.obstacles.append(obstacle_patch)
            self.ax.add_patch(obstacle_patch)


    def add_omni_patches(self, omni_patches):
        self.omni_patches.append(omni_patches)

    def on_running(self):
        """
        self.ax.patches.clear()

        for omni_patch in self.omni_patches:
            for patch in omni_patch.get_patches():
                self.ax.add_patch(patch)
        """
        #self.envelope.xy = self.mathbot.get_envelope_i()
        self.chassis.xy = self.mathbot.chassis_body.get_envelope_i()
        self.patch1.xy = self.mathbot.left_wheel_body.get_envelope_i()
        self.patch2.xy = self.mathbot.right_wheel_body.get_envelope_i()

        sensor_length = len(self.sensors)

        for i in range(sensor_length):
            self.sensors[i].xy = self.mathbot.sensors[i].get_envelope_plot()
        #Need both of these in order to rescale
        #self.ax.relim()
        #self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
