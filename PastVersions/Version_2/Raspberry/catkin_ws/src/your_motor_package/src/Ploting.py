#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float64
import math
import matplotlib.pyplot as plt

class RobotPlotter:
    def __init__(self):
        rospy.init_node('robot_plotter', anonymous=True)
        self.distance_D = None
        self.angle_z = None

        # Subscribe to /distance_D and /angle_z topics
        rospy.Subscriber('/distance_D', Float64, self.distance_callback)
        rospy.Subscriber('/angle_z', Float64, self.angle_callback)

        # Initialize plot
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'o-', label='Robot Path')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()

    def distance_callback(self, msg):
        self.distance_D = msg.data

    def angle_callback(self, msg):
        self.angle_z = msg.data
        self.plot_robot_path()

    def plot_robot_path(self):
        if self.distance_D is not None and self.angle_z is not None:
            x = self.distance_D * math.cos(self.angle_z)
            y = self.distance_D * math.sin(self.angle_z)

            # Update plot
            self.line.set_xdata([x])
            self.line.set_ydata([y])
            self.ax.relim()
            self.ax.autoscale_view()

            # Redraw the plot
            plt.draw()
            plt.pause(0.01)

if __name__ == '__main__':
    try:
        robot_plotter = RobotPlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
