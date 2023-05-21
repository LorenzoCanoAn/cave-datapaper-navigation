#!/bin/python3
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import threading
import numpy as np


class PlotterNode:
    def __init__(self):
        self._updated_vector = False
        rospy.init_node(name="~plotter_node")
        self._plot_vector = rospy.get_param("~plot_vector", default=False)
        if self._plot_vector:
            rospy.Subscriber(
                name="internals/vector",
                data_class=Float32MultiArray,
                callback=self.vector_callback,
            )

    def vector_callback(self, msg: Float32MultiArray):
        self._vector_data_y = msg.data
        self._vector_data_x = np.linspace(0, 2 * np.pi, len(msg.data))
        self._updated_vector = True

    def draw_loop(self):
        self._fig = plt.figure(figsize=(5, 5))
        self._ax1 = self._fig.add_subplot(111, projection="polar")
        self._ax1.set_theta_zero_location("N")
        # Actors
        (self._vector_artist,) = self._ax1.plot([], lw=6, c="b")
        # Draw
        self._fig.canvas.draw()
        self._ax1background = self._fig.canvas.copy_from_bbox(self._ax1.bbox)
        plt.show(block=False)
        while not rospy.is_shutdown():
            if self._updated_vector:
                self._vector_artist.set_data(self._vector_data_x, self._vector_data_y)
            # Restore background
            self._fig.canvas.restore_region(self._ax1background)
            # Redraw artists
            self._ax1.draw_artist(self._vector_artist)
            # Fill axis rectangle
            self._fig.canvas.blit(self._ax1.bbox)
            self._fig.canvas.flush_events()


def main():
    node = PlotterNode()
    rospy_thread = threading.Thread(target=rospy.spin)
    rospy_thread.start()
    node.draw_loop()
    rospy_thread.join()


if __name__ == "__main__":
    main()
