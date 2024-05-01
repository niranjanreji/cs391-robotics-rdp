import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self):
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', label='Altitude above surface')
        self.ax.set_autoscaley_on(True)
        self.ax.set_xlabel('Time (50ms)')
        self.ax.set_ylabel('Altitude Above Surface (cm)')
        self.ax.set_title('Altitude Map of the Surface')
        self.ax.grid(True)
        plt.legend()

    def update_plot(self, data_buffer):
        x = np.arange(len(data_buffer))
        y = np.array(data_buffer)
        self.line.set_data(x, y)
        self.ax.relim()  # Recalculate limits
        self.ax.autoscale_view(True, True, True)  # Autoscale view based on the limits
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

class MySub(Node):
    def __init__(self):
        super().__init__('dronecomm_plotter')
        self.subscription = self.create_subscription(Int32MultiArray, 'lidar_data', self.plotter, 10)
        self.plot = Plotter()

    def plotter(self, msg):
        data_buffer = msg.data
        self.plot.update_plot(data_buffer)

def main():
    try:
        rclpy.init()
        sub = MySub()
        rclpy.spin(sub)
    except KeyboardInterrupt:
        sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
