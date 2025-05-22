import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class RealTimePlot:
    def __init__(self, port="COM3", baud_rate=9600):
        self.data = []
        self.fig, self.ax = plt.subplots()
        self.serial = serial.Serial(port, baud_rate, timeout=None)
        time.sleep(3)
        
    def update(self, frame):
        try:
            data_str = self.serial.read(self.serial.in_waiting).decode('ascii').splitlines()[1]
            self.data.append(float(data_str))
        except (IndexError, ValueError):
            pass
            
        self.ax.clear()
        
        # Format plot
        self.ax.set_ylim([-180, 180])
        self.ax.set_title("Gyroscope Data")
        self.ax.set_ylabel("Angle (degrees)")
        self.ax.set_xlabel("Samples")
        
        # Set y-axis ticks every 10 degrees
        self.ax.set_yticks(np.arange(-180, 181, 10))
        self.ax.yaxis.grid(True, linestyle='--', alpha=0.7)
        self.ax.xaxis.grid(True, linestyle='--', alpha=0.7)
        self.ax.plot(self.data)
        
    def start(self):
        self.anim = animation.FuncAnimation(
            self.fig, 
            self.update,
            frames=None,
            interval=100
        )
        plt.show()
        self.serial.close()

if __name__ == "__main__":
    plotter = RealTimePlot()
    plotter.start()