import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class AnimationPlot:
    def animate(self, i, dataList, ser):
        try:
            arduinoData_string = ser.read(ser.in_waiting).decode('ascii').splitlines()[1] # Decode receive Arduino data as a formatted string
            print(arduinoData_string)                                           
            try:
                arduinoData_float = float(arduinoData_string)   # Convert to float
                dataList.append(arduinoData_float)              # Add to the list holding the fixed number of points to animate
            except:                                             # Pass if data point is bad                               
                pass
        except:                                                       
            pass
        dataList = dataList[-50:]                           # Fix the list size so that the animation plot 'window' is x number of points
        ax.clear()                                          # Clear last data frame
        self.getPlotFormat()
        ax.plot(dataList)                                   # Plot new data frame
    def getPlotFormat(self):
        ax.set_ylim([-180, 180])                              # Set Y axis limit of plot
        ax.set_yticks(np.arange(-180, 181, 10))               # Set y-axis ticks every 10 degrees
        ax.yaxis.grid(True, linestyle='--', alpha=0.7)        # Add y-axis grid
        ax.xaxis.grid(True, linestyle='--', alpha=0.7)        # Add x-axis grid
        ax.set_title("Accelerometer Data")                    # Set title of figure
        ax.set_ylabel("Sample Number")                        # Set title of x axis 
        ax.set_ylabel("Angle (degrees)")                      # Set title of y axis 

dataList = []                                           # Create empty list variable for later use
                                                        
fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                               # Add subplot to main fig window
realTimePlot = AnimationPlot()

 
ser = serial.Serial("COM3", 9600, write_timeout=1, timeout = None)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(2)                                           # Time delay for Arduino Serial initialization 

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, realTimePlot.animate, frames=100, fargs=(dataList, ser), interval=100) 

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()                                             # Close Serial connection when plot is closed