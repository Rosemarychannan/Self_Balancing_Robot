import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import re

# Initialize serial with a shorter timeout and higher baudrate
ser = serial.Serial("COM3", 9600, timeout=0.1)
time.sleep(2)                                           # Time delay for Arduino Serial initialization 

class AnimationPlot:
    def __init__(self):
        self.sample_count = 0
        
    def is_valid_number(self, value):
        # Check if the string matches our required format:
        # - Optional negative sign
        # - At least one digit before decimal point
        # - Exactly two decimal places
        pattern = r'^-?\d+\.\d{2}$'
        if not re.match(pattern, value):
            return False
            
        # Convert to float and check range
        try:
            num = float(value)
            return -180 <= num <= 180
        except ValueError:
            return False
        
    def animate(self, frames, dataList_acc, dataList_gyro, dataList_theta, ser):
        # Clear any old data in the buffer
        ser.reset_input_buffer()
        
        try:
            arduinoData_string = ser.readline().decode('ascii').strip().split()
            print(arduinoData_string)
            
            if len(arduinoData_string) >= 3 and all(self.is_valid_number(val) for val in arduinoData_string[:3]):
                data_acc_float = float(arduinoData_string[0])
                data_gyro_float = float(arduinoData_string[1])
                data_theta_float = float(arduinoData_string[2])
                
                dataList_acc.append(data_acc_float)
                dataList_gyro.append(data_gyro_float)
                dataList_theta.append(data_theta_float)
                self.sample_count += 1
            else:
                print("Invalid data format received:", arduinoData_string)

        except Exception as e:                                             
            print(f"Error processing data: {e}")
            pass

        # Keep last 50 points but maintain correct sample numbers
        if len(dataList_acc) > 50:
            dataList_acc.pop(0)
            dataList_gyro.pop(0)
            dataList_theta.pop(0)

        # Calculate x-axis values for proper sample numbering
        x_values = list(range(max(0, self.sample_count - len(dataList_acc)), self.sample_count))

        ax.clear()
        self.getPlotFormat()
        
        # Plot with proper x values
        ax.plot(x_values, dataList_acc, color='crimson', label='Accelerometer', linewidth=2.5, alpha=1.0)
        ax.plot(x_values, dataList_gyro, color='royalblue', label='Gyroscope', linewidth=2, alpha=0.8)
        ax.plot(x_values, dataList_theta, color='forestgreen', label='Filtered', linewidth=2, alpha=0.8)
        
        # Add legend with opaque background
        ax.legend(loc='upper right', framealpha=1.0)
        
        # Show current values
        if len(dataList_acc) > 0:
            ax.text(0.02, 0.98, 
                   f'Current Values:\nAcc: {dataList_acc[-1]:.1f}°\nGyro: {dataList_gyro[-1]:.1f}°\nFiltered: {dataList_theta[-1]:.1f}°', 
                   transform=ax.transAxes, 
                   bbox=dict(facecolor='white', alpha=1.0, edgecolor='none'),
                   verticalalignment='top',
                   color='black')

    def getPlotFormat(self):
        ax.set_ylim([-180, 180])                            # Set Y axis limit of plot
        ax.set_yticks(np.arange(-180, 181, 20))            # Set y-axis ticks every 20 degrees
        ax.yaxis.grid(True, linestyle='--', alpha=0.3)      # Add y-axis grid
        ax.xaxis.grid(True, linestyle='--', alpha=0.3)      # Add x-axis grid
        ax.set_title("Multi-Sensor Angle Measurement")       # Set title of figure
        ax.set_ylabel("Angle (degrees)")                     # Set title of y axis
        ax.set_xlabel("Sample Number")                       # Set title of x axis
        ax.set_facecolor('white')                           # White background
        
dataList_acc = []                                           # Create empty list variable for later use
dataList_gyro = [] 
dataList_theta = [] 
                                                        
fig = plt.figure(figsize=(12, 6))                          # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                                   # Add subplot to main fig window

realTimePlot = AnimationPlot()
                                                        # Matplotlib Animation Function that takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, realTimePlot.animate, frames=None, 
                            fargs=(dataList_acc, dataList_gyro, dataList_theta, ser), 
                            interval=50)  # Faster updates

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()   