import sys
from cffi import FFI
import matplotlib
import matplotlib.animation
import matplotlib.pyplot as plt
import threading
import queue
import numpy as np
import time
import matplotlib.animation as animation
import tkinter as tk
import matplotlib.widgets as widgets

ffi = FFI()

# Load the shared library
lib = ffi.dlopen("./control_system.dll")

# Define the C functions that we want to use
ffi.cdef(
    """
    void* control_system_new();
    void controlSystemRun(void*, int);
    double controlSystemGetTemperature(void*);
    double controlSystemGetPressure(void*);
    double controlSystemGetTemperatureFluctuation(void*);
    double controlSystemGetPressureFluctuation(void*);
    double controlSystemGetTemperatureControlSignal(void*);
    double controlSystemGetPressureControlSignal(void*);
    void controlSystemStart(void*);
    void controlSystemStop(void*);
    void controlSystemPause(void*);
"""
)

# Define the ControlSystem class and its methods
class ControlSystem:
    def __init__(self):
        self.obj = lib.control_system_new()
        self.pressure_time = 0
        self.temperature_time = 0

    # run the C++ function
    def run(self, num_iterations):
        lib.controlSystemRun(self.obj, num_iterations)

    # retreive the temprature 
    def getTemperature(self):
        return lib.controlSystemGetTemperature(self.obj)

    # retreive the pressure
    def getPressure(self):
        return lib.controlSystemGetPressure(self.obj)
    
    # retreive the temperature fluctuation
    def getTemperatureFluctuation(self):
        return lib.controlSystemGetTemperatureFluctuation(self.obj)

    # retreive the pressure fluctuation
    def getPressureFluctuation(self):
        return lib.controlSystemGetPressureFluctuation(self.obj)

    # retreive the temperature control signal
    def getTemperatureControlSignal(self):
        return lib.controlSystemGetTemperatureControlSignal(self.obj)

    # retreive the pressure control signal
    def getPressureControlSignal(self):
        return lib.controlSystemGetPressureControlSignal(self.obj)

    # start sampling and plotting
    def start(self):
        lib.controlSystemStart(self.obj)

    # terminate the application in both side
    def stop(self):
        lib.controlSystemStop(self.obj)

    # pause sampling and plotting
    def pause(self):
        lib.controlSystemPause(self.obj)

    # print pressure parameters on the screen
    def printPressureParameters(self, pressure, fluctuation, control_signal):
        message = f"{self.pressure_time:04d} - Pressure = {pressure:.5f}\t\tfluctuation = {fluctuation:.5f}\t\tControl Signal = {control_signal:.5f}"
        print(message)
        self.pressure_time += 1

    # print temperature parameters on the screen
    def printTemperatureParameters(self, temperature, fluctuation, control_signal):
        message = f"{self.temperature_time:04d} - Temperatuer = {temperature:.5f}\t\tfluctuation = {fluctuation:.5f}\t\tControl Signal = {control_signal:.5f}"
        print(message)
        self.temperature_time += 1
    
# global variables
cs = ControlSystem ()           # Create a ControlSystem object
stop_event = threading.Event()  # Create an event to signal the threads to stop
is_paused = True                # check flack to find if the system is paused

# Define a function to pause the control system
def pause_control_system():
    global is_paused
    is_paused = False
    cs.pause()                  # pause sampling in C++ side
    anim.event_source.stop()    # pause plotting

def stop_control_system():
    cs.stop()                   # terminate the run in C++ side
    stop_event.set()            # Set the stop event to signal the data thread to stop
    plt.close()                 # Close the matplotlib figure    
    sys.exit(0)                 # terminate the program

def start_control_system():
    global is_paused    
    is_paused = True    
    cs.start()                  # start sampling
    anim.event_source.start()   # start plotting
    
# Run the control system in a separate thread
num_iterations = 100
cs_thread = threading.Thread(target=cs.run, args=(num_iterations,))
cs_thread.start()


# Define a function to get sensor readings from the control system
def get_sensor_data(cs, stop_event):
    global is_paused
    while not stop_event.is_set(): 
        # don't do it when the system in paused   
        if is_paused: 
            # retrieve the machine parameters from C++ side           
            temperature = cs.getTemperature()
            pressure = cs.getPressure()            
            pressure_fluctuation = cs.getPressureFluctuation()
            temperature_fluctuation = cs.getTemperatureFluctuation()
            pressure_control_signal = cs.getPressureControlSignal()
            temperature_control_signal = cs.getTemperatureControlSignal()
            # push the parameters in a queue
            sensor_data.put((temperature, pressure, pressure_fluctuation,
                             temperature_fluctuation,pressure_control_signal,
                             temperature_control_signal))
        # each 100 ms
        time.sleep(0.1)
    

# Create a queue to hold sensor readings
sensor_data = queue.Queue()

# Get sensor readings in a separate thread
data_thread = threading.Thread(target=get_sensor_data, args=(cs, stop_event))
data_thread.start()


# Create a figure and axis for the plot
fig, ax = plt.subplots()

# Initialize the data and line objects
x_data = []
temp_data = []
pressure_data = []
(temp_line,) = ax.plot(x_data, temp_data, label="Temperature")
(pressure_line,) = ax.plot(x_data, pressure_data, label="Pressure")

# Add horizontal lines for the setpoints
temp_setpoint_line = ax.axhline(
    y=20, color="r", linestyle="--", label="Temperature Setpoint"
)
pressure_setpoint_line = ax.axhline(
    y=1, color="g", linestyle="--", label="Pressure Setpoint"
)

# Create the pause button
pause_button_ax = fig.add_axes([0.69, 0.9, 0.1, 0.05])  
pause_button = widgets.Button(pause_button_ax, 'Pause')
pause_button.on_clicked(lambda _: pause_control_system())

# Create the stop button
stop_button_ax = fig.add_axes([0.80, 0.9, 0.1, 0.05])  
stop_button = widgets.Button(stop_button_ax, 'Stop')
stop_button.on_clicked(lambda _: stop_control_system())

# Create the start button
start_button_ax = fig.add_axes([0.58, 0.9, 0.1, 0.05])  
start_button = widgets.Button(start_button_ax, 'Start')
start_button.on_clicked(lambda _: start_control_system())

# each frame is one sample added
def update_plot(num, x_data, temp_data, pressure_data, temp_line, pressure_line):
    global num_iterations
    if num == (num_iterations - 1):
       anim.event_source.stop()     # pause when reached to the end
    try:       
        # retrieve the machine parameters from queue
        (temperature, pressure, pressure_fluctuation,
        temperature_fluctuation,pressure_control_signal,
        temperature_control_signal) = sensor_data.get_nowait()

        x_data.append(num)
        temp_data.append(temperature)
        pressure_data.append(pressure) 
        temp_line.set_data(x_data, temp_data)
        pressure_line.set_data(x_data, pressure_data)
        ax.relim()
        ax.autoscale_view()
        #write log for pressure parameters
        cs.printPressureParameters(pressure, pressure_fluctuation, pressure_control_signal)
        #write log for temperature parameters
        cs.printTemperatureParameters(temperature, temperature_fluctuation, temperature_control_signal)
        return temp_line, pressure_line
    except queue.Empty:        
        return temp_line, pressure_line


anim = matplotlib.animation.FuncAnimation(
    fig,
    update_plot,
    frames=range(num_iterations + 1),
    fargs=(x_data, temp_data, pressure_data, temp_line, pressure_line),
)
ax.legend()
plt.show()
try:
    # Wait for KeyboardInterrupt (Ctrl+C) to stop the program
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    # terminate the program
    stop_control_system()

    # Wait for the threads to finish
    cs_thread.join()
    data_thread.join()

    
    # Close the matplotlib figure and terminate the event loop
    plt.close()
    sys.exit(0)