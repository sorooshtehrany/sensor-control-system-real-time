from cffi import FFI
import matplotlib
import matplotlib.animation
import matplotlib.pyplot as plt
import threading
import queue
import numpy as np
import time

ffi = FFI()

# Load the shared library
lib = ffi.dlopen("./control_system.dll")

# Define the C functions that we want to use
ffi.cdef(
    """
    void* ControlSystem_new();
    void ControlSystem_run(void*, int);
    int ControlSystem_get_temperature(void*);
    int ControlSystem_get_pressure(void*);
"""
)


# Define the ControlSystem class and its methods
class ControlSystem:
    def __init__(self):
        self.obj = lib.ControlSystem_new()

    def run(self, num_iterations):
        lib.ControlSystem_run(self.obj, num_iterations)

    def get_temperature(self):
        return lib.ControlSystem_get_temperature(self.obj)

    def get_pressure(self):
        return lib.ControlSystem_get_pressure(self.obj)


# Create a ControlSystem object
cs = ControlSystem()

# Create an event to signal the threads to stop
stop_event = threading.Event()

# Run the control system in a separate thread
num_iterations = 100
cs_thread = threading.Thread(target=cs.run, args=(num_iterations,))
cs_thread.start()


# Define a function to get sensor readings from the control system
def get_sensor_data(cs, stop_event):
    while not stop_event.is_set():
        temperature = cs.get_temperature()
        pressure = cs.get_pressure()
        sensor_data.put((temperature, pressure))
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


def update_plot(num, x_data, temp_data, pressure_data, temp_line, pressure_line):
    try:
        temperature, pressure = sensor_data.get_nowait()
        x_data.append(num)
        temp_data.append(temperature)
        pressure_data.append(pressure)
        temp_line.set_data(x_data, temp_data)
        pressure_line.set_data(x_data, pressure_data)
        ax.relim()
        ax.autoscale_view()
        return temp_line, pressure_line
    except queue.Empty:
        return temp_line, pressure_line


_ = matplotlib.animation.FuncAnimation(
    fig,
    update_plot,
    frames=range(num_iterations),
    fargs=(x_data, temp_data, pressure_data, temp_line, pressure_line),
)
ax.legend()
plt.show()

# Signal the threads to stop
stop_event.set()

# Wait for the threads to finish
cs_thread.join()
data_thread.join()
