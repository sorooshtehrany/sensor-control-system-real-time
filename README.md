# Solution

Compiler version:
                
                g++ (Rev7, Built by MSYS2 project) 13.1.0
                Python 3.11.4

C++ file:
                
                control_system.cpp
                In this file, if you change the value of the #define TEST macro to 1, you can enable standalone testing
                in a main function. By default, the TEST macro is set to 0. By running the executable file standalone_test.exe, 
                you can observe the output in the TEST mode. 

python file:
                
                control_system.py
                I have implemented three buttons in the interface: Start: By default, we are in the start mode, and the graph 
                displays movement. This button sends a command to the C++ code to initiate the process. Pause: This button allows 
                pausing the graph and sends a command to the C++ code to pause the ongoing tasks. Stop: Clicking this button 
                terminates the program and sends a command to the C++ code to terminate all running tasks. when the plot is 
                closed Ctrl+C can terminate the program on both sides

log files:                
                
                Running the program in real mode(NOT TEST MODE) makes two log files. 1- temperature log.txt and 2- pressure log.txt. 
                You can see some errors and machine parameters in the log files.

The flow of the program:
                
                On the C++ side, the control circle can be initiated from the base class DataSampler, which has two derivatives. 
                This class consists of three parameters: signal, noise, and control signal. The task of this class is to generate 
                a noise signal every 100 ms at regular intervals. The PID controller task is synchronized with the data sampler 
                using an event mechanism. It calculates the control signal and places it in a queue for the adjustment task. The 
                adjustment task receives the control signal and sends it to two override functions for temperature and pressure, 
                which calculate the adjustment signal and log the results in a file. The ControlSystem serves as the main interface 
                with Python. It creates three tasks for each parameter, resulting in a total of six tasks, and initializes them 
                using their respective constructors. On the Python side, the ControlSystem class acts as the simulation machine 
                with a global object named 'cs'. It runs the machine and retrieves its parameters. The "get_sensor_data" function 
                retrieves the machine parameters and pushes them into a queue. The "update_plot" function displays the graphs and 
                prints the machine parameters on the screen. Each button is associated with its respective event handler and 
                triggers the necessary actions when the button is clicked.

# Setup Instructions

Install the latest version of Python.

Install a C++ compiler. If you're using Windows, you can install MinGW. If you're using macOS, you can install the Xcode command 
line tools. If you're using Linux, you can install the g++ package using your package manager.

Clone this repository to your local machine.

Navigate to the directory where you cloned the repository.

Create the Shared library using "g++ -m64 -shared -o control_system.dll control_system.cpp -lpthread"

Install the required Python packages using "pip install -r requirements.txt"

Run the Python file using "python control_system.py"
