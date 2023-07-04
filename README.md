# Solution

Compiler version:
                g++ (Rev7, Built by MSYS2 project) 13.1.0
                Python 3.11.4

C++ file:
                control_system.cpp
                In this file, if you change the value of the #define TEST macro to 1, you can enable standalone testing in a main function. By default, the TEST macro is set to 0. By running the executable file standalone_test.exe, you can observe the output in the TEST mode. The control_system.dll file, compiled with the real mode, is located in the same directory.

python file:
                control_system.py
                I have implemented three buttons in the interface:
                Start: By default, we are in the start mode, and the graph displays movement. This button sends a command to the C++ code to initiate the process. 
                Pause: This button allows pausing the graph and sends a command to the C++ code to pause the ongoing tasks.
                Stop: Clicking this button terminates the program and sends a command to the C++ code to terminate all running tasks.
                when the plot is close Ctrl+C can terminate the program in both sides

log files:                
                Running the program in the real mode(NOT TEST MODE) makes two log files. 1- temperature log.txt and 2- pressure log.txt. You can see some errors and machine parameters in the log files

flow of program:
                In the C++ side, the control circle can be initiated from the base class DataSampler, which has two derivatives. This class consists of three parameters: signal, noise, and control signal. The task of this class is to generate a noise signal every 100 ms at regular intervals.

                The PID controller task is synchronized with the data sampler using an event mechanism. It calculates the control signal and places it in a queue for the adjustment task.

                The adjustment task receives the control signal and sends it to two override functions for temperature and pressure, which calculate the adjustment signal and log the results in a file. 

                The ControlSystem serves as the main interface with Python. It creates three tasks for each parameter, resulting in a total of six tasks, and initializes them using their respective constructors. 


                In the Python side, the ControlSystem class acts as the simulation machine with a global object named 'cs'. It runs the machine and retrieves its parameters.

                
                The "get_sensor_data" function retrieves the machine parameters and pushes them into a queue. 

                The "update_plot" function displays the graphs and prints the machine parameters on the screen.
                
                Each button is associated with its respective event handler and triggers the necessary actions when the button is clicked.

