#include <random>    // to generate random number
#include <windows.h> // Include the Windows API header
#include <iostream>
#include <vector>    // to make a dynamic array of tasks
#include <queue>     // to sync data communication between tasks
#include <atomic>    // to lock a task in order to log in file
#include <fstream>   // Include the header for file operations
#include <iomanip>   // to use std::setprecision
#include <sstream>   // used to work with strings

#define TEST  0      // in TEST = 1 you can see all the logs

// using a lock mechanism to use shared resourses
#define MESSAGE(lock, message, output_stream)            \
    while (lock.test_and_set(std::memory_order_acquire)) \
    {                                                    \
    }                                                    \
    output_stream << message << std::endl;               \
    lock.clear(std::memory_order_release);

enum
{
    START = 0,      // the value used to start the system
    PAUSE = 1       // the value used to pause the system
};

// to lock a task in order to log in file
static std::atomic_flag lock = ATOMIC_FLAG_INIT;
// it makes possible the stop mechanism by terminating the tasks
unsigned int running;
// it makes possible the pause mechanism by stopping the DataSamplerTask
bool ignition = START;

// a base class to hold machine parameters
class DataSampler
{
public:
    // mean and stddev are initialized here. signal gets the start point
    DataSampler(double mean, double stddev, double start_point)
        : signal(start_point), noise_distribution(mean, stddev), event_handle(nullptr) {}

    // updates the signal after each adjustment
    void setSignal(double value)
    {
        signal = value;
    }

    double getSignal()
    {
        return signal;
    }

    double getNoise()
    {
        return noise;
    }

    void setControlSignal(double value)
    {
        control_signal = value;
    }

    double getControlSignal()
    {
        return control_signal;
    }

   // starts the data sampler task
    HANDLE startDataSamplerTask(HANDLE event_handle)
    {
        this->event_handle = event_handle;
        HANDLE thread_handle = CreateThread(NULL, 0, ThreadProc, this, 0, NULL);
        return thread_handle;
    }
private:
    double signal;                                       // could be temperature or pressure
    double noise;                                        // it fluctuates the signal
    double control_signal;                               // latest control signal
    std::default_random_engine generator;                // it helps to generate random
    std::normal_distribution<double> noise_distribution; // how wide the noise could be
    HANDLE event_handle;                                 // used to signal the pid controller

    // this task makes noise
    void dataSamplerTask()
    {
        while (running) // when running is zero the task terminates
        {
            while (ignition) // when stops here pid and adjustment don't get signal
                Sleep(1);
            noise = noise_distribution(generator);  // made noise is saved
            SetEvent(event_handle); // gives the signal to pid task
            Sleep(100);             // each 100 ms one noise sample
        }
#if TEST
        MESSAGE(lock, "data sampler task is terminated\r\n", std::cout);
#endif
        SetEvent(event_handle); // it's the last signal to pid task
    }


    // makes the data sampler task
    static DWORD WINAPI ThreadProc(LPVOID lpParam)
    {
        DataSampler *ds = static_cast<DataSampler *>(lpParam);
        ds->dataSamplerTask();
        return 0;
    }
};

// derivative class(temperature) inherited from DataSampler
class TemperatureDS : public DataSampler
{
public:
    TemperatureDS(double mean, double stddev, double start_point) : DataSampler(mean, stddev, start_point) {}
};

// derivative class inherited from DataSampler
class PressureDS : public DataSampler
{
public:
    PressureDS(double mean, double stddev, double start_point) : DataSampler(mean, stddev, start_point) {}
};

// control class
class PIDController
{
public:
    // the pid constructor
    PIDController(double kp, double ki, double kd, DataSampler *ds, std::queue<double> &queue) : 
        // the queue is used to syncronize pid and adjustment
        kp(kp), ki(ki), kd(kd), ds(ds), control_signal_queue(queue)
    { }

    // starts the pid task
    HANDLE startPIDTask(HANDLE event_handle)
    {
        this->event_handle = event_handle;
        HANDLE threadHandle = CreateThread(NULL, 0, ThreadProc, this, 0, NULL);
        return threadHandle;
    }

    // NOT USED: we need SetSetpoint() function to change the setpoint dinamycally
    void setSetpoint(double value)
    {
        setpoint = value;
    }

private:
    double kp, ki, kd;      // pid quotients
    double prev_error = 0;  // previous error
    double integral = 0;    // integral of errors
    double setpoint;        // the target point of control system
    DataSampler *ds;        // used to retreive signal data
    HANDLE event_handle;                      // gets the signal from DataSampler
    std::queue<double> &control_signal_queue; // give the result to adjustment task

    // the pid operation
    double control(double setpoint, double process_variable)
    {
        // Calculate the error as the difference between the setpoint and process variable
        double error = setpoint - process_variable;

        // Calculate the proportional term by multiplying the error with the proportional gain (kp)
        double proportional = kp * error;

        // Update the integral term by accumulating the error multiplied by the integral gain (ki)
        integral += error * ki;

        // Calculate the derivative term by multiplying the difference between the current error and previous error with the derivative gain (kd)
        double derivative = kd * (error - prev_error);

        // Calculate the control signal as the sum of the proportional, integral, and derivative terms
        double control_signal = proportional + integral + derivative;

        // Update the previous error for the next iteration
        prev_error = error;

        // Return the calculated control signal
        return control_signal;
    }

    void pidTask()
    {
        while (running) // when equal false the task terminates
        {
            // Wait for the event to be signaled from data sampler
            WaitForSingleObject(event_handle, INFINITE);

            // Perform PID control calculations here
            double control_signal = control(setpoint, ds->getSignal());

            // push the result of control into the queue
            control_signal_queue.push(control_signal);
        }
#if TEST
        MESSAGE(lock, "pid task is terminated\r\n", std::cout);
#endif
    }

    // makes the pid task
    static DWORD WINAPI ThreadProc(LPVOID lpParam)
    {
        PIDController *pidController = static_cast<PIDController *>(lpParam);
        pidController->pidTask();
        return 0;
    }
};

class Adjustment
{
public:
    // constructor initializes queue and file name
    Adjustment(std::queue<double> &queue, std::string file_name): 
        control_signal_queue(queue), file_name(file_name)
    {
        // opens a log file for temprature data or pressure data
        log_file.open(file_name);
    }

    // each sensor has its own updataSignalAndLog function
    virtual void updataSignalAndLog(double control_signal) = 0;

    // starts the adjustment task
    HANDLE startAdjustmentTask(void)
    {
        HANDLE thread_handle = CreateThread(NULL, 0, ThreadProc, this, 0, NULL);
        return thread_handle;
    }

protected:
    std::ofstream log_file; // file stream for logging data
    unsigned int time = 0;  // the time written in the log

private:
    std::queue<double> &control_signal_queue;   // gets the control signal from pid
    std::string file_name;                      //  a file to record system logs
    void adjustmentTask()
    {
        while (running) // when equal false the task terminates
        {
            // Check if there is a new control signal in the queue
            if (!control_signal_queue.empty())
            {
                // gets the last sample put by the pid task
                double control_signal = control_signal_queue.front();
                try
                {
                    // Limit the control signal to a reasonable range (e.g., -10 to 10)
                    if (control_signal < -10)
                    {
                        control_signal = -10;
                        throw std::range_error("Control signal out of range");
                    }
                    if(control_signal > 10)
                    {
                        control_signal = 10;
                        throw std::range_error("Control signal out of range");
                    }
                }
                catch (const std::range_error& e)
                {
                    // Handle the exception
                    MESSAGE(lock,   std::endl << std::setw(79)<< "  Error: " << e.what()  ,
                            #if TEST
                            std::cerr
                            #else
                            log_file
                            #endif
                            );
                }
                // discard the last sample
                control_signal_queue.pop();
                if (running)    // control the sample numbers that need to be processed
                {
                    // it adjust the output, updates value and makes log
                    updataSignalAndLog(control_signal); 
                    --running;                                       
                }
           }
        }

#if TEST
        MESSAGE(lock, "adjustmentTask is terminated\r\n", std::cout);
#endif // TEST
    }

    // makes the adjustment task
    static DWORD WINAPI ThreadProc(LPVOID lpParam)
    {
        Adjustment *adjustment = static_cast<Adjustment *>(lpParam);
        adjustment->adjustmentTask();
        return 0;
    }
};

// Derived class representing adjustment for temperature
class TemperatureAdjustment : public Adjustment
{
public:
    // the constructor
    TemperatureAdjustment(std::queue<double> &queue, std::string file_name, TemperatureDS *temp_ds)
    : Adjustment(queue,file_name), temp_ds(temp_ds)
    {}
private:
    TemperatureDS *temp_ds;  // used to retreive data

    // the override function for temprature
    void updataSignalAndLog(double control_signal) override
    {
        // don't execute for more than num_iterations
        if (running == 0)    return;
        // Update temperature based on previous temperature, random fluctuations, and control input
        double temperature = temp_ds->getSignal();
        temperature += -0.1 * (temperature - 20) + temp_ds->getNoise() + control_signal;
        // updates the temprature data
        temp_ds->setSignal(temperature);
        // update the control signal
        temp_ds->setControlSignal(control_signal);
        // log in the file
        MESSAGE(lock,
                std::fixed << std::setprecision(5)
                << std::setw(4) << time++ << " - "
                << "Temperature = " << std::setw(7) << temperature
                << "\t\tfluctuation = " << std::setw(7) << temp_ds->getNoise()
                << "\t\tControl Signal = " << std::setw(7) << control_signal,
                #if TEST
                std::cout
                #else
                log_file
                #endif // TEST
                );
    }
};

// Derived class representing adjustment for pressure
class PressureAdjustment : public Adjustment
{
public:
    // the constructor
    PressureAdjustment(std::queue<double> &queue,std::string file_name, PressureDS *press_ds, TemperatureDS *temp_ds)
        :Adjustment(queue, file_name), press_ds(press_ds), temp_ds(temp_ds)
    {}

private:
    PressureDS *press_ds;        // used to retreive data
    TemperatureDS *temp_ds;      // used to retreive data
    // the override function for pressure
    void updataSignalAndLog(double control_signal) override
    {
        // don't execute for more than num_iterations
        if (running == 0)    return;
        // Perform pressure-specific calculations to determine the sigma value
        double pressure = press_ds->getSignal();
        double temperature = temp_ds->getSignal();
        // Update pressure based on temperature (assuming ideal gas), random fluctuations, and control input
        pressure += 0.05 * (temperature - pressure) + press_ds->getNoise() + control_signal;
        press_ds->setSignal(pressure);
        // update the control signal
        press_ds->setControlSignal(control_signal);
        // log in the file
        MESSAGE(lock,
                std::fixed << std::setprecision(5)
                << std::setw(4) << time++ << " - "
                << "Pressure = " << std::setw(7) << pressure
                << "\t\tfluctuation = " << std::setw(7) << press_ds->getNoise()
                << "\t\tControl Signal = " << std::setw(7) << control_signal,
                #if TEST
                std::cout
                #else
                log_file
                #endif // TEST
                );
    }
};

// the integrator class
class ControlSystem
{
public:
    // Constructor initializes the noise distribution and start points
    ControlSystem() : temp_ds(2.0, 2.0, 20.0), press_ds(0.5, 0.5, 1.0) {}
    void run(unsigned int num_iterations)
    {
        std::vector<HANDLE> thread_handles;
        // used to signal from data sampler to pid
        HANDLE temp_event = CreateEvent(NULL, FALSE, FALSE, NULL);
        HANDLE press_event = CreateEvent(NULL, FALSE, FALSE, NULL);

        // to synchronize PID controller with adjustment
        std::queue<double> control_temp_queue;
        std::queue<double> control_press_queue;

        // in each data sampling one unit decreases
        running = num_iterations * 2;

        // p = 1     i = 0.1     d = 0.05
        PIDController temp_controller(1.0, 0.1, 0.05, &temp_ds, control_temp_queue);
        // p = 1     i = 0.1     d = 0.05
        PIDController press_controller(1.0, 0.1, 0.05, &press_ds, control_press_queue);

        // Setpoint temperature in degrees Celsius
        temp_controller.setSetpoint(50.0);      
        // Setpoint pressure in atmospheres
        press_controller.setSetpoint(2.5);      

        // open log files in adjustment objects to record machine parameters
        TemperatureAdjustment temp_adj(control_temp_queue,"1- temperature log.txt", &temp_ds);
        PressureAdjustment press_adj(control_press_queue, "2- pressure log.txt", &press_ds, &temp_ds);

        // add the threads to the system
        thread_handles.push_back(temp_ds.startDataSamplerTask(temp_event));
        thread_handles.push_back(press_ds.startDataSamplerTask(press_event));

        thread_handles.push_back(temp_controller.startPIDTask(temp_event));
        thread_handles.push_back(press_controller.startPIDTask(press_event));

        thread_handles.push_back(temp_adj.startAdjustmentTask());
        thread_handles.push_back(press_adj.startAdjustmentTask());

        // Wait for all threads to finish
        WaitForMultipleObjects(static_cast<DWORD>(thread_handles.size()), thread_handles.data(), TRUE, INFINITE);

#if TEST
        MESSAGE(lock, "Here is the end\n", std::cout);
#endif // TEST

        // Close the thread handles
        for (HANDLE handle : thread_handles)
        {
            CloseHandle(handle);
        }

        // Close the event handles
        CloseHandle(temp_event);
        CloseHandle(press_event);
    }

    double getTemperature()
    {
        return temp_ds.getSignal();
    }

    double getPressure()
    {
        return press_ds.getSignal();
    }

    double getTemperatureFluctuation()
    {
        return temp_ds.getNoise();
    }

    double getPressureFluctuation()
    {
        return press_ds.getNoise();
    }

    double getTemperatureControlSignal()
    {
        return temp_ds.getControlSignal();
    }

    double getPressureControlSignal()
    {
        return press_ds.getControlSignal();
    }

    void start(void)
    {
        ignition = START;
    }

    void pause(void)
    {
        ignition = PAUSE;
    }

    void stop(void)
    {
        ignition = false;   // make sure the dataSamplerTask can terminate
        running = 0;
    }
private:
    TemperatureDS temp_ds;
    PressureDS press_ds;
};

extern "C"
{
    ControlSystem *control_system_new() { return new ControlSystem(); }
    void controlSystemRun(ControlSystem *cs, int num_iterations) { cs->run(num_iterations); }
    double controlSystemGetTemperature(ControlSystem *cs) { return cs->getTemperature(); }
    double controlSystemGetPressure(ControlSystem *cs) { return cs->getPressure(); }
    double controlSystemGetTemperatureControlSignal(ControlSystem *cs)
            { return cs->getTemperatureControlSignal(); }
    double controlSystemGetPressureControlSignal(ControlSystem *cs)
            { return cs->getPressureControlSignal(); }
    double controlSystemGetTemperatureFluctuation(ControlSystem *cs)
            { return cs->getTemperatureFluctuation(); }
    double controlSystemGetPressureFluctuation(ControlSystem *cs)
            { return cs->getPressureFluctuation(); }
    void controlSystemStart(ControlSystem *cs){cs->start();}
    void controlSystemStop(ControlSystem *cs){cs->stop();}
    void controlSystemPause(ControlSystem *cs){cs->pause();}
}

#if TEST
int main()
{
    ControlSystem test;
    test.run(100);
    while(1){}
    return 0;
}
#endif
