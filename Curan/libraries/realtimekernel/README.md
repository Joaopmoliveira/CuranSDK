# Real Time controller

Given that realtime systems are usually expensive, and dificult to set up we developed a group of applications that together provide realtime capabilities, or as close as possible without messing with the kernel of the operating system.
Our system is inspired in the Fast Research Interface provided by KUKA. Instead of writing our software with real time in mind, i.e. no heap allocation can ever be requested whilst running the program, which invalidates most of the cpp standard library, we instead write three processes:

1. Process 1 - serves as a watchdog with a state machine which guarantees that the entire control loop is forced to run at a given frequency. 
2. Process 2 - receives the information from the sensors and writes the control action to the watchdog
3. Process 3 - reads sensors from peripherals in realtime and communicate with watchdog


These three processes communicate between eachother through two mechanism, through sockets and through shared memory. The shared memory has a specific layout which we use to write the readings of the sensors (we want the communication between the processes to be as fast as possible).The socket communication is used as a syncronization mechanism to control who can access the shared memory at a given moment. To simplify the life of developers whilst manuseating the shared memory (which is manipulated with directy access the to a giant blob of memory) we implemented a compiler which generates classes to read and write into this shared memory. 

# Compiler

The compiler receives a json format which describes the layout of the memory. Once this process runs two header files are generated which can be used to read and write from the shared memory. Lets look at a Json file as an example. Assume that you have two periperals, one is a GPS which provides three readings:

1. Velocity - array of three doubles
2. Acceleration - array of three doubles
3. Orientation - array of three doubles

and a camera which provides an image with a fixed size, e.g. a 200x200 image with one byte per pixel. One can specify the json describing our fields from the peripherals as 

```json
{
    "shared_memory_name" : "my_custom_name",
    "messages" : [
        {
        "message" : "gps_reading",
        "fields" : [
            {"name" : "counter", "type" : "int", "array" : 1},
            {"name" : "velocity", "type" : "double" , "array" : 3},
            {"name" : "acceleration", "type" : "double" , "array" : 3},
            {"name" : "orientation", "type" : "double" , "array" : 3},
        ]
        },
        {
        "message" : "grayscale_image_1",
        "fields" : [
            {"name" : "counter", "type" : "int", "array" : 1},
            {"name" : "data", "type" : "bytes", "array" : 40000  }
        ]  
        }
        ]
}
```

This file will be converted into two files, each called 

# Process 2

This process is a simple control loop which uses multithreading to manage multiple peripherals. Assume that you have N peripherals, then Process 1 will be composed of N+1 threads, where N threads read the sensors 

```cpp

//Peripheral 1

void read_peripheral_1(std::atomic<double>& reading_1){
    double local_reading_1 = 0.0;
    while(true){
        // custom code to request reading from peripheral 1 (write into local_reading_1)

        // write in atomic fasion into atomic variable reading_1
        reading_1.store(local_reading_1);
    }
}

// ...

//Peripheral N

void read_peripheral_N(std::atomic<double>& reading_N){
    double local_reading_N = 0.0;
    while(true){
        // custom code to request reading from peripheral N (write into local_reading_N)

        // write in atomic fasion into atomic variable reading_N
        reading_N.store(local_reading_N);
    }
}

```

The last N+1 thread controls the frequency at which we read from these variables which should look something like this

```cpp
int main(){
    std::atomic<double> reading_1;
    auto callable_1 = [&](){
        read_peripheral_1(reading_1);
    };
    std::thread thread_1{callable_1};

    // ...

    std::atomic<double> reading_N;
    auto callable_N = [&](){
        read_peripheral_N(reading_N);
    };
    std::thread thread_N{callable_N};

    for(size_t counter = 0; ; ++counter){
        auto atomic_reading_1 = reading_1;
        // ...
        auto atomic_reading_N = reading_N;
    }
}
```

# Process 2

# Process 3

