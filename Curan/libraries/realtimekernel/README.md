# Real Time controller

Given that realtime systems are usually expensive, and dificult to set up we developed a group of applications that together provide realtime capabilities, or as close as possible without messing with the kernel of the operating system.
Our system is inspired in the Fast Research Interface provided by KUKA. Instead of writing our software with real time in mind, i.e. no heap allocation can ever be requested whilst running the program, which invalidates most of the cpp standard library, we instead write three processes:

1. Process 1 - serves as a watchdog with a state machine which guarantees that the entire control loop is forced to run at a given frequency. 
2. Process 2 - receives the information from the sensors and writes the control action to the watchdog
3. Process 3 - reads sensors from peripherals in realtime and communicate with watchdog


These three processes communicate between eachother through two mechanism, through sockets and through shared memory. 

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

