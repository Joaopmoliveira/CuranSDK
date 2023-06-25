---
layout: "page"
title: "Tutorials" 
---

Once you have understood the layout of the curan project we are ready to dwelve deep into the curan API and how you can achieve your goals.

## All the tutorials currently available

* Utilities :
* User Interface :
* Image Processing : 
* 3D Rendering : 
* Optimization : 
* Communication :

## Utilities

The utilities library is contained in the Curan API is located in the library folders in the utils folder. In CMAKE the target of the library is 'utils' and tu use it you can define a CMakeLists.txt with the following content 

```cmake
add_exectutable(myexample main.cpp)

target_link_libraries(myexample PUBLIC
utils
)
```

This code signals to CMake that our target depends on utils and when we compile it we must have the relative paths to both the include directories and the library files of the utils target. Now we will introduce a bit of the library for you to get a better graps of when and where to use it.

# SafeQueue

Assume that you have two functions, one which reads input from the keyboard and commands how large the gains of your controller (lets call this function foo) and another that establishes a serial connection with an arduino where you send the control commands in real time (bar function) and you want to use the information from the first function to update the controllers of the second. 

```cpp
#include <iostream>
#include <asio.hpp>

int foo();
int bar();

int main(){
    std::thread io_thread{foo};
    bar();
    io_thread.join();
    return 0;
}

```

Where the source code of the foo function is given by 

```cpp
int foo(){
    char input;
    while(true){
        std::cin >> input; //this call blocks until input is provided
        switch(input){
            case 'a': //agressive -> gain 3
            break;
            case 's': //smooth -> gain 1
            break;
            case 'x': //stop control
            break;
            default: //do nothing
            break;
        }
    }
}
```

and the bar function is given by 

```cpp
int bar(){
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open("COM1");
    double gain = 1;
    std::string control_action;
    while(true){
        //we need to check if our agressive control law must change or not 
        asio::write(serial,asio::buffer(control_action.data(),control_action.size()));
    }
}
```

How would you establish the communication between these two functions?
You could develop a atomic flag which signals when something has changed in one thread and read a value when this flag is changed, i.e.

```cpp
int foo(std::atomic<bool>& flag_to_signal,double* value){
    char input;
    while(true){
        std::cin >> input; //this call blocks until input is provided
        switch(input){
            case 'a': //agressive -> gain 3
            *value = 3;
            flag_to_signal.store(true);
            break;
            case 's': //smooth -> gain 1
            *value = 1;
            flag_to_signal.store(true);
            break;
            case 'x': //stop control
            flag_to_signal.store(true);
            *value = 0;
            break;
            default: //do nothing
            break;
        }
    }
}
```

```cpp
int bar(std::atomic<bool>& flag_to_signal,double* value){
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open("COM1");
    double gain = 1;
    std::string control_action;
    while(true){
        if(flag_to_signal.load()){
            flag_to_signal.store(false);
            gain = *value;
            if(gain==0){ //x was pressed we need to stop
                serial.close();
                return 1;
            }
        }
        control_action = std::to_string(gain);
        asio::write(serial,asio::buffer(control_action.data(),control_action.size()));
    }
}
```

And your main function is something similar to 


```cpp
#include <iostream>
#include <asio.hpp>

int foo();
int bar();

int main(){
    std::atomic<bool> signal_flag = false;
    double gain_value = 1.0;
    std::thread io_thread{foo(signal_flag,&gain_value)};
    bar(signal_flag,&gain_value);
    io_thread.join();
    return 0;
}

```

This almost works, there is a bug hidden in the example. Because we are acessing the control_law double memory location from both threads we have a [race condition](https://en.wikipedia.org/wiki/Race_condition) (to understand the read the [cpp memory model](https://en.cppreference.com/w/cpp/language/memory_model)) (one thread could be stopped whilst we were writing to the double and in the meantime we read this value which contains nonsense until the writing operation is finished). To avoid this, the double variable should also be atomic to guarantee that changes are updated in a single shot. As you can see, designing memory safe code is dificult and requires constant attention. 

To deal with these problems curan proposes the class 'SafeQueue' which is basically a queue which we can put things into and request to pull things out of as needeed with guaranteed memory safety. 

```cpp
#include "utilities/SafeQueue.h"

int foo(curan::utilitites::SafeQueue<double>& queue){
    char input;
    while(true){
        std::cin >> input; //this call blocks until input is provided
        switch(input){
            case 'a': //agressive -> gain 3
                queue.push(3.0);
            break;
            case 's': //smooth -> gain 1
                queue.push(1.0);
            break;
            case 'x': //stop control
                queue.push(0.0);
                return 0;
            break;
            default: //do nothing
            break;
        }
    }
}
```

```cpp 
#include "utilities/SafeQueue.h"
int bar(curan::utilitites::SafeQueue<double>& queue){
    asio::io_context context;
    asio::serial_port serial(context);
    serial.open("COM1");
    double gain = 1;
    std::string control_action;
    while(true){
        if(queue.try_pop(gain) && gain==0){
            serial.close();
            return 1;
        }
        control_action = std::to_string(gain);
        asio::write(serial,asio::buffer(control_action.data(),control_action.size()));
    }
}
```

```cpp
#include "utilities/SafeQueue.h"

int foo(curan::utilitites::SafeQueue<double>& queue);

int bar(curan::utilitites::SafeQueue<double>& queue);

int main(){
    curan::utilitites::SafeQueue<double> queue;
    std::thread io_thread{foo(queue)};
    bar(queue);
    io_thread.join();
    return 0;
}

```

This solution reduces code size and guarantees that reading and writing to the queue is safe across threads.

# ThreadPool and Jobs

Now lets look at another handy tool in curan which you might need to use in your code. You have an operation which might take a long time, e.g. image prcessing, and as soon as you receive an image you want to launch a thread to process this task whilst dealing with other things, you don't want to wait for the operation to finish in your main thread. 

Here is how one might try and implement this solution

```cpp 

#include "itkImage.h"

using PixelType = unsigned char;
using Dimension = 2;
using ImageType = itk::Image<PixelType, Dimension>;

ImageType::Pointer magicfunction_returns_image();
void slow_image_filter(ImageType::Pointer);

int main(){
    bool continue_running = true;
    std::vector<std::thread> list_of_threads_running;
    while(continue_running){
        ImageType::Pointer image = magicfunction_returns_image();
        if(image.get()!=nullptr)
            list_of_threads_running.emplace_back(std::thread(slow_image_filter(image)));
        else
            continue_running = false;
    }
    for(auto & thread : list_of_threads_running)
        thread.join();
    return 0;
}
```
We keep looping and getting more images and our slow filter takes these images and is started on a parallel thread which runs our task. This solution has several problems, namely, the number of threads we create is unlimited, which is not always want we want. Remember that your core has a finite number of cores and at some point you will create too many threads which the kernel of the operating system might need to switch between (which takes time). The other drawback of the tecnique is that creating a thread is an "expensive" operation. This is where thread pools come into play.
What if you create a pool of prealocated threads and give them the task you wish to execute? This number of threads would be limited and you would not pay for the creation of the threads as the while loop runs. 

To achieve this solution curan has the concept of a Job. A job contains a description of the task being executed and a [lambda](https://en.cppreference.com/w/cpp/language/lambda) which captures our local variables that we want to use in the future. Inside this lambda we provide it with a copy of the pointer to our image and we call our slow filter. Once the while filter is finished we terminate the thread pool. If you are curious, check how the thread pool is implemented. Its not that difficult to understand.

```cpp

#include "itkImage.h"

using PixelType = unsigned char;
using Dimension = 2;
using ImageType = itk::Image<PixelType, Dimension>;

ImageType::Pointer magicfunction_returns_image();
void slow_image_filter(ImageType::Pointer);

int main(){
    //initualize the thread pool;
	curan::utilities::initialize_thread_pool(10);
    bool continue_running = true;
    
    while(continue_running){
        ImageType::Pointer image = magicfunction_returns_image();
        if(image.get()==nullptr){
            continue_running = false;
            continue;
        }
        curan::utilities::Job job_to_execute;
	job_to_execute.description = "Job to execute";
        job_to_execute.function_to_execute = [image]() {
            slow_image_filter(image);
	    };
        curan::utilities::pool->submit(job_to_execute);
    }
    curan::utilities::terminate_thread_pool();
    return 0;
}
```

# Flags

The last flag which is notable and usefull in your day to day inside the utilities target are multihtreaded safe flags. 

## User Interface

## Image Processing

## 3D Rendering

## Optimization

## Communication

