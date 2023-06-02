# Curan

This file describes the Curan Library. The intent is to provide a general
idea on how the library is constructed, how to use the building blocks
supplied to create higher level abstraction until a fully usable UI with
proper functioning is supplied. 

Curan is a medical viewer, developed by IST, with contribution from
the contribution list and the end of the file. The library provides interfaces which
simplify the implementation of medical demos with real time
capabilities. 

The library is divided into four main modules
1. The utilities library contains base classes used by all other modules,
such as thread safe queues, blocks used to connect objects, memory buffers
which can usurp the ownbership of shared pointers, etc..
2. The communication library supplies higher level abstractions over asio which
allow us to lauch an application with a indifinite number of clients, with any protocol.
Currently only the OpenIGTlink protocol is implemented in the library. More protocols
can be added at will without any effort.
3. The userinferface library contains higher level abstractions over the Skia library
to render 2D entities in real time. 
4. The imageprocessing library contains the simplified wrappers around ITK which 
should simplify the development speed whilst implementing sequencial filters.
5. The optimization library contains classes dedicated to particular optimization problems
which are usefull to us (for now only in the medical context, but this can be extended
to other types of optimizations).

# Build intructions

1. First install Git in your system it you do not have it installed yet.

2. Second install Vulkan from the website https://vulkan.lunarg.com/sdk/home (this is a graphics API to communicate with the GPU of your computer)

Use vcpkg to build the required third party libraries
To use this strategy we can first install vcpkg as:

```sh
~path >> mkdir development
~path >> cd development
~path/development >> git clone https://github.com/Microsoft/vcpkg.git
~path/development >> ./vcpkg/bootstrap-vcpkg.bat
```

Now we have two options, the first one is to install universally the dependencies 
of our project (this is called the classic mode). The second strategy is the manifest
mode where we define a file, vcpkg.json, with the dependencies of our project. Currently our
project has the following dependencies

```
{
  "dependencies": [
    "eigen3",
    "itk",
    "ceres",
    "openigtlink",
    "asio",
    "glfw3",
    {
      "name": "skia",
      "features": [ "vulkan" ]
    }
  ]
}
```

Note that the rendering dependencies are still missing, this is because vcpkg still has a tiny bug 
which is currently being solved in a commit from the team. Now to compile the project execute the 
following lines in your command line. 

```sh
~path/development >> cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE="~path/development/vcpkg/scripts/buildsystems/vcpkg.cmake"
```

And the project should just compile out of the box (this will take a LOOOOONG TIME to compile 
because ITK and SKIA are huge). Reserve atleast 30Gb of memory for vcpkg to compile all the dependencies.

## Contributions 

> -João Oliveira (Lead developer)
> 
> -Filipe Varela (3D rendering patch of the library)
