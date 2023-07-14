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

3. Install the lattest version of Mycrosoft Visual Studio - Community Edition which integrates C++ compilers in your system. 

4. Search in your system for the command window "x64 native tools command prompt for vs 20XX" where XX is whatever version of Visual Studio that you
installed.

5. Use vcpkg to build the required third party libraries
To use this strategy we can first install vcpkg as: (this is all done in the x64 native tools command prompt)

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
    "assimp",
    {
      "name": "skia",
      "features": [ "vulkan" ]
    }
  ]
}
```

Note that the rendering dependencies are still missing, this is because vcpkg still has a tiny bug 
which is currently being solved in a commit from the team. (To solve this problem we use a super
build arquitecture to achieve our goals)

```sh
~path/development >> cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE="~path/development/vcpkg/scripts/buildsystems/vcpkg.cmake" -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$<$<CONFIG:Debug>:Debug>
~path/development >> cmake --build build
```

And the project should just compile out of the box (this will take a LOOOOONG TIME to compile 
because ITK and SKIA are huge). Reserve atleast 30Gb of memory for vcpkg to compile all the dependencies.

## Integration with a proper IDE 

Usualy my IDE of choice is either vscode or visual studio. I will prove the instructions for vscode.
Follow the following steps 

1. First install Git in your system it you do not have it installed yet.

2. Second install Vulkan from the website https://vulkan.lunarg.com/sdk/home (this is a graphics API to communicate with the GPU of your computer)

3. Install the lattest version of Mycrosoft Visual Studio - Community Edition which integrates C++ compilers in your system. 

4. Now you can install vscode by downloading it from the website https://code.visualstudio.com/

Once the download is finished you can open the vscode IDE, go to the extensions tab, install the c++ extension from windows, the cmake extension
and also install the vcpkg extension. Once this is done go to the page of the vcpkg extension and enable it (this should create a folder in your 
project called .vscode) with a file inside it called settings.json with the following contents

```
{
    "cmake.generator": "Ninja",
    "cmake.configureArgs": [
        "-DVCPKG_APPLOCAL_DEPS=ON",
        "-DX_VCPKG_APPLOCAL_DEPS_INSTALL=ON",
        "-DVCPKG_MANIFEST_MODE=ON",
        "-DVCPKG_TARGET_TRIPLET=x64-windows-static"
    ],
    "vcpkg.general.enable": true,
    "vcpkg.target.hostTriplet": "x64-windows-static",
    "vcpkg.target.defaultTriplet": "x64-windows-static",
    "vcpkg.target.useStaticLib": true,
    "cmake.configureSettings": {
        "CMAKE_TOOLCHAIN_FILE": "path to your vcpkg instalation directory",
        "CMAKE_MSVC_RUNTIME_LIBRARY" : "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    },
    "vcpkg.target.installDependencies": true,
    "vcpkg.target.preferSystemLibs": false,
    "vcpkg.target.useManifest": true
}
```

Notice that we are forcing the cmake extension to pass the arguments of where vcpkg is installed in the line --CMAKE_TOOLCHAIN_FILE: "path to your vcpkg instalation directory"--. This should compile out of the box once all the steps are solved.

## Build Problems

When using VScode to compile the project, if the previous order of the build instructions is not followed properly, the build fails due to incorrect configurations. When you find yourself faced with these problems the simplest solution is to delete the .vscode folder and the build folder and reconfigure the project, this usually solves all the problems. 

## API Reference

Curan finaly has a [website](https://human-robotics-lab.github.io/CuranWeb/) where you can follow our tutorials on how to use the SDK we developed at the surgical robotics lab.

## Acknowledgments 
The volume reconstruction code is essencially a copy of [IGSIO](https://github.com/IGSIO/IGSIO) modified to work with the interal classes and structures of Curan. 

## Contributions 

> -João Oliveira (Lead developer)
> 
> -Filipe Varela (3D rendering patch of the library)
> 
> -Manuel Carvalho (Implementation of registration algorithms)