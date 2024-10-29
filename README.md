# Curan

Welcome to the Curan SDK, a framework developed to explore surgical navigation platforms with flexible-joint robots. 
At its essence Curan is a medical viewer, developed at IST, with contribution from
the contribution list and the end of the file. The library provides interfaces which
simplify the implementation of medical demos with real time
capabilities. 

The SDK is divided into four main modules
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

(if on windows)
3.* Install the build tools required for your operating system - Install the lattest version of Mycrosoft Visual Studio - Community Edition which integrates C++ compilers in your system. 

3.1* Search in your system for the command window "x64 native tools command prompt for vs 20XX" where XX is whatever version of Visual Studio that you
installed.

(if on linux)
3.* Install the build tools required for your operating system - Install the lattest build-tools which integrates C++ compilers in your system.


4. To set up the package manager that deals with our third party dependencies go to your command prompt and write
```sh
~path >> git clone https://github.com/Joaopmoliveira/CuranSDK.git
~path >> cd CuranSDK
~path/CuranSDK >> cd vcpkg
~path/CuranSDK/vcpkg >> ./vcpkg/bootstrap-vcpkg.bat
```

5. Now we have to specify a couple of details which are specific to each computer. In modern build systems CMake is the tool for the job.
We can configure CMake through a file called CMakePresents.json which specifies flags required for proper compilation of the project. s

```
{
  "version": 5,
  "configurePresets": [
    {
      "name": "debug",
      "displayName": "Debug",
      "generator": "Ninja",
      "binaryDir": "build/debug",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug",
        "CURAN_PLUS_EXECUTABLE_PATH": "~path_to_plus/bin/PlusServer.exe"
      }
    },
    {
      "name": "release",
      "displayName": "Release",
      "generator": "Ninja",
      "binaryDir": "build/release",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "CURAN_PLUS_EXECUTABLE_PATH": "~path_to_plus/bin/PlusServer.exe"
      }
    }
  ]
}
```

Now we could just go to the command line and install the dependencies as in

```sh
~path/CuranSDK >> cmake -B build -S . 
~path/CuranSDK >> cmake --build build
```

And the project should just compile out of the box (this will take a LOOOOONG TIME to compile 
because ITK and SKIA are huge). Reserve at least 30Gb of memory for vcpkg to compile all the dependencies.


# Supported operating systems

Althought the software was developed based on open source solutions which are compatible accross multiple operating systems, the build system, aka VCPKG, might only provide experimental support for obscure platforms. If you face any problem with vcpkg try to search online for custom solutions for your particular system. The officially supported operating systems are :

1. Windows 
2. Ubuntu - Linux

## Integration with a proper IDE 

Usualy our IDE of choice is either vscode or visual studio. We prove the instructions for vscode.
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

## Integrating Plus directly in the source code

When doing demonstrations for outsiders, or trying to use the software in real world environments, having to deal with manually lauching Plus
can be a bother. Thus CURAN allows you to attach the root path to the Plus Server so that when you launch the main application, ApplicationLauncher
plus is automatically launches as well, reducing the ammount of work required by you. This is an optinal behavior, thus if you desire to trigger it,
you must pass to CMAKE the command line option 
```
"-DCURAN_PLUS_EXECUTABLE_PATH=C:/Users/joaom/PlusApp-2.8.0.20191105-Win32/bin/PlusServer.exe"
```
Assume that you have your plus executable installed in the root folder

```
C:/Users/example/PlusApp-2.8.0.20191105-Win32/bin/PlusServer.exe
```

If you are using vscode as your development environment, you can instead modify settings.json with the following extra parameter

```
{
    "cmake.generator": "Ninja",
    "cmake.configureArgs": [
        "-DVCPKG_APPLOCAL_DEPS=ON",
        "-DX_VCPKG_APPLOCAL_DEPS_INSTALL=ON",
        "-DVCPKG_MANIFEST_MODE=ON",
        "-DVCPKG_TARGET_TRIPLET=x64-windows-static",
        "-DCURAN_PLUS_EXECUTABLE_PATH=C:/Users/example/PlusApp-2.8.0.20191105-Win32/bin/PlusServer.exe"
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

Now once you build the executable ApplicationLauncher will deal with the pesky details of launching plus for you.

## Build Problems

When using VScode to compile the project, if the previous order of the build instructions is not followed properly, the build fails due to incorrect configurations. When you find yourself faced with these problems the simplest solution is to delete the .vscode folder and the build folder and reconfigure the project, this usually solves all the problems. 

## API Reference

Curan finaly has a [website](https://human-robotics-lab.github.io/CuranWeb/) where you can follow our tutorials on how to use the SDK we developed at the surgical robotics lab.

## Acknowledgments 
The volume reconstruction code is essencially a copy of [IGSIO](https://github.com/IGSIO/IGSIO) modified to work with the interal classes and structures of Curan. 

## Contributions 

> -João Oliveira (Lead developer)
> 
> -Filipe Varela (commited to the 3D rendering patch of the library)
> 
> -Manuel Carvalho (commited to the implementation of registration algorithms)
>
> -Rui Coelho (commited to the implementation of the RealTime kernel)
>
> -Álvaro Lopes (commited to temporal calibration and registration) 
> 
> - (You) Become a contributer!
