# Curan

This file describes the Curan Library. The intent is to provide a general
idea on how the library is constructed, how to use the building blocks
supplied to create higher level abstraction until a fully usable UI with
proper functioning is supplied. 

Curan is a medical viewer, developed by IST, with contribution from
Jo�o Oliveira (more to come?). The library provides interfaces which
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

# Compilation

Most third party requirments are included directly in the source code to guarantee
 as streamlined a building procedure as possible. There are three main exceptions, namely:
Vulkan, Skia and ITK. To compile Curan start by creating a folder called development.

´´´code

~path >> mkdir development
~path >> cd development
~path\development >> git clone https://github.com/Human-Robotics-Lab/Curan.git

´´´

Now we need to install Vulkan system wide. For that download the SDK from https://vulkan.lunarg.com/. 
Install the library system wide so that CMAKE can find it.

Once this step is done go back to the command line and we will procede to install SKIA on
our system. To do that do the following

´´´code
~path\development >> git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git
´´´

Now you need to add depot_tools to your system path. Follow the instructions described in 
https://commondatastorage.googleapis.com/chrome-infra-docs/flat/depot_tools/docs/html/depot_tools_tutorial.html#_setting_up.
Once this is done do the following

´´´code
~path\development >> cd depot_tools
~path\development\depot_tools >> git clone https://skia.googlesource.com/skia.git
~path\development\depot_tools >> cd skia
~path\development\depot_tools\skia >> python3 tools/git-sync-deps
~path\development\depot_tools\skia >> bin/fetch-ninja
´´´
If everything went well then you can first create the debug version of the library with

´´´code
~path\development\depot_tools\skia >> bin\gn gen out\Debug --args="is_official_build=false is_debug=true
     skia_use_vulkan=true win_vc=\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\"
     skia_use_system_libjpeg_turbo=false skia_use_system_zlib=false skia_use_system_harfbuzz=false 
     skia_use_system_libpng=false skia_use_system_icu=false skia_use_system_expat=false 
     skia_use_system_libwebp=false extra_cflags=[\"/MTd\"] skia_use_gl=false"
~path\development\depot_tools\skia >> ninja -C out\Debug
´´´

If everything compiled then we create the release version of the library

´´´code
~path\development\depot_tools\skia >> bin\gn gen out\Release --args="is_official_build=false is_debug=false
     skia_use_vulkan=true win_vc=\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\"
     skia_use_system_libjpeg_turbo=false skia_use_system_zlib=false skia_use_system_harfbuzz=false 
     skia_use_system_libpng=false skia_use_system_icu=false skia_use_system_expat=false 
     skia_use_system_libwebp=false extra_cflags=[\"/MT\"] skia_use_gl=false"
~path\development\depot_tools\skia >> ninja -C out\Release
´´´

And how we have SKIA compiled. To install ITK go back to the command line and write

´´´code
~path\development\depot_tools\skia >> cd ..
~path\development\depot_tools >> cd ..
~path\development >> mkdir ITK
~path\development\ITK >> git clone https://github.com/InsightSoftwareConsortium/ITK
´´´

Which will create a folder with the version of the library.

If you are using Visual Studio Code or Visual Studio, you can specify a file in your source directory
arguments to be passed to cmake. 

To build and install SKIA on windows follow the steps described in.
Here is a snipit of the installation procedure

https://skia.org/docs/user/download/

First we install depot_tools by 

Once you have downloaded depot_tools, download skia. To compile the following is the command line 
argument which was used by me to compile. 

Because ITK is a huge library with a lot of source code, and to avoid spending uncessary 
time with build steps we also build this library out of source.  

