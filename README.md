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
Vulkan, Skia and ITK. Vulkan is the low level GPU API which allows one to control the execution 
of the GPU. It can be downloaded from XX. 
Install the library system wide so that CMAKE can find it.

If you are using Visual Studio Code or Visual Studio, you can specify a file in your source directory
arguments to be passed to cmake. 

To build and install SKIA on windows follow the steps described in.
Here is a snipit of the installation procedure

Once you have downloaded depot_tools, download skia. To compile the following is the command line 
argument which was used by me to compile. 

Because ITK is a huge library with a lot of source code, and to avoid spending uncessary 
time with build steps we also build this library out of source.  

