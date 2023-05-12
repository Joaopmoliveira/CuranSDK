# Timeline

We need to fully flesh out some parts of the library before we can claim that it
is ready to be used in other units, i.e. use it for creating discrete executables
built upon common code.

The project scope lies in implementing a communication layer, a 2D visualization toolset
that we can use for beatifull and FAST UI, a 3D visualization toolset that speeds up the 
implementation of ideas, and most importantly we must be able to scale fast.

To get to this ideal we define in this file how to reach this goal.

# Implementation steps

Implement a common utilities API that we can use across all libraries, i.e. thread safe things that we want to 
control precisly how they work. This can improve performance at the expense that any change in
the utilities library might break all the other libraries. After this the communication library must be implemented 
upon ASIO. Its also important to define the protocols that we wish to interact with and define a common API
that is extendeble for new protocols if need be. The graphics library contains Widgets implementations built 
upon SKIA. With this code we wish to provide tooling to facilitate implementing 2D UIs where interaction is important. 
The last important component is the 3D visualization that is built upon the VSG library to perform real time 
rendering of both simple primitives and more complex tools like robots, real time textures that are associated with 
ultrasounds and other requirments. This can be cleanly separated into:

1. Utils
2. Communication
3. 2D visualization
4. 3D visualization

# Work being done

In general, the thing that is missing the most work is the 3D visualization which is still being developed. To 
achieve good results we must wrap VSG in a good independent logic. For now we dont want to integrate everything in common window,
so we can focus our testing on utilities which work independently. 

1. In terms of priorities we need to finish the Communication - mostly done.
2. Reimplement the 2D library in proper cpp fashion, multiple header files and so on - mostly done.
3. Implement the 3D library - mostly done.


