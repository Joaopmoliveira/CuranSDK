So you have found yourself in front of Curan and you have sucessefully compiled the Curan SDK without any errors. Now you have a goal in mind and want to start implementing your own custom solutions for your medical applications. Well, this tutorial introduces the base cases of the classes available to achieve your goals. 

Curan is divided into sic main portions, so called libraries. You have a utilities library which contains things usefull in multithreaded scenarios, blocking queues, atomic flags and so on. Then you have a user interface library. This component of curan is build on top of two main libraries, Vulkan to actually communicate with your local machine and SKIA which allows you to render any geometry on screen. We choose these two solutions for two main reasons, Vulkan is a recent API, which means that it will have support for a long time whilst SKIA is the rendering engine used by google to draw geometries across most browsers. The third library is for communication, where currently Serial communication is implemented and TCP/IP communication is implemented with the OpenIGTLink protocol. The fourth library is the imageprocessing module, where volumetric reconstruction algorithms are implemented. The fifth library is the rendering one, which uses VSG to render 3D scenes on the environment. The last patch is the optimization library which contains the Ceres solver to optimize the configuration of the wires whilst doing ultrasound calibration. 

We will introduce each of these modules in separate sections of this website, but for now we will also give you an overview of the logic behind the project layout, for you to feel more confortable manipulating and changing the website as required. On all CMake based projects there is always a root directory to declare and configure a given project. This is the folder Curan with the following structure

Curan ---
        |->Curan
        |
        |->Documentation
        |
        |->CMakeLists.txt
        |
        |->vcpkg.json

The reason we have two folders named curan is because we are using a tecnique simillar to what some might call super build in cmake parley. This strategy is required because the libraries associated with VSG currently have not been integrated into the other strategy we use to pull in 3d party libraries, namely vcpkg. This is a package manager where you tell it which libraries you want, and the manager pulls in the necessary projects into your own project. This is how most of our third_parties are pulled in. Now that you understand why we have this commical structure lets go into the actual folder which contains Curan

Curan ---
        |->Curan  <- this one
        |
        |->Documentation
        |
        |->CMakeLists.txt
        |
        |->vcpkg.json

Inside this project you have a cmake which defines the curan project. This folder contains a third_party folder (ignore this, we need it because some libraries we need are not compatible with cmake, thus we had to write a local port in our project to use them), a tests folder, a src folder (this is for legacy purposes, ignore it this is not used by Curan), a resources folder, libraries and an applications folder. Now we can explain the purpose of each of them 

Curan ---
        |-> applications
        |
        |-> libraries
        |
        |-> resources
        |
        |-> tests