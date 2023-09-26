---
layout: "page"
title: "Demos" 
permalink: /screenshots/
---

Here are some of the things that Curan can do for you!

# Communicate with the LBR Med in realtime

To communicate with the robot in real time, two main options exist:
1. Start a TCP connection inside the robot controller that broadcasts the current pose in OpenIGTLink format
2. Connect yourself through FRI to control the robot at 1KHz

We have implemented both, but to give us the freedom to control the robot whilst doing data collection we prefer the later option. To this end we always have a linux machine running the process FRIPlusBridge which does two things. Establishes a server which broadcast the current pose of the end-effector and a server that broadcasts the joint configuration of the robot in realtime. This is usefull for rendering purpouses. 
To launch this process go to your command line and run

```
>> FRIPlusBridge Base 40
```

where Base if the name of the frame of reference of the world frame, as convention we always use Base in our XML configuration files for Plus. The integer represents the delay between each image which is broadcasted to the outside world. Make sure this delay is large enough to give time to the Plus Server to process the previous combination of Image+RobotPose before sending a new pose. 

# Calibrate the Ultrasound 

> warning: **To run this application you should start the FRIPlusBridge described in the previous section**

Calibration algorithms are an important part of any system, which can either make or break the system. We developed our own calibration algorithm that takes the pose of the flange of the robot and with any phantom, with any geometry**, can provide with the unknown transformation between the flange of the robot and the coordinate system of the ultrasound image. 

We use our custom UI to render things on screen. This UI has been improved and worked on to simplify as much as possible to the parameterization of the parameters required to calibrate the filter parameters. Lets take a look at how the UI works

Once the program is launched, a window is created where by default the connection with Plus is not established. Launch the PlusServer to collect both pose and images from the robotic system

![github desktop](/assets/images/ultrasound_calibration_ui.png) 

once you have launched PlusServer you can click the connect button. If everything works smoothly, the button will turn green, indicating that the connection to Plus was succefull

![github desktop](/assets/images/ultrasound_calibration_connected.png) 

now, the application is receiving an ultrasound image, and we need to filter the positions of the wires in real time from said image. To do this, a series of filters are used, which can be calibrated in real time, so that you can validade if they are working properly or not. Notice that the UI is overlayed over the real image, thus you can verify the quality of the segmentation in real time

![github desktop](/assets/images/ultrasound_calibration_filter_options.png) 

to check where the application is identifying wires you can request it to show the wire positions in real time. 

![github desktop](/assets/images/ultrasound_calibration_circles.png) 

by default if you want to segment N wires, the application will initialy only segment N wires, but as soon as you start the data collection, i.e. put the positions of the wires in a containers a move the robot, the application segmentes N+3 wires, and computes the distance from the previous wires (which is assumed to be correct) and selects which wire of the current frame belongs to the previous segmented wires. This provides robustness to the segmentation in real time. Once you decide you have collected enough data you can terminate the application, i.e. close the window, and a json file will be produced in the current directory of the shell where the application is running. This file looks something like 

```json 
{
   "homogeneous_transformation":"   0.753799,   0.656902, -0.0163297,  -0.318765 \n   0.566919,  -0.637577,    0.52163,  -0.136224 \n   0.332248,  -0.402462,  -0.853015,   0.125395 \n          0,          0,          0,          1 \n",
   "optimization_error":0.008909722128714685,
   "timestamp":"2023-09-12 14:24:55"
}
```

where a timestamp is associated with the homogeneous transformation estimated, the optimization error, so that processes further down the stream can decide if the error is small enough and a string, containing the estimated homogeneous transformation.

# Visualize the calibrated ultrasound image with the robot display

> warning: **To run this application you should start the FRIPlusBridge previously described**

You have just performed the calibration of your setup and you want to test if everything is perfect, according to your specifications. We wrote an application where you can see the rendered robot, next to the ultrasound image positioned with respect to the flange of the robotic system 

![github desktop](/assets/images/robot_ultrasound_visualization.png) 

this gives you an idea if the relative position of the system is proper. 

# Select a region of interest with the flange of the robot

> warning: **To run this application you should start the FRIPlusBridge previously described**

Although a trivial task, we wrote a software solution that shows you the pose of the robot in real time as you specify the region of interest in space you wish to reconstruct. Two main reasons for this, first because its fun, second because it gives you some idea of how large this box is with respect to the size of the world.

# Do volumetric reconstruction in real-time with high fidelity and low latency

> warning: **To run this application you should start the FRIPlusBridge previously described**

Humans, similarly to control systems, trive with low latency solutions. We found that previous volumetric reconstruction techniques were unsatisfactory to our needs. Thus we wrote our own volumetric reconstructor, which is thightly integrated with Vulkan to improve latency of the display and the events in the real world. 

# Control the robot with Learning from Demonstration frame-works

To encode behavior in the robotic system, we prove control systems that 