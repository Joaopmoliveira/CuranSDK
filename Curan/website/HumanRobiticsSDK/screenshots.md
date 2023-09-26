---
layout: "page"
title: "Demos" 
permalink: /screenshots/
---

Here are some of the things that Curan can do for you!

# Calibrate the Ultrasound 
Calibration algorithms are an important part of any system, which can either make or break the system. We developed our own calibration algorithm that takes the pose of the flange of the robot and with any phantom, with any geometry**, can provide with the unknown transformation between the flange of the robot and the coordinate system of the ultrasound image. 

We use our custom UI to render things on screen. This UI has been improved and worked on to simplify as much as possible to the parameterization of the parameters required to calibrate the filter parameters

# Select a region of interest with the flange of the robot

Although a trivial task, we wrote a software solution that shows you the pose of the robot in real time as you specify the region of interest in space you wish to reconstruct. 

# Do volumetric reconstruction in real-time with high fidelity and low latency

Humans, similarly to control systems, trive with low latency solutions. We found that previous volumetric reconstruction techniques were unsatisfactory to our needs. Thus we wrote our own volumetric reconstructor, which is thightly integrated with Vulkan to improve latency of the display and the events in the real world. 

# Control the robot with Learning from Demonstration frame-works

To encode behavior in the robotic system, we prove control systems that 