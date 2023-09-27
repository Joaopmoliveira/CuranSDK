## Available applications

This folder contains our main applications avaialable in our build system. They are;

> box_specification

- An executable that takes a json file describing the calibrated paratemeters of the ultrasound pose with respect to the flange of the robot and allows us to define a region of interest in space. This renders the robot configuration in real time

> calibrate_ultrasound
- An executable that takes the image and homogeneous transformation from the base of the robot to the flange of the robot, the number of wires used to perform the calibration and after collecting some images from the environment outputs the pose of the ultrasound image with respect to the flange of the robot such that the curvature of the wires is minimizeds

> friopenigtlinktracker
- An executable that connects to the robot and records the current joint configuration and pose of the ultrasound image and creates two servers, server A broadcasts the current homogenenous transformation in the OpenIGTLink protocol, whilst server B broadcasts the joint configurations of the robot.

> gaussian_mixture
- A series of executables that implement the control law in real time given a pre-trained Gaussian Mixture Model

>integrated_3d_ui
- An executable (work in progress) that allows one to both specify the region of interest and perform the volumetric reconstruction of the environment in a single run. The application  is still unfinished because the previous setup works and other development activities take precedence

>registration
- An executable that takes two volumes, one which contains the CAT-MRI of the cranium and the reconstructed volume and outputs the rigid transformation between them.

>robot_ultrasound_visualization
- An executable that showcases the rendering in real time and shows the ultrasound image in space