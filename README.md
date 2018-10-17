# project_cuboid
Code for cuboid locomoting and mapping robot for Intelligent Robotics Lab - ROS catkin workspace

## Contributors
Andrew Morgan
Robert Baines
Hayley McClintock

## System 
This package was created and utilized with ROS Kinetic (Ubuntu 16.04). OpenCV version 3.3.1-dev provides a stable bridge from OpenCV2 to OpenCV3. An arduino microcontroller was utilized to capture sensor data and control locomotion bladders. This communication in Python is trivial with the pyserial interface. 

## Packages

### Vision Processor
This package takes in an image from the USB cam and processes it accordingly. It will then publish the result according to desired block resolutions. 

### USB Camera
Standard ROS camera package for working with and publishing an image from a webcam. Our main interest in this package is the use of  the`/camera/image_raw` published image.

### ROS Video Recorder 
Video Recorder node used for capturing published video channels in ROS. Specific parameter files must be set, allowing for multiple  videos to be recorded at the same time.  
