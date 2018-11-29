# PROJECT CUBOID
This repository contains the code and supplements to the cuboid locomoting and mapping robot for Intelligent Robotics Lab. In this repo, you will find the ROS code, paper, presentations, and data. In the following sections, we will talk about how you can get started with this project!

### Contributors
Andrew Morgan  
Robert Baines  
Hayley McClintock  

## System Requirements 
The code for this repository was mostly developed with/for ROS Kinetic (Ubuntu 16.04), with Python 2.7. OpenCV version 3.3.1-dev provides a stable bridge from OpenCV2 to OpenCV3, and was used for vision processing in this environment. If you try to run this package and have errors, please ensure you have a similar version. An Arduino Mega 256 microcontroller was utilized to control locomotion bladders and LED sequences. This communication was performed in Python and is fairly trivial with the pyserial interface. Please note that the Arduino code utilized is located within the ROS `hardware_coms` package, and is specific to the setup of this cuboid. LED addressing will need adjusted according to your specific unit.   

## ROS Packages
This section serves to introduce the reader to the packages involved in the ROS workspace. In general,the main packages are as follows:
### Vision Processor
This package takes in an image from the USB cam `topic: /camera/image_raw` and processes it accordingly. It will then publish the result according to desired block resolutions determined inside of the file (since we did not change this much, we stuck with 40 rows and 50 columns = 2000 data points).  

### USB Camera
Standard ROS camera package for working with and publishing an image from a webcam. Our main interest in this package is the use of  the`/camera/image_raw` published image. This is an open package for ROS that many people use. It is important to note that your camera does limit the resolution of this node. For the cameras used in this project, we were restricted to a `640 x 480` image, where typically you will work with 720p or 1080p. If you are requesting too high of a resolution, you will receive a camera error that eludes to improper communication. As of ubuntu 10, all drivers should be installed on your system upon setup, so therefore you need to change you camera resolution. 

### ROS Video Recorder 
Video Recorder node used for capturing published video channels in ROS. Specific parameter files must be set, allowing for multiple  videos to be recorded at the same time. To set these files, simply change data in the associated launch file in this package. In general, you will want your output and input video to be the same resolution (the package allows for users to record multiple streams at the same time, but we do not need that feature). 

### Master Control
This package will be used as the controller for all processes and timing for our system. This node communicates with the others, in a top down approach (as presented in the paper), to coordinate actuation, take pictures given specific lighting condiitions, and create the final map. You must run this node and tell it to start. Upon starting, it will complete the following sequence:  

```python
SIDE = 1
while True:
  flipCuboid()
  for edge in sideEdges:
    lightUpSequence(Edge)
    s = takePicture()
    out = processData(s)
    save(out)
  SIDE++
  SIDE = (SIDE-1)%4+1  #Sides are 1-4

```
## Running the Code
To get started, you will need the requirements specified in the previous section. After completing this task, you can launch the main cuboid program by:  
``roslaunch master_controller cuboid.launch``
After the system is up and running, you will need to enable movement and data collection by calling the roservice:  
``rosservice call /master_controller/enable_collection "data= true"``  
  
A useful tool for debugging is by running:  
`roslaunch hardware_coms hardware_coms_std.launch`  
This will allow you to interract with the arduino and send individual commands to the cuboid (turn on/off lights, inflate/deflate bladders). Please follow the integer values specifies in `ROS_ws/src/hardware_coms/Arduino Code/Arduino Communication.xlsx`. For example, if you wanted to inflate the bladders for the third face,  
``rosservice call /hardware_coms/do_something "data=3"``  
or if you would like to light up edge 3 of the second face,  
``rosservice call /hardware_coms/do_something "data=23"``  










