# MOVR's Arduino Driver
> Arduino driver for MOVR - a small-size autonomous vehicle

This Arduino sketch subscribes to a "drive command" topic.
The messages received are used to control the vehicle's (front) steering motor and (rear) drive-motor.
The sketch also creates a ROS publisher to publish odometry data generated from the wheel encoders (and IMU).

# Table of Contents
- [Dependencies](#dependencies)
- [How to use](#how-to-use)
- [Development](#development)
- [References](#references)

## Dependencies
- **rosserial_arduino**   
  - Arduino [ROS extensions](http://wiki.ros.org/rosserial_arduino) to run rosserial_clients.      
- **ackermann_msgs**   
  - [ROS messages](http://wiki.ros.org/ackermann_msgs) for vehicles using [ackermann steering](https://en.wikipedia.org/wiki/Ackermann_steering_geometry).    
- **PID Library**   
  - An easy to use Arduino [PID library](http://playground.arduino.cc/Code/PIDLibrary).  

## Installation

```
# If the ROS 'ackermann_msgs/AckermannDrive' has not been installed
$ sudo apt install ros-kinetic-ackermann-msgs
```

```
# Install ROS rosserial, rosserial-arduino, rosserial-python
$sudo apt install ros-kinetic-rosserial ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python
```

```
# Generate Arduino ros_lib
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
```

## How to use 
``` 
# STEP 1: Run roscore
$ roscore
```

```
# STEP 2: Run rosserial node
$ rosrun rosserial_python serial_node.py /dev/ttyACM1
```

```
# STEP 3: Run the node that will publish the drive commands
# For example the movr_teleop hello_movr.py
# Note that you must 'source' the movr_ws for the movr_teleop package to be visible
$ rosrun movr_teleop hello_mvr.py
```

## Development

### Robot odometry
#### Encoder ####
[LPD3806 600BM G5 24C Rotary Encoder](http://www.made-in-china.com/showroom/jn-syjm/product-detailtsLnXSdoEaWV/China-Lpd3806-600bm-G5-24c-Ab-Two-Phase-5-24V-600-Pulses-Incremental-Optical-Rotary-Encoder.html)   

##### Wire Mappings: #####   
```
Wire Color     Connections   
RED            5 - 24V DC   
BLACK          Ground   
GREEN          A Phase   
WHITE          B Phase   
```

##### Spefications: #####    
- 600 pulses/revolution for a single phase.   
- Therefore, two-phase output leads to 24000   
- Maximum mechanical speed: 5000 Revolutions / minute   
- Response Frequence: 0 - 20KHz   

##### Basic math #####

```
CALIBRATION_FACTOR (Distance Traveled Per Click)
= PI * Wheel Diameter / Number of Clicks per Revolution
``` 

```
DISTANCE TRAVELED BY A WHEEL
= Calibration Factor * Number of Clicks Traveled by Wheel
```

```
AVERAGE DISPLACEMENT BY BOTH WHEELS
= (Distance Traveled by Left Wheel + Distance Traveled by Right Wheel) / Wheel Baseline

# Wheel Baseline = The Distance Between the Left Wheel and Right Wheel Contact Point
```

## References
- [Brett Bearegard's PID Introduction](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/)
- [Henry's Bench Digital to Analog Tutorial](http://henrysbench.capnfatz.com/henrys-bench/arduino-voltage-measurements/arduino-pcf8591-digital-to-analog-tutorial/)
- [Tori's Electronics Lab Rotary Encoder Tutorial](https://toriilab.blogspot.com/2016/09/a-rotary-encoder-is-used-to-measure.html)
- [ROS Serial](http://wiki.ros.org/rosserial)
- [ROS Serial Arduino](http://wiki.ros.org/rosserial_arduino)
- [TTU Advanced Robotics Odometry Tutorial](http://ttuadvancedrobotics.wikidot.com/odometry#toc9)
- [Correl Lab's Forward Kinematics of Car-Like Mechanisms](http://correll.cs.colorado.edu/?p=1869)
- [Steven LaValle's Kinematics of a Simple Car](http://planning.cs.uiuc.edu/node658.html)
- [ROS Q&A on Twist to Ackermann-type Conversion](https://answers.ros.org/question/260935/twist-message-working-ackermann-type-conversion/)
- [Ackermann Messages Defined by the ROS Ackermann Interest Group](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html)
