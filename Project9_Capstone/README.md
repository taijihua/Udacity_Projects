# Capstone Project (system integration project)
This Project is submitted for Self-Driving Car Engineer Nanodegree Program, as a final project.

[//]: # (Image References)
[image_ROS]: /imgs/final-project-ros-graph-v2.png


### Team member
Junhua Tang (tangjunhua.osu@gmail.com)

### Goal of the project
The project code should enable a car driving it self in a udacity simulator, and the same code could also autonomously drive a real car (Carla,an autonomous Lincoln MKZ, from Udacity) in a test lot with traffic lights. Detailed checklist for the project code:
* Launch correctly using the launch files provided in the capstone repo
* Smoothly follow waypoints in the simulator.
* Stop at traffic lights when needed.
* Stop and restart PID controllers depending on the state of /vehicle/dbw_enabled.
* Publish throttle, steering, and brake commands at 50hz.

### Architecture (ROS nodes)
![Udacity ros nodes][image_ROS]
### How i did it
* Perception (detection of traffic light status)

A pretrained MobileNet-SSD model from [tensorflow object detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) was used for traffic light detection, the model was previously trained on COCO data set and was able to recognize traffic lights (this saved me a lot of time and effort!). After the tensorflow model gave me the boundingbox of the traffic lights, i used opencv to analyze the cropped area in HSV space and by comparing its red, yellow and green component, i was able to get reliable detection of light status.

* Planning

The project code starter provides basic waypoints that were mapped out on the roads, so we only need to localize the vehicle and use the basic waypoints that are in front of the vehicle to plan the path forward.

* Control

A PID controlled is used for controlling throttle and brake. A seperate yaw controller for steering control.
