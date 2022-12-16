# Challenge 2: Pedestrian Braking

In this challenge, we program the vehicle to brake when it detects a pedestrian in its way. The vehicle again accelerates in the absence of any pedestrian.

## Approach

We use the YOLO-V5 model to detect obstacles from the camera feed of the vehicle.

## Demo

[![Demo 1 - Vehicle brakes upon detecting a pedestrian](https://img.youtube.com/vi/AnTwoKCJmUU/0.jpg)](https://www.youtube.com/watch?v=AnTwoKCJmUU)

[![Demo 2 - Vehicle brakes upon detecting a pedestrian](https://img.youtube.com/vi/AsS8P6OSIcg/0.jpg)](https://www.youtube.com/watch?v=AsS8P6OSIcg)


## Run
First, compile the setup to generate auxillary files.
```commandline
$ cd ~/demo_ws/
$ catkin_make
```

In another terminal, run the following command to enable the control
```commandline
$ source devel/setup.bash
$ roslaunch basic_launch gem_dbw_joystick.launch
```

> Press Back and Start to Enable

In another terminal, run the following command to run the code on the vehicle.
```commandline
$ source devel/setup.bash
$ rosrun mp0 mp0.py
```
