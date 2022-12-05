# Challenge 1: Flashing Distress

In this challenge, we flash the distress signal on the vehicle using the turn indicator lights.

## Demo
[![Vehicle flashes Distress Signal via turn indicator lights](https://img.youtube.com/vi/goFwmhLmExY/0.jpg)](https://www.youtube.com/watch?v=goFwmhLmExY)


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