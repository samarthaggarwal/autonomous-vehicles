# Challenge 4: GPS Scribbling

In this challenge, we run the vehicle as per the waypoints (GPS coordinates) given to it via a csv file.

## Approach

1. Identify the coordinates of High Bay in the coordinate system of the simulator.
2. Mark the coordinates of the vertex key points to form the 8-shape.
3. Interpolate the coordinates of key-points to get a large number of waypoints.
4. At every step, move the vehicle such that it approaches the next waypoint.

## Demo

[![Vehicle follows a pattern of '8' using GPS Scribbling](https://img.youtube.com/vi/6lJewkgkqH8/0.jpg)](https://youtu.be/6lJewkgkqH8)


## Run

The following command starts RViz with the vehicle in the High Bay environment at the start position.
```commandline
roslaunch gem_launch gem_init.launch world_name:="highbay_track.world" x:=-30.25 y:=-16.5
```

In another terminal, run the following command to visualise the simulation data.
```commandline
roslaunch gem_launch gem_sensor_info.launch
```

In another terminal, run the following command to run the vehicle as per the waypoints.
```commandline
# for simulation
rosrun gem_pure_pursuit_sim pure_pursuit_sequential_sim.py

# for production
rosrun gem_gnss pure_pursuit_pp_tracker_pid.py
```

