# Project: Smart Summon

As part of the final course project, we program the ability to _smartly summon_ the vehicle.

The vehicle knows its current location (localisation using GPS). The user enters destination coordinates. In the real world scenario, users can tell their current location to the parked car via smartphone application. The car then goes from source (current location) to destination and avoids any obstacle on the way.

## Test Instructions
- Specify the destination coordinates [here](https://github.com/samarthaggarwal/autonomous-vehicles/blob/main/project/config.py#L1). Note that the destination coordinates are specified in metric coordinate system with origin as the taped box on highbay.
<img src="../assets/highbay.png" alt="coordinate system" width="300"/>

- Specify the testing mode [here](https://github.com/samarthaggarwal/autonomous-vehicles/blob/main/project/lidar/config.py#L1) i.e. simulation mode or gem vehicle mode

Run the following commands in separate terminals. Run `source devel/setup.bash` before running each of the following commands.

### Simulator Test
- `roslaunch gem_launch gem_init.launch world_name:="project.world" x:=-19.89 y:=-17.35`
- `roslaunch gem_launch gem_sensor_info.launch`
- cd `./project/lidar` and run `python3 detect_obstacles.py`
- `rosrun summon summon_sim.py`

### Gem Vehicle Test
- `roslaunch basic_launch gem_sensor_init.launch`
- `roslaunch basic_launch gem_dbw_joystick.launch`
- cd `./project/lidar` and run `python3 detect_obstacles.py`
- `rosrun summon summon_voronoi.py`
- `rosrun summon summon_vehicle.py`

### Voronoi Test
Generate a random grid of obstacles and see its voronoi diagram.

```python
python voronoi.py TestVoronoi.test_random
```

Run all tests

```python
python voronoi.py
```

The following code generates a grid of 100*100 pixels with randomly placed 0s (free space) and 1s (obstacles). It randomly generates a source and destination pixel location and then visualises the voronoi path to go from source to the destination.
```python
import cv2
import matplotlib.pyplot as plt
from voronoi import Voronoi
from generate import *

m,n = 100, 100
numOnes = 10
grid = generate_random_grid(m, n, numOnes)
voronoi = Voronoi(grid)
src = (random.randint(0, m-1), random.randint(0, n-1))
dest = (random.randint(0, m-1), random.randint(0, n-1))
print(f"{src} -> {dest}")
path = voronoi.path(src, dest)
print(path)
voronoi.visualise_path(path)
img = cv2.imread('path.png')
plt.imshow(img)
```

## Demo

### Simulator

#### Full Demo
[![Smart Summon - Smart Summon Demo on Simulator](https://img.youtube.com/vi/i-lD2U4WHMg/0.jpg)](https://youtu.be/i-lD2U4WHMg)

#### Lidar based map generation
[![Lidar based obstacle map generation](https://img.youtube.com/vi/RhOWXrxQTsM/0.jpg)](https://youtu.be/RhOWXrxQTsM)

### GEM Vehicle

#### Smart Summon - Building the obstacle map with Lidar

[![Smart Summon - Building the obstacle map with Lidar](https://img.youtube.com/vi/RybiM3pVGuM/0.jpg)](https://youtu.be/RybiM3pVGuM)

#### Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Inside View

[![Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Inside View](https://img.youtube.com/vi/vb3W54FPeVw/0.jpg)](https://youtu.be/vb3W54FPeVw)


#### Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Outside View

[![Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Outside View](https://img.youtube.com/vi/TV2HIjJie4Y/0.jpg)](https://youtu.be/TV2HIjJie4Y)

### Failures

#### Lidar failure on snowy day
[![Lidar failure on snowy day](https://img.youtube.com/vi/8132gOTaBkk/0.jpg)](https://youtu.be/8132gOTaBkk)