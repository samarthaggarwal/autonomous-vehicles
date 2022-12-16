# Project: Smart Summon

As part of the final course project, we program the ability to _smartly summon_ the vehicle.

The vehicle knows its current location (localisation using Camera-based SLAM or GPS). The user enters destination coordinates using the command line. The car then goes from source (current location) to destination and avoids any obstacle on the way.

## Test Instructions

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

### Smart Summon - Building the obstacle map with Lidar

[![Smart Summon - Building the obstacle map with Lidar](https://img.youtube.com/vi/RybiM3pVGuM/0.jpg)](https://youtu.be/RybiM3pVGuM)

### Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Inside View

[![Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Inside View](https://img.youtube.com/vi/vb3W54FPeVw/0.jpg)](https://youtu.be/vb3W54FPeVw)


### Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Outside View

[![Smart Summon with Obstacle Detection (Lidar) and Dynamic Path Planning (Voronoi) - Outside View](https://img.youtube.com/vi/TV2HIjJie4Y/0.jpg)](https://youtu.be/TV2HIjJie4Y)
