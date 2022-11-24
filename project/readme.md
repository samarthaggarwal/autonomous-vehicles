# Project: Smart Summon

As part of the final course project, we program the ability to _smartly summon_ the vehicle.

The vehicle knows its current location (localisation using Camera (RTabMap) or GPS). The user enters destination coordinates using the command line. The car then goes from source (current location) to destination and avoids any obstacle on the way.

---
## Test Instructions

Generate a random grid of obstacles and see its voronoi diagram. \
```python voronoi.py TestVoronoi.test_random```

