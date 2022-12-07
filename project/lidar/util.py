import numpy as np


def transform_to_image_coordinates(x_points, y_points, resolution):
    """
    converts from coordinates in simulator's coordinate system to image coordinate system
    """

    # convert points to image coords with resolution
    x_img = np.floor(x_points / resolution).astype(np.int32)
    y_img = np.floor(-y_points / resolution).astype(np.int32)

    # shift coords to new original
    x_img += int(np.floor(20 / resolution))
    y_img += int(np.floor(10 / resolution))

    return x_img, y_img


def transform_to_taped_coordinates(x_img, y_img, resolution):
    """
    converts from image coordinate system to simulator's coordinate system
    """
    205, 90

    # shift coordinates to new origin
    x_img -= int(np.floor(20 / resolution))
    y_img -= int(np.floor(10 / resolution))

    # scale image coordinates by resolution
    x_points = x_img * resolution
    y_points = y_img * resolution

    return x_points, -1 * y_points

def transform_tape_to_simulator(x,y):
    return x-20.24, y-17.66

def transform_simulator_to_tape(x,y):
    return x+20.24, y+17.66

if __name__=='__main__':
    print(transform_to_taped_coordinates(205, 70, 0.2))
    print(transform_to_image_coordinates(21,-4,0.2))
