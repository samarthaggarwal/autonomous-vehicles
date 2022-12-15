import numpy as np
import matplotlib.pyplot as plt
# from bezier_impl.bezier_scipy import get_bezier
from bezier_impl.bezier_package import get_bezier


def bezier_curve(points, nTimes, voronoi, resolution, distance_from_obstacle=2.0):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1],
                 [2,3],
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """
    ans = get_bezier(points, nTimes)
    distance = int(distance_from_obstacle / resolution)
    count = 2
    # print(f"original points: {points}")
    while not voronoi.are_waypoints_clear(ans, distance):
        npoints = []
        for p in points:
            for c in range(count):
                npoints.append(p)
        if len(npoints) > 900:  # bezier can't fit curve on >1000 points
            break
        ans = get_bezier(np.array(npoints), nTimes)
        count += 1
    # print("final count: {count}")
    return ans

if __name__ == "__main__":
    #     points = np.random.rand(nPoints,2)*200
    # xpoints = [0, 50, 100, 150]
    # ypoints = [0, 200, 0, 200]

    points = np.array([[ 61, 123],
                        [ 80,  93],
                        [ 73, 150],
                        [ 60, 153],
                        [ 57, 164],
                        [ 72, 194],
                        [ 59, 220],
                        [ 73, 277],
                        [ 76, 281],
                        [100, 350]]
                    )
    xpoints, ypoints = points[:, 0], points[:, 1]

    # print(f"xpoints: {xpoints} \nypoints: {ypoints}")

    xpoints2 = []
    ypoints2 = []
    count = 102
    for i in range(len(xpoints)):
        for j in range(count):
            xpoints2.append(xpoints[i])
            ypoints2.append(ypoints[i])
    xpoints, ypoints = xpoints2, ypoints2

    bezier_points = get_bezier(np.array([xpoints, ypoints]).transpose(), nTimes=1000)
    # print(f"length of bezier: {len(bezier_points)}")
    plt.plot(bezier_points[:, 0], bezier_points[:, 1])
    plt.plot(xpoints, ypoints, "ro")
    for nr in range(len(xpoints)):
        plt.text(xpoints[nr], ypoints[nr], nr)

    plt.show()