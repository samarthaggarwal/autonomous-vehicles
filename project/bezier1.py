import numpy as np
from scipy.special import comb


def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """

    return comb(n, i) * (t ** (n - i)) * (1 - t) ** i


def bezier_curve(points, nTimes=1000):
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

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([bernstein_poly(i, nPoints - 1, t) for i in range(0, nPoints)])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)
    xvals = [int(x) for x in xvals]
    yvals = [int(y) for y in yvals]
    return np.array([xvals, yvals]).transpose()


if __name__ == "__main__":
    from matplotlib import pyplot as plt

    nPoints = 3
    #     points = np.random.rand(nPoints,2)*200
    xpoints = [0, 50, 100, 150]
    ypoints = [0, 200, 0, 200]

    bezier_points = bezier_curve(np.array([xpoints, ypoints]).transpose(), nTimes=1000)
    plt.plot(bezier_points[:, 0], bezier_points[:, 1])
    plt.plot(xpoints, ypoints, "ro")
    for nr in range(len(xpoints)):
        plt.text(xpoints[nr], ypoints[nr], nr)

    plt.show()
