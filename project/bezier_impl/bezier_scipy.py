import numpy as np
from scipy.special import comb


def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """

    return comb(n, i) * (t ** (n - i)) * (1 - t) ** i


def get_bezier(points, nTimes=1000):
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

    deduplicatedPath = []
    prevPoint = None
    for i in range(len(xvals)):
        if prevPoint is None or prevPoint[0] != xvals[i] or prevPoint[1] != yvals[i]:
            deduplicatedPath.append([xvals[i], yvals[i]])
            prevPoint = [xvals[i], yvals[i]]
    # print(f"length: {len(ans)} -> {len(deduplicatedPath)}")
    return np.array(deduplicatedPath)
