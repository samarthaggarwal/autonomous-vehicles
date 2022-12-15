import bezier
import numpy as np
import matplotlib.pyplot as plt


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

    nodes = np.asfortranarray([points.transpose()[0], points.transpose()[1], ])
    curve = bezier.Curve(nodes, degree=len(points) - 1)
    s_vals = np.linspace(0.0, 1.0, nTimes)
    ans = curve.evaluate_multi(s_vals)

    return ans[0], ans[1]


if __name__ == "__main__":
    #     points = np.random.rand(nPoints,2)*200
    xpoints = [0, 50, 100, 150]
    ypoints = [0, 200, 0, 200]

    xvals, yvals = bezier_curve(np.array([xpoints, ypoints]).transpose(), nTimes=1000)
    print(len(xvals), len(yvals))
    plt.plot(xvals, yvals)
    plt.plot(xpoints, ypoints, "ro")
    for nr in range(len(xpoints)):
        plt.text(xpoints[nr], ypoints[nr], nr)

    plt.show()
