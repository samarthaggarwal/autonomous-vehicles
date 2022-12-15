import numpy as np
import bezier


def get_bezier(points, nTimes=1000):
    nodes = np.asfortranarray([points.transpose()[0], points.transpose()[1], ])
    # print(f"Nodes: {nodes} | len: {len(nodes[0])} ")
    curve = bezier.Curve(nodes, degree=len(points) - 1)
    s_vals = np.linspace(0.0, 1.0, nTimes)
    ans = curve.evaluate_multi(s_vals).transpose().astype(int)
    deduplicatedPath = []
    prevPoint = None
    for i, point in enumerate(ans):
        if prevPoint is None or prevPoint[0]!=point[0] or prevPoint[1]!=point[1]:
            deduplicatedPath.append(point)
            prevPoint = point
    # print(f"length: {len(ans)} -> {len(deduplicatedPath)}")
    return np.array(deduplicatedPath)






