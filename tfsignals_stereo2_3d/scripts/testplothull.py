# Convex hull of a random set of points:

from scipy.spatial import ConvexHull
import numpy as np
points = np.random.rand(10, 2)   # 30 random points in 2-D
hull = ConvexHull(points)

# Plot it:

import matplotlib.pyplot as plt
plt.plot(points[:,0], points[:,1], 'o')
a = (points[:,0], points[:,1])
print("a1")
print(a)
for simplex in hull.simplices:
    plt.plot(points[simplex, 0], points[simplex, 1], 'k-')

# We could also have directly used the vertices of the hull, which
# for 2-D are guaranteed to be in counterclockwise order:






a = (points[hull.vertices,0], points[hull.vertices,1])
print("a2")
print(a)
plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
# plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
plt.show()