import numpy as np

l = []
arr = np.array([[5, 6, 7], [8, 9, 10], [11, 12, 13]])

indices = np.where(arr > 10)

points = list(zip(indices[0], indices[1]))

points = [list(point) for point in zip(indices[0], indices[1])]
# print(points)

for i in points:
    l.append(i)

print(l)