import numpy as np

# l = []
arr = [[5, 6, 7], [8, 9, 10], [11, 12, 13]]

# arr = [a for a in arr if a != [8,9,10]]

arr.remove([8,9,10])

# indices = np.where(arr > 10)

# points = list(zip(indices[0], indices[1]))

# points = [list(point) for point in zip(indices[0], indices[1])]
# # print(points)

# for i in points:
#     l.append(i)

# print(l)

# A = np.array([  [ 1,  2,  3,  4,  5],
#                 [ 6,  7,  8,  9, 10],
#                 [11, 12, 13, 14, 15],
#                 [16, 17, 18, 19, 20],
#                 [21, 22, 23, 24, 25]])

# x,y=3,3
# r=2
# print(A[max(x-r,0):x+r+1,max(y-r,0):y+r+1])

print(arr)

