import numpy as np


value=[2,1,0,3,8,0]
path_points_x=[1]
min=np.argmin(value)
dist_array=np.array([1])
for i in range(len(path_points_x)-1):
    dist_array[i] = path_points_x[i]

print "min's index is %d" %(min)
print dist_array

