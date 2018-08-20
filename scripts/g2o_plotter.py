import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy
import IPython

filePath = sys.argv[1]

g2oFile  = open(filePath, 'r')




points = numpy.empty((0, 3), float)
cameras = numpy.empty((0, 3), float)
projections = numpy.empty((0, 2), int)

line = g2oFile.readline()
while line:
    line = g2oFile.readline()
    data = line.split(' ')
    if(data[0] == 'VERTEX_SE3:EXPMAP'): # Camera
        cameras = numpy.append(cameras,numpy.array([[float(data[2]), float(data[3]), float(data[4])]]),axis=0)
    if(data[0] == 'VERTEX_XYZ'): # point
        points =numpy.append(points,numpy.array([[float(data[2]), float(data[3]), float(data[4])]]),axis=0)
    if(data[0] == 'EDGE_PROJECT_XYZ2UV:EXPMAP'): # projection   
        projections = numpy.append(projections,numpy.array([[int(data[1]), int(data[2])]]),axis=0)
    
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter3D(cameras[:,0], cameras[:,1], cameras[:,2], c='r', marker='o') # cameras
ax.scatter3D(points[:,0],points[:,1],points[:,2], c='b', marker='o') # Points

# lines

plt.show()
