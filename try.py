import numpy as np
import random
import math
import cv2
from collections import Counter

from scipy import spatial

obstacle_coords = [ [0, 0], [0, 1], [1, 0], [1, 1], [2, 0], [2, 1] ]
tree = spatial.cKDTree( obstacle_coords )

angles = np.linspace(0, 180, num = 10)
z = np.array([3.0, 2.0, 0.7, 3.2, 1.0, 2.0, 1.5, 0.4, 0.3, 0.2])
x = [1,2,20]

angles_nuevos = angles + x[2]

cosenos = np.cos(angles_nuevos)
senos = np.sin(angles_nuevos)
z_x = np.multiply(cosenos, z)
z_y = np.multiply(senos, z)
z_x = z_x + x[0]
z_y = z_y + x[1]

tuplas = zip(z_x, z_y)
print(type(tuplas))

dist, point_id = tree.query( [tuplas] )
print(dist)
print(np.prod(dist))
print(type(dist))

def gaussiana(valor):
    des = 0.2
    formula = (1/math.sqrt(2*math.pi*(des**2)))*math.exp(-(valor**2)/(2*(des**2)))
    print(formula)
    return formula

gauss_vect = np.vectorize(gaussiana)

print(gauss_vect(dist[0]))
print(type(gauss_vect(dist[0])))

