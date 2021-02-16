import numpy as np
import math
def signe(speed1,speed2):
    return (speed1[0]*speed2[0]>=0 and speed1[1]*speed2[1]>=0)

     
a=np.dot(np.array((-195.4768824306473, 208.8372763300108)),np.array((736.613394530771, 3.4097156771718375)))

def distance(p,q):
    return math.sqrt((p[0]-q[0])**2 +(p[1]-q[1])**2)
print(distance((346.76409912109375, 245.25283813476562),[262.66351318, 246.55212402]))
from scipy.spatial import distance as dist
L1=np.array([[7,2],[8,1],[2,4]], dtype="float")
L2=np.array([[1,2],[2,5]], dtype="float")
D = dist.cdist(L1, L2)
rows = D.min(axis=0).argsort()
cols = D.argmin(axis=1)[rows]
print(D)
print(rows)
