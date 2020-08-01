import numpy as np
import csv
from math import atan2, sin, cos
from scipy import sparse
from scipy.sparse.linalg import inv

def t2v(T):
    
    '''(Done)
    homogeneous transformation to vector
    '''
    v = np.zeros((3,1))
    v[0:2, 0] = T[0:2, 2]
    v[2, 0] = atan2(T[1,0], T[0,0])
    return v


def v2t(vector):
    
    '''(Done)
    vector to homogeneous transformation
    From local to global
    [              |
            Rotaion  | Translation
        _____________|____________
            0   |   0 |      1
    ]
    '''
    c = cos(vector[2])
    s = sin(vector[2])
    x = float(vector[0])
    y = float(vector[1])
    T = np.array([
        [c,  -s,  x],
        [s,   c,  y],
        [0,   0,  1]
    ])
    return T

a = []

for i in range(5):
    a.append([
                int(i),
                float(i+1),
                float(i+2),
                float(i+3)
            ])

b = [i[0] for i in a]
print(a)
print(b)