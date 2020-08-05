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


node = []
for i in range(10):
    node.append([
        i,
        1,
        5,
        i*4
    ])

dpose = np.array([
    [1,1,1,1,1,1,1,1,1,9],
    [2,1,1,1,1,1,1,1,1,1],
    [3,1,1,1,1,1,1,1,1,1]
])
for i_node in range(10):
    for n in range(len(dpose)):
        node[i_node][n+1] = node[i_node][n+1] + dpose[n, i_node]


x = [i[1] for i in node]
y = [i[2] for i in node]

print(x)
print(y)