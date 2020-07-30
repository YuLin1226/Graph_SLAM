import numpy as np
import csv
from math import atan2, sin, cos

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


a = np.array([
    [1, 2, 3,9,8],
    [1, 1, 1,7,7],
    [9, 8, 7,8,8]
])
b = np.eye(2)
a[0:2,0:2] = a[0:2,0:2] + b
# v = t2v(a)
# print(np.linalg.inv(a).dot(v))
print(len(a)-1 )




# with open('edge.csv') as f:
#     myCsv = csv.reader(f)
#     data = []
#     for row in myCsv:
#         data.append(row)
#     w = np.array(data)

# print(np.shape(w))
# print(np.size(w,1))