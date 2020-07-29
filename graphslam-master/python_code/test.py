import numpy as np
import csv

a = np.array([1, [2,3], [[1,7],[5,6]]])

print(np.shape(a))

c = 1
s = 0

T = np.array([
            [c,  -s,  5],
            [s,   c,  6],
            [0,   0,          1]
        ])


print(type(T))

def no():
    print("Do nothing")
    return

no()


# with open('node.csv') as f:
#     myCsv = csv.reader(f)
#     # headers = next(myCsv)
#     # print(type(myCsv))
#     data = []
#     for row in myCsv:
#         # print(type(row))
#         data.append(row)
#     w = np.array(data)
#     print(np.shape(w))