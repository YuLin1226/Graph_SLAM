import numpy as np
import csv

a = np.array([1, 2, 3])
k = float(5)
print(type(k),k)
print(np.shape(a))
c = []
b = [ a, [], 5 ]
c = c.append(b)
print(c)

# with open('edge.csv') as f:
#     myCsv = csv.reader(f)
#     data = []
#     for row in myCsv:
#         data.append(row)
#     w = np.array(data)

# print(np.shape(w))
# print(np.size(w,1))