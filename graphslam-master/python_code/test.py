import numpy as np

a = np.array([1, [2,3], [[1,7],[5,6]]])

# print(a[0])
# print(a[1][0])
# print(a[2])

print(np.shape(a))

c = 1
s = 0

T = np.array([
            [c,  -s,  5],
            [s,   c,  6],
            [0,   0,          1]
        ])

R = T[0:2 , 0:2]
print(R)

x = range(9)
y = np.reshape(x,(3,3))
d = np.array([
    [5,4,6],
    [0,1,2],
    [-1,1,1]
])
print(y)
for i in range(len(y)):
    y[1:3, i] = y[1:3, i] + d[1:3, i]
    print(y)

def optimize(num_iteration=10):

    '''(Done)
    Implement optimization to find a best solution for the graph.
    Optimization will stop when maximal iteration is reached.
    '''
    for i in range(num_iteration):
        print("No. %d iteration of optimization ..." %(i+1))
optimize()
# print(h)