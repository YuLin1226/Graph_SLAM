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
print( R.dot(R).dot(R) )
def id2ind(id):
    '''(Done)
    Converts id to indices in H and b
    '''
    return ,

x= id2ind(5)
print(x)