import numpy as np

types = ['uint8', 'float32', 'complex64', 'int16']

sizes = [(1,1), (2,1), (2,2), (2,3), (2,4), (2,5), (3,1), (3,2), (3,3), (3,4), (3,5), (67,201), (68,201), (67,202), (68,201)]

for type in types:
    a_ = dataObject([3,1000,1200], type)

    for s in sizes:
        a = a_[:,500:500+s[0],700:700+s[1]]

        for i in range(0,a.shape[1]//2):
            a[:,i,:] = i
            a[:,i+a.shape[0]//2,:] = 100+i
        if a.shape[0] % 2 > 0:
            a[:,-1,:] = 100+a.shape[1]//2+1

        for j in range(0,a.shape[2]//2):
            a[:,:,j] += j
            a[:,:,j+a.shape[2]//2] += ( 50 + j)
        if a.shape[1] % 2 > 0:
            a[:,:,-1] += (50+a.shape[2]//2+1)

        b = a.copy()
        filter("fftshift",b,0)
        filter("ifftshift",b,0)
        s1 = np.sum(b!=a)
        b = a.copy()
        filter("fftshift",b,1)
        filter("ifftshift",b,1)
        s2 = np.sum(b!=a)
        b = a.copy()
        filter("fftshift",b,-1)
        filter("ifftshift",b,-1)
        s3 = np.sum(b!=a)
        if s1 != 0 or s2 != 0 or s3 != 0:
            print("fftshift error")

print("test finished (if no previous prints the test was successful)")
