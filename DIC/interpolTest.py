import numpy as np
import time
from scipy.interpolate import *

sx = 903
sy = 1070
#sx = 100
#sy = 100
x0 = 20
y0 = 20
dx = 21
dy = 16
#dx = 901
#dy = 903
ox = 23
oy = 22
dox = 13
doy = 10

useCuda = 0
interpolAlgo = 1
numRuns = 1

#i1 = dataObject.zeros([sy, sx], dtype='float64')
i1 = dataObject.zeros([sy, sx], dtype='float32')
i1[oy:oy+doy, ox:ox+dox] = 128

i2 = dataObject()
pts = dataObject.zeros([dy * dx, 2], dtype='float32')

for y in range(0, dy):
    for x in range(0, dx):
        pts[y * dx + x, 0] = x + x0
        pts[y * dx + x, 1] = y + y0

pts[:, 1] += 0.5
pts[:, 0] -= 0.5
pts = dataObject(pts)

start = time.perf_counter()
if useCuda:
    if numRuns > 2:
        filter("DICInterpolation", i1, i2, pts, algorithm=interpolAlgo, withDer=1, useCuda=useCuda, keepImage=-1)
        numRuns = numRuns - 2
    elif numRuns == 2:
        filter("DICInterpolation", i1, i2, pts, algorithm=interpolAlgo, withDer=1, useCuda=useCuda, keepImage=-1)
        filter("DICInterpolation", i1, i2, pts, algorithm=interpolAlgo, withDer=1, useCuda=useCuda, keepImage=0)
        numRuns = 0
    else:
        filter("DICInterpolation", i1, i2, pts, algorithm=interpolAlgo, withDer=1, useCuda=useCuda, keepImage=-1)
        numRuns = 0
        
for n in range(0, numRuns):
    filter("DICInterpolation", i1, i2, pts, algorithm=interpolAlgo, withDer=1, useCuda=useCuda, keepImage=1)
    
if useCuda:
    filter("DICInterpolation", i1, i2, pts, algorithm=interpolAlgo, withDer=1, useCuda=useCuda, keepImage=0)
    
stop = time.perf_counter()
print("dt: ", stop - start)
plot(i2[:,0].reshape([dy, dx]))
plot(i2[:,1].reshape([dy, dx]))
plot(i2[:,2].reshape([dy, dx]))

#xvec = np.arange(0, sx)
#yvec = np.arange(0, sy)
#pyip = interp2d(xvec, yvec, i1, kind='quintic')
##xvec = xvec + 0.5;
#yvec = yvec + 0.5;
#i3 = pyip(xvec, yvec)
#plot(i3.reshape([sy,sx]))