import matplotlib
matplotlib.use('module://mpl_itom.backend_itomagg',False)
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sig

rad = 2.0
dr = 0.01
h = 3.5
dh = -0.5
nx = 100
nh = 60
phi = np.arange(-np.pi / 2.0, np.pi / 2.0 + np.pi/nx, np.pi/(nx - 1)) # last value is exclusive
zvals = np.arange(0, 1.0 + 1.0 / nh, 1.0 / (nh - 1))

xvals = np.sin(phi)
yvals = np.cos(phi)
dyvals = np.cos(phi)

xMat = rad * np.ones([nh,1]) * xvals
zMat = (np.ones([nx,1]) * (h * zvals)).transpose()
yMat = rad * np.ones([nh,1]) * yvals
dyMat = rad * np.ones([nh,1]) * dyvals
#dyMat = np.ones([nh,1]) * dyvals
dxMat = (rad + dr) * np.ones([nh,1]) * xvals
dzMat = (np.ones([nx,1]) * ((h + dh) * zvals)).transpose()
#plt.figure()
#f1 = plt.quiver(xMat, zMat, dxMat, dzMat)

#https://elearning.physik.uni-frankfurt.de/data/FB13-PhysikOnline/lm_data/lm_8846/daten/teil_9/node29.htm
grad =[ [1.0, 0.0, -1.0]]
nVecxMat = sig.convolve2d(dxMat, grad, mode='same')
nVecyMat = sig.convolve2d(dyMat, grad, mode='same')
normMat = (nVecxMat ** 2 + nVecyMat ** 2)
nVecxMat = nVecxMat / normMat
nVecyMat = nVecyMat  / normMat
#proVecMatX = -nVecyMat * (nVecxMat * dxMat + nVecyMat * dyMat)
#proVecMatY = nVecxMat * (nVecxMat * dxMat + nVecyMat * dyMat)
proVecMatX = -nVecyMat * (nVecyMat * dxMat)
proVecMatY = nVecxMat * (nVecyMat * dxMat)

plt.figure()
#f1 = plt.quiver(xMat[1:-1,1:-1], zMat[1:-1,1:-1], proVecMatX[1:-1,1:-1], proVecMatY[1:-1,1:-1])
f1 = plt.quiver(xMat[2,1:-1], yMat[2,1:-1], proVecMatX[2,1:-1], proVecMatY[2,1:-1])


#lgrad =[1.0, 0.0, -1.0]
#xvals = rad * np.sin(phi)
#yvals = rad * np.cos(phi)
#dx = sig.convolve(xvals, lgrad)
#dy = sig.convolve(yvals, lgrad)
#norm = np.sqrt(dx ** 2 + dy ** 2)
#plt.figure()
#f2 = plt.quiver(xvals, yvals, -dy/norm, (dx/norm))

plt.show()
