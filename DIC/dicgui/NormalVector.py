import matplotlib
matplotlib.use('module://mpl_itom.backend_itomagg',False)
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sig

nx = 100
phi = np.arange(-np.pi / 2.0, np.pi / 2.0 + np.pi/nx, np.pi/(nx - 1)) # last value is exclusive

lgrad =[1.0, 0.0, -1.0]
xvals = np.sin(phi)
yvals = np.cos(phi)
dx = sig.convolve(xvals, lgrad)
dy = sig.convolve(yvals, lgrad)
norm = np.sqrt(dx ** 2 + dy ** 2)
plt.figure()
f2 = plt.quiver(xvals, yvals, -dy/norm, (dx/norm))
plt.show() (
