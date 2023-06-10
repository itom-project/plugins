import numpy as np

cx1 = 501
cy1 = 501
r = 10
dx = 59
dy = 0

obj = dataObject.zeros([1001,1001],dtype='float32')

for ny in np.arange(cy1 - r, cy1 + r + 1, 1):
    for nx in np.arange(cx1 - r, cx1 + r + 1, 1):
        #print((ny - cy1) ** 2 + (nx - cx1) ** 2)
        if (ny - cy1) ** 2 + (nx - cx1) ** 2 <= r**2:
            obj[int(ny), int(nx)] = 1

objOut = dataObject()
filter("fftw2D", obj, objOut)
filter("fftshift", objOut)

#plot(np.abs(obj))
if dy == 0:
    endy = objOut.size(0)
else:
    endy = -dy

if dx == 0:
    endx = objOut.size(1)
else:
    endx = -dx

objOut2 = np.abs(objOut[dy:, dx:])**2 + np.abs(objOut[:endy, :endx])**2
#objOut2 = np.abs(objOut[dy:, dx:]) + np.abs(objOut[:endy, :endx])
plot(objOut2)
plot(objOut2[cy1, :])
plot(np.abs(objOut2), "TwipOGLFigure")
