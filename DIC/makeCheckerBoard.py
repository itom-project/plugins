import numpy as np

cbPx = 50
cbPy = 50
borderSize = 50
sizeX = 800
sizeY = 600
X0 = 600
Y0 = 200
#Pixel Pitch LG 22MP58VQ: 0.24795mm x 0.24795mm


checkerDObj = dataObject.ones([sizeY, sizeX]) * 255
col = 0
numSy = int((sizeY - 2 * borderSize) / cbPy)
numSx = int((sizeX - 2 * borderSize) / cbPx)
# number of squares must be odd
if numSy / 2 == np.floor(numSy / 2):
    numSy -= 1
if numSx / 2 == np.floor(numSx / 2):
    numSx -= 1

borderY = int((sizeY - numSy * cbPy) / 2)
borderX = int((sizeX - numSx * cbPx) / 2)
for ny in range(0, numSy):
    ys = int(borderY + ny * cbPy)
    ye = ys + cbPy + 1
    for nx in range(0, numSx):
        xs = int(borderX + nx * cbPx)
        xe = xs + cbPx + 1
        checkerDObj[ys:ye, xs:xe] = col
        if col > 0:
            col = 0
        else:
            col = 255
win = dataIO("DispWindow", x0=X0, y0=Y0, xsize=sizeX, ysize=sizeY)
win.setParam("dObj", checkerDObj)
