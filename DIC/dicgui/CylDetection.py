import numpy as np
import cv2 as cv

#tstImg = np.zeros([200, 305], dtype='uint8')
#tstImg[32:41, 65:79] = 255
tstImg = np.array(data["imageOrig"])
tstImg = tstImg - np.min(tstImg)
tstImg = tstImg / np.max(tstImg) * 255
#tstImg = cv.bilateralFilter(tstImg, -1, 13, 13)
tstImg = tstImg.astype("uint8")
tstEdges = cv.Canny(tstImg, 180, 200)
#tstEdges = cv.bilateralFilter(tstEdges, -1, 9, 9)
#tstEdges = cv.fastNlMeansDenoising (tstEdges)
#tstEdges = cv.medianBlur(tstEdges, 3)

#lines = cv.HoughLines(tstEdges, 1, np.pi/180.0, 400)
#shapes = set()
#for n in range(0, lines.shape[0]):
    #y0 = 0
    #y1 = tstImg.shape[0]
    #xp = np.cos(lines[n, 0, 1]) * lines[n, 0, 0]
    #yp = np.sin(lines[n, 0, 1]) * lines[n, 0, 0]
    #x0 = xp - np.sin(lines[n, 0, 1]) * (y0 - yp)
    #x1 = xp + np.sin(lines[n, 0, 1]) * (y1 - yp)
    #shapes.add(shape(4, [x0, y0], [x1, y1]))

lines = cv.HoughLinesP(tstEdges, 1, np.pi/180.0, 300)
shapes = set()
for n in range(0, lines.shape[0]):
    x0 = lines[n, 0, 0]
    x1 = lines[n, 0, 2]
    y0 = lines[n, 0, 1]
    y1 = lines[n, 0, 3]
    shapes.add(shape(4, [x0, y0], [x1, y1]))
plt = plot(tstImg)
plt[1].call("setGeometricShapes", list(shapes))