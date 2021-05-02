import sys
import numpy as np
#from scipy import signal
import numpy.random as rand

sizeY = 512
sizeX = 512
size = 99

def SpeckleIntFkt(sizeX, sizeY, step, maxVal):
    SInt = np.zeros([sizeY, sizeX], dtype='float64')
    if step > 0:
        stepX = step
        stepY = step
        for y in range(0, sizeY, stepY):
            for x in range(0, sizeX, stepX):
                SInt[y : y + stepY, x : x + stepX] = rand.normal(0.0, 1.0)
        SInt = dataObject(SInt)
        filter("lowPassFilter", SInt, SInt, 5, 5)
        SInt = SInt - np.min(SInt)
        SInt = SInt / np.max(SInt) * maxVal
    return SInt

imOrig = SpeckleIntFkt(sizeX, sizeY, 6, 255)
distCoeff = dataObject.zeros([12, 1], dtype='float64')
distCoeff[0, 0] = 0.7
distCoeff[1, 0] = 0.4
imDef = dataObject()
filter("DICGenImages", imOrig, imDef, 0, distCoeff)

pos = dataObject().ones([1,4])
pos[0,0] = 25 # x0
pos[0,1] = 50 # y0
pos[0,2] = 20 # dx
pos[0,3] = 20 # dy

outVec = dataObject()
outVecCC = dataObject()
filter("DICDisplacement", imOrig, imDef, pos, outVec, outVecCC, initialGuessType=1)
sys.exit()

## Image generation test
imOrig = dataObject.zeros([sizeY, sizeX])
#imgOut = dataObject([sizeY, sizeX])
imDef = dataObject()
imOrig[10:57, 23:39] = 1
distCoeff = dataObject.zeros([12, 1], dtype='float64')
distCoeff[0, 0] = 0.7
distCoeff[1, 0] = 0.4
filter("DICGenImages", imOrig, imDef, 0, distCoeff)

pos = dataObject().ones([1,4])
pos[0,0] = 25 # x0
pos[0,1] = 50 # y0
pos[0,2] = 20 # dx
pos[0,3] = 20 # dy

outVec = dataObject()
outVecCC = dataObject()
filter("DICDisplacement", imOrig, imDef, pos, outVec, outVecCC, initialGuessType=1)
sys.exit()

## Image generation test

## Image correlation test
# generated image
inMat = np.array(np.arange(-sizeX / 2, sizeX / 2, 1)[np.newaxis])
inMat = dataObject(np.dot(inMat.transpose(), inMat))
inMat = inMat - min(inMat)
inMat = dataObject(np.floor(inMat / max(inMat) * 255))
imOrig = inMat[0 : sizeY - 2, 0 : sizeX - 2].copy()
imDef = inMat[1 : sizeY - 1, 0 : sizeX - 2].copy()

#loading image
#path = "E:/OneDrive/Documents/Mestrado/Dissertacao/UFAL/Algoritmos/Alterada/"
#imOrigName = "Img_red_or_1.bmp"
#imDefName = "Img_red_def_1.bmp"
#imOrig = dataObject()
#imDef = dataObject()
#filter("loadAnyImage", imOrig, path + imOrigName)
#filter("loadAnyImage", imDef, path + imDefName)

imOrig = imOrig.astype("float64")
imDef = imDef.astype("float64")

pos = dataObject().ones([1,4])
pos[0,0] = 60
pos[0,1] = 60
pos[0,2] = 30
pos[0,3] = 30

outVec = dataObject()

inpPos = dataObject([size, 2], dtype='float64')
inpPos[:,0] = np.arange(0.5, size + 0.5, 1.0)
#inpPos[:,1] = np.arange(0.5, size + 0.5, 1.0)
inpPos[:, 1] = 1

#filter("DICInterpolation", Mat1, outVec, inpPos, algorithm=1)
#plot(outVec[:, 0])
#plot(outVec[:, 1])
#plot(outVec[:, 2])
#plot(outVec)

outVecCC = dataObject()
filter("DICDisplacement", imOrig, imDef, pos, outVec, outVecCC, initialGuessType=0)

#kernel = np.array([[-0.5, 0, 0.5]], dtype="float64")
#grady = signal.convolve2d(np.array(imDef), kernel, mode='same', boundary="symm")
#gradx = signal.convolve2d(np.array(imDef), kernel.transpose(), mode='same', boundary="symm")
#plot(grad)