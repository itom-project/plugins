from itom import *
import numpy as np
import numpy.random as rand

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
