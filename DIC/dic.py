from itom import *
from itom import ui
from itomUi import ItomUi
import os.path
import pickle
import numpy as np
import numpy.fft as fft
import __main__

class DIC():
    imageObj1 = dataObject()
    imageObj2 = dataObject()
    
    def __init__(self):
        pass
        
    def init(self):
        pass
    
    def loadImage(self, filename):
        if filename[-3] == "nef":
            try:
                resObj = dataObject()
                filter("importRAW", filename, resObj)
            except:
                print("error loading image")
        else:
            try:
                filter("loadAnyImage", resObj, filename)
            except:
                print("error loading image")
        loadImage = resObj
        
    def rigidBodyMovement(self):
        #fftObj1 = fft.fft2(self.imageObj1)
        #fftObj2 = fft.fft2(self.imageObj2)
        ccr = np.abs(fft.ifft2(fft.fft2(self.imageObj1) * fft.fft2(self.imageObj2).conj()))
        [sy, sx] = ccr.shape
        maxPos = np.argmax(ccr)
        offsetX = maxPos % sx
        offsetY = (maxPos - offsetX) / sx
        
        if (offsetX > sx / 2):
            offsetX = offsetX - sx
        if (offsetY > sy / 2):
            offsetY = offsetY - sy
        
        return (offsetX, offsetY)
    
if __name__ == "__main__":
    try: 
        myDIC = DIC()
        myDIC.init()
    except:
        raise