try:
    from .auxFuncs import *
except:
    from auxFuncs import *

#imageOrig = dataObject()
#imageOrig = SpeckleIntFkt(1024, 1024, 5, 2**8 - 1)
#imageOrig = dataObject.zeros([50,50])
#imageOrig[21:31, 11:26] = 255

distCoeff = dataObject.zeros([12,1], dtype='float64')
distCoeff[0, 0] = 0.9
distCoeff[1, 0] = 0.9
filenameBase = "dic_rb_x0{0:d}_y0{1:d}".format(int(distCoeff[0, 0] * 10), int(distCoeff[1, 0] * 10))
imageDef = dataObject()
filter("DICGenImages", imageOrig, imageDef, 0, distCoeff)
genImages = {'imageOrig' : imageOrig, 'imageDef' : imageDef}
#p=plot(imageOrig - imageDef)
#p[1]["zAxisInterval"] =  [-255, 255]

filename = filenameBase + ".idc"
saveIDC(filename, genImages)

fnameOrig = filenameBase + "_orig.mat"
saveMatlabMat(fnameOrig, imageOrig)
fnameDeform = filenameBase + "_def.mat"
saveMatlabMat(fnameDeform, imageDef)
