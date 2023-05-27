# coding=iso-8859-15
'''
By twipOS 2016.
'''

#The algorithm is based on Python 3.2 and must be used with "itom" ==> https://github.com/itom-project/itom
#Import basic function and packages used by this file
#See python help for more details
import cv2
import numpy as np
import pickle
import json
import inspect
import weakref
import os
from glob import glob
import time
import datetime
from itom import *
from itom import userIsDeveloper
from itom import userIsAdmin
from itom import dataObject
from itomUi import ItomUi
from itom import ui

try:
    from .dicgui_ui import dicGuiUi
except:
    from dicgui_ui import dicGuiUi

reloadModules = 1


class dicGui(ItomUi):
    '''
    In the class dicGui all operations to XXX are contained.
    '''

    def __init__(self, modulePath = None):
        '''
        Internal constructor of the image processing system.

        Parameters
        -----------
        modulePath:
            Path where the system is located, if no parameter is giving the files has to be in the working directory

        Returns
        -------
        None

        Notes
        -----
        This function initializes the image processing system.
        '''

        self.softwareVersion = "0.0.1"

        if(modulePath is None):
            self.modulePath = getCurrentPath()+"/"
            sys.path.append( self.modulePath )
        else:
            sys.path.append( modulePath )
            self.modulePath = modulePath

        self.GUI = None

        self.startDir = ""
        self.timer = None

        self.cfg = {}
        self.eval = {}
        self.data = {}
        self.eval["fromFolder"] = 0
        self.cfg["RVecs"] = None
        self.cfg["TVecs"] = None
        self.cfg["CamMat"] = None
        self.cfg["DistCoeff"] = None
        self.cfg["CamPixelPitch"] = None
        self.cfg["CamAperture"] = None
        self.cfg["cam.calibFile"] = ""
        self.cfg["serport"] = 3
        self.cfg["rangeVals"] = [0.003, 0.012, 0.125, 0.5]
        self.cfg["ranges"] = [2, 2, 2, 2]
        self.cfg["bridges"] = [0, 0, 0, 0]
        self.cfg["scales"] = [1, 1, 1, 1]
        self.cfg["measTypes"] = ["strain", "strain", "strain", "strain"]
        self.cfg["freq"] = 1200
        self.cfg["samples"] = 32
        self.cfg["channels"] = []
        self.cfg["enableCam"] = 0
        self.cfg["enableSpider"] = 1
        self.cfg["spiderDelay"] = 1
        self.cfg["camDelay"] = 10
        self.cfg["downloadImgs"] = False
        self.cfg["imgsPath"] = ""
        self.data["imageOrig"] = None
        self.data["imageDef"] = None
        self.data["displ"] = None
        self.data["displCC"] = None
        self.data["exxmat"] = None
        self.data["eyymat"] = None
        self.data["exymat"] = None
        #self.data["cellsX"] = None
        #self.data["cellsY"] = None
        self.data["nodes"] = None
        self.data["nodesX"] = 0
        self.data["nodesY"] = 0
        self.data["displResults"] = {}
        #self.data["cellsXVecs"] = {}
        #self.data["cellsYVecs"] = {}
        #self.data["displVecs"] = {}
        self.data["deformVecs"] = {}

        self.liveData = dataObject()
        self.seriesData = dataObject()
        self.resultObj = dataObject()
        self.resultDic = {}
        self.hasLiveData = 0
        self.numMeas = 0
        self.numImg = 0
        self.preAllocSize  = 8

        self.downloadImgsMeas = False
        self.imgsPathMeas = ""
        self.timer = None
        self.timerCam = None
        self.tmpImg = dataObject()

        self.isMeasuring = False

        self.cam = None
        self.ser = None
        self.hbm = None
        #self.imageOrig = None
        #self.imageDef = None
        self.loadedCalib = None

        self.loadConfig()

    def __del__(self):
        '''
        Deletes all objects

        '''
        try:
            if not self.GUI is None and self.GUI.gui.isVisible():
                self.GUI.gui.hide()
        except:
            pass

        try:
            removeButton("DIC", "showGUI")
        except:
            print("\n deleting button bar failed")

        self.saveConfig()

    def deleteUI(self):
        if (self.timer != None):
            self.timer.stop()
            time.sleep(1)
            self.timer = None
        if (self.timerCam != None):
            self.timerCam.stop()
            self.timerCam = None
        if self.cam != None:
            try:
                try:
                    self.cam.stopDevice()
                except:
                    pass
                del self.cam
                self.cam = None
            except:
                pass
        if self.GUI != None:
            del self.GUI
        if (self.hbm != None):
            try:
                del self.hbm
                self.hbm = None
            except:
                pass
        if (self.ser != None):
            try:
                del self.ser
                self.ser = None
            except:
                pass

    def init(self):
        '''
        Set up values of the GUI-elementes.

        Parameters
        -----------
        None

        Returns
        -------
        None

        Notes
        -----
        This function initializes the gui elements and set the values to start the image processing.
        '''
        pass

    def saveConfig(self):
        try:
            file = open(self.modulePath + 'systemConfig.json', mode ='w')
            saveDict = {}
            saveDict["cfg"] = {}
            for key in self.cfg:
                NoneType = type(None)
                if not isinstance(self.cfg[key], NoneType):
                    if type(self.cfg[key]) == dataObject or isinstance(self.cfg[key], np.ndarray):
                        saveDict["cfg"][key] = np.array(self.cfg[key]).tolist()
                    elif isinstance(self.cfg[key], list):
                        saveDict["cfg"][key] = self.cfg[key]
                    else:
                        saveDict["cfg"][key] = self.cfg[key]
            #saveDict["cfg"] = self.cfg
            json.dump(saveDict, file, indent = 1)
            file.close()
        except:
            ui.msgCritical("Error", "Failed saving system parameters.")

    def loadConfig(self):
        #try to open file and overwrite default values with json values
        try:
            jfile = open(self.modulePath + 'systemConfig.json', mode ='r')
            jsonFile = json.load(jfile)
            jfile.close()
        except:
            jsonFile = {}

        if(type(jsonFile) == dict and "cfg" in jsonFile.keys() and jsonFile["cfg"] is not None):
            cfgDict = jsonFile.get('cfg')
            for key in cfgDict:
                if type(cfgDict[key]) == list:
                    tmpArr = np.array(cfgDict[key])
                    if tmpArr.dtype == '<U6':
                        self.cfg.update({key : cfgDict[key]})
                    else:
                        self.cfg.update({key : np.array(cfgDict[key])})
                else:
                    self.cfg.update({key : cfgDict[key]})
            #self.cfg.update(jsonFile.get('cfg'))

    def show(self, modalLevel = 0):
        '''
        Internal constructor of the GUI class.

        Parameters
        -----------
        sys: {bvSystem}
            a weak reference to the image processing system
        systemPath:
            Path where the system is located, if no parameter is giving the files has to be in the working directory

        Returns
        -------
        None

        Notes
        -----
        This function loads the ui-dialog and takes a weak reference on the image processing system.
        '''

        self.GUI = dicGuiUi(weakref.ref(self), self.modulePath, )
        self.GUI.init()
        try:
            removeButton("DIC", "showGUI")
        except:
            self.buttonHandle = None

        self.buttonHandle = addButton("DIC","showGUI","dicgui.show()", "ui/dicIcon.png")
        self.GUI.show(modalLevel)

    def loadImageDict(self, dictPath, openItem = None):
        tmpImg = loadIDC(dictPath)
        for key in tmpImg:
            if key == "raw":
                self.imageRaw = tempImg[key]
            elif key == "input":
                self.imageInput = tempImg[key]
            elif key == "eval":
                self.imageEval = tempImg[key]

    def findCBPoints(self, img, numPts):
        try:
            img2 = img.normalize(0, 255).astype('uint8')
            #img2[img2 < 20] = 0
            #img3 = cv2.threshold(np.array(img2), 128, 1, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
            #img2 = img2.mul(dataObject(img3[1]))
            #img3 = cv2.threshold(np.array(img2), 128, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)[1]
            #res = filter("cvFindChessboardCorners", dataObject(img3), numPts, pts, flags=11)
            imgBCBD = cv2.GaussianBlur(np.array(img2), (7, 7), 2)
            tmpRes = cv2.findChessboardCorners(imgBCBD, tuple(numPts), \
                cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_FAST_CHECK|cv2.CALIB_CB_NORMALIZE_IMAGE)

            res = tmpRes[0]
            # do corner refinement as suggested in opencv calibration example
            if res == 1:
                #filter("cvCornerSubPix", img, pts, (5, 5), maxCount = 30)
                pts = dataObject(cv2.cornerSubPix(np.array(img2), tmpRes[1], (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))).squeeze()
            else:
                pts = dataObject()
                res = -1
        except:
            res = -1
            pts = dataObject()
        return [res, pts]

    def detNearestPt(self, Pt, pts, nptsInLine, dst):
        '''
        Insert descritopn here

        Parameters
        -----------


        Returns
        -------


        Notes
        -----

        '''
        d1 = np.array(Pt) - pts[0, :]
        d1 = d1.dot(d1.transpose())
        d2 = np.array(Pt) - pts[nptsInLine - 1, :]
        d2 = d2.dot(d2.transpose())
        d3 = np.array(Pt) - pts[pts.size(0) - nptsInLine, :]
        d3 = d3.dot(d3.transpose())
        d4 = np.array(Pt) - pts[pts.size(0) - 1, :]
        d4 = d4.dot(d4.transpose())

        if d1 <= dst[0]:
            ptRet = 0
        elif d2 <= dst[1]:
            ptRet = nptsInLine - 1
        elif d3 <= dst[2]:
            ptRet = pts.size(0) - nptsInLine
        elif d4 <= dst[3]:
            ptRet = pts.size(0) - 1
        else:
            ptRet = -1

        return ptRet

    def detUsableImgRange(self, img1, img2):
        diffOfs = np.zeros([2, 2], dtype='int')
        diffSizes = np.zeros([2, 2], dtype='int')
        if isinstance(img1, dataObject) and isinstance(img2, dataObject):
            diffOfs[:, 0] =  img1.axisOffsets
            diffOfs[:, 1] =  img2.axisOffsets
            diffOfs[0, :] = diffOfs[0, :] - min(diffOfs[0, :])
            diffOfs[1, :] = diffOfs[1, :] - min(diffOfs[1, :])
            diffSizes[:, 0] = img1.size()
            diffSizes[:, 1] = img2.size()
            diffSizes = diffSizes - diffOfs
            diffSizes[0, :] = min(diffSizes[0, :]) + diffOfs[0, :]
            diffSizes[1, :] = min(diffSizes[1, :]) + diffOfs[1, :]
        return {"dO" : diffOfs, "dS" : diffSizes}

    def defineCoordSys(self, img, pts, markerSize):
        '''
        Insert descritopn here

        Parameters
        -----------


        Returns
        -------


        Notes
        -----

        '''
        self.GUI.gui.pltCalibration.call("clearGeometricShapes")
        self.GUI.gui.pltCalibration["source"] = img
        shapes = set()
        numPts = pts.size(0)
        for np in range(0, numPts):
            pt = pts[np, :]
            shapes.add(shape(2, pt[:, :2]))
            #self.GUI.gui.pltCalibration.call("addGeometricShape", shape(2, pt[:, :2]))
        self.GUI.gui.pltCalibration.call("setGeometricShapes", list(shapes))
        #self.GUI.gui.pltCalibration["geometryModificationModes"] = 2
        #self.GUI.gui.pltCalibration.call("userInteractionStart", 4, 1, 2)
        #self.GUI.gui.pltCalibration.connect("userInteractionDone(int,bool,QVector<ito::Shape>)",self.finishedInput)
        #shapes = list(self.GUI.gui.pltCalibration["geometricShapes"])

        found = 0
        while not found:
            shapes = plotItem(self.GUI.gui.pltCalibration).drawAndPickElements(shape.Line, 2)
            if len(shapes) != 2:
                return
            dv1 = pts[0, :] - pts[1, :]
            dst1 = dv1 * dv1.trans()
            dstl = dst1
            nptsInLine = 1
            isInLine = 1
            while isInLine:
                dvn = pts[nptsInLine, :] - pts[nptsInLine + 1, :]
                dstn = dvn * dvn.trans()
                nptsInLine = nptsInLine + 1
                if dstn[0, 0] > 2 * dstl[0, 0]:
                    isInLine = 0
                else:
                    dstl[0, 0] = dstn[0, 0]
            # calculate comparison distances for corner points, using always the four corner pointSize
            # and its next (x and y) neighbours
            dst = [0, 0, 0, 0]
            dv1 = pts[1, :] - pts[0, :]
            dst1 = dv1 * dv1.trans()
            dv1 = pts[0, :] - pts[nptsInLine, :]
            dst[0] = 0.7 * min(dst1[0,0], (dv1 * dv1.trans())[0,0])
            dv2 = pts[nptsInLine - 2, :] - pts[nptsInLine - 1, :]
            dst2 = dv2 * dv2.trans()
            dv2 = pts[nptsInLine + nptsInLine - 1, :] - pts[nptsInLine - 1, :]
            dst[1] = 0.7 * min(dst2[0,0], (dv2 * dv2.trans())[0,0])
            dv3 = pts[pts.size(0) - nptsInLine, :] - pts[pts.size(0) - 2 * nptsInLine, :]
            dst3 = dv3 * dv3.trans()
            dv3 = pts[pts.size(0) - nptsInLine, :] - pts[pts.size(0) - nptsInLine + 1, :]
            dst[2] = 0.7 * min(dst3[0,0], (dv3 * dv3.trans())[0,0])
            dv4 = pts[pts.size(0) - 1, :] - pts[pts.size(0) - nptsInLine - 1, :]
            dst4 = dv4 * dv4.trans()
            dv4 = pts[pts.size(0) - 1, :] - pts[pts.size(0) - 2, :]
            dst[3] = 0.7 * min(dst4[0,0], (dv4 * dv4.trans())[0,0])

            ptY1 = self.detNearestPt(shapes[-2].point1, pts, nptsInLine, dst)
            ptX0 = self.detNearestPt(shapes[-2].point2, pts, nptsInLine, dst)
            ptX1 = self.detNearestPt(shapes[-1].point2, pts, nptsInLine, dst)

            if ptY1 != -1 and ptX0 != -1 and ptX1 != -1 \
            and ptY1 != ptX1 and ptY1 != ptX0 and ptX1 != ptX0:
                found = 1

                npts = dataObject.zeros([pts.size(0), 5], dtype=pts.dtype)
                npt = 0
                numLines = int(pts.size(0) / nptsInLine)

                if abs(ptX1 - ptX0) < abs(ptY1 - ptX0):
                    colwise = 1
                else:
                    colwise = 0

                if ptX1 > ptX0:
                    xstart = 0
                    xfact = 1
                else:
                    xstart = nptsInLine - 1
                    xfact = -1
                if ptY1 < ptX0:
                    ystart = numLines - 1
                    yfact = -1
                else:
                    ystart = 0
                    yfact = 1

                if colwise:
                    for py in range(0, numLines):
                        for px in range(0, nptsInLine):
                            npts[npt, :2] = pts[npt, :]
                            npts[npt, 2] = (xstart + xfact * px) * markerSize[0]
                            npts[npt, 3] = (ystart + yfact * py) * markerSize[1]
                            npt = npt + 1
                else:
                    for px in range(0, nptsInLine):
                        for py in range(0, numLines):
                            npts[npt, :2] = pts[npt, :]
                            npts[npt, 2] = (xstart + xfact * px) * markerSize[0]
                            npts[npt, 3] = (ystart + yfact * py) * markerSize[1]
                            npt = npt + 1

                return npts
            else:
                ptShapes = self.GUI.gui.pltCalibration["geometricShapes"]
                self.GUI.gui.pltCalibration["geometricShapes"] = ptShapes[:-2]
                ui.msgCritical("Error", "Could not detect chessboard orientation, try marking again or press ESC to abort!")

    def showQuivers(self, posAndSizes):
        shapes = self.GUI.gui.pltDisplacement["geometricShapes"]
        newShapes = set()

        # maintain marked fields but remove old quivers
        if not shapes is None and len(shapes) > 0:
            for ns in range(0, len(shapes)):
                if shapes[ns].type != 8:
                    continue
                newShapes.add(shapes[ns])

        # add quivers
        if (posAndSizes.dims > 0):
            for nq in range(0, posAndSizes.size(0)):
                newShapes.add(shape(4, [posAndSizes[nq, 0], posAndSizes[nq, 1]], \
                    [posAndSizes[nq, 2], posAndSizes[nq, 3]]))

        # allow lines - just for displaying
        self.GUI.gui.pltDisplacement["allowedGeometricShapes"] = 28
        self.GUI.gui.pltDisplacement.call("setGeometricShapes", list(newShapes))
        self.GUI.gui.pltDisplacement["allowedGeometricShapes"] = 24

    def range2CB(self, range, guiCB):
        if (range == 0):
            guiCB["currentText"] = "3 mV/V"
        elif (range == 1):
            guiCB["currentText"] = "12 mV/V"
        elif (range == 2):
            guiCB["currentText"] = "125 mV/V"
        else:
            guiCB["currentText"] = "500 mV/V"

    def CB2range(self, guiCB):
        if (guiCB["currentText"] == "3 mV/V"):
            range = 0
        elif (guiCB["currentText"] == "12 mV/V"):
            range = 1
        elif (guiCB["currentText"] == "125 mV/V"):
            range = 2
        elif (guiCB["currentText"] == "500 mV/V"):
            range = 3
        else:
            range = 3
        return range

    def CB2bridge(self, guiCB):
        if guiCB["currentText"] == "Full":
            bridge = 0
        elif guiCB["currentText"] == "Half":
            bridge = 1
        elif guiCB["currentText"] == "Quarter":
            bridge = 2
        else:
            bridge = 0
        return bridge

    def initHBM(self):
        serialErr = 0
        if (self.hbm != None):
            del self.hbm
            self.hbm = None
        if (self.ser != None):
            del self.ser
            self.set = None
        if (self.cam != None):
            del self.cam
            self.cam = None

        retval = 0
        retStr = ""
        if self.cfg["enableSpider"] == 1:
            try:
                self.ser = dataIO("serialIO", self.cfg["serport"], 9600, "\r\n")
            except:
                ui.msgCritical("ERROR", "Serial port not open or HBM not initialized")
                retStr = 'error opening default com-port, need to initialize spider via gui-button'
                print(retStr)
                #self.gui.leStatus["text"] = errStr
                serialErr = 1
                self.ser = None
                retval = retval -1
            try:
                if retval == 0:
                    self.hbm = dataIO("HBMSpider8", self.ser)
                    #self.gui.leStatus["text"] = self.hbm.getParam("status")
                    retStr = self.hbm.getParam("status")
            except:
                if serialErr == 0:
                    retStr = 'error opening default com-port, need to initialize spider via gui-button'
                    print(retStr)
                    #self.gui.leStatus["text"] = retStr
                retval = retval -2
                self.hbm = None

        if self.cfg["enableCam"] == 1:
            try:
                self.cam = dataIO("DslrRemote2")
            except:
                ui.msgCritical("ERROR", "Camera could not be opened")
                retStr = 'error opening camera, all camera operations disabled'
                print(retStr)
                self.cam = None
                retval = retval -4
        return [retval, retStr]

    def initCam(self):
        retval = 0
        if self.cfg["enableCam"] == 1:
            try:
                self.cam = dataIO("DslrRemote2")
            except:
                ui.msgCritical("ERROR", "Camera could not be opened")
                retStr = 'error opening camera, all camera operations disabled'
                print(retStr)
                self.cam = None
                retval = retval -4

    def prepMeasure(self):
        if self.hbm is None:
            [retval, retstr] = self.initHBM()
            if retval != 0:
                ui.msgCritical("Error", "Could not initialize Spider!\nCheck settings and try again!")
                print("Error: Could not initialize Spider! Check settings and try again!")
                return

        if (self.hbm != None and self.GUI.gui.cbEnableSpider["checked"] == True):
            # if there are alreay measurement values available measurement was probably
            # only paused. so just restart device
            if self.numMeas == 0:
                self.hbm.stopDevice()
                self.hbm.getParam("reset")
                numCha = 0
                #self.cfg["channels"].clear()
                self.cfg["channels"] = []
                if self.GUI.gui.cbChan4["checked"]:
                    self.cfg["channels"].append(4)
                    numCha += 1
                if self.GUI.gui.cbChan5["checked"]:
                    self.cfg["channels"].append(5)
                    numCha += 1
                if self.GUI.gui.cbChan6["checked"]:
                    self.cfg["channels"].append(6)
                    numCha += 1
                if self.GUI.gui.cbChan7["checked"]:
                    self.cfg["channels"].append(7)
                    numCha += 1

                self.resultObj = dataObject([numCha + 1, self.preAllocSize],dtype='float64')

                for nc in range(0, len(self.cfg["channels"])):
                    self.hbm.setParam("aiChParams", str(self.cfg["channels"][nc]) + "," \
                        + str(self.cfg["bridges"][self.cfg["channels"][nc] - 4]) + "," \
                        + str(self.cfg["ranges"][self.cfg["channels"][nc] - 4]))
                self.cfg["scales"][0] = self.GUI.gui.dsbScale4["value"]
                self.cfg["scales"][1] = self.GUI.gui.dsbScale5["value"]
                self.cfg["scales"][2] = self.GUI.gui.dsbScale6["value"]
                self.cfg["scales"][3] = self.GUI.gui.dsbScale7["value"]
                self.hbm.setParam("samplingRate", self.cfg["freq"])
                self.hbm.setParam("numSamples", self.cfg["samples"])

                maxrange = 0
                for nc in range(0, len(self.cfg["channels"])):
                    if self.cfg["ranges"][self.cfg["channels"][nc] - 4] > maxrange:
                        maxrange = self.cfg["ranges"][self.cfg["channels"][nc] - 4]
                self.GUI.axisScaleLive = [self.cfg["rangeVals"][maxrange] * -1.05, self.cfg["rangeVals"][maxrange] * 1.05]

            self.hbm.startDevice()
            #self.GUI.gui.leStatus["text"] = self.hbm.getParam("status")

    def newMeasVal(self):
        if self.GUI.upDating == True:
            return
        self.GUI.upDating = True
        tmpObj = dataObject()

        try:
            self.hbm.acquire()
            self.resultObj[0, self.numMeas] = time.clock()
            time.sleep(self.cfg["samples"] / self.cfg["freq"])
            self.hbm.getVal(tmpObj)
            for nc in range(0, tmpObj.size(0)):
                self.resultObj[nc + 1, self.numMeas] = np.sum(tmpObj[nc, :] \
                    / self.cfg["samples"] * self.cfg["scales"][self.cfg["channels"][nc] - 4])

            if tmpObj.size(0) >= 1:
                self.GUI.gui.leLastValue4["text"] = "{0:.5g}".format(self.resultObj[1, self.numMeas])
            else:
                self.GUI.gui.leLastValue4["text"] =  ""
            if tmpObj.size(0) >= 2:
                self.GUI.gui.leLastValue5["text"] = "{0:.5g}".format(self.resultObj[2, self.numMeas])
            else:
                self.GUI.gui.leLastValue5["text"] =  ""
            if tmpObj.size(0) >= 3:
                self.GUI.gui.leLastValue6["text"] = "{0:.5g}".format(self.resultObj[3, self.numMeas])
            else:
                self.GUI.gui.leLastValue6["text"] = ""
            if tmpObj.size(0) >= 4:
                self.GUI.gui.leLastValue7["text"] = "{0:.5g}".format(self.resultObj[4, self.numMeas])
            else:
                self.GUI.gui.leLastValue7["text"] =  ""
            self.GUI.plotAutoScale()

            if ((self.numMeas + 1) > self.GUI.gui.tbMeas["rowCount"]):
                self.GUI.gui.tbMeas.call("insertRow", self.numMeas)
            if (not self.GUI.gui.cbZero["checked"]):
                self.GUI.gui.tbMeas.call("setItem", self.numMeas, 0, self.resultObj[0, self.numMeas])
                for nc in range(0, tmpObj.size(0)):
                    self.GUI.gui.tbMeas.call("setItem", self.numMeas, self.cfg["channels"][nc] - 4 + 1, self.resultObj[nc + 1, self.numMeas])
            else:
                self.GUI.gui.tbMeas.call("setItem", self.numMeas, 0, self.resultObj[0, self.numMeas] - self.resultObj[0, 0])
                for nc in range(0, tmpObj.size(0)):
                    self.gui.tbMeas.call("setItem", self.numMeas, self.cfg["channels"][nc] - 4 + 1, self.resultObj[nc + 1, self.numMeas] - self.resultObj[nc + 1, 0])

            self.numMeas += 1
            if self.numMeas == self.preAllocSize:
                tmp = dataObject.zeros([1 + tmpObj.size(0), self.preAllocSize * 2], dtype='float64')
                tmp[:, 0:self.preAllocSize] = self.resultObj
                self.resultObj = tmp.copy()
                self.preAllocSize *= 2
        except:
            pass
        self.GUI.upDating = False

    def newImage(self):
        if not self.cam is None:
            self.numImg += 1
            try:
                self.cam.acquire()
            except:
                ui.msgCritical("Error", "Could not capture image, maybe out of focus or no memory!")
            if self.downloadImgsMeas == True:
                self.tmpImg = dataObject()
                self.cam.getVal(self.tmpImg)
                self.tmpImg.setTag("numMeas", self.numMeas)
                self.tmpImg.setTag("measTime", time.clock())
                self.GUI.gui.plotCamImage["source"] = self.tmpImg
                incr = 0
                if (os.path.isfile("{0}\\img{1}.idc".format(self.imgsPathMeas, self.numImg))):
                    while (os.path.isfile("{0}\\img{1}_{2}.idc".format(self.imgsPathMeas, self.numImg, incr))):
                        incr += 1
                if (incr == 0):
                    exec("img{1}=self.tmpImg.copy()\nsaveIDC(\"{0}\\Img{1}.idc\", {{{1}:img{1}}})".format(self.imgsPathMeas, self.numImg))
                else:
                    exec("saveIDC(\"{0}\\Img{1}_{2}.idc\", {{{1}:img{1}}})".format(self.imgsPathMeas, self.numImg, incr))

    def CalcAverageMagnification(self, ptsObj, ptsImg):
        avgx = 0
        avgy = 0
        for nimg in range(0, ptsObj.shape[0]):
            npx = 0
            npy = 0
            colwise = 0
            dx = 1
            dy = 1
            iavgx = 0
            iavgy = 0
            if np.abs(ptsObj[nimg, 0, 0] - ptsObj[nimg, 1, 0]) < 0.0001:
                while np.abs(ptsObj[nimg, npx, 0] - ptsObj[nimg, 0, 0]) < 0.0001 and npx < ptsObj.shape[1]:
                    npx += 1
                npy = int(ptsObj.shape[1] / npx)
                if npx == ptsObj.shape[1] or npx * npy != ptsObj.shape[1]:
                    ui.msgCritical("Error", "Could not determine checkerboard size!")
                    return -1
                dx = np.abs(ptsObj[nimg, npx, 0] - ptsObj[nimg, 0, 0])
                dy = np.abs(ptsObj[nimg, 1, 1] - ptsObj[nimg, 0, 1])
            elif np.abs(ptsObj[nimg, 0, 1] - ptsObj[nimg, 1, 1]) < 0.0001:
                colwise = 1
                while np.abs(ptsObj[nimg, npy, 1] - ptsObj[nimg, 0, 1]) < 0.0001 and npy < ptsObj.shape[1]:
                    npy += 1
                npx = int(ptsObj.shape[1] / npy)
                if npy == ptsObj.shape[1] or npx * npy != ptsObj.shape[1]:
                    ui.msgCritical("Error", "Could not determine checkerboard size!")
                    return -1
                dy = np.abs(ptsObj[nimg, npy, 1] - ptsObj[nimg, 0, 1])
                dx = np.abs(ptsObj[nimg, 1, 0] - ptsObj[nimg, 0, 0])
            else:
                ui.msgCritical("Error", "Unknown pattern type, currently only checkerboard type supported!")
                return -1

            if colwise:
                for nc in range(0, npx):
                    iavgx += np.mean(np.sqrt(np.sum((ptsImg[nimg, 1 + nc * npy : (nc + 1) * npy, :] \
                        -  ptsImg[nimg, nc * npy : (nc + 1) * npy - 1, :])**2, axis=2)))
                for nc in range(0, npx - 1):
                    iavgy += np.mean(np.sqrt(np.sum((ptsImg[nimg, (nc + 1) * npy : (nc + 2) * npy, :] \
                        -  ptsImg[nimg, nc * npy : (nc + 1) * npy, :])**2, axis=2)))
                iavgx /= npx
                iavgy /= (npx - 1)
                pass
            else:
                for nc in range(0, npy):
                    iavgy += np.mean(np.sqrt(np.sum((ptsImg[nimg, 1 + nc * npx : (nc + 1) * npx, :] \
                        -  ptsImg[nimg, nc * npx : (nc + 1) * npx - 1, :])**2, axis=2)))
                for nc in range(0, npy - 1):
                    iavgx += np.mean(np.sqrt(np.sum((ptsImg[nimg, (nc + 1) * npx : (nc + 2) * npx, :] \
                        -  ptsImg[nimg, nc * npx : (nc + 1) * npx, :])**2, axis=2)))
                iavgy /= npy
                iavgx /= (npy - 1)
                pass

            avgx += dx / iavgx
            avgy += dy / iavgy

        avgx = (avgx / ptsObj.shape[0])
        avgy = (avgy / ptsObj.shape[0])

        return (avgx + avgy) / 2.0

if( __name__ == '__main__'):
    try:
        dicgui = dicGui()
    except:
        raise RuntimeError("Could not load dataViewer.")

    dicgui.init()
    dicgui.show(0)
