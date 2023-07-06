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
import __main__

try:
    from .auxFuncs import *
except:
    from auxFuncs import *

reloadModules = 1

class dicGuiUi(ItomUi):

    def __init__(self, sys ,systemPath = None):
        '''
        Internal constructor of the GUI class.

        Parameters
        -----------
        sys: {dicgui}
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
        self.sys = sys
        self.calibImages = None
        self.numCalibImg = 0
        self.calibImg = None
        self.openItem = None
        self.dispWin = None
        self.updating = False
        self.upDating = False
        self.cellUpdating = False
        self.loadedImgListFile = -1
        self.axisScaleLive = [-0.125 * 1.05, 0.125 * 1.05]
        self.generateRun = 0

        if(systemPath is None):
            ownFilename = inspect.getfile(inspect.currentframe())
            self.ownDir = os.path.dirname(os.path.realpath(__file__)) #os.path.dirname(ownFilename)
        else:
            self.ownDir = systemPath

        self.lastSaveDir = None
        self.calibDir = None
        ownFilename = inspect.getfile(inspect.currentframe())
        ownDir = os.path.dirname(ownFilename)

        uiFile = os.path.join(ownDir, "ui/dicgui.ui")
        uiFile = os.path.abspath(uiFile)
        if userIsDeveloper() :
            ItomUi.__init__(self, uiFile, ui.TYPEWINDOW, childOfMainWindow=True, deleteOnClose = True)
        else:
            ItomUi.__init__(self, uiFile, ui.TYPEDOCKWIDGET, childOfMainWindow=True, deleteOnClose = True)
        if (self.gui != None):
            self.gui.ple_selCalibDir["currentPath"] = ownDir
            self.gui.le_baseFName["text"] = "distCalib_" + time.strftime("%Y%m%d_%H%M")
        self.gui.teCalibData["plainText"] = "N/A"

        if self.sys().cfg["channels"] != []:
            numChan = len(self.sys().cfg["channels"])
        else:
            numChan = 0
        self.sys().resultObj = dataObject([5, self.sys().preAllocSize],dtype='float64')

    def __del__(self):
        if not self.sys() is None:
            self.sys().saveConfig()

    def init(self):
        '''
        Set up values of the GUI-elementes.

        Parameters
        -----------
        cam: camera for the live image

        Returns
        -------
        None

        Notes
        -----
        This function initializes the gui elements and set the values to start the image processing.
        '''

        self.gui.twDicGui["currentIndex"] = 0
        #self.gui.pe_openFolder["filters"] = "Dirs"

        self.upDating = True
        self.gui.leSamples["text"] = self.sys().cfg["samples"]
        self.gui.lePort["text"] = self.sys().cfg["serport"]
        self.gui.leSpiderDelay["text"] = self.sys().cfg["spiderDelay"]

        self.gui.cbRange4["currentIndex"] = int(self.sys().cfg["ranges"][0])
        self.gui.cbRange5["currentIndex"] = int(self.sys().cfg["ranges"][1])
        self.gui.cbRange6["currentIndex"] = int(self.sys().cfg["ranges"][2])
        self.gui.cbRange7["currentIndex"] = int(self.sys().cfg["ranges"][3])

        self.gui.cbBridge4["currentIndex"] = int(self.sys().cfg["bridges"][0])
        self.gui.cbBridge5["currentIndex"] = int(self.sys().cfg["bridges"][1])
        self.gui.cbBridge6["currentIndex"] = int(self.sys().cfg["bridges"][2])
        self.gui.cbBridge7["currentIndex"] = int(self.sys().cfg["bridges"][3])

        self.gui.cbFrequency["currentText"] = self.sys().cfg["freq"]
        scaleMin = 0
        scaleMax = 0

        self.sys().range2CB(self.sys().cfg["ranges"][0], self.gui.cbRange4)
        self.sys().range2CB(self.sys().cfg["ranges"][1], self.gui.cbRange5)
        self.sys().range2CB(self.sys().cfg["ranges"][2], self.gui.cbRange6)
        self.sys().range2CB(self.sys().cfg["ranges"][3], self.gui.cbRange7)

        for nc in range(4, 8):
            exec("self.gui.cbChan" + str(nc) + "[\"checked\"] = False")
        for nc in range(0, len(self.sys().cfg["channels"])):
            exec("self.gui.cbChan" + str(self.sys().cfg["channels"][nc]) + "[\"checked\"] = True")

        if self.sys().cfg["enableCam"] == 1:
            self.gui.cbEnableCam["checked"] = True
        else:
            self.gui.cbEnableCam["checked"] = False

        if self.sys().cfg["enableSpider"] == 1:
            self.gui.cbEnableSpider["checked"] = True
        else:
            self.gui.cbEnableSpider["checked"] = False

        self.gui.plotSpiderGraph["yAxisInterval"] = self.axisScaleLive

        self.gui.cbDownloadImgs["checked"] = self.sys().cfg["downloadImgs"]
        self.gui.pleImgsPath["enabled"] = self.sys().cfg["downloadImgs"]
        self.gui.pleImgsPath["currentPath"] = self.sys().cfg["imgsPath"]
        self.upDating = False

        self.gui.leMeasTime["text"] = float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"])


    def show(self, modalLevel):
        '''
        Displayes the User Interface.
        '''
        ret = self.gui.show(modalLevel)


    def getMinMax(self, points):
        if len(points["point1"]) == 2:
            if points["point1"][0] > points["point2"][0]:
                xmin = int(points["point2"][0])
                xmax = int(points["point1"][0])
            else:
                xmin = int(points["point1"][0])
                xmax = int(points["point2"][0])
            if points["point1"][1] > points["point2"][1]:
                ymin = int(points["point2"][1])
                ymax = int(points["point1"][1])
            else:
                ymin = int(points["point1"][1])
                ymax = int(points["point2"][1])
        else:
            if not self.sys().imageInput is None:
                ymin = 0
                ymax = self.sys().imageInput.size(self.sys().imageInput.dims - 2) - 1
                xmin = 0
                xmax = self.sys().imageInput.size(self.sys().imageInput.dims - 1) - 1
            else:
                ymin = 0
                ymax = 1
                xmin = 0
                xmax = 1
        return [xmin, ymin, xmax, ymax]

    def updateInpImage(self):
        # determine usable image range
        if not 'imageOrig' in self.sys().data or not 'imageDef' in self.sys().data \
            or self.sys().data['imageOrig'] is None or self.sys().data['imageDef'] is None:
            return
        usDic = self.sys().detUsableImgRange(self.sys().data["imageOrig"], self.sys().data["imageDef"])

        if self.gui.cbDispType["currentIndex"] == 0:
            self.gui.pltImageGen["source"] = self.sys().data["imageOrig"]
        elif self.gui.cbDispType["currentIndex"] == 1:
            self.gui.pltImageGen["source"] = self.sys().data["imageDef"]
        elif self.gui.cbDispType["currentIndex"] == 2 and not self.sys().data["imageDef"] is None and not self.sys().data["imageOrig"] is None:
            self.gui.pltImageGen["source"] = self.sys().data["imageDef"][usDic["dO"][0,1] : usDic["dS"][0, 1], \
                usDic["dO"][1, 1] : usDic["dS"][1, 1]] \
                - self.sys().data["imageOrig"][usDic["dO"][0, 0] : usDic["dS"][0, 0], usDic["dO"][1, 0] : usDic["dS"][1, 0]]
        self.gui.pltDisplacement["source"] = self.sys().data["imageDef"][usDic["dO"][0, 1] : usDic["dS"][0, 1], \
                usDic["dO"][1, 1] : usDic["dS"][1, 1]]

    def fillDispTable(self, outVec):
        largestShift = 0
        self.gui.tblDisplRes.call("clearContents")
        for nc in range(0, self.gui.tblDisplRes["rowCount"]):
            self.gui.tblDisplRes.call("removeRow", 0)
        if outVec is None:
            ui.msgCritical("Error", "No displacement data loaded!")
            return
        # subset output
        if (outVec.size(1) == 12):
            for nc in range(0, outVec.size(0)):
                self.gui.tblDisplRes.call("insertRow", nc)
                if self.gui.cbDisplMScaling["checked"] :
                    actShift = self.mscale * np.sqrt(outVec[nc, 0] ** 2 + outVec[nc, 1] ** 2)
                    if actShift > largestShift:
                        largestShift = actShift
                    for nco in range(0, outVec.size(1)):
                        self.gui.tblDisplRes.call("setItem", nc, nco, self.mscale * outVec[nc, nco])
                else:
                    actShift = np.sqrt(outVec[nc, 0] ** 2 + outVec[nc, 1] ** 2)
                    if actShift > largestShift:
                        largestShift = actShift
                    for nco in range(0, outVec.size(1)):
                        self.gui.tblDisplRes.call("setItem", nc, nco, outVec[nc, nco])
        # full field output
        if (outVec.size(1) == 4):
            for nc in range(0, outVec.size(0)):
                self.gui.tblDisplRes.call("insertRow", nc)
                if self.gui.cbDisplMScaling["checked"]:
                    actShift =  self.mscale * np.sqrt(outVec[nc, 2] ** 2 + outVec[nc, 3] ** 2)
                    if actShift > largestShift:
                        largestShift = actShift
                    self.gui.tblDisplRes.call("setItem", nc, 0, self.mscale * outVec[nc, 2])
                    self.gui.tblDisplRes.call("setItem", nc, 1, self.mscale * outVec[nc, 3])
                else:
                    actShift = np.sqrt(outVec[nc, 2] ** 2 + outVec[nc, 3] ** 2)
                    if actShift > largestShift:
                        largestShift = actShift
                    self.gui.tblDisplRes.call("setItem", nc, 0, outVec[nc, 2])
                    self.gui.tblDisplRes.call("setItem", nc, 1, outVec[nc, 3])

        return largestShift
        pass

    @ItomUi.autoslot("")
    def on_Form_destroyed(self):
        self.sys().deleteUI()

    #@ItomUi.autoslot("")
    #def on_Dialog_destroyed(self):
        #self.sys().deleteUI()

    #@ItomUi.autoslot("QString")
    #def on_pe_savePath_currentPathChanged(self, pathString):
        #self.lastSaveDir = pathString
        #self.gui.pe_savePath.call("addCurrentPathToHistory")

    @ItomUi.autoslot("int")
    def on_twDicGui_currentChanged(self,index):
        if index==0:
            pass
        elif index == 3:
            self.gui.lwDeform.call("clear")
            keylist = list(self.sys().data["displResults"].keys())
            if keylist != []:
                for ndis in range(0, len(keylist)):
                    self.gui.lwDeform.call("addItem", keylist[ndis])
                self.gui.lwDeform["currentRow"] = 0
                self.sys().data["displ"] = self.sys().data["displResults"][keylist[0]]["displVecs"][0]
                self.sys().data["displCC"] = self.sys().data["displResults"][keylist[0]]["displVecs"][1]

                if self.sys().data["displResults"][keylist[0]]["imageDefPath"] != "":
                    self.gui.pltDeformation["source"] = self.loadDisplImages(self.sys().data["displResults"][keylist[0]]["imageDefPath"])
                else:
                    self.gui.pltDeformation["source"] = self.sys().data["imageDef"]

        elif index==4:
            pass
        else:
            pass

    @ItomUi.autoslot("int")
    def on_cbDispType_currentIndexChanged(self,index):
        self.updateInpImage()

    @ItomUi.autoslot("int")
    def on_cbDefType_currentIndexChanged(self,index):
        if index == 0:
            self.gui.dsbNu["enabled"] = False
            self.gui.dsbE["enabled"] = False
            self.gui.dsbT["enabled"] = False
            self.gui.dsbM["enabled"] = False
            self.gui.dsbI["enabled"] = False
            self.gui.dsbTe["enabled"] = False
            self.gui.dsbTr["enabled"] = False
            self.gui.dsbPa["enabled"] = False
            self.gui.dsbPc["enabled"] = False
            self.gui.dsbD1["enabled"] = True
            self.gui.dsbD2["enabled"] = True
            self.gui.dsbD3["enabled"] = False
            self.gui.dsbD4["enabled"] = False
            self.gui.dsbD5["enabled"] = False
            self.gui.dsbD6["enabled"] = False
            self.gui.dsbD7["enabled"] = False
            self.gui.dsbD8["enabled"] = False
            self.gui.dsbD9["enabled"] = False
            self.gui.dsbD10["enabled"] = False
            self.gui.dsbD11["enabled"] = False
            self.gui.dsbD12["enabled"] = False
        elif index == 1:
            self.gui.dsbNu["enabled"] = True
            self.gui.dsbE["enabled"] = True
            self.gui.dsbT["enabled"] = True
            self.gui.dsbM["enabled"] = False
            self.gui.dsbI["enabled"] = False
            self.gui.dsbTe["enabled"] = False
            self.gui.dsbTr["enabled"] = False
            self.gui.dsbPa["enabled"] = False
            self.gui.dsbPc["enabled"] = False
            self.gui.dsbD1["enabled"] = False
            self.gui.dsbD2["enabled"] = False
            self.gui.dsbD3["enabled"] = False
            self.gui.dsbD4["enabled"] = False
            self.gui.dsbD5["enabled"] = False
            self.gui.dsbD6["enabled"] = False
            self.gui.dsbD7["enabled"] = False
            self.gui.dsbD8["enabled"] = False
            self.gui.dsbD9["enabled"] = False
            self.gui.dsbD10["enabled"] = False
            self.gui.dsbD11["enabled"] = False
            self.gui.dsbD12["enabled"] = False
        elif index == 2:
            self.gui.dsbNu["enabled"] = True
            self.gui.dsbE["enabled"] = True
            self.gui.dsbT["enabled"] = False
            self.gui.dsbM["enabled"] = True
            self.gui.dsbI["enabled"] = True
            self.gui.dsbTe["enabled"] = False
            self.gui.dsbTr["enabled"] = True
            self.gui.dsbPa["enabled"] = False
            self.gui.dsbPc["enabled"] = False
            self.gui.dsbD1["enabled"] = False
            self.gui.dsbD2["enabled"] = False
            self.gui.dsbD3["enabled"] = False
            self.gui.dsbD4["enabled"] = False
            self.gui.dsbD5["enabled"] = False
            self.gui.dsbD6["enabled"] = False
            self.gui.dsbD7["enabled"] = False
            self.gui.dsbD8["enabled"] = False
            self.gui.dsbD9["enabled"] = False
            self.gui.dsbD10["enabled"] = False
            self.gui.dsbD11["enabled"] = False
            self.gui.dsbD12["enabled"] = False
        elif index == 3:
            self.gui.dsbNu["enabled"] = True
            self.gui.dsbE["enabled"] = True
            self.gui.dsbT["enabled"] = False
            self.gui.dsbM["enabled"] = False
            self.gui.dsbI["enabled"] = False
            self.gui.dsbTe["enabled"] = False
            self.gui.dsbTr["enabled"] = True
            self.gui.dsbPa["enabled"] = True
            self.gui.dsbPc["enabled"] = False
            self.gui.dsbD1["enabled"] = False
            self.gui.dsbD2["enabled"] = False
            self.gui.dsbD3["enabled"] = False
            self.gui.dsbD4["enabled"] = False
            self.gui.dsbD5["enabled"] = False
            self.gui.dsbD6["enabled"] = False
            self.gui.dsbD7["enabled"] = False
            self.gui.dsbD8["enabled"] = False
            self.gui.dsbD9["enabled"] = False
            self.gui.dsbD10["enabled"] = False
            self.gui.dsbD11["enabled"] = False
            self.gui.dsbD12["enabled"] = False
        elif index == 4:
            self.gui.dsbNu["enabled"] = True
            self.gui.dsbE["enabled"] = True
            self.gui.dsbT["enabled"] = False
            self.gui.dsbM["enabled"] = False
            self.gui.dsbI["enabled"] = False
            self.gui.dsbTe["enabled"] = True
            self.gui.dsbTr["enabled"] = False
            self.gui.dsbPa["enabled"] = False
            self.gui.dsbPc["enabled"] = True
            self.gui.dsbD1["enabled"] = False
            self.gui.dsbD2["enabled"] = False
            self.gui.dsbD3["enabled"] = False
            self.gui.dsbD4["enabled"] = False
            self.gui.dsbD5["enabled"] = False
            self.gui.dsbD6["enabled"] = False
            self.gui.dsbD7["enabled"] = False
            self.gui.dsbD8["enabled"] = False
            self.gui.dsbD9["enabled"] = False
            self.gui.dsbD10["enabled"] = False
            self.gui.dsbD11["enabled"] = False
            self.gui.dsbD12["enabled"] = False
        elif index == 5:
            self.gui.dsbNu["enabled"] = False
            self.gui.dsbE["enabled"] = False
            self.gui.dsbT["enabled"] = False
            self.gui.dsbM["enabled"] = False
            self.gui.dsbI["enabled"] = False
            self.gui.dsbTe["enabled"] = False
            self.gui.dsbTr["enabled"] = False
            self.gui.dsbPa["enabled"] = False
            self.gui.dsbPc["enabled"] = False
            self.gui.dsbD1["enabled"] = True
            self.gui.dsbD2["enabled"] = True
            self.gui.dsbD3["enabled"] = True
            self.gui.dsbD4["enabled"] = True
            self.gui.dsbD5["enabled"] = True
            self.gui.dsbD6["enabled"] = True
            self.gui.dsbD7["enabled"] = True
            self.gui.dsbD8["enabled"] = True
            self.gui.dsbD9["enabled"] = True
            self.gui.dsbD10["enabled"] = True
            self.gui.dsbD11["enabled"] = True
            self.gui.dsbD12["enabled"] = True
        else:
            pass

    @ItomUi.autoslot("")
    def on_cb3DProj_clicked(self):
        if self.gui.cb3DProj["checked"]:
            self.gui.pleCalibFile["enabled"] = True
            self.gui.cbCoordSysNum["enabled"] = True
            self.gui.dsbObjSizeX["enabled"] = True
            self.gui.dsbObjSizeY["enabled"] = True
            self.gui.dsbZValue["enabled"] = True
            self.gui.dsbNu3D["enabled"] = True
            self.gui.dsbE3D["enabled"] = True
            self.gui.dsbPc3D["enabled"] = True
            self.gui.cbDisplUndistort["enabled"] = True
        else:
            self.gui.pleCalibFile["enabled"] = False
            self.gui.cbCoordSysNum["enabled"] = False
            #self.gui.teCalibData["plainText"] = "N/A"
            self.gui.dsbObjSizeX["enabled"] = False
            self.gui.dsbObjSizeY["enabled"] = False
            self.gui.dsbZValue["enabled"] = False
            self.gui.dsbNu3D["enabled"] = False
            self.gui.dsbE3D["enabled"] = False
            self.gui.dsbPc3D["enabled"] = False
            self.gui.cbDisplUndistort["enabled"] = False

    @ItomUi.autoslot("int")
    def on_cbInpFileType_currentIndexChanged(self,index):
        if index == 0:
            self.gui.pleImageFile["enabled"] = False
            self.gui.sbSizeX["enabled"] = True
            self.gui.sbSizeY["enabled"] = True
            self.gui.sbSpeckleSize["enabled"] = True
            self.gui.sbBit["enabled"] = True
        elif index == 1:
            self.gui.pleImageFile["enabled"] = True
            self.gui.sbSizeX["enabled"] = False
            self.gui.sbSizeY["enabled"] = False
            self.gui.sbSpeckleSize["enabled"] = False
            self.gui.sbBit["enabled"] = False
        else:
            pass


    @ItomUi.autoslot("")
    def on_pbShowCameraCalib_pressed(self):
                ui.msgInformation("Calibration result", "Camera Matrix:\n{:>9.2f} {:9.2f} {:9.2f}\n\
{:>9.2f} {:>9.2f} {:>9.2f}\n{:>9.2f} {:>9.2f} {:>9.2f}\n\n\
".format(self.sys().cfg["CamMat"] [0,0], \
            self.sys().cfg["CamMat"] [0, 1], self.sys().cfg["CamMat"] [0, 2], self.sys().cfg["CamMat"] [1, 0], \
            self.sys().cfg["CamMat"] [1, 1], self.sys().cfg["CamMat"] [1, 2], self.sys().cfg["CamMat"] [2, 0], \
            self.sys().cfg["CamMat"] [2, 1], self.sys().cfg["CamMat"] [2, 2]))

    @ItomUi.autoslot("QString")
    def on_pleDisplCalibrationFile_currentPathChanged(self, pathString):
        self.gui.pbShowCameraCalib["enabled"] = True

    @ItomUi.autoslot("QString")
    def on_pleCalibFile_currentPathChanged(self, pathString):
        #filename = ui.getOpenFileName('Select file to load', self.ownDir, 'Itom Dictionary (*.idc)')
        if not pathString:
            return

        try:
            calib = loadIDC(pathString)
        except:
            pass

        try:
            self.sys().cfg["CamMat"] = calib["CamMat"]
            self.sys().cfg["DistCoeff"] = calib["DistCoeff"]
            self.sys().cfg["RVecs"] = calib["RVecs"]
            self.sys().cfg["TVecs"] = calib["TVecs"]
            self.sys().cfg["cam.calibFile"] = pathString
            self.sys().saveConfig()
            camMat = calib["CamMat"]
            tVecs = calib["TVecs"]
            rVecs = calib["RVecs"]
            distCoeff = calib["DistCoeff"]
            calibText = "Camera Matrix:\n{:>9.2f} {:9.2f} {:9.2f}\n\
{:>9.2f} {:>9.2f} {:>9.2f}\n{:>9.2f} {:>9.2f} {:>9.2f}\n\n\
Distortion Coefficients:\n{:7.4f} {:7.4f} {:7.4f} {:7.4f} {:7.4f}\n\n\
Translation Vectors:\n".format(camMat[0,0], \
            camMat[0, 1], camMat[0, 2], camMat[1, 0], camMat[1, 1], camMat[1, 2], \
            camMat[2, 0], camMat[2, 1], camMat[2, 2], distCoeff[0, 0], distCoeff[0, 1], \
            distCoeff[0, 2], distCoeff[0, 3], distCoeff[0, 4])
            for n in range(0, 3):
                for tv in range(0, tVecs.size(1)):
                    calibText = calibText + "{:>9.2f} ".format(tVecs[n, tv])
                if n < 2:
                    calibText = calibText + "\n"
            calibText = calibText + "\n\nRotation Vectors:\n"
            for n in range(0, 3):
                for tv in range(0, tVecs.size(1)):
                    calibText = calibText + "{:>9.2f} ".format(rVecs[n, tv])
                if n < 2:
                    calibText = calibText + "\n"
            self.gui.teCalibData["plainText"] = str(calibText)
            self.sys().loadedCalib = 1
            self.gui.cbCoordSysNum.call("clear")
            self.gui.cbCoordSysNum["maxCount"] = tVecs.size(1)
            for tv in range(0, tVecs.size(1)):
                self.gui.cbCoordSysNum.call("addItem", tv)
            self.gui.cbCoordSysNum["currentIndex"] = 0
        except:
            ui.msgCritical("Error", "Could not load calibration data, verfiy file format!")
            self.sys().loadedCalib = None
            self.gui.teCalibData["plainText"] = "N/A"
            self.gui.cbCoordSysNum.call("clear")

    @ItomUi.autoslot("")
    def on_pb_LoadCalibImages_pressed(self):
        filenames = ui.getOpenFileNames('Select files to load', self.ownDir, 'RAW Image (*.nef);; Itom Dictionary (*.idc);; Bitmap (*.bmp);; JPG FIle (*.jpg);; Tiff Image (*.tiff);; All Files (*.*)')
        if not filenames:
            pass
        else:
            self.gui.lw_Files.call("clear")
            self.gui.lw_Files.call("addItem","Live Image")
            nimg = 1
            self.calibImages = dict()
            if not filenames is None:
                for nf in filenames:
                    if nf[-8:-4] == '_pts':
                        continue
                    self.gui.lw_Files.call("addItem",nf)
                    tmpImg = dataObject()
                    # ignore point files
                    if str.lower(nf[-4:]) == '.idc':
                        tmpImg = loadIDC(nf)
                        tmpImg = list(tmpImg.values())[0]
                    elif str.lower(nf[-4:]) == '.nef':
                        tmpImg = dataObject()
                        filter("loadRawImage", nf, tmpImg, "-o 0 -4 -D -j -T -t 0")
                        tagss = tmpImg.tags
                        tmpImg = dataObject(cv2.cvtColor(np.array(tmpImg), cv2.COLOR_BayerRG2GRAY))
                        tmpImg.tags = dict(tagss)
                    else:
                        filter("loadAnyImage", tmpImg, nf)

                    # convert to grayscale image
                    if tmpImg.dtype == 'rgba32':
                        filter("cvCvtColor", tmpImg, tmpImg, 7)

                    ptsFileName = nf[:-4] + "_pts.idc"
                    try:
                        tmpPts = loadIDC(ptsFileName)
                        if list(tmpPts.values())[0].size(1) < 5:
                            markerSize = [self.gui.dsb_SizeX["value"], self.gui.dsb_SizeY["value"]]
                            pts = self.sys().defineCoordSys(tmpImg, list(tmpPts.values())[0], markerSize)
                            saveIDC(ptsFileName, {"pts" : pts})
                        self.calibImages[nimg]  = {'img' : tmpImg.copy(), 'pts' : list(tmpPts.values())[0].copy(), 'name': nf}
                    except:
                        pts = dataObject()
                        nptsx = self.gui.sb_ptsX["value"]
                        nptsy = self.gui.sb_ptsY["value"]
                        [res, pts] = self.sys().findCBPoints(tmpImg, [nptsx, nptsy])
                        if res <= 0:
                            ui.msgCritical("Error", "Could not detect chessboard corners.\nCheck if number of points (x/y) is correct!")
                            self.calibImages[nimg] = {'img' : tmpImg.copy(), 'name' : nf}
                        else:
                            try:
                                markerSize = [self.gui.dsb_SizeX["value"], self.gui.dsb_SizeY["value"]]
                                pts = self.sys().defineCoordSys(tmpImg, pts, markerSize)
                                saveIDC(ptsFileName, {"pts" : pts})
                                self.calibImages[nimg] = {'img' : tmpImg.copy(), 'pts' : pts, 'name' : nf}
                            except:
                                self.calibImages[nimg] = {'img' : tmpImg.copy(), 'name' : nf}

                    nimg = nimg + 1
            self.gui.lw_Files["currentRow"] = 1

    @ItomUi.autoslot("")
    def on_pb_RedefineCoordSys_pressed(self):
        nimg = self.gui.lw_Files["currentRow"]
        if nimg < 0:
            return
        try:
            filename = self.gui.lw_Files.call("item", self.gui.lw_Files["currentRow"])
        except:
            return
        ptsFileName = filename[:-4] + "_pts.idc"
        tmpImg = self.gui.pltCalibration["source"]
        # ignore point files

        pts = dataObject()
        nptsx = self.gui.sb_ptsX["value"]
        nptsy = self.gui.sb_ptsY["value"]
        [res, pts] = self.sys().findCBPoints(tmpImg, [nptsx, nptsy])
        if res <= 0:
            ui.msgCritical("Error", "Could not detect chessboard corners.\nCheck if number of points (x/y) is correct!")
            self.calibImages[nimg] = {'img' : tmpImg.copy(), 'name' : filename}
        else:
            try:
                markerSize = [self.gui.dsb_SizeX["value"], self.gui.dsb_SizeY["value"]]
                pts = self.sys().defineCoordSys(tmpImg, pts, markerSize)
                saveIDC(ptsFileName, {"pts" : pts})
                self.calibImages[nimg] = {'img' : tmpImg.copy(), 'pts' : pts, 'name' : filename}
            except:
                self.calibImages[nimg] = {'img' : tmpImg.copy(), 'name' : filename}
        pass

    @ItomUi.autoslot("")
    def on_lw_Files_itemSelectionChanged(self):
        curRow = self.gui.lw_Files["currentRow"]
        if curRow <= 0:
            self.gui.pltCalibration.call("clearGeometricShapes")
        else:
            self.gui.pltCalibration["camera"] = None
            self.gui.pltCalibration.call("clearGeometricShapes")
            self.gui.pltCalibration["source"] = self.calibImages[self.gui.lw_Files["currentRow"]]["img"]
            try:
                ptsObj = self.calibImages[self.gui.lw_Files["currentRow"]]["pts"]
                numPts = ptsObj.size(0)
                shapes = set()
                for np in range(0, numPts):
                    pt = ptsObj[np, :]
                    shapes.add(shape(2, pt[:, :2]))
                self.gui.pltCalibration.call("setGeometricShapes", list(shapes))
                self.gui.pltCalibration["geometryModificationModes"] = 0
            except:
                pass

    @ItomUi.autoslot("")
    def on_pb_CaptureCB_pressed(self):

        self.calibPath = self.sys().modulePath + self.gui.le_baseFName["text"] +"/"

        if not os.path.exists(self.calibPath):
            os.makedirs(self.calibPath)

        if self.sys().cam != None:
            calibImg = dataObject()
            try:
                self.sys().cam.acquire()
                self.sys().cam.getVal(calibImg)
            except:
                ui.msgCritical("Error", "Error acquiring image!")

            if calibImg.dtype == 'rgba32':
                # demosaicing
                tagsSave = calibImg.tags
                calibImg = dataObject(cv2.cvtColor(np.array(calibImg), cv2.COLOR_BayerRG2GRAY))
                #tmpObj = dataObject(cv2.cvtColor(np.array(tmpObj), cv2.COLOR_BayerRG2RGB))
                calibImg.tags = dict(tagsSave)
        else:
            ui.msgCritical("Error", "No camera initialized, aborting!")
            return

        #self.gui.pltCalibration["source"]
        tempDict = {}
        tempDict.update({self.gui.le_baseFName["text"] + "_" + str(self.numCalibImg) : calibImg})
        filename = self.calibPath + self.gui.le_baseFName["text"] + "_" + str(self.numCalibImg) + ".idc"
        self.numCalibImg = self.numCalibImg + 1
        saveIDC(filename, tempDict)
        numImgs = self.gui.lw_Files["count"]
        if numImgs <= 0:
            self.gui.lw_Files.call("addItem", "Live Image")
            numImgs = 1
        self.gui.lw_Files.call("addItem",filename)
        if self.calibImages is None:
            self.calibImages = dict()
        self.calibImages[numImgs] = {"img": calibImg.copy()}
        self.calibImages[numImgs]["name"] = filename
        self.gui.lw_Files["currentRow"] = numImgs
        nptsx = self.gui.sb_ptsX["value"]
        nptsy = self.gui.sb_ptsY["value"]
        pts = dataObject()
        res = self.sys().findCBPoints(calibImg, [nptsx, nptsy], pts)
        if res == 0:
            ui.msgCritical("Error", "Could not detect chessboard corners")
        else:
            try:
                markerSize = [self.gui.dsb_SizeX["value"], self.gui.dsb_SizeY["value"]]
                pts = self.sys().defineCoordSys(calibImg, pts, markerSize)
                ptsFilename = filename[:-4] + "_pts.idc"
                saveIDC(ptsFilename, {"pts" : pts})
                self.calibImages[numImgs]["pts"] = pts.copy()
            except:
                ptsFilename = filename[:-4] + "_pts.idc"
                saveIDC(ptsFilename, {"pts" : pts})
                self.calibImages[numImgs]["pts"] = pts.copy()

    @ItomUi.autoslot("")
    def on_pb_DispCheckerBoard_pressed(self):
        if self.dispWin != None:
            try:
                del self.dispWin
                self.dispWin = None
                return
            except:
                pass

        self.gui.sb_ptsY["value"] = np.floor(self.gui.sb_ptsY["value"] / 2) * 2
        self.gui.sb_ptsX["value"] = np.floor(self.gui.sb_ptsX["value"] / 2) * 2
        numSqrsY = self.gui.sb_ptsY["value"] + 1
        numSqrsX = self.gui.sb_ptsX["value"] + 1
        cbPy = int(self.gui.sb_sizeY["value"] / (numSqrsY + 2))
        cbPx = int(self.gui.sb_sizeX["value"] / (numSqrsX + 2))
        self.gui.dsb_SizeX["value"] = cbPx * self.gui.dsb_pixelPitch["value"]
        self.gui.dsb_SizeY["value"] = cbPy * self.gui.dsb_pixelPitch["value"]

        borderY = int((self.gui.sb_sizeY["value"] - (numSqrsY) * cbPy) / 2)
        borderX = int((self.gui.sb_sizeX["value"] - (numSqrsX) * cbPx) / 2)
        col = 0
        checkerDObj = dataObject.ones([self.gui.sb_sizeY["value"], self.gui.sb_sizeX["value"]], dtype='uint8') * 255
        for ny in range(0, numSqrsY):
            ys = int(borderY + ny * cbPy)
            ye = ys + cbPy + 1
            for nx in range(0, numSqrsX):
                xs = int(borderX + nx * cbPx)
                xe = xs + cbPx + 1
                checkerDObj[ys:ye, xs:xe] = col
                if col > 0:
                    col = 0
                else:
                    col = 255

        if self.gui.sb_sizeX["value"] > 100:
            fmarkerLengthX = 50
        else:
            fmarkerLengthX = int(self.gui.sb_sizeX["value"] * 0.3)

        if self.gui.sb_sizeY["value"] > 100:
            fmarkerLengthY = 50
        else:
            fmarkerLengthY = int(self.gui.sb_sizeY["value"] * 0.3)

        # draw focus markers
        checkerDObj[0, 0:fmarkerLengthX] = 0
        checkerDObj[2, 2:fmarkerLengthX] = 0
        checkerDObj[0:fmarkerLengthY, 0] = 0
        checkerDObj[2:fmarkerLengthY, 2] = 0

        checkerDObj[0, self.gui.sb_sizeX["value"] - 1 - fmarkerLengthX:self.gui.sb_sizeX["value"] - 1] = 0
        checkerDObj[2, self.gui.sb_sizeX["value"] - 1 - fmarkerLengthX:self.gui.sb_sizeX["value"] - 3] = 0
        checkerDObj[0:fmarkerLengthY, self.gui.sb_sizeX["value"] - 1] = 0
        checkerDObj[2:fmarkerLengthY, self.gui.sb_sizeX["value"] - 3] = 0

        checkerDObj[self.gui.sb_sizeY["value"] - 1, 0:fmarkerLengthX] = 0
        checkerDObj[self.gui.sb_sizeY["value"] - 3, 2:fmarkerLengthX] = 0
        checkerDObj[self.gui.sb_sizeY["value"] - 1 - fmarkerLengthY:self.gui.sb_sizeY["value"] - 1, 0] = 0
        checkerDObj[self.gui.sb_sizeY["value"] - 3 - fmarkerLengthY:self.gui.sb_sizeY["value"] - 3, 2] = 0

        checkerDObj[self.gui.sb_sizeY["value"] - 1, self.gui.sb_sizeX["value"] - 1 - fmarkerLengthX:self.gui.sb_sizeX["value"] - 1] = 0
        checkerDObj[self.gui.sb_sizeY["value"] - 3, self.gui.sb_sizeX["value"] - 1 - fmarkerLengthX:self.gui.sb_sizeX["value"] - 3] = 0
        checkerDObj[self.gui.sb_sizeY["value"] - 1 - fmarkerLengthY:self.gui.sb_sizeY["value"] - 1, self.gui.sb_sizeX["value"] - 1] = 0
        checkerDObj[self.gui.sb_sizeY["value"] - 3 - fmarkerLengthY:self.gui.sb_sizeY["value"] - 3, self.gui.sb_sizeX["value"] - 3] = 0

        # draw origin marker
        checkerDObj[6, 4:9] = 0
        checkerDObj[4:9, 6] = 0

        self.dispWin = dataIO("DispWindow", x0=self.gui.sb_x0["value"], y0=self.gui.sb_y0["value"], xsize=self.gui.sb_sizeX["value"], ysize=self.gui.sb_sizeY["value"])
        self.dispWin.setParam("dObj", checkerDObj)

    @ItomUi.autoslot("int")
    def on_sb_sizeX_valueChanged(self,val):
        if self.updating:
            return
        else:
            self.updating = True
        #self.gui.sb_sizeX["value"] = int(self.gui.sb_sizeX["value"] / (self.gui.sb_ptsX["value"] + 2))
        self.updating = False

    @ItomUi.autoslot("int")
    def on_sb_sizeY_valueChanged(self,val):
        if self.updating:
            return
        else:
            self.updating = True
        #self.gui.sb_sizeY["value"] = int(self.gui.sb_sizeY["value"] / (self.gui.sb_ptsY["value"] + 2))
        self.updating = False

    @ItomUi.autoslot("")
    def on_pb_ClearImageList_pressed(self):
        self.gui.lw_Files.call("clear")
        self.calibImages = dict()

    @ItomUi.autoslot("")
    def on_pb_RemoveImage_pressed(self):
        if self.gui.lw_Files["currentRow"] > 0:
            remItm = self.gui.lw_Files["currentRow"]
            self.gui.lw_Files.call("takeItem", self.gui.lw_Files["currentRow"])
            numItm = self.gui.lw_Files["count"]
            for ni in range(remItm, numItm):
                self.calibImages[ni] = self.calibImages[ni + 1]
            del self.calibImages[numItm]

    @ItomUi.autoslot("")
    def on_pb_EvaluateDist_pressed(self):
        numImgs = self.gui.lw_Files["count"] - 1
        nviews = []
        for ni in range(0, numImgs):
            if 'pts' in self.calibImages[ni  +1]:
                pts = self.calibImages[ni  +1]["pts"]
                if pts.size(1) == 5:
                    nviews.append(1)
                else:
                    nviews.append(0)
            else:
                nviews.append(0)
        numViews = sum(nviews)
        npts = pts.size(0)
        nimgp = []
        for nip in range(0, len(self.calibImages)):
            if 'pts' in list(self.calibImages.values())[nip].keys():
                nimgp.append(nip)
        objPts = dataObject([len(nimgp), npts, 3], dtype = list(self.calibImages.values())[nimgp[0]]['pts'].dtype)
        imgPts = dataObject([len(nimgp), npts, 2], dtype = list(self.calibImages.values())[nimgp[0]]['pts'].dtype)
        niv = 0
        for nv in range(0, len(nviews)):
            if nviews[nv]:
                objPts[niv, :, :] = list(self.calibImages.values())[nv]['pts'][:,2:5]
                imgPts[niv, :, :] = list(self.calibImages.values())[nv]['pts'][:,0:2]
                niv = niv + 1

        if 'FocalLength' in self.calibImages[list(self.calibImages.keys())[nimgp[0]]]['img'].tags:
            focalLengthExif = float(self.calibImages[list(self.calibImages.keys())[nimgp[0]]]['img'].tags['FocalLength'])
        else:
            focalLengthExif = -1
        if 'Aperture' in self.calibImages[list(self.calibImages.keys())[nimgp[0]]]['img'].tags:
            apertureExif = float(self.calibImages[list(self.calibImages.keys())[nimgp[0]]]['img'].tags['Aperture'])
        else:
            apertureExif = -1

        # seeding camera matrix with reasonable values, if possible
        camMat = dataObject.zeros([3, 3], dtype='float64')
        camMat[0, 2] = list(self.calibImages.values())[0]['img'].size(1) / 2.0
        camMat[1, 2] = list(self.calibImages.values())[0]['img'].size(0) / 2.0
        camMat[2, 2] = 1
        if focalLengthExif > 0 and float(self.gui.dsb_pixelPitchCamera["value"]) > 0:
            camMat[0, 0] = focalLengthExif / float(self.gui.dsb_pixelPitchCamera["value"])
            camMat[1, 1] = focalLengthExif / float(self.gui.dsb_pixelPitchCamera["value"])
        distCoeff = dataObject.zeros([1, 5], dtype='float64')
        rvecs = dataObject.zeros([3, numViews], dtype='float64')
        tvecs = dataObject.zeros([3, numViews], dtype='float64')
        reproErr = filter("cvCalibrateCamera", objPts, imgPts, list(self.calibImages.values())[nimgp[0]]['img'].size(), camMat, distCoeff, rvecs, tvecs)
        focalLengthCalib = (camMat[0, 0] + camMat[1, 1]) / 2 * self.gui.dsb_pixelPitchCamera["value"]

        # calculate average distance of calibration patterns and depth of field
        avgDist = np.mean(tvecs[2, :])

        if apertureExif > 0:
            frontfocalplane = avgDist * focalLengthCalib**2 / (focalLengthCalib**2 - 2 * self.gui.dsb_pixelPitchCamera["value"] \
                * apertureExif * (focalLengthCalib + avgDist))
            backfocalplane = avgDist * focalLengthCalib**2 / (focalLengthCalib**2 + 2 * self.gui.dsb_pixelPitchCamera["value"] \
                * apertureExif * (focalLengthCalib + avgDist))
            depthOfField = np.abs(backfocalplane - frontfocalplane)
        else:
            depthOfField = -1
        avgMagPts = self.sys().CalcAverageMagnification(objPts, imgPts)
        avgMagCalib = avgDist / ((camMat[0, 0] + camMat[1, 1]) / 2.0)

        ui.msgInformation("Calibration result", "Camera Matrix:\n{:>9.2f} {:9.2f} {:9.2f}\n\
{:>9.2f} {:>9.2f} {:>9.2f}\n{:>9.2f} {:>9.2f} {:>9.2f}\n\n\
Distortion Coefficients:\n{:7.4f} {:7.4f} {:7.4f} {:7.4f} {:7.4f}\n\n\
Reprojection Error:\n{:7.5f}\n\nFocal Length (exif): {:7.4f}\nAperture (exif): {:7.4f}\n\
Focal Length (calib): {:7.4f}\n\nAverage Distance: {:9.2f}\n\
Average Magnification (Calib): {:7.4f}\nAverage Magification (Pts): {:7.4f}\n\
Depth of Field (1 pixel defocus): {:7.2f}".format(camMat[0,0], \
            camMat[0, 1], camMat[0, 2], camMat[1, 0], camMat[1, 1], camMat[1, 2], \
            camMat[2, 0], camMat[2, 1], camMat[2, 2], distCoeff[0, 0], distCoeff[0, 1], \
            distCoeff[0, 2], distCoeff[0, 3], distCoeff[0, 4], reproErr, focalLengthExif, apertureExif, focalLengthCalib, \
            avgDist, avgMagCalib, avgMagPts, depthOfField))
        self.sys().cfg["CamMat"] = camMat
        self.sys().cfg["DistCoeff"] = distCoeff
        self.sys().cfg["RVecs"] = rvecs.copy()
        self.sys().cfg["TVecs"] = tvecs.copy()
        self.sys().cfg["CamPixelPitch"]  = self.gui.dsb_pixelPitchCamera["value"]
        self.sys().cfg["CamAperture"]  = apertureExif
        self.sys().cfg["cam.calibFile"] = "unsaved"
        self.sys().loadedCalib = 1

    @ItomUi.autoslot("")
    def on_pb_SaveCalib_pressed(self):
        self.sys().modulePath + self.gui.le_baseFName["text"] +"/"
        if not self.sys().cfg["RVecs"] is None and not self.sys().cfg["TVecs"] is None \
           and not self.sys().cfg["CamMat"] is None and not self.sys().cfg["DistCoeff"] is None:
               calibRes = {'CamMat' : self.sys().cfg["CamMat"], 'DistCoeff' : self.sys().cfg["DistCoeff"], \
                    'RVecs' : self.sys().cfg["RVecs"], 'TVecs' : self.sys().cfg["TVecs"], \
                    'CamPixelPitch' : self.sys().cfg["CamPixelPitch"], 'CamAperture' : self.sys().cfg["CamAperture"]}
               filename = self.gui.ple_selCalibDir["currentPath"] + "\\" + self.gui.le_baseFName["text"] + "_calib" + ".idc"
               saveIDC(filename, calibRes)
               self.sys().cfg["cam.calibFile"] = filename
               ui.msgInformation("Camera Calibration", "Calibration data saved successfully!")
        else:
            ui.msgCritical("Error", "No calibration results to save, run calibration first!")

        self.sys().saveConfig()

    @ItomUi.autoslot("")
    def on_pbGenImages_pressed(self):
        if self.gui.cbInpFileType["currentIndex"] == 0:
            self.sys().data["imageOrig"] = SpeckleIntFkt(self.gui.sbSizeX["value"], self.gui.sbSizeY["value"], self.gui.sbSpeckleSize["value"], 2**self.gui.sbBit["value"] - 1)
        else:
            if not self.gui.pleImageFile is None:
                tmpImg = dataObject()
                nf = self.gui.pleImageFile["currentPath"]
                if nf[-4:] == '.idc':
                    tmpImg = loadIDC(nf)
                    tmpImg = list(tmpImg.values())[0]
                elif nf[-4:].lower() == '.nef':
                    tmpObj = dataObject()
                    filter("loadRawImage", nf, tmpObj, "-o 0 -4 -D -j -T -t 0")
                    # demosaicing
                    tagsSave = tmpObj.tags
                    tmpImg = dataObject(cv2.cvtColor(np.array(tmpObj), cv2.COLOR_BayerRG2GRAY))
                    tmpImg.tags = dict(tagsSave)
                else:
                    filter("loadAnyImage", tmpImg, nf)

                # convert to grayscale image
                if tmpImg.dtype == 'rgba32':
                    filter("cvCvtColor", tmpImg, tmpImg, 7)
                self.sys().data["imageOrig"] = tmpImg.copy()

        self.sys().data["imageDef"] = dataObject()
        defType = self.gui.cbDefType["currentIndex"]
        distCoeff = dataObject.zeros([12, 1], dtype='float64')
        distCoeff[0, 0] = self.gui.dsbD1["value"]
        distCoeff[1, 0] = self.gui.dsbD2["value"]
        distCoeff[2, 0] = self.gui.dsbD3["value"]
        distCoeff[3, 0] = self.gui.dsbD4["value"]
        distCoeff[4, 0] = self.gui.dsbD5["value"]
        distCoeff[5, 0] = self.gui.dsbD6["value"]
        distCoeff[6, 0] = self.gui.dsbD7["value"]
        distCoeff[7, 0] = self.gui.dsbD8["value"]
        distCoeff[8, 0] = self.gui.dsbD9["value"]
        distCoeff[9, 0] = self.gui.dsbD10["value"]
        distCoeff[10, 0] = self.gui.dsbD11["value"]
        distCoeff[11, 0] = self.gui.dsbD12["value"]

        if (not self.gui.cb3DProj["checked"] and self.sys().loadedCalib is None) or \
            (self.gui.cb3DProj["checked"] and self.gui.cbObjectType["currentIndex"] == 0):
            filter("DICGenImages", self.sys().data["imageOrig"], self.sys().data["imageDef"], defType, distCoeff, nu=self.gui.dsbNu["value"], \
                E=self.gui.dsbE["value"], T=self.gui.dsbT["value"], M=self.gui.dsbM["value"], I=self.gui.dsbI["value"], \
                te=self.gui.dsbTe["value"], tr=self.gui.dsbTr["value"], Pa=self.gui.dsbPa["value"], \
                Pc=self.gui.dsbPc["value"])
        else:
            self.sys().data["imageDef"] = self.sys().data["imageOrig"].copy()

        if self.gui.cb3DProj["checked"] and not self.sys().loadedCalib is None:
            useCoordSys = self.gui.cbCoordSysNum["currentIndex"]
            objSX = self.gui.dsbObjSizeX["value"]
            objSY = self.gui.dsbObjSizeY["value"]
            zval = self.gui.dsbZValue["value"]
            imageSX = self.sys().data["imageOrig"].size(1)
            imageSY = self.sys().data["imageOrig"].size(0)

            imgOut = dataObject()
            if self.gui.cbObjectType["currentIndex"] == 0:
                # first calculating undeformed image, Pc = 0
                filter("DICGenProjectedImages", self.sys().data["imageOrig"], self.sys().data["imageOrig"], objSX, objSY, zval, self.sys().cfg["CamMat"], \
                    self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys], \
                    shapeType=0)
                # then calculate deformed image
                filter("DICGenProjectedImages", self.sys().data["imageDef"], self.sys().data["imageDef"], objSX, objSY, zval, self.sys().cfg["CamMat"], \
                    self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys], \
                    shapeType=0)
            elif self.gui.cbObjectType["currentIndex"] == 1:
                # first calculating undeformed image, Pc = 0
                filter("DICGenProjectedImages", self.sys().data["imageOrig"], self.sys().data["imageOrig"], objSX, objSY, zval, self.sys().cfg["CamMat"], \
                    self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys], nu=self.gui.dsbNu3D["value"], \
                    shapeType=1, E=self.gui.dsbE3D["value"], Pc=0.0)
                # then calculate deformed image
                filter("DICGenProjectedImages", self.sys().data["imageDef"], self.sys().data["imageDef"], objSX, objSY, zval, self.sys().cfg["CamMat"], \
                    self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys], nu=self.gui.dsbNu3D["value"], \
                    shapeType=1, E=self.gui.dsbE3D["value"], Pc=self.gui.dsbPc3D["value"])
            else:
                pass
            #filter("DICGenProjectedImages", self.sys().data["imageOrig"], self.sys().data["imageOrig"], objSX, objSY, zval, self.sys().cfg["CamMat"], \
                #self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys])
            #filter("DICGenProjectedImages", self.sys().imageDef, self.sys().imageDef, objSX, objSY, zval, self.sys().cfg["CamMat"], \
                #self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys])
            pass
            #inpPts = dataObject.zeros([imageSY, imageSX, 3], dtype="float32")
            #inPtsX = np.array(np.arange(0, objSX, objSX / imageSX)[np.newaxis], dtype='float32')
            #inPtsY = np.array(np.arange(0, objSY, objSY / imageSY)[np.newaxis], dtype='float32')
            #inPtsOX = np.ones([imageSX, 1], dtype='float32')
            #inPtsOY = np.ones([1, imageSY], dtype='float32')
            #inpPts[:, :, 0] = dataObject(np.dot(inPtsOX, inPtsX))
            #inpPts[:, :, 1] = dataObject(np.dot(inPtsY.transpose(), inPtsOY))
            #inpPts = inpPts.reshape([imageSY * imageSX, 3])

            #imgPts = dataObject()
            #filter("cvProjectPoints", inpPts, imgPts, self.sys().cfg["CamMat"], \
                #self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys])
            ##filter("cvProjectPoints", self.sys().imageDef, self.sys().imageDef, self.sys().cfg["CamMat"], \
                #self.sys().cfg["DistCoeff"], self.sys().cfg["RVecs"][:, useCoordSys], self.sys().cfg["TVecs"][:, useCoordSys])

        self.generateRun = self.generateRun + 1
        self.updateInpImage()

    @ItomUi.autoslot("")
    def on_pbSaveImages_pressed(self):
        #saveFName = self.gui.pleImagePath["currentPath"]
        filename  = ui.getSaveFileName('Select file to load', self.ownDir, 'Itom Dictionary (*.idc);; Matlab (*.mat);; Bitmap (*.bmp);; Tiff Image (*.tiff)')
        if not filename:
            ui.msgCritical("Error", "Select filename first!")
            return
        if not self.sys().data["imageDef"] is None and not self.sys().data["imageOrig"] is None:
            genImages = {'imageOrig' : self.sys().data["imageOrig"], 'imageDef' : self.sys().data["imageDef"]}
            #filename = self.gui.pleImagePath["currentPath"]
            if str.lower(filename[-4:]) == '.bmp':
                fnameOrig = filename[:-4] + "_orig.bmp"
                filter("saveBMP", fnameOrig, self.sys().data["imageOrig"], 'gray')
                fnameDeform = filename[:-4] + "_def.bmp"
                filter("saveBMP", fnameDeform, self.sys().data["imageDef"], 'gray')
            elif str.lower(filename[-4:]) == '.tif':
                fnameOrig = filename[:-4] + "_orig.tif"
                filter("saveTiff", fnameOrig, self.sys().data["imageOrig"], 'gray')
                fnameDeform = filename[:-4] + "_def.tif"
                filter("saveTiff", fnameDeform, self.sys().data["imageDef"], 'gray')
            elif str.lower(filename[-4:]) == '.mat':
                fnameOrig = filename[:-4] + "_orig.mat"
                saveMatlabMat(fnameOrig, self.sys().data["imageOrig"])
                fnameDeform = filename[:-4] + "_def.mat"
                saveMatlabMat(fnameDeform, self.sys().data["imageDef"])
            elif filename[-4:] == '.idc':
                saveIDC(filename, genImages)
        else:
            print('no images generated')

    @ItomUi.autoslot("")
    def on_pbLoadImages_pressed(self):
        filename = ui.getOpenFileName('Select file to load', self.ownDir, 'Itom Dictionary (*.idc);;')
        if not filename:
            pass
        else:
            if str.lower(filename[-4:]) == '.idc':
                tmpImg = loadIDC(filename)
                self.sys().data["imageOrig"] = tmpImg['imageOrig']
                self.sys().data["imageDef"] = tmpImg['imageDef']
                self.updateInpImage()
            else:
                print('only idc files can be loaded')

    @ItomUi.autoslot("ito::Shape")
    def on_pltDisplacement_geometricShapeCurrentChanged(self, shape):
        try:
            self.gui.sbCurX0["value"] = shape.point1[0]
            self.gui.sbCurY0["value"] = shape.point1[1]
            self.gui.sbCurX1["value"] = shape.point2[0]
            self.gui.sbCurY1["value"] = shape.point2[1]
            self.gui.sbCurDx["value"] = abs(shape.point1[0] - shape.point2[0])
            self.gui.sbCurDy["value"] = abs(shape.point1[1] - shape.point2[1])
        except:
            self.gui.sbCurX0["value"] = 0
            self.gui.sbCurY0["value"] = 0
            self.gui.sbCurX1["value"] = 0
            self.gui.sbCurY1["value"] = 0
            self.gui.sbCurDx["value"] = 0
            self.gui.sbCurDy["value"] = 0

    @ItomUi.autoslot("")
    def on_pbGenFields_pressed(self):
        if not self.gui.pltDisplacement["source"] is None:
            imgSizeX = self.gui.pltDisplacement["source"].size(1)
            imgSizeY = self.gui.pltDisplacement["source"].size(0)
            if imgSizeX < 1 or imgSizeY < 1:
                return
            shapes = set()
            numFieldsX = self.gui.sbFieldsX["value"]
            numFieldsY = self.gui.sbFieldsY["value"]
            sizex = int((imgSizeX - 2 * self.gui.sbBorderX["value"]) / numFieldsX)
            sizey = int((imgSizeY - 2 * self.gui.sbBorderY["value"]) / numFieldsY)
            if sizex < 1 or sizey < 1:
                ui.msgCritical("Error", "Fields have meaningless size, check parameters!")

            startx = self.gui.sbBorderX["value"] - self.gui.pltDisplacement["source"].axisOffsets[1]
            starty = self.gui.sbBorderY["value"] - self.gui.pltDisplacement["source"].axisOffsets[0]
            overlx = int(self.gui.sbOverlapX["value"] / 2)
            overly = int(self.gui.sbOverlapY["value"] / 2)
            for ny in range(0, numFieldsY):
                for nx in range(0, numFieldsX):
                    if nx > 0 and nx < numFieldsX - 1:
                        uox = 1
                    else:
                        uox = 0
                    if ny > 0 and ny < numFieldsY - 1:
                        uoy = 1
                    else:
                        uoy = 0
                    shapes.add(shape(8, [startx + nx * sizex - uox * overlx, starty + ny * sizey - uoy * overly], \
                        [startx + nx * sizex - uox * overlx + sizex, starty + ny * sizey - uoy * overly + sizey]))
            self.gui.pltDisplacement.call("setGeometricShapes", list(shapes))
            #self.gui.pltDisplacement["geometryModificationModes"] = 2
            self.generateRun = self.generateRun + 1

    @ItomUi.autoslot("")
    def on_pbClearFields_pressed(self):
        self.gui.pltDisplacement.call("clearGeometricShapes")

    @ItomUi.autoslot("QString")
    def on_pleDisplFolder_currentPathChanged(self, path):
        if (os.path.isdir(path)):
            self.gui.lwDisplFiles.call("clear")
            self.loadedImgListFile = -1
            for file in os.scandir(path):
                if file.is_file() and (str.lower(file.name[-4:]) == '.nef' \
                    or str.lower(file.name[-4:]) == '.bmp' or str.lower(file.name[-4:]) == '.tif' \
                    or str.lower(file.name[-4:]) == '.png' or str.lower(file.name[-4:]) == '.idc' \
                    or str.lower(file.name[-4:]) == '.ido'):
                    self.gui.lwDisplFiles.call("addItem", file.name)

    def loadDisplImages(self, fname):
        tmpObj = dataObject()
        if (fname[-4:].lower() == '.nef'):
            filter("loadRawImage", fname, tmpObj, "-o 0 -4 -D -j -T -t 0")

            if self.gui.rbNefDemosaicing["checked"]:
                # demosaicing
                tagsSave = tmpObj.tags
                tmpObj = dataObject(cv2.cvtColor(np.array(tmpObj), cv2.COLOR_BayerRG2GRAY))
                #tmpObj = dataObject(cv2.cvtColor(np.array(tmpObj), cv2.COLOR_BayerRG2RGB))

                tmpObj.tags = dict(tagsSave)
            elif self.gui.rbNefRed["checked"]:
                tmpObj = dataObject(np.array(tmpObj)[0:-1:2, 0:-1:2])
            elif self.gui.rbNefGreen["checked"]:
                tmpObj = dataObject(np.array(tmpObj)[0:-1:2, 1::2])
            elif self.gui.rbNefBlue["checked"]:
                tmpObj = dataObject(np.array(tmpObj)[1::2, 1::2])
        elif (fname[-4:].lower() == '.idc'):
            tmpObj = loadIDC(fname)[0].copy()
        elif (fname[-4:].lower() == '.ido'):
            filter("loadIDO", tmpObj, fname)
        else:
            filter("loadAnyImage", tmpObj, fname)
        return tmpObj

    def runDisplacementCalc(self, posObj):
        outVec = dataObject()
        outVecCC = dataObject()

        usDic = self.sys().detUsableImgRange(self.sys().data["imageOrig"], self.sys().data["imageDef"])
        initialGuessT = self.gui.cbDisplInitialGuess["currentIndex"]

        if self.gui.cb3DProj["checked"] and self.gui.cbDisplUndistort["checked"] and not self.sys().loadedCalib is None:
            imgOrigTemp = self.sys().data["imageOrig"].copy()
            imgDefTemp = self.sys().data["imageDef"].copy()
            filter("cvUndistort", imgOrigTemp, imgOrigTemp, self.sys().cfg["CamMat"], self.sys().cfg["DistCoeff"])
            filter("cvUndistort", imgDefTemp, imgDefTemp, self.sys().cfg["CamMat"], self.sys().cfg["DistCoeff"])
            if self.gui.cbDisplType["currentIndex"] == 0:
                filter("DICDisplacement", imgOrigTemp[usDic["dO"][0, 0] : usDic["dS"][0, 0], \
                        usDic["dO"][1, 0] : usDic["dS"][1, 0]], \
                        imgDefTemp[usDic["dO"][0, 1] : usDic["dS"][0, 1], \
                        usDic["dO"][1, 1] : usDic["dS"][1, 1]] , posObj, outVec, outVecCC, initialGuessType=initialGuessT, interpolAlgo=1)
            elif self.gui.cbDisplType["currentIndex"] > 0:
                filter("DICDisplacementFF", imgOrigTemp[usDic["dO"][0, 0] : usDic["dS"][0, 0], \
                        usDic["dO"][1, 0] : usDic["dS"][1, 0]], \
                        imgDefTemp[usDic["dO"][0, 1] : usDic["dS"][0, 1], \
                        usDic["dO"][1, 1] : usDic["dS"][1, 1]] , posObj, outVec, outVecCC, initialGuessType=initialGuessT, cellType=self.gui.cbDisplType["currentIndex"] - 1, interpolAlgo=1)
        else:
            if self.gui.cbDisplType["currentIndex"] == 0:
                filter("DICDisplacement", self.sys().data["imageOrig"][usDic["dO"][0, 0] : usDic["dS"][0, 0], \
                        usDic["dO"][1, 0] : usDic["dS"][1, 0]], \
                        self.sys().data["imageDef"][usDic["dO"][0, 1] : usDic["dS"][0, 1], \
                        usDic["dO"][1, 1] : usDic["dS"][1, 1]] , posObj, outVec, outVecCC, initialGuessType=initialGuessT, interpolAlgo=1)
            elif self.gui.cbDisplType["currentIndex"] > 0:
                filter("DICDisplacementFF", self.sys().data["imageOrig"][usDic["dO"][0, 0] : usDic["dS"][0, 0], \
                        usDic["dO"][1, 0] : usDic["dS"][1, 0]], \
                        self.sys().data["imageDef"][usDic["dO"][0, 1] : usDic["dS"][0, 1], \
                        usDic["dO"][1, 1] : usDic["dS"][1, 1]] , posObj, outVec, outVecCC, initialGuessType=initialGuessT, cellType=self.gui.cbDisplType["currentIndex"] - 1, interpolAlgo=1)
                #filter("DICDisplacementFF", self.sys().imageOrig, \
                        #self.sys().imageDef, \
                        #posObj, outVec, outVecCC, initialGuessType=2, cellType=self.gui.cbDisplType["currentIndex"] - 1, interpolAlgo=1)
        if self.sys().data["imageOrig"].axisScales[0] == self.sys().data["imageOrig"].axisScales[1]:
            outVec = outVec * self.sys().data["imageOrig"].axisScales[0]
        return [outVec, outVecCC]

    @ItomUi.autoslot("")
    def on_lwDisplFiles_itemSelectionChanged(self):
        if self.gui.lwDisplFiles["count"] > 0:
            fpath = self.gui.pleDisplFolder["currentPath"]
            lwfiles = self.gui.lwDisplFiles.call("selectedRows")
            if self.loadedImgListFile != lwfiles[0]:
                tmpObj = self.loadDisplImages(fpath + "/" + self.gui.lwDisplFiles.call("item", lwfiles[0]))
                self.gui.pltDisplacement["source"] = tmpObj
                self.loadedImgListFile = lwfiles[0]

    @ItomUi.autoslot("")
    def on_pbRunDisplacement_pressed(self):
        if self.gui.lwDisplFiles["count"] > 2 and len(self.gui.lwDisplFiles.call("selectedRows")) < 2:
            ui.msgWarning("Warning", "More than 2 files available and less than 2 selected.\nSelect at least 2 files to continue")
            return

        if self.gui.lwDisplFiles["count"] < 2 and (self.sys().data["imageOrig"] is None or self.sys().data["imageDef"] is None):
            ui.msgWarning("Warning", "Original / deformed image not set. Etiher load files or generate images")
            return

        runCount = 1
        if self.gui.lwDisplFiles["count"] > 2:
            runCount = self.gui.lwDisplFiles["count"]  - 1

        if self.gui.pleDisplCalibrationFile["currentPath"] != '' and os.path.exists(self.gui.pleDisplCalibrationFile["currentPath"]):
            try:
                calib = loadIDC(self.gui.pleDisplCalibrationFile["currentPath"])
            except:
                ui.msgCritical("Error", "Could not load calibration file, setting scale to 1")
                self.mscale = 1.0

            try:
                self.sys().cfg["CamMat"] = calib["CamMat"]
                self.sys().cfg["DistCoeff"] = calib["DistCoeff"]
                self.sys().cfg["RVecs"] = calib["RVecs"]
                self.sys().cfg["TVecs"] = calib["TVecs"]
                self.sys().cfg["cam.calibFile"] = self.gui.pleDisplCalibrationFile["currentPath"]
                self.sys().saveConfig()
                camMat = calib["CamMat"]
                tVecs = calib["TVecs"]
                rVecs = calib["RVecs"]
                distCoeff = calib["DistCoeff"]
                self.mscale = np.mean(self.sys().cfg["TVecs"][2, :]) / ((self.sys().cfg["CamMat"][0, 0] + self.sys().cfg["CamMat"][1, 1]) / 2.0)
            except:
                pass
        else:
            self.sys().cfg["CamMat"] = None
            self.mscale = 1.0

        for nimg in range(0, runCount):
            if self.gui.lwDisplFiles["count"] > 2:
                fpath = self.gui.pleDisplFolder["currentPath"]
                lwfiles = self.gui.lwDisplFiles.call("selectedRows")
                if nimg == 0:
                    self.sys().data["imageOrig"] = self.loadDisplImages(fpath + "/" + self.gui.lwDisplFiles.call("item", lwfiles[0]))
                    if self.gui.cbDisplUndistort["checked"] and (self.sys().cfg["CamMat"].dtype != None):
                        filter("cvUndistort", self.sys().data["imageOrig"], self.sys().data["imageOrig"], self.sys().cfg["CamMat"], self.sys().cfg["DistCoeff"])
                    if self.gui.cbDisplMScaling["checked"]:
                        self.sys().data["imageOrig"].axisScales = (self.mscale, self.mscale)
                        self.sys().data["imageOrig"].axisUnits = ('mm','mm')

                self.sys().data["imageDef"] = self.loadDisplImages(fpath + "/" + self.gui.lwDisplFiles.call("item", lwfiles[nimg + 1]))
                if self.gui.cbDisplUndistort["checked"] and (self.sys().cfg["CamMat"].dtype != None):
                    filter("cvUndistort", self.sys().data["imageDef"], self.sys().data["imageDef"], self.sys().cfg["CamMat"], self.sys().cfg["DistCoeff"])
                if self.gui.cbDisplMScaling["checked"]:
                    self.sys().data["imageDef"].axisScales = (self.mscale, self.mscale)
                    self.sys().data["imageDef"].axisUnits = ('mm','mm')

                if (self.sys().data["imageOrig"].size(0) != self.sys().data["imageDef"].size(0) or \
                    self.sys().data["imageOrig"].size(1) != self.sys().data["imageDef"].size(1)):
                    ui.msgCritical("Error", "Image size of original {0} and deformed [1] image different, aborting!".format(\
                        self.gui.lwDisplFiles.call("item", lwfiles[0]), self.gui.lwDisplFiles.call("item", lwfiles[nimg + 1])))
                    return
            else:
                lwfiles = []

            shapes = self.gui.pltDisplacement["geometricShapes"]
            if self.sys().data["imageOrig"].size(0) > self.sys().data["imageOrig"].size(1):
                minCellDim = self.sys().data["imageOrig"].size(0)
            else:
                minCellDim = self.sys().data["imageOrig"].size(1)

            outVec = dataObject()
            outVecCC = dataObject()
            if not shapes is None and len(shapes) > 0:
                validShapes = 0
                for ns in range(0, len(shapes)):
                    if shapes[ns].type != 8:
                        continue
                    else:
                        validShapes = validShapes + 1
                #posObj = dataObject.zeros([validShapes, 4], dtype='float64')
                posObj = np.zeros([validShapes, 4], dtype='float64')

                imgSizeX = self.sys().data["imageOrig"].size(1)
                imgSizeY = self.sys().data["imageOrig"].size(0)
                numFieldsX = self.gui.sbFieldsX["value"]
                numFieldsY = self.gui.sbFieldsY["value"]
                sizex = int((imgSizeX - 2 * self.gui.sbBorderX["value"]) / numFieldsX)
                sizey = int((imgSizeY - 2 * self.gui.sbBorderY["value"]) / numFieldsY)
                startx = self.gui.sbBorderX["value"] - self.sys().data["imageOrig"].axisOffsets[1]
                starty = self.gui.sbBorderY["value"] - self.sys().data["imageOrig"].axisOffsets[0]

                #for ny in range(0, numFieldsY):
                    #for nx in range(0, numFieldsX):
                        #if nx > 0 and nx < numFieldsX - 1:
                            #uox = 1
                        #else:
                            #uox = 0
                        #if ny > 0 and ny < numFieldsY - 1:
                            #uoy = 1
                        #else:
                            #uoy = 0
                        #posObj[ny*numFieldsX+nx, 0] = ((startx + nx * sizex) + (startx + nx * sizex + sizex)) / 2.0
                        #posObj[ny*numFieldsX+nx, 1] = ((starty + ny * sizey) + (starty + ny * sizey + sizey)) / 2.0
                        #posObj[ny*numFieldsX+nx, 2] = sizex
                        #posObj[ny*numFieldsX+nx, 3] = sizey

                for ns in range(0, len(shapes)):
                    if shapes[ns].type != 8:
                        continue
                    # cx, cy
                    posObj[ns, 0] = (shapes[ns].point1[0] + shapes[ns].point2[0]) / 2.0 + self.sys().data["imageOrig"].axisOffsets[1]
                    posObj[ns, 1] = (shapes[ns].point1[1] + shapes[ns].point2[1]) / 2.0 + self.sys().data["imageOrig"].axisOffsets[0]
                    # dx, dy
                    posObj[ns, 2] = abs(shapes[ns].point2[0] - shapes[ns].point1[0])
                    posObj[ns, 3] = abs(shapes[ns].point2[1] - shapes[ns].point1[1])
                    if posObj[ns, 2] < minCellDim:
                        minCellDim = posObj[ns, 2]
                    if posObj[ns, 3] < minCellDim:
                        minCellDim = posObj[ns, 3]
                indices = np.argsort(posObj[:,1])
                posObj2 = np.zeros([validShapes, 4], dtype='float64')

                totIdx = 0
                startIdx = 0
                endIdx = 0
                while totIdx < len(indices):
                    #tmpObj =  np.zeros([len(indices), 4], dtype='float64')
                    tmpObj = []
                    colIdx = 0
                    while endIdx < len(indices) and posObj[indices[startIdx], 1] == posObj[indices[endIdx], 1]:
                        if colIdx == 0:
                            tmpObj = posObj[indices[endIdx], :]
                        else:
                            tmpObj = np.vstack([tmpObj, posObj[indices[endIdx], :]])
                        endIdx += 1
                        totIdx += 1
                        colIdx += 1
                    if (len(tmpObj.shape)) > 1:
                        indices2 = np.argsort(tmpObj[:, 0])
                    else:
                        indices2 = np.zeros([1])
                        indices2[0] = 0
                    for nx in range(0, colIdx):
                        posObj2[startIdx + nx, :] = posObj[indices[startIdx + indices2[nx]], :]
                    startIdx = endIdx

                posObj = dataObject(posObj2)
                #self.sys().data["cells"] = posObj
                #self.sys().data["cellsX"] = colIdx
                #self.sys().data["cellsY"] = int(totIdx / colIdx)

                [outVec, outVecCC] = self.runDisplacementCalc(posObj)
                if lwfiles != []:
                    dictkey = self.gui.lwDisplFiles.call("item", lwfiles[0]) \
                        + " -> " + self.gui.lwDisplFiles.call("item", lwfiles[nimg + 1])

                    self.sys().data["displResults"][dictkey] = {"displVecs" : [outVec, outVecCC], "cellsXVecs" : colIdx, "cellsYVecs" : int(totIdx / colIdx), \
                        "cells" : posObj, "imageOrigPath" : fpath + "/" + self.gui.lwDisplFiles.call("item", lwfiles[0]), \
                        "imageDefPath" : fpath + "/" + self.gui.lwDisplFiles.call("item", lwfiles[nimg + 1])}
                else:
                    dictkey = str(self.generateRun)
                    self.sys().data["displResults"][dictkey] = {"displVecs" : [outVec, outVecCC], "cellsXVecs" : colIdx, "cellsYVecs" : int(totIdx / colIdx), \
                        "cells" : posObj, "imageOrigPath" : "", \
                        "imageDefPath" : ""}

                #self.sys().data["displVecs"][dictkey] = [outVec, outVecCC]
                #self.sys().data["cellsXVecs"][dictkey] = colIdx
                #self.sys().data["cellsYVecs"][dictkey] = int(totIdx / colIdx)

            else:
                ui.msgCritical("Error", "No Analysis Mesh / Areas defined, define mesh / areas first!")
                return
            largestShift = self.fillDispTable(outVec)

            if largestShift > 0:
                posAndSizes = dataObject.zeros([outVec.size(0), 4], dtype="int16")
                # subset output
                self.sys().data["displ"] = outVec
                self.sys().data["displCC"] = outVecCC
                if (outVec.size(1) == 12):
                    for nc in range(0, outVec.size(0)):
                        posAndSizes[nc, 0] = posObj[nc, 0] - self.gui.pltDisplacement["source"].axisOffsets[1]
                        posAndSizes[nc, 1] = posObj[nc, 1] - self.gui.pltDisplacement["source"].axisOffsets[0]
                        actLen = np.sqrt(outVec[nc, 0] ** 2 + outVec[nc, 1] ** 2)
                        qlen = actLen / largestShift * minCellDim / 2.0 * 0.95
                        if actLen > 0:
                            posAndSizes[nc, 2] = posAndSizes[nc, 0] + outVec[nc, 0] / actLen * qlen
                            posAndSizes[nc, 3] = posAndSizes[nc, 1] + outVec[nc, 1] / actLen * qlen
                        else:
                            posAndSizes[nc, 2] = 0
                            posAndSizes[nc, 3] = 0
                # fullfield output
                if (outVec.size(1) == 4):
                    for nc in range(0, int(outVec.size(0))):
                        posAndSizes[nc, 0] = outVec[nc, 0]
                        posAndSizes[nc, 1] = outVec[nc, 1]
                        actLen = np.sqrt(outVec[nc, 2] ** 2 + outVec[nc, 3] ** 2)
                        qlen = actLen / largestShift * minCellDim / 2.0 * 0.95
                        if actLen > 0:
                            posAndSizes[nc, 2] = posAndSizes[nc, 0] + outVec[nc, 2] / actLen * qlen
                            posAndSizes[nc, 3] = posAndSizes[nc, 1] + outVec[nc, 3] / actLen * qlen
                        else:
                            posAndSizes[nc, 2] = 0
                            posAndSizes[nc, 3] = 0
                self.sys().showQuivers(posAndSizes)
            else:
                self.sys().showQuivers(dataObject())

    @ItomUi.autoslot("")
    def on_pbRunDeformation_pressed(self):
        keylist = list(self.sys().data["displResults"].keys())
        dictkey = keylist[self.gui.lwDeform["currentRow"]]
        self.sys().data["displ"] = self.sys().data["displResults"][dictkey]["displVecs"][0]
        self.sys().data["displCC"] = self.sys().data["displResults"][dictkey]["displVecs"][1]
        self.sys().data["cellsX"] = self.sys().data["displResults"][dictkey]["cellsXVecs"]
        self.sys().data["cellsY"] = self.sys().data["displResults"][dictkey]["cellsYVecs"]
        if not self.sys().data["displ"] is None and not self.sys().data["cellsX"] is None \
            and not self.sys().data["cellsY"] is None:
            tmpRes = dataObject()
            if self.sys().data["displ"].shape[1] > 4:
                ui.msgWarning("Warning", "Subset displacement calculation used, currently only the two first coefficients are considered in deformation calculation!")
                filter("DICDeformation", self.sys().data["displ"][:, 0:2], self.sys().data["cellsX"], self.sys().data["cellsY"], tmpRes)
            else:
                filter("DICDeformation", self.sys().data["displ"], self.sys().data["cellsX"] + 1, self.sys().data["cellsY"] + 1, tmpRes)
            self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]] = tmpRes
        else:
            ui.msgInformation("Displacement", "No displacement data found, run or load first!")
            return
        pass

        self.gui.tblDeformRes.call("clearContents")
        for nc in range(0, self.gui.tblDeformRes["rowCount"]):
            self.gui.tblDeformRes.call("removeRow", 0)

        for nc in range(0, tmpRes.size(0)):
            self.gui.tblDeformRes.call("insertRow", nc)
            #curDeform = np.sqrt(tmpRes[nc, 0] ** 2 + tmpRes[nc, 1] ** 2)
            #if curDeform > largestDeform:
            #    largestDeform = curDeform
            for nco in range(0, tmpRes.size(1)):
                self.gui.tblDeformRes.call("setItem", nc, nco, tmpRes[nc, nco])

        if not self.sys().data["displ"] is None and not self.sys().data["cellsX"] is None \
            and not self.sys().data["cellsY"] is None \
            and not self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]] is None:
            if self.sys().data["displ"].shape[1] > 2:
                exxmat = self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]][:,0].reshape((self.sys().data["cellsY"], self.sys().data["cellsX"]))
                eyymat = self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]][:,1].reshape((self.sys().data["cellsY"], self.sys().data["cellsX"]))
                exymat = self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]][:,2].reshape((self.sys().data["cellsY"], self.sys().data["cellsX"]))
            else:
                exxmat = self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]][:,0].reshape((self.sys().data["cellsY"]+1, self.sys().data["cellsX"]+1))
                eyymat = self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]][:,1].reshape((self.sys().data["cellsY"]+1, self.sys().data["cellsX"]+1))
                exymat = self.sys().data["deformVecs"][keylist[self.gui.lwDeform["currentRow"]]][:,2].reshape((self.sys().data["cellsY"]+1, self.sys().data["cellsX"]+1))
            cellsVec = self.sys().data["displResults"][dictkey]["cells"].copy()
            cellsVec1 = cellsVec.copy()
            cellsVec[:,0] -= cellsVec[:,2] / 2
            cellsVec[:,1] -= cellsVec[:,3] / 2
            cellsVec1[:,0] += cellsVec1[:,2] / 2
            cellsVec1[:,1] += cellsVec1[:,3] / 2
            minx = np.min(cellsVec[:,0])
            maxx = np.max(cellsVec1[:,0])
            miny = np.min(cellsVec[:,1])
            maxy = np.max(cellsVec1[:,1])
            self.sys().data["exxmat"] = dataObject(cv2.resize(np.array(exxmat), (int(maxy-miny+1), int(maxx-minx+1)), cv2.INTER_CUBIC))
            self.sys().data["eyymat"] = dataObject(cv2.resize(np.array(eyymat), (int(maxy-miny+1), int(maxx-minx+1)), cv2.INTER_CUBIC))
            self.sys().data["exymat"] = dataObject(cv2.resize(np.array(exymat), (int(maxy-miny+1), int(maxx-minx+1)), cv2.INTER_CUBIC))
            self.sys().data["exxmat"].axisOffsets = (-miny, -minx)
            self.sys().data["eyymat"].axisOffsets = (-miny, -minx)
            self.sys().data["exymat"].axisOffsets = (-miny, -minx)

            self.gui.pltDeformation["overlayImage"] = self.sys().data["exxmat"]
            self.gui.pltDeformation["overlayColorMap"]='falseColorIR'
            self.gui.pltDeformation["overlayAlpha"] = 128
            self.gui.cbAnalResult["currentIndex"] = 0
            # self.gui.pltDeformation["overlayColorMap"] = 'red'

    @ItomUi.autoslot("")
    def on_pbMeasSave_pressed(self):
        self.Save()

    @ItomUi.autoslot("")
    def on_pbDisplSave_pressed(self):
        self.Save()

    @ItomUi.autoslot("")
    def on_pbDefSave_pressed(self):
        self.Save()

    def Save(self):
        filename  = ui.getSaveFileName('Select file to load', self.ownDir, 'Itom Dictionary (*.idc)')
        if not filename:
            pass
        else:
            if filename[-4:] != '.idc':
                filename += '.idc'

            saveDic = {'cfg' : self.sys().cfg, 'data' : self.sys().data, 'resultObj' : self.sys().resultObj, 'resultDic' : self.sys().resultDic}
            saveIDC(filename, saveDic)
            ui.msgInformation("Save System Data", "All system data saved successfully!")

        #self.sys().saveConfig()

    @ItomUi.autoslot("")
    def on_pbMeasLoad_pressed(self):
        self.Load()

    @ItomUi.autoslot("")
    def on_pbDisplLoad_pressed(self):
        self.Load()

    @ItomUi.autoslot("")
    def on_pbDefLoad_pressed(self):
        self.Load()

    def Load(self):
        filename = ui.getOpenFileName('Select file to load', self.ownDir, 'Itom Dictionary (*.idc);;')
        if not filename:
            pass
        else:
            if filename[-4:] == '.idc':
                lsys = loadIDC(filename)
                try:
                    self.sys().cfg = lsys["cfg"]
                except:
                    ui.msgWarning("Warning", "No system configuration data found in file!")
                try:
                    self.sys().data = lsys["data"]
                except:
                    ui.msgWarning("Warning", "No system data found in file!")
                try:
                    self.sys().resultObj = lsys["resultObj"]
                except:
                    ui.msgWarning("Warning", "No result object found in file!")
                try:
                    self.sys().resultDic = lsys["resultDic"]
                except:
                    ui.msgWarning("Warning", "No result dictionary found in file!")

                #self.sys().imageOrig = tmpImg['imageOrig']
                #self.sys().imageDef = tmpImg['imageDef']
                self.updateInpImage()
                self.fillDispTable(self.sys().data["displ"])
                if "cells" in self.sys().data and not self.sys().data["cells"] is None:
                    shapes = set()
                    for n in range(0, self.sys().data["cells"].shape[0]):
                        x0 = int(self.sys().data["cells"][n, 0] - self.sys().data["cells"][n, 2] / 2)
                        x1 = int(self.sys().data["cells"][n, 0] + self.sys().data["cells"][n, 2] / 2)
                        y0 = int(self.sys().data["cells"][n, 1] - self.sys().data["cells"][n, 3] / 2)
                        y1 = int(self.sys().data["cells"][n, 1] + self.sys().data["cells"][n, 3] / 2)
                        shapes.add(shape(8, [x0, y0], [x1, y1]))
                    self.gui.pltDisplacement.call("setGeometricShapes", list(shapes))

                self.fillTable()
            else:
                print('only idc files can be loaded')

    @ItomUi.autoslot("")
    def on_pbDisplWorkSpaceExp_pressed(self):
        self.on_WorkSpaceExp()

    @ItomUi.autoslot("")
    def on_pbDefWorkSpaceExp_pressed(self):
        self.on_WorkSpaceExp()

    def on_WorkSpaceExp(self):
        __main__.__dict__["dic"] = {'cfg' : self.sys().cfg, 'data' : self.sys().data}

    @ItomUi.autoslot("")
    def on_pbDisplWorkSpaceImp_pressed(self):
        self.on_WorkSpaceImp()

    @ItomUi.autoslot("")
    def on_pbDefWorkSpaceImp_pressed(self):
        self.on_WorkSpaceImp()

    def on_WorkSpaceImp(self):
        if not __main__.__dict__["dic"] is None:
            self.sys().cfg = __main__.__dict__["dic"]["cfg"]
            self.sys().data = __main__.__dict__["dic"]["data"]
        pass

    def updateGraph(self):
        if (self.gui.pbSpiderLive["checked"] == True and self.upDating == False):
            self.upDating = True
            try:
                self.sys().hbm.getVal(self.sys().liveData)
            except:
                pass
            try:
                self.gui.plotSpiderGraph["source"] = self.sys().liveData
                self.gui.plotSpiderGraph["yAxisInterval"] = self.axisScaleLive
                #self.gui.leAverage["text"] = "{0:.5g}".format(np.sum(self.sys().liveData) / self.sys().liveData.size(1))
                if self.sys().liveData.size(0) >= 1:
                    self.gui.leLastValue4["text"] = "{0:.5g}".format(np.sum(self.sys().liveData[0,:]) \
                    / self.sys().liveData.size(1) * self.sys().cfg["scales"][self.sys().cfg["channels"][0] - 4])
                else:
                    self.gui.leLastValue4["text"] =  ""
                if self.sys().liveData.size(0) >= 2:
                    self.gui.leLastValue5["text"] = "{0:.5g}".format(np.sum(self.sys().liveData[1,:]) \
                    / self.sys().liveData.size(1) * self.sys().cfg["scales"][self.sys().cfg["channels"][1] - 4])
                else:
                    self.gui.leLastValue5["text"] =  ""
                if self.sys().liveData.size(0) >= 3:
                    self.gui.leLastValue6["text"] = "{0:.5g}".format(np.sum(self.sys().liveData[2,:]) \
                    / self.sys().liveData.size(1) * self.sys().cfg["scales"][self.sys().cfg["channels"][2] - 4])
                else:
                    self.gui.leLastValue6["text"] = ""
                if self.sys().liveData.size(0) >= 4:
                    self.gui.leLastValue7["text"] = "{0:.5g}".format(np.sum(self.sys().liveData[3,:]) \
                    / self.sys().liveData.size(1) * self.sys().cfg["scales"][self.sys().cfg["channels"][3] - 4])
                else:
                    self.gui.leLastValue7["text"] =  ""
            except:
                print('error plotting data')
            try:
                self.sys().hbm.acquire()
            except:
                pass
        self.upDating = False

    def plotAutoScale(self):
        if len(self.sys().resultObj.shape) < 1 or self.sys().resultObj.shape[0] < 2:
            return
        if (self.sys().numMeas > 0):
            self.gui.plotSpiderGraph["source"] = self.sys().resultObj[1:, 0:self.sys().numMeas]
        else:
            self.gui.plotSpiderGraph["source"] = self.sys().resultObj[1:, :]
        minVal = min(self.sys().resultObj[1:, :])
        if minVal < 0:
            minVal *= 1.1
        else:
            minVal *= 0.9

        maxVal = max(self.sys().resultObj[1:, :])
        if maxVal < 0:
            maxVal *= 0.9
        else:
            maxVal *= 1.1
        self.gui.plotSpiderGraph["yAxisInterval"] = [minVal, maxVal]

    def fillTable(self):
        self.gui.tbMeas.call("clearContents")
        for nc in range(0, self.gui.tbMeas["rowCount"]):
            self.gui.tbMeas.call("removeRow", 0)
        if (self.gui.cbZero["checked"]):
            if (self.sys().numMeas == 0):
                for no in range(0, self.sys().resultObj.size(1)):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.sys().resultObj.size(0)):
                    for na in range(0, self.sys().resultObj.size(1)):
                        self.gui.tbMeas.call("setItem", na, no, self.sys().resultObj[no, na] - self.sys().resultObj[no, 0])
            else:
                for no in range(0, self.sys().numMeas):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.sys().resultObj.size(0)):
                    for na in range(0, self.sys().numMeas):
                        self.gui.tbMeas.call("setItem", na, no, self.sys().resultObj[no, na] - self.sys().resultObj[no, 0])
        else:
            if (self.sys().numMeas == 0):
                for no in range(0, self.sys().resultObj.size(1)):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.sys().resultObj.size(0)):
                    for na in range(0, self.sys().resultObj.size(1)):
                        self.gui.tbMeas.call("setItem", na, no, self.sys().resultObj[no, na])
            else:
                for no in range(0, self.sys().numMeas):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.sys().resultObj.size(0)):
                    for na in range(0, self.sys().numMeas):
                        self.gui.tbMeas.call("setItem", na, no, self.sys().resultObj[no, na])

    @ItomUi.autoslot("QString")
    def on_cbFrequency_currentIndexChanged(self, text):
        if (int(float(text)) != self.sys().cfg["freq"]):
            self.sys().cfg["freq"] = int(float(text))
            if (self.sys().hbm != None):
                self.sys().hbm.setParam("samplingRate", self.sys().cfg["freq"])
                #self.gui.leStatus["text"] = self.sys().hbm.getParam("status")
            self.gui.leMeasTime["text"] = float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"])

    @ItomUi.autoslot("")
    def on_leSpiderDelay_editingFinished(self):
        try:
            self.sys().cfg["spiderDelay"] = float(self.gui.leSpiderDelay["text"])
        except:
            pass

    @ItomUi.autoslot("")
    def on_leSpiderDelay_returnPressed(self):
        try:
            self.sys().cfg["spiderDelay"] = float(self.gui.leSpiderDelay["text"])
        except:
            pass

    @ItomUi.autoslot("")
    def on_dsbCamDelay_editingFinished(self):
        try:
            self.sys().cfg["camDelay"] = float(self.gui.dsbCamDelay["value"])
        except:
            pass

    @ItomUi.autoslot("")
    def on_leSamples_editingFinished(self):
        try:
            if (int(float(self.gui.leSamples["text"])) != self.sys().cfg["samples"]):
                self.sys().cfg["samples"] = int(float(self.gui.leSamples["text"]))
                if self.sys().hbm != None:
                    self.sys().hbm.setParam("numSamples", self.sys().cfg["samples"])
                    #self.gui.leStatus["text"] = self.sys().hbm.getParam("status")
                self.gui.leMeasTime["text"] = float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"])
        except:
            pass

    @ItomUi.autoslot("")
    def on_leSamples_returnPressed(self):
        if (int(float(self.gui.leSamples["text"])) != self.sys().cfg["samples"]):
            self.sys().cfg["samples"] = int(float(self.gui.leSamples["text"]))
            if self.sys().hbm != None:
                self.sys().hbm.setParam("numSamples", self.sys().cfg["samples"])
                #self.gui.leStatus["text"] = self.sys().hbm.getParam("status")
            self.gui.leMeasTime["text"] = "{0:.3f}".format(float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"]))

    @ItomUi.autoslot("")
    def on_pbReInit_pressed(self):
        #check principal hbm functionality first
        port = int(float(self.gui.lePort["text"]))
        if ((self.sys().cfg["serport"] != port) or (self.sys().ser == None)):
            self.sys().cfg["serport"] = port
            [retval, retstr] = self.sys().initHBM()
            if retval != 0:
                return

        self.sys().cfg["frequency"] = int(float(self.gui.cbFrequency["currentText"]))
        #self.sys().cfg["channels"] = int(float(self.gui.cbChannel["currentText"]))

        self.sys().cfg["bridges"][0] = self.sys().CB2bridge(self.gui.cbBridge4)
        self.sys().cfg["bridges"][1] = self.sys().CB2bridge(self.gui.cbBridge5)
        self.sys().cfg["bridges"][2] = self.sys().CB2bridge(self.gui.cbBridge6)
        self.sys().cfg["bridges"][3] = self.sys().CB2bridge(self.gui.cbBridge7)

        self.sys().cfg["measTypes"][0] = self.gui.cbMeasType4["currentText"]
        self.sys().cfg["measTypes"][1] = self.gui.cbMeasType5["currentText"]
        self.sys().cfg["measTypes"][2] = self.gui.cbMeasType6["currentText"]
        self.sys().cfg["measTypes"][3] = self.gui.cbMeasType7["currentText"]
        # todo need to check if channel type is compatible with selected channel!

        self.sys().cfg["ranges"][0] = self.sys().CB2range(self.gui.cbRange4)
        self.sys().cfg["ranges"][1] = self.sys().CB2range(self.gui.cbRange5)
        self.sys().cfg["ranges"][2] = self.sys().CB2range(self.gui.cbRange6)
        self.sys().cfg["ranges"][3] = self.sys().CB2range(self.gui.cbRange7)

        #self.axisScaleLive = [-0.003 * 1.05, 0.003 * 1.05]
        #self.axisScaleLive = [-0.012 * 1.05, 0.012 * 1.05]
        #self.axisScaleLive = [-0.125 * 1.05, 0.125 * 1.05]
        #self.axisScaleLive = [-0.5 * 1.05, 0.5 * 1.05]
        self.gui.plotSpiderGraph["yAxisInterval"] = self.axisScaleLive

        if (self.sys().cam == None):
            self.sys().initCam()

        try:
            self.sys().prepMeasure()
        except:
            print('error setting hbm parameters please check parameters')
            print(self.sys().hbm.getParam("status"))

    @ItomUi.autoslot("")
    def on_pbMeasStart_pressed(self):
        if self.sys().timer != None:
            self.sys().timer.stop()
            time.sleep(self.sys().cfg["samples"] / self.sys().cfg["freq"])
            self.sys().timer = None
            #self.upDating = True

        if self.sys().timerCam != None:
            self.sys().timerCam.stop()
            self.sys().timerCam = None
            if self.sys().cam != None:
                try:
                    self.sys().cam.stopDevice()
                except:
                    pass

        if (self.sys().isMeasuring == True):
            self.sys().isMeasuring = False
            self.gui.pbSpiderLive["enabled"] = True
            self.gui.pbMeasStart["text"] = "Continue Measurement"
            self.gui.pbMeasStart["styleSheet"] = "background-color: #f06d6f;"
        else:
            self.sys().prepMeasure()
            self.sys().isMeasuring = True

            try:
                self.sys().newMeasVal()
                self.sys().timer = timer(int(float(self.gui.leSpiderDelay["text"]) * 1000.0), self.sys().newMeasVal)
            except:
                ui.msgCritical("Error", "Could not start measurement!")
                print("Error retrieving measurement values, aborting!")
                self.sys().isMeasuring = False
                self.gui.pbMeasStart["styleSheet"] = "background-color: #f06d6f;"
                pass

            try:
                if (self.gui.cbEnableCam["checked"] == True):
                    try:
                        self.sys().cam.startDevice()
                    except:
                        ui.msgCritical("Error", "Could not start camera!")
                    try:
                        self.sys().cam.acquire()
                    except:
                        ui.msgCritical("Error", "Could not capture image, maybe out of focus, or no memory!")
                    if (self.gui.cbDownloadImgs["checked"] == True):
                        if os.path.isdir(self.gui.pleImgsPath["currentPath"]):
                            self.sys().downloadImgsMeas = True
                            self.sys().imgsPathMeas = self.gui.pleImgsPath["currentPath"]
                        else:
                            ui.msgCritical("Error", "Path for saving images invalid. Images will not be downloaded!")
                            print("Path for saving images invalid. Images will not be downloaded!")
                    self.sys().timerCam = timer(int(self.gui.dsbCamDelay["value"] * 1000.0 * 1.0), self.sys().newImage)
            except:
                print("could not start camera")

            self.gui.pbMeasStart["text"] = "Pause Measurement"
            self.gui.pbMeasStart["styleSheet"] = "background-color: #80f07a;"


    @ItomUi.autoslot("")
    def on_pbMeasStop_pressed(self):
        if self.sys().timer != None:
            self.sys().timer.stop()
            time.sleep(float(self.gui.leSpiderDelay["text"]))
            self.sys().timer = None
        if self.sys().timerCam != None:
            self.sys().timerCam.stop()
            self.sys().timerCam = None
        try:
            self.sys().cam.stopDevice()
        except:
            pass
        self.upDating = False
        self.sys().isMeasuring = False
        self.gui.pbMeasStart["styleSheet"] = "background-color: #E1E1E1;"
        self.gui.pbMeasStart["text"] = "Start Measurement"
        self.gui.pbSpiderLive["enabled"] = True

    @ItomUi.autoslot("")
    def on_pbMeasReset_pressed(self):
        self.sys().numMeas = 0
        self.sys().numImg = 0
        self.sys().preAllocSize = 32
        # in doubt allocate for all 4 channels
        self.sys().resultObj = dataObject.zeros([5, self.sys().preAllocSize], dtype='float64')
        self.sys().resultDic = {}
        self.gui.plotSpiderGraph["source"] = dataObject()
        self.gui.tbMeas.call("clearContents")
        for nc in range(0, self.gui.tbMeas["rowCount"]):
            self.gui.tbMeas.call("removeRow", 0)
        self.gui.leLastValue4["text"] = ""
        self.gui.leLastValue5["text"] = ""
        self.gui.leLastValue6["text"] = ""
        self.gui.leLastValue7["text"] = ""
        self.upDating = False
        self.sys().isMeasuring = False

    @ItomUi.autoslot("bool")
    def on_pbSpiderLive_toggled(self, state):
        if self.sys().hbm is None:
            [retval, retstr] = self.sys().initHBM()
            if retval != 0:
                ui.msgCritical("Error", "Could not initialize Spider!\nCheck settings and try again!")
                print("Error: Could not initialize Spider! Check settings and try again!")
                return
        if (self.sys().isMeasuring == True):
            return

        if (state == True):
            if (self.sys().timer != None):
                self.sys().timer.stop()
                self.sys().timer = None
            self.sys().prepMeasure()
            time.sleep(1)
            try:
                self.sys().hbm.acquire()
                self.sys().timer = timer(int(self.sys().cfg["samples"] / self.sys().cfg["freq"] * 1000.0 * 1.0), self.updateGraph)
            except:
                pass
        else:
            if (self.sys().timer != None):
                self.sys().timer.stop()
                self.sys().timer = None
            self.plotAutoScale()

    @ItomUi.autoslot("bool")
    def on_cbEnableCam_toggled(self, state):
        self.sys().cfg["enableCam"] = state
        if (state == True):
            self.gui.dsbCamDelay["enabled"] = True
            self.gui.cbDownloadImgs["enabled"] = True
            self.gui.pleImgsPath["enabled"] = True
        else:
            self.gui.dsbCamDelay["enabled"] = False
            self.gui.cbDownloadImgs["enabled"] = False
            self.gui.pleImgsPath["enabled"] = False

    @ItomUi.autoslot("bool")
    def on_cbEnableSpider_toggled(self, state):
        self.sys().cfg["enableSpider"] = state
        if (state == True):
            self.gui.lePort["enabled"] = True
            self.gui.pbReInit["enabled"] = True
            self.gui.cbFrequency["enabled"] = True
            self.gui.leSamples["enabled"] = True
            self.gui.leMeasTime["enabled"] = True
            self.gui.leSpiderDelay["enabled"] = True
            self.gui.cbChan4["enabled"] = True
            self.gui.cbChan5["enabled"] = True
            self.gui.cbChan6["enabled"] = True
            self.gui.cbChan7["enabled"] = True
            self.gui.cbMeasType4["enabled"] = True
            self.gui.cbMeasType5["enabled"] = True
            self.gui.cbMeasType6["enabled"] = True
            self.gui.cbMeasType7["enabled"] = True
            self.gui.cbBridge4["enabled"] = True
            self.gui.cbBridge5["enabled"] = True
            self.gui.cbBridge6["enabled"] = True
            self.gui.cbBridge7["enabled"] = True
            self.gui.cbRange4["enabled"] = True
            self.gui.cbRange5["enabled"] = True
            self.gui.cbRange6["enabled"] = True
            self.gui.cbRange7["enabled"] = True
            self.gui.dsbScale4["enabled"] = True
            self.gui.dsbScale5["enabled"] = True
            self.gui.dsbScale6["enabled"] = True
            self.gui.dsbScale7["enabled"] = True
        else:
            self.gui.lePort["enabled"] = False
            self.gui.pbReInit["enabled"] = False
            self.gui.cbFrequency["enabled"] = False
            self.gui.leSamples["enabled"] = False
            self.gui.leMeasTime["enabled"] = False
            self.gui.leSpiderDelay["enabled"] = False
            self.gui.cbChan4["enabled"] = False
            self.gui.cbChan5["enabled"] = False
            self.gui.cbChan6["enabled"] = False
            self.gui.cbChan7["enabled"] = False
            self.gui.cbMeasType4["enabled"] = False
            self.gui.cbMeasType5["enabled"] = False
            self.gui.cbMeasType6["enabled"] = False
            self.gui.cbMeasType7["enabled"] = False
            self.gui.cbBridge4["enabled"] = False
            self.gui.cbBridge5["enabled"] = False
            self.gui.cbBridge6["enabled"] = False
            self.gui.cbBridge7["enabled"] = False
            self.gui.cbRange4["enabled"] = False
            self.gui.cbRange5["enabled"] = False
            self.gui.cbRange6["enabled"] = False
            self.gui.cbRange7["enabled"] = False
            self.gui.dsbScale4["enabled"] = False
            self.gui.dsbScale5["enabled"] = False
            self.gui.dsbScale6["enabled"] = False
            self.gui.dsbScale7["enabled"] = False

    @ItomUi.autoslot("bool")
    def on_cbDownloadImgs_toggled(self, state):
        self.sys().cfg["downloadImgs"] = state
        self.gui.pleImgsPath["enabled"] = state

    @ItomUi.autoslot("")
    def on_pbSnapImage_pressed(self):
        if self.sys().cam != None:
            tmpObj = dataObject()
            try:
                self.sys().cam.acquire()
                self.sys().cam.getVal(tmpObj)
            except:
                ui.msgCritical("Error", "Error acquiring image!")

            # demosaicing
            tagsSave = tmpObj.tags
            tmpObj = dataObject(cv2.cvtColor(np.array(tmpObj), cv2.COLOR_BayerRG2GRAY))
            #tmpObj = dataObject(cv2.cvtColor(np.array(tmpObj), cv2.COLOR_BayerRG2RGB))
            tmpObj.tags = dict(tagsSave)

            self.gui.plotCamImage["source"] = tmpObj

    @ItomUi.autoslot("QString")
    def on_pleImgsPath_currentPathChanged(self, pathString):
        self.sys().cfg["imgsPath"] = pathString

    @ItomUi.autoslot("QString")
    def on_cbAnalResult_currentIndexChanged(self, text):
        if text.lower() == "exx":
            self.gui.pltDeformation["overlayImage"] = self.sys().data["exxmat"]
        elif text.lower() == "eyy":
            self.gui.pltDeformation["overlayImage"] = self.sys().data["eyymat"]
        elif text.lower() == "exy":
            self.gui.pltDeformation["overlayImage"] = self.sys().data["exymat"]
        pass
