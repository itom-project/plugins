from itom import *
from itom import ui
from itomUi import ItomUi
import time as tim
import math
import pickle
import inspect
import os.path
import numpy as np
import __main__

class hbmMeasureSeries(ItomUi):
    lastFolder = None
    lastEvalFolder = None
    ser = None
    port = 3
    hbm = None
    cam = None
    freq = 1200
    samples = 30
    channels = []
    #ranges = [2, 0, 2, 2]      # settings for lvdt & load cell
    ranges = [2, 2, 2, 2]
    bridges = [0, 0, 0, 0]
    scales = [1, 1, 1, 1]
    measTypes = ["strain", "strain", "strain", "strain"]
    liveData = dataObject()
    seriesData = dataObject()
    resultObj = dataObject()
    resultDic = {}
    hasLiveData = 0
    numMeas = 0
    preAllocSize  = 5
    timer = None
    timerCam = None
    axisScaleLive = [-0.125, 0.125]
    upDating = False
    isMeasuring = False
    cellUpdating = False
    gui = None
    buttonHandle = None

    def __init__(self, systemPath = None):
        try:
            self.lastFolder
        except:
            self.lastFolder = None

        self.upDating = True
        print(os.path.dirname(os.path.realpath(__file__)))

        if(systemPath is None):
            ownFilename = inspect.getfile(inspect.currentframe())
            self.ownDir = os.path.dirname(os.path.realpath(__file__)) #os.path.dirname(ownFilename)
        else:
            self.ownDir = systemPath
        self.targetDir = self.ownDir

        uiFile = os.path.join(self.ownDir, "UI/measureSeriesLTMCam.ui")
        uiFile = os.path.abspath(uiFile)

        ItomUi.__init__(self, uiFile, ui.TYPEWINDOW, childOfMainWindow=True, deleteOnClose=True)

        numChan = 0
        if self.gui.cbChan4["checked"]:
            numChan += 1
        if self.gui.cbChan5["checked"]:
            numChan += 1
        if self.gui.cbChan6["checked"]:
            numChan += 1
        if self.gui.cbChan7["checked"]:
            numChan += 1
        self.resultObj = dataObject([numChan + 1, self.preAllocSize],dtype='float64')
        #if userIsDeveloper() :
        #       ItomUi.__init__(self, uiFile, ui.TYPEWINDOW, childOfMainWindow=True)
        #else:
        #       ItomUi.__init__(self, uiFile, ui.TYPEDOCKWIDGET, childOfMainWindow=True)

    def initHBM(self):
        serialErr = 0
        if (self.hbm != None):
            del self.hbm
        if (self.ser != None):
            del self.ser

        retval = 0
        try:
            self.ser = dataIO("serialIO", self.port, 9600, "\r\n")
        except:
            ui.msgCritical("ERROR", "Serial port not open or HBM not initialized")
            errStr = 'error opening default com-port, need to initialize spider via gui-button'
            print(errStr)
            self.gui.leStatus["text"] = errStr
            serialErr = 1
            self.ser = None
            retval = retval -1
        try:
            if retval == 0:
                self.hbm = dataIO("HBMSpider8", self.ser)
                self.gui.leStatus["text"] = self.hbm.getParam("status")
        except:
            if serialErr == 0:
                errStr = 'error opening default com-port, need to initialize spider via gui-button'
                print(errStr)
                self.gui.leStatus["text"] = errStr
            retval = retval -2
            self.hbm = None
        try:
            self.cam = dataIO("DslrRemote2")
        except:
            ui.msgCritical("ERROR", "Camera could not be opened")
            errStr = 'error opening camera, all camera operations disabled'
            print(errStr)
            self.cam = None
            retval = retval -4
        return retval

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

    def init(self):
        self.upDating = True
        self.initHBM()
        self.gui.leSamples["text"] = self.samples
        self.gui.lePort["text"] = self.port

        self.gui.cbRange4["currentIndex"] = self.ranges[0]
        self.gui.cbRange5["currentIndex"] = self.ranges[1]
        self.gui.cbRange6["currentIndex"] = self.ranges[2]
        self.gui.cbRange7["currentIndex"] = self.ranges[3]

        self.gui.cbBridge4["currentIndex"] = self.bridges[0]
        self.gui.cbBridge5["currentIndex"] = self.bridges[1]
        self.gui.cbBridge6["currentIndex"] = self.bridges[2]
        self.gui.cbBridge7["currentIndex"] = self.bridges[3]

        self.gui.cbFrequency["currentText"] = self.freq
        scaleMin = 0
        scaleMax = 0

        self.range2CB(self.ranges[0], self.gui.cbRange4)
        self.range2CB(self.ranges[1], self.gui.cbRange5)
        self.range2CB(self.ranges[2], self.gui.cbRange6)
        self.range2CB(self.ranges[3], self.gui.cbRange7)

        #self.axisScaleLive = [-0.003 * 1.05, 0.003 * 1.05]
        #self.axisScaleLive = [-0.012 * 1.05, 0.012 * 1.05]
        #self.axisScaleLive = [-0.125 * 1.05, 0.125 * 1.05]
        #self.axisScaleLive = [-0.5 * 1.05, 0.5 * 1.05]

        self.gui.plot["yAxisInterval"] = self.axisScaleLive
        self.upDating = False

        self.gui.leMeasTime["text"] = float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"])
#        if (self.gui != None):
#            self.gui.connect("destroyed()",self.destroyed)

    def show(self,modalLevel = 0):
        '''
        Set up values of the GUI-elementes.

        Parameters
        -----------
        modalLevel: {int}, optional
            Toggle between modal and non-modal execution of the GUI, defaults is 0.

        '''

        try:
            removeButton("SpiderLTMCam", "showGUI")
        except:
            self.buttonHandle = None

        self.buttonHandle = addButton("SpiderLTMCam","showGUI","hbmMeas.show()", "ui/spider.png")
        if self.gui == None:
            uiFile = os.path.join(self.ownDir, "UI/measureSeries.ui")
            uiFile = os.path.abspath(uiFile)
            ItomUi.__init__(self, uiFile, ui.TYPEWINDOW, childOfMainWindow=True, deleteOnClose=True)
        ret = self.gui.show(modalLevel)

    def prepMeasure(self):
        if (self.hbm != None):
            self.hbm.stopDevice()
            self.hbm.getParam("reset")
            numCha = 0
            self.channels.clear()
            if self.gui.cbChan4["checked"]:
                self.channels.append(4)
                numCha += 1
            if self.gui.cbChan5["checked"]:
                self.channels.append(5)
                numCha += 1
            if self.gui.cbChan6["checked"]:
                self.channels.append(6)
                numCha += 1
            if self.gui.cbChan7["checked"]:
                self.channels.append(7)
                numCha += 1
            self.resultObj = dataObject([numCha + 1, self.preAllocSize],dtype='float64')
            for nc in range(0, len(self.channels)):
                self.hbm.setParam("aiChParams", str(self.channels[nc]) + "," + str(self.bridges[self.channels[nc] - 4]) + "," + str(self.ranges[self.channels[nc] - 4]))
            self.scales[0] = self.gui.dsbScale4["value"]
            self.scales[1] = self.gui.dsbScale5["value"]
            self.scales[2] = self.gui.dsbScale6["value"]
            self.scales[3] = self.gui.dsbScale7["value"]
            self.hbm.setParam("samplingRate", self.freq)
            self.hbm.setParam("numSamples", self.samples)
            self.hbm.startDevice()
            self.gui.leStatus["text"] = self.hbm.getParam("status")

    def updateGraph(self):
        if (self.gui.rbLive["checked"] == True and self.upDating == False):
            self.upDating = True
            try:
                self.hbm.getVal(self.liveData)
            except:
                pass
            try:
                self.gui.plot["source"] = self.liveData
                self.gui.plot["yAxisInterval"] = self.axisScaleLive
                #self.gui.leAverage["text"] = "{0:.5g}".format(np.sum(self.liveData) / self.liveData.size(1))
                if self.liveData.size(0) >= 1:
                    self.gui.leLastValue4["text"] = f"{np.sum(self.liveData[0,:]) / self.liveData.size(1) * self.scales[self.channels[0] - 4]:.5g}"
                else:
                    self.gui.leLastValue4["text"] =  ""
                if self.liveData.size(0) >= 2:
                    self.gui.leLastValue5["text"] = f"{np.sum(self.liveData[1,:]) / self.liveData.size(1) * self.scales[self.channels[1] - 4]:.5g}"
                else:
                    self.gui.leLastValue5["text"] =  ""
                if self.liveData.size(0) >= 3:
                    self.gui.leLastValue6["text"] = f"{np.sum(self.liveData[2,:]) / self.liveData.size(1) * self.scales[self.channels[2] - 4]:.5g}"
                else:
                    self.gui.leLastValue6["text"] = ""
                if self.liveData.size(0) >= 4:
                    self.gui.leLastValue7["text"] = f"{np.sum(self.liveData[3,:]) / self.liveData.size(1) * self.scales[self.channels[3] - 4]:.5g}"
                else:
                    self.gui.leLastValue7["text"] =  ""
            except:
                print('error plotting data')
            try:
                self.hbm.acquire()
            except:
                pass
        self.upDating = False

    def plotAutoScale(self):
        if len(self.resultObj.shape) < 1:
            return

        if (self.numMeas > 0):
            self.gui.plot["source"] = self.resultObj[1:, 0:self.numMeas]
        else:
            self.gui.plot["source"] = self.resultObj[1:, :]
        minVal = min(self.resultObj[1:, :])
        if minVal < 0:
            minVal *= 1.1
        else:
            minVal *= 0.9

        maxVal = max(self.resultObj[1:, :])
        if maxVal < 0:
            maxVal *= 0.9
        else:
            maxVal *= 1.1
        self.gui.plot["yAxisInterval"] = [minVal, maxVal]

    def fillTable(self):
        self.gui.tbMeas.call("clearContents")
        for nc in range(0, self.gui.tbMeas["rowCount"]):
            self.gui.tbMeas.call("removeRow", 0)
        if (self.gui.cbZero["checked"]):
            if (self.numMeas == 0):
                for no in range(0, self.resultObj.size(1)):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.resultObj.size(0)):
                    for na in range(0, self.resultObj.size(1)):
                        self.gui.tbMeas.call("setItem", na, no, self.resultObj[no, na] - self.resultObj[no, 0])
            else:
                for no in range(0, self.numMeas):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.resultObj.size(0)):
                    for na in range(0, self.numMeas):
                        self.gui.tbMeas.call("setItem", na, no, self.resultObj[no, na] - self.resultObj[no, 0])
        else:
            if (self.numMeas == 0):
                for no in range(0, self.resultObj.size(1)):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.resultObj.size(0)):
                    for na in range(0, self.resultObj.size(1)):
                        self.gui.tbMeas.call("setItem", na, no, self.resultObj[no, na])
            else:
                for no in range(0, self.numMeas):
                    self.gui.tbMeas.call("insertRow", no)
                for no in range(0, self.resultObj.size(0)):
                    for na in range(0, self.numMeas):
                        self.gui.tbMeas.call("setItem", na, no, self.resultObj[no, na])

    @ItomUi.autoslot("QString")
    def on_cbFrequency_currentIndexChanged(self, text):
        if (int(float(text)) != self.freq):
            self.freq = int(float(text))
            if (self.hbm != None):
                self.hbm.setParam("samplingRate", self.freq)
                self.gui.leStatus["text"] = self.hbm.getParam("status")
            self.gui.leMeasTime["text"] = float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"])

    @ItomUi.autoslot("")
    def on_leSamples_editingFinished(self):
        if (int(float(self.gui.leSamples["text"])) != self.samples):
            self.samples = int(float(self.gui.leSamples["text"]))
            if self.hbm != None:
                self.hbm.setParam("numSamples", self.samples)
                self.gui.leStatus["text"] = self.hbm.getParam("status")
            self.gui.leMeasTime["text"] = float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"])

    @ItomUi.autoslot("")
    def on_leSamples_returnPressed(self):
        if (int(float(self.gui.leSamples["text"])) != self.samples):
            self.samples = int(float(self.gui.leSamples["text"]))
            if self.hbm != None:
                self.hbm.setParam("numSamples", self.samples)
                self.gui.leStatus["text"] = self.hbm.getParam("status")
            self.gui.leMeasTime["text"] = "{:.3f}".format(float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"]))

    @ItomUi.autoslot("")
    def on_pbReInit_pressed(self):
        #check principal hbm functionality first
        port = int(float(self.gui.lePort["text"]))
        if ((self.port != port) or (self.ser == None)):
            self.port = port
            res = self.initHBM()
            if res != 0:
                return

        self.frequency = int(float(self.gui.cbFrequency["currentText"]))
        #self.channel = int(float(self.gui.cbChannel["currentText"]))

        self.bridges[0] = self.CB2bridge(self.gui.cbBridge4)
        self.bridges[1] = self.CB2bridge(self.gui.cbBridge5)
        self.bridges[2] = self.CB2bridge(self.gui.cbBridge6)
        self.bridges[3] = self.CB2bridge(self.gui.cbBridge7)

        self.measTypes[0] = self.gui.cbMeasType4["currentText"]
        self.measTypes[1] = self.gui.cbMeasType5["currentText"]
        self.measTypes[2] = self.gui.cbMeasType6["currentText"]
        self.measTypes[3] = self.gui.cbMeasType7["currentText"]
        # todo need to check if channel type is compatible with selected channel!

        self.ranges[0] = self.CB2range(self.gui.cbRange4)
        self.ranges[1] = self.CB2range(self.gui.cbRange5)
        self.ranges[2] = self.CB2range(self.gui.cbRange6)
        self.ranges[3] = self.CB2range(self.gui.cbRange7)

        #self.axisScaleLive = [-0.003 * 1.05, 0.003 * 1.05]
        #self.axisScaleLive = [-0.012 * 1.05, 0.012 * 1.05]
        #self.axisScaleLive = [-0.125 * 1.05, 0.125 * 1.05]
        #self.axisScaleLive = [-0.5 * 1.05, 0.5 * 1.05]
        self.gui.plot["yAxisInterval"] = self.axisScaleLive

        try:
            self.prepMeasure()
        except:
            print('error setting hbm parameters please check parameters')
            print(self.hbm.getParam("status"))

    @ItomUi.autoslot("bool")
    def on_rbLive_toggled(self, state):
        if (self.isMeasuring == True):
            self.gui.rbOverview["checked"] = True
            return

        if (self.timer != None):
            self.timer.stop()
            self.timer = None
        self.prepMeasure()
        tim.sleep(1)
        try:
            self.hbm.acquire()
            self.timer = timer(int(self.samples / self.freq * 1000.0 * 1.0), self.updateGraph)
        except:
            pass

    @ItomUi.autoslot("bool")
    def on_rbOverview_toggled(self, state):
        if (self.isMeasuring == False):
            if (self.timer != None):
                self.timer.stop()
                self.timer = None
        self.plotAutoScale()

    @ItomUi.autoslot("")
    def on_pbLoad_pressed(self):
        '''
        '''
        # load file
        if self.lastEvalFolder is None:
            filename = itom.ui.getOpenFileName('Measurement data file')
        else:
            filename = itom.ui.getOpenFileName('Measurement data file', self.lastEvalFolder)

        #open the idc-file
        if filename[-3: ] == 'idc':
            try:
                dataTmp = loadIDC(filename)

                if(len(dataTmp.keys()) != 2):
                    raise
                else:
                    print("File loaded successfully\n", filename)
                    try:
                        self.resultDic = dataTmp['dic']
                        self.resultObj = dataTmp['obj']
                        self.plotAutoScale()
                        self.fillTable()
                    except:
                        pass

            except:
                ui.msgCritical("ReadError", "Error in opening idc file", ui.MsgBoxOk)
                raise

        elif filename[-3: ] == 'rpm':
            ui.msgCritical("File Format", "RPM files not supported", ui.MsgBoxOk)
        elif filename[-3: ] == 'mat':
            ui.msgCritical("File Format", "Matlab files not supported", ui.MsgBoxOk)
        elif filename[-3: ] == 'txt':
            ui.msgCritical("File Format", "Txt files not supported", ui.MsgBoxOk)
        else:
            ui.msgCritical("FileName", "File could not be opened", ui.MsgBoxOk)

        self.lastEvalFolder = filename

        return

    @ItomUi.autoslot("")
    def on_pbSave_pressed(self):
        '''
        '''
        # save file
        if self.lastEvalFolder is None:
            filename = itom.ui.getSaveFileName('Measurement data file',filters='*.idc')
        else:
            filename = itom.ui.getSaveFileName('Measurement data file', self.lastEvalFolder,filters='*.idc')

        #open the idc-file
        try:
            if (self.numMeas == 0):
                saveObj = self.resultObj
            else:
                saveObj = dataObject.zeros([self.resultObj.size(0), self.numMeas], dtype=self.resultObj.dtype)
                saveObj[:, :self.numMeas] = self.resultObj[:, :self.numMeas]
            dataTmp = {'dic' : self.resultDic, 'obj' : saveObj}
            if (not filename[-4:] == ".idc"):
                filename += ".idc"
            saveIDC(filename, dataTmp)
            print("File saved successfully\n", filename)
        except:
            pass

    @ItomUi.autoslot("bool")
    def on_cbZero_toggled(self, state):
        self.fillTable()

    def newMeasVal(self):
        if self.upDating == True:
            return
        self.upDating = True
        tmpObj = dataObject()

        try:
            self.hbm.acquire()
            self.resultObj[0, self.numMeas] = tim.clock()
            tim.sleep(self.samples / self.freq)
            self.hbm.getVal(tmpObj)
            for nc in range(0, tmpObj.size(0)):
                self.resultObj[nc + 1, self.numMeas] = np.sum(tmpObj[nc, :] / self.samples * self.scales[self.channels[nc] - 4])

            if tmpObj.size(0) >= 1:
                self.gui.leLastValue4["text"] = f"{self.resultObj[1, self.numMeas]:.5g}"
            else:
                self.gui.leLastValue4["text"] =  ""
            if tmpObj.size(0) >= 2:
                self.gui.leLastValue5["text"] = f"{self.resultObj[2, self.numMeas]:.5g}"
            else:
                self.gui.leLastValue5["text"] =  ""
            if tmpObj.size(0) >= 3:
                self.gui.leLastValue6["text"] = f"{self.resultObj[3, self.numMeas]:.5g}"
            else:
                self.gui.leLastValue6["text"] = ""
            if tmpObj.size(0) >= 4:
                self.gui.leLastValue7["text"] = f"{self.resultObj[4, self.numMeas]:.5g}"
            else:
                self.gui.leLastValue7["text"] =  ""
            self.plotAutoScale()

            if ((self.numMeas + 1) > self.gui.tbMeas["rowCount"]):
                self.gui.tbMeas.call("insertRow", self.numMeas)
            if (not self.gui.cbZero["checked"]):
                self.gui.tbMeas.call("setItem", self.numMeas, 0, self.resultObj[0, self.numMeas])
                for nc in range(0, tmpObj.size(0)):
                    self.gui.tbMeas.call("setItem", self.numMeas, self.channels[nc] - 4 + 1, self.resultObj[nc + 1, self.numMeas])
            else:
                self.gui.tbMeas.call("setItem", self.numMeas, 0, self.resultObj[0, self.numMeas] - self.resultObj[0, 0])
                for nc in range(0, tmpObj.size(0)):
                    self.gui.tbMeas.call("setItem", self.numMeas, self.channels[nc] - 4 + 1, self.resultObj[nc + 1, self.numMeas] - self.resultObj[nc + 1, 0])

            self.numMeas += 1
            if self.numMeas == self.preAllocSize:
                tmp = dataObject.zeros([1 + tmpObj.size(0), self.preAllocSize * 2], dtype='float64')
                tmp[:, 0:self.preAllocSize] = self.resultObj
                self.resultObj = tmp.copy()
                self.preAllocSize *= 2
        except:
            pass
        self.upDating = False

    def newImage(self):
        if not self.cam is None:
            self.cam.acquire()

    @ItomUi.autoslot("")
    def on_pbMeasStart_pressed(self):
        if self.timer != None:
            self.timer.stop()
            tim.sleep(self.samples / self.freq)
            self.timer = None
            #self.upDating = True
        if self.timerCam != None:
            self.timerCam.stop()
            self.timerCam = None
            self.cam.stopDevice()

        self.prepMeasure()
        self.isMeasuring = True
        self.gui.rbOverview["checked"] = True

        try:
            self.newMeasVal()
            self.timer = timer(int(float(self.gui.leDelay["text"]) * 1000.0), self.newMeasVal)
        except:
            pass

        try:
            self.cam.startDevice()
            self.cam.acquire()
            self.timerCam = timer(int(self.gui.dsbCamDelay["value"] * 1000.0 * 1.0), self.newImage)
        except:
            print("could not start camera")

    @ItomUi.autoslot("")
    def on_pbMeasStop_pressed(self):
        if self.timer != None:
            self.timer.stop()
            tim.sleep(float(self.gui.leDelay["text"]))
            self.timer = None
        if self.timerCam != None:
            self.timerCam.stop()
            self.timerCam = None
        self.cam.stopDevice()
        self.upDating = False
        self.isMeasuring = False

    @ItomUi.autoslot("")
    def on_pbReset_pressed(self):
        self.numMeas = 0
        self.preAllocSize = 32
        # in doubt allocate for all 4 channels
        self.resultObj = dataObject.zeros([5, self.preAllocSize], dtype='float64')
        self.resultDic = {}
        self.gui.plot["source"] = dataObject()
        self.gui.tbMeas.call("clearContents")
        for nc in range(0, self.gui.tbMeas["rowCount"]):
            self.gui.tbMeas.call("removeRow", 0)
        self.gui.leLastValue4["text"] = ""
        self.gui.leLastValue5["text"] = ""
        self.gui.leLastValue6["text"] = ""
        self.gui.leLastValue7["text"] = ""
        self.upDating = False
        self.isMeasuring = False

    def __del__(self):
        try:
            if (self.timer != None):
                self.timer.stop()
                tim.sleep(1)
            if self.gui != None:
                del self.gui
            if (self.hbm != None):
                del self.hbm
            if (self.ser != None):
                del self.ser
        except:
            pass

        try:
            removeButton("SpiderLTMCam", "showGUI")
        except:
            print("\n deleting button bar failed")

    @ItomUi.autoslot("")
    def on_Dialog_destroyed(self):
        if (self.timer != None):
            self.timer.stop()
            tim.sleep(1)
            self.timer = None
        if (self.timerCam != None):
            self.timerCam.stop()
            self.timerCam = None
        # if self.cam != None:
            # self.cam.stopDevice()
            # del self.cam
        if self.gui != None:
            del self.gui
        # if (self.hbm != None):
            # del self.hbm
        # if (self.ser != None):
            # del self.ser

if __name__ == "__main__":
    try:
        hbmMeas = hbmMeasureSeries()
        hbmMeas.init()
        pass
    except:
        raise

    hbmMeas.show()
