import matplotlib
matplotlib.use('module://mpl_itom.backend_itomagg')
import matplotlib.pyplot as plt
from itom import *
from itom import ui
from itomUi import ItomUi
import time as tim
import math
import pickle
import inspect
from scipy import stats
import os.path
import numpy as np
import numpy.matlib
import __main__

class hbmMeasureSeries(ItomUi):
    resultDic = {}
    resultObj = None
    lastFolder = None
    lastEvalFolder = ""
    ser = None
    port = 4
    hbm = None
    freq = 1200
    samples = 1200
    channel = 4
    range = 0
    bridge = 0
    measType = "strain"
    liveData = dataObject()
    seriesData = dataObject()
    hasLiveData = 0
    numMeas = 0
    lastSeriesSize = 0
    preAllocSize  = 5
    timer = None
    axisScaleLive = [-0.125, 0.125]
    upDating = False
    cellUpdating = False
    numSeries = 1
    invertSeries = 0
    scale = 23894.0
    offset = 1.2275
    gui = None
    buttonHandle = None

    def __init__(self, systemPath = None):
        try:
            self.lastFolder
        except:
            self.lastFolder = None

        self.resultDic = {}
        self.resultObj = dataObject([self.preAllocSize, self.numSeries + 1],dtype='float64')

        self.upDating = True
        print(os.path.dirname(os.path.realpath(__file__)))

        if(systemPath is None):
            ownFilename = inspect.getfile(inspect.currentframe())
            self.ownDir = os.path.dirname(os.path.realpath(__file__)) #os.path.dirname(ownFilename)
        else:
            self.ownDir = systemPath
        self.targetDir = self.ownDir

        uiFile = os.path.join(self.ownDir, "UI/measureSeries.ui")
        uiFile = os.path.abspath(uiFile)

        ItomUi.__init__(self, uiFile, ui.TYPEWINDOW, childOfMainWindow=True, deleteOnClose=True)
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

        try:
            self.ser = dataIO("serialIO", self.port, 9600, "\r\n")
        except:
            errStr = 'error opening default com-port, need to initialize spider via gui-button'
            print(errStr)
            self.gui.leStatus["text"] = errStr
            serialErr = 1
            self.ser = None
            return -1
        try:
            self.hbm = dataIO("HBMSpider8", self.ser)
            self.gui.leStatus["text"] = self.hbm.getParam("status")
        except:
            if serialErr == 0:
                errStr = 'error opening default com-port, need to initialize spider via gui-button'
                print(errStr)
                self.gui.leStatus["text"] = errStr
                return -2
            self.hbm = None

        self.gui.dsbOffset["value"] = self.offset
        self.gui.dsbScale["value"] = self.scale
        return 0

    def init(self):
        self.upDating = True
        self.initHBM()
        self.gui.leSamples["text"] = self.samples
        self.gui.lePort["text"] = self.port
        self.gui.cbChannel["currentIndex"] = self.channel - 1
        self.gui.cbRange["currentIndex"] = self.range
        self.gui.cbFrequency["currentText"] = self.freq
        self.gui.cbBridge["currentIndex"] = self.bridge
        if self.gui.cbRange["currentText"] == "3 mV/V":
            self.range = 0
            self.axisScaleLive = [-0.003 * 1.05, 0.003 * 1.05]
        if self.gui.cbRange["currentText"] == "12 mV/V":
            self.range = 1
            self.axisScaleLive = [-0.012 * 1.05, 0.012 * 1.05]
        if self.gui.cbRange["currentText"] == "125 mV/V":
            self.range = 2
            self.axisScaleLive = [-0.125 * 1.05, 0.125 * 1.05]
        if self.gui.cbRange["currentText"] == "500 mV/V":
            self.range = 3
            self.axisScaleLive = [-0.5 * 1.05, 0.5 * 1.05]

        self.gui.plot["yAxisInterval"] = self.axisScaleLive
        self.upDating = False
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
            removeButton("Spider", "showGUI")
        except:
            self.buttonHandle = None

        self.buttonHandle = addButton("Spider","showGUI","hbmMeas.show()", "ui/spider.png")
        if self.gui == None:
            uiFile = os.path.join(self.ownDir, "UI/measureSeries.ui")
            uiFile = os.path.abspath(uiFile)
            ItomUi.__init__(self, uiFile, ui.TYPEWINDOW, childOfMainWindow=True, deleteOnClose=True)

        ret = self.gui.show(modalLevel)

    def prepMeasure(self):
        if (self.hbm != None):
            self.hbm.stopDevice()
            self.hbm.getParam("reset")
            self.hbm.setParam("aiChParams", str(self.channel) + "," + str(self.bridge) + "," + str(self.range))
            self.hbm.setParam("samplingRate", self.freq)
            self.hbm.setParam("numSamples", self.samples)
            self.hbm.startDevice()
            self.gui.leStatus["text"] = self.hbm.getParam("status")

    def updateGraph(self):
        if (self.gui.rbLive["checked"] == True and self.upDating == False):
            self.upDating = True
            try:
                self.hbm.getVal(self.liveData)
                if self.gui.cbUseCalib["checked"]:
                    self.liveData = self.liveData * self.gui.dsbScale["value"] - self.gui.dsbOffset["value"]
            except:
                pass
            try:
                self.gui.plot["source"] = self.liveData
                self.gui.plot["yAxisInterval"] = self.axisScaleLive
                self.gui.leAverage["text"] = "{0:.5g}".format(np.sum(self.liveData) / self.liveData.size(1))
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

        hasSetValues = self.resultObj.size(1) > 1
        if self.numMeas > self.lastSeriesSize:
            numVals = self.numMeas
        else:
            numVals = self.lastSeriesSize

        if numVals < 1:
            return
        if hasSetValues:
            self.gui.plot["source"] = self.resultObj[0:self.numMeas, 1 :]
            minVal = min(self.resultObj[:, 1 : ])
        else:
            self.gui.plot["source"] = self.resultObj[0:self.numMeas, 0 :]
            minVal = min(self.resultObj[:, 0 : ])
        if minVal < 0:
            minVal *= 1.1
        else:
            minVal *= 0.9
        if hasSetValues:
            maxVal = max(self.resultObj[:, 1 : ])
        else:
            maxVal = max(self.resultObj[:, 0 : ])
        if maxVal < 0:
            maxVal *= 0.9
        else:
            maxVal *= 1.1
        self.gui.plot["yAxisInterval"] = [minVal, maxVal]

    def fillTable(self):
        self.gui.tbMeas.call("clearContents")
        if (self.numMeas < self.gui.tbMeas["rowCount"]):
            for nr in range(self.numMeas, self.gui.tbMeas["rowCount"]):
                self.gui.tbMeas.call("removeRow", 0)
        else:
            for nr in range(self.gui.tbMeas["rowCount"], self.numMeas):
                self.gui.tbMeas.call("insertRow", nr)
        if (self.numSeries + 1 < self.gui.tbMeas["columnCount"]):
            for nc in range(self.numSeries, self.gui.tbMeas["columnCount"]):
                self.gui.tbMeas.call("removeColumn", 0)
        else:
            for nc in range(self.gui.tbMeas["columnCount"], self.numSeries + 1):
                self.gui.tbMeas.call("insertColumn", nc)
        #for nc in range(0, self.gui.tbMeas["rowCount"]):
            #self.gui.tbMeas.call("removeRow", 0)

        hasNoSetValues = self.resultObj.size(1) < 2
        if (self.gui.cbZero["checked"]):
            if (self.numMeas == 0):
                for no in range(0, self.resultObj.size(1)):
                    for na in range(0, self.resultObj.size(0)):
                        self.gui.tbMeas.call("setItem", na + hasNoSetValues, no, self.resultObj[na, no] - self.resultObj[0, no])
            else:
                for no in range(0, self.resultObj.size(1)):
                    for na in range(0, self.numMeas):
                        self.gui.tbMeas.call("setItem", na + hasNoSetValues, no, self.resultObj[na, no] - self.resultObj[0, no])
        else:
            if (self.numMeas == 0):
                for no in range(0, self.resultObj.size(1)):
                    for na in range(0, self.resultObj.size(0)):
                        self.gui.tbMeas.call("setItem", na + hasNoSetValues, no, self.resultObj[na, no])
            else:
                for no in range(0, self.resultObj.size(1)):
                    for na in range(0, self.numMeas):
                        self.gui.tbMeas.call("setItem", na + hasNoSetValues, no, self.resultObj[na, no])

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
            self.gui.leMeasTime["text"] = "{0:.3f}".format(float(self.gui.leSamples["text"]) / float(self.gui.cbFrequency["currentText"]))

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
        self.channel = int(float(self.gui.cbChannel["currentText"]))

        if self.gui.cbBridge["currentText"] == "Full":
            self.bridge = 0
        elif self.gui.cbBridge["currentText"] == "Half":
            self.bridge = 1
        elif self.gui.cbBridge["currentText"] == "Quarter":
            self.bridge = 2
        self.measType = self.gui.cbMeasType["currentText"]
        # todo need to check if channel type is compatible with selected channel!

        if self.gui.cbRange["currentText"] == "3 mV/V":
            self.range = 0
            self.axisScaleLive = [-0.003 * 1.05, 0.003 * 1.05]
        if self.gui.cbRange["currentText"] == "12 mV/V":
            self.range = 1
            self.axisScaleLive = [-0.012 * 1.05, 0.012 * 1.05]
        if self.gui.cbRange["currentText"] == "125 mV/V":
            self.range = 2
            self.axisScaleLive = [-0.125 * 1.05, 0.125 * 1.05]
        if self.gui.cbRange["currentText"] == "500 mV/V":
            self.range = 3
            self.axisScaleLive = [-0.5 * 1.05, 0.5 * 1.05]
        self.gui.plot["yAxisInterval"] = self.axisScaleLive

        try:
            self.prepMeasure()
        except:
            print('error setting hbm parameters please check parameters')
            print(self.hbm.getParam("status"))

    @ItomUi.autoslot("bool")
    def on_rbLive_toggled(self, state):
        if (self.timer != None):
            self.timer.stop()
        self.prepMeasure()
        tim.sleep(1)
        try:
            self.hbm.acquire()
        except:
            pass
        self.timer = timer(int(self.samples / self.freq * 1000.0 * 1.0), self.updateGraph)

    @ItomUi.autoslot("bool")
    def on_rbOverview_toggled(self, state):
        if (self.timer != None):
            self.timer.stop()
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
                    try:
                        self.resultDic = dataTmp['dic']
                        self.resultObj = dataTmp['obj']
                        self.numMeas = self.resultObj.size(0)
                        self.preAllocSize = self.numMeas
                        self.numSeries = self.resultObj.size(1) - 1
                        self.plotAutoScale()
                        self.fillTable()
                        print("File loaded successfuly\n", filename)
                    except:
                        print("Error loading file\n", filename)
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
            filename = itom.ui.getSaveFileName('Measurement data file',filters='(*.idc);;(*.txt)')
        else:
            filename = itom.ui.getSaveFileName('Measurement data file', self.lastEvalFolder,filters='(*.idc);;(*.txt)')

        #open the idc-file
        try:
            try:
                for no in range(0, len(self.resultDic)):
                    self.resultObj[no, 0] = float(self.gui.tbMeas.call("getItem", no, 0))
            except:
                pass # could not read values from table, maybe table empty
            if self.lastSeriesSize > self.numMeas:
                numVals = self.lastSeriesSize
            else:
                numVals = self.numMeas

            if filename[-4:] == '.txt':
                filter("saveTXT", self.resultObj[:numVals,:], filename, separatorSign="\t", decimalSign=",")
            else:
                dataTmp = {'dic' : self.resultDic, 'obj' : self.resultObj[:numVals,:]}
                if filename[-4:] != '.idc':
                    filename += ".idc"
                saveIDC(filename, dataTmp)

            print("File saved successfuly\n", filename)
        except:
            pass

    @ItomUi.autoslot("")
    def on_pbMeasAdd_pressed(self):
        self.upDating = True
        wasLive = 0
        if self.timer != None:
            self.timer.stop()
            tim.sleep(self.samples / self.freq)
            self.upDating = True
            wasLive = 1
        else:
            self.prepMeasure()

        if self.hbm is None:
            ui.msgCritical("Error", "HBM not initialized!")
            return
        self.hbm.acquire()
        tim.sleep(self.samples / self.freq * 1.5)
        self.hbm.getVal(self.seriesData)
        if self.gui.cbUseCalib["checked"]:
            self.seriesData = self.seriesData * self.gui.dsbScale["value"] - self.gui.dsbOffset["value"]
        self.invertSeries = self.gui.cbSeriesInverted["checked"]

        self.gui.leAverage["text"] = "{0:.5g}".format(np.sum(self.seriesData) / self.seriesData.size(1))
        self.resultDic['s' + str(self.numSeries) + 'n' + str(self.numMeas)] = self.seriesData.copy()
        if self.invertSeries:
            if (self.lastSeriesSize - self.numMeas >= 0):
                insertPos = self.lastSeriesSize - self.numMeas - 1
                self.resultObj[insertPos, self.numSeries] = np.sum(self.seriesData) / self.seriesData.size(1)
            else:
                print('Warning, already reached first value of series, former first value was overwritten')
                ui.msgWarning("Warning", "Already reached first value of series, former first value was overwritten")
                insertPos = 0
                self.resultObj[0, self.numSeries] = np.sum(self.seriesData) / self.seriesData.size(1)
        else:
            insertPos = self.numMeas
            self.resultObj[self.numMeas, self.numSeries] = np.sum(self.seriesData) / self.seriesData.size(1)

        if ((self.numMeas + 1) > self.gui.tbMeas["rowCount"] and self.invertSeries == 0):
            self.gui.tbMeas.call("insertRow", self.numMeas)

        #self.gui.tbMeas.call("setItem", self.numMeas, self.numSeries, self.resultObj[self.numMeas, self.numSeries])

        if self.invertSeries:
            if (not self.gui.cbZero["checked"]):
                for nc in range(0, self.resultObj.size(0)):
                    self.gui.tbMeas.call("setItem", insertPos, self.numSeries, self.resultObj[insertPos, self.numSeries])
            else:
                # this does not make too much sense in this way ... but for the sake of less complexity we ignore it at the moment
                for nc in range(0, self.resultObj.size(0)):
                    self.gui.tbMeas.call("setItem", insertPos, self.numSeries, self.resultObj[insertPos, self.numSeries] - self.resultObj[0, self.numSeries])
        else:
            if (not self.gui.cbZero["checked"]):
                for nc in range(0, self.resultObj.size(0)):
                    self.gui.tbMeas.call("setItem", self.numMeas, self.numSeries, self.resultObj[self.numMeas, self.numSeries])
            else:
                for nc in range(0, self.resultObj.size(0)):
                    self.gui.tbMeas.call("setItem", self.numMeas, self.numSeries, self.resultObj[self.numMeas, self.numSeries] - self.resultObj[0, self.numSeries])

        self.numMeas += 1
        if self.numMeas == self.preAllocSize:
            tmp = dataObject.zeros([self.preAllocSize * 2, 1 + self.numSeries], dtype='float64')
            tmp[0:self.preAllocSize,:] = self.resultObj
            self.resultObj = tmp.copy()
            self.preAllocSize *= 2

        if (self.gui.rbOverview["checked"]):
            self.plotAutoScale()

        if wasLive:
            self.timer.start()
        self.upDating = False

    @ItomUi.autoslot("")
    def on_pbMeasDel_pressed(self):
        curRow = self.gui.tbMeas.call("currentRow")
        curCol = self.gui.tbMeas.call("currentColumn")

        try:
            # todo check when delete error here
            if self.numSeries == 1:
                self.gui.tbMeas.call("removeRow", curRow)
                self.resultObj[curRow : self.preAllocSize - 1, self.numSeries] = self.resultObj[curRow + 1 : , self.numSeries]
                self.resultObj[self.numMeas, self.numSeries] = 0
                del self.resultDic['s' + str(self.numSeries) + 'n' + str(curRow)]
                for ni in range(curRow, self.numMeas - 1):
                    self.resultDic['s' + str(self.numSeries) + 'n' + str(ni)] = self.resultDic['s' + str(self.numSeries) + 'n' + str(ni + 1)]
                self.numMeas -= 1
                del self.resultDic['s' + str(self.numSeries) + 'n' + str(self.numMeas)]
            elif curCol > 0:
                self.resultObj[curRow, curCol] = 0
                del self.resultDic['s' + str(curCol) + 'n' + str(curRow)]
        except:
            print('error deleting value')

        if (self.gui.rbOverview["checked"]):
            self.plotAutoScale

    @ItomUi.autoslot("")
    def on_pbAddSeries_pressed(self):
        self.numSeries += 1
        tmpObj = dataObject.zeros([self.preAllocSize, self.numSeries + 1], dtype='float64')
        tmpObj[:, : self.numSeries] = self.resultObj[:, :]
        self.resultObj = tmpObj.copy()
        self.gui.tbMeas.call("insertColumn", self.numSeries)
        if self.numMeas > self.lastSeriesSize:
            self.lastSeriesSize = self.numMeas
        self.numMeas = 0
        invert = self.gui.cbSeriesInverted["checked"]
        self.gui.cbSeriesInverted["checked"] = not invert
        self.invertSeries = self.gui.cbSeriesInverted["checked"]

    #@ItomUi.autoslot("int, bool")
    #def on_inpImage_plotItemsFinished(self, type, aborted):
        #geometricElements = self.gui.inpImage["geometricElements"]
        #newElement = self.gui.inpImage["geometricElementsCount"]
        #try:
            #tempObj = self.gui.inpImage["source"]
#
        #except:
            #pass

    @ItomUi.autoslot("int,int")
    def on_tbMeas_cellChanged(self,row,col):
        pass
        #if self.cellUpdating == False and self.upDating == False:
            #self.cellUpdating = True
            #if col == 0:
                #self.resultObj[row, col] = float(self.gui.tbMeas.call("getItem", row, col))
            #else:
                #self.gui.tbMeas.call("setItem", row, col, self.resultObj[row, col])
            #pass
            #self.cellUpdating = False

    @ItomUi.autoslot("")
    def on_pbReset_pressed(self):
        self.numMeas = 0
        self.numSeries = 1
        self.preAllocSize = 32
        self.resultObj = dataObject.zeros([self.preAllocSize, 1 + self.numSeries], dtype='float64')
        self.resultDic = {}
        self.gui.plot["source"] = dataObject()
        self.gui.tbMeas.call("clearContents")
        for nc in range(0, self.gui.tbMeas["rowCount"]):
            self.gui.tbMeas.call("removeRow", 0)
        self.gui.leAverage["text"] = ""

    @ItomUi.autoslot("bool")
    def on_cbZero_toggled(self, state):
        self.fillTable()

    @ItomUi.autoslot("int")
    def on_tabHBMMeas_currentChanged(self,index):
        if (index == 1):
            self.gui.tbEval.call("clearContents")
            self.gui.txtEval.call("setText", "")
            if (self.numMeas < self.gui.tbEval["rowCount"]):
                for nr in range(self.numMeas, self.gui.tbEval["rowCount"]):
                    self.gui.tbEval.call("removeRow", 0)
            else:
                for nr in range(self.gui.tbEval["rowCount"], self.numMeas):
                    self.gui.tbEval.call("insertRow", nr)
            if (self.numSeries + 6 < self.gui.tbEval["columnCount"]):
                for nc in range(self.numSeries, self.gui.tbEval["columnCount"]):
                    self.gui.tbEval.call("removeColumn", 0)
            else:
                for nc in range(self.gui.tbEval["columnCount"], self.numSeries + 6):
                    self.gui.tbEval.call("insertColumn", nc)

            if self.numSeries > 1:
                avg = np.sum(self.resultObj[0, 1:]) / self.numSeries
                dp = np.sqrt(np.sum((self.resultObj[0, 1:] - avg) ** 2) / (self.numSeries - 1))
                for nm in range(0, self.numMeas):
                    largeDiff = 0
                    if nm > 0:
                        maxDiff = 0.5 * np.abs(self.resultObj[nm, 0] - self.resultObj[nm - 1, 0])
                    else:
                        maxDiff = 0.5 * np.abs(self.resultObj[nm + 1, 0] - self.resultObj[nm, 0])
                    self.gui.tbEval.call("setItem", nm, 0, self.resultObj[nm, 0])
                    for ns in range(1, self.numSeries + 1):
                        self.gui.tbEval.call("setItem", nm, ns + 5, self.resultObj[nm, ns])
                        if np.abs(self.resultObj[nm, 1] - self.resultObj[nm, ns]) > maxDiff:
                            largeDiff = 1
                            break
                    if largeDiff == 0:
                        # Calculate average and standard deviation for each set value
                        avg = np.sum(self.resultObj[nm, 1:]) / self.numSeries
                        dp = np.sqrt(np.sum((self.resultObj[nm, 1:] - avg) ** 2) / (self.numSeries - 1))
                        self.gui.tbEval.call("setItem", nm, 1, avg)
                        self.gui.tbEval.call("setItem", nm, 2, dp)
                        self.gui.tbEval.call("setItem", nm, 3, dp/np.sqrt(self.numSeries))

                        #Student, n=self.numSeries, p<0.05, 2-tail
                        self.gui.tbEval.call("setItem", nm, 4, dp/np.sqrt(self.numSeries) * stats.t.ppf(1-0.025, self.numSeries - 1))
                        #Student, n=self.numSeries, p<0.01, 2-tail
                        self.gui.tbEval.call("setItem", nm, 5, dp/np.sqrt(self.numSeries) * stats.t.ppf(1-0.005, self.numSeries - 1))

            if self.numSeries > 0 and self.numMeas > 0:
                # Calculate linear regression model
                A = dataObject.ones([self.numMeas * (self.resultObj.shape[1] - 1), 2], dtype=self.resultObj.dtype)
                A[:,0] = self.resultObj[:self.numMeas,1:].trans().reshape([self.numMeas * (self.resultObj.shape[1] - 1), 1])
                y = np.matlib.repmat(self.resultObj[:self.numMeas,0].copy(), self.numSeries, 1)
                fitResult = np.linalg.lstsq(A, y)
                [scale, offset] = fitResult[0]

                # Calculate determination coefficient R^2
                ym = np.sum(self.resultObj[:,0])/self.resultObj.shape[0]
                SStot = np.sum((self.resultObj[:,0] - ym)**2)
                SSreg = 0
                for nv in range(0, self.numMeas):
                    for ns in range(0, self.numSeries):
                        SSreg  = SSreg + (self.resultObj[nv, ns + 1] * scale + offset - self.resultObj[nv, 0]) ** 2
                R2 = 1 - SSreg / SStot
                resStr = "y = {0:.5e} + {1:.5e} * x\nR^2 = {2:7.5f}\n".format(float(offset), float(scale), float(R2))
                self.gui.txtEval.call("setText", resStr)

                # Calculate total variance of fitted model, we have numMeas - 2 degrees of freedom, as we are fitting
                # a linear model
                ym = np.sum(self.resultObj[:self.numMeas,0]) / self.resultObj.shape[0]
                SMtot = np.sqrt(np.sum((self.resultObj[:self.numMeas,0] - ym)**2) / (self.numMeas - 2))
                xm = np.sum(self.resultObj[:self.numMeas,1:]) / (self.numMeas * self.numSeries)
                # Calculate variance for scale variable and its t-statistics (n - 2) dof. Test statistics is of type t, as we have few values
                # Test hypothesis is H0: scale = 0, H1: scale != 0, so we want to reject H0
                SMv = SMtot / (np.sqrt(np.sum((self.resultObj[:self.numMeas,1:] - xm)**2)))
                tSMv = np.abs(scale / SMv)
                tCrit5 = stats.t.ppf(1-0.025, self.numMeas * self.numSeries - 2)
                tCrit1 = stats.t.ppf(1-0.005, self.numMeas * self.numSeries - 2)
                if tSMv > tCrit1:
                    resStr = "t_scale = {0:.2f}\nt_critical_5 = {1:.2f}\n\
t_critical_1 = {2:.2f}\nt_scale > t_critical_1 --> parameter is significant @ 99%\n".format(float(tSMv), float(tCrit5), float(tCrit1))
                elif tSMv > tCrit5:
                    resStr = "t_scale = {0:.2f}\nt_critical_5 = {1:.2f}\n\
t_critical_1 = {2:.2f}\nt_scale > t_critical_5 --> parameter is significant @ 95%\n".format(float(tSMv), float(tCrit5), float(tCrit1))
                else:
                    resStr = "t_scale = {0:.2f}\nt_critical_5 = {1:.2f}\n\
t_critical_1 = {2:.2f}\nt_scale < t_critical_5 --> parameter is NOT significant @ 95%\n".format(float(tSMv), float(tCrit5), float(tCrit1))
                self.gui.txtEval.call("append", "Checking for significance of scale (m):\nTest statistics is of type t, as we have few values.\nTest hypothesis is H0: scale = 0,\nH1: scale != 0,\nso we want to reject H0\n")
                self.gui.txtEval.call("append", resStr)

                fitObj = dataObject(offset + self.resultObj[:self.numMeas,1:] * scale)
                SQR = np.sum((fitObj - ym)**2)
                SQE = 0
                for ns in range(0, self.numSeries):
                    SQE = SQE + np.sum((self.resultObj[:self.numMeas,0] - fitObj[:self.numMeas,ns])**2)
                FVal = SQR / SQE * (self.numSeries * self.numMeas - 2)
                # F statistics for self.numSeries * self.numMeas - 2, 1 dof, test statistics is of type F, as we compare the explained variance
                # (Yf - Ym)^2 to the not explained variance (Yi - Yf)^2
                # Test hypothesis H0: is the data lies on a horizontal line, H1: data is NOT on a horizonzal line, so we want to reject H0
                FCrit5 = stats.f.ppf(0.05, self.numSeries * self.numMeas - 2, 1)
                FCrit1 = stats.f.ppf(0.01, self.numSeries * self.numMeas - 2, 1)
                if FVal > FCrit1:
                    resStr = "F_model = {0:.2f}\nF_critical_5 = {1:.2f}\n\
F_critical_1 = {2:.2f}\nF_model > F_critical_1 --> model is significant @ 99%\n".format(float(FVal), float(FCrit5), float(FCrit1))
                elif FVal > FCrit5:
                    resStr = "F_model = {0:.2f}\nF_critical_5 = {1:.2f}\n\
F_critical_1 = {2:.2f}\nF_model > F_critical_5 --> model is significant @ 95%\n".format(float(FVal), float(FCrit5), float(FCrit1))
                else:
                    resStr = "F_model = {0:.2f}\nF_critical_5 = {1:.2f}\n\
F_critical_1 = {2:.2f}\nF_model < F_critical_5 --> model is NOT significant @ 95%\n".format(float(FVal), float(FCrit5), float(FCrit1))
                self.gui.txtEval.call("append", "Checking for significance of whole model:\nTest statistics is of type F, as we compare\n\
the explained variance (Yf - Ym)^2 to\nthe not explained variance (Yi - Yf)^2\n\
Test hypothesis H0: is the data lies on a horizontal line,\n\
H1: data is NOT on a horizonzal line,\nso we want to reject H0.\n")
                self.gui.txtEval.call("append", resStr)

                # finally calculate the confidence interval for the fitted model
                xm = np.sum(self.resultObj[:self.numMeas,1:]) / (self.numSeries * self.numMeas)
                xd = np.sum((self.resultObj[:self.numMeas,1:] - xm) ** 2)
                se = np.sqrt(SQE / (self.numSeries * self.numMeas - 2))
                tCI1 = stats.t.ppf(1-0.005, self.numMeas * self.numSeries - 2)
                tCI5 = stats.t.ppf(1-0.025, self.numMeas * self.numSeries - 2)

                canvas = self.gui.plotEval
                fig = plt.figure(canvas=canvas)
                ax1 = fig.add_subplot(111)
                for ns in range(0, self.numSeries):
                    ax1.plot(self.resultObj[:self.numMeas,1 + ns], self.resultObj[:self.numMeas, 0], 'x', color='mediumblue')
                minVal = np.min(self.resultObj[:self.numMeas, 1:])
                maxVal = np.max(self.resultObj[:self.numMeas, 1:])
                inc = (maxVal - minVal) / 10.0

                xvals = np.arange(minVal, maxVal + inc, inc)
                yvals = offset + scale * xvals
                ax1.plot(xvals, yvals, '-d', color='black')

                ydvals = se * np.sqrt(1.0/(self.numMeas * self.numSeries) + (xvals - xm) ** 2 / xd)
                yvalsPCI1 = yvals + tCI1 * ydvals
                yvalsMCI1 = yvals - tCI1 * ydvals
                yvalsPCI5 = yvals + tCI5 * ydvals
                yvalsMCI5 = yvals - tCI5 * ydvals
                ax1.plot(xvals, yvalsPCI1, '-.', color='lightcoral')
                ax1.plot(xvals, yvalsMCI1, '-.', color='lightcoral')
                ax1.plot(xvals, yvalsPCI5, '--', color='indianred')
                ax1.plot(xvals, yvalsMCI5, '--', color='indianred')
                plt.show()

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
            removeButton("Spider", "showGUI")
        except:
            print("\n deleting button bar failed")

    @ItomUi.autoslot("")
    def on_Dialog_destroyed(self):
        if (self.timer != None):
            self.timer.stop()
            tim.sleep(1)
        if self.gui != None:
            del self.gui
        #if (self.hbm != None):
            #del self.hbm
        #if (self.ser != None):
            #del self.ser

if __name__ == "__main__":
    try:
        hbmMeas = hbmMeasureSeries()
        hbmMeas.init()
        pass
    except:
        raise

    hbmMeas.show()
