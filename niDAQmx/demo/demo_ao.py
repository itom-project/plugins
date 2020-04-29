import time

plugin=dataIO("NI-DAQmx","analogOutput", taskName = "analogOutput", taskMode = "finite", samplingRate = 800)
plugin.setParam("samplesPerChannel", 800)
plugin.setParam("channels", "Dev1/ao0,-10,10;Dev1/ao1,-5,5")
plugin.setParam("startTriggerMode", "digitalEdge")
plugin.setParam("startTriggerSource", "PFI0")
plugin.setParam("startTriggerRisingEdge", 0)

plugin.startDevice()

a = dataObject.randN([2,100],'float64')
plugin.setVal(a)

while(plugin.getParam("taskStarted") > 0):
    print(".", end = '')
    time.sleep(0.5)
print("done")
plugin.setParam("setValWaitForFinish", 1)


a = dataObject.randN([2,100],'float64')
plugin.setVal(a)
print("done")
plugin.stopDevice()