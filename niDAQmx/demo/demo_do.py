import time

plugin=dataIO("NI-DAQmx","digitalOutput", taskName = "digitalOutput", taskMode = "finite", samplingRate = 800)
plugin.setParam("samplesPerChannel", 800)
plugin.setParam("channels", "Dev4/port0")
#plugin.setParam("startTriggerMode", "digitalEdge")
#plugin.setParam("startTriggerSource", "")
#plugin.setParam("startTriggerRisingEdge", 0)

plugin.startDevice()

a = dataObject.randN([1,100],'int32')
plugin.setVal(a)

while(plugin.getParam("taskStarted") > 0):
    print(".", end = '')
    time.sleep(0.5)
print("done")

plugin.setParam("setValWaitForFinish", 1)
a = dataObject.randN([1,100],'int32')
plugin.setVal(a)
print("done")
plugin.stopDevice()