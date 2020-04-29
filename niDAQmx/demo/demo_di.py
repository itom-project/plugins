plugin=dataIO("NI-DAQmx","digitalInput", taskName = "myDigitalInputTask2", taskMode = "finite", samplingRate = 800)
plugin.setParam("samplesPerChannel", 100)
#print(plugin.getParam("supportedChannels"))
plugin.setParam("sampleClockSource", "/Dev2/PFI1")
plugin.setParam("channels", "Dev2/port0") #;Dev1/port1")

plugin.startDevice()

a = []
import time
for i in range(0,5):
    #plugin.setParam("samplingRate", 1000 + i*100)
    t = time.time()
    plugin.acquire()
    d=dataObject()
    plugin.getVal(d)
    a.append(d)
    print(time.time()-t)
plot1(dataObject.dstack(a).squeeze(), properties = {"curveStyle":"Steps"})

#plugin.stopDevice()

plugin.setParam("sampleClockSource", "PFI0")

plugin.setParam("channels", "Dev2/port0/line6;Dev2/port0/line5;Dev2/port0/line1")
plugin.setParam("samplingRate", 1600)

#plugin.startDevice()

for i in range(0,10):
    plugin.acquire()
    plugin.getVal(d)

plugin.stopDevice()
plot1(d)