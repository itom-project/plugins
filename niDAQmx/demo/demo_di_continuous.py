import time
import numpy as np

plugin=dataIO("NI-DAQmx","digitalInput", taskName = "myDigitalInputTask", taskMode = "continuous", samplingRate = 800)
plugin.setParam("samplesPerChannel", 800)
plugin.setParam("channels", "Dev1/line0;Dev1/line1")

plugin.startDevice()

plugin.acquire()
t = time.time()

alldata = []

for i in range(0,10):
    time.sleep(0.1)
    d = dataObject()
    plugin.getVal(d)
    print(time.time()-t, d.shape)
    alldata.append(d)
    
    
if 0:
    print(time.time()-t)
    time.sleep(2)
    d = dataObject()
    plugin.getVal(d) #should give a 
alldata.append(d)
plugin.stopDevice()

alldata = dataObject(np.hstack(alldata))
#alldata.copyMetaInfo(d, True)
