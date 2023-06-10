import time as timer
if "ser" not in globals():
    ser=dataIO("serialIO",5,9600,"\r\n")
if "daq" not in globals():
    daq=dataIO("HBMSpider8",ser)
# Channel 4, full bridge, 3mV/V
#daq.setParam("aiChParams","4,0,0")
# Channel 4, full bridge 125 mV/V
daq.setParam("aiChParams","4,0,2")
scales=(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0)
offsets=(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
daq.setParam("scale",scales)
daq.setParam("offset",offsets)
# Channel 4, quarter bridge, 3mv/V
#daq.setParam("aiChParams","4,2,0")
# Channel 5 full bridge
#daq.setParam("aiChParams","5,0,0")
####
#daq.setParam("aiChParams","6,2,1")
daq.startDevice()
d=dataObject()

for n in range(0, 200):
    daq.acquire()
    timer.sleep(1)
    daq.getVal(d)
    if n == 0:
        plt = plot(d,"Itom1DQwtPlot")
    else:
        plt[1]["source"] = d
    minVal = min(d)
    if minVal > 0:
        minVal = 0
    plt[1]["yAxisInterval"]  = [minVal, max(d) * 1.1]
