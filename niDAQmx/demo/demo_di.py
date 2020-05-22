# coding=utf8

"""Finite digital input task.

Demo script for acquiring a finite set of digital
values with a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCI-6220 with 3x8 digital
inputs is used. These inputs are distributed over 3 ports
(ports0, ports1, ports2). Every port has 8 bit (line0-line7).

You can either record an entire port, such that the obtained
number is a bitmask of all lines in the port, or you record
selected lines only. Then, every line is acquired into a
different row of the obtained dataObject.

The channel configuration string for digital input tasks always
follows this pattern:

DeviceName/PortName

or

DeviceName/PortName/LineName

Hint: It depends on the NI DAQ devices, if they allow
integrating different devices into the same measurement
task or not. Many devices do not allow this.
"""

import time

# create a new plugin instance, configured
# as finite digital input task with a freq. of 800 samples per second.
plugin = dataIO("NI-DAQmx", "digitalInput", taskName="myDigitalInputTask",
              taskMode="finite", samplingRate=800)

# print a list of supported channels:
print("Channels:", plugin.getParam("supportedChannels"))

plugin.setParam("channels", "Dev2/port0")

# number of finite samples: 800
plugin.setParam("samplesPerChannel", 800)

# this is the trigger source for the internal clock.
# In this case the PFI1 trigger input of Dev2 is used, however
# it would also be possible to use the "OnboardClock". Only,
# the demo device NI PCI-6220 does not support the OnboardClock here.
plugin.setParam("sampleClockSource", "/Dev2/PFI1")

# configure the task in the device
plugin.startDevice()

# acquire 5x 800 samples with 800 samples / second
a = []

for i in range(0,5):
    t = time.time()
    
    # start the finite task
    plugin.acquire()
    d=dataObject()
    
    # getVal waits for the finite task to be finished and reads out the values.
    plugin.getVal(d)
    a.append(d)
    print(time.time() - t)
plot1(dataObject.dstack(a).squeeze(), properties = {"curveStyle":"Steps"})


# change some parameters on the fly...
plugin.setParam("sampleClockSource", "PFI0")

# do not acquire an entire port, but single lines.
# this leads to an acquired dataObject whose row count is
# equal to the number of connected lines.
plugin.setParam("channels", "Dev2/port0/line6;Dev2/port0/line5;Dev2/port0/line1")
plugin.setParam("samplingRate", 1600)

# the device is still started (however due to the change of channels,
# it was internally stopped and restarted)

for i in range(0,10):
    plugin.acquire()
    plugin.getVal(d)

plot1(d)

# stops and terminates the task
plugin.stopDevice()
