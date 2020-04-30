# coding=utf8

"""Finite analog output task.

Demo script for sending a series of number to different
channels of a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6321 with 16 analog input (AI)
ports was created and named "Dev1".

If a task is configured with N channels, the value, that is sent
via setVal must have N rows and as many columns than samples to write.

The approach is, that the plugin must be parameterized, then
the task is configured using 'startDevice'. 
"""


import time

# create the plugin instance
plugin = dataIO(
    "NI-DAQmx",
    "analogOutput",
    taskName="analogOutput",
    taskMode="finite",
    samplingRate=800)

# select two channels.
plugin.setParam("channels", "Dev1/ao0,-10,10;Dev1/ao1,-5,5")
plugin.setParam("startTriggerMode", "digitalEdge")
plugin.setParam("startTriggerSource", "PFI0")
plugin.setParam("startTriggerRisingEdge", 0)

plugin.startDevice()

a = dataObject.randN([2,100],'float64')
plugin.setVal(a)

while(plugin.getParam("taskStarted") > 0):
    print(".", end = '')
    plugin.stop()
    time.sleep(0.5)
print("done")
plugin.setParam("setValWaitForFinish", 1)


a = dataObject.randN([2,100],'float64')
plugin.setVal(a)
print("done")
plugin.stopDevice()