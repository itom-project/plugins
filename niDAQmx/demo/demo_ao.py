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
    samplingRate=200)

# select two channels.
plugin.setParam("channels", "Dev4/ao0,-10,10;Dev4/ao1,-5,5;Dev4/ao2,-5,5")
plugin.setParam("startTriggerMode", "off")
plugin.setParam("startTriggerSource", "PFI0")
plugin.setParam("startTriggerRisingEdge", 0)

plugin.showToolbox()

plugin.startDevice()

a = dataObject.randN([3,400],'float64')

# It is very important to the samplesPerChannel accordingly,
# since the columns of the dataObject to write will be repeated
# during one setVal operation until the number of samples per
# channel have been written.
plugin.setParam("samplesPerChannel", 400)
plugin.setVal(a)

t = time.time()

while(plugin.getParam("taskStarted") > 0):
    print(".", end = '')
    time.sleep(0.2)
print("done in %.2f s" % (time.time() - t))
plugin.stop()

time.sleep(0.5)


# the setVal command will now block until all 
# 'samplesPerChannel' values have been written
plugin.setParam("setValWaitForFinish", 1)
a = dataObject.randN([3,20000],'float64')
# if a has more columns than 'samplesPerChannel',
# a will be cropped.
plugin.setParam("samplesPerChannel", 2)

t = time.time()
plugin.setVal(a)
print("done in %.2f s" % (time.time() - t))
plugin.stopDevice()