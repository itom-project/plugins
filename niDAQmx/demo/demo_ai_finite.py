# coding=utf8

"""Finite analog input task.

Demo script for acquiring a finite set of analog
values with a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6321 with 16 analog input (AI)
ports was created and named "Dev1".

Additionally, one other device NI PCI-6111 with 2 analog input (AI)
ports was added in NI-MAX with the name "Dev3".

The channel configuration string for analog input tasks always
follows this pattern:

DeviceName/PortName,ConfigMode,MinVoltage,MaxVoltage

where ConfigMode is an integer from this list
(see also argument terminalConfig from command
DAQmxCreateAIVoltageChan):

DAQmx_Val_Cfg_Default = 0, 
DAQmx_Val_Diff = 1, 
DAQmx_Val_RSE = 2, 
DAQmx_Val_NRSE = 3, 
DAQmx_Val_PseudoDiff = 4

Hint: It depends on the NI DAQ devices, if they allow
integrating different devices into the same measurement
task or not. Many devices do not allow this.
"""

import time

# Demo 1: Analog input task, finite acquisition, 80 samples / sec
plugin = dataIO(
    "NI-DAQmx",
    "analogInput",
    taskName="myTaskName",
    taskMode="finite",
    samplingRate=80
)

plugin.showToolbox()

# A total number of 800 samples should be acquired from each port
plugin.setParam("samplesPerChannel", 800)

# Configure the channels (semicolon-separated list of single channel config strings):
# Ch1: Dev1, AI0, connection type 2 (RSE), -10V..+10V
# Ch2: Dev1, AI2, connection type 0 (Default), -5V..+5V
plugin.setParam("channels", "Dev1/ai0,2,-10.0,10.0;Dev1/ai2,0,-5,5")

# enable a start trigger: here acquisition starts with a falling
# edge on the digital trigger input PFI0 (simulated devices will
# automatically send this trigger).
plugin.setParam("startTriggerMode", "digitalEdge")
plugin.setParam("startTriggerSource", "PFI0")
plugin.setParam("startTriggerRisingEdge", 0)

# enable the on-board clock as continuous trigger
plugin.setParam("sampleClockSource", "OnboardClock")

# after having configured the task, start the device.
# The task is then configured in the device. It will be
# started with plugin.acquire() later.
plugin.startDevice()

a = []

# repeat the configured acquisition task 5x.
for i in range(0,5):
    # modify the sampling rate to different values (Hz)
    plugin.setParam("samplingRate", 1000 + i*100)
    t = time.time()
    # start the acquisition of the given number of samples per channel.
    plugin.acquire()
    d=dataObject()
    # getVal will return if all samples have been acquired (or timeout)
    plugin.getVal(d)
    a.append(d)
    print(time.time()-t)
    time.sleep(0.5)

# plot the acquired values from both channels from the last run.
# the output dataObject already contains the correct axes units,
# descriptions etc...
plot1(a[-1],
      properties={"legendPosition": "Right", "legendTitles": ("AI0", "AI2")})

# stop and remove the configured task
plugin.stopDevice()

# change the analog input task to another channel and
# another trigger condition.
plugin.setParam("channels", "Dev3/ai1,4,-6,8")
plugin.setParam("startTriggerMode", "off")
plugin.setParam("startTriggerSource", "PFI0")
plugin.setParam("startTriggerRisingEdge", 0)

# restart the task and do another finite measurement.
plugin.startDevice()
plugin.acquire()
plugin.getVal(d)
plugin.stopDevice()
plot1(d)