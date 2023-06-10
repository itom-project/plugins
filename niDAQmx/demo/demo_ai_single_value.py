# coding=utf8

"""Finite analog input task for single value acquisitions.

Demo script for acquiring exactly one analog value
per channel per acquire() command
with a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6321 with 16 analog input (AI)
ports was created and named "Dev1".

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

# Demo 1: Analog input task, finite acquisition
# the sampling rate is unimportant
plugin = dataIO(
    "NI-DAQmx",
    "analogInput",
    taskName="myTaskName",
    taskMode="finite"
)

plugin.showToolbox()

# A total number of 1 samples should be acquired from each port
plugin.setParam("samplesPerChannel", 1)

# Configure the channels (semicolon-separated list of single channel config strings):
# Ch1: Dev1, AI0, connection type 2 (RSE), -10V..+10V
# Ch2: Dev1, AI2, connection type 0 (Default), -5V..+5V
plugin.setParam("channels", "Dev1/ai0,2,-10.0,10.0;Dev1/ai2,0,-5,5")

# If single values are acquired (samplesPerChannel=1), the start
# trigger must be off, since this acquisition on demand only operates
# upon a software trigger.
plugin.setParam("startTriggerMode", "off")


# after having configured the task, start the device.
# The task is then configured in the device. It will be
# started with plugin.acquire() later.
plugin.startDevice()

a = dataObject.zeros([2, 50], 'float64')
t = time.time()

# repeat the configured acquisition task 5x.
for i in range(0, 50):
    # start the acquisition of the given number of samples per channel.
    plugin.acquire()

    # getVal will return if all samples have been acquired (or timeout)
    plugin.copyVal(a[:, i])

print(time.time()-t)

# plot the acquired values from both channels from the last run.
# the output dataObject already contains the correct axes units,
# descriptions etc...
plot1(a,
      properties={"legendPosition": "Right", "legendTitles": ("AI0", "AI2")})

# stop and remove the configured task
plugin.stopDevice()
