# coding=utf8

import time
import numpy as np

"""Continuous analog input task with optional logging (TDMS files).

Demo script for acquiring a continuous set of analog
values with a National Instruments DAQ device.

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

Data from a continuous task can be obtained by regularily
calling getVal / copyVal or by enabling the TDMS file logging
technique.

Reading TDMS files via Python is possible by the package npTDMS
(https://pypi.org/project/npTDMS).
"""

# initialize the plugin for continuous analog input tasks
plugin = dataIO(
    "NI-DAQmx",
    "analogInput",
    taskName="demoAiContinuous",
    taskMode="continuous",
    samplingRate=10000)

# The NI-DAQ device uses the 'samplesPerChannel' in case of continuous
# tasks to define the internal buffer size. However if the number of
# samples, obtained by 'samplesPerChannel' * noOfChannels is lower
# than the values in the following table, NI-DAQ uses the values from
# the table:
#
# no sampling rate:      10000 samples
# 0 - 100 samples / sec: 1 kS
# 101 - 10000 S/s:       10 kS
# 10001 - 1000000 S/s:   100 kS
# else:                  1 MS
plugin.setParam("samplesPerChannel", 5000)

# if this value is -1, the NI-DAQ device will calculate the internal
# buffer size depending on the samplingRate and the parameter
# 'samplesPerChannel'. Else, the internal buffer size can be overwritten
# by this parameter.
plugin.setParam("bufferSize", -1)

# the readTimeout is important for continuous acquisitions.
# It is considered during getVal/copyVal.
# If it is set to -1.0, each getVal/copyVal command will wait
# until 'samplesPerChannel' samples have been received for each channel.
# This cannot be stopped.
# If it is set to 0.0, getVal/copyVal will always return immediately
# and return up to 'samplesPerChannel' values per channel. The dataObject
# argument can also have less number of columns after these calls.
# Values > 0.0 are considered as real timeout. If the requested
# number of samples per channel are not received within this timeout,
# an error is raised (Status Code: -200284).
# In this example, the immediate return is used, but getVal will be
# called after a certain delay to wait for a certain number of values
# before getting them.
plugin.setParam("readTimeout", 0.0)

# assign some channels
plugin.setParam("channels", "Dev1/ai0,0,-10,10;Dev1/ai1,0,-8,8")

# configure the task based on the configurations above.
plugin.startDevice()

# 1. sub-demo: Start the continuous task and get all new values
#    every 500ms. Update a plot with the new values. Repeat this
#    10 times and then stop the task. Afterwards concatenate all
#    intermediate results and display them in a new plot.
arrays = []

# open an empty 1D plot
[i, h] = plot1(dataObject())

for j in range(0, 2):

    print(f"Run {j+1}/2...", end="")

    # start the task
    plugin.acquire()

    for i in range(0, 5):
        t = time.time()
        # the following sleep must not be too long, since NI raises
        # an exception if the internal intermediate buffer gets full.
        # The programmer has to assure, that this buffer is continuously
        # read out using getVal or copyVal.
        time.sleep(0.5)
        d = dataObject()
        # getVal will only return a reference to the internal object.
        # since we want to store the intermediate result for later
        # processing, we would like to get an unmutable array. Therefore: copyVal.
        plugin.copyVal(d)
        print(f", step {i+1}/5 in %.2f s" % (time.time() - t), end="")
        h["source"] = d  # update the plot
        arrays.append(d)

    # stop the task
    plugin.stop()

    print(" done")

# print the shapes of all subobjects
print([i.shape for i in arrays])

# concatenate the intermediate results and plot them.
total = np.hstack(arrays)
plot1(total)

# 2. sub-demo: It is also possible to enable a logging of the task
#    into NIs own TDMS file format. Then, all values, which are acquired
#    during a running task will be written into the TDMS file. It is
#    also possible to get the values to python during logging (depending
#    on the configuration). However, it is not necessary to continuously
#    getVal/copyVal values in order to not raise a timeout / unsufficient
#    buffer size error.
#
#    The logging is enabled via the parameters 'loggingMode',
#    'loggingFilePath', 'loggingGroupName' and 'loggingOperation':
#
#    loggingMode: 0 -> disable logging
#                 1 -> enable fast mode logging
#                      (no simultaneous read via getVal/copyVal allowed),
#                 2 -> standard logging enabled (getVal/copyVal is possible,
#                      however data will only streamed to file if it has been
#                      obtained via getVal/copyVal).
#    filePath: path to output tdms file.
#    groupName: The name of the group to create within the TDMS file
#    operation (optional, default: createOrReplace):
#                 open: an existing tdms file is opened and new data is appended
#                       Raises an exception during task start if the file
#                       does not exist.
#                 openOrCreate: data is appended to an existing file or a new
#                       file is created.
#                 createOrReplace: always create a new file. An existing one
#                       will be deleted first.
#                 create: create a new file and raises an exception if it
#                       already exists.

plugin.setParam("loggingMode", 1)
plugin.setParam("loggingFilePath", "D:/temp/demo_ai_continuous.tdms")
plugin.setParam("loggingGroupName", "group1")
plugin.setParam("loggingOperation", "createOrReplace")

for i in range(0, 3):
    print(f"logged acquisition {i+1}/3: ", end="")

    # start the continuous task again
    plugin.acquire()

    # wait for 3 seconds (data are acquired and stored into the file)
    for j in range(0, 3):
        print(".", end="")
        time.sleep(1)

    # stop the task
    plugin.stop()

    print(" done")

# stop the device (if there are still running \
# tasks, they will also be stopped here)
plugin.stopDevice()
