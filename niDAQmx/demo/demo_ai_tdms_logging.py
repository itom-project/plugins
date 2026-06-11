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

Data from a continuous task can be obtained by regularly
calling getVal / copyVal or by enabling the TDMS file logging
technique.

Reading TDMS files via Python is possible by the package npTDMS
(https://pypi.org/project/npTDMS).
"""
import time

# initialize the plugin for continuous analog input tasks
plugin = dataIO(
    "NI-DAQmx",
    "analogInput",
    taskName="demoAiContinuous",
    taskMode="continuous",
    samplingRate=10000)

plugin.showToolbox()

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

# Step 1: LoggingMode is a fast logging, in this mode, all acquired samples
# are automatically logged into the given tdms file. You must not use
# getVal or copyVal in this logging mode.
plugin.setParam("loggingMode", 1)

plugin.setParam("loggingFilePath", "D:/temp/demo_ai_continuous.tdms")

# when opening a tdms file in append mode and if the group name
# already exists, a new group with a '#number' suffix will be appended
# to the group name.
plugin.setParam("loggingGroupName", "group1")

# 'open': Always appends data to an existing TDMS file. If it does not exist
#         yet, the task start operation will return with an error.
# 'openOrCreate': Creates a new TDMS file or appends data to the existing one.
# 'createOrReplace' (default): Creates a new TDMS file or replaces an existing
#                              one.
# 'create': Newly creates the TDMS file. If it already exists, a task start
#           operation will return with an error.
plugin.setParam("loggingOperation", "createOrReplace")

# configure the task based on the configurations above.
plugin.startDevice()

for i in range(0, 10):

    t = time.time()
    print(f"Fast, direct logging run {i+1}/10...", end="")
    # start the continuous task again
    plugin.acquire()

    # wait for 1 seconds (data are acquired and stored into the file)
    time.sleep(1)

    # stop the task
    plugin.stop()
    print(" done in %.3f s" % (time.time() - t))



# Step 2: choose another logging type. Usually it is recommended to
# stop the device before changing the logging modes. However,
# it the device is still started if the logging parameters
# will be changed, it will automatically be stopped and restarted
# again.

# switch to loggingMode 2: Here only data that has been received
# via getVal / copyVal is additionally stored in the tdms file
plugin.setParam("loggingMode", 2)
plugin.setParam("loggingFilePath", "D:/temp/demo_ai_continuous2.tdms")
plugin.setParam("loggingOperation", "createOrReplace")


print(f"Simultaneous logging during getVal/copyVal (5sec)...", end="")
t = time.time()

# start the continuous task again
plugin.acquire()

for i in range(0, 10):
    # wait a little bit

    time.sleep(0.5)

    # receive data that is automatically stored in the file, too
    # getVal has to be called faster than the internal buffer of
    # the device will exceed.
    plugin.getVal(dataObject())


# stop the task
plugin.stop()

print(" done in %.2f s" % (time.time() - t))

# stop the device (if there are still running \
# tasks, they will also be stopped here)
plugin.stopDevice()
