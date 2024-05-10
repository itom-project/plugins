"""Finite digital output task.

Demo script for sending a series of number to different
channels of a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6323 with 1 digital port (DO)
ports was created and named "Dev4".

Values are always written by a uint8, uint16 or int32 dataObject, that is passed
as argument to the 'setVal' method. This dataObject must be 2-dimensional
and the number of rows must be equal to the number of channels. The int32
object is internally casted to uint32 (however int32 is no valid dataObject
data type). The datatype itself depends on the number of lines of each
selected port. If the port has 0-8 lines, uint8 is required, for 9-16 lines
uint16, else int32 (usually uint8).

The number of samples that are written to each channel for each setVal
command is mainly determined by the 'samplesPerChannel' argument.

There are two cases to handle:

1. If the dataObject has one column OR 'samplesPerChannel' is equal
   to 1, an unbuffered, single write operation is executed. This
   operation does not allow a hardware based start trigger.

2. If the dataObject has more than one column, a buffered write
   operation is executed and any start trigger source is possible).
   If 'samplesPerChannel' is <= number of columns, the first columns
   will be send, until 'samplesPerChannel' samples have been sent to
   each channel. If its value is bigger than the number of columns,
   the dataObject will be repeated until the desired number of samples
   have been sent.
"""


import time

# create the plugin instance
plugin = dataIO(
    "NI-DAQmx",
    "digitalOutput",
    taskName="digitalOutput",
    taskMode="finite",
    samplingRate=800)

# Select port0 (32 lines from Dev4)
plugin.setParam("channels", "Dev4/port0")

# show the toolbox
plugin.showToolbox()


plugin.setParam("startTriggerMode", "off")
plugin.setParam("setValWaitForFinish", 0)

plugin.startDevice()

# Example 1: Non-blocking setVal command

a = dataObject.randN([1, 100], 'int32')
plugin.setParam("samplesPerChannel", 1000)
plugin.setVal(a)

while(plugin.getParam("taskStarted") > 0):
    print(".", end='')
    time.sleep(0.5)
print("Example 1:done")

# Example 2: blocking setVal command
plugin.setParam("setValWaitForFinish", 1)
a = dataObject.randN([1, 100], 'int32')
plugin.setVal(a)
print("Example 2: done")

# Example 3: single value setVal
plugin.setParam("setValWaitForFinish", 1)
a = dataObject.randN([1,1],'int32')
plugin.setParam("samplesPerChannel", 1)

for i in range(0,100):
    plugin.setVal(a)
print("Example 3: done")

# Example 4: single line output
plugin.setParam("channels", "Dev4/port0/line0")
plugin.setParam("setValWaitForFinish", 1)
a = dataObject.randN([1,100],'int32')
plugin.setParam("samplesPerChannel", 1000)
plugin.setVal(a)
print("Example 4: done")

plugin.stopDevice()
