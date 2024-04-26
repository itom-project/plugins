"""Continuous analog output task.

Demo script for sending a series of number to different
channels of a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6323 with 4 analog output (AO)
ports was created and named "Dev4".

Values are always written by a uint8, uint16 or int32 dataObject, that is passed
as argument to the 'setVal' method. This dataObject must be 2-dimensional
and the number of rows must be equal to the number of channels. The int32
object is internally casted to uint32 (however int32 is no valid dataObject
data type). The datatype itself depends on the number of lines of each
selected port. If the port has 0-8 lines, uint8 is required, for 9-16 lines
uint16, else int32.

The number of samples that is written to each channel for each setVal
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
    "analogOutput",
    taskName="analogOutput",
    taskMode="continuous",
    samplingRate=800)

# select three channels.
# The channel description for an analog output
# channel description is: dev-name
plugin.setParam("channels", "Dev4/ao0,-10,10;Dev4/ao1,-5,5;Dev4/ao2,-5,5")

# show the toolbox
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

plugin.startDevice()

a = dataObject.randN([3,800], 'float64')

for i in range(0,3):
    plugin.setVal(a)

    print(f"run {i+1}/3: write for 3 sec", end="")

    for i in range(0, 3):
        print(".", end="")
        time.sleep(1)

    print(" done")
    plugin.stop()
