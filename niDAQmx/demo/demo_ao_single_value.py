"""Finite analog output task for single value output.

Demo script for sending single values to different
channels of a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6323 with 4 analog output (AO)
ports was created and named "Dev4".

Values are always written by a float64 dataObject, that is passed
as argument to the 'setVal' method. This dataObject must be 2 dimensional
and the number of rows must be equal to the number of channels.

The number of samples that is written to each channel for each setVal
command is mainly determined by the 'samplesPerChannel' argument.

For creating a single value write, 'samplesPerChannel' must
be 1 such that an unbuffered single write operation is executed upon
each call of 'setVal'. This operation does not allow a hardware based
start trigger.
"""

import time

# create the plugin instance
plugin = dataIO(
    "NI-DAQmx",
    "analogOutput",
    taskName="analogOutput",
    taskMode="finite",
    samplingRate=200)

# select three channels.
# The channel description for an analog output
# channel description is: dev-name,min-output-voltage,max-output-voltage
plugin.setParam("channels", "Dev4/ao0,-10,10;Dev4/ao1,-5,5;Dev4/ao2,-5,5")

# show the toolbox
plugin.showToolbox()

# Single value writes requires a disabled start trigger (software trigger only)
plugin.setParam("startTriggerMode", "off")

# the number of samples per channel must be 1
plugin.setParam("samplesPerChannel", 1)

plugin.startDevice()

# generate a random object
a = dataObject.randN([3,1],'float64')


for i in range(0, 2):

    t = time.time()
    print(f"run {i+1}/2 ... ", end='')

    for i in range(0, 100):
        plugin.setVal(a)

    print("done in %.2f s" % (time.time() - t))

    # a finite task with more than one sample per channel
    # is automatically stopped at the end. It is not
    # necessary to call stop() again.
    time.sleep(0.5)

plugin.stopDevice()
