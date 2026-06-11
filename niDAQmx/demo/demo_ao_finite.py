"""Finite analog output task.

Demo script for sending a series of number to different
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
    taskMode="finite",
    samplingRate=200)

# select three channels.
# The channel description for an analog output
# channel description is: dev-name,min-output-voltage,max-output-voltage
plugin.setParam("channels", "Dev4/ao0,-10,10;Dev4/ao1,-5,5;Dev4/ao2,-5,5")

# show the toolbox
plugin.showToolbox()

# 1. First demo: send 2 times 400 samples to all three channels.
#    The setVal command is not blocked and returns directly after
#    having started the write operation. The finish state can
#    be read via getParam("taskStarted")

#    A start hardware trigger (falling digital edge of source PFIO)
#    is set:
plugin.setParam("startTriggerMode", "digitalEdge")
plugin.setParam("startTriggerSource", "PFI0")
plugin.setParam("startTriggerRisingEdge", 0)

# non-blocking setVal command
plugin.setParam("setValWaitForFinish", 0)

plugin.startDevice()

# generate a random object
a = dataObject.randN([3,400],'float64')

# It is very important to the samplesPerChannel accordingly,
# since the columns of the dataObject to write will be repeated
# during one setVal operation until the number of samples per
# channel have been written.
plugin.setParam("samplesPerChannel", 400)

for i in range(0, 2):
    plugin.setVal(a)

    t = time.time()
    print(f"run {i+1}/2 ", end='')

    # check if already finished...
    while(plugin.getParam("taskStarted") > 0):
        print(".", end='')
        time.sleep(0.2)

    print("done in %.2f s" % (time.time() - t))

    # a finite task with more than one sample per channel
    # is automatically stopped at the end. It is not
    # necessary to call stop() again.
    time.sleep(0.5)

# 2. Second demo: send 100x times 1 sample to all three channels.
#    The setVal command will block until every sample has been written.
#    Sending 1 sample per channel is an unbuffer operation. A hardware start
#    trigger is therefore not possible.

# the setVal command will now block until all
# 'samplesPerChannel' values have been written
plugin.setParam("setValWaitForFinish", 1)
plugin.setParam("startTriggerMode", "off")

a = dataObject.randN([3,1],'float64')
# 'samplesPerChannel' must also be equal to 1.
plugin.setParam("samplesPerChannel", 1)

t = time.time()

for i in range(0, 100):
    plugin.setVal(a)

print("done in %.2f s" % (time.time() - t))

# 2. Third demo: send 5 times 200 samples to all three channels.
#    However the input object only has 2 columns, such that this
#    object will be repeated 100 times per setVal command to send all
#    6 requested values per channel.
#    The setVal command is still a blocking command.
#    Hint: Repeating the input dataObject is only possible if it
#    has more than one column. Else a single value, unbuffered,
#    write operation is assumed.
a = dataObject.randN([3, 2], 'float64')
plugin.setParam("samplesPerChannel", 200)

t = time.time()

for i in range(0, 5):
    plugin.setVal(a)

print("done in %.2f s" % (time.time() - t))

plugin.stopDevice()
