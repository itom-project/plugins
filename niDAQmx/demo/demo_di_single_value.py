"""Finite digital input task for single value input.

Demo script for acquiring exactly one digital value
per channel per acquire() command
with a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6321 with 3x8 digital input (DI)
ports was created and named "Dev1".

At the end of this demo, "Dev5" (NI PCI-6520) with 1 digital input
port (port 0) and 1 digital output port (port 1) 0 is used. This device
is very simple and only supports on demand inputs. In this case, this
is a single value acquisition with a software trigger.

The channel configuration string for digital input tasks always
follows this pattern:

DeviceName/PortName

or

DeviceName/PortName/LineName

Hint: It depends on the NI DAQ devices, if they allow
integrating different devices into the same measurement
task or not. Many devices do not allow this.
"""

import time

# Demo 1: Analog input task, finite acquisition
# the sampling rate is unimportant
plugin = dataIO(
    "NI-DAQmx",
    "digitalInput",
    taskName="myTaskName",
    taskMode="finite"
)

plugin.showToolbox()

# A total number of 1 samples should be acquired from each port
plugin.setParam("samplesPerChannel", 1)

# Configure the channels (semicolon-separated list of single channel config strings):
plugin.setParam("channels", "Dev1/port0;Dev1/port1")

# If single values are acquired (samplesPerChannel=1), the start
# trigger must be off, since this acquisition on demand only operates
# upon a software trigger.
plugin.setParam("startTriggerMode", "off")


# after having configured the task, start the device.
# The task is then configured in the device. It will be
# started with plugin.acquire() later.
plugin.startDevice()

a = dataObject.zeros([2, 500], 'uint8')
t = time.time()

# repeat the configured acquisition task 5x.
for i in range(0,500):
    # start the acquisition of the given number of samples per channel.
    plugin.acquire()
    d=dataObject()
    # getVal will return if all samples have been acquired (or timeout)
    plugin.copyVal(a[:, i])

print(time.time()-t)

# plot the acquired values from both channels from the last run.
# the output dataObject already contains the correct axes units,
# descriptions etc...
plot1(a,
      properties={"legendPosition": "Right", "legendTitles": ("DI0", "DI2")})

# stop and remove the configured task
plugin.stopDevice()


# Switch now to the simple device Dev5 and acquire 8 lines from port0
plugin.setParam("channels", "Dev5/port0")

# number of finite samples: 1
plugin.setParam("samplesPerChannel", 1)

# configure the task in the device
plugin.startDevice()

# acquire 500 x 1 samples
a = []
print("acquire 500x 1 sample from Dev5/port0...", end="")
t = time.time()

for i in range(0, 500):
    # start the finite task
    plugin.acquire()
    d = dataObject()

    # getVal waits for the finite task to be finished and reads out the values.
    plugin.copyVal(d)
    a.append(d)

print(" done in %.2f sec" % (time.time() - t))

plot1(dataObject.dstack(a).squeeze(), properties={"curveStyle": "Steps"})

# Switch now to the simple device Dev5 and acquire lines 3 and 6 from port 0
plugin.setParam("channels", "Dev5/port0/line3;Dev5/port0/line6")

# number of finite samples: 1
plugin.setParam("samplesPerChannel", 1)

# configure the task in the device
plugin.startDevice()

# acquire 500 x 1 samples
a = []
print("acquire 500x 1 sample from Dev5/port0/line3 and line6...", end="")
t = time.time()

for i in range(0, 500):
    # start the finite task
    plugin.acquire()
    d = dataObject()

    # getVal waits for the finite task to be finished and reads out the values.
    plugin.copyVal(d)
    a.append(d)

print(" done in %.2f sec" % (time.time() - t))

plot1(dataObject.dstack(a).squeeze(), properties={"curveStyle": "Steps"})
