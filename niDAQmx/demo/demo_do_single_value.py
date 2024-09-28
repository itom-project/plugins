"""Finite digital output task for software triggered single value output.

Demo script for writing exactly one digital value
per channel per setVal() command
with a National Instruments DAQ device.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6321 with 3x8 digital output (DO)
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
    "digitalOutput",
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
plugin.setParam("setValWaitForFinish", 1)


# after having configured the task, start the device.
# The task is then configured in the device. It will be
# started with plugin.acquire() later.
plugin.startDevice()

a = dataObject.randN([2, 1], 'uint8')
t = time.time()

# repeat the configured acquisition task 5x.
for i in range(0,500):
    # start the single value output
    plugin.setVal(a)

print("Write 500 values to two ports in %.2f s" % (time.time() - t))


# stop and remove the configured task
plugin.stopDevice()


# Switch now to the simple device Dev5 and write 8 lines to port1
plugin.setParam("channels", "Dev5/port1")

# number of finite samples: 1
plugin.setParam("samplesPerChannel", 1)

# configure the task in the device
plugin.startDevice()

# write 500 x 1 samples
a = dataObject.randN([1,1], 'uint8')
print("write 500 x 1 sample to Dev5/port1...", end="")
t = time.time()

for i in range(0, 500):
    # start the finite task
    plugin.setVal(a)

    # single value write task is always automatically stopped
    # after setVal.

print(" done in %.2f sec" % (time.time() - t))


# Switch now to the simple device Dev5 and write to lines 3 and 6 from port 1
plugin.setParam("channels", "Dev5/port1/line3;Dev5/port1/line6")

# number of finite samples: 1
plugin.setParam("samplesPerChannel", 1)

# configure the task in the device
plugin.startDevice()

# write 500 x 1 samples
a = dataObject.randN([2, 1], 'uint8')
print("write 500 x 1 sample to Dev5/port1/line3 and line6...", end="")
t = time.time()

for i in range(0, 500):
    # start the finite task
    plugin.setVal(a)

print(" done in %.2f sec" % (time.time() - t))
