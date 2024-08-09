"""Finite analog input task with a reference trigger.

Demo script for acquiring a finite (but unknown) number of analog
values with a National Instruments DAQ device, where both the start
and end of the acquisition is given by triggers.

To test this script, the NI MAX (Measurement & Automation
Explorer) has been used to create simulated devices.

In this test, a simulated device NI PCIe-6321 with 16 analog input (AI)
ports was created and named "Dev1".

The start trigger can watch a raising or falling edge of an analog
or digital signal. If an analog signal is chosen, a certain threshold
value has to be given, too (see parameter 'startTriggerLevel').

The stop trigger is given by a so called reference trigger. This
can only be enabled for finite, input tasks. However, such a trigger
will implicitly let the finite task behave like a continuous task.
This means, that you have to continuously retrieve the newest data using
'getVal' or 'copyVal' such that the internal buffer does not overflow.
The stop event for the task is defined by three conditions, that have
to be met: At first, a certain number of samples (refTriggerPreTriggerSamples)
have to be acquired, before the raising or falling edge of the given
refTriggerSource is monitored. Then, this source must have the requested
signal change. Once, this change is detected, the task will record further
samples, whose number is called postTriggerSamples. They are calculated by
"samplesPerChannel" - "refTriggerPreTriggerSamples". Then, the task is
stopped and the parameter "taskStarted" becomes 0.

Hint: It depends on the NI DAQ devices, if they allow
integrating different devices into the same measurement
task or not. Many devices do not allow this.

Hint: The reference trigger could only be tested by the developer
by a simulated NI device. This immediately fires the refTriggerSources, such
that a 100% testing could not be executed.
"""

import time

# Demo 1: Analog input task, finite acquisition, 80 samples / sec
plugin = dataIO(
    "NI-DAQmx",
    "analogInput",
    taskName="myTaskName",
    taskMode="finite",
    samplingRate=200
)

plugin.showToolbox()

# Each getVal / copyVal command will retrieve 800 samples per
# channel. This is also the number used to calculate the post-trigger
# samples ("samplesPerChannel" - "refTriggerPreTriggerSamples")
plugin.setParam("samplesPerChannel", 800)

# Configure the channels:
plugin.setParam("channels", "Dev1/ai0,2,-10.0,10.0;Dev1/ai2,0,-5,5")

# enable a start trigger: here acquisition starts with a raising
# edge on the digital trigger input PFI0 (simulated devices will
# automatically send this trigger).
plugin.setParam("startTriggerMode", "digitalEdge")
plugin.setParam("startTriggerSource", "PFI0")
plugin.setParam("startTriggerRisingEdge", 1)


# enable a reference trigger using a digital, falling edge of PFI0 as
# trigger signal. The task is only stopped, if the trigger has been
# detected, at least pre-trigger samples have been acquired and after
# the trigger signal, another ("samplesPerChannel" - preTriggerSamples)
# will be acquired.
plugin.setParam("refTriggerMode", "digitalEdge")
plugin.setParam("refTriggerSource", "PFI0")
plugin.setParam("refTriggerRisingEdge", 0)
plugin.setParam("refTriggerPreTriggerSamples", 200)

# enable the on-board clock as continuous trigger
plugin.setParam("sampleClockSource", "OnboardClock")

# after having configured the task, start the device.
# The task is then configured in the device. It will be
# started with plugin.acquire() later.
plugin.startDevice()

# start the acquisition
plugin.acquire()
a = []

# continuously obtain new data until the task is not started
# any more (since the ref. trigger conditions are all met):
while plugin.getParam("taskStarted"):
    print("retrieve subset of data...")
    d = dataObject()
    plugin.copyVal(d)
    a.append(d)

print("The ref. trigger conditions are fulfilled.")

# plot the acquired values from both channels from the last run.
# the output dataObject already contains the correct axes units,
# descriptions etc...
plot1(a[-1],
      properties={"legendPosition": "Right", "legendTitles": ("AI0", "AI2")})

# stop and remove the configured task
plugin.stopDevice()
