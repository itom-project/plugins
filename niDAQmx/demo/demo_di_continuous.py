# coding=utf8

import time
import numpy as np

"""Continuous digital input task with optional logging (TDMS files).

Demo script for acquiring a continuous set of digital
values with a National Instruments DAQ device.

For understanding this demo, one is referred to the documentations
in the scripts demo_di_finite.py and demo_ai_continuous.py. Together, they are
very similar to this script.

Data from a continuous task can be obtained by regularily
calling getVal / copyVal or by enabling the TDMS file logging
technique.

If an entire port is read, the data type is either uint8, uint16 or int32,
depending on the number of lines per port (usually uint8). If single lines
are read, each line is written to one row (usually to an uint8 dataObject, too).

Reading TDMS files via Python is possible by the package npTDMS
(https://pypi.org/project/npTDMS).
"""

plugin = dataIO(
    "NI-DAQmx",
    "digitalInput",
    taskName="myDigitalInputTask",
    taskMode="continuous",
    samplingRate=800
)

plugin.setParam("samplesPerChannel", 800)
plugin.setParam("channels", "Dev1/line0;Dev1/line1")

# configure the task
plugin.startDevice()

# start the task
plugin.acquire()
t = time.time()

alldata = []

for i in range(0, 10):
    time.sleep(0.1)
    d = dataObject()
    plugin.copyVal(d)
    print(time.time() - t, d.shape)
    alldata.append(d)

plugin.stopDevice()

plot1(np.hstack(alldata))


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
plugin.setParam("loggingFilePath", "D:/temp/demo_di_continuous.tdms")
plugin.setParam("loggingOperation", "createOrReplace")
plugin.setParam("loggingGroupName", "group1")

# create and configure the task
plugin.startDevice()

# start the continuous task again
plugin.acquire()

# wait for 5 seconds (data are acquired and stored into the file)
time.sleep(5)

# stop the task
plugin.stop()

# stop the device (if there are still running \
# tasks, they will also be stopped here)
plugin.stopDevice()
