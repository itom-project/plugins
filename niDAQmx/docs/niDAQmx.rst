===================
 NI-DAQmx
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`NI-DAQmx`
**Type**:       :plugintype:`NI-DAQmx`
**License**:    :pluginlicense:`NI-DAQmx`
**Platforms**:  Windows, Linux (NI officially only supports RRM installers for Red Hat, SUSE, CentOS)
**Devices**:    Nation Instruments Analog and digital I/O devices
**Author**:     :pluginauthor:`NI-DAQmx`
**Requires**:   NI-DAQmx driver from National Instruments
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: NI-DAQmx

This plugin has mainly be tested with simulated NI I/O devices under Windows (using the NI Measurement and
Automation Explorer software - NI MAX). Simulated tested devices were: NI PCIe-6321, NI PCI-6220, NI PCI-6111,
NI PCIe-6323 and NI PCI-6520.


Driver Installation
====================

In order to run the **NI-DAQmx** plugin in *itom*, the **NI-DAQmx** driver from National Instruments
has to be installed on the computer (considering the right bitness 32bit or 64bit).

Windows
--------

Download and install the latest NI-DAQmx driver from
https://www.ni.com/en-us/support/downloads/drivers/download.ni-daqmx.html#346240

The major part of this driver is usually installed under **C:/Program Files/National Instruments** and
**C:/Program Files (x86)/National Instruments**. Some important libraries are also copied to the 
**C:/Windows/System32**, where they are found from the itom plugin.

If you also want to compile the plugin, make sure that you enable the option
**Applications Development Supports / ANSI C Support** in the installer. This can also be done
by modifying the installation using the tab **Progam and Features** in the Windows Control Panel
(see https://knowledge.ni.com/KnowledgeArticleDetails?id=kA00Z000000P9udSAC ).

The itom **niDAQmx** plugin has mainly been developed and tested using **NI-DAQmx 18.06.0**.


Linux
------

National Instruments has only a limited support for Linux operating systems, such that
the **NI-DAQmx** driver is only available as RPM-package for Red Hat, SUSE and CentOS.

See this knowledge base entry (https://knowledge.ni.com/KnowledgeArticleDetails?id=kA00Z00000159XISAY)
and see the links, where to download the linux driver and how to install it.

Initialization
==============

The plugin is initialized with a set of mandatory and optional parameters:
    
    .. plugininitparams::
        :plugin: NI-DAQmx

If the **taskName** is not given or an empty string, a default task name is given. All
parameters can be changed during the runtime of the instance, besides the major **task type**
and **task mode**. That means, that every instance of this plugin can be operated with one of the
following combinations (of course multiple instances can be run in parallel):

* Analog input task with a finite number of acquired samples (per run)
* Analog input task with continuous acquisition (from start to stop signal)
* Analog output task with a finite number of written samples (per run)
* Analog output task with continuous write (from start to stop signal)
* Digital input task with a finite number of acquired samples (per run)
* Digital input task with continuous acquisition (from start to stop signal)
* Digital output task with a finite number of written samples (per run)
* Digital output task with continuous write (from start to stop signal)

All analog tasks are created as voltage tasks, only. Transforming voltages into other physical units
must be done within the script.

It is not possible to combine analog and digital ports within one task, hence, once instance
of the plugin. Finite input tasks can also have a **reference trigger**, that is usually used
to stop the acquisition upon a certain input trigger. If this **refTriggerMode** is set, the
finite task behaves like a continuous task (see examples below) and acquires data from the start
signal until all conditions of the reference trigger are fulfilled.

Parameters
==========

These parameters are available and can be used to configure the **NI-DAQmx** instance. 
During the runtime of an instance, the values of these parameters are obtained by the method *getParam*, 
writeable parameters can be changed using *setParam*.

**availableDevices**: {str}, read-only
    comma-separated list of all detected and available devices
**availableTerminals**: {str}, read-only
    comma-separated list of all detected and available terminals (e.g. for
    'sampleClockSource' or 'startTriggerSource'). The standard sample clock source
    'OnboardClock' is not contained in this list.
**bufferSize**: {int}
    Sets and changes the automatic input / output buffer allocation mode. If -1 (default),
    the automatic allocation is enabled. Else defines the number of samples the buffer can
    hold for each channel (only recommended for continuous acquisition). In automatic mode
    and continuous acquisition, the standard is a buffer size of 1 kS for a sampling rate
    < 100 S/s, 10 kS for 100-10000 S/s, 100 kS for 10-1000 kS/s and 1 MS else. For input
    tasks, this size changes the input buffer size of the device, else the output buffer
    size.
**channels**: {str}
    semicolon-separated list of all channels that should be part of this task. Every item is
    a comma separated string that defines and parameterizes every channel.
**loggingActive**: {int}, read-only
    Indicates if TDMS file logging has been enabled and which mode was accepted by the
    device. The value has the same meaning than 'loggingMode'.
**loggingFilePath**: {str}
    The path to the TDMS file to which you want to log data.
**loggingGroupName**: {str}
    The name of the group to create within the TDMS file for data from this task. If empty,
    the task name is taken. If data is appended to a TDMS file, a number symbol (e.g. Task
    #1, Task #2...) is added at each run.
**loggingMode**: {int}
    0: logging is disabled (default), 1: logging is enabled with disabled read (fast, but no
    data can simultaneously read via getVal/copyVal), 2: logging is enabled with allowed
    reading of data.
**loggingOperation**: {str}
    Specifies how to open the TDMS file. 'open': Always appends data to an existing TDMS
    file. If it does not exist yet, the task start operation will return with an error;
    'openOrCreate': Creates a new TDMS file or appends data to the existing
    one;'createOrReplace' (default): Creates a new TDMS file or replaces an existing one;
    'create': Newly creates the TDMS file. If it already exists a task start operation will
    return with an error.
**name**: {str}, read-only
    NI-DAQmx
**readTimeout**: {float}
    Timeout when reading up to 'samplesPerChannel' values (per channel) in seconds. If -1.0
    (default), the timeout is set to infinity (recommended for finite tasks). If 0.0,
    getVal/copyVal will return all values which have been recorded up to this call.
**refTriggerLevel**: {float}
    Only for 'refTriggerMode' == 'analogEdge': The threshold at which to stop acquiring
    samples. Specify this value in the units of the measurement or generation.
**refTriggerMode**: {str}
    A reference trigger can be enabled to stop an acquisition when the device acquired all
    pre-trigger samples, an analog or digital signal reaches a specified level and and the
    device acquired all post-trigger samples. 'off': no reference trigger, 'digitalEdge':
    The trigger event is given if 'refTriggerSource' is activated (based on
    'refTriggerRisingEdge'), 'analogEdge': similar to 'digitalEdge', but the analog input
    'refTriggerSource' has to pass the value 'refTriggerLevel'. A reference trigger can only
    be set for finite, input tasks. The reference trigger can only be enabled for finite
    input tasks. However these tasks then behave like continuous tasks, but the stop command
    is automatically generated by the reference trigger event (plus additional post trigger
    values)
**refTriggerPreTriggerSamples**: {int}
    The minimum number of samples per channel to acquire before recognizing the Reference
    Trigger. The number of posttrigger samples per channel is equal to 'samplesPerChannel'
    minus this value.
**refTriggerRisingEdge**: {int}
    Specifies on which slope of the signal to stop acquiring samples. 1: rising edge
    (default), 0: falling edge.
**refTriggerSource**: {str}
    The source terminal of the trigger source (if 'refTriggerMode' is set to 'digitalEdge'
    or 'analogEdge').
**sampleClockRisingEdge**: {int}
    If 1, samples are acquired on a rising edge of the sample clock (default), else they are
    acquired on a falling edge.
**sampleClockSource**: {str}
    The source terminal of the Sample Clock. To use the internal clock of the device, use an
    empty string or 'OnboardClock' (default). An example for an external clock source is
    'PFI0' or PFI1'.
**samplesPerChannel**: {int}
    The number of samples to acquire or generate for each channel in the task (if taskMode
    is 'finite'). If taskMode is 'continuous', NI-DAQmx uses this value to determine the
    buffer size. This parameter is ignored for output tasks.If 'samplesPerChannel' is 1, one
    single value is read or written by asoftware trigger only. The parameters
    'samplingRate', 'bufferSize', 'sampleClockSource' and 'sampleClockRisingEdge' are
    ignored then.
**samplingRate**: {float}
    The sampling rate in samples per second per channel. If you use an external source for
    the Sample Clock, set this value to the maximum expected rate of that clock.
**setValWaitForFinish**: {int}
    If 1, the **setVal** call will block until all data has been written (only valid for finite
    tasks). If 0, setVal will return immediately, then use 'taskStarted' to verify if the
    operation has been finished.
**startTriggerLevel**: {float}
    Only for 'startTriggerMode' == 'analogEdge': The threshold at which to start acquiring
    or generating samples. Specify this value in the units of the measurement or generation.
**startTriggerMode**: {str}
    Specifies the start trigger mode. 'off': software-based start trigger, 'digitalEdge':
    The start of acquiring or generating samples is given if the 'startTriggerSource' is
    activated (based on 'startTriggerRisingEdge'), 'analogEdge': similar to 'digitalEdge',
    but the analog input 'startTriggerSource' has to pass the value 'startTriggerLevel'.
**startTriggerRisingEdge**: {int}
    Specifies on which slope of the signal to start acquiring or generating samples. 1:
    rising edge (default), 0: falling edge.
**startTriggerSource**: {str}
    The source terminal of the trigger source (if 'startTriggerMode' is set to 'digitalEdge'
    or 'analogEdge').
**supportedChannels**: {str}, read-only
    comma-separated list of all detected and supported channels with respect to the task
    type. Every item consists of the device name / channel name
**taskConfigured**: {int}, read-only
    Indicates if the task is properly configured (1, all task related parameters where
    accepted) or not (0).
**taskMode**: {str}
    mode of the task recording / data generation: finite, continuous
**taskName**: {str}, read-only
    name of the NI task that is related to this instance
**taskStarted**: {int}, read-only
    Indicates if the task is currently running (1) or stopped / inactive (0).
**taskType**: {str}, read-only
    task type: analogInput, analogOutput, digitalInput, digitalOutput

Usage
======

Channels
---------

Every task can consist of one or multiple channels, that are given as semicolon-separated list in the
**channels** parameter. Please note, that a digital task can only consist of digital channels and analog tasks
only of analog channels. Read the parameter **supportedChannels** to get all available channel names for
your task.

Every channel item in the semicolon-separated list consists of a configuration string (see examples below),
whose exact meaning depend on the task mode and type. The configuration string is a comma-separated list of items.
The first item is always the physical name of the channel, that usually consists of the device name (e.g. Dev1),
followed by a slash and the port name (e.g. AI0). For digital tasks, the physical name can also consist of
the device name, the port name and the line name (divided by slashes) if one single line (pin) of a port should
be used only (e.g. Dev0/port0/line1). All possible physical names are listed in **supportedChannels**.

For analog input and output tasks, a certain minimum and maximum voltage must be given (as integer or floating point number).

Analog input tasks have an additional parameter **ConfigMode**, that defines the input terminal configuration
of the channel. Possible values are:

.. code-block::
    
    DAQmx_Val_Cfg_Default = 0, 
    DAQmx_Val_Diff = 1, 
    DAQmx_Val_RSE = 2, 
    DAQmx_Val_NRSE = 3, 
    DAQmx_Val_PseudoDiff = 4

The possible configuration strings for one channel are:

* **Analog input channel**: PhysicalName,ConfigMode,MinVoltage,MaxVoltage
* **Analog output channel**: PhysicalName,MinVoltage,MaxVoltage
* **Digital input channel**: PhysicalName
* **Digital output channel**: PhysicalName

Data Types
-----------

Analog input or output tasks are always based on 2-dimensional dataObjects of dtype **float64**.
The rows corresponds to the channels in the active task and the columns correspond to the acquired samples.

Digital input or output tasks are always based on 2-dimensional dataObjects of dtypes **uint8**, **uint16** or
**int32**. The type **int32** is internally casted to **uint32** (however uint32 is not officially supported for
dataObjects). The correct bitdepth depend on whether a channel is assigned to a single line or an entire port.
In the first case, the datatype is usually **uint8**, in the latter case, the bitdepth depend on the number of
lines, that are covered by the port. If the wrong datatype is used, an appropriate error message will appear,
that indicates the desired bitdepth.

General
--------

The general approach to use a NI I/O device can be seen in one of the examples below. In general, it is 
recommended to configure a plugin instance as far as possible. Then the task will be created and configured using
the **startDevice** command. In case of invalid parameters, **startDevice** will raise an exception, whose
error message usually gives detailed information about an invalid parameterization and possible different solutions.

The task can finally be deleted using **stopDevice**.

Input tasks will always be started using **acquire**. If a start trigger is given, the real acquisition will
be started if the trigger event is signalled (but after having called **acquire**). Finite tasks will automatically be 
stopped if the requested number of samples per channel (**samplesPerChannel**) are acquired (if no reference 
trigger is given). The values can then be obtained via **getVal** or **copyVal** (like for any other grabber or 
dataIO device).

The acquisition of **continuous** tasks is also started by **acquire** (and an optional start trigger). Then all
data is temporarily stored into an internal buffer of the NI driver. The buffer size is usually automatically
determined based on **samplesPerChannel** (as far as this value is big enough, else NI determines its own internal
buffer size; see also the parameter **bufferSize**). As far as no fast TDMS logging is enabled, you have to 
continuously receive the latest data via **getVal** or **copyVal** in order to avoid that the internal buffer
overflows. The continuous task is then stopped via **stop**.

For output tasks, a **MxN** data object must be passed to **setVal**, where **M** must correspond to the number
of channels in the task. **N** are the number of samples. However **N** is **not** the number that defines the
number of transmitted samples per channel. This is again defined by the parameter **samplesPerChannel**. If
**samplesPerChannel** is smaller than **N**, only the **samplesPerChannel** columns are written to the output
channels (using the sampling rate or sample clock). If **samplesPerChannel** is bigger than **N**, the write
operation restarts at the first column once the last column of the dataObject has been written.

For **finite output tasks**, it is possible to block the call of **setVal** until all samples have been
written by setting **setValWaitForFinish** to 1. Else **setVal** will return immediately. The end of the task
can then be continously checked by getting the parameter **taskStarted** and check if it drops to 0 again.

Continuous output tasks will continuously write the columns and restart from the beginning until the task
will be stopped via **stop**.

Reference Trigger
------------------

The reference trigger can be configured in order to stop a task upon a certain trigger signal. This
reference trigger can only be applied to finite input tasks. Although the parameter **refTriggerMode** can
only be set to something else than **off** for finite input tasks, such a task will then behave like a continuous
input task. That means, that the task will be started via **acquire** and data will be continously recorded
until the stop trigger condition is fulfilled (or the internal buffer overflows). To avoid the latter, it is
again necessary to continously get intermediate data via **getVal** / **copyVal** or enable a fast TDMS logging.

The reference trigger listens to either a falling or raising edge of a digital line, or when an analog trigger
input jumps over (or below) a certain threshold value. However the task is only stopped if three different conditions
are met:

1. The trigger must be signalled (see **refTriggerSource**, **refTriggerMode**, **refTriggerRisingEdge** and **refTriggerLevel**)
2. A certain number of samples per channel must have been recorded (see **refTriggerPreTriggerSamples**)
3. If cond 1 and 2 are met, a certain number of post trigger samples will be recorded before stopping the task.
   This number is defined by **samplesPerChannel** - **refTriggerPreTriggerSamples**.


TDMS Logging
=============

NI provides a possibility to record all acquired values from all **input tasks** in the NI file format **TDMS**
(see https://www.ni.com/en-US/support/documentation/supplemental/07/tdms-file-format-internal-structure.html).

The filename of the tdms file, that should be used for the upcoming logging can be set by the parameter
**loggingFilePath**. The TDMS file format can contain multiple arrays from different recordings. Each array
is a two dimensional array, where each row belongs to one channel and the columns are the recorded samples.
Each array is stored under a certain path, where each node of the path is denoted as group.
Separate the different group names by a single slash, to provide a full path. The group name of the upcoming
recording is set via **loggingGroupName**. If the group name already exists in the tdms file, a suffix **#1**, **#2**, ...
is added to the leaf group name.
There are different options how to open or create a **TDMS** file. These can be set via **loggingOperation**.
It is for instance possible to always create a new file, to append data to an existing file among others.

The logging itself must be enabled by the parameter **loggingMode**. Set this value to 0 in order to disable logging.
Set it to 1 in order to enable a fast logging. Then all data is automatically logged into the TDMS file after
having called **acquire**, but it is not possible to simultaneously get the recorded data via **getVal** or **copyVal**.
The recording is done via background thread in the NI driver. The last possible value is 2. Then the task is
started like an ordinary task and whenever data is received via **getVal** or **copyVal**, the same data is
stored into TDMS file, too. No data is recorded if **getVal** or **copyVal** are not called.

It is recommended to set all logging parameters before calling **startDevice**. If one of these parameters
is changed later, the device is internally stopped, then the parameters are changed and the device is
reconfigured with the new logging properties.

If the logging is activated, the parameter **loggingActive** will be set to the currently active **loggingMode**.

See the example **demo_ai_tdms_logging.py** for a demo about the TDMS logging.

A TDMS file can for instance be read via the Python package **npTDMS** (https://pypi.org/project/npTDMS):

.. code-block:: python
    
    # coding=utf8

    """Demo to load and read a TDMS file

    Here, we read the TMDS files, that have been created
    by the ai_continuous and di_continuous demo scripts.

    This script requires the Python package npTDMS
    (https://pypi.org/project/npTDMS).
    """

    import nptdms as tdms
    import numpy as np

    # step 1: read the file demo_ai_continuous.tdms

    file = tdms.TdmsFile(r"D:\temp\demo_ai_continuous.tdms")

    print("Available groups:")
    print(file.groups())

    # access group object
    groupObject= file.object(file.groups()[0])


    data = []

    for obj in file.group_channels('group1'):
        data.append(obj.data)

    total2= np.vstack(data)
    plot1(total2)

Configuration Dialog and Toolbox
==================================

The plugin provides both a configuration dialog as well as a toolbox.

The toolbox comes with an overview panel, that shows some basic information about connected channels
and about the current run state of the task. In a 2nd tab, a general list of all parameters is displayed
by means of a generic parameter widget.

The configuration dialog let you configure all channels as well as provide access to all major
parameters, separated into different groups (like start trigger, reference trigger, sample clock etc.).

Compilation
============

To compile this plugin, make sure that the **ANSI C Support** of the NI-DAQmx driver
has been installed, too (see *driver* section above).

Then enable the option **PLUGIN_niDAQmx** in the CMake configuration of the plugins repository.
If the NI-DAQmx driver has been installed to default directories, CMake should automatically
detect the include and library path of the NI-DAQmx driver. Else, try to set

**NIDAQMX_DIR** to the main directory of the NI-DAQ driver (e.g. the directory, that contains
the *include* folder with the *NIDAQmx.h* file. Alternatively directly set the following
variables:

**NIDAQMX_INCLUDE_DIR** must point to the include directory of the NI-DAQmx driver. This is
the directory, that contains the file **NIDAQmx.h** and **NIDAQMX_LIBRARY** must point to the
linkable library of NIDAQmx, e.g. **NIDAQmx.lib** under Windows.

Examples
=========

All these examples can also be found in the **demo** folder of the **niDAQmx** plugin sources:

Analog Input Tasks
--------------------

**demo_ai_finite.py**

.. code-block:: python
    
    # coding=utf8

    """Finite analog input task.

    Demo script for acquiring a finite set of analog
    values with a National Instruments DAQ device.

    To test this script, the NI MAX (Measurement & Automation
    Explorer) has been used to create simulated devices.

    In this test, a simulated device NI PCIe-6321 with 16 analog input (AI)
    ports was created and named "Dev1".

    Additionally, one other device NI PCI-6111 with 2 analog input (AI)
    ports was added in NI-MAX with the name "Dev3".

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
    """

    import time

    # Demo 1: Analog input task, finite acquisition, 80 samples / sec
    plugin = dataIO(
        "NI-DAQmx",
        "analogInput",
        taskName="myTaskName",
        taskMode="finite",
        samplingRate=80
    )

    plugin.showToolbox()

    # A total number of 800 samples should be acquired from each port
    plugin.setParam("samplesPerChannel", 800)

    # Configure the channels (semicolon-separated list of single channel config strings):
    # Ch1: Dev1, AI0, connection type 2 (RSE), -10V..+10V
    # Ch2: Dev1, AI2, connection type 0 (Default), -5V..+5V
    plugin.setParam("channels", "Dev1/ai0,2,-10.0,10.0;Dev1/ai2,0,-5,5")

    # enable a start trigger: here acquisition starts with a falling
    # edge on the digital trigger input PFI0 (simulated devices will
    # automatically send this trigger).
    plugin.setParam("startTriggerMode", "digitalEdge")
    plugin.setParam("startTriggerSource", "PFI0")
    plugin.setParam("startTriggerRisingEdge", 0)

    # enable the on-board clock as continuous trigger
    plugin.setParam("sampleClockSource", "OnboardClock")

    # after having configured the task, start the device.
    # The task is then configured in the device. It will be
    # started with plugin.acquire() later.
    plugin.startDevice()

    a = []

    # repeat the configured acquisition task 5x.
    for i in range(0,5):
        # modify the sampling rate to different values (Hz)
        plugin.setParam("samplingRate", 1000 + i*100)
        t = time.time()
        # start the acquisition of the given number of samples per channel.
        plugin.acquire()
        d=dataObject()
        # getVal will return if all samples have been acquired (or timeout)
        plugin.getVal(d)
        a.append(d)
        print(time.time()-t)
        time.sleep(0.5)

    # plot the acquired values from both channels from the last run.
    # the output dataObject already contains the correct axes units,
    # descriptions etc...
    plot1(a[-1],
          properties={"legendPosition": "Right", "legendTitles": ("AI0", "AI2")})

    # stop and remove the configured task
    plugin.stopDevice()

    # change the analog input task to another channel and
    # another trigger condition.
    plugin.setParam("channels", "Dev3/ai1,4,-6,8")
    plugin.setParam("startTriggerMode", "off")
    plugin.setParam("startTriggerSource", "PFI0")
    plugin.setParam("startTriggerRisingEdge", 0)

    # restart the task and do another finite measurement.
    plugin.startDevice()
    plugin.acquire()
    plugin.getVal(d)
    plugin.stopDevice()
    plot1(d)

**demo_ai_continuous.py**

.. code-block:: python
    
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



**demo_ai_single_value.py**

.. code-block:: python
    
    # coding=utf8

    """Finite analog input task for single value acquisitions.

    Demo script for acquiring exactly one analog value
    per channel per acquire() command 
    with a National Instruments DAQ device.

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
    """

    import time

    # Demo 1: Analog input task, finite acquisition
    # the sampling rate is unimportant
    plugin = dataIO(
        "NI-DAQmx",
        "analogInput",
        taskName="myTaskName",
        taskMode="finite"
    )

    plugin.showToolbox()

    # A total number of 1 samples should be acquired from each port
    plugin.setParam("samplesPerChannel", 1)

    # Configure the channels (semicolon-separated list of single channel config strings):
    # Ch1: Dev1, AI0, connection type 2 (RSE), -10V..+10V
    # Ch2: Dev1, AI2, connection type 0 (Default), -5V..+5V
    plugin.setParam("channels", "Dev1/ai0,2,-10.0,10.0;Dev1/ai2,0,-5,5")

    # If single values are acquired (samplesPerChannel=1), the start
    # trigger must be off, since this acquisition on demand only operates
    # upon a software trigger.
    plugin.setParam("startTriggerMode", "off")


    # after having configured the task, start the device.
    # The task is then configured in the device. It will be
    # started with plugin.acquire() later.
    plugin.startDevice()

    a = dataObject.zeros([2, 50], 'float64')
    t = time.time()

    # repeat the configured acquisition task 5x.
    for i in range(0, 50):
        # start the acquisition of the given number of samples per channel.
        plugin.acquire()
        
        # getVal will return if all samples have been acquired (or timeout)
        plugin.copyVal(a[:, i])

    print(time.time()-t)

    # plot the acquired values from both channels from the last run.
    # the output dataObject already contains the correct axes units,
    # descriptions etc...
    plot1(a,
          properties={"legendPosition": "Right", "legendTitles": ("AI0", "AI2")})

    # stop and remove the configured task
    plugin.stopDevice()

    

**demo_ai_finite_ref_trigger.py**

.. code-block:: python
    
    # coding=utf8

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
    will implicitely let the finite task behave like a continuous task.
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
    

**demo_ai_tdms_logging.py**

.. code-block:: python
    
    # coding=utf8

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
    # stop the device before chaning the logging modes. However,
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



Analog Output Tasks
--------------------

**demo_ao_finite.py**

.. code-block:: python
    
    # coding=utf8

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

**demo_ao_continuous.py**

.. code-block:: python
    
    # coding=utf8

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


**demo_ao_single_value.py**

.. code-block:: python
    
    # coding=utf8

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

Digital Input Tasks
--------------------

**demo_di_finite.py**

.. code-block:: python
    
    # coding=utf8

    """Finite digital input task.

    Demo script for acquiring a finite set of digital
    values with a National Instruments DAQ device.

    To test this script, the NI MAX (Measurement & Automation
    Explorer) has been used to create simulated devices.

    In this test, a simulated device NI PCI-6220 (Dev2) with 3x8 digital
    inputs is used. These inputs are distributed over 3 ports
    (ports0, ports1, ports2). Every port has 8 bit (line0-line7).

    You can either record an entire port, such that the obtained
    number is a bitmask of all lines in the port, or you record
    selected lines only. Then, every line is acquired into a
    different row of the obtained dataObject.

    The channel configuration string for digital input tasks always
    follows this pattern:

    DeviceName/PortName

    or

    DeviceName/PortName/LineName

    If an entire port is read, the data type is either uint8, uint16 or int32,
    depending on the number of lines per port (usually uint8). If single lines
    are read, each line is written to one row (usually to an uint8 dataObject, too).

    Hint: It depends on the NI DAQ devices, if they allow
    integrating different devices into the same measurement
    task or not. Many devices do not allow this.
    """

    import time

    # create a new plugin instance, configured
    # as finite digital input task with a freq. of 800 samples per second.
    plugin = dataIO("NI-DAQmx", "digitalInput", taskName="myDigitalInputTask",
                  taskMode="finite", samplingRate=800)

    # print a list of supported channels:
    print("Channels:", plugin.getParam("supportedChannels"))

    plugin.setParam("channels", "Dev2/port0")

    # number of finite samples: 800
    plugin.setParam("samplesPerChannel", 800)

    # this is the trigger source for the internal clock.
    # In this case the PFI1 trigger input of Dev2 is used, however
    # it would also be possible to use the "OnboardClock". Only,
    # the demo device NI PCI-6220 does not support the OnboardClock here.
    plugin.setParam("sampleClockSource", "/Dev2/PFI1")

    # configure the task in the device
    plugin.startDevice()

    # acquire 5x 800 samples with 800 samples / second
    a = []

    print("acquire 5x800 samples...")

    for i in range(0, 5):
        print(f"run {i+1}/5...", end="")
        t = time.time()
        
        # start the finite task
        plugin.acquire()
        d = dataObject()
        
        # getVal waits for the finite task to be finished and reads out the values.
        plugin.getVal(d)
        a.append(d)
        print("done in %.2f s" % (time.time() - t))

    print("datatype:", d.dtype)
    plot1(dataObject.dstack(a).squeeze(), properties={"curveStyle": "Steps"})


    # change some parameters on the fly...
    plugin.setParam("sampleClockSource", "PFI0")

    # do not acquire an entire port, but single lines.
    # this leads to an acquired dataObject whose row count is
    # equal to the number of connected lines.
    plugin.setParam("channels", "Dev2/port0/line6;Dev2/port0/line5;Dev2/port0/line1")
    plugin.setParam("samplingRate", 1600)

    # the device is still started (however due to the change of channels,
    # it was internally stopped and restarted)

    for i in range(0, 10):
        print(f"acquire run {i+1}/10...", end="")
        plugin.acquire()
        plugin.getVal(d)
        print(" done")

    plot1(d)

    # stops and terminates the task
    plugin.stopDevice()


**demo_di_continuous.py**

.. code-block:: python

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


**demo_di_single_value.py**

.. code-block:: python
    
    # coding=utf8
    
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


Digital Output Tasks
---------------------

**demo_do_finite.py**

.. code-block:: python
    
    # coding=utf8

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


**demo_do_continuous.py**

.. code-block:: python
    
    # coding=utf8

    """Continuous digital output task.

    Demo script for sending a series of number to different
    channels of a National Instruments DAQ device.

    To test this script, the NI MAX (Measurement & Automation
    Explorer) has been used to create simulated devices.

    In this test, a simulated device NI PCIe-6323 (Dev4) with 1 digital output 
    port (DO) with 32 lines (buffered) and two other ports (port 1 and port 2), that
    support only unbuffered outputs.

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
        "digitalOutput",
        taskName="digitalOutput",
        taskMode="continuous",
        samplingRate=800)

    # select port 0 (32 lines) from Dev4.
    plugin.setParam("channels", "Dev4/port0")

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

    a = dataObject.randN([1,100], 'int32')

    for j in range(0,2):
        for i in range(0,3):
            plugin.setVal(a)
            
            print(f"Run {j+1}/2, iteration {i+1}/3: write for 3 sec ", end="")
            
            for i in range(0, 3):
                print(".", end="")
                time.sleep(1)
            
            print(" done")
            plugin.stop()


**demo_do_single_value.py**

.. code-block:: python
    
    # coding=utf8

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
    
Known Issues
============

- Counter tasks are not implemented yet.
- The development has only been tested based on simulated NI I/O devices. Therefore hardware-based start
  and reference triggers, as well as clock sources could not fully be tested, since they are all
  emulated as software triggers.

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using the NI-DAQmx 18.1.0 (Linux)
* itom setup 3.1.0: This plugin has been compiled using the NI-DAQmx 18.6.0 (Windows)
* itom setup 4.0.0: Complete renewed plugin implementation. This plugin is incompatible to earlier version of
  this plugin and provides much more features. It has been compiled using NI-DAQmx 18.6.0 (Windows)
