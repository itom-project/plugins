===================
 niDAQmx
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`niDAQmx`
**Type**:       :plugintype:`niDAQmx`
**License**:    :pluginlicense:`niDAQmx`
**Platforms**:  Windows, Linux
**Devices**:    NI-ADDA Converter
**Author**:     :pluginauthor:`niDAQmx`
**Requires**:   NI-DAQmx Lib and DLL
=============== ========================================================================================================
 
Overview
========

The plugin implements the DAQmx functions for analog-digital-converters from National Instruments. The installation needs the NI-DAQmx Library that can be downloaded from the NI website (http://www.ni.com/download/ni-daqmx-14.2/5046/en/)

.. pluginsummaryextended::
    :plugin: niDAQmx

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: niDAQmx
        
Parameters
==========

These parameters are available and can be used to configure the **niDAQmx** instance. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*. In general the string returned is a semicolon comma separated string. Groups are separated by semicolon, elements inside groups with a comma. 
Example: the answer of taskStatus => "ai,0;ao,0;ci,-1;co,-1;di,-1;do,-1"

**availableDevices**: {str}, read-only
    comma-separated list of all detected and available devices
**availableTerminals**: {str}, read-only
    comma-separated list of all detected and available terminals (e.g. for 'sampleClockSource' or 'startTriggerSource'). The standard sample clock source 'OnboardClock' is not contained in this list.
**bufferSize**: {int}
    Sets and changes the automatic input / output buffer allocation mode. If -1 (default), the automatic allocation is enabled. Else defines the number of samples the buffer can hold for each channel (only recommended for continuous acquisition). In automatic mode and continuous acquisition, the standard is a buffer size of 1 kS for a sampling rate < 100 S/s, 10 kS for 100-10000 S/s, 100 kS for 10-1000 kS/s and 1 MS else. For input tasks, this size changes the input buffer size of the device, else the output buffer size.
**channels**: {str}
    semicolon-separated list of all channels that should be part of this task. Every item is a comma separated string that defines and parameterizes every channel.
**loggingActive**: {int}, read-only
    Indicates if TDMS file logging has been enabled and which mode was accepted by the device. The value has the same meaning than 'loggingMode'.
**loggingFilePath**: {str}
    The path to the TDMS file to which you want to log data. 
**loggingGroupName**: {str}
    The name of the group to create within the TDMS file for data from this task. If empty, the task name is taken. If data is appended to a TDMS file, a number symbol (e.g. Task #1, Task #2...) is added at each run.
**loggingMode**: {int}
    0: logging is disabled (default), 1: logging is enabled with disabled read (fast, but no data can simultaneously read via getVal/copyVal), 2: logging is enabled with allowed reading of data.
**loggingOperation**: {str}
    Specifies how to open the TDMS file. 'open': Always appends data to an existing TDMS file. If it does not exist yet, the task start operation will return with an error; 'openOrCreate': Creates a new TDMS file or appends data to the existing one;'createOrReplace' (default): Creates a new TDMS file or replaces an existing one; 'create': Newly creates the TDMS file. If it already exists a task start operation will return with an error.
**name**: {str}, read-only
    
**readTimeout**: {float}
    Timeout when reading up to 'samplesPerChannel' values (per channel) in seconds. If -1.0 (default), the timeout is set to infinity (recommended for finite tasks). If 0.0, getVal/copyVal will return all values which have been recorded up to this call.
**sampleClockRisingEdge**: {int}
    If 1, samples are acquired on a rising edge of the sample clock (default), else they are acquired on a falling edge.
**sampleClockSource**: {str}
    The source terminal of the Sample Clock. To use the internal clock of the device, use an empty string or 'OnboardClock' (default). An example for an external clock source is 'PFI0' or PFI1'.
**samplesPerChannel**: {int}
    The number of samples to acquire or generate for each channel in the task (if taskMode is 'finite'). If taskMode is 'continuous', NI-DAQmx uses this value to determine the buffer size. This parameter is ignored for output tasks.If 'samplesPerChannel' is 1, one single value is read or written by asoftware trigger only. The parameters 'samplingRate', 'bufferSize', 'sampleClockSource' and 'sampleClockRisingEdge' are ignored then.
**samplingRate**: {float}
    The sampling rate in samples per second per channel. If you use an external source for the Sample Clock, set this value to the maximum expected rate of that clock.
**setValWaitForFinish**: {int}
    If 1, the setVal call will block until all data has been written (only valid for finite tasks). If 0, setVal will return immediately, then use 'taskStarted' to verify if the operation has been finished.
**startTriggerLevel**: {float}
    Only for 'startTriggerMode' == 'analogEdge': The threshold at which to start acquiring or generating samples. Specify this value in the units of the measurement or generation.
**startTriggerMode**: {str}
    Specifies the start trigger mode. 'off': software-based start trigger, 'digitalEdge': The start of acquiring or generating samples is given if the 'startTriggerSource' is activated (based on 'startTriggerRisingEdge'), 'analogEdge': similar to 'digitalEdge', but the analog input 'startTriggerSource' has to pass the value 'startTriggerLevel'.
**startTriggerRisingEdge**: {int}
    Specifies on which slope of the signal to start acquiring or generating samples. 1: rising edge (default), 0: falling edge.
**startTriggerSource**: {str}
    The source terminal of the trigger source (if 'startTriggerMode' is set to 'digitalEdge' or 'analogEdge').
**supportedChannels**: {str}, read-only
    comma-separated list of all detected and supported channels with respect to the task type. Every item consists of the device name / channel name
**taskConfigured**: {int}, read-only
    Indicates if the task is properly configured (1, all task related parameters where accepted) or not (0).
**taskMode**: {str}
    mode of the task recording / data generation: finite, continuous, onDemand
**taskName**: {str}, read-only
    name of the NI task that is related to this instance
**taskStarted**: {int}, read-only
    Indicates if the task is currently running (1) or stopped / inactive (0).
**taskType**: {str}, read-only
    task type: analogInput, analogOutput, digitalInput, digitalOutput


Example
=======


Create new Instance:

.. code-block:: python

    plugin = dataIO("niDAQmx")
    d = dataObject([2,100], 'float64')

.. code-block:: python  

    # setup Analog-Input-Task
    # 20000 samples/sec; 100 samples; finite mode
    plugin.setParam("aiTaskParams", "20000,100,0")

    # setup the first two Analog-Input-Channel 
    plugin.setParam("aiChParams", "Dev1/ai0,4,10") # (channel0, PseudoDiff, +-10V)
    plugin.setParam("aiChParams", "Dev1/ai1,4,42") # (channel1, PseudoDiff, +-42V)
    
    # Acquire the Data (start measurement)
    plugin.acquire(1) # 1 = Analog-Input-Task

    # copy Data in DataObject
    plugin.getVal(d)

    # plot dataObject
    plot(d, "itom1dqwtplot") # Pay attention, that the scaling of two different channels with different VoltageRange is not as it´s shown in the diagramm
    
Known Issues
============

- Counter tasks are not implemented yet.

Changelog
=========

* itom setup 3.1.0: This plugin has been compiled using the niDAQmx 18.1.0 (Linux)
* itom setup 3.1.0: This plugin has been compiled using the niDAQmx 18.6.0 (Windows)
