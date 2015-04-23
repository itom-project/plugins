===================
 NI-DAQmx
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`NI-DAQmx`
**Type**:       :plugintype:`NI-DAQmx`
**License**:    :pluginlicense:`NI-DAQmx`
**Platforms**:  Windows, Linux
**Devices**:    NI-ADDA Converter
**Author**:     :pluginauthor:`Martin Hoppe`
**Requires**:   NI-DAQmx Lib and DLL
=============== ========================================================================================================
 
Overview
========

The plugin implements the DAQmx functions for analog-digital-converters from National Instruments. The installation needs the NI-DAQmx Library that can be downloaded from the NI website (http://www.ni.com/download/ni-daqmx-14.2/5046/en/)

.. pluginsummaryextended::
    :plugin: NI-DAQmx

Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: NI-DAQmx
        
Parameters
==========

These parameters are available and can be used to configure the **NI-DAQmx** instance. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*. In general the string returned is a semicolon comma separated string. Groups are separated by semicolon, elements inside groups with a comma. 
Example: the answer of taskStatus => "ai,0;ao,0;ci,-1;co,-1;di,-1;do,-1"

**name**: {str}, read-only
    name of the plugin
**channel**: {str}, read-only
    returns the channels your device supports as a comma separated string. ("Dev1/ai0,Dev1/ai1,...")
**chAssociated**: {str}, read-only
    Lists all the port that are associated to a their corresponding task.
    "Dev1/ai0" means that this channel is created and connected to the analog input task.
**XXTaskParams**
    The XX can be replaced by ai, ao, di, do, ci, co to set and get the parameters of the corresponding task.
    "<Frequency in Hz>,<SamplesToRead>,<mode=0>" always use mode 0, other modes are not tested yet!    
    Example:("aiTaskParams", "250000,100,0")
    aiTaskParams: {str},
    Initializes the analog input task. The parameters are:
    "<samplesPerSec>,<NoOfSamples>,<mode>" and optional for an external trigger: "</TriggerChannel>,<rising/falling>".
    <mode> can be a number from 0 to 2
    0 = finite (best for usage with itom. Size of dataobject can be defined and read when task is done)
    1 = continuous (not tested)
    2 = on demand (not tested)
    The optional parameters are for an external trigger. The trigger can be connected to PFIX for example. The task will be triggered after if a <rising/falling> (digital) edge occurs on the defined trigger input. 
    Pay attention to the naming of the trigger input: /Dev1/ with a leading "/"
    Example: "20000,100,0"
    Example: "20000,100,0,/Dev1/PFI0,rising"
    aoTaskParams    : {str},
    Not tested yet.
    ciTaskParams: {str},
    Not implemented yet.
    coTaskParams: {str},
    Not implemented yet.
    diTaskParams: {str},
    Not implemented yet.
    doTaskParams: {str},
    Not implemented yet.
**aiChParams**: {str},
    Set/Get the parameters for an analog input channel. For further information refer to the gui dialog.
    "<device>/<channel>,<mode>,<+-VoltageRange>" 
    <mode> can be a number from 0 to 4
    0 = default
    1 = differential
    2 = RSE
    3 = NRSE
    4 = Pseudodiff
    Example: "Dev1/ai0,4,10"
**aoChParams**: {str},
    Set/Get the parameters for an analog input channel. For further information refer to the gui dialog. !Not tested yet!
    "<device>/<channel>,<minVolt>,<maxVolt>" 
    Example: "Dev1/ao0,4,-2,3"
**diChParams**: {str},
    Not implemented yet.
**doChParams**: {str},
    Not implemented yet.
**ciChParams**: {str},
    Not implemented yet.
**coChParams**: {str},
    Not implemented yet.
**taskStatus**: {str}, read-only
    Returns a list of all taks with their actual status:
    -1 = not initialized
    0 = initialized
    1 = running (not tested yet)
**setValMode**: {str},
    This parameter is important for writing any kind of data to an output. The setVal method doesn't support a parameter for the kind of data you want to write (analog, digital, counter). There for this parameter defines where to send the data you set via setVal. 
    1 = Analog
    2 = Digital
    3 = Counter
    Note: 
    This parameter is only required for writing data. When reading data the type of data you want to read is defined by a parameter of the acquire command.


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

- Digital and Counter tasks, channels, etc are not implemented yet.

- After the analog input task is done and the data read, the task is erased from the memory. So it´s not possible to start that task again. The task must be recreated using plugin.setParam("aiTaskParams", "20000,100,0"). 










