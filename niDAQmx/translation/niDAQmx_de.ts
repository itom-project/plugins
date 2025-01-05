<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="de">
<context>
    <name>DialogNiDAQmx</name>
    <message>
        <source>finite</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>continuous</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Configuration Dialog</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>DockWidgetNIDAQmx</name>
    <message>
        <source>Form</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>General</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Channels:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task name:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[Identifier]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[Mode]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[Type]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task type:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task mode:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task configured:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task running:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>TDMS logging active:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Parameters</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>NiDAQmx</name>
    <message>
        <source>name of the NI task that is related to this instance</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>comma-separated list of all detected and available devices</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>comma-separated list of all detected and available terminals (e.g. for &apos;sampleClockSource&apos; or &apos;startTriggerSource&apos;). The standard sample clock source &apos;OnboardClock&apos; is not contained in this list.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>mode of the task recording / data generation: finite, continuous</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>task type: analogInput, analogOutput, digitalInput, digitalOutput</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Indicates if TDMS file logging has been enabled and which mode was accepted by the device. The value has the same meaning than &apos;loggingMode&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Indicates if the task is currently running (1) or stopped / inactive (0).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Indicates if the task is properly configured (1, all task related parameters where accepted) or not (0).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>comma-separated list of all detected and supported channels with respect to the task type. Every item consists of the device name / channel name</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>semicolon-separated list of all channels that should be part of this task. Every item is a comma separated string that defines and parameterizes every channel.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The sampling rate in samples per second per channel. If you use an external source for the Sample Clock, set this value to the maximum expected rate of that clock.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Timeout when reading up to &apos;samplesPerChannel&apos; values (per channel) in seconds. If -1.0 (default), the timeout is set to infinity (recommended for finite tasks). If 0.0, getVal/copyVal will return all values which have been recorded up to this call.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>If 1, the setVal call will block until all data has been written (only valid for finite tasks). If 0, setVal will return immediately, then use &apos;taskStarted&apos; to verify if the operation has been finished.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The number of samples to acquire or generate for each channel in the task (if taskMode is &apos;finite&apos;). If taskMode is &apos;continuous&apos;, NI-DAQmx uses this value to determine the buffer size. This parameter is ignored for output tasks.If &apos;samplesPerChannel&apos; is 1, one single value is read or written by asoftware trigger only. The parameters &apos;samplingRate&apos;, &apos;bufferSize&apos;, &apos;sampleClockSource&apos; and &apos;sampleClockRisingEdge&apos; are ignored then.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Sets and changes the automatic input / output buffer allocation mode. If -1 (default), the automatic allocation is enabled. Else defines the number of samples the buffer can hold for each channel (only recommended for continuous acquisition). In automatic mode and continuous acquisition, the standard is a buffer size of 1 kS for a sampling rate &lt; 100 S/s, 10 kS for 100-10000 S/s, 100 kS for 10-1000 kS/s and 1 MS else. For input tasks, this size changes the input buffer size of the device, else the output buffer size.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The source terminal of the Sample Clock. To use the internal clock of the device, use an empty string or &apos;OnboardClock&apos; (default). An example for an external clock source is &apos;PFI0&apos; or PFI1&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>If 1, samples are acquired on a rising edge of the sample clock (default), else they are acquired on a falling edge.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Specifies the start trigger mode. &apos;off&apos;: software-based start trigger, &apos;digitalEdge&apos;: The start of acquiring or generating samples is given if the &apos;startTriggerSource&apos; is activated (based on &apos;startTriggerRisingEdge&apos;), &apos;analogEdge&apos;: similar to &apos;digitalEdge&apos;, but the analog input &apos;startTriggerSource&apos; has to pass the value &apos;startTriggerLevel&apos;.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The source terminal of the trigger source (if &apos;startTriggerMode&apos; is set to &apos;digitalEdge&apos; or &apos;analogEdge&apos;).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Specifies on which slope of the signal to start acquiring or generating samples. 1: rising edge (default), 0: falling edge.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Only for &apos;startTriggerMode&apos; == &apos;analogEdge&apos;: The threshold at which to start acquiring or generating samples. Specify this value in the units of the measurement or generation.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>A reference trigger can be enabled to stop an acquisition when the device acquired all pre-trigger samples, an analog or digital signal reaches a specified level and and the device acquired all post-trigger samples. &apos;off&apos;: no reference trigger, &apos;digitalEdge&apos;: The trigger event is given if &apos;refTriggerSource&apos; is activated (based on &apos;refTriggerRisingEdge&apos;), &apos;analogEdge&apos;: similar to &apos;digitalEdge&apos;, but the analog input &apos;refTriggerSource&apos; has to pass the value &apos;refTriggerLevel&apos;. A reference trigger can only be set for finite, input tasks. The reference trigger can only be enabled for finite input tasks. However these tasks then behave like continuous tasks, but the stop command is automatically generated by the reference trigger event (plus additional post trigger values)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The source terminal of the trigger source (if &apos;refTriggerMode&apos; is set to &apos;digitalEdge&apos; or &apos;analogEdge&apos;).</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Specifies on which slope of the signal to stop acquiring samples. 1: rising edge (default), 0: falling edge.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Only for &apos;refTriggerMode&apos; == &apos;analogEdge&apos;: The threshold at which to stop acquiring samples. Specify this value in the units of the measurement or generation.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The minimum number of samples per channel to acquire before recognizing the Reference Trigger. The number of posttrigger samples per channel is equal to &apos;samplesPerChannel&apos; minus this value. </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>0: logging is disabled (default), 1: logging is enabled with disabled read (fast, but no data can simultaneously read via getVal/copyVal), 2: logging is enabled with allowed reading of data.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Specifies how to open the TDMS file. &apos;open&apos;: Always appends data to an existing TDMS file. If it does not exist yet, the task start operation will return with an error; &apos;openOrCreate&apos;: Creates a new TDMS file or appends data to the existing one;&apos;createOrReplace&apos; (default): Creates a new TDMS file or replaces an existing one; &apos;create&apos;: Newly creates the TDMS file. If it already exists a task start operation will return with an error.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The name of the group to create within the TDMS file for data from this task. If empty, the task name is taken. If data is appended to a TDMS file, a number symbol (e.g. Task #1, Task #2...) is added at each run.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The path to the TDMS file to which you want to log data. </source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>&apos;getVal&apos; cannot read data because none was acquired.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>&apos;copyVal&apos; - cannot read data because none was acquired.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>&apos;copyVal&apos; - Empty object handle provided by caller</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>NiDAQmx::retrieveData - cannot retrieve data because none was acquired.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during check data: The given dataObject must have one plane or an empty dataObject, that is then created with the right shape and type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during check data: The given dataObject must be of type &apos;float64&apos; or an empty dataObject, that is then created with the right shape and type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during check data: The given dataObject must be of type &apos;uint8&apos; or an empty dataObject, that is then created with the right shape and type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during check data: The given dataObject must be of type &apos;uint16&apos; or an empty dataObject, that is then created with the right shape and type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during check data: The given dataObject must be of type &apos;int32&apos; or an empty dataObject, that is then created with the right shape and type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during check data: The given dataObject has the wrong data type. Allocate it with the right type or pass an empty dataObject, that is then created with the right shape and type.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Error during check data: The given dataObject has the wrong shape. Pass an object with the right shape (%1 x %2) or pass an empty dataObject, that is then created with the right shape and type</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Detect available devices.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>NiDAQmx::readAnalog - Tried to return data without first calling acquire</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>NiDAQmx::readAnalog - internal error invalid mode passed to readAnalog</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>NiDAQmx::readDigital - Tried to return data without first calling acquire</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>NiDAQmx::readDigital - internal error invalid mode passed to readDigital</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>NiDAQmxInterface</name>
    <message>
        <source>type of the task related to this instance of the NI-DAQmx plugin (analogInput, digitalInput, analogOutput, digitalOutput)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>desired name of the underlying NI task (this might be changed by the NI task creation method)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>mode of the task recording / data generation: finite, continuous</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The sampling rate in samples per second per channel. If you use an external source for the Sample Clock, set this value to the maximum expected rate of that clock.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The number of samples to acquire or generate for each channel in the task (if taskMode is &apos;finite&apos;). If taskMode is &apos;continuous&apos;, NI-DAQmx uses this value to determine the buffer size.</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>QObject</name>
    <message>
        <source>Analog and digital input and output tasks for National Instruments devices, based on NI DAQmx.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The plugin implements the DAQmx functions for analog and digital I/O devices from National Instruments.
The installation needs the NI-DAQmx Library that can be downloaded from the NI website
(https://www.ni.com/en-us/support/downloads/drivers/download.ni-daqmx.html).</source>
        <translation type="unfinished"></translation>
    </message>
</context>
<context>
    <name>niDAQmx</name>
    <message>
        <source>Dialog</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>General</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task mode:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task name:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Task type:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[Type]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>[Identifier]</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Acquisition / Data Write</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Input / Output Buffer Size:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source> Hz</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source> s</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Block &apos;setVal&apos; until all data is written</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Samples Per Channel:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Data Read Timeout:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Sampling Rate:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>This corresponds to the number of samples that are read or written during one acquire() / setVal() for finite tasks</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>If -1, the buffer sizes are automatically chosen (default)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Sample Clock</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Sample Clock Source:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Sample Clock on Rising Edge</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Start Trigger</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Start Trigger Source:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Trigger on Rising Edge</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Start Trigger Mode:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Start Trigger Level:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Reference Trigger (Stop Trigger)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Reference Trigger Source:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Reference Trigger Level:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Reference Trigger Mode:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Number of Pre-Trigger Samples:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>The number of post-trigger samples is equal to &quot;Samples Per Channel&quot; - &quot;Number of Pre-Trigger Samples&quot;</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Channels</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Properties</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Minimum Voltage:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source> V</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Maximum Voltage:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Terminal Config:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Default = 0</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Diff = 1</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>RSE = 2</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>NRSE = 3</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>PseudoDiff = 4</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Current Channel Configuration String:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>TDMS Logging</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Logging Mode</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Off</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Automatic, fast logging enabled. No simultaneous getVal / copyVal operation possible.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Default logging enabled. Data is only logged upon each getVal / copyVal call.</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Logging Options</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>TDMS Group Name:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>If empty, the task name is taken</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>TDMS Filename:</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>TDMS-File (*.tdms)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>All Files (*.*)</source>
        <translation type="unfinished"></translation>
    </message>
    <message>
        <source>Operation Mode:</source>
        <translation type="unfinished"></translation>
    </message>
</context>
</TS>
