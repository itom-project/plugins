===================
 HBMSpider8
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`HMBSpider8`
**Type**:       :plugintype:`HBMSpider8`
**License**:    :pluginlicense:`HBMSpider8`
**Platforms**:  Windows, Linux
**Devices**:    Legacy HBM Spider 8 AD converter
**Author**:     :pluginauthor:`HBMSpider8`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: HBMSpider8

This plugins allows the use of the legacy HBM Spider 8 AD converter series
(https://www.hbm.com/en/2464/software-and-firmware-downloads-for-legacy-products/), offering port configuration using
the integrated plugin dialog. In order to connect to the device an open serial port on which the Spider is
connected must be provided when initializing.
Together with the plugin are provided three different dialogs showing the function of the plugin and which can also be
used to carry out simple measurement series of calibration tasks. The measureSeries dialog provides a small statistical
evaluation of the measured series values according to MSA.

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: HBMSpider8
		**SerialIO**: ito::ParamBase::HWRef
			An open serial port on which the HBM Spider 8 is connected

Parameters
===========

An instance of this plugin has the following parameters:

**binning**: int
    Binning of different pixel, binning = x-factor * 100 + y-factor

    *Value range: [101, 404], Default: 101*

**name**: string
	Returns the plugin name

**channelList**: string
	Returns a list with the available channels, readonly

**actChannelList**: string
	Returns a list with the currently active channels, readonly

**samplingRate**: double
	Sampling rate used for measurements (all channels)

	*Value range: [1.0, 9600.0], Default: 1200.0*

**trigger**: string
	trigger for starting measurement with trigger condition ( acquire(3) ) [Channel,Mode,Level], Mode - 0: above level, 1: below level, positive edge, negative edge, level: -32769 ... 32767

**bufferManagement**: int
	0: clear data on second read out, 1: clear data on first read out

	*Value range: [0, 1], Default: 1*

**numSamples**: int
	Number of samples for a single measurement. Defines in combination with the parameter *samplingRate* the measurement time

	*Value range: [1, 2000000000], Default: 1200*

**preTrgSamples**: int
	number of samples recorded before trigger is active

	*Value range: [1, 500], Default: 1*

**numCycles**: int
	number of measurement cycles of numSamples

	*Value range: [0, 2000000000], Default: 0*

**reset**: string
	resets the error state of the HBM, readonly

**statusStr**: string
	read out status string, readonly

**status**: int
	read current status
	0 no valid measured values available
	1 Spider8 is in a waiting phase, i.e. transients caused by switching to a different measuring range have not yet decayed.
	2 the pre-trigger buffer is being filled
	3 the pre-trigger buffer is full; waiting for trigger event
	4 the post-trigger buffer is being filled
	5 the acquisition terminated with error
	6 the acquisition terminated without error

	*value range: [0, 6], Default: 6*

**offsets**: double vector
	can be used to apply an offset to values after measurement, thus converting the valus from ADUs to a physical meaningful value
	in combination with the scale values. Offset is first.

	*Value range: [-inf, inf], Default: 0.0*

**scales**: double vector
	can be used to apply a scale to values after measurement, thus converting the valus from ADUs to a physical meaningful value
	in combination with the offsets values. Offset is first.

	*Value range: [-inf, inf], Default: 1.0*
