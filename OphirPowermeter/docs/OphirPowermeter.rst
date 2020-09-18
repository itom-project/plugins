===================
 OphirPowermeter
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`OphirPowermeter`
**Type**:       :plugintype:`OphirPowermeter`
**License**:    :pluginlicense:`OphirPowermeter`
**Platforms**:  Windows, (Linux possible but yet not implemented)
**Devices**:    Powermeter VEGA from company *Ophir*
**Author**:     :pluginauthor:`OphirPowermeter`
=============== ========================================================================================================
 
Overview
========

.. pluginsummaryextended::
    :plugin: OphirPowermeter
    
Initialization
==============
  
The following parameters are mandatory or optional for initializing an instance of this plugin:
    
    .. plugininitparams::
        :plugin: OphirPowermeter

Parameters
==========


Additional functions (exec functions)
=====================================


Exemplary usage from Python
============================

In the following script, the first detectable power meter is connected and a oscilloscope-like
plot is opened that displays a moving graph of recent intensity values:

.. code-block:: python
	
    adda = dataIO("OphirPowermeter", "")
    
	numPoints = 1000
	image = dataObject.zeros([1, numPoints], 'float64')
	[i, plotHandle] = plot1(image)

	def timeout():
		global timerId
		d = dataObject()
		adda.acquire() #acquire new intensity value
		
		image[0,0: numPoints - 1] = image[0, 1:] #shift pixels to the left by one...
		
		adda.getVal(d) #get the recently acquired value
		image.copyMetaInfo(d)
		image[0, numPoints - 1] = d[0, 0] #...append new value to the end of image
		
		if plotHandle.exists():
			try:
				plotHandle["source"] = image #update the displayed image
			except:
				pass
		else:
			print("Figure has been closed. Stop acquisition...")
			timerId.stop()
			del timerId

	timerId = timer(50, timeout) #call timeout every 50ms

Changelog
=========
