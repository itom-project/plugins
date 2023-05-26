===================
 USBMotion3XIII
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`USBMotion3XIII`
**Type**:       :plugintype:`USBMotion3XIII`
**License**:    :pluginlicense:`USBMotion3XIII`
**Platforms**:  Windows
**Devices**:    USB Motion 3x III controller from COPTONIX GmbH (www.coptonix.com)
**Author**:     :pluginauthor:`USBMotion3XIII`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: USBMotion3XIII

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: USBMotion3XIII

Parameters
===========

An instance of this plugin has the following internal parameters:

**name**: {str}, read-only
    USBMotion3XIII
**connected**: {int}, read-only
    indicates whether motor driver is connected (1) or not (0)
**serialNumber**: {str}, read-only
    serial number for this motor driver
**productVersion**: {str}, read-only
    product version for this motor driver
**vendorName**: {str}, read-only
    vendor name for this motor driver
**productName**: {str}, read-only
    product name for this motor driver
**async**: {int}
    asynchronous move (1), synchronous (0) [default]
**axisSteps1**: {float}, read-only
    number of full steps per turn of motor 1, 0: motor not connected
**vMin1**: {float}
    minimal speed in degree per second of motor 1
**vMax1**: {float}
    maximal speed in degree per second of motor 1
**aMax1**: {float}
    maximal acceleration in degree/s^2 of motor 1
**coilCurrentHigh1**: {float}
    coil current if acceleration is higher than coilCurrentThreshold1 [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentLow1**: {float}
    coil current if acceleration is lower than coilCurrentThreshold1 [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentRest1**: {float}
    coil current if motor 1 is in rest [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentThreshold1**: {float}
    threshold acceleration for distinction between coilCurrentHigh and coilCurrentLow
**axisEnabled1**: {int}
    determine if motor 1 is enabled (1) or disabled (0). If disabled, this motor is manually moveable
**microSteps1**: {int}
    micro steps for motor 1 [1,2,4,8,16,32,64]
**switchSettings1**: {int}
    bitmask of switch settings (bit 1: DISABLE_STOP_L, bit 2: DISABLE_STOP_R, bit 3: SOFT_STOP, bit 4: REF_RnL
**axisSteps2**: {float}, read-only
    number of full steps per turn of motor 2, 0: motor not connected
**vMin2**: {float}
    minimal speed in degree per second of motor 2
**vMax2**: {float}
    maximal speed in degree per second of motor 2
**aMax2**: {float}
    maximal acceleration in degree/s^2 of motor 2
**coilCurrentHigh2**: {float}
    coil current if acceleration is higher than coilCurrentThreshold2 [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentLow2**: {float}
    coil current if acceleration is lower than coilCurrentThreshold2 [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentRest2**: {float}
    coil current if motor 2 is in rest [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentThreshold2**: {float}
    threshold acceleration for distinction between coilCurrentHigh and coilCurrentLow
**axisEnabled2**: {int}
    determine if motor 2 is enabled (1) or disabled (0). If disabled, this motor is manually moveable
**microSteps2**: {int}
    micro steps for motor 2 [1,2,4,8,16,32,64]
**switchSettings2**: {int}
    bitmask of switch settings (bit 1: DISABLE_STOP_L, bit 2: DISABLE_STOP_R, bit 3: SOFT_STOP, bit 4: REF_RnL
**axisSteps3**: {float}, read-only
    number of full steps per turn of motor 3, 0: motor not connected
**vMin3**: {float}
    minimal speed in degree per second of motor 3
**vMax3**: {float}
    maximal speed in degree per second of motor 3
**aMax3**: {float}
    maximal acceleration in degree/s^2 of motor 3
**coilCurrentHigh3**: {float}
    coil current if acceleration is higher than coilCurrentThreshold3 [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentLow3**: {float}
    coil current if acceleration is lower than coilCurrentThreshold3 [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentRest3**: {float}
    coil current if motor 3 is in rest [12.5%, 25%, ... 87.5%, 100%]
**coilCurrentThreshold3**: {float}
    threshold acceleration for distinction between coilCurrentHigh and coilCurrentLow
**axisEnabled3**: {int}
    determine if motor 3 is enabled (1) or disabled (0). If disabled, this motor is manually moveable
**microSteps3**: {int}
    micro steps for motor 3 [1,2,4,8,16,32,64]
**switchSettings3**: {int}
    bitmask of switch settings (bit 1: DISABLE_STOP_L, bit 2: DISABLE_STOP_R, bit 3: SOFT_STOP, bit 4: REF_RnL
