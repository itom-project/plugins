===================
 DummyGrabber
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`DummyGrabber`
**Type**:       :plugintype:`DummyGrabber`
**License**:    :pluginlicense:`DummyGrabber`
**Platforms**:  Windows, Linux
**Devices**:    Virtual camera providing random noise images
**Author**:     :pluginauthor:`DummyGrabber`
=============== ========================================================================================================

Overview
========

.. pluginsummaryextended::
    :plugin: DummyGrabber

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: DummyGrabber

Parameters
===========

An instance of this plugin has the following parameters:

**binning**: int
    Binning of different pixel, binning = x-factor * 100 + y-factor

    *Value range: [101, 404], Default: 101*
**bpp**: int
    bitdepth of images

    *Value range: [8:2:30], Default: 8*
**demoArbitraryString**: str
    any string allowed

    *Match: <no pattern given>, Default: "any string"*
**demoEnumString**: str
    enumeration string (mode 1, mode 2, mode 3)

    *Match: ["mode 1", "mode 2", "mode 3"], Default: "mode 1"*
**demoEnumStringList**: Sequence[str]
    one or two options allowed.

    *Allowed number of values: 1 - 2, Value rules: Match: ["option1", "option2", "option3"],
    Default: [option1, option3]*
**demoRegexpString**: str
    matches strings without whitespaces

    *RegExp: "^\S+$", Default: <empty str>*
**demoWildcardString**: str
    dummy filename of a bmp file, pattern: *.bmp

    *Wildcard: "*.bmp", Default: "test.bmp"*
**frame_time**: float
    Minimum time between the start of two consecutive acquisitions [s], default: 0.0.

    *Value range: [0, 60], Unit: s, Default: 0*
**gain**: float
    Virtual gain

    *Value range: [0, 1], Default: 1*
**integration_time**: float
    Minimum integration time for an acquisition [s], default: 0.0 (as fast as possible).

    *Value range: [0, 60], Unit: s, Default: 0*
**name**: str, read-only
    GrabberName

    *Match: "General", Default: "DummyGrabber"*
**offset**: float
    Virtual offset

    *Value range: [0, 1], Default: 0*
**roi**: Tuple[int,int,int,int] (rect [x0,y0,width,height])
    ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]

    *Height: Value range: [0:4:479], Default: [0, 0, 640, 480]*
**sizex**: int, read-only
    size in x (cols) [px]

    *Value range: [4:4:640], Default: 640*
**sizey**: int, read-only
    size in y (rows) [px]

    *Value range: [4:4:480], Default: 480*


Changelog
==========

* itom setup 1.2.0: Release
