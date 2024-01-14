/* ********************************************************************
    Plugin "DummyMultiChannelGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dummyMultiChannelGrabberInterface.h"

#include <QtCore/QtPlugin>

#include "gitVersion.h"
#include "pluginVersion.h"
#include "dummyMultiChannelGrabber.h"


//-------------------------------------------------------------------------------------
/*!
    \class DummyMultiChannelGrabberInterface
    \brief Small interface class for class DummyMultiChannelGrabber. This class contains basic
   information about DummyMultiChannelGrabber as is able to create one or more new instances of
   DummyMultiChannelGrabber.
*/

//-------------------------------------------------------------------------------------
//! creates new instance of DummyMultiChannelGrabber and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created
   DummyMultiChannelGrabber-instance is stored in *addInInst \return retOk \sa
   DummyMultiChannelGrabber
*/
ito::RetVal DummyMultiChannelGrabberInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(DummyMultiChannelGrabber)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
//! deletes instance of DummyMultiChannelGrabber. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa DummyMultiChannelGrabber
*/
ito::RetVal DummyMultiChannelGrabberInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(DummyMultiChannelGrabber)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
//! constructor for interace
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real
   plugin (here: DummyMultiChannelGrabber) should or must be initialized (e.g. by a Python call)
   with mandatory or optional parameters, please initialize both vectors m_initParamsMand and
   m_initParamsOpt within this constructor.
*/
DummyMultiChannelGrabberInterface::DummyMultiChannelGrabberInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("DummyMultiChannelGrabber");

    // for the docstring, please don't set any spaces at the beginning of the line.
    /*    char docstring[] = \
    "The DummyMultiChannelGrabber is a virtual camera which emulates a camera with white noise. \n\
    \n\
    The camera is initialized with a maximum width and height of the simulated camera chip (both
    need to be a multiple
    of 4). \
    The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per
    pixel). The real size of
    the camera \
    image is controlled using the parameter 'roi' if the sizes stay within the limits given by the
    size of the camera
    chip.\n\
    \n\
    You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line
    camera whose height is
    equal to 1. \n\
    \n\
    This plugin can also be used as template for other grabber.";*/

    m_description = QObject::tr("A virtual white noise grabber");
    //    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
        "The DummyMultiChannelGrabber is a virtual camera which emulates a camera with multiple channels with white noise. \n\
\n\
The camera is initialized with a maximum width and height of the simulated camera chip (both need to be a multiple of 4). \
The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). The real size of the camera \
image is controlled using the parameter 'roi' if the sizes stay within the limits given by the size of the camera chip.\n\
\n\
You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line camera whose height is equal to 1. \n\
\n\
This plugin can also be used as template for other grabbers.");

    m_author = "R. Hahn, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = CREATEVERSION(1, 4, 0);
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LPGL.");
    m_aboutThis = tr(GITVERSION);

    m_initParamsMand.clear();

    ito::Param param(
        "sensorWidth",
        ito::ParamBase::Int,
        640,
        new ito::IntMeta(4, 4096, 4),
        tr("Width of sensor chip. In this demo, the width of the sensor is the same for "
            "all channels (could be different, if implemented).").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param(
        "sensorHeight",
        ito::ParamBase::Int,
        480,
        new ito::IntMeta(1, 4096, 1),
        tr("Height of sensor chip. In this demo, the height is the same for all "
            "channels. However, it could also be different, if implemented.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param(
        "pixelFormatChannel1",
        ito::ParamBase::String,
        "mono8",
        tr("Pixel format for the 1st channel (here: a grayscale intensity image).").toLatin1().data());
    ito::StringMeta* m = new ito::StringMeta(ito::StringMeta::String, "mono8");
    m->addItem("mono10");
    m->addItem("mono12");
    m->addItem("mono16");
    param.setMeta(m, true);
    m_initParamsOpt.append(param);

    param = ito::Param(
        "pixelFormatChannel2",
        ito::ParamBase::String,
        "float32",
        tr("Pixel format for the 2nd channel (here: a float32 or float64 disparity image).").toLatin1().data());
    m = new ito::StringMeta(ito::StringMeta::String, "float32");
    m->addItem("float64");
    param.setMeta(m, true);
    m_initParamsOpt.append(param);

    param = ito::Param(
        "pixelFormatChannel3",
        ito::ParamBase::String,
        "rgba8",
        tr("Pixel format for the 3nd channel (here: color image with or without alpha channel).").toLatin1().data());
    m = new ito::StringMeta(ito::StringMeta::String, "rgba8");
    m->addItem("rgb8");
    param.setMeta(m, true);
    m_initParamsOpt.append(param);
}

//-------------------------------------------------------------------------------------
//! destructor
/*!

*/
DummyMultiChannelGrabberInterface::~DummyMultiChannelGrabberInterface()
{
}
