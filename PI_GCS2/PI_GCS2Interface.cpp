/* ********************************************************************
    Plugin "PI_GCS2" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "PI_GCS2Interface.h"
#include <QtCore/QtPlugin>
#include "pluginVersion.h"
#include "gitVersion.h"

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of PIPiezoCtrlInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created PIPiezoCtrlInterface-instance is stored in *addInInst
    \return retOk
    \sa PIPiezoCtrl
*/
ito::RetVal PI_GCS2Interface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PIPiezoCtrl)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of PIPiezoCtrlInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa PIPiezoCtrl
*/
ito::RetVal PI_GCS2Interface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(PIPiezoCtrl)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
PI_GCS2Interface::PI_GCS2Interface()
{
    m_type = ito::typeActuator;

    setObjectName("PI_GCS2");

    m_description = QObject::tr("PI actuators following the GCS2 command set (e.g. E753)");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = "PI actuators following the GCS2 command set (e.g. E753) \n\
This plugin is developped for single axis controllers following the GCS2 commandset. \n\
\n\
Tested with E753, don't work with E-662 and E-665! \n\
\n\
For the initialization you can connect to the device (if possible) via a USB port, a serial port \n\
or a TCP/IP connection. Depending on the connection you should use the initialization parameters \n\
in the following way: \n\
* RS232: give the COM-port number (number only in Windows, COM-port name in Linux) as deviceName and indicate a valid baudrate as optional parameter. \n\
* TCP/IP: give the full name of the device as deviceName or let deviceName empty in order to print out a list of all detected devices (the device name is one full line of the output! \n\
* USB: similar to TCP/IP \n\
\n\
Please note that you only need to indicate a baudrate for RS232 connections, in the other cases, \n\
the default baudrate 0 forces the controller to connect with a default baudrate which is recommended for TCP/IP or USB connections.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("PI actuators following the GCS2 command set (e.g. E753) \n\
This plugin is developped for single axis controllers following the GCS2 commandset. \n\
\n\
Tested with E753 and C663, don't work with E-662 and E-665! \n\
\n\
For the initialization you can connect to the device (if possible) via a USB port, a serial port \n\
or a TCP/IP connection. Depending on the connection you should use the initialization parameters \n\
in the following way: \n\
* RS232: give the COM-port number (number only in Windows, COM-port name in Linux) as deviceName and indicate a valid baudrate as optional parameter. \n\
* TCP/IP: give the full name of the device as deviceName or let deviceName empty in order to print out a list of all detected devices (the device name is one full line of the output! \n\
* USB: similar to TCP/IP \n\
\n\
Please note that you only need to indicate a baudrate for RS232 connections, in the other cases, \n\
the default baudrate 0 forces the controller to connect with a default baudrate which is recommended for TCP/IP or USB connections.");

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);    

    ito::Param p("connection", ito::ParamBase::String, "RS232", "type of the connection ('RS232', 'USB', 'TCPIP')");
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "RS232");
    sm->addItem("USB");
    sm->addItem("TCPIP");
    p.setMeta(sm, true);
    m_initParamsMand.append(p);

    m_initParamsMand.append( ito::Param("deviceName", ito::ParamBase::String, "", "name of the device to connect or in case of a RS232 connection the number of the com port or the name of the com device (linux)"));

    m_initParamsOpt.append(ito::Param("baudRate", ito::ParamBase::Int, 0, 512000, 0, "baudrate to use for a RS232 or USB connection. If 0 is given, a default baudrate is used"));
    //this->m_initParamsMand.append
}