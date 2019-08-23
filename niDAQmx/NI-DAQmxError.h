/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
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

#ifndef niDAQmxError_H
#define niDAQmxError_H

#include "common/retVal.h"
#include <qstring.h>
#include "NIDAQmx.h"


/* This namespace encapsulates the single function checkError(), which
    is used in both NI-DAQmx.cpp and NI-PeripheralClasses.cpp. It checks
    return value of a NI C library routine call and translates it into
    readable text.
*/

namespace niDAQmxError
{
    ito::RetVal checkError(int error, const QString&);
}

using niDAQmxError::checkError;

#endif //#define NI-DAQmxError_h
