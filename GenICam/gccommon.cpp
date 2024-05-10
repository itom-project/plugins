/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#include "gccommon.h"

#include <qobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal checkGCError(const GenTL::GC_ERROR &error, const QString &suffix /*= ""*/)
{
    if (error == GenTL::GC_ERR_SUCCESS)
    {
        return ito::retOk;
    }

    QString suffix_(suffix);
    if (suffix_.size() > 0)
    {
        suffix_.append(" (GenTL error):");
    }
    else
    {
        suffix_ = "(GenTL error):";
    }

    switch (error)
    {
    case GenTL::GC_ERR_ERROR:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 general error").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_NOT_INITIALIZED:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_NOT_INITIALIZED").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_NOT_IMPLEMENTED:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_NOT_IMPLEMENTED").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_RESOURCE_IN_USE:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_RESOURCE_IN_USE").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_ACCESS_DENIED:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_ACCESS_DENIED").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_INVALID_HANDLE:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_INVALID_HANDLE").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_INVALID_ID:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_INVALID_ID").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_NO_DATA:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_NO_DATA").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_INVALID_PARAMETER:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_INVALID_PARAMETER").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_IO:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_IO").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_TIMEOUT:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_TIMEOUT").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_ABORT:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_ABORT").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_INVALID_BUFFER:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_INVALID_BUFFER").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_NOT_AVAILABLE:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_NOT_AVAILABLE").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_INVALID_ADDRESS:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_INVALID_ADDRESS").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_BUFFER_TOO_SMALL:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_BUFFER_TOO_SMALL").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_INVALID_INDEX:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_INVALID_INDEX").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_PARSING_CHUNK_DATA:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_PARSING_CHUNK_DATA").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_INVALID_VALUE:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_INVALID_VALUE").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_RESOURCE_EXHAUSTED:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_RESOURCE_EXHAUSTED").arg(suffix_).toLatin1().constData());
    case GenTL::GC_ERR_OUT_OF_MEMORY:
        return ito::RetVal(ito::retError, error, QObject::tr("%1 GC_ERR_OUT_OF_MEMORY").arg(suffix_).toLatin1().constData());
    }

    return ito::RetVal(ito::retError, error, QObject::tr("%1 unknown error").arg(suffix_).toLatin1().constData());

}
