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

#ifndef GCCOMMON_H
#define GCCOMMON_H

#include "common/retVal.h"
#include <qstring.h>

#include "GenTL_v1_5.h"

#define VERBOSE_ERROR 1
#define VERBOSE_WARNING 2
#define VERBOSE_INFO 3
#define VERBOSE_DEBUG 4
#define VERBOSE_ALL 5

#define FRAMEGRABBER_PREFIX "Fg_"

ito::RetVal checkGCError(const GenTL::GC_ERROR &error, const QString &suffix = "");

#endif // GCCOMMON_H
