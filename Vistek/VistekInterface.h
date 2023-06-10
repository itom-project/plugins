/* ********************************************************************
    Plugin "Vistek" for itom software
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

#ifndef VISTEKINTERFACE_H
#define VISTEKINTERFACE_H

#include "common/addInGrabber.h"

#include <qsharedpointer.h>
#include <QTimerEvent>
#include <qmutex.h>

class VistekInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        VistekInterface(QObject *parent = 0);
        ~VistekInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

        //! auto-increment, static instance counter for all dummy-grabber instances
        static int m_instCounter;
};

#endif // VISTEKINTERFACE_H
