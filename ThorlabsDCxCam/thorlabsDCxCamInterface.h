/* ********************************************************************
    Plugin "ThorlabsDCxCam" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2016, Institut für Technische Optik, Universität Stuttgart

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

#if !defined(THORLABSDCXCAMINTERFACE_H)
#define THORLABSDCXCAMINTERFACE_H

// itom standard-stuff
#include "common/addInInterface.h"
#include "common/addInGrabber.h"

class ThorlabsDCxCamInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:

        /**
         *  Defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: Vistek) should or must
         *  be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
         *  and m_initParamsOpt within this constructor.
         *
         *  @param [in] parent is the plugin interface's parent object
         **/
        ThorlabsDCxCamInterface(QObject *parent = 0);
        ~ThorlabsDCxCamInterface();

        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal checkVersionConsistency();
};

#endif // #if !defined(THORLABSDCXCAMINTERFACE_H)
