/* ********************************************************************
    Plugin "QuantumComposer" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "dialogQuantumComposer.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogQuantumComposer::DialogQuantumComposer(ito::AddInBase* instance) :
    AbstractAddInConfigDialog(instance),
    m_firstRun(true)
{
    ui.setupUi(this);
    
    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableGUI(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogQuantumComposer::~DialogQuantumComposer()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogQuantumComposer::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        setWindowTitle(QString(params["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        m_firstRun = false;
    }
    
    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableGUI(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogQuantumComposer::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    
    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
void DialogQuantumComposer::enableGUI(bool enabled)
{
}