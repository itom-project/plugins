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
    
    enableGUI(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogQuantumComposer::~DialogQuantumComposer()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogQuantumComposer::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString(params["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        
        ui.spinBoxTimeout->setValue(params["requestTimeout"].getVal<int>());

        ito::StringMeta* sm = (ito::StringMeta*)(params["mode"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxMode->addItem(sm->getString(x));
            int index = ui.comboBoxMode->findText(params["mode"].getVal<char*>());
        }

        m_firstRun = false;
    }

    enableGUI(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogQuantumComposer::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    if (ui.spinBoxTimeout->isEnabled())
    {
        int bin = ui.spinBoxTimeout->value();
        if (m_currentParameters["requestTimeout"].getVal<int>() != bin)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("requestTimeout", ito::ParamBase::Int, bin)));
        }
    }

    if (ui.comboBoxMode->isEnabled())
    {
        QString mode = ui.comboBoxMode->currentText();
        if (m_currentParameters["mode"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase(
                "mode", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    
    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
void DialogQuantumComposer::enableGUI(bool enabled)
{
}
