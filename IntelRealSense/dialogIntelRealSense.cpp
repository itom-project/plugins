/* ********************************************************************
Plugin "IntelRealSense" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut fuer Technische Optik (ITO),
Universitaet Stuttgart, Germany

This file is part of a plugin for the measurement software itom.

This itom-plugin is free software; you can redistribute it and/or modify it
under the terms of the Apache Licence as published INTEL.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dialogIntelRealSense.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogIntelRealSense::DialogIntelRealSense(ito::AddInBase *grabber) :
	AbstractAddInConfigDialog(grabber),
	m_firstRun(true)
	
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known yet. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
	QPointer<ito::AddInBase> m_pluginPointer(grabber);
	ui.paramEditorWidget->setPlugin(m_pluginPointer);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogIntelRealSense::parametersChanged(QMap<QString, ito::Param> params)
{
    //save the currently set parameters to m_currentParameters
    m_currentParameters = params;
    
    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        m_firstRun = false;
        
    }
	//now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
	enableDialog(true);

    //set the status of all widgets depending on the values of params

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogIntelRealSense::applyParameters()
{
    ito::RetVal retValue(ito::retOk);

	QVector<QSharedPointer<ito::ParamBase> > values = ui.paramEditorWidget->getAndResetChangedParameters();

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogIntelRealSense::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else
    {
        applyParameters(); //ApplyRole
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogIntelRealSense::enableDialog(bool enabled)
{
	ui.paramEditorWidget->setEnabled(enabled);
}