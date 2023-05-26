/* ********************************************************************
    Plugin "SuperlumBL" for itom software
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

#include "dialogSuperlumBL.h"
//#include "SuperlumBL.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//#include "common/addInInterface.h"

//#include <qmetaobject.h>
//#include <qdialogbuttonbox.h>
//#include <qmessagebox.h>

//#include "ui_dialogSuperlumBL.h"

//#include <qdialog.h>
//#include <qstring.h>
//#include <qmap.h>
//#include <qabstractbutton.h>
//#include <qvector.h>
//#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogSuperlumBL::DialogSuperlumBL(ito::AddInBase *motor) :
    AbstractAddInConfigDialog(motor),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSuperlumBL::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        enableDialog(true);
		//use params (identical to m_params of the plugin)
        //__________________________________________________________________________________________________________ General Information
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        ui.label_Device->setText(params["serial_number"].getVal<char*>());
        ui.label_COM_Port->setText(QString::number(params["comPort"].getVal<int>()));
        ui.label_device_name->setText(QString((params)["name"].getVal<char*>()));

        //__________________________________________________________________________________________________________ Remote
        ui.comboBox_Remote->setCurrentIndex( params["local"].getVal<int>());

        //__________________________________________________________________________________________________________ Optical Output
        if (params["optical_output"].getVal<int>() == 1)
        {
            ui.btn_Optical_Output->setChecked( 1 );
        }
        else if (params["optical_output"].getVal<int>() == 0)
        {
            ui.btn_Optical_Output->setChecked( 0 );
        }

        //__________________________________________________________________________________________________________ Power Mode
        ui.comboBox_Power_Mode->setCurrentIndex( params["power_mode"].getVal<int>());

        m_firstRun = false;
    }

    //__________________________________________________________________________________________________________ Editing
    if (!m_inEditing)
    {
        m_inEditing = true;

        //__________________________________________________________________________________________________________ Power Mode
        if (params.contains("power_mode"))
        {
            ui.comboBox_Power_Mode->setCurrentIndex( params["power_mode"].getVal<int>());
        }

        //__________________________________________________________________________________________________________ Remote
        if (params.contains("local"))
        {
            if (params["local"].getVal<int>() == 1)
            {
                enableDialog(true);
            }
            else if (params["local"].getVal<int>() == 0)
            {
                enableDialog(false);
            }
            ui.comboBox_Remote->setCurrentIndex( params["local"].getVal<int>());
        }

        //__________________________________________________________________________________________________________ Optical Output
        if (params.contains("optical_output"))
        {
            if (params["optical_output"].getVal<int>() == 1)
            {
                ui.btn_Optical_Output->setChecked( 1 );
            }
            else if (params["optical_output"].getVal<int>() == 0)
            {
                ui.btn_Optical_Output->setChecked( 0 );
            }
        }

        m_inEditing = false;
    }

    m_currentParameters = params;
}

//---------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogSuperlumBL::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    int val;
    int valpower;
    bool valoptical;

    //__________________________________________________________________________________________________________ Power Mode
    valpower = ui.comboBox_Power_Mode->currentIndex();
    if (m_currentParameters["power_mode"].getVal<int>() != valpower)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("power_mode", ito::ParamBase::Int, valpower)));
    }

    //__________________________________________________________________________________________________________Remote/ Local
    val = ui.comboBox_Remote->currentIndex();
    if (m_currentParameters["local"].getVal<int>() != val)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("local", ito::ParamBase::Int, val)));
    }

    //__________________________________________________________________________________________________________Optical Output
    valoptical = ui.btn_Optical_Output->isChecked();
    val = int(valoptical);
    if (m_currentParameters["optical_output"].getVal<int>() != val)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("optical_output", ito::ParamBase::Int, val)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSuperlumBL::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    //cancel button, emit reject() -> dialog is closed
    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    //ok button, emit accept() -> dialog is closed
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else //apply button, only call applyParameters
    {
        applyParameters(); //ApplyRole
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSuperlumBL::enableDialog(bool enabled)
{
    ui.groupBox_Power->setEnabled(enabled);
}
