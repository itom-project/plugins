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

#include "dockWidgetSuperlumBL.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetSuperlumBL::DockWidgetSuperlumBL(int uniqueID, ito::AddInBase *dataIO) :
    AbstractAddInDockWidget(dataIO),
    m_inEditing(false),
    m_firstRun(true)
{
     ui.setupUi(this);

     identifierChanged(QString::number(uniqueID));

     enableWidget(true);
}

 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetSuperlumBL::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        enableWidget(true);
		//use params (identical to m_params of the plugin)
        //and initialize all widgets (e.g. min, max values, labels, enable some,...)
        //__________________________________________________________________________________________________________ General Information
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
                enableWidget(true);
            }
            else if (params["local"].getVal<int>() == 0)
            {
                enableWidget(false);
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

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSuperlumBL::on_btn_Optical_Output_clicked(bool checked)
{
    if (!m_inEditing) //only send the value to the plugin if not inEditing mode
    {
        m_inEditing = true;
        if ( checked )
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("optical_output",ito::ParamBase::Int,1));
            setPluginParameter(p, msgLevelWarningAndError);
        }
        else if ( !checked)
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("optical_output",ito::ParamBase::Int,0));
            setPluginParameter(p, msgLevelWarningAndError);
        }
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSuperlumBL::on_comboBox_Remote_activated(int combo)
{
    if (!m_inEditing) //only send the value to the plugin if not inEditing mode
    {
        m_inEditing = true;
        if ( combo == 1 )
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("local",ito::ParamBase::Int,1));
            setPluginParameter(p, msgLevelWarningAndError);
        }
        else if ( combo == 0 )
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("local",ito::ParamBase::Int,0));
            setPluginParameter(p, msgLevelWarningAndError);
        }
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSuperlumBL::on_comboBox_Power_Mode_activated(int combo)
{
    if (!m_inEditing) //only send the value to the plugin if not inEditing mode
    {
        m_inEditing = true;
        if ( combo == 1 )
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("power_mode",ito::ParamBase::Int,1));
            setPluginParameter(p, msgLevelWarningAndError);
        }
        else if ( combo == 0 )
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("power_mode",ito::ParamBase::Int,0));
            setPluginParameter(p, msgLevelWarningAndError);
        }
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSuperlumBL::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSuperlumBL::enableWidget(bool enabled)
{
    ui.groupBox_Power->setEnabled(enabled);
}
