/* ********************************************************************
    Plugin "GWInstekPSP" for itom software
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

#include "dockWidgetGWInstekPSP.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetGWInstekPSP::DockWidgetGWInstekPSP(QMap<QString, ito::Param> params, int uniqueID) :
    m_timerID(-1)
{
    ui.setupUi(this);

    char *temp = params["name"].getVal<char*>(); //borrowed reference
//    ui.lblName->setText(temp);
    ui.lblID->setText(QString::number(uniqueID));

    valuesChanged(params);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGWInstekPSP::valuesChanged(QMap<QString, ito::Param> params)
{
    QMap<QString, ito::Param>::const_iterator paramIt1, paramIt2;

    paramIt1 = params.constFind("voltage");    // To check if this parameter exist
    paramIt2 = params.constFind("voltage_limit");    // To check if this parameter exist
    if (paramIt1 != params.constEnd() && paramIt2 != params.constEnd())
    {
        ui.dbVoltage->setMaximum(params["voltage_limit"].getVal<double>());
        ui.dbVoltage->setValue(params["voltage"].getVal<double>());

        ui.dbVoltageLimit->setMaximum(params["voltage_limit"].getMax());
        ui.dbVoltageLimit->setValue(params["voltage_limit"].getVal<double>());
    }

    paramIt1 = params.constFind("current");    // To check if this parameter exist
    paramIt2 = params.constFind("current_limit");    // To check if this parameter exist
    if (paramIt1 != params.constEnd() && paramIt2 != params.constEnd())
    {
        ui.dbCurrent->setMaximum(params["current_limit"].getVal<double>());
        ui.dbCurrent->setValue(params["current"].getVal<double>());

        ui.dbCurrentLimit->setMaximum(params["current_limit"].getMax());
        ui.dbCurrentLimit->setValue(params["current_limit"].getVal<double>());
    }

    paramIt1 = params.constFind("load");    // To check if this parameter exist
    paramIt2 = params.constFind("load_limit");    // To check if this parameter exist
    if (paramIt1 != params.constEnd() && paramIt2 != params.constEnd())
    {
        ui.dbLoad->setMaximum(params["load_limit"].getVal<double>());
        ui.dbLoad->setValue(params["load"].getVal<double>());

        ui.dbLoadLimit->setMaximum(params["load_limit"].getMax());
        ui.dbLoadLimit->setValue(params["load_limit"].getVal<double>());
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGWInstekPSP::on_cbAuto_clicked()
{
    ui.dbVoltage->setEnabled(!ui.cbAuto->isChecked());
    ui.pbStart->setEnabled(!ui.cbAuto->isChecked());

    if (ui.cbAuto->isChecked())
    {
        m_timerID=startTimer(1000);
    }
    else
    {
        if (m_timerID > -1)
        {
            killTimer(m_timerID);
            m_timerID = -1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGWInstekPSP::timerEvent(QTimerEvent * /*event*/)
{
    setParamVoltage(ui.dbVoltage->value());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGWInstekPSP::on_pbStart_clicked()
{
    setParamVoltage(ui.dbVoltage->value());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
