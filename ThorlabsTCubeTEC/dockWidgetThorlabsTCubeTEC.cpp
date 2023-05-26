/* ********************************************************************
Plugin "ThorlabsTCubeTEC" for itom software
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

#include "dockWidgetThorlabsTCubeTEC.h"
#include "Thorlabs.MotionControl.TCube.TEC.h"

//-------------------------------------------------------------------------------------
DockWidgetThorlabsTCubeTEC::DockWidgetThorlabsTCubeTEC(ito::AddInDataIO* plugin) :
    AbstractAddInDockWidget(plugin), m_inEditing(false), m_firstRun(true), m_unitSuffix("")
{
    ui.setupUi(this);
}

//-------------------------------------------------------------------------------------
void DockWidgetThorlabsTCubeTEC::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        m_inEditing = true;
        QByteArray sensorType = params["sensorType"].getVal<const char*>();
        ui.lblDeviceName->setText(params["deviceName"].getVal<const char*>());
        ui.lblSensorType->setText(sensorType);


        if (sensorType == "Transducer")
        {
            m_unitSuffix = QLatin1String(" \u00B0C"); // degree Celcius
        }
        else if (sensorType == "TH20kOhm")
        {
            m_unitSuffix = " kOhm";
        }
        else if (sensorType == "TH200kOhm")
        {
            m_unitSuffix = " kOhm";
        }
        else
        {
            m_unitSuffix = "";
        }

        ui.spinTarget->setSuffix(m_unitSuffix);
        ui.spinTarget->setMinimum(params["targetTemperature"].getMin());
        ui.spinTarget->setMaximum(params["targetTemperature"].getMax());

        m_firstRun = false;
        m_inEditing = false;
    }

    // update the current temperature
    ui.lblCurrent->setText(QString("%1%2")
                               .arg(params["currentTemperature"].getVal<double>(), 0, 'f', 2)
                               .arg(m_unitSuffix));

    // update the target temperature
    ui.spinTarget->setValue(params["targetTemperature"].getVal<double>());
}

//-------------------------------------------------------------------------------------
void DockWidgetThorlabsTCubeTEC::on_spinTarget_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(
            new ito::ParamBase("targetTemperature", ito::ParamBase::Double, value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------
void DockWidgetThorlabsTCubeTEC::identifierChanged(const QString& identifier)
{
    ui.lblIdentifier->setText(identifier);
}
