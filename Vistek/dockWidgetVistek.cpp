/* ********************************************************************
    Plugin "Vistek" for itom software
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

#include "dockWidgetVistek.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetVistek::DockWidgetVistek(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::parametersChanged(QMap<QString, ito::Param> params)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.lblBitDepth->setText(QString::number(params["bpp"].getVal<int>()));
        ui.lblWidth->setText(QString::number(params["sizex"].getVal<int>()));
        ui.lblHeight->setText(QString::number(params["sizey"].getVal<int>()));

        ui.sliderGain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );
        ui.sliderOffset->setDisabled( params["offset"].getFlags() & ito::ParamBase::Readonly );
        ui.sliderExposure->setDisabled( params["integration_time"].getFlags() & ito::ParamBase::Readonly );

        if (params.contains("cameraModel"))
        {
            ui.ModelLabel->setText(params["cameraModel"].getVal<char*>());
        }

        if (params.contains("cameraSerialNo"))
        {
            ui.SerialLabel->setText(params["cameraSerialNo"].getVal<char*>());
        }

        if (params.contains("cameraIP"))
        {
            ui.IPLabel->setText(params["cameraIP"].getVal<char*>());
        }

        if (params.contains("cameraManufacturer"))
        {
            ui.ManufacturerLabel->setText(params["cameraManufacturer"].getVal<char*>());
        }

        if (params.contains("integration_time"))
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["integration_time"].getMeta());
            ui.sliderExposure->setMinimum(dm->getMin() * 1000);
            ui.sliderExposure->setMaximum(dm->getMax() * 1000);
            ui.sliderExposure->setSingleStep((dm->getMax() - dm->getMin()) * 10);
            ui.sliderExposure->setValue(params["integration_time"].getVal<double>() * 1000);
        }

        if (params.contains("gain"))
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.sliderGain->setMinimum(dm->getMin());
            ui.sliderGain->setMaximum(dm->getMax());
            ui.sliderGain->setSingleStep((dm->getMax() - dm->getMin()) / 100);
            ui.sliderGain->setValue(params["gain"].getVal<double>());
        }

        if (params.contains("offset"))
        {
            ui.sliderOffset->setValue(params["offset"].getVal<double>());
        }

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::on_sliderExposure_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,value / 1000.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::on_sliderGain_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sliderGain->setValue(value);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::on_sliderOffset_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sliderOffset->setValue(value);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
