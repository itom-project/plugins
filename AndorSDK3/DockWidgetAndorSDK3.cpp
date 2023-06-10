/* ********************************************************************
    Plugin "AndorSDK3" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut fuer Technische Optik, Universitaet Stuttgart

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

#include "DockWidgetAndorSDK3.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetSDK3::DockWidgetSDK3(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSDK3::parametersChanged(QMap<QString, ito::Param> params)
{
    int inEditing = m_inEditing;
    m_inEditing = true;

    ui.lblWidth->setText(QString::number(params["sizex"].getVal<int>()));
    ui.lblHeight->setText(QString::number(params["sizey"].getVal<int>()));

    ui.lblGain->setVisible( !(params["gain"].getFlags() & ito::ParamBase::Readonly) );
    ui.sliderGain->setVisible( !(params["gain"].getFlags() & ito::ParamBase::Readonly) );
    ui.sliderExposure->setDisabled( params["integration_time"].getFlags() & ito::ParamBase::Readonly );

    if (m_firstRun)
    {
        if (params.contains("camera_model"))
        {
            ui.lblModel->setText(params["camera_model"].getVal<const char*>());
        }

        if (params.contains("camera_name"))
        {
            ui.lblName->setText(params["camera_name"].getVal<const char*>());
        }

        if (params.contains("serial_number"))
        {
            ui.lblSerial->setText(params["serial_number"].getVal<const char*>());
        }

        if (params.contains("bpp"))
        {
            ui.lblBitDepth->setText(QString("%1 bit").arg(params["bpp"].getVal<int>()));
        }

        m_firstRun = false;
    }

    ParamMapIterator it = params.find("integration_time");
    if (it != params.end())
    {
        ito::DoubleMeta *dm = (ito::DoubleMeta*)(it->getMeta());
        ui.sliderExposure->setMinimum(dm->getMin());
        ui.sliderExposure->setMaximum(dm->getMax());
        if (dm->getStepSize() != 0)
        {
            ui.sliderExposure->setSingleStep(std::max(dm->getStepSize(), 0.00001)); //0.00001 is the minimal step of the spin box
        }
        else
        {
            ui.sliderExposure->setSingleStep((dm->getMax() - dm->getMin()) / 100);
        }
        ui.sliderExposure->setValue(it->getVal<double>());
    }

    if ((it = params.find("gain")) != params.end())
    {
        ui.sliderGain->setValue(it->getVal<double>());
    }

    m_inEditing = inEditing;

    m_currentParams = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSDK3::on_sliderExposure_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSDK3::on_sliderGain_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
