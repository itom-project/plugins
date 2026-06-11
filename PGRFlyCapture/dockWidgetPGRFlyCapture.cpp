/* ********************************************************************
    Plugin "PGRFlyCapture" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2017, twip optical solutions GmbH
    Copyright (C) 2017, Institut für Technische Optik, Universität Stuttgart

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

#include "dockWidgetPGRFlyCapture.h"


#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetPGRFlyCapture::DockWidgetPGRFlyCapture(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPGRFlyCapture::parametersChanged(QMap<QString, ito::Param> params)
{

    ui.spinBpp->setValue(params["bpp"].getVal<int>());
    ui.spinWidth->setValue(params["sizex"].getVal<int>());
    ui.spinHeight->setValue(params["sizey"].getVal<int>());

    if (m_firstRun)
    {
        ui.spinGain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );
        ui.sliderGain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );

        ui.spinOffset->setDisabled( params["offset"].getFlags() & ito::ParamBase::Readonly );
        ui.sliderOffset->setDisabled( params["offset"].getFlags() & ito::ParamBase::Readonly );

        ui.spinIntegrationTime->setDisabled( params["integration_time"].getFlags() & ito::ParamBase::Readonly );

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.sliderGain->setValue((int)(params["gain"].getVal<double>()*100.0+0.5));
        //ui.spinGain->setValue(ui.sliderGain->value());

        ui.sliderOffset->setValue((int)(params["offset"].getVal<double>()*100.0+0.5));
        //ui.spinOffset->setValue(ui.sliderOffset->value());

        ui.spinIntegrationTime->setMaximum(params["integration_time"].getMax() *1000.0);
        ui.spinIntegrationTime->setMinimum(params["integration_time"].getMin() *1000.0);
        ui.spinIntegrationTime->setValue(params["integration_time"].getVal<double>()*1000.0);
        ui.spinIntegrationTime->setDisabled(params["integration_time"].getFlags() & ito::ParamBase::Readonly);

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPGRFlyCapture::on_spinOffset_valueChanged(int value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sliderOffset->setValue(value);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,(double)value / 100.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPGRFlyCapture::on_spinGain_valueChanged(int value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sliderGain->setValue(value);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,(double)value / 100.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPGRFlyCapture::on_spinIntegrationTime_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,d/1000.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPGRFlyCapture::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}
