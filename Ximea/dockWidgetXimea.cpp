/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2018, Institut für Technische Optik, Universität Stuttgart

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

#include "dockWidgetXimea.h"
#include "common/addInInterface.h"
#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetXimea::DockWidgetXimea(int uniqueID, ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
     ui.setupUi(this);
     identifierChanged(QString::number(uniqueID));
     enableWidget(true);
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetXimea::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.sliderWidget_gain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );
    ui.sliderWidget_gain->setVisible( !(params["gain"].getFlags() & ito::ParamBase::Readonly ));
    ui.sliderWidget_integrationtime->setDisabled( params["integration_time"].getFlags() & ito::ParamBase::Readonly );
    ui.sliderWidget_integrationtime->setVisible( !(params["integration_time"].getFlags() & ito::ParamBase::Readonly ));
    if (m_firstRun)
    {
        ui.label_sensor->setText(params["sensor_type"].getVal<char*>());
        ui.label_serial->setText(params["serial_number"].getVal<char*>());
        ui.label_width->setText(QString::number(params["sizex"].getVal<int>()));
        ui.label_height->setText(QString::number(params["sizey"].getVal<int>()));

        if (params["bpp"].getVal<int>() == 32)
        {
            ui.label_bits->setText("32bit, rgba");
        }
        else
        {
            ui.label_bits->setText(QString("%1, gray").arg(params["bpp"].getVal<int>()));
        }
        //use params (identical to m_params of the plugin)
        //and initialize all widgets (e.g. min, max values, labels, enable some,...)

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        if (params.contains("gain") & !(params["gain"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.sliderWidget_gain->setMinimum(dm->getMin() * 100);
            ui.sliderWidget_gain->setMaximum(dm->getMax() * 100);
            ui.sliderWidget_gain->setValue(params["gain"].getVal<double>() * 100);
        }

        if (params.contains("integration_time") & !(params["integration_time"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["integration_time"].getMeta());
            ui.sliderWidget_integrationtime->setMinimum(dm->getMin()*1000);
            ui.sliderWidget_integrationtime->setMaximum(dm->getMax()*1000);
            ui.sliderWidget_integrationtime->setValue(params["integration_time"].getVal<double>()*1000);
        }
    }
    m_inEditing = false;

    m_currentParams = params;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetXimea::on_sliderWidget_offset_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetXimea::on_sliderWidget_gain_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,value / 100.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetXimea::on_sliderWidget_integrationtime_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        value = value/1000;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetXimea::enableWidget(bool enabled)
{
    ui.sliderWidget_gain->setEnabled(enabled);
    ui.sliderWidget_integrationtime->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetXimea::identifierChanged(const QString &identifier)
{
    ui.label_ID->setText(identifier);
}
