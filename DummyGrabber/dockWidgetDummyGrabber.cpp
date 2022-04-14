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

#include "dockWidgetDummyGrabber.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetDummyGrabber::DockWidgetDummyGrabber(ito::AddInDataIO *dataIO) :
    AbstractAddInDockWidget(dataIO),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetDummyGrabber::parametersChanged(QMap<QString, ito::Param> params)
 {
    ui.spinBpp->setValue(params["bpp"].getVal<int>());
    ui.spinWidth->setValue(params["sizex"].getVal<int>());
    ui.spinHeight->setValue(params["sizey"].getVal<int>());

    if (m_firstRun)
    {
        ui.spinBox_gain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );
        ui.horizontalSlider_gain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );

        ui.spinBox_offset->setDisabled( params["offset"].getFlags() & ito::ParamBase::Readonly );
        ui.horizontalSlider_offset->setDisabled( params["offset"].getFlags() & ito::ParamBase::Readonly );

        ui.doubleSpinBox_integration_time->setDisabled( params["integration_time"].getFlags() & ito::ParamBase::Readonly );

        m_firstRun = false;
    }
    
    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.doubleSpinBox_integration_time->setMaximum(params["integration_time"].getMax() *1000.0);
        ui.doubleSpinBox_integration_time->setMinimum(params["integration_time"].getMin() *1000.0);
        ui.doubleSpinBox_integration_time->setSingleStep(1);
        ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>() *1000.0);

        ui.spinBox_offset->setValue((int)(params["offset"].getVal<double>()*100.0+0.5));
        ui.spinBox_gain->setValue((int)(params["gain"].getVal<double>()*100.0+0.5));

        m_inEditing = false;
    }
 }

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyGrabber::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyGrabber::on_spinBox_gain_valueChanged(int d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,d/100.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyGrabber::on_spinBox_offset_valueChanged(int d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,d/100.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyGrabber::on_doubleSpinBox_integration_time_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,d/1000.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}



