/* ********************************************************************
    Plugin "FireGrabber" for itom software
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

#include "dockWidgetFireGrabber.h"

#include <qmetaobject.h>

//----------------------------------------------------------------------------
DockWidgetFireGrabber::DockWidgetFireGrabber(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
    ui.groupIntegration->setEnabled(false);
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFireGrabber::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        ui.groupIntegration->setEnabled(true);
        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        ParamMapIterator it = params.find("gain");
        if (it != params.end())
        {
            ui.spinBox_gain->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.spinBox_gain->setMinimum(it->getMin() * 100);
            ui.spinBox_gain->setMaximum(it->getMax() * 100);
            ui.spinBox_gain->setValue(it->getVal<double>() * 100);
        }
        else
        {
            ui.spinBox_gain->setVisible(false);
        }

        it = params.find("gamma");
        if (it != params.end())
        {
            ui.checkGamma->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.checkGamma->setChecked(it->getVal<int>() > 0);
        }
        else
        {
            ui.checkGamma->setVisible(false);
        }

        it = params.find("integration_time");
        if (it != params.end())
        {
            ui.doubleSpinBox_integration_time->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.doubleSpinBox_integration_time->setMinimum(it->getMin() * 1000);
            ui.doubleSpinBox_integration_time->setMaximum(it->getMax() * 1000);
            ui.doubleSpinBox_integration_time->setSingleStep((it->getMax() - it->getMin()) * 10.0);
            ui.doubleSpinBox_integration_time->setValue(it->getVal<double>() * 1000);
        }
        else
        {
            ui.doubleSpinBox_integration_time->setVisible(false);
        }

        it = params.find("bpp");
        if (it != params.end())
        {
            ui.spinBpp->setValue(it->getVal<int>());
        }

        it = params.find("sizex");
        if (it != params.end())
        {
            ui.spinWidth->setValue(it->getVal<int>());
        }

        it = params.find("sizey");
        if (it != params.end())
        {
            ui.spinHeight->setValue(it->getVal<int>());
        }


        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

void DockWidgetFireGrabber::on_spinBox_gain_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain", ito::ParamBase::Double, d / 100.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFireGrabber::on_doubleSpinBox_integration_time_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time", ito::ParamBase::Double, d / 1000.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFireGrabber::on_checkGamma_clicked(bool value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gamma", ito::ParamBase::Int, value ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetFireGrabber::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}
