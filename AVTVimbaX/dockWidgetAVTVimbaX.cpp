/* ********************************************************************
Plugin "Roughness" for itom software
URL : http ://www.uni-stuttgart.de/ito
Copyright(C) 2016, Institut fuer Technische Optik (ITO),
Universitaet Stuttgart, Germany;
IPROM, TU Braunschweig, Germany

This file is part of a plugin for the measurement software itom.

This itom - plugin is free software; you can redistribute it and / or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or(at
your option) any later version.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dockWidgetAvtVimbaX.h"

#include <qmetaobject.h>


//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetAvtVimbaX::DockWidgetAvtVimbaX(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
    ui.groupAcquisition->setEnabled(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimbaX::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        ui.lblInterface->setText(params["interface"].getVal<char*>());
        ui.groupAcquisition->setEnabled(true);
        m_firstRun = false;
    }

    ParamMapIterator it = params.find("gain");
    if (it != params.end())
    {
        //this is a special case, since the auto-gain checkbox has to trigger the enable property of the gain slider
        ui.sW_Gain->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        it = params.find("gain");
        if (it != params.end())
        {
            ui.sW_Gain->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.sW_Gain->setMinimum(it->getMin());
            ui.sW_Gain->setMaximum(it->getMax());
            ui.sW_Gain->setValue(it->getVal<double>());
        }
        else
        {
            ui.lbl_Gain->setVisible(false);
            ui.sW_Gain->setVisible(false);
        }

        it = params.find("gain_auto");
        if (it != params.end())
        {
            ui.check_GainAuto->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.check_GainAuto->setChecked(it->getVal<int>() > 0);
            ui.sW_Gain->setEnabled(ui.sW_Gain->isEnabled() && (it->getVal<int>() == 0));
        }
        else
        {
            ui.check_GainAuto->setVisible(false);
        }

        it = params.find("offset");
        if (it != params.end())
        {
            ui.sW_Offset->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.sW_Offset->setMinimum(it->getMin());
            ui.sW_Offset->setMaximum(it->getMax());
            ui.sW_Offset->setValue(it->getVal<double>());
        }
        else
        {
            ui.lbl_Offset->setVisible(false);
            ui.sW_Offset->setVisible(false);
        }

        it = params.find("integration_time");
        if (it != params.end())
        {
            ui.sW_IntTime->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.sW_IntTime->setMinimum(it->getMin());
            ui.sW_IntTime->setMaximum(it->getMax());
            ui.sW_IntTime->setValue(it->getVal<double>());
        }
        else
        {
            ui.lbl_IntTime->setVisible(false);
            ui.sW_IntTime->setVisible(false);
        }


        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

void DockWidgetAvtVimbaX::on_sW_Gain_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimbaX::on_sW_IntTime_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimbaX::on_sW_Offset_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimbaX::on_check_GainAuto_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain_auto",ito::ParamBase::Int, checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimbaX::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}
