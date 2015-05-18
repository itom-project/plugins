/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
	Copyright (C) 2013, Institut für Technische Optik, Universität Stuttgart

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

DockWidgetXimea::DockWidgetXimea(int uniqueID, ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),  
	m_inEditing(false),  
	m_firstRun(true)
{
     ui.setupUi(this); 

     identifierChanged(QString::number(uniqueID));

     enableWidget(true);
 }


void DockWidgetXimea::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.sliderWidget_gain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );
    ui.sliderWidget_gain->setVisible( !(params["gain"].getFlags() & ito::ParamBase::Readonly ));
    ui.sliderWidget_integrationtime->setDisabled( params["integration_time"].getFlags() & ito::ParamBase::Readonly );
    ui.sliderWidget_integrationtime->setVisible( !(params["integration_time"].getFlags() & ito::ParamBase::Readonly ));

    if (m_firstRun)
    {
        //use params (identical to m_params of the plugin)
        //and initialize all widgets (e.g. min, max values, labels, enable some,...)

        
        if (!(params["gain"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::DoubleMeta *gain = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.sliderWidget_gain->setMinimum(gain->getMin());
            ui.sliderWidget_gain->setMaximum(gain->getMax());
            if (gain->getStepSize() != 0)
            {
                ui.sliderWidget_gain->setSingleStep(std::max(gain->getStepSize(), 0.00001)); //0.00001 is the minimal step of the spin box
            }
            else
            {
                ui.sliderWidget_gain->setSingleStep((gain->getMax() - gain->getMin()) / 100);
            }
            ui.sliderWidget_gain->setValue(params["gain"].getVal<double>());
        }
        else
        {
            ui.sliderWidget_gain->setValue( 0 );
        }
        
        if (!(params["integration_time"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::DoubleMeta *integration_time = (ito::DoubleMeta*)(params["integration_time"].getMeta());
            ui.sliderWidget_integrationtime->setMinimum(integration_time->getMin()*1000);
            ui.sliderWidget_integrationtime->setMaximum(integration_time->getMax()*1000);
            if (integration_time->getStepSize() != 0)
            {
                ui.sliderWidget_integrationtime->setSingleStep(std::max(integration_time->getStepSize()*1000, 0.00001)); //0.00001 is the minimal step of the spin box
            }
            else
            {
                ui.sliderWidget_integrationtime->setSingleStep((integration_time->getMax() - integration_time->getMin()) / 100 *1000);
            }
            ui.sliderWidget_integrationtime->setValue(params["integration_time"].getVal<double>()*1000);
        }
        else
        {
            ui.sliderWidget_integrationtime->setValue( 0 );
        }
        

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        if (params.contains("gain") & !(params["gain"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.sliderWidget_gain->setMinimum(dm->getMin());
            ui.sliderWidget_gain->setMaximum(dm->getMax());
            if (dm->getStepSize() != 0)
            {
                ui.sliderWidget_gain->setSingleStep(std::max(dm->getStepSize(), 0.00001)); //0.00001 is the minimal step of the spin box
            }
            else
            {
                ui.sliderWidget_gain->setSingleStep((dm->getMax() - dm->getMin()) / 100);
            }
            ui.sliderWidget_gain->setValue(params["gain"].getVal<double>());
        }

        if (params.contains("integration_time") & !(params["integration_time"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["integration_time"].getMeta());
            ui.sliderWidget_integrationtime->setMinimum(dm->getMin()*1000);
            ui.sliderWidget_integrationtime->setMaximum(dm->getMax()*1000);
            if (dm->getStepSize() != 0)
            {
                ui.sliderWidget_integrationtime->setSingleStep(std::max(dm->getStepSize()*1000, 0.00001)); //0.00001 is the minimal step of the spin box
            }
            else
            {
                ui.sliderWidget_integrationtime->setSingleStep((dm->getMax() - dm->getMin()) / 100 *1000);
            }
            ui.sliderWidget_integrationtime->setValue(params["integration_time"].getVal<double>()*1000);
        }
    }    
    m_inEditing = false;

    m_currentParams = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetXimea::on_sliderWidget_gain_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
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
    ui.lblID->setText(identifier);
}