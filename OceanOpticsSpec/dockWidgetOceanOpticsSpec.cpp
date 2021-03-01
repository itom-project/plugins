/* ********************************************************************
    Plugin "OceanOpticsSpec" for itom software
    URL: http://www.bitbucket.org/itom/plugins
    Copyright (C) 2016, Institut fuer Technische Optik, Universitaet Stuttgart

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

#include "dockWidgetOceanOpticsSpec.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetOceanOpticsSpec::DockWidgetOceanOpticsSpec(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetOceanOpticsSpec::parametersChanged(QMap<QString, ito::Param> params)
 {
    if (m_firstRun)
    {
		m_inEditing = true;

        ui.rangeWidget_ROI->setMaximum(params["sizex"].getVal<int>());

        ui.doubleSpinBox_integration_time->setMaximum(params["integration_time"].getMax() *1000.0);
        ui.doubleSpinBox_integration_time->setMinimum(params["integration_time"].getMin() *1000.0);
        ui.doubleSpinBox_integration_time->setSingleStep(1);
        ui.spinBox_average->setMaximum(params["average"].getMax());
        ui.spinBox_average->setMinimum(params["average"].getMin()); 
        ui.spinBox_average->setSingleStep(1);

		/*ui.comboDarkCorrection->clear();
		ui.comboDarkCorrection->addItem("No");
		ui.comboDarkCorrection->addItem("Static correction");
		ui.comboDarkCorrection->addItem("Dynamic correction");
		ui.comboDarkCorrection->setToolTip( \
"Some detectors have dark pixels, that can be used for a dark detection. If enabled, the output \n\
dataObject will always be float32. Static (1) subtracts the mean value of all dark pixels from all values. \n\
Dynamic (2) is only available for some devices (see if dyn. dark correction is enabled in the software \n\
AvaSpec) and subtracts different mean values for odd and even pixels.");*/

        m_firstRun = false;
		m_inEditing = false;
    }
    
    if (!m_inEditing)
    {
        m_inEditing = true;
        
        ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>() *1000.0);

        int *roi = params["roi"].getVal<int*>();
        ui.rangeWidget_ROI->setMaximumValue(roi[2]);

        ui.spinBox_average->setValue(params["average"].getVal<int>());
		//ui.comboDarkCorrection->setCurrentIndex(params["dark_correction"].getVal<int>());
		//ui.comboDarkCorrection->setDisabled(params["dark_correction"].getFlags() & ito::ParamBase::Readonly);

        m_inEditing = false;
    }
    m_currentParams = params; // move into ifs?
 }

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOceanOpticsSpec::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOceanOpticsSpec::on_doubleSpinBox_integration_time_valueChanged(double d)
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


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOceanOpticsSpec::on_rangeWidget_ROI_maximumValueChanged(int value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        int roi[] = {0,0,0,0};                    // search for nicer way?
        memcpy(roi, m_currentParams["roi"].getVal<int*>(), 4*sizeof(int));
        roi[2] = value-roi[0];            //+1??
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("roi",ito::ParamBase::IntArray,4,roi));
        setPluginParameter(p, msgLevelWarningAndError);
        
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOceanOpticsSpec::on_rangeWidget_ROI_minimumValueChanged(int value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        int roi[] = {0,0,0,0};                    // search for nicer way?
        memcpy(roi, m_currentParams["roi"].getVal<int*>(), 4*sizeof(int));
        roi[0] = value;
        int curMaxVal = ui.rangeWidget_ROI->maximumValue();
        roi[2] = curMaxVal-roi[0];            //+1??
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("roi",ito::ParamBase::IntArray,4,roi));
        setPluginParameter(p, msgLevelWarningAndError);
        
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOceanOpticsSpec::on_spinBox_average_valueChanged(int value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("average",ito::ParamBase::Int,value));
        setPluginParameter(p, msgLevelWarningAndError);
        
        m_inEditing = false;
    }
}

/*
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOceanOpticsSpec::on_comboDarkCorrection_currentIndexChanged(int d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("dark_correction",ito::ParamBase::Int,d));
        setPluginParameter(p, msgLevelWarningAndError);
        
        m_inEditing = false;
    }
}*/