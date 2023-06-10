/* ********************************************************************
Plugin "NITWidySWIR" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "dockWidgetNITWidySWIR.h"

//----------------------------------------------------------------------------------------------------------------------------------
dockWidgetNITWidySWIR::dockWidgetNITWidySWIR(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dockWidgetNITWidySWIR::parametersChanged(QMap<QString, ito::Param> params)
{
	if (m_firstRun)
	{
		//first time call
		//get all given parameters and adjust all widgets according to them (min, max, stepSize, values...)

		ui.sliderWidget_offset->setDisabled(params["offset"].getFlags() & ito::ParamBase::Readonly);
		ui.sliderWidget_offset->setVisible(!(params["offset"].getFlags() & ito::ParamBase::Readonly));

		ui.sliderWidget_gain->setDisabled(params["gain"].getFlags() & ito::ParamBase::Readonly);
		ui.sliderWidget_gain->setVisible(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

		ui.sliderWidget_integrationTime->setDisabled(params["gain"].getFlags() & ito::ParamBase::Readonly);
		ui.sliderWidget_integrationTime->setVisible(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

		ui.label_modelId->setText(params["model_id"].getVal<char*>());
		ui.label_serial->setText(QString::number(params["serial_number"].getVal<int>()));
		ui.label_imageWidth->setText(QString::number(params["sizex"].getVal<int>()));
		ui.label_imageHeight->setText(QString::number(params["sizey"].getVal<int>()));
		ui.label_imageBPP->setText(QString::number(params["bpp"].getVal<int>()));
		ui.label_firmware->setText(QString::number(params["firmware_version"].getVal<int>()));

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        //check the value of all given parameters and adjust your widgets according to them (value only should be enough)
		if (params.contains("bpp") & !(params["bpp"].getFlags() & ito::ParamBase::Readonly))
		{
			ui.label_imageBPP->setText(QString::number(params["bpp"].getVal<int>()));
		}

		if (params.contains("roi") & !(params["roi"].getFlags() & ito::ParamBase::Readonly))
		{
			ui.label_imageWidth->setText(QString::number(params["sizex"].getVal<int>()));
			ui.label_imageHeight->setText(QString::number(params["sizey"].getVal<int>()));
		}

		if (params.contains("gain") & !(params["gain"].getFlags() & ito::ParamBase::Readonly))
		{
			ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["gain"].getMeta());
			ui.sliderWidget_gain->setMinimum(dm->getMin());
			ui.sliderWidget_gain->setMaximum(dm->getMax());
			ui.sliderWidget_gain->setSingleStep(dm->getStepSize());
			ui.sliderWidget_gain->setValue(params["gain"].getVal<double>());
		}

		if (params.contains("offset") & !(params["offset"].getFlags() & ito::ParamBase::Readonly))
		{
			ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["offset"].getMeta());
			ui.sliderWidget_offset->setMinimum(dm->getMin());
			ui.sliderWidget_offset->setMaximum(dm->getMax());
			ui.sliderWidget_offset->setSingleStep(dm->getStepSize());
			ui.sliderWidget_offset->setValue(params["offset"].getVal<double>());
		}

		if (params.contains("integration_time") & !(params["integration_time"].getFlags() & ito::ParamBase::Readonly))
		{
			ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["integration_time"].getMeta());
			ui.sliderWidget_integrationTime->setMinimum(dm->getMin());
			ui.sliderWidget_integrationTime->setMaximum(dm->getMax());
			ui.sliderWidget_integrationTime->setSingleStep(dm->getStepSize());
			ui.sliderWidget_integrationTime->setValue(params["integration_time"].getVal<double>());
		}

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void dockWidgetNITWidySWIR::identifierChanged(const QString &identifier)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void dockWidgetNITWidySWIR::on_sliderWidget_offset_valueChanged(double value)
{
	m_inEditing = true;

	QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset", ito::ParamBase::Double, value));
	setPluginParameter(p, msgLevelWarningAndError);

	m_inEditing = false;

}

//----------------------------------------------------------------------------------------------------------------------------------
void dockWidgetNITWidySWIR::on_sliderWidget_gain_valueChanged(double value)
{
	m_inEditing = true;

	QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain", ito::ParamBase::Double, value));
	setPluginParameter(p, msgLevelWarningAndError);

	m_inEditing = false;

}

//----------------------------------------------------------------------------------------------------------------------------------
void dockWidgetNITWidySWIR::on_sliderWidget_integrationTime_valueChanged(double value)
{
	m_inEditing = true;

	QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time", ito::ParamBase::Double, value));
	setPluginParameter(p, msgLevelWarningAndError);

	m_inEditing = false;

}
