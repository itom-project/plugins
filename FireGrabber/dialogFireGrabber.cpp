/* ********************************************************************
    Plugin "FireGrabber" for itom software
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

#include "dialogFireGrabber.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogFireGrabber::DialogFireGrabber(ito::AddInBase *grabber) :
AbstractAddInConfigDialog(grabber),
m_firstRun(true)
{
	ui.setupUi(this);

	//disable dialog, since no parameters are known yet. Parameters will immediately be sent by the slot parametersChanged.
	enableDialog(false);
};


//---------------------------------------------------------------------------------------------------------------------
void DialogFireGrabber::on_buttonBox_clicked(QAbstractButton* btn)
{
	ito::RetVal retValue(ito::retOk);

	QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

	if (role == QDialogButtonBox::RejectRole)
	{
		reject(); //close dialog with reject
	}
	else if (role == QDialogButtonBox::AcceptRole)
	{
		accept(); //AcceptRole
	}
	else
	{
		applyParameters(); //ApplyRole
	}
}

//---------------------------------------------------------------------------------------------------------------------
void DialogFireGrabber::enableDialog(bool enabled)
{
	//e.g.
	ui.groupBoxBinning->setEnabled(enabled);
	ui.groupBoxIntegration->setEnabled(enabled);
	ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogFireGrabber::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
	ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogFireGrabber::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
	ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//------------------------------------------------------------------------------
void DialogFireGrabber::on_btnFullROI_clicked()
{
	if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
	{
		ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
		ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
	}
}


//----------------------------------------------------------------------------------------------------------------------------------
void DialogFireGrabber::parametersChanged(QMap<QString, ito::Param> params)
{
	if (m_firstRun)
	{
		setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
		ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
		ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
		ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());
		m_firstRun = false;

		//now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
		enableDialog(true);
	}

	int *roi = params["roi"].getVal<int*>();
	ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
	ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
	ui.rangeX01->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly));
	ui.rangeY01->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly));
	ui.spinSizeX->setValue(params["sizex"].getVal<int>());
	ui.spinSizeY->setValue(params["sizey"].getVal<int>());

	ParamMapIterator it = params.find("gain");
	if (it != params.end())
	{
		ui.sliderGain->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
		ui.sliderGain->setMinimum(it->getMin() * 100.0);
		ui.sliderGain->setMaximum(it->getMax() * 100.0);
		ui.sliderGain->setValue(it->getVal<double>() * 100.0);
	}
	else
	{
		ui.label_gain->setVisible(false);
		ui.sliderGain->setVisible(false);
	}

	it = params.find("offset");
	if (it != params.end())
	{
		ui.sliderOffset->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
		ui.sliderOffset->setMinimum(it->getMin() * 100.0);
		ui.sliderOffset->setMaximum(it->getMax() * 100.0);
		ui.sliderOffset->setValue(it->getVal<double>() * 100.0);
	}
	else
	{
		ui.label_offset->setVisible(false);
		ui.sliderOffset->setVisible(false);
	}

	it = params.find("integration_time");
	if (it != params.end())
	{
		ui.sliderIntegrationTime->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
		ui.sliderIntegrationTime->setMinimum(it->getMin());
		ui.sliderIntegrationTime->setMaximum(it->getMax());
		ui.sliderIntegrationTime->setValue(it->getVal<double>());
	}
	else
	{
		ui.label_integration_time->setVisible(false);
		ui.sliderIntegrationTime->setVisible(false);
	}

	it = params.find("frame_time");
	if (it != params.end())
	{
		ui.sliderFrameTime->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
		ui.sliderFrameTime->setMinimum(it->getMin());
		ui.sliderFrameTime->setMaximum(it->getMax());
		ui.sliderFrameTime->setValue(it->getVal<double>());
	}
	else
	{
		ui.label_frame_time->setVisible(false);
		ui.sliderFrameTime->setVisible(false);
	}

	it = params.find("bpp");
	if (it != params.end())
	{
		ui.combo_bpp->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
		switch (it->getVal<int>())
		{
		case 8:
			ui.combo_bpp->setCurrentIndex(0);
			break;
		case 16:
			ui.combo_bpp->setCurrentIndex(1);
			break;
		default:
			ui.combo_bpp->setEnabled(false);
			break;
		}
	}



	//save the currently set parameters to m_currentParameters
	m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogFireGrabber::applyParameters()
{
	ito::RetVal retValue(ito::retOk);
	QVector<QSharedPointer<ito::ParamBase> > values;
	bool success = false;

	if (ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
	{
		int x0, x1, y0, y1;
		ui.rangeX01->values(x0, x1);
		ui.rangeY01->values(y0, y1);
		int roi[] = { 0, 0, 0, 0 };
		memcpy(roi, m_currentParameters["roi"].getVal<int*>(), 4 * sizeof(int));

		if (roi[0] != x0 || roi[1] != y0 || roi[2] != (x1 - x0 + 1) || roi[3] != (y1 - y0 + 1))
		{
			roi[0] = x0;
			roi[1] = y0;
			roi[2] = x1 - x0 + 1;
			roi[3] = y1 - y0 + 1;
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, roi)));
		}
	}


	if (ui.combo_bpp->isEnabled())
	{
		QVariant qvar = ui.combo_bpp->currentIndex();
		int bppNew = -1;
		switch (qvar.toInt())
		{
		case 0:
			bppNew = 8;
			break;
		case 1:
			bppNew = 16;
			break;
		}
		if ((bppNew > 0) && (m_currentParameters["bpp"].getVal<double>() != bppNew))
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bppNew)));
		}
	}

	if (dblEq(m_currentParameters["offset"].getVal<double>(), ui.sliderOffset->value() / 100.0) == 0)
	{
		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, ui.sliderOffset->value() / 100.0)));
	}

	if (dblEq(m_currentParameters["gain"].getVal<double>(), ui.sliderGain->value() / 100.0) == 0)
	{
		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, ui.sliderGain->value() / 100.0)));
	}


	if (ui.sliderIntegrationTime->isEnabled() && dblEq(m_currentParameters["integration_time"].getVal<double>(), ui.sliderIntegrationTime->value()) == 0)
	{
		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, ui.sliderIntegrationTime->value())));
	}

	if (ui.sliderFrameTime->isEnabled() && dblEq(m_currentParameters["frame_time"].getVal<double>(), ui.sliderFrameTime->value()) == 0)
	{
		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_time", ito::ParamBase::Double, ui.sliderFrameTime->value())));
	}

	retValue += setPluginParameters(values, msgLevelWarningAndError);

	return retValue;
}
