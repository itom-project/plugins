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

#include "dialogNITWidySWIR.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
//#include <qvector.h>
//#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
dialogNITWidySWIR::dialogNITWidySWIR(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known yet. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void dialogNITWidySWIR::parametersChanged(QMap<QString, ito::Param> params)
{
    //save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

		//set bpp comboBox
		ui.comboBox_bpp->addItem("8");
		ui.comboBox_bpp->addItem("14");

		for (int i = 0; i < ui.comboBox_bpp->count(); ++i)
		{
			if (ui.comboBox_bpp->itemData(i, Qt::UserRole).toInt() == params["bpp"].getVal<int>())
			{
				ui.comboBox_bpp->setCurrentIndex(i);
				break;
			}
		}

		for (int i = 0; i < ui.comboBox_bpp->count(); ++i)
		{
			if (ui.comboBox_bpp->itemText(i).toInt() == params["bpp"].getVal<int>())
			{
				ui.comboBox_bpp->setCurrentIndex(i);
				break;
			}
		}
		ui.comboBox_bpp->setEnabled(!(params["bpp"].getFlags() & ito::ParamBase::Readonly));

		// set shutter comboBox
		ui.comboBox_shutterMode->addItem("Global Shutter");
		ui.comboBox_shutterMode->addItem("Rolling");

		for (int i = 0; i < ui.comboBox_bpp->count(); ++i)
		{
			if (ui.comboBox_shutterMode->itemText(i).toStdString() == params["shutter_mode"].getVal<char*>())
			{
				ui.comboBox_shutterMode->setCurrentIndex(i);
				break;
			}
		}

		ui.comboBox_shutterMode->setEnabled(!(params["shutter_mode"].getFlags() & ito::ParamBase::Readonly));


		// set trigger comboBox
		ui.comboBox_triggerMode->addItem("Disabled");
		ui.comboBox_triggerMode->addItem("Input");
		ui.comboBox_triggerMode->addItem("Output");

		for (int i = 0; i < ui.comboBox_triggerMode->count(); ++i)
		{
			if (ui.comboBox_triggerMode->itemData(i, Qt::UserRole).toString() == params["trigger_mode"].getVal<char*>())
			{
				ui.comboBox_triggerMode->setCurrentIndex(i);
				break;
			}
		}
		ui.comboBox_triggerMode->setEnabled(!(params["trigger_mode"].getFlags() & ito::ParamBase::Readonly));

		// set pixel clock slider
		ui.sliderWidget_pixelClock->setValue(params["pixel_clock"].getVal<double>());
		ui.sliderWidget_pixelClock->setEnabled(!(params["pixel_clock"].getFlags() & ito::ParamBase::Readonly));
		ito::DoubleMeta *pixelClockMeta = static_cast<ito::DoubleMeta*>(params["pixel_clock"].getMeta());
		ui.sliderWidget_pixelClock->setMinimum(pixelClockMeta->getMin());
		ui.sliderWidget_pixelClock->setMaximum(pixelClockMeta->getMax());
		ui.sliderWidget_pixelClock->setSingleStep(pixelClockMeta->getStepSize());

		// set histogram threshold slider
		ui.sliderWidget_histogram->setValue(params["histogram_threshold"].getVal<double>());
		ui.sliderWidget_histogram->setEnabled(!(params["histogram_threshold"].getFlags() & ito::ParamBase::Readonly));
		ito::DoubleMeta *histoMeta = static_cast<ito::DoubleMeta*>(params["histogram_threshold"].getMeta());
		ui.sliderWidget_histogram->setMinimum(histoMeta->getMin());
		ui.sliderWidget_histogram->setMaximum(histoMeta->getMax());
		ui.sliderWidget_histogram->setSingleStep(histoMeta->getStepSize());

		// set histogram threshold slider
		ui.sliderWidget_framerate->setValue(params["framerate"].getVal<int>());
		ui.sliderWidget_framerate->setEnabled(!(params["framerate"].getFlags() & ito::ParamBase::Readonly));
		ito::IntMeta *framerateMeta = static_cast<ito::IntMeta*>(params["framerate"].getMeta());
		ui.sliderWidget_framerate->setMinimum(framerateMeta->getMin());
		ui.sliderWidget_framerate->setMaximum(framerateMeta->getMax());
		ui.sliderWidget_framerate->setSingleStep(framerateMeta->getStepSize());

		// set offset slider
		ui.sliderWidget_offset->setValue(params["offset"].getVal<double>());
		ui.sliderWidget_offset->setEnabled(!(params["offset"].getFlags() & ito::ParamBase::Readonly));
		ito::DoubleMeta *offsetMeta = static_cast<ito::DoubleMeta*>(params["offset"].getMeta());
		ui.sliderWidget_offset->setMinimum(offsetMeta->getMin());
		ui.sliderWidget_offset->setMaximum(offsetMeta->getMax());
		ui.sliderWidget_offset->setSingleStep(offsetMeta->getStepSize());

		// set gain slider
		ui.sliderWidget_gain->setValue(params["gain"].getVal<double>());
		ui.sliderWidget_gain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));
		ito::DoubleMeta *gainMeta = static_cast<ito::DoubleMeta*>(params["gain"].getMeta());
		ui.sliderWidget_gain->setMinimum(gainMeta->getMin());
		ui.sliderWidget_gain->setMaximum(gainMeta->getMax());
		ui.sliderWidget_gain->setSingleStep(gainMeta->getStepSize());

		// set integration time slider
		ui.sliderWidget_integrationTime->setValue(params["integration_time"].getVal<double>());
		ui.sliderWidget_integrationTime->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));
		ito::DoubleMeta *integrationMeta = static_cast<ito::DoubleMeta*>(params["integration_time"].getMeta());
		ui.sliderWidget_integrationTime->setMinimum(integrationMeta->getMin());
		ui.sliderWidget_integrationTime->setMaximum(integrationMeta->getMax());
		ui.sliderWidget_integrationTime->setSingleStep(integrationMeta->getStepSize());


		//set roi slider
		ito::RectMeta *roiMeta = static_cast<ito::RectMeta*>(params["roi"].getMeta());
		int *roi = params["roi"].getVal<int*>();
		ui.rangeWidget_width->setLimitsFromIntervalMeta(roiMeta->getWidthRangeMeta());
		ui.rangeWidget_height->setLimitsFromIntervalMeta(roiMeta->getHeightRangeMeta());
		ui.rangeWidget_width->setValues(roi[0], roi[0] + roi[2] -1);
		ui.rangeWidget_height->setValues(roi[1], roi[1] + roi[3] - 1);


		ui.rangeWidget_width->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly));
		ui.rangeWidget_height->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly));

		m_firstRun = false;

        //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
        enableDialog(true);
    }

    //set the status of all widgets depending on the values of params

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal dialogNITWidySWIR::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

	if (ui.comboBox_bpp->isEnabled())
	{
		int bpp = ui.comboBox_bpp->currentText().toInt();
		if (m_currentParameters["bpp"].getVal<int>() != bpp)
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bpp)));
		}
	}

	if (ui.comboBox_shutterMode->isEnabled())
	{
		if (QString::compare(ui.comboBox_shutterMode->currentText().toLatin1().data(), m_currentParameters["shutter_mode"].getVal<char*>(), Qt::CaseInsensitive) != 0)
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("shutter_mode", ito::ParamBase::String, ui.comboBox_shutterMode->currentText().toLatin1().data())));
		}
	}


	if (ui.comboBox_triggerMode->isEnabled())
	{

		if (QString::compare(ui.comboBox_triggerMode->currentText().toLatin1().data(), m_currentParameters["trigger_mode"].getVal<char*>(), Qt::CaseInsensitive) != 0)
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::String, ui.comboBox_triggerMode->currentText().toLatin1().data())));
		}
	}

	if (ui.sliderWidget_pixelClock->isEnabled())
	{
		double pixelClock = ui.sliderWidget_pixelClock->value();
		if (m_currentParameters["pixel_clock"].getVal<double>() != pixelClock)
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("pixel_clock", ito::ParamBase::Double, pixelClock)));
		}
	}

	if (ui.sliderWidget_histogram->isEnabled())
	{
		double dval = ui.sliderWidget_histogram->value();
		if (qAbs(m_currentParameters["histogram_threshold"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("histogram_threshold", ito::ParamBase::Double, dval)));
		}
	}

	if (ui.sliderWidget_framerate->isEnabled())
	{
		int fps = ui.sliderWidget_framerate->value();
		if (m_currentParameters["framerate"].getVal<int>() != fps)
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("framerate", ito::ParamBase::Int, fps)));
		}
	}

	if (ui.sliderWidget_offset->isEnabled())
	{
		double dval = ui.sliderWidget_offset->value();
		if (qAbs(m_currentParameters["offset"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
		}
	}

	if (ui.sliderWidget_gain->isEnabled())
	{
		double dval = ui.sliderWidget_gain->value();
		if (qAbs(m_currentParameters["gain"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
		}
	}

	if (ui.sliderWidget_integrationTime->isEnabled())
	{
		double dval = ui.sliderWidget_integrationTime->value();
		if (qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
		}
	}

	if (ui.rangeWidget_width->isEnabled() || ui.rangeWidget_height->isEnabled())
	{
		int x0, x1, y0, y1;
		ui.rangeWidget_width->values(x0, x1);
		ui.rangeWidget_height->values(y0, y1);
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

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void dialogNITWidySWIR::on_buttonBox_clicked(QAbstractButton* btn)
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
void dialogNITWidySWIR::enableDialog(bool enabled)
{


}
