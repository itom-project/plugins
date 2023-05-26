/* ********************************************************************
    Plugin "VRMagic" for itom software
    URL: http://www.uni-stuttgart.de/ito
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

#include "dialogVRMagic.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogVRMagic::DialogVRMagic(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
	m_inEditing(false),
    timing_mode_changed(false)
{
    ui.setupUi(this);
    ui.tabWidget->setCurrentIndex(0);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//---------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
		enableDialog(true);
		setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

		ui.label_sensor->setText(params["vendor_id"].getVal<char*>());
		ui.label_serial->setText(params["serial_number"].getVal<char*>());
		/*
		int binning = params["binning"].getVal<int>();
		ito::IntMeta *binningMeta = static_cast<ito::IntMeta*>(params["binning"].getMeta());
		ui.combo_bin->clear();
		for (int i = binningMeta->getMin(); i <= binningMeta->getMax(); i += 101)
		{
            if (i != 303)
            {
			    ui.combo_bin->addItem(QString("%1x%1").arg(i % 100), i);
            }
		}
		ui.combo_bin->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));
		*/
		int bpp = params["bpp"].getVal<int>();
		ito::IntMeta *bppMeta = static_cast<ito::IntMeta*>(params["bpp"].getMeta());
		ui.combo_bpp->clear();
		for (int i = bppMeta->getMin(); i <= bppMeta->getMax(); i+=2)
		{
			ui.combo_bpp->addItem(QString("%1").arg(i), i);
		}

        for (int i = 0; i < ui.combo_bpp->count(); ++i)
        {
            ui.combo_bpp->setCurrentIndex(i);
            if (ui.combo_bpp->currentText().toInt() == bpp)
            {
                ui.combo_bpp->setCurrentIndex(i);
                break;
            }
        }
		/*
		ui.btnFullROI->setEnabled(true);

        ui.checkTimingMode->setEnabled(!(params["timing_mode"].getFlags() & ito::ParamBase::Readonly));
        ui.comboTriggerMode->setEnabled(!(params["trigger_mode"].getFlags() & ito::ParamBase::Readonly));
        ui.comboTriggerSelector->setEnabled(!(params["trigger_selector"].getFlags() & ito::ParamBase::Readonly));
        int numGPIs = params["gpi_mode"].getLen();
        ui.comboGPI1->setEnabled(numGPIs >= 1);
        ui.comboGPI2->setEnabled(numGPIs >= 2);
        ui.comboGPI3->setEnabled(numGPIs >= 3);
        ui.comboGPI4->setEnabled(numGPIs >= 4);
		*/
        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
		/*
        ui.combo_bpp->setEnabled(!(params["bpp"].getFlags() & ito::ParamBase::Readonly));
		*/
		ito::IntMeta *brightness = static_cast<ito::IntMeta*>(params["brightness"].getMeta());
		ui.sliderWidget_brightness->setMinimum(brightness->getMin());
		ui.sliderWidget_brightness->setMaximum(brightness->getMax());
		ui.sliderWidget_brightness->setValue(params["brightness"].getVal<int>());
		ui.sliderWidget_brightness->setEnabled(!(params["brightness"].getFlags() & ito::ParamBase::Readonly));

		ito::IntMeta *contrast = static_cast<ito::IntMeta*>(params["contrast"].getMeta());
		ui.sliderWidget_contrast->setMinimum(contrast->getMin());
		ui.sliderWidget_contrast->setMaximum(contrast->getMax());
		ui.sliderWidget_contrast->setValue((params["contrast"].getVal<int>()));
		ui.sliderWidget_contrast->setEnabled(!(params["contrast"].getFlags() & ito::ParamBase::Readonly));
		/*
        int timing_mode = params["timing_mode"].getVal<int>();
        ui.checkTimingMode->setChecked(timing_mode == XI_ACQ_TIMING_MODE_FRAME_RATE);
        ui.sliderWidget_framerate->setEnabled(timing_mode == XI_ACQ_TIMING_MODE_FRAME_RATE);

		ito::DoubleMeta *framerate = static_cast<ito::DoubleMeta*>(params["framerate"].getMeta());
		ui.sliderWidget_framerate->setMinimum(framerate->getMin());
		ui.sliderWidget_framerate->setMaximum(framerate->getMax());
		ui.sliderWidget_framerate->setValue(params["framerate"].getVal<double>());
		ui.sliderWidget_framerate->setEnabled(!(params["framerate"].getFlags() & ito::ParamBase::Readonly));

        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());
        ui.rangeX->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly) && params["sizex"].getMax() > 1);
        ui.rangeY->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly) && params["sizey"].getMax() > 1);

		int *roi = params["roi"].getVal<int*>();
		ui.rangeX->setValues(roi[0], roi[0] + roi[2] - 1);
		ui.rangeY->setValues(roi[1], roi[1] + roi[3] - 1);

		ui.spinSizeX->setValue(params["sizex"].getVal<int>());
		ui.spinSizeY->setValue(params["sizey"].getVal<int>());

		int bin = params["binning"].getVal<int>();
		for (int i = 0; i < ui.combo_bin->count(); ++i)
		{
			if (ui.combo_bin->itemData(i, Qt::UserRole).toInt() == bin)
			{
				ui.combo_bin->setCurrentIndex(i);
				break;
			}
		}

		int bpp = params["bpp"].getVal<int>();
		for (int i = 0; i < ui.combo_bpp->count(); ++i)
		{
			if (ui.combo_bpp->itemData(i, 32).toInt() == bpp)
			{
				ui.combo_bpp->setCurrentIndex(i);
				break;
			}
		}

        const int *gpi_mode = params["gpi_mode"].getVal<int*>();
        switch (std::min(4,params["gpi_mode"].getLen()))
        {
        case 4:
            ui.comboGPI4->setCurrentIndex(gpi_mode[3]);
        case 3:
            ui.comboGPI3->setCurrentIndex(gpi_mode[2]);
        case 2:
            ui.comboGPI2->setCurrentIndex(gpi_mode[1]);
        case 1:
            ui.comboGPI1->setCurrentIndex(gpi_mode[0]);
        }

        ui.comboTriggerSelector->setCurrentIndex(params["trigger_selector"].getVal<int>());
        ui.comboTriggerMode->setCurrentIndex(params["trigger_mode"].getVal<int>());

        if (params["frame_burst_count"].getMax() > 1 && params["trigger_mode"].getVal<int>() > XI_TRG_OFF && params["trigger_selector"].getVal<int>() == XI_TRG_SEL_FRAME_BURST_START)
        {
            ui.sliderFrameBurst->setEnabled(true);
            ui.sliderFrameBurst->setMaximum(params["frame_burst_count"].getMax());
            ui.sliderFrameBurst->setValue(params["frame_burst_count"].getVal<int>());
        }
        else
        {
            ui.sliderFrameBurst->setEnabled(false);
            ui.sliderFrameBurst->setMaximum(params["frame_burst_count"].getMax());
            ui.sliderFrameBurst->setValue(1);
        }
		*/
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogVRMagic::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

 //   //only send parameters which are changed
	//if (ui.rangeX->isEnabled() || ui.rangeY->isEnabled())
 //   {
 //       int x0, x1, y0, y1;
 //       ui.rangeX->values(x0,x1);
 //       ui.rangeY->values(y0,y1);
 //       int roi[] = {0,0,0,0};
 //       memcpy(roi, m_currentParameters["roi"].getVal<int*>(), 4*sizeof(int));

 //       if (roi[0] != x0 || roi[1] != y0 || roi[2] != (x1-x0+1) || roi[3] != (y1-y0+1))
 //       {
 //           roi[0] = x0;
 //           roi[1] = y0;
 //           roi[2] = x1-x0+1;
 //           roi[3] = y1-y0+1;
 //           values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, roi)));
 //       }
 //   }
 //
	//if (ui.combo_bin->isEnabled())
	//{
	//	int bin = ui.combo_bin->itemData(ui.combo_bin->currentIndex()).toInt();
	//	if (m_currentParameters["binning"].getVal<int>() != bin)
	//	{
	//		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, bin)));
	//	}
	//}

	//if (ui.combo_bpp->isEnabled())
	//{
	//	int bpp = ui.combo_bpp->itemData(ui.combo_bpp->currentIndex()).toInt();
	//	if (m_currentParameters["bpp"].getVal<int>() != bpp)
	//	{
	//		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bpp)));
	//	}
	//}

 //   if (ui.comboTriggerMode->isEnabled())
	//{
	//	int trigger_mode = ui.comboTriggerMode->currentIndex();
	//	if (m_currentParameters["trigger_mode"].getVal<int>() != trigger_mode)
	//	{
	//		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::Int, trigger_mode)));
	//	}
	//}

 //   if (ui.comboTriggerSelector->isEnabled())
	//{
	//	int trigger_selector = ui.comboTriggerSelector->currentIndex();
	//	if (m_currentParameters["trigger_selector"].getVal<int>() != trigger_selector)
	//	{
	//		values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_selector", ito::ParamBase::Int, trigger_selector)));
	//	}
	//}

	//int frame_burst_count = (int)ui.sliderFrameBurst->value();
	//if (m_currentParameters["frame_burst_count"].getVal<int>() != frame_burst_count)
	//{
	//	values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_burst_count", ito::ParamBase::Int, frame_burst_count)));
	//}

 //   if (ui.comboGPI1->isEnabled())
 //   {
 //       int gpi[] = {XI_GPI_OFF,XI_GPI_OFF,XI_GPI_OFF,XI_GPI_OFF};
 //       if (ui.comboGPI1->isEnabled())
 //       {
 //           gpi[0] = ui.comboGPI1->currentIndex();
 //       }
 //       if (ui.comboGPI2->isEnabled())
 //       {
 //           gpi[1] = ui.comboGPI2->currentIndex();
 //       }
 //       if (ui.comboGPI3->isEnabled())
 //       {
 //           gpi[2] = ui.comboGPI3->currentIndex();
 //       }
 //       if (ui.comboGPI4->isEnabled())
 //       {
 //           gpi[3] = ui.comboGPI4->currentIndex();
 //       }

 //       if (memcmp(m_currentParameters["gpi_mode"].getVal<int*>(), gpi, std::min(4, m_currentParameters["gpi_mode"].getLen()) * sizeof(int)) != 0)
 //       {
 //           values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gpi_mode", ito::ParamBase::IntArray, m_currentParameters["gpi_mode"].getLen(), gpi)));
 //       }
 //   }

 //   if (ui.checkTimingMode->isEnabled())
 //   {
 //       int timing_mode = (ui.checkTimingMode->isChecked() ? XI_ACQ_TIMING_MODE_FRAME_RATE : XI_ACQ_TIMING_MODE_FREE_RUN);
 //       if (timing_mode != m_currentParameters["timing_mode"].getVal<int>())
 //       {
 //           values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timing_mode", ito::ParamBase::Int, timing_mode)));
 //       }

	//    if (timing_mode == XI_ACQ_TIMING_MODE_FRAME_RATE && ui.sliderWidget_framerate->isEnabled())
	//    {
	//	    double framerate = ui.sliderWidget_framerate->value();
	//	    if (qAbs(m_currentParameters["framerate"].getVal<double>() - framerate) > std::numeric_limits<double>::epsilon())
	//	    {
	//		    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("framerate", ito::ParamBase::Double, framerate)));
	//	    }
	//    }
 //   }

	if (ui.sliderWidget_brightness->isEnabled())
    {
        int brightness = ui.sliderWidget_brightness->value();
        if (qAbs(m_currentParameters["brightness"].getVal<int>() - brightness) > std::numeric_limits<int>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("brightness", ito::ParamBase::Int, brightness)));
        }
    }

	if (ui.sliderWidget_contrast->isEnabled())
	{
		int contrast = ui.sliderWidget_contrast->value();
        if (qAbs(m_currentParameters["contrast"].getVal<int>() - contrast) > std::numeric_limits<int>::epsilon())
		{
			values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("contrast", ito::ParamBase::Int, contrast)));
		}
	}

    //check further parameters...

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//------------------------------------------------------------------------------
void DialogVRMagic::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY->setValues(0, m_currentParameters["sizey"].getMax());
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        if (timing_mode_changed)
        {
            QMessageBox::information(this, tr("timing mode already updated"), tr("You changed the parameter timing_mode. The changed value is automatically applied"));
        }

        reject(); //close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else
    {
        timing_mode_changed = false;
        applyParameters(); //ApplyRole
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::on_rangeX_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::on_rangeY_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::on_checkTimingMode_clicked(bool checked)
{
    /*QVector<QSharedPointer<ito::ParamBase> > values;

    int timing_mode = (checked ? XI_ACQ_TIMING_MODE_FRAME_RATE : XI_ACQ_TIMING_MODE_FREE_RUN);
    if (timing_mode != m_currentParameters["timing_mode"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timing_mode", ito::ParamBase::Int, timing_mode)));
    }

    timing_mode_changed = true;

    setPluginParameters(values, msgLevelWarningAndError);*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::on_comboTriggerSelector_currentIndexChanged(int index)
{
    /*if (m_currentParameters["frame_burst_count"].getMax() > 1 && ui.comboTriggerMode->currentIndex() > XI_TRG_OFF && index == XI_TRG_SEL_FRAME_BURST_START)
    {
        ui.sliderFrameBurst->setEnabled(true);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
    }
    else
    {
        ui.sliderFrameBurst->setEnabled(false);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
        ui.sliderFrameBurst->setValue(1);
    }*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVRMagic::on_comboTriggerMode_currentIndexChanged(int index)
{
   /* if (m_currentParameters["frame_burst_count"].getMax() > 1 && index > XI_TRG_OFF && ui.comboTriggerSelector->currentIndex() == XI_TRG_SEL_FRAME_BURST_START)
    {
        ui.sliderFrameBurst->setEnabled(true);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
    }
    else
    {
        ui.sliderFrameBurst->setEnabled(false);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
        ui.sliderFrameBurst->setValue(1);
    }*/
}
