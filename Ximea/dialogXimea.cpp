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

#include "dialogXimea.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
dialogXimea::dialogXimea(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//---------------------------------------------------------------------------------------------------------------------
void dialogXimea::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}
//---------------------------------------------------------------------------------------------------------------------
void dialogXimea::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
		enableDialog(true);
		setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

		ui.label_sensor->setText(params["sensor_type"].getVal<char*>());
		ui.label_serial->setText(QString::number(params["serialNumber"].getVal<int>()));

		ui.spinBox_bin->setMaximum(params["binning"].getMax());
		ui.spinBox_bin->setMinimum(params["binning"].getMin());
		ui.spinBox_bin->setValue(params["binning"].getVal<int>());
		ui.spinBox_bin->setEnabled(true); //TODO

		int bpp = params["bpp"].getVal<int>();
		ito::IntMeta *bppMeta = static_cast<ito::IntMeta*>(params["bpp"].getMeta());
		ui.combo_bpp->clear();
		for(int i = bppMeta->getMin(); i <= bppMeta->getMax(); i+=2)
		{
			ui.combo_bpp->addItem(QString("%1").arg(i), i);
		}

		for (int i = 0; i < ui.combo_bpp->count(); ++i)
		{
			if (ui.combo_bpp->itemData(i, 32).toInt() == bpp)
			{
				ui.combo_bpp->setCurrentIndex(i);
				break;
			}
		}

		ito::DoubleMeta *offset = static_cast<ito::DoubleMeta*>(params["offset"].getMeta());
		ui.sliderWidget_Offset->setMinimum(offset->getMin());
		ui.sliderWidget_Offset->setMaximum(offset->getMax());
		ui.sliderWidget_Offset->setSingleStep(offset->getStepSize());
		ui.sliderWidget_Offset->setValue(params["offset"].getVal<int>());
		ui.sliderWidget_Offset->setEnabled(!(params["offset"].getFlags() & ito::ParamBase::Readonly));

		ito::DoubleMeta *gain = static_cast<ito::DoubleMeta*>(params["gain"].getMeta());
		ui.sliderWidget_Gain->setMinimum(gain->getMin());
		ui.sliderWidget_Gain->setMaximum(gain->getMax());
		ui.sliderWidget_Gain->setSingleStep(gain->getStepSize());
		ui.sliderWidget_Gain->setValue(params["gain"].getVal<int>());
		ui.sliderWidget_Gain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

		ito::DoubleMeta *integrationtime = static_cast<ito::DoubleMeta*>(params["integration_time"].getMeta());
		ui.sliderWidget_integrationtime->setMinimum(integrationtime->getMin());
		ui.sliderWidget_integrationtime->setMaximum(integrationtime->getMax());
		ui.sliderWidget_integrationtime->setSingleStep(integrationtime->getStepSize());
		ui.sliderWidget_integrationtime->setValue(params["integration_time"].getVal<int>());
		ui.sliderWidget_integrationtime->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

		ito::DoubleMeta *framerate = static_cast<ito::DoubleMeta*>(params["framerate"].getMeta());
		ui.sliderWidget_framerate->setMinimum(framerate->getMin());
		ui.sliderWidget_framerate->setMaximum(framerate->getMax());
		ui.sliderWidget_framerate->setSingleStep(framerate->getStepSize());
		ui.sliderWidget_framerate->setValue(params["framerate"].getVal<int>());
		ui.sliderWidget_framerate->setEnabled(!(params["framerate"].getFlags() & ito::ParamBase::Readonly));
		/*
		btnFullROI

		spinSizeX
		spinSizeY
		*/

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());
		
#else
        ito::IntMeta *im;
        im = static_cast<ito::IntMeta*>(params["x0"].getMeta());
        ui.rangeX->setSingleStep(im->getStepSize());
        ui.rangeX->setMinimum(0);
        ui.rangeX->setMinimumValue(0);
        im = static_cast<ito::IntMeta*>(params["x1"].getMeta());
        ui.rangeX->setMaximum(im->getMax());
        ui.rangeX->setMaximumValue(im->getMax());

        im = static_cast<ito::IntMeta*>(params["y0"].getMeta());
        ui.rangeY->setSingleStep(im->getStepSize());
        ui.rangeY->setMinimum(0);
        ui.rangeY->setMinimumValue(0);
        im = static_cast<ito::IntMeta*>(params["y1"].getMeta());
        ui.rangeY->setMaximum(im->getMax());
        ui.rangeY->setMaximumValue(im->getMax());
#endif
        m_firstRun = false;
    }
	/*
	int *roi = params["roi"].getVal<int*>();
	ui.rangeX->setValues(roi[0], roi[0] + roi[2] - 1);
	ui.rangeY->setValues(roi[1], roi[1] + roi[3] - 1);
	ui.rangeX->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
	ui.rangeY->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
	*/
    if (!m_inEditing)
    {
        m_inEditing = true;

        

        m_inEditing = false;
    }

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal dialogXimea::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    //only send parameters which are changed
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300


#else

#endif
   

    //check further parameters...

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}


//------------------------------------------------------------------------------
void dialogXimea::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY->setValues(0, m_currentParameters["sizey"].getMax());
    }
}

//---------------------------------------------------------------------------------------------------------------------
void dialogXimea::on_buttonBox_clicked(QAbstractButton* btn)
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


//----------------------------------------------------------------------------------------------------------------------------------
void dialogXimea::on_rangeX_valuesChanged(int minValue, int maxValue)
{
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    ui.spinSizeX->setValue(maxValue - minValue + 1);
#else
    int min_ = minValue;
    int max_ = maxValue;
    int stepOffset = static_cast<ito::IntMeta*>( m_currentParameters["x0"].getMeta() )->getStepSize();
    int imageOffset = static_cast<ito::IntMeta*>( m_currentParameters["sizex"].getMeta() )->getStepSize();
    int maxWidth = static_cast<ito::IntMeta*>( m_currentParameters["x1"].getMeta() )->getMax() + 1;

    if ((min_ % stepOffset) != 0)
    {
        min_ = stepOffset * qRound((float)min_ / (float)stepOffset);
        if (min_ >= max_)
        {
            min_ = stepOffset * floor((float)min_ / (float)stepOffset);
        }
    }
    min_ = qBound<int>(0, min_, max_);

    if (((max_ - min_ + 1) % imageOffset) != 0)
    {
        max_ = min_ - 1 + imageOffset * qRound((float)(max_ - min_ + 1) / (float)imageOffset);
    }
    
    max_ = qBound<int>(0, max_, maxWidth-1);

    if (min_ != minValue || max_ != maxValue)
    {
        ui.rangeX->setValues(min_,max_);
    }
    else
    {
        ui.spinSizeX->setValue(maxValue - minValue + 1);
    }

#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogXimea::on_rangeY_valuesChanged(int minValue, int maxValue)
{
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    ui.spinSizeY->setValue(maxValue - minValue + 1);
#else
    int min_ = minValue;
    int max_ = maxValue;
    int stepOffset = static_cast<ito::IntMeta*>( m_currentParameters["y0"].getMeta() )->getStepSize();
    int imageOffset = static_cast<ito::IntMeta*>( m_currentParameters["sizey"].getMeta() )->getStepSize();
    int maxHeight = static_cast<ito::IntMeta*>( m_currentParameters["y1"].getMeta() )->getMax() + 1;

    if ((min_ % stepOffset) != 0)
    {
        min_ = stepOffset * qRound((float)min_ / (float)stepOffset);
        if (min_ >= max_)
        {
            min_ = stepOffset * floor((float)min_ / (float)stepOffset);
        }
    }
    min_ = qBound<int>(0, min_, max_);

    if (((max_ - min_ + 1) % imageOffset) != 0)
    {
        max_ = min_ - 1 + imageOffset * qRound((float)(max_ - min_ + 1) / (float)imageOffset);
    }
    
    max_ = qBound<int>(0, max_, maxHeight - 1);

    if (min_ != minValue || max_ != maxValue)
    {
        ui.rangeY->setValues(min_,max_);
    }
    else
    {
        ui.spinSizeY->setValue(maxValue - minValue + 1);
    }
#endif
}





