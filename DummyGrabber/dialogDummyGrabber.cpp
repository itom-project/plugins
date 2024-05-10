/* ********************************************************************
    Plugin "DummyGrabber" for itom software
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

#include "dialogDummyGrabber.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogDummyGrabber::DialogDummyGrabber(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyGrabber::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ito::IntMeta *bppMeta = static_cast<ito::IntMeta*>(params["bpp"].getMeta());
        ui.combo_bpp->clear();
        int count = 0;
        for (int i = bppMeta->getMin(); i <= bppMeta->getMax(); i += bppMeta->getStepSize())
        {
            ui.combo_bpp->addItem(QString::number(i));
            ui.combo_bpp->setItemData(count++, i, Qt::UserRole);
        }

        ui.comboBinningX->clear();
        ui.comboBinningX->addItem(QString::number(1), 1);
        ui.comboBinningX->addItem(QString::number(2), 2);
        ui.comboBinningX->addItem(QString::number(4), 4);

        ui.comboBinningY->clear();
        ui.comboBinningY->addItem(QString::number(1), 1);
        ui.comboBinningY->addItem(QString::number(2), 2);
        ui.comboBinningY->addItem(QString::number(4), 4);

        ui.doubleSpinBox_integration_time->setDisabled( params["integration_time"].getFlags() & ito::ParamBase::Readonly );
        ui.doubleSpinBox_frame_time->setDisabled( params["frame_time"].getFlags() & ito::ParamBase::Readonly );

        m_firstRun = false;
    }

    ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
    ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
    ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());

    int *roi = params["roi"].getVal<int*>();
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled((!(params["roi"].getFlags() & ito::ParamBase::Readonly)) && params["sizey"].getMax() > 1);

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    double dval = params["gain"].getVal<double>();
    ui.sliderGain->setValue(dval*100.0);
    ui.sliderGain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));
    ui.spinBox_gain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

    dval = params["offset"].getVal<double>();
    ui.sliderOffset->setValue(dval*100.0);
    ui.sliderOffset->setEnabled(!(params["offset"].getFlags() & ito::ParamBase::Readonly));
    ui.spinBox_offset->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

    ui.combo_bpp->setEnabled(!(params["bpp"].getFlags() & ito::ParamBase::Readonly));

    for (int i = 0; i < ui.combo_bpp->count(); ++i)
    {
        if (ui.combo_bpp->itemData(i, Qt::UserRole).toInt() == params["bpp"].getVal<int>())
        {
            ui.combo_bpp->setCurrentIndex(i);
            break;
        }
    }

    int ival = params["binning"].getVal<int>();
    int ivalY = ival % 100;
    int ivalX = (ival - ivalY) / 100;
    ui.comboBinningX->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));
    ui.comboBinningY->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));

    int idx = ui.comboBinningX->findData(ivalX, Qt::UserRole);
    if (idx >= 0)
    {
        ui.comboBinningX->setCurrentIndex(idx);
    }

    idx = ui.comboBinningY->findData(ivalY, Qt::UserRole);
    if (idx >= 0)
    {
        ui.comboBinningY->setCurrentIndex(idx);
    }

    ui.doubleSpinBox_integration_time->setMaximum(params["integration_time"].getMax() *1000.0);
    ui.doubleSpinBox_integration_time->setMinimum(params["integration_time"].getMin() *1000.0);
    ui.doubleSpinBox_integration_time->setSingleStep(1.0);
    ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>() *1000.0);

    ui.doubleSpinBox_frame_time->setMaximum(params["frame_time"].getMax() *1000.0);
    ui.doubleSpinBox_frame_time->setMinimum(params["frame_time"].getMin() *1000.0);
    ui.doubleSpinBox_frame_time->setSingleStep(1.0);
    ui.doubleSpinBox_frame_time->setValue(params["frame_time"].getVal<double>() *1000.0);


    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogDummyGrabber::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    if (ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
    {
        int x0, x1, y0, y1;
        ui.rangeX01->values(x0,x1);
        if (ui.rangeY01->isEnabled())
        {
            ui.rangeY01->values(y0,y1);
        }
        else
        {
            y0 = 0;
            y1 = 0;
        }

        int roi[] = {0,0,0,0};
        memcpy(roi, m_currentParameters["roi"].getVal<int*>(), 4*sizeof(int));

        if (roi[0] != x0 || roi[1] != y0 || roi[2] != (x1-x0+1) || roi[3] != (y1-y0+1))
        {
            roi[0] = x0;
            roi[1] = y0;
            roi[2] = x1-x0+1;
            roi[3] = y1-y0+1;
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, roi)));
        }
    }

    if (ui.comboBinningX->isEnabled() && ui.comboBinningY->isEnabled())
    {
        int bin = ui.comboBinningX->currentText().toInt() * 100 + ui.comboBinningY->currentText().toInt();
        if (m_currentParameters["binning"].getVal<int>() != bin)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, bin)));
        }
    }

    if (ui.sliderGain->isEnabled())
    {
        double dval = ui.sliderGain->value()/100.0;
        if (qAbs(m_currentParameters["gain"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.sliderOffset->isEnabled())
    {
        double dval = ui.sliderOffset->value()/100.0;
        if (qAbs(m_currentParameters["offset"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.doubleSpinBox_integration_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_integration_time->value();
        if (qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= 0.00001) //the smallest range is 1musec, given by the number of decimals of the spin box. //std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.doubleSpinBox_frame_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_frame_time->value();
        if (qAbs(m_currentParameters["frame_time"].getVal<double>() - dval) >= 0.00001) //the smallest range is 1musec, given by the number of decimals of the spin box. //std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_time", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.combo_bpp->isEnabled())
    {
        int bpp = ui.combo_bpp->itemData(ui.combo_bpp->currentIndex()).toInt();

        if (m_currentParameters["bpp"].getVal<int>() !=  bpp)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bpp)));
        }
    }



    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogDummyGrabber::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogDummyGrabber::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyGrabber::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyGrabber::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogDummyGrabber::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}
