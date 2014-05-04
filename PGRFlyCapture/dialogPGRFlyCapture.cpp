/* ********************************************************************
    Plugin "PGRFlyCapture" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2014, twip optical solutions GmbH
	Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

#include "dialogPGRFlyCapture.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogPGRFlyCapture::DialogPGRFlyCapture(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogPGRFlyCapture::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.rangeX01->setMinimum(0);
        ui.rangeX01->setMinimumValue(0);
        ui.rangeX01->setMaximum(params["x1"].getMax());
        ui.rangeX01->setMaximumValue(params["x1"].getMax());

        ui.rangeY01->setMinimum(0);
        ui.rangeY01->setMinimumValue(0);
        ui.rangeY01->setMaximum(params["y1"].getMax());
        ui.rangeY01->setMaximumValue(params["y1"].getMax());

        m_firstRun = false;
    }

    bool updateSizeX = false;
    bool updateSizeY = false;
    
    ui.rangeX01->setValues(params["x0"].getVal<int>(), params["x1"].getVal<int>());
    ui.rangeY01->setValues(params["y0"].getVal<int>(), params["y1"].getVal<int>());
    ui.rangeX01->setEnabled(! (params["x0"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["y0"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    ui.doubleSpinBox_frame_time->setMinimum(params["frame_time"].getMin()*1000);
    ui.doubleSpinBox_frame_time->setMaximum(params["frame_time"].getMax()*1000);
    ui.doubleSpinBox_frame_time->setValue(params["frame_time"].getVal<double>()*1000);
    ui.doubleSpinBox_frame_time->setEnabled(!(params["frame_time"].getFlags() & ito::ParamBase::Readonly));

    ui.doubleSpinBox_integration_time->setMinimum(params["integration_time"].getMin()*1000);
    ui.doubleSpinBox_integration_time->setMaximum(params["integration_time"].getMax()*1000);
    ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>()*1000);
    ui.doubleSpinBox_integration_time->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

    double dval = params["gain"].getVal<double>();
    ui.sliderGain->setValue(dval*100.0);
    ui.sliderGain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

    dval = params["offset"].getVal<double>();
    ui.sliderOffset->setValue(dval*100.0);
    ui.sliderOffset->setEnabled(!(params["offset"].getFlags() & ito::ParamBase::Readonly));             

    int ival = params["binning"].getMin();
    int ivalX = (int)(ival/100);
    int ivalY = ival - ivalX * 100;

    ui.spinBox_binX->setMinimum(ivalX);
    ui.spinBox_binY->setMinimum(ivalY);

    ival = params["binning"].getMax();
    ivalX = (int)(ival/100);
    ivalY = ival - ivalX * 100;

    ui.spinBox_binX->setMaximum(ivalX);
    ui.spinBox_binY->setMaximum(ivalY);

    ival = params["binning"].getVal<int>();
    ivalX = (int)(ival/100);
    ivalY = ival - ivalX * 100;

    ui.spinBox_binX->setValue(ivalX);
    ui.spinBox_binY->setValue(ivalY);

    ui.spinBox_binX->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));
    ui.spinBox_binY->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));

    ival = params["bpp"].getVal<int>();

    ui.combo_bpp->setEnabled(!(params["bpp"].getFlags() & ito::ParamBase::Readonly));

    switch (ival)
    {
        case 8:
            ui.combo_bpp->setCurrentIndex(0);
        break;
        case 10:
            ui.combo_bpp->setCurrentIndex(1);
        break;
        case 12:
            ui.combo_bpp->setCurrentIndex(2);
        break;
        case 14:
            ui.combo_bpp->setCurrentIndex(3);
        break;
        case 16:
            ui.combo_bpp->setCurrentIndex(4);
        break;
        case 24:
            ui.combo_bpp->setCurrentIndex(5);
        break;
        case 30:
            ui.combo_bpp->setCurrentIndex(6);
        break;
        case 32:
            ui.combo_bpp->setCurrentIndex(7);
        break;
        default:
            ui.combo_bpp->setEnabled(false);
        break;
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPGRFlyCapture::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;
    bool binning_changed = false;

    if((ui.spinBox_binX->isEnabled() || ui.spinBox_binY->isEnabled()))
    {
        int ival = ui.spinBox_binX->value() *100 + ui.spinBox_binY->value();
        if((m_currentParameters["binning"].getVal<int>() !=  ival))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, ival)));
            binning_changed = true;
        }
    }

    if(!binning_changed)
    {
        int ivalFirst, ivalLast;
        bool changeX0 = false;
        bool changeX1 = false;
        bool changeY0 = false;
        bool changeY1 = false;

        if(ui.rangeX01->isEnabled())
        {
            int x0;
            int x1;
            ui.rangeX01->values(x0,x1);

            if((m_currentParameters["x0"].getVal<int>() !=  x0))
            {
                values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, x0)));
            }
            if((m_currentParameters["x1"].getVal<int>() !=  x1))
            {
                values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, x1)));
            }
        }

        if(ui.rangeY01->isEnabled())
        {
            int y0;
            int y1;
            ui.rangeY01->values(y0, y1);

            if((m_currentParameters["y0"].getVal<int>() !=  y0))
            {
                values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, y0)));
            }
            if((m_currentParameters["y1"].getVal<int>() !=  y1))
            {
                values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, y1)));
            }
        }

    }

    if(ui.sliderGain->isEnabled())
    {
        double dval = ui.sliderGain->value()/100.0;
        if(qAbs(m_currentParameters["gain"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.sliderOffset->isEnabled())
    {
        double dval = ui.sliderOffset->value()/100.0;
        if(qAbs(m_currentParameters["offset"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.doubleSpinBox_integration_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_integration_time->value()/1000.0;
        if(qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.doubleSpinBox_frame_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_frame_time->value()/1000.0;
        if(qAbs(m_currentParameters["frame_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_time", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.combo_bpp->isEnabled())
    {
        QVariant qvar = ui.combo_bpp->currentIndex();
        int bppNew = -1;
        switch (qvar.toInt())
        {
            case 0:
                bppNew = 8;
            break;
            case 1:
                bppNew = 10;
            break;
            case 2:
                bppNew = 12;
            break;
            case 3:
                bppNew = 14;
            break;
            case 4:
                bppNew = 16;
            break;
            case 5:
                bppNew = 24;
            break;
            case 6:
                bppNew = 30;
            break;
            case 7:
                bppNew = 32;
            break;
        }
        if((bppNew > 0) && (m_currentParameters["bpp"].getVal<int>() !=  bppNew))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bppNew)));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    if (!retValue.containsError())
    {
        enableDialog(true); //enable size group again if disabled due to changes in binning
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPGRFlyCapture::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogPGRFlyCapture::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}



//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the binning is activated, further settings of size will be disabled until apply is pressed
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void DialogPGRFlyCapture::on_spinBox_binX_valueChanged(int value)
{
    ui.groupBoxSize->setTitle("Size (Binning changed, press apply or save)");
    ui.groupBoxSize->setEnabled(false);

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the binning is activated, further settings of size will be disabled until apply is pressed
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void DialogPGRFlyCapture::on_spinBox_binY_valueChanged(int value)
{
    ui.groupBoxSize->setTitle("Size (Binning changed, press apply or save)");
    ui.groupBoxSize->setEnabled(false);
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPGRFlyCapture::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPGRFlyCapture::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}