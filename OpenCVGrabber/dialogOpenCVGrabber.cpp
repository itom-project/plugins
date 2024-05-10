/* ********************************************************************
    Plugin "OpenCV-Grabber" for itom software
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

#include "dialogOpenCVGrabber.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

#include "common/addInInterface.h"

#include "OpenCVGrabber.h"
#include <qmessagebox.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogOpenCVGrabber::DialogOpenCVGrabber(ito::AddInBase *grabber, bool colorCam, bool hasNativeSettingsDialog) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
    m_colorCam(colorCam),
    m_pGrabber(grabber)
{
    ui.setupUi(this);

    ui.comboColorMode->clear();
    if (colorCam)
    {
        ui.comboColorMode->addItem("auto");
        ui.comboColorMode->addItem("color");
        ui.comboColorMode->addItem("red");
        ui.comboColorMode->addItem("green");
        ui.comboColorMode->addItem("blue");
        ui.comboColorMode->addItem("gray");
    }
    else
    {
        ui.comboColorMode->addItem("auto");
        ui.comboColorMode->addItem("gray");
    }

    ui.btnShowNativeSettings->setVisible(hasNativeSettingsDialog);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());

        m_firstRun = false;
    }

    int *roi = params["roi"].getVal<int*>();
    qDebug() << roi[0] << roi[1] << roi[2] << roi[3];
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());



    ui.spinBpp->setValue( params["bpp"].getVal<int>() );

    if (params.contains("color_mode"))
    {
        QString mode = params["color_mode"].getVal<char*>();
        mode = mode.toLower();
        int idx;
        if (m_colorCam)
        {
            if (mode == "auto")
            {
                idx = 0;
            }
            else if (mode == "color")
            {
                idx = 1;
            }
            else if (mode == "red")
            {
                idx = 2;
            }
            else if (mode == "green")
            {
                idx = 3;
            }
            else if (mode == "blue")
            {
                idx = 4;
            }
            else
            {
                idx = 5;
            }
        }
        else
        {
            if (mode == "auto")
            {
                idx = 0;
            }
            else if (mode == "gray")
            {
                idx = 1;
            }
            else
            {
                idx = 0;
            }
        }

        ui.comboColorMode->setCurrentIndex(idx);
    }

    /*ui.doubleSpinBox_integration_time->setMinimum(params["integration_time"].getMin()*1000);
    ui.doubleSpinBox_integration_time->setMaximum(params["integration_time"].getMax()*1000);
    ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>()*1000);
    ui.doubleSpinBox_integration_time->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));*/

    ui.sliderDumpGrab->setMaximum(params["dump_grabs"].getMax());
    ui.sliderDumpGrab->setEnabled(!(params["dump_grabs"].getFlags() & ito::ParamBase::Readonly));
    ui.sliderDumpGrab->setValue(params["dump_grabs"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogOpenCVGrabber::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    if(ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
    {
        int x0, x1, y0, y1;
        ui.rangeX01->values(x0,x1);
        ui.rangeY01->values(y0,y1);
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

    //if(ui.doubleSpinBox_integration_time->isEnabled())
    //{
    //    double dval = ui.doubleSpinBox_integration_time->value()/1000.0;
    //    if(qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= 1e-6) //the smallest range is 1musec, given by the number of decimals of the spin box. //std::numeric_limits<double>::epsilon())
    //    {
    //        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
    //    }
    //}

    if(ui.sliderDumpGrab->isEnabled())
    {
        int val = qRound(ui.sliderDumpGrab->value());
        if(m_currentParameters["dump_grabs"].getVal<int>() != val)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("dump_grabs", ito::ParamBase::Int, val)));
        }
    }

    if(ui.spinBpp->isEnabled())
    {
        int bpp = ui.spinBpp->value();

        if(m_currentParameters["bpp"].getVal<int>() !=  bpp)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bpp)));
        }
    }

    if (ui.comboColorMode->isEnabled())
    {
        QString text = ui.comboColorMode->currentText();

        if (text.compare(m_currentParameters["color_mode"].getVal<char*>()) != 0)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("color_mode", ito::ParamBase::String, text.toLatin1().data())));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogOpenCVGrabber::enableDialog(bool enabled)
{
    ui.groupBuffer->setEnabled(enabled);
    ui.groupColorMode->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_btnShowNativeSettings_clicked()
{
    if (m_pGrabber)
    {
        if (!((OpenCVGrabber*)m_pGrabber)->showNativeSettingsDialog())
        {
            QMessageBox::critical(this, "Native settings dialog", "Native settings dialog not available for this type of device");
        }
    }
}
