/* ********************************************************************
    Plugin "MSMediaFoundation" for itom software
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

#include "dialogMSMediaFoundation.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogMSMediaFoundation::DialogMSMediaFoundation(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogMSMediaFoundation::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ito::StringMeta* sm = (ito::StringMeta*)(params["colorMode"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboColorMode->addItem(sm->getString(x));
        }

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());
#else
        ito::IntMeta *im;
        im = static_cast<ito::IntMeta*>(params["x0"].getMeta());
        ui.rangeX01->setSingleStep(im->getStepSize());
        ui.rangeX01->setMinimum(0);
        ui.rangeX01->setMinimumValue(0);
        im = static_cast<ito::IntMeta*>(params["x1"].getMeta());
        ui.rangeX01->setMaximum(im->getMax());
        ui.rangeX01->setMaximumValue(im->getMax());

        im = static_cast<ito::IntMeta*>(params["y0"].getMeta());
        ui.rangeY01->setSingleStep(im->getStepSize());
        ui.rangeY01->setMinimum(0);
        ui.rangeY01->setMinimumValue(0);
        im = static_cast<ito::IntMeta*>(params["y1"].getMeta());
        ui.rangeY01->setMaximum(im->getMax());
        ui.rangeY01->setMaximumValue(im->getMax());
#endif

        m_firstRun = false;
    }

    ui.comboColorMode->setCurrentIndex(ui.comboColorMode->findText(params["colorMode"].getVal<char*>()));

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    int *roi = params["roi"].getVal<int*>();
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
#else
    ui.rangeX01->setValues(params["x0"].getVal<int>(), params["x1"].getVal<int>());
    ui.rangeY01->setValues(params["y0"].getVal<int>(), params["y1"].getVal<int>());
    ui.rangeX01->setEnabled(! (params["x0"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["y0"].getFlags() & ito::ParamBase::Readonly));
#endif

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogMSMediaFoundation::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    //only send parameters which are changed

    if (QString::compare(m_currentParameters["colorMode"].getVal<char*>(), ui.comboColorMode->currentText()) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("colorMode", ito::ParamBase::String, ui.comboColorMode->currentText().toLatin1().data())));
    }
    
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    if (ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
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
#else
    if (ui.rangeX01->isEnabled())
    {
        int x0;
        int x1;
        ui.rangeX01->values(x0,x1);

        if ((m_currentParameters["x0"].getVal<int>() !=  x0))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, x0)));
        }
        if ((m_currentParameters["x1"].getVal<int>() !=  x1))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, x1)));
        }
    }

    if (ui.rangeY01->isEnabled())
    {
        int y0;
        int y1;
        ui.rangeY01->values(y0, y1);

        if ((m_currentParameters["y0"].getVal<int>() !=  y0))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, y0)));
        }
        if ((m_currentParameters["y1"].getVal<int>() !=  y1))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, y1)));
        }
    }
#endif

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogMSMediaFoundation::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogMSMediaFoundation::enableDialog(bool enabled)
{
    ui.groupColorMode->setEnabled(enabled);
    ui.groupROI->setEnabled(enabled);
}


//----------------------------------------------------------------------------------------------------------------------------------
void DialogMSMediaFoundation::on_rangeX01_valuesChanged(int minValue, int maxValue)
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
        ui.rangeX01->setValues(min_,max_);
    }
    else
    {
        ui.spinSizeX->setValue(maxValue - minValue + 1);
    }

#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMSMediaFoundation::on_rangeY01_valuesChanged(int minValue, int maxValue)
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
        ui.rangeY01->setValues(min_,max_);
    }
    else
    {
        ui.spinSizeY->setValue(maxValue - minValue + 1);
    }
#endif
}

//------------------------------------------------------------------------------
void DialogMSMediaFoundation::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}