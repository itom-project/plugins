/* ********************************************************************
    Plugin "MSMediaFoundation" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2014, Institut für Technische Optik (ITO),
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

#include "dockWidgetMSMediaFoundation.h"

#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetMSMediaFoundation::DockWidgetMSMediaFoundation(ito::AddInDataIO *grabber) :
    m_pMSMediaFoundation(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);

    ui.sW_Brightness->setTracking(false);
    ui.sW_Contrast->setTracking(false);
    ui.sW_Gain->setTracking(false);
    ui.sW_Saturation->setTracking(false);
    ui.sW_Sharpness->setTracking(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::valuesChanged(QMap<QString, ito::Param> params)
{
//qDebug() << "----------------- valuesChanged m_firstRun: " << m_firstRun << "; m_inEditing: " << m_inEditing;
    if (m_firstRun)
    {
        int propCount = 0;
        if (params.contains("brightness"))
        {
            ui.lB_Brightness->setVisible(true);
            ui.cB_Brightness->setVisible(true);
            ui.sW_Brightness->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["brightness"].getMeta());
            ui.sW_Brightness->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Brightness->setVisible(false);
            ui.cB_Brightness->setVisible(false);
            ui.sW_Brightness->setVisible(false);
        }

        if (params.contains("contrast"))
        {
            ui.lB_Contrast->setVisible(true);
            ui.cB_Contrast->setVisible(true);
            ui.sW_Contrast->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["contrast"].getMeta());
            ui.sW_Contrast->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Contrast->setVisible(false);
            ui.cB_Contrast->setVisible(false);
            ui.sW_Contrast->setVisible(false);
        }

        if (params.contains("gain"))
        {
            ui.lB_Gain->setVisible(true);
            ui.cB_Gain->setVisible(true);
            ui.sW_Gain->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.sW_Gain->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Gain->setVisible(false);
            ui.cB_Gain->setVisible(false);
            ui.sW_Gain->setVisible(false);
        }

        if (params.contains("saturation"))
        {
            ui.lB_Saturation->setVisible(true);
            ui.cB_Saturation->setVisible(true);
            ui.sW_Saturation->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["saturation"].getMeta());
            ui.sW_Saturation->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Saturation->setVisible(false);
            ui.cB_Saturation->setVisible(false);
            ui.sW_Saturation->setVisible(false);
        }

        if (params.contains("sharpness"))
        {
            ui.lB_Sharpness->setVisible(true);
            ui.cB_Sharpness->setVisible(true);
            ui.sW_Sharpness->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["sharpness"].getMeta());
            ui.sW_Sharpness->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Sharpness->setVisible(false);
            ui.cB_Sharpness->setVisible(false);
            ui.sW_Sharpness->setVisible(false);
        }

        if (params.contains("sizex"))
        {
            ui.lblWidth->setText(QString("%1").arg(params["sizex"].getVal<int>()));
        }

        if (params.contains("sizey"))
        {
            ui.lblHeight->setText(QString("%1").arg(params["sizey"].getVal<int>()));
        }

        if (params.contains("bpp"))
        {
            ui.lblBitDepth->setText(QString("%1").arg(params["bpp"].getVal<int>()));
        }

        if (propCount == 0)
        {
            ui.groupBox_3->setVisible(false);
            setMinimumHeight(109);
        }
        else
        {
            setMinimumHeight((26 * propCount) + 129);
        }

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        if (params.contains("brightness"))
        {
            ui.cB_Brightness->setChecked(params["brightnessAuto"].getVal<int>());
            ui.sW_Brightness->setValue(params["brightness"].getVal<double>());
        }

        if (params.contains("contrast"))
        {
            ui.cB_Contrast->setChecked(params["contrastAuto"].getVal<int>());
            ui.sW_Contrast->setValue(params["contrast"].getVal<double>());
        }

        if (params.contains("gain"))
        {
            ui.cB_Gain->setChecked(params["gainAuto"].getVal<int>());
            ui.sW_Gain->setValue(params["gain"].getVal<double>());
        }

        if (params.contains("saturation"))
        {
            ui.cB_Saturation->setChecked(params["saturationAuto"].getVal<int>());
            ui.sW_Saturation->setValue(params["saturation"].getVal<double>());
        }

        if (params.contains("sharpness"))
        {
            ui.cB_Sharpness->setChecked(params["sharpnessAuto"].getVal<int>());
            ui.sW_Sharpness->setValue(params["sharpness"].getVal<double>());
        }

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::sendParameters(const int type, const double d)
{
//qDebug() << "----------------- sendParameters type: " << type << "; d: " << d << "; int: " << (int)d;
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    switch(type)
        {
        case 1:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("brightness", ito::ParamBase::Double, d)));
            break;
        case 2:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("contrast", ito::ParamBase::Double, d)));
            break;
        case 3:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, d)));
            break;
        case 4:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("saturation", ito::ParamBase::Double, d)));
            break;
        case 5:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sharpness", ito::ParamBase::Double, d)));
            break;
        case 11:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("brightnessAuto", ito::ParamBase::Int, (int)d)));
            break;
        case 22:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("contrastAuto", ito::ParamBase::Int, (int)d)));
            break;
        case 33:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gainAuto", ito::ParamBase::Int, (int)d)));
            break;
        case 44:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("saturationAuto", ito::ParamBase::Int, (int)d)));
            break;
        case 55:
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sharpnessAuto", ito::ParamBase::Int, (int)d)));
            break;
        }
    
    if (m_pMSMediaFoundation)
    {
        if (values.size() > 0)
        {
            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(m_pMSMediaFoundation, "setParamVector", Q_ARG( const QVector<QSharedPointer<ito::ParamBase> >, values), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

            while(!success)
            {
                if (locker.getSemaphore()->wait(PLUGINWAIT) == true)
                {
                    success = true;
                }
                if (!m_pMSMediaFoundation->isAlive())
                {
                    break;
                }
            }

            if (!success)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("timeout while setting parameters of plugin.").toLatin1().data());
            }
            if (locker.getSemaphore()->returnValue.containsError())
            {
                retValue += ito::RetVal(ito::retError, 0, locker.getSemaphore()->returnValue.errorMessage());
            }
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("plugin instance not defined.").toLatin1().data());
    }

    if (retValue.containsError())
    {
        QMessageBox::information(this, tr("error"), retValue.errorMessage());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::sendParameter(QSharedPointer<ito::ParamBase> &param)
{
    ito::RetVal retValue;

    if (m_pMSMediaFoundation)
    {
        bool success = false;
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pMSMediaFoundation, "setParam", Q_ARG(QSharedPointer<ito::ParamBase>, param), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        while(!success)
        {
            if (locker.getSemaphore()->wait(PLUGINWAIT) == true)
            {
                success = true;
            }
            if (!m_pMSMediaFoundation->isAlive())
            {
                break;
            }
        }

        if (!success)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("timeout while setting parameter of plugin.").toLatin1().data());
        }
        else
        {
            retValue += locker.getSemaphore()->returnValue;
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("plugin instance not defined.").toLatin1().data());
    }

    if (retValue.containsError())
    {
        QMessageBox::information(this, tr("error"), retValue.errorMessage());
    }
    
}

//----------------------------------------------------------------------------------------------------------------------------------
double getStepValue(double value, double stepSize)
{
    int stepCount = (int)((value / stepSize) + .5);
qDebug() << "----------------- getStepValue stepCount: " << stepCount << "; alt: " << value / stepSize;  // getStepValue stepCount:  77.4818 ; alt:  77.4 
    return stepSize * stepCount;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Brightness_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        d = getStepValue(d, ui.sW_Brightness->singleStep());
        ui.sW_Brightness->setValue(d);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("brightness",ito::ParamBase::Double,d));
        sendParameter(p);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Contrast_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        sendParameters(2, getStepValue(d, ui.sW_Contrast->singleStep()));
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Gain_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        sendParameters(3, getStepValue(d, ui.sW_Gain->singleStep()));
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Saturation_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        sendParameters(4, getStepValue(d, ui.sW_Saturation->singleStep()));
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Sharpness_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        sendParameters(5, getStepValue(d, ui.sW_Sharpness->singleStep()));
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Brightness_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Brightness->setEnabled(!checked);
        double v = checked ? 1.0 : 0.0;
        sendParameters(11, v);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Contrast_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Contrast->setEnabled(!checked);
        double v = checked ? 1.0 : 0.0;
        sendParameters(22, v);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Gain_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Gain->setEnabled(!checked);
        double v = checked ? 1.0 : 0.0;
        sendParameters(33, v);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Saturation_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Saturation->setEnabled(!checked);
        double v = checked ? 1.0 : 0.0;
        sendParameters(44, v);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Sharpness_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Sharpness->setEnabled(!checked);
        double v = checked ? 1.0 : 0.0;
        sendParameters(55, v);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::propertiesChanged(QString identifier)
{
    ui.label_Identifier->setText(identifier);
}