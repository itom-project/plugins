/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
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

#include "dialogPIPiezoCtrl.h"
#include "PIPiezoCtrl.h"


#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogPIPiezoCtrl::DialogPIPiezoCtrl(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogPIPiezoCtrl::parametersChanged(QMap<QString, ito::Param> params)
{

    QString m_ctrlType = params["ctrlType"].getVal<char*>();

    if (QString::compare(m_ctrlType, "C663") == 0)
    {
        m_scale = 1.0f;
        m_unit = u8" mm";
        ui.dblSpinPosLimitLow->setSuffix(m_unit);
        ui.dblSpinPosLimitHigh->setSuffix(m_unit);
        ui.label_Piezo->setVisible(false);
        ui.lblPiezo->setVisible(false);
    }
    else
    {
        m_scale = 1000.0f;
        m_unit = u8" µm";
        ui.dblSpinPosLimitLow->setSuffix(m_unit);
        ui.dblSpinPosLimitHigh->setSuffix(m_unit);
        ui.label_Velocity->setVisible(false);
        ui.dblSpinBox_Velocity->setVisible(false);

    }

    setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    ui.lblDevice1->setText(params["ctrlType"].getVal<char*>());
    ui.lblDevice2->setText(params["ctrlName"].getVal<char*>());
    ui.lblPiezo->setText(params["piezoName"].getVal<char*>());
    ui.dblSpinBox_Velocity->setValue(params["velocity"].getVal<double>());

    bool hasMode = params["hasLocalRemote"].getVal<int>() > 0;
    ui.comboMode->setVisible(hasMode);
    ui.lblMode->setVisible(hasMode);

    if (params["local"].getVal<int>() > 0)
    {
        ui.comboMode->setCurrentIndex(1);
    }
    else
    {
        ui.comboMode->setCurrentIndex(0);
    }

    ui.checkAsync->setChecked(params["async"].getVal<int>() > 0);
    ui.dblSpinPosLimitHigh->setValue(params["posLimitHigh"].getVal<double>() * m_scale);
    ui.dblSpinPosLimitLow->setValue(params["posLimitLow"].getVal<double>() * m_scale);

    ui.spinDelayOffset->setMinimum(0);
    ui.spinDelayOffset->setMaximum(params["delayOffset"].getMax() * m_scale);
    ui.spinDelayOffset->setValue(params["delayOffset"].getVal<double>() * m_scale);

    ui.spinDelayProp->setMinimum(0);
    ui.spinDelayProp->setMaximum(params["delayProp"].getMax() * m_scale);
    ui.spinDelayProp->setValue(params["delayProp"].getVal<double>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPIPiezoCtrl::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    //only send parameters which are changed

    int i = ui.comboMode->currentIndex() > 0 ? 1 : 0;
    if (m_currentParameters["local"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("local", ito::ParamBase::Int, i)));
    }

    i = ui.checkAsync->isChecked() ? 1 : 0;
    if (m_currentParameters["async"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, i)));
    }

    double v_high = ui.dblSpinPosLimitHigh->value() / m_scale;
    double v_low = ui.dblSpinPosLimitLow->value() / m_scale;
    if (v_high < v_low)
    {
        std::swap(v_high,v_low);
    }
    if (m_currentParameters["posLimitHigh"].getVal<double>() != v_high)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("posLimitHigh", ito::ParamBase::Double, v_high)));
    }

    if (m_currentParameters["posLimitLow"].getVal<double>() != v_low)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("posLimitLow", ito::ParamBase::Double, v_low)));
    }

    double v = ui.spinDelayOffset->value() / m_scale;
    if (m_currentParameters["delayOffset"].getVal<double>() != v)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("delayOffset", ito::ParamBase::Double, v)));
    }

    v = ui.spinDelayProp->value();
    if (m_currentParameters["delayProp"].getVal<double>() != v)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("delayProp", ito::ParamBase::Double, v)));
    }

    v = ui.dblSpinBox_Velocity->value();
    if (m_currentParameters["velocity"].getVal<double>() != v)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("velocity", ito::ParamBase::Double, v)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPIPiezoCtrl::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogPIPiezoCtrl::enableDialog(bool enabled)
{
    ui.group1->setEnabled(enabled);
    ui.group2->setEnabled(enabled);
    ui.group3->setEnabled(enabled);
    ui.group4->setEnabled(enabled);
}
