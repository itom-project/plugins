/* ********************************************************************
    Plugin "Standa ST8SMC4USB" for itom software
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

#include "dialogST8SMC4USB.h"
#include "ST8SMC4USB.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogST8SMC4USB::DialogST8SMC4USB(ito::AddInBase *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogST8SMC4USB::~DialogST8SMC4USB()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogST8SMC4USB::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.idLabel->setText(params["device_id"].getVal<char*>());
        ui.numAxisLabel->setText(params["device_port"].getVal<char*>());

        m_firstRun = false;

        ui.cb_microSteps->clear();
        int pow = 0;
        int count = 0;
        for (int i = 0; i <= 8; i ++)
        {
            pow = 1 << i;
            ui.cb_microSteps->addItem(QString::number(pow));
            ui.cb_microSteps->setItemData(count++, pow, Qt::UserRole);
        }

        if (params["unit"].getVal<int>() > 0)
        {
            ui.sb_speed->setSuffix(" mm/s");
            ui.sb_accel->setSuffix(" mm/s²");
            ui.sb_decel->setSuffix(" mm/s²");
        }
        else
        {
            ui.sb_speed->setSuffix(" °/s");
            ui.sb_accel->setSuffix(" °/s²");
            ui.sb_decel->setSuffix(" °/s²");
        }
    }

    QString tmp;
    tmp.setNum(params["units_per_step"].getVal<double>());
    ui.lb_unitPerSteps->setText(tmp);

    if (params["unit"].getVal<double>() == 0)
    {
        tmp = tr("degree");
    }
    else
    {
        tmp = tr("mm");
    }
    ui.lb_unitOfAxis->setText(tmp);

    int microSteps = params["micro_steps"].getVal<int>();
    for (int i = 0; i < ui.cb_microSteps->count(); ++i)
    {
        if (ui.cb_microSteps->itemData(i, Qt::UserRole).toInt() == microSteps)
        {
            ui.cb_microSteps->setCurrentIndex(i);
            break;
        }
    }

    ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["accel"].getMeta());
    ui.sb_accel->setMaximum(dm->getMax());
    ui.sb_accel->setMinimum(dm->getMin());
    ui.sb_accel->setSingleStep(dm->getStepSize());
    ui.sb_accel->setValue(params["accel"].getVal<double>());


    dm = (ito::DoubleMeta*)(params["decel"].getMeta());
    ui.sb_decel->setMaximum(dm->getMax());
    ui.sb_decel->setMinimum(dm->getMin());
    ui.sb_decel->setSingleStep(dm->getStepSize());
    ui.sb_decel->setValue(params["decel"].getVal<double>());


    dm = (ito::DoubleMeta*)(params["speed"].getMeta());
    ui.sb_speed->setMaximum(dm->getMax());
    ui.sb_speed->setMinimum(dm->getMin());
    ui.sb_speed->setSingleStep(dm->getStepSize());
    ui.sb_speed->setValue(params["speed"].getVal<double>());

    ui.checkAsync->setChecked(params["async"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogST8SMC4USB::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    int microsteps = ui.cb_microSteps->currentText().toInt();
    bool microsteps_changed = false;
    if (microsteps != m_currentParameters["micro_steps"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("micro_steps", ito::ParamBase::Int, microsteps)));
        microsteps_changed = true;
    }

    if (microsteps_changed || qAbs(ui.sb_accel->value() - m_currentParameters["accel"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("accel", ito::ParamBase::Double, ui.sb_accel->value())));
    }

    if (microsteps_changed || qAbs(ui.sb_decel->value() - m_currentParameters["decel"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("decel", ito::ParamBase::Double, ui.sb_decel->value())));
    }

    if (microsteps_changed || qAbs(ui.sb_speed->value() - m_currentParameters["speed"].getVal<double>()) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed", ito::ParamBase::Double, ui.sb_speed->value())));
    }

    int async = ui.checkAsync->isChecked() ? 1 : 0;
    if (m_currentParameters["async"].getVal<int>() != async)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, async)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogST8SMC4USB::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogST8SMC4USB::enableDialog(bool enabled)
{
    ui.groupSettings->setEnabled(enabled);
    ui.groupMode->setEnabled(enabled);
}
