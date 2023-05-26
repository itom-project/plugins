/* ********************************************************************
    Plugin "FirgelliLAC" for itom software
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

#include "dialogFirgelliLAC.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogFirgelliLAC::DialogFirgelliLAC(ito::AddInBase *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogFirgelliLAC::~DialogFirgelliLAC()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogFirgelliLAC::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.idLabel->setText(params["deviceID"].getVal<char*>());
        ui.numLabel->setText(QString("%1").arg(params["deviceNum"].getVal<int>()));

        m_firstRun = false;
    }

    ui.lb_spoolMax->setText(QString("%1 mm").arg(params["spoolMax"].getVal<double>(), 0, 'f'));

    ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["speed"].getMeta());
    ui.sb_speed->setMaximum(dm->getMax());
    ui.sb_speed->setMinimum(dm->getMin());
    ui.sb_speed->setSingleStep(dm->getStepSize());
    ui.sb_speed->setValue(params["speed"].getVal<double>());

    dm = (ito::DoubleMeta*)(params["accuracy"].getMeta());
    ui.sb_accuracy->setMaximum(dm->getMax());
    ui.sb_accuracy->setMinimum(dm->getMin());
    ui.sb_accuracy->setSingleStep(dm->getStepSize());
    ui.sb_accuracy->setValue(params["accuracy"].getVal<double>());

    ui.checkAsync->setChecked(params["async"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogFirgelliLAC::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    // This is a general option that does not need the config mode
    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, ui.checkAsync->isChecked())));

    double dtemp = ui.sb_speed->value();
    if (m_currentParameters["speed"].getVal<double>() != dtemp)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed", ito::ParamBase::Double, dtemp)));
    }

    dtemp = ui.sb_accuracy->value();
    if (m_currentParameters["accuracy"].getVal<double>() != dtemp)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("accuracy", ito::ParamBase::Double, dtemp)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogFirgelliLAC::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogFirgelliLAC::enableDialog(bool enabled)
{
    ui.groupSettings->setEnabled(enabled);
    ui.groupMode->setEnabled(enabled);
}
