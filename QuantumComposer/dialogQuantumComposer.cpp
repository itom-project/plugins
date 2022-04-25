/* ********************************************************************
    Plugin "QuantumComposer" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "dialogQuantumComposer.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogQuantumComposer::DialogQuantumComposer(ito::AddInBase* instance) :
    AbstractAddInConfigDialog(instance),
    m_firstRun(true)
{
    ui.setupUi(this);
    
    enableGUI(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogQuantumComposer::~DialogQuantumComposer()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogQuantumComposer::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString(params["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        
        ui.spinBoxTimeout->setValue(params["requestTimeout"].getVal<int>());

        ito::StringMeta* sm = (ito::StringMeta*)(params["mode"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxMode->addItem(sm->getString(x));
        }
        ui.comboBoxMode->setCurrentIndex(ui.comboBoxMode->findText(params["mode"].getVal<char*>()));
        sm->clearItems();

        sm = (ito::StringMeta*)(params["gateMode"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxGateMode->addItem(sm->getString(x));
        }
        ui.comboBoxGateMode->setCurrentIndex(ui.comboBoxGateMode->findText(params["gateMode"].getVal<char*>()));
        sm->clearItems();

        sm = (ito::StringMeta*)(params["gateLogic"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxGateLogic->addItem(sm->getString(x));
        }
        ui.comboBoxGateLogic->setCurrentIndex(
            ui.comboBoxGateLogic->findText(params["gateLogic"].getVal<char*>()));
        sm->clearItems();

        sm = (ito::StringMeta*)(params["triggerMode"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxTriggerMode->addItem(sm->getString(x));
        }
        ui.comboBoxTriggerMode->setCurrentIndex(
            ui.comboBoxTriggerMode->findText(params["triggerMode"].getVal<char*>()));
        sm->clearItems();

        sm = (ito::StringMeta*)(params["triggerEdge"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxTriggerEdge->addItem(sm->getString(x));
        }
        ui.comboBoxTriggerEdge->setCurrentIndex(
            ui.comboBoxTriggerEdge->findText(params["triggerEdge"].getVal<char*>()));
        sm->clearItems();

        sm = (ito::StringMeta*)(params["icLock"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxICLock->addItem(sm->getString(x));
        }
        ui.comboBoxICLock->setCurrentIndex(
            ui.comboBoxICLock->findText(params["icLock"].getVal<char*>()));
        sm->clearItems();

        sm = (ito::StringMeta*)(params["ocLock"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxOCLock->addItem(sm->getString(x));
        }
        ui.comboBoxOCLock->setCurrentIndex(
            ui.comboBoxOCLock->findText(params["ocLock"].getVal<char*>()));
        sm->clearItems();


        ui.spinBoxBurstCounter->setMinimum(params["burstCounter"].getMin());
        ui.spinBoxBurstCounter->setMaximum(params["burstCounter"].getMax());
        ui.spinBoxBurstCounter->setValue(params["burstCounter"].getVal<int>());

        ui.spinBoxPulseCounter->setMinimum(params["pulseCounter"].getMin());
        ui.spinBoxPulseCounter->setMaximum(params["pulseCounter"].getMax());
        ui.spinBoxPulseCounter->setValue(params["pulseCounter"].getVal<int>());

        ui.spinBoxOffCounter->setMinimum(params["offCounter"].getMin());
        ui.spinBoxOffCounter->setMaximum(params["offCounter"].getMax());
        ui.spinBoxOffCounter->setValue(params["offCounter"].getVal<int>());

        ui.doubleSpinBoxGateLevel->setMinimum(params["gateLevel"].getMin());
        ui.doubleSpinBoxGateLevel->setMaximum(params["gateLevel"].getMax());
        ui.doubleSpinBoxGateLevel->setValue(params["gateLevel"].getVal<double>());

        ui.doubleSpinBoxTriggerLevel->setMinimum(params["triggerLevel"].getMin());
        ui.doubleSpinBoxTriggerLevel->setMaximum(params["triggerLevel"].getMax());
        ui.doubleSpinBoxTriggerLevel->setValue(params["triggerLevel"].getVal<double>());

        ui.doubleSpinBoxPeriod->setMinimum(params["period"].getMin());
        ui.doubleSpinBoxPeriod->setMaximum(params["period"].getMax());
        ui.doubleSpinBoxPeriod->setValue(params["period"].getVal<double>());

        ui.spinBoxCounterCounts->setMinimum(params["counterCounts"].getMin());
        ui.spinBoxCounterCounts->setMaximum(params["counterCounts"].getMax());
        ui.spinBoxCounterCounts->setValue(params["counterCounts"].getVal<int>());

        ui.checkBoxCounter->setChecked(params["counterState"].getVal<int>());
                            
        m_firstRun = false;
    }

    enableGUI(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogQuantumComposer::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    if (ui.spinBoxTimeout->isEnabled())
    {
        int bin = ui.spinBoxTimeout->value();
        if (m_currentParameters["requestTimeout"].getVal<int>() != bin)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("requestTimeout", ito::ParamBase::Int, bin)));
        }
    }

    if (ui.comboBoxMode->isEnabled())
    {
        QString mode = ui.comboBoxMode->currentText();
        if (m_currentParameters["mode"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase(
                "mode", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.comboBoxGateMode->isEnabled())
    {
        QString mode = ui.comboBoxGateMode->currentText();
        if (m_currentParameters["gateMode"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("gateMode", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.comboBoxGateLogic->isEnabled())
    {
        QString mode = ui.comboBoxGateLogic->currentText();
        if (m_currentParameters["gateLogic"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("gateLogic", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.comboBoxTriggerMode->isEnabled())
    {
        QString mode = ui.comboBoxTriggerMode->currentText();
        if (m_currentParameters["triggerMode"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("triggerMode", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.comboBoxTriggerEdge->isEnabled())
    {
        QString mode = ui.comboBoxTriggerEdge->currentText();
        if (m_currentParameters["triggerEdge"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("triggerEdge", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.comboBoxICLock->isEnabled())
    {
        QString mode = ui.comboBoxICLock->currentText();
        if (m_currentParameters["icLock"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("icLock", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.comboBoxOCLock->isEnabled())
    {
        QString mode = ui.comboBoxOCLock->currentText();
        if (m_currentParameters["ocLock"].getVal<char*>() != mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("ocLock", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.spinBoxBurstCounter->isEnabled())
    {
        int count = ui.spinBoxBurstCounter->value();
        if (m_currentParameters["burstCounter"].getVal<int>() != count)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("burstCounter", ito::ParamBase::Int, count))); 
        }
    }

    if (ui.spinBoxPulseCounter->isEnabled())
    {
        int count = ui.spinBoxPulseCounter->value();
        if (m_currentParameters["pulseCounter"].getVal<int>() != count)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("pulseCounter", ito::ParamBase::Int, count)));
        }
    }

    if (ui.spinBoxOffCounter->isEnabled())
    {
        int count = ui.spinBoxOffCounter->value();
        if (m_currentParameters["offCounter"].getVal<int>() != count)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("offCounter", ito::ParamBase::Int, count)));
        }
    }

    if (ui.doubleSpinBoxGateLevel->isEnabled())
    {
        double count = ui.doubleSpinBoxGateLevel->value();
        if (m_currentParameters["gateLevel"].getVal<double>() != count)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("gateLevel", ito::ParamBase::Double, count)));
        }
    }

    if (ui.doubleSpinBoxTriggerLevel->isEnabled())
    {
        double count = ui.doubleSpinBoxTriggerLevel->value();
        if (m_currentParameters["triggerLevel"].getVal<double>() != count)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("triggerLevel", ito::ParamBase::Double, count)));
        }
    }

    if (ui.doubleSpinBoxPeriod->isEnabled())
    {
        double count = ui.doubleSpinBoxPeriod->value();
        if (m_currentParameters["period"].getVal<double>() != count)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("period", ito::ParamBase::Double, count)));
        }
    }

    if (ui.checkBoxCounter->isEnabled())
    {
        bool state = ui.checkBoxCounter->isChecked();
        if (bool(m_currentParameters["counterState"].getVal<int>()) != state)
        {
            values.append(QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("counterState", ito::ParamBase::Int, state)));
        }
    }
    
    retValue += setPluginParameters(values, msgLevelWarningAndError);
    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
void DialogQuantumComposer::enableGUI(bool enabled)
{
}
