/* ********************************************************************
    Plugin "OphirPowermeter" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut für Technische Optik (ITO),
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

#include "dockWidgetOphirPowermeter.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetOphirPowermeter::DockWidgetOphirPowermeter(int uniqueID, ito::AddInDataIO *adda) :
    AbstractAddInDockWidget(adda),
    m_inEditing(false),
    m_plugin(adda),
    m_firstRun(true),
    m_timerIsRunning(false)
{
     ui.setupUi(this);

     identifierChanged(QString::number(uniqueID));

     enableWidget(true);
}

DockWidgetOphirPowermeter::~DockWidgetOphirPowermeter()
{
    killTimer(m_timerId);
}

 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetOphirPowermeter::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        m_inEditing = true;
        ui.lblComPort->setText(QString::number(params["comPort"].getVal<int>()));
        ui.lblSerialNo->setText(params["serialNumber"].getVal<char*>());
        ui.lblROM->setText(params["ROMVersion"].getVal<char*>());
        ui.lblDeviceType->setText(params["deviceType"].getVal<char*>());
        ui.lblHead->setText(params["headType"].getVal<char*>());
        ui.lblHeadSerialNumber->setText(params["headSerialNumber"].getVal<char*>());
        ui.lblCalibrationDueDate->setText(params["calibrationDueDate"].getVal<char*>());

        ito::IntMeta *intmeta = static_cast<ito::IntMeta*>(params["range"].getMeta());
        ui.comboBoxRange->clear();
        int count = 0;
        for (int i = intmeta->getMin(); i <= intmeta->getMax(); i += intmeta->getStepSize())
        {
            ui.comboBoxRange->addItem(QString::number(i));
        }
        int index = ui.comboBoxRange->findText(QString::number(params["range"].getVal<int>()));

        ito::StringMeta* sm = (ito::StringMeta*)(params["measurementType"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboBoxMeasurementType->addItem(sm->getString(x));
            int index = ui.comboBoxMeasurementType->findText(params["measurementType"].getVal<char*>());
        }

        char* set = params["wavelengthSet"].getVal<char*>();

        if (QString::fromLatin1(set).compare("CONTINUOUS") == 0)
        {
            ito::IntMeta *im;
            im = static_cast<ito::IntMeta*>(params["wavelength"].getMeta());
            ui.spinBoxWavelength->setSingleStep(im->getStepSize());
            ui.spinBoxWavelength->setMinimum(im->getMin());
            ui.spinBoxWavelength->setMaximum(im->getMax());
        }
        else
        {
            ui.stackedWidgetWavelengthSet->setCurrentIndex(1);

            ito::StringMeta* sm = (ito::StringMeta*)(params["wavelength"].getMeta());
            for (int x = 0; x < sm->getLen(); x++)
            {
                ui.comboBoxWavelength->addItem(sm->getString(x));
            }
            int index = ui.comboBoxWavelength->findText(params["wavelength"].getVal<char*>());
        }

        ui.lblUnit->setText(params["unit"].getVal<char*>());

        m_firstRun = false;
        m_inEditing = false;
        enableWidget(true);
    }

    //__________________________________________________________________________________________________________ Editing
    if (!m_inEditing)
    {
        m_inEditing = true;

        int index = ui.comboBoxRange->findText(QString::number(params["range"].getVal<int>()));
        ui.comboBoxRange->setCurrentIndex(index);

        char* set = params["wavelengthSet"].getVal<char*>();
        ui.lblWavelengthSet->setText(set);

        if (QString::fromLatin1(set).compare("CONTINUOUS") == 0)
        {
            ui.spinBoxWavelength->setValue(params["wavelength"].getVal<int>());
        }
        else
        {
            ui.comboBoxWavelength->setCurrentIndex(ui.comboBoxWavelength->findText(params["wavelength"].getVal<char*>()));
        }

        ui.comboBoxMeasurementType->setCurrentIndex(ui.comboBoxMeasurementType->findText(params["measurementType"].getVal<char*>()));

        m_inEditing = false;
    }

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::enableWidget(bool enabled)
{
    ui.groupBoxSettings->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::on_checkAutograbbing_stateChanged(int val)
{
    manageTimer(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::manageTimer(const bool &visible)
{
    if (visible && ui.checkAutograbbing->checkState() == Qt::Checked)
    {
        m_timerId = startTimer(100);
        m_timerIsRunning = true;
    }
    else if (m_timerIsRunning)
    {
        killTimer(m_timerId);
        m_timerIsRunning = false;
        ui.lblUnit->setText(QString("").toLatin1().data());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::timerEvent(QTimerEvent *event)
{
    ito::RetVal retval(ito::retOk);
    ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
    QSharedPointer<double> value = QSharedPointer<double>(new double);
    QSharedPointer<QString> unit = QSharedPointer<QString>(new QString);

    QMetaObject::invokeMethod(m_plugin, "acquireAutograbbing", Q_ARG(QSharedPointer<double>, value), Q_ARG(QSharedPointer<QString>, unit), Q_ARG(ItomSharedSemaphore*, waitCond));

    if (waitCond)
    {
        observeInvocation(waitCond, msgLevelWarningAndError);
    }


    if (value && unit)
    {
        ui.lcdNumber->display(*value);
        ui.lblUnit->setText(*unit);
    }


    if (waitCond)
    {
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::on_spinBoxWavelength_valueChanged(double val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("wavelength", ito::ParamBase::Double, val));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::on_comboBoxWavelength_currentIndexChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("wavelength", ito::ParamBase::String, ui.comboBoxWavelength->currentText().toLatin1().data()));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::on_comboBoxRange_currentIndexChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("range", ito::ParamBase::Int, ui.comboBoxRange->currentText().toInt()));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirPowermeter::on_comboBoxMeasurementType_currentIndexChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("measurementType", ito::ParamBase::Int, ui.comboBoxWavelength->currentText().toLatin1().data()));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
