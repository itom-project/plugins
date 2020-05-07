/* ********************************************************************
    Plugin "IDSuEye" for itom software
    URL: http://www.bitbucket.org/itom/plugins
    Copyright (C) 2014, Pulsar Photonics GmbH, Aachen
    Copyright (C) 2014, Institut fuer Technische Optik, Universitaet Stuttgart

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


#include "dialogNI-DAQmx.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include <qmath.h>
#include <qtreewidget.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogNiDAQmx::DialogNiDAQmx(ito::AddInBase *adda) :
    AbstractAddInConfigDialog(adda),
    m_firstRun(true)
{
    ui.setupUi(this);

    enableDialog(false);

    // Task Mode
    ui.comboTaskMode->addItem(tr("finite"), "finite");
    ui.comboTaskMode->addItem(tr("continuous"), "continuous");
    ui.comboTaskMode->addItem(tr("onDemand"), "onDemand");
};

//----------------------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        // Tab General, Group General
        ui.lblName->setText(params["taskName"].getVal<const char*>());
        ui.lblType->setText(params["taskType"].getVal<const char*>());

        // Tab General, Group Acquisition / Data Write
        ui.doubleSpinReadTimeout->setEnabled(ui.lblName->text().endsWith("Input"));
        ui.lblReadTimeout->setEnabled(ui.lblName->text().endsWith("Input"));
        ui.checkSetValWaitForFinish->setEnabled(ui.lblName->text().endsWith("Output"));

        // Tab General, Start Trigger
        ito::StringMeta *sm = params["startTriggerMode"].getMetaT<ito::StringMeta>();
        for (int i = 0; i < sm->getLen(); ++i)
        {
            ui.comboStartTriggerMode->addItem(sm->getString(i));
        }
        

        QStringList availChannels = QString(params["supportedChannels"].getVal<const char*>()).split(",");
        availChannels.sort(Qt::CaseInsensitive);

        QList<QTreeWidgetItem*> items;
        QTreeWidgetItem* currentDeviceItem = NULL;
        QString currentDevice;
        QString device;
        QString port;
        QTreeWidgetItem* child = NULL;

        foreach(const QString &channel, availChannels)
        {
            if (channel != "")
            {
                device = channel.split("/")[0];
                port = channel.mid(device.size() + 1);

                if (device != currentDevice)
                {
                    currentDeviceItem = new QTreeWidgetItem();
                    currentDeviceItem->setText(0, device);
                    items.append(currentDeviceItem);
                    currentDeviceItem->setFlags(Qt::ItemIsEnabled);
                    currentDevice = device;
                }

                child = new QTreeWidgetItem();
                m_channelItems[channel] = child;
                currentDeviceItem->addChild(child);
                child->setText(0, port);
                child->setData(0, Qt::UserRole, channel);
                child->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable);
            }
        }

        ui.treeChannels->addTopLevelItems(items);

        QStringList availTerminals = QString(params["availableTerminals"].getVal<const char*>()).split(",");

        ui.comboSampleClockSource->addItem("OnboardClock");
        ui.comboSampleClockSource->addItems(availTerminals);

        ui.comboStartTriggerSource->addItem("OnboardClock");
        ui.comboStartTriggerSource->addItems(availTerminals);

        m_firstRun = false;
    }

    // Tab General, Group General
    QByteArray taskMode = params["taskMode"].getVal<const char*>();

    for (int i = 0; i < ui.comboTaskMode->count(); ++i)
    {
        if (ui.comboTaskMode->itemData(i, Qt::UserRole).toByteArray() == taskMode)
        {
            ui.comboTaskMode->setCurrentIndex(i);
            break;
        }
    }

    // Tab General, Group Acquisition / Data Write
    ui.doubleSpinSamplingRate->setValue(params["samplingRate"].getVal<double>());
    ui.doubleSpinReadTimeout->setValue(params["readTimeout"].getVal<double>());
    ui.checkSetValWaitForFinish->setChecked(params["setValWaitForFinish"].getVal<int>() > 0);
    ui.spinSamplesPerChannel->setValue(params["samplesPerChannel"].getVal<int>());
    ui.spinInputBufferSize->setValue(params["inputBufferSize"].getVal<int>());

    // Tab General, Sample Clock
    ui.comboSampleClockSource->setCurrentText(params["sampleClockSource"].getVal<const char*>());
    ui.checkSampleClockRisingEdge->setChecked(params["sampleClockRisingEdge"].getVal<int>() > 0);

    // Tab General, Start Trigger
    ui.comboStartTriggerSource->setCurrentText(params["startTriggerSource"].getVal<const char*>());
    ui.checkStartTriggerRisingEdge->setChecked(params["startTriggerRisingEdge"].getVal<int>() > 0);
    ui.comboStartTriggerMode->setCurrentText(params["startTriggerMode"].getVal<const char*>());

    ui.doubleSpinStartTriggerLevel->setEnabled(ui.comboStartTriggerMode->currentText() == "analogEdge");
    ui.lblStartTriggerLevel->setEnabled(ui.comboStartTriggerMode->currentText() == "analogEdge");
    ui.doubleSpinStartTriggerLevel->setValue(params["startTriggerLevel"].getVal<double>());
    
    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogNiDAQmx::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase> > values;
    int boolVal;
    QByteArray byteArrayVal;

    // Tab General, Group General
    if (ui.comboTaskMode->currentData(Qt::UserRole).toByteArray() 
        != m_currentParameters["taskMode"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("taskMode", ito::ParamBase::String, 
                ui.comboTaskMode->currentData(Qt::UserRole).toByteArray().data())
            );
    }

    // Tab General, Group Acquisition / Data Write
    if (std::abs(ui.doubleSpinSamplingRate->value()
        - m_currentParameters["samplingRate"].getVal<double>()) 
        > std::numeric_limits<double>::epsilon())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("samplingRate", ito::ParamBase::Double,
                ui.doubleSpinSamplingRate->value())
            );
    }

    if (std::abs(ui.doubleSpinReadTimeout->value()
        - m_currentParameters["readTimeout"].getVal<double>())
        > std::numeric_limits<double>::epsilon())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("readTimeout", ito::ParamBase::Double,
                ui.doubleSpinReadTimeout->value())
            );
    }

    boolVal = ui.checkSetValWaitForFinish->isChecked() ? 1 : 0;
    if (boolVal != m_currentParameters["setValWaitForFinish"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("setValWaitForFinish", ito::ParamBase::Int,
                boolVal)
            );
    }

    if (ui.spinSamplesPerChannel->value()
        != m_currentParameters["samplesPerChannel"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("samplesPerChannel", ito::ParamBase::Int,
                ui.spinSamplesPerChannel->value())
            );
    }

    if (ui.spinInputBufferSize->value()
        != m_currentParameters["inputBufferSize"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("inputBufferSize", ito::ParamBase::Int,
                ui.spinInputBufferSize->value())
            );
    }

    // Tab General, Sample Clock
    boolVal = ui.checkSampleClockRisingEdge->isChecked() ? 1 : 0;
    if (boolVal != m_currentParameters["sampleClockRisingEdge"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("sampleClockRisingEdge", ito::ParamBase::Int,
                boolVal)
            );
    }

    byteArrayVal = ui.comboSampleClockSource->currentText().toLatin1();
    if (byteArrayVal != m_currentParameters["sampleClockSource"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("sampleClockSource", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    // Tab General, Start Trigger
    boolVal = ui.checkStartTriggerRisingEdge->isChecked() ? 1 : 0;
    if (boolVal != m_currentParameters["startTriggerRisingEdge"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("startTriggerRisingEdge", ito::ParamBase::Int,
                boolVal)
            );
    }

    byteArrayVal = ui.comboStartTriggerSource->currentText().toLatin1();
    if (byteArrayVal != m_currentParameters["startTriggerSource"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("startTriggerSource", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    byteArrayVal = ui.comboStartTriggerMode->currentText().toLatin1();
    if (byteArrayVal != m_currentParameters["startTriggerMode"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("startTriggerMode", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    if (std::abs(ui.doubleSpinStartTriggerLevel->value()
        - m_currentParameters["startTriggerLevel"].getVal<double>())
        > std::numeric_limits<double>::epsilon())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("startTriggerLevel", ito::ParamBase::Double,
                ui.doubleSpinStartTriggerLevel->value())
            );
    }

    return setPluginParameters(values, msgLevelWarningAndError);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogNiDAQmx::on_comboStartTriggerMode_currentTextChanged(QString text)
{
    ui.doubleSpinStartTriggerLevel->setEnabled(text == "analogEdge");
    ui.lblStartTriggerLevel->setEnabled(text == "analogEdge");
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::enableDialog(bool enabled)
{
    ui.tabWidget->setEnabled(enabled);
}


