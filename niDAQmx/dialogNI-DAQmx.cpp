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
#include <qdebug.h>
#include "NI-PeripheralClasses.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogNiDAQmx::DialogNiDAQmx(ito::AddInBase *adda) :
    AbstractAddInConfigDialog(adda),
    m_firstRun(true),
    m_channelsModified(false)
{
    ui.setupUi(this);

    enableDialog(false);

    // Task Mode
    ui.comboTaskMode->addItem(tr("finite"), "finite");
    ui.comboTaskMode->addItem(tr("continuous"), "continuous");
    ui.comboTaskMode->addItem(tr("onDemand (not supported yet)"), "onDemand");
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
        ui.doubleSpinReadTimeout->setEnabled(ui.lblType->text().endsWith("Input"));
        ui.lblReadTimeout->setEnabled(ui.lblType->text().endsWith("Input"));
        ui.checkSetValWaitForFinish->setEnabled(ui.lblType->text().endsWith("Output"));

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
                child->setData(0, CrPhysicalName, channel);
                child->setData(0, CrConfigStr, "");
                child->setData(0, CrModified, false);
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

    // Tab Channels
    QStringList channels = QString(params["channels"].getVal<const char*>()).split(";");
    QStringList channelFullnames;
    int idx;
    m_channelsModified = true;

    foreach(const QString &c, channels)
    {
        idx = c.indexOf(",");

        if (idx >= 0)
        {
            channelFullnames << c.left(idx);
        }
    }

    // 1. uncheck all items
    foreach(QTreeWidgetItem *item, m_channelItems)
    {
        idx = channelFullnames.indexOf(item->data(0, CrPhysicalName).toString());

        if (idx == -1)
        {
            item->setCheckState(0, Qt::Unchecked);
            item->setData(0, CrConfigStr, "");
            item->setData(0, CrModified, false);
        }
        else
        {
            item->setCheckState(0, Qt::Checked);
            item->setData(0, CrConfigStr, channels[idx]); //full string with physical name
            item->setData(0, CrModified, false);
            ui.treeChannels->expandItem(item);
        }
    }

    if (ui.treeChannels->selectedItems().size() > 0)
    {
        on_treeChannels_currentItemChanged(ui.treeChannels->selectedItems()[0], NULL);
    }
    else
    {
        on_treeChannels_currentItemChanged(NULL, NULL);
    }
    
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

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_treeChannels_currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem * /*previous*/)
{
    QString configString;
    ito::RetVal retVal;

    if (m_channelItems.values().contains(current) && current->checkState(0) == Qt::Checked)
    {
        if (ui.lblType->text().endsWith("Input"))
        {
            if (ui.lblType->text().startsWith("analog"))
            {
                NiAnalogInputChannel* chn = static_cast<NiAnalogInputChannel*>(
                    NiAnalogInputChannel::fromConfigurationString(current->data(0, CrConfigStr).toString(), retVal));

                if (chn && !retVal.containsError())
                {
                    setChannelPropsAI(chn->getTerminalConfig(), chn->getMinInputLim(), chn->getMaxInputLim());
                }
                else
                {
                    disableChannelProps();
                }

                DELETE_AND_SET_NULL(chn);
            }
            else
            {
                setChannelPropsDI();
            }
        }
        else //Output
        {
            if (ui.lblType->text().startsWith("analog"))
            {
                NiAnalogOutputChannel* chn = static_cast<NiAnalogOutputChannel*>(
                    NiAnalogOutputChannel::fromConfigurationString(current->data(0, CrConfigStr).toString(), retVal));

                if (chn && !retVal.containsError())
                {
                    setChannelPropsAO(chn->getMinOutputLim(), chn->getMaxOutputLim());
                }
                else
                {
                    disableChannelProps();
                }

                DELETE_AND_SET_NULL(chn);
            }
            else
            {
                setChannelPropsDO();
            }
        }
    }
    else
    {
        disableChannelProps();
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_treeChannels_itemChanged(QTreeWidgetItem *item, int column)
{
    qDebug() << "itemChanged" << item << column;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::disableChannelProps()
{
    ui.lblTerminalConfig->setVisible(false);
    ui.comboTerminalConfig->setVisible(false);
    ui.lblMinimumVoltage->setVisible(false);
    ui.doubleSpinMinimumVoltage->setVisible(false);
    ui.lblMaximumVoltage->setVisible(false);
    ui.doubleSpinMaximumVoltage->setVisible(false);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::setChannelPropsAI(int terminalConfig, double minV, double maxV)
{
    ui.lblTerminalConfig->setVisible(true);
    ui.comboTerminalConfig->setVisible(true);
    ui.lblMinimumVoltage->setVisible(true);
    ui.doubleSpinMinimumVoltage->setVisible(true);
    ui.lblMaximumVoltage->setVisible(true);
    ui.doubleSpinMaximumVoltage->setVisible(true);
    ui.comboTerminalConfig->setCurrentIndex(terminalConfig);
    ui.doubleSpinMinimumVoltage->setValue(minV);
    ui.doubleSpinMaximumVoltage->setValue(maxV);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::setChannelPropsAO(double minV, double maxV)
{
    ui.lblTerminalConfig->setVisible(false);
    ui.comboTerminalConfig->setVisible(false);
    ui.lblMinimumVoltage->setVisible(true);
    ui.doubleSpinMinimumVoltage->setVisible(true);
    ui.lblMaximumVoltage->setVisible(true);
    ui.doubleSpinMaximumVoltage->setVisible(true);
    ui.doubleSpinMinimumVoltage->setValue(minV);
    ui.doubleSpinMaximumVoltage->setValue(maxV);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::setChannelPropsDI()
{
    disableChannelProps();
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::setChannelPropsDO()
{
    disableChannelProps();
}



