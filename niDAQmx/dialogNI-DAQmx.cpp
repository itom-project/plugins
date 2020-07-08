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
    m_channelsModified(false),
    m_taskType(TypeUnknown),
    m_channelPropsChanging(false)
{
    ui.setupUi(this);

    enableDialog(false);

    // Task Mode
    ui.comboTaskMode->addItem(tr("finite"), "finite");
    ui.comboTaskMode->addItem(tr("continuous"), "continuous");
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

        QString taskType = params["taskType"].getVal<const char*>();
        ui.lblType->setText(taskType);

        if (taskType == "analogInput")
        {
            m_taskType = TypeAnalogInput;
        }
        else if (taskType == "analogOutput")
        {
            m_taskType = TypeAnalogOutput;
        }
        else if (taskType == "digitalInput")
        {
            m_taskType = TypeDigitalInput;
        }
        else if (taskType == "digitalOutput")
        {
            m_taskType = TypeDigitalOutput;
        }

        // Tab General, Group Acquisition / Data Write
        ui.doubleSpinReadTimeout->setEnabled(m_taskType & TypeInput);
        ui.lblReadTimeout->setEnabled(m_taskType & TypeInput);
        ui.checkSetValWaitForFinish->setEnabled(m_taskType & TypeOutput);

        // Tab General, Start Trigger
        ito::StringMeta *sm = params["startTriggerMode"].getMetaT<ito::StringMeta>();
        for (int i = 0; i < sm->getLen(); ++i)
        {
            ui.comboStartTriggerMode->addItem(sm->getString(i));
        }

        // Tab General, Ref Trigger
        sm = params["refTriggerMode"].getMetaT<ito::StringMeta>();
        for (int i = 0; i < sm->getLen(); ++i)
        {
            ui.comboRefTriggerMode->addItem(sm->getString(i));
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

        // Tab Logging
        sm = params["loggingOperation"].getMetaT<ito::StringMeta>();
        for (int i = 0; i < sm->getLen(); ++i)
        {
            ui.comboLoggingOperation->addItem(sm->getString(i));
        }

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
    ui.spinBufferSize->setValue(params["bufferSize"].getVal<int>());

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

    // Tab General, Ref Trigger
    ui.groupRefTrigger->setEnabled(params["refTriggerMode"].getFlags() == 0);
    ui.comboRefTriggerSource->setCurrentText(params["refTriggerSource"].getVal<const char*>());
    ui.checkRefTriggerRisingEdge->setChecked(params["refTriggerRisingEdge"].getVal<int>() > 0);
    ui.comboRefTriggerMode->setCurrentText(params["refTriggerMode"].getVal<const char*>());

    ui.doubleSpinRefTriggerLevel->setEnabled(ui.comboRefTriggerMode->currentText() == "analogEdge");
    ui.lblRefTriggerLevel->setEnabled(ui.comboRefTriggerMode->currentText() == "analogEdge");
    ui.doubleSpinRefTriggerLevel->setValue(params["refTriggerLevel"].getVal<double>());
    ui.spinRefTriggerPreTriggerSamples->setValue(params["refTriggerPreTriggerSamples"].getVal<int>());

    // Tab Channels
    QStringList channels = QString(params["channels"].getVal<const char*>()).split(";");
    QStringList channelFullnames;
    int idx;
    m_channelsModified = false;

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

            if (item->parent())
            {
                ui.treeChannels->expandItem(item->parent());
            }

            ui.treeChannels->expandItem(item);
        }
    }

    m_channelsModified = false;

    if (ui.treeChannels->selectedItems().size() > 0)
    {
        on_treeChannels_currentItemChanged(ui.treeChannels->selectedItems()[0], NULL);
    }
    else
    {
        on_treeChannels_currentItemChanged(NULL, NULL);
    }

    // Tab Logging
    ui.comboLoggingOperation->setCurrentText(params["loggingOperation"].getVal<const char*>());
    ui.pathLoggingFilename->setCurrentPath(params["loggingFilePath"].getVal<const char*>());
    ui.txtLoggingGroupName->setText(params["loggingGroupName"].getVal<const char*>());

    switch (params["loggingMode"].getVal<int>())
    {
    default:
    case 0:
        ui.radioLoggingOff->setChecked(true);
        break;
    case 1:
        ui.radioLoggingFast->setChecked(true);
        break;
    case 2:
        ui.radioLoggingNormal->setChecked(true);
        break;
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

    if (ui.spinBufferSize->value()
        != m_currentParameters["bufferSize"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("bufferSize", ito::ParamBase::Int,
                ui.spinBufferSize->value())
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

    // Tab General, Ref Trigger
    boolVal = ui.checkRefTriggerRisingEdge->isChecked() ? 1 : 0;
    if (boolVal != m_currentParameters["refTriggerRisingEdge"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("refTriggerRisingEdge", ito::ParamBase::Int,
                boolVal)
            );
    }

    byteArrayVal = ui.comboRefTriggerSource->currentText().toLatin1();
    if (byteArrayVal != m_currentParameters["refTriggerSource"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("refTriggerSource", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    byteArrayVal = ui.comboRefTriggerMode->currentText().toLatin1();
    if (byteArrayVal != m_currentParameters["refTriggerMode"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("refTriggerMode", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    if (std::abs(ui.doubleSpinRefTriggerLevel->value()
        - m_currentParameters["refTriggerLevel"].getVal<double>())
        > std::numeric_limits<double>::epsilon())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("refTriggerLevel", ito::ParamBase::Double,
                ui.doubleSpinRefTriggerLevel->value())
            );
    }

    if (ui.spinRefTriggerPreTriggerSamples->value()
        != m_currentParameters["refTriggerPreTriggerSamples"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("refTriggerPreTriggerSamples", ito::ParamBase::Int,
                ui.spinRefTriggerPreTriggerSamples->value())
            );
    }

    // Channels
    if (m_channelsModified)
    {
        QStringList channelsCfgStrings;

        foreach(const QTreeWidgetItem* item, m_channelItems)
        {
            if (item && item->checkState(0) == Qt::Checked)
            {
                channelsCfgStrings << item->data(0, CrConfigStr).toString();
            }
        }

        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("channels", ito::ParamBase::String,
                channelsCfgStrings.join(";").toLatin1().data())
            );
    }

    m_channelsModified = false;

    // Tab Logging
    byteArrayVal = ui.txtLoggingGroupName->text().toLatin1();
    if (byteArrayVal != m_currentParameters["loggingGroupName"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("loggingGroupName", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    byteArrayVal = ui.comboLoggingOperation->currentText().toLatin1();
    if (byteArrayVal != m_currentParameters["loggingOperation"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("loggingOperation", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    byteArrayVal = ui.pathLoggingFilename->currentPath().toLatin1();
    if (byteArrayVal != m_currentParameters["loggingFilePath"].getVal<const char*>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("loggingFilePath", ito::ParamBase::String,
                byteArrayVal.data())
            );
    }

    int intVal = 0;

    if (ui.radioLoggingFast->isChecked())
    {
        intVal = 1;
    }
    else if (ui.radioLoggingNormal->isChecked())
    {
        intVal = 2;
    }

    if (intVal != m_currentParameters["loggingMode"].getVal<int>())
    {
        values << QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("loggingMode", ito::ParamBase::Int,
                boolVal)
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
        switch (m_taskType)
        {
        case TypeAnalogInput:
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
            break;
        case TypeDigitalInput:
            setChannelPropsDI();
            break;
        case TypeAnalogOutput:
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
            break;
        case TypeDigitalOutput:
            setChannelPropsDO();
            break;
        }

        updateChannelCfg();
    }
    else
    {
        disableChannelProps();
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_treeChannels_itemChanged(QTreeWidgetItem *item, int column)
{
    if (item)
    {
        if (item->checkState(0) == Qt::Checked)
        {
            if (item->data(0, CrConfigStr).toString() == "")
            {
                m_channelsModified = true;
            }
        }
        else
        {
            if (item->data(0, CrConfigStr).toString() != "")
            {
                item->setData(0, CrConfigStr, "");
                m_channelsModified = true;
            }
        }
    }

    on_treeChannels_currentItemChanged(item, NULL);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::disableChannelProps()
{
    m_channelPropsChanging = true;
    ui.lblTerminalConfig->setVisible(false);
    ui.comboTerminalConfig->setVisible(false);
    ui.lblMinimumVoltage->setVisible(false);
    ui.doubleSpinMinimumVoltage->setVisible(false);
    ui.lblMaximumVoltage->setVisible(false);
    ui.doubleSpinMaximumVoltage->setVisible(false);
    ui.lineChannelCfgStr->setVisible(false);
    ui.lblChannelCfgStr->setVisible(false);
    ui.txtChannelCfgStr->setVisible(false);
    m_channelPropsChanging = false;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::setChannelPropsAI(int terminalConfig, double minV, double maxV)
{
    m_channelPropsChanging = true;
    ui.lblTerminalConfig->setVisible(true);
    ui.comboTerminalConfig->setVisible(true);
    ui.lblMinimumVoltage->setVisible(true);
    ui.doubleSpinMinimumVoltage->setVisible(true);
    ui.lblMaximumVoltage->setVisible(true);
    ui.doubleSpinMaximumVoltage->setVisible(true);
    ui.comboTerminalConfig->setCurrentIndex(terminalConfig);
    ui.doubleSpinMinimumVoltage->setValue(minV);
    ui.doubleSpinMaximumVoltage->setValue(maxV);
    ui.lineChannelCfgStr->setVisible(true);
    ui.lblChannelCfgStr->setVisible(true);
    ui.txtChannelCfgStr->setVisible(true);
    m_channelPropsChanging = false;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::setChannelPropsAO(double minV, double maxV)
{
    m_channelPropsChanging = true;
    ui.lblTerminalConfig->setVisible(false);
    ui.comboTerminalConfig->setVisible(false);
    ui.lblMinimumVoltage->setVisible(true);
    ui.doubleSpinMinimumVoltage->setVisible(true);
    ui.lblMaximumVoltage->setVisible(true);
    ui.doubleSpinMaximumVoltage->setVisible(true);
    ui.doubleSpinMinimumVoltage->setValue(minV);
    ui.doubleSpinMaximumVoltage->setValue(maxV);
    ui.lineChannelCfgStr->setVisible(true);
    ui.lblChannelCfgStr->setVisible(true);
    ui.txtChannelCfgStr->setVisible(true);
    m_channelPropsChanging = false;
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

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::updateChannelCfg()
{
    QTreeWidgetItem *current = ui.treeChannels->currentItem();
    QString cfgString;

    double minV = ui.doubleSpinMinimumVoltage->value();
    double maxV = ui.doubleSpinMaximumVoltage->value();
    int terminalCfg = ui.comboTerminalConfig->currentIndex();

    if (current != NULL)
    {
        switch (m_taskType)
        {
        case TypeAnalogInput:
        {
            NiAnalogInputChannel chn(current->data(0, CrPhysicalName).toString());

            chn.setMaxInputLim(maxV);
            chn.setMinInputLim(minV);

            switch (terminalCfg)
            {
            case NiAnalogInputChannel::NiTerminalConfDefault:
            default:
                chn.setTerminalConfig(NiAnalogInputChannel::NiTerminalConfDefault);
                break;
            case NiAnalogInputChannel::NiTerminalConfDifferential:
                chn.setTerminalConfig(NiAnalogInputChannel::NiTerminalConfDifferential);
                break;
            case NiAnalogInputChannel::NiTerminalConfRSE:
                chn.setTerminalConfig(NiAnalogInputChannel::NiTerminalConfRSE);
                break;
            case NiAnalogInputChannel::NiTerminalConfNRSE:
                chn.setTerminalConfig(NiAnalogInputChannel::NiTerminalConfNRSE);
                break;
            case NiAnalogInputChannel::NiTerminalConfPseudoDiff:
                chn.setTerminalConfig(NiAnalogInputChannel::NiTerminalConfPseudoDiff);
                break;
            }
            
            cfgString = chn.getConfigurationString();
        }
        break;
        case TypeAnalogOutput:
        {
            NiAnalogOutputChannel chn(current->data(0, CrPhysicalName).toString());

            chn.setMaxOutputLim(maxV);
            chn.setMinOutputLim(minV);
            cfgString = chn.getConfigurationString();
        }
        break;
        case TypeDigitalInput:
        case TypeDigitalOutput:
            cfgString = current->data(0, CrPhysicalName).toString();
            break;
        
        }
    }

    if (!m_channelPropsChanging && current != NULL && cfgString != "")
    {
        ui.treeChannels->blockSignals(true);

        if (current->data(0, CrConfigStr).toString() != cfgString)
        {
            qDebug() << "Channels Modified";
            current->setData(0, CrConfigStr, cfgString);
            m_channelsModified = true;
        }

        ui.treeChannels->blockSignals(false);
    }

    ui.txtChannelCfgStr->setText(cfgString);
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_comboTerminalConfig_currentIndexChanged(int index)
{
    updateChannelCfg();
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_doubleSpinMaximumVoltage_valueChanged(double value)
{
    updateChannelCfg();
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_doubleSpinMinimumVoltage_valueChanged(double value)
{
    updateChannelCfg();
}

//----------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::on_comboStartTriggerMode_currentIndexChanged(QString text)
{
    ui.doubleSpinStartTriggerLevel->setEnabled(text == "analogEdge");
}



