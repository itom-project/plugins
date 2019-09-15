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

//----------------------------------------------------------------------------------------------------------------------------------
DialogNiDAQmx::DialogNiDAQmx(ito::AddInBase *grabber, void *plugin) :
    m_pPlugin(plugin),
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    // Analog Input Config
    ui.aiConfigCombo->insertItem(0, QIcon(), "PseudoDiff = 4", 4);
    ui.aiConfigCombo->insertItem(0, QIcon(), "NRSE = 3", 3);
    ui.aiConfigCombo->insertItem(0, QIcon(), "RSE = 2", 2);
    ui.aiConfigCombo->insertItem(0, QIcon(), "Differential = 1", 1);
    ui.aiConfigCombo->insertItem(0, QIcon(), "Default = 0", 0);

    // Voltage Ranges
    ui.aiRangeCombo->insertItem(0, QIcon(), "±0.2 V", 0.2);
    ui.aiRangeCombo->insertItem(0, QIcon(), "±0.5 V", 0.5);
    ui.aiRangeCombo->insertItem(0, QIcon(), "±1.0 V", 1.0);
    ui.aiRangeCombo->insertItem(0, QIcon(), "±2.0V" , 2.0);
    ui.aiRangeCombo->insertItem(0, QIcon(), "±5.0 V" , 5.0);
    ui.aiRangeCombo->insertItem(0, QIcon(), "±10.0 V", 10.0);
    ui.aiRangeCombo->insertItem(0, QIcon(), "±20.0V" , 20.0);
    ui.aiRangeCombo->insertItem(0, QIcon(), "±42.0 V", 42.0);

    // ADC-Bit
    ui.aiBitCombo->insertItem(0, QIcon(), "18 Bit", 18);
    ui.aiBitCombo->insertItem(0, QIcon(), "16 Bit", 16);
    ui.aiBitCombo->insertItem(0, QIcon(), "12 Bit", 12);
    
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogNiDAQmx::parametersChanged(QMap<QString, ito::Param> params)
{
    // Populate channel boxes
    m_params.clear();
    m_params = params;

    // different Tasks
    QString p;
    QMap<QString, QString> tasks;
    tasks.insert("Counter write", "co");
    tasks.insert("Counter read", "ci");
    tasks.insert("Digital write", "do");
    tasks.insert("Digital read", "di");
    tasks.insert("Analog write", "ao");
    tasks.insert("Analog read", "ai");

    ui.taskCombo->clear();
    foreach(const QString &t, tasks)
    {
        p = QString(params["taskStatus"].getVal<char*>());
        QString post = "";
        if (p.split(";").filter(t)[0].split(",")[1] == "-1")
        {
            if (t == "ai")
            {
                ui.aiApplyButton->setEnabled(false);
            }
            else if (t == "ao")
            {
                ui.aoApplyButton->setEnabled(false);
            }
            else if (t[0] == 'd')
            {
                ui.dioApplyButton->setEnabled(false);
            }
            else if (t[0] == 'c')
            {
                ui.cioApplyButton->setEnabled(false);
            }
            post = " (not initialized)";
        }
        else 
        {
            if (t == "ai")
            {
                ui.aiApplyButton->setEnabled(true);
            }
            else if (t == "ao")
            {
                ui.aoApplyButton->setEnabled(true);
            }
            else if (t[0] == 'd')
            {
                ui.dioApplyButton->setEnabled(true);
            }
            else if (t[0] == 'c')
            {
                ui.cioApplyButton->setEnabled(true);
            }
        }
        ui.taskCombo->insertItem(0, QIcon(), tasks.key(t)+post, t);
    }
    ui.taskCombo->model()->sort(0);

    ui.aiChannelCombo->clear();
    ui.aoChannelCombo->clear();
    ui.dioChannelCombo->clear();
    ui.cioChannelCombo->clear();
    QString channel = QString(params["channel"].getVal<char*>());
    QString associated = QString(params["chAssociated"].getVal<char*>());
    foreach(const QString &s, channel.split(","))
    {
        QString ch = s;
        if (!associated.contains(ch))
        {
            ch.append(" (not created yet)");
        }

        if (ch.contains("ai"))
        {
            ui.aiChannelCombo->insertItem(ui.aiChannelCombo->count(), QIcon(), ch, s);
        }
        else if (ch.contains("ao"))
        {
            ui.aoChannelCombo->insertItem(ui.aiChannelCombo->count(), QIcon(), ch, s);
        }
        else if (ch.contains("port"))
        {
            ui.dioChannelCombo->insertItem(ui.aiChannelCombo->count(), QIcon(), ch, s);
        }
        else if (ch.contains("ctr"))
        {
            ui.cioChannelCombo->insertItem(ui.aiChannelCombo->count(), QIcon(), ch, s);
        }
    }
}


ito::RetVal DialogNiDAQmx::applyParameters()
{
    return ito::retOk;
}


// Task Slots
void DialogNiDAQmx::on_taskApplyButton_clicked(bool checked)
{
    NiDAQmx *plugin = (NiDAQmx*) m_pPlugin;
    QString p = ui.taskCombo->itemData(ui.taskCombo->currentIndex()).toString()+"TaskParams";
    QStringList params;
    params.append(QString::number(ui.taskRateSpin->value()));
    params.append(QString::number(ui.taskSampleSpin->value()));
    params.append(QString::number(0));
    plugin->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase(p.toLatin1().data(), ito::ParamBase::String, params.join(",").toLatin1().data())),0);
}

void DialogNiDAQmx::on_taskCombo_currentIndexChanged(int index)
{
    ui.taskChannelList->clear();
    QString prefix = ui.taskCombo->itemData(index).toString();
    if (m_params.contains(prefix+"TaskParams"))
    {
        QString params = QString(m_params[prefix+"TaskParams"].getVal<char*>());
        if (params != "-1")
        {
            QStringList pL = params.split(",", QString::SkipEmptyParts);
            ui.taskRateSpin->setValue(pL[0].toInt());
            ui.taskSampleSpin->setValue(pL[1].toInt());
            // get corresponding channels
            if (m_params.contains("chAssociated"))
            {
                QStringList channels = QString(m_params["chAssociated"].getVal<char*>()).split(";", QString::SkipEmptyParts);
                foreach(const QString &ch, channels)
                {
                    if (ch.contains(prefix))
                    {
                        ui.taskChannelList->insertItems(0,ch.split(","));
                    }
                }
            }
        }
        else
        {
            ui.taskRateSpin->setValue(1);
            ui.taskSampleSpin->setValue(1);
        }
    }
}


// Analog Input Slots
void DialogNiDAQmx::on_aiApplyButton_clicked(bool checked)
{
    NiDAQmx *plugin = (NiDAQmx*) m_pPlugin;
    QString channel = ui.aiChannelCombo->itemData(ui.aiChannelCombo->currentIndex()).toString();
    QStringList params;
    params.append(channel);
    params.append(ui.aiConfigCombo->itemData(ui.aiConfigCombo->currentIndex()).toString());
    params.append(ui.aiRangeCombo->itemData(ui.aiRangeCombo->currentIndex()).toString());
    plugin->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("aiChParams", ito::ParamBase::String, params.join(",").toLatin1().data())),0);
}

void DialogNiDAQmx::on_aiChannelCombo_currentIndexChanged(int index)
{
    QStringList chParams = QString(m_params["aiChParams"].getVal<char*>()).split(";", QString::SkipEmptyParts);
    if (chParams.size() > 0)
    {
        bool found = false;
        foreach(const QString &ch, chParams)
        {
            if (ch.split(",")[0]+"/"+ch.split(",")[1] == ui.aiChannelCombo->itemData(index).toString())
            {
                ui.aiConfigCombo->setCurrentIndex(ch.split(",")[2].toInt());
                ui.aiRangeCombo->setCurrentIndex(ch.split(",")[3].toInt());
                found = true;
            }
        }
        if (!found)
        {
            ui.aiConfigCombo->setCurrentIndex(-1);
            ui.aiRangeCombo->setCurrentIndex(0);
        }
    }
}

void DialogNiDAQmx::on_aiBitCombo_currentIndexChanged(int index)
{
    calculateResolution();
}

void DialogNiDAQmx::on_aiRangeCombo_currentIndexChanged(int index)
{
    calculateResolution();
}

void DialogNiDAQmx::calculateResolution()
{
    double range = 2*ui.aiRangeCombo->itemData(ui.aiRangeCombo->currentIndex()).toDouble();
    int bit = ui.aiBitCombo->itemData(ui.aiBitCombo->currentIndex()).toInt();
    ui.aiResolutionLabel->setText(QString::number(range/qPow(2,bit)*1000)+" mV/bit");
}

// Analog Output Slots
void DialogNiDAQmx::on_aoApplyButton_clicked(bool checked)
{
    NiDAQmx *plugin = (NiDAQmx*) m_pPlugin;
    QString channel = ui.aoChannelCombo->itemData(ui.aoChannelCombo->currentIndex()).toString();
    QStringList params;
    params.append(channel);
    params.append(QString::number(ui.aoMinSpin->value()));
    params.append(QString::number(ui.aoMaxSpin->value()));
    plugin->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("aoChParams", ito::ParamBase::String, params.join(",").toLatin1().data())),0);
}

void DialogNiDAQmx::on_aoChannelCombo_currentIndexChanged(int index)
{
    QStringList chParams = QString(m_params["aoChParams"].getVal<char*>()).split(";", QString::SkipEmptyParts);
    if (chParams.size() > 0)
    {
        bool found = false;
        foreach(const QString &ch, chParams)
        {
            if (ch.split(",")[0]+"/"+ch.split(",")[1] == ui.aoChannelCombo->itemData(index).toString())
            {
                ui.aoMinSpin->setValue(ch.split(",")[2].toInt());
                ui.aoMaxSpin->setValue(ch.split(",")[3].toInt());
                found = true;
            }
        }
        if (!found)
        {
            ui.aoMinSpin->setValue(-10);
            ui.aoMaxSpin->setValue(10);
        }
    }
}


// Digital Slots
void DialogNiDAQmx::on_dioApplyButton_clicked(bool checked)
{

}

void DialogNiDAQmx::on_dioChannelCombo_currentIndexChanged(int index)
{
    // just copy the code from the ai comboBox and adjust the parameters
}


// Counter Slots
void DialogNiDAQmx::on_cioApplyButton_clicked(bool checked)
{

}

void DialogNiDAQmx::on_cioChannelCombo_currentIndexChanged(int index)
{
    // just copy the code from the ai comboBox and adjust the parameters
}
