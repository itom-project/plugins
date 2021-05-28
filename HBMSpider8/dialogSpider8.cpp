/* ********************************************************************
    Plugin "Spider8" for itom software
    URL: http://lccv.ufal.br/
    Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil

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

#include "dialogSpider8.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include <qmath.h>
#include <qmessagebox.h>
#include <qgraphicsitem.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogSpider8::DialogSpider8(ito::AddInBase *dataIO, void *plugin) :
    m_pPlugin((Spider8*)plugin),
    AbstractAddInConfigDialog(dataIO),
    m_firstRun(true)
{
    ui.setupUi(this);

    ui.frequencyCombo->clear();
    ui.frequencyCombo->addItem("1.0", 0);
    ui.frequencyCombo->addItem("2.0", 1);
    ui.frequencyCombo->addItem("5.0", 2);
    ui.frequencyCombo->addItem("10.0", 3);
    ui.frequencyCombo->addItem("25.0", 4);
    ui.frequencyCombo->addItem("50.0", 5);
    ui.frequencyCombo->addItem("60.0", 6);
    ui.frequencyCombo->addItem("75.0", 7);
    ui.frequencyCombo->addItem("100.0", 8);
    ui.frequencyCombo->addItem("150.0", 9);
    ui.frequencyCombo->addItem("200.0", 10);
    ui.frequencyCombo->addItem("300.0", 11);
    ui.frequencyCombo->addItem("400.0", 12);
    ui.frequencyCombo->addItem("600.0", 13);
    ui.frequencyCombo->addItem("800.0", 14);
    ui.frequencyCombo->addItem("1200.0", 15);
    ui.frequencyCombo->addItem("1600.0", 16);
    ui.frequencyCombo->addItem("2400.0", 17);
    ui.frequencyCombo->addItem("3200.0", 18);
    ui.frequencyCombo->addItem("4800.0", 19);
    ui.frequencyCombo->addItem("9600.0", 20);

    ui.frequencyCombo->setCurrentIndex(15);

    ui.aiConfigCombo->clear();
    ui.aiConfigCombo->addItem("V/V - Full Bridge", 0);
    ui.aiConfigCombo->addItem("V/V - Half Bridge", 1);
    ui.aiConfigCombo->addItem("V/V - Quarter Bridge ", 2);
    ui.aiConfigCombo->addItem("V", 70);
    ui.aiConfigCombo->addItem("A", 71);
    ui.aiConfigCombo->addItem("V - Thermocopule J", 100);
    ui.aiConfigCombo->addItem("V - Thermocopule K", 101);
    ui.aiConfigCombo->addItem("V - Thermocopule T", 102);
    ui.aiConfigCombo->addItem("V - Thermocopule S", 103);
    ui.aiConfigCombo->addItem("V - Thermocopule B", 104);
    ui.aiConfigCombo->addItem("V - Thermocopule E", 105);
    ui.aiConfigCombo->addItem("V - Thermocopule R", 106);
    ui.aiConfigCombo->addItem("V - internal Thermo", 160);
    ui.aiConfigCombo->addItem("ohm - Resistor", 125);
    ui.aiConfigCombo->addItem("ohm - Pt100", 151);
    ui.aiConfigCombo->addItem("ohm - Pt500", 152);
    ui.aiConfigCombo->addItem("ohm - Pt1000", 153);
    ui.aiConfigCombo->addItem("V - Signal Edges", 170);
    ui.aiConfigCombo->addItem("V - Signal Edges & dir", 171);
    ui.aiConfigCombo->addItem("V - 2 Phases, single Edge", 172);
    ui.aiConfigCombo->addItem("V - 2 Phases, four Edges", 173);
    ui.aiConfigCombo->addItem("Digital I/O", 178);

    ui.aiChannelCombo->clear();
    ui.cioChannelCombo->clear();
    ui.dioChannelCombo->clear();

    m_pqgscene = new QGraphicsScene();
    ui.graphicsView->setScene(m_pqgscene);
    QPixmap sview(":dialog/spiderimg_small.png");
    m_pqgscene->addPixmap(sview);

    if (m_pPlugin)
    {
        QMap<int, Spider8Channel> *channels = m_pPlugin->getChannels();
        for (int nc = 0; nc < 8; nc++)
        {
            int fSize = 13;
            int y = nc < 4 ? 108 : 54;
            int x = 95 + 114 * (nc % 4);
            int type = 0;
            if (channels->contains(nc))
            {
                type = channels->value(nc).m_type - 5050;;
            }
            QGraphicsSimpleTextItem *st = NULL;
            switch (type)
            {
                case 0:
                    st = m_pqgscene->addSimpleText("N/A", QFont("Arial", fSize));
                    st->setPos(x, y);
                break;

                case 1:
                case 2:
                case 7:
                    st = m_pqgscene->addSimpleText("CF", QFont("Arial", fSize));
                    st->setPos(x, y);
                break;

                case 3:
                    st = m_pqgscene->addSimpleText("SR55", QFont("Arial", fSize));
                    st->setPos(x, y);
                break;

                case 4:
                    st = m_pqgscene->addSimpleText("SR01", QFont("Arial", fSize));
                    st->setPos(x, y);
                break;

                case 5: 
                    st = m_pqgscene->addSimpleText("D I/O", QFont("Arial", fSize));
                    st->setPos(x, y);
                break;

                case 6:
                    st = m_pqgscene->addSimpleText("AUX", QFont("Arial", fSize));
                    st->setPos(x, y);
                break;

                case 8:
                    st = m_pqgscene->addSimpleText("SR30", QFont("Arial", fSize));
                    st->setPos(x, y);
                break;
            }
        }
    }
    else
    {
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::parametersChanged(QMap<QString, ito::Param> params)
{
    // Populate channel boxes
    m_params.clear();
    m_params = params;

    QMap<int, Spider8Channel> *channels = m_pPlugin->getChannels();
    if (m_firstRun)
    {
        ui.aiChannelCombo->clear();
        ui.dioChannelCombo->clear();
        ui.cioChannelCombo->clear();
        for (int nc = 0; nc < channels->size(); nc++)
        {
            ui.aiChannelCombo->addItem(QString::number(nc));
        }
        m_firstRun = false;
        ui.aiChannelCombo->setCurrentIndex(0);
    }
    int curCha = ui.aiChannelCombo->currentIndex();
    int unit = (*channels)[curCha].m_mode;
    int range = (*channels)[curCha].m_range;
    setRanges((*channels)[unit].m_range);
    for (int ni = 0; ni < ui.aiConfigCombo->count(); ni++)
    {
        if (ui.aiConfigCombo->itemData(ni) == unit)
        {
            ui.aiConfigCombo->setCurrentIndex(ni);
            break;
        }
    }

    for (int ni = 0; ni < ui.aiRangeCombo->count(); ni++)
    {
        if (ui.aiRangeCombo->itemData(ni) == range)
        {
            ui.aiRangeCombo->setCurrentIndex(ni);
            break;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::setRanges(const int unit)
{
    ui.aiRangeCombo->clear();
    switch (unit)
    {
        case 0:
            ui.aiRangeCombo->addItem("0.003 V/V", 0);
            ui.aiRangeCombo->addItem("0.012 V/V", 1);
            ui.aiRangeCombo->addItem("0.125 V/V", 2);
            ui.aiRangeCombo->addItem("0.500 V/V", 3);
        break;

        case 1:
            ui.aiRangeCombo->addItem("0.1 V", 10);
            ui.aiRangeCombo->addItem("1.0 V", 11);
            ui.aiRangeCombo->addItem("10.0 V", 12);
        break;
        
        case 2:
            ui.aiRangeCombo->addItem("0.02 A", 20);
            ui.aiRangeCombo->addItem("0.2 A", 21);
        break;
        
        case 3:
            ui.aiRangeCombo->addItem("400.0 ohm", 30);
            ui.aiRangeCombo->addItem("4000.0 ohm", 31);
        break;

        case 4:
            ui.aiRangeCombo->addItem("100.0 Hz", 44);
            ui.aiRangeCombo->addItem("1.0 kHz", 43);
            ui.aiRangeCombo->addItem("10.0 kHz", 42);
            ui.aiRangeCombo->addItem("100.0 kHz", 41);
            ui.aiRangeCombo->addItem("1.0 MHz", 40);
            ui.aiRangeCombo->addItem("0.01 s", 50);
            ui.aiRangeCombo->addItem("0.1 s", 51);
            ui.aiRangeCombo->addItem("1.0 s", 52);
            ui.aiRangeCombo->addItem("10.0 s", 53);
            ui.aiRangeCombo->addItem("100.0 s", 54);
            ui.aiRangeCombo->addItem("1 Count", 60);
            ui.aiRangeCombo->addItem("100 Count", 61);
        break;

        case 5:
            ui.aiRangeCombo->addItem("1 digit", 65);
        break;

        case 6:
            ui.aiRangeCombo->addItem("1 °C", 70);
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogSpider8::applyParameters()
{
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
// Task Slots
void DialogSpider8::on_taskApplyButton_clicked(bool checked)
{
    ito::RetVal retval(ito::retOk);

    retval += m_pPlugin->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("samplingRate", ito::ParamBase::Double, atof(ui.frequencyCombo->currentText().toLatin1().data()))), 0);
    retval += m_pPlugin->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("numSamples", ito::ParamBase::Int, ui.samplesSpin->value())), 0);
    retval += m_pPlugin->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("numCycles", ito::ParamBase::Int, ui.cyclesSpin->value())), 0);
    if (retval.containsWarningOrError())
        QMessageBox::information(this, tr("Error"), QString(retval.errorMessage()));
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_taskCombo_currentIndexChanged(int index)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_aiConfigCombo_currentIndexChanged(int index)
{
    switch (index)
    {
        case 0:
        case 1:
        case 2:
            setRanges(0);
        break;
        case 3:
            setRanges(1);
        break;
        case 4:
            setRanges(2);
        break;
        case 5:
        case 6:
        case 7:
        case 8:
        case 10:
        case 11:
        case 12:
        case 13:
            setRanges(1);
        break;
        case 14:
        case 15:
        case 16:
        case 17:
            setRanges(3);
        break;
        case 19:
        case 20:
        case 21:
        case 22:
            setRanges(4);
        break;
    }
    Spider8Channel *cha = m_pPlugin->getChannel(ui.aiChannelCombo->currentIndex());
    if (cha != NULL)
    {
        for (int ni = 0; ni < ui.aiRangeCombo->count(); ni++)
        {
            if (ui.aiRangeCombo->itemData(ni) == cha->m_range)
            {
                ui.aiRangeCombo->setCurrentIndex(ni);
                break;
            }
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// Analog Input Slots
void DialogSpider8::on_aiApplyButton_clicked(bool checked)
{
    QString channel = QString::number(ui.aiChannelCombo->currentIndex());
    QStringList params;
    params.append(channel);
    params.append(ui.aiConfigCombo->itemData(ui.aiConfigCombo->currentIndex()).toString());
    params.append(ui.aiRangeCombo->itemData(ui.aiRangeCombo->currentIndex()).toString());
    ito::RetVal retval = m_pPlugin->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("aiChParams", ito::ParamBase::String, params.join(",").toLatin1().data())),0);
    if (retval.containsWarningOrError())
        QMessageBox::information(this, tr("Error"), QString(retval.errorMessage()));
    else
    {
        ui.cmdOutputLE->setText(QString("setParam(\"aiChParams\",\"") + params.join(",") + QString("\")"));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_aiChannelCombo_currentIndexChanged(int index)
{
    QStringList chParams = QString(m_params["aiChParams"].getVal<char*>()).split(";", QString::SkipEmptyParts);
    if (chParams.size() > 0)
    {
        bool found = false;
        foreach(const QString &ch, chParams)
        {
            if (ch.split(",")[0] == QString::number(ui.aiChannelCombo->currentIndex()))
            {
                int mode = ch.split(",")[1].toInt();
                for (int ni = 0; ni < ui.aiConfigCombo->count(); ni++)
                {
                    if (ui.aiConfigCombo->itemData(ni) == mode)
                    {
                        ui.aiConfigCombo->setCurrentIndex(ni);
                        break;
                    }
                }
                int range = ch.split(",")[2].toInt();
                for (int ni = 0; ni < ui.aiRangeCombo->count(); ni++)
                {
                    if (ui.aiRangeCombo->itemData(ni) == range)
                    {
                        ui.aiRangeCombo->setCurrentIndex(ni);
                        break;
                    }
                }
                found = true;
                break;
            }
        }
        if (!found)
        {
            ui.aiConfigCombo->setCurrentIndex(-1);
            ui.aiRangeCombo->setCurrentIndex(0);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_aiBitCombo_currentIndexChanged(int index)
{
    calculateResolution();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_aiRangeCombo_currentIndexChanged(int index)
{
    calculateResolution();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::calculateResolution()
{
//    double range = 2*ui.aiRangeCombo->itemData(ui.aiRangeCombo->currentIndex()).toDouble();
    int range = ui.aiRangeCombo->currentData().toInt();
    int channel = ui.aiChannelCombo->currentIndex();
    Spider8Channel *cha = NULL;
    if ((cha = m_pPlugin->getChannel(channel)) != NULL)
    {
        double value = cha->mapSFactors[range + 700];
        ui.aiResolutionLabel->setText(QString::number(value)+" mV/digit");        
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// Digital Slots
void DialogSpider8::on_dioApplyButton_clicked(bool checked)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_dioChannelCombo_currentIndexChanged(int index)
{
    // just copy the code from the ai comboBox and adjust the parameters
}

//----------------------------------------------------------------------------------------------------------------------------------
// Counter Slots
void DialogSpider8::on_cioApplyButton_clicked(bool checked)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_cioChannelCombo_currentIndexChanged(int index)
{
    // just copy the code from the ai comboBox and adjust the parameters
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSpider8::on_buttonBox_clicked(QAbstractButton *btn)
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

//----------------------------------------------------------------------------------------------------------------------------------