/* ********************************************************************
Plugin "Roughness" for itom software
URL : http ://www.uni-stuttgart.de/ito
Copyright(C) 2016, Institut fuer Technische Optik (ITO),
Universitaet Stuttgart, Germany;
IPROM, TU Braunschweig, Germany

This file is part of a plugin for the measurement software itom.

This itom - plugin is free software; you can redistribute it and / or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or(at
your option) any later version.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dialogAvtVimba.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogAvtVimba::DialogAvtVimba(ito::AddInBase *grabber, const BppEnum *bppEnum/*, const TriggerSourceEnum *triggerSourceEnum, const TriggerActivationEnum *triggerActivationEnum*/) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    ui.comboBppMode->clear();
    if (bppEnum->bppMono8 >= 0) ui.comboBppMode->addItem("8 bit", 8);
    if (bppEnum->bppMono10 >= 0) ui.comboBppMode->addItem("10 bit", 10);
    if (bppEnum->bppMono12 >= 0) ui.comboBppMode->addItem("12 bit", 12);
    if (bppEnum->bppMono14 >= 0) ui.comboBppMode->addItem("14 bit", 14);

    //disable dialog, since no parameters are known yet. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogAvtVimba::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());
#else
        ito::IntMeta *im;
        im = static_cast<ito::IntMeta*>(params["x0"].getMeta());
        ui.rangeX01->setSingleStep(im->getStepSize());
        ui.rangeX01->setMinimum(0);
        ui.rangeX01->setMinimumValue(0);
        im = static_cast<ito::IntMeta*>(params["x1"].getMeta());
        ui.rangeX01->setMaximum(im->getMax());
        ui.rangeX01->setMaximumValue(im->getMax());

        im = static_cast<ito::IntMeta*>(params["y0"].getMeta());
        ui.rangeY01->setSingleStep(im->getStepSize());
        ui.rangeY01->setMinimum(0);
        ui.rangeY01->setMinimumValue(0);
        im = static_cast<ito::IntMeta*>(params["y1"].getMeta());
        ui.rangeY01->setMaximum(im->getMax());
        ui.rangeY01->setMaximumValue(im->getMax());
#endif

        ui.groupGigE->setVisible( QString("GigE") == params["interface"].getVal<char*>());
        //window()->resize(window()->size());

        int binV_min = (int)(params["binning"].getMin()) % 100;
        int binH_min = ((int)(params["binning"].getMin()) - binV_min) / 100;
        int binV_max = (int)(params["binning"].getMax()) % 100;
        int binH_max = ((int)(params["binning"].getMax()) - binV_max) / 100;
        ui.comboBinHor->clear();
        ui.comboBinVer->clear();
        for (int i = binH_min; i <= binH_max; ++i)
        {
            ui.comboBinHor->addItem(QString("%1x").arg(i), i);
        }
        for (int i = binV_min; i <= binV_max; ++i)
        {
            ui.comboBinVer->addItem(QString("%1x").arg(i), i);
        }

        ui.comboTriggerActivation->clear();
        const ito::StringMeta *sm = (const ito::StringMeta*)(params["trigger_activation"].getMeta());
        for (int i = 0; i < sm->getLen(); ++i)
        {
            ui.comboTriggerActivation->addItem(sm->getString(i), sm->getString(i));
        }

        ui.comboTriggerSource->clear();
        sm = (const ito::StringMeta*)(params["trigger_source"].getMeta());
        for (int i = 0; i < sm->getLen(); ++i)
        {
            ui.comboTriggerSource->addItem(sm->getString(i), sm->getString(i));
        }

        m_firstRun = false;

        //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
        enableDialog(true);
    }

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    int *roi = params["roi"].getVal<int*>();
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
#else
    ui.rangeX01->setValues(params["x0"].getVal<int>(), params["x1"].getVal<int>());
    ui.rangeY01->setValues(params["y0"].getVal<int>(), params["y1"].getVal<int>());
    ui.rangeX01->setEnabled(! (params["x0"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["y0"].getFlags() & ito::ParamBase::Readonly));
#endif

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    ParamMapIterator it = params.find("gain");
    if (it != params.end())
    {
        ui.sliderGain->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.sliderGain->setMinimum(it->getMin());
        ui.sliderGain->setMaximum(it->getMax());
        ui.sliderGain->setValue(it->getVal<double>());
    }
    else
    {
        ui.label_gain->setVisible(false);
        ui.sliderGain->setVisible(false);
    }

    it = params.find("gain_auto");
    if (it != params.end())
    {
        ui.checkGainAuto->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.checkGainAuto->setChecked(it->getVal<int>() > 0);
        ui.sliderGain->setEnabled(ui.sliderGain->isEnabled() && (it->getVal<int>() == 0));
    }
    else
    {
        ui.checkGainAuto->setVisible(false);
    }

    it = params.find("offset");
    if (it != params.end())
    {
        ui.sliderOffset->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.sliderOffset->setMinimum(it->getMin());
        ui.sliderOffset->setMaximum(it->getMax());
        ui.sliderOffset->setValue(it->getVal<double>());
    }
    else
    {
        ui.label_offset->setVisible(false);
        ui.sliderOffset->setVisible(false);
    }

    it = params.find("gamma");
    if (it != params.end())
    {
        ui.sliderGamma->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.sliderGamma->setMinimum(it->getMin());
        ui.sliderGamma->setMaximum(it->getMax());
        ui.sliderGamma->setValue(it->getVal<double>());
    }
    else
    {
        ui.label_gamma->setVisible(false);
        ui.sliderGamma->setVisible(false);
    }

    it = params.find("integration_time");
    if (it != params.end())
    {
        ui.sliderIntegrationTime->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.sliderIntegrationTime->setMinimum(it->getMin());
        ui.sliderIntegrationTime->setMaximum(it->getMax());
        ui.sliderIntegrationTime->setValue(it->getVal<double>());
    }
    else
    {
        ui.label_integration_time->setVisible(false);
        ui.sliderIntegrationTime->setVisible(false);
    }

    it = params.find("timeout");
    ui.spinTimeout->setMinimum(it->getMin());
    ui.spinTimeout->setMaximum(it->getMax());
    ui.spinTimeout->setValue(it->getVal<double>());

    it = params.find("bpp");
    if (it != params.end())
    {
        ui.comboBppMode->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        for (int i = 0; i < ui.comboBppMode->count(); ++i)
        {
            if (ui.comboBppMode->itemData(i).toInt() == it->getVal<int>())
            {
                ui.comboBppMode->setCurrentIndex(i);
                break;
            }
        }
    }

    it = params.find("binning");
    if (it != params.end())
    {
        int binV = it->getVal<int>() % 100;
        int binH = (it->getVal<int>() - binV) / 100;

        ui.comboBinHor->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        for (int i = 0; i < ui.comboBinHor->count(); ++i)
        {
            if (ui.comboBinHor->itemData(i).toInt() == binH)
            {
                ui.comboBinHor->setCurrentIndex(i);
                break;
            }
        }

        ui.comboBinVer->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        for (int i = 0; i < ui.comboBinVer->count(); ++i)
        {
            if (ui.comboBinVer->itemData(i).toInt() == binH)
            {
                ui.comboBinVer->setCurrentIndex(i);
                break;
            }
        }
    }

    it = params.find("trigger_mode");
    if (it != params.end())
    {
        ui.checkTriggerMode->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.checkTriggerMode->setChecked(it->getVal<int>() > 0);
    }

    it = params.find("trigger_source");
    if (it != params.end())
    {
        ui.comboTriggerSource->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        for (int i = 0; i < ui.comboTriggerSource->count(); ++i)
        {
            if (ui.comboTriggerSource->itemData(i).toString() == it->getVal<char*>())
            {
                ui.comboTriggerSource->setCurrentIndex(i);
                break;
            }
        }
    }

    it = params.find("trigger_activation");
    if (it != params.end())
    {
        ui.comboTriggerActivation->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        for (int i = 0; i < ui.comboTriggerActivation->count(); ++i)
        {
            if (ui.comboTriggerActivation->itemData(i).toString() == it->getVal<char*>())
            {
                ui.comboTriggerActivation->setCurrentIndex(i);
                break;
            }
        }
    }

    it = params.find("stream_bps");
    if (it != params.end())
    {
        ui.spinStreamBpS->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.spinStreamBpS->setMinimum(it->getMin());
        ui.spinStreamBpS->setMaximum(it->getMax());
        ui.spinStreamBpS->setValue(it->getVal<int>());
    }

    it = params.find("packet_size");
    if (it != params.end())
    {
        ui.spinPacketSize->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
        ui.spinPacketSize->setMinimum(it->getMin());
        ui.spinPacketSize->setMaximum(it->getMax());
        ui.spinPacketSize->setValue(it->getVal<int>());
    }

    //save the currently set parameters to m_currentParameters
    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogAvtVimba::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    int newBpp = ui.comboBppMode->itemData(ui.comboBppMode->currentIndex()).toInt();
    if (newBpp != m_currentParameters["bpp"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, newBpp)));
    }

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    if (ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
    {
        int x0, x1, y0, y1;
        ui.rangeX01->values(x0,x1);
        ui.rangeY01->values(y0,y1);
        int roi[] = {0,0,0,0};
        memcpy(roi, m_currentParameters["roi"].getVal<int*>(), 4*sizeof(int));

        if (roi[0] != x0 || roi[1] != y0 || roi[2] != (x1-x0+1) || roi[3] != (y1-y0+1))
        {
            roi[0] = x0;
            roi[1] = y0;
            roi[2] = x1-x0+1;
            roi[3] = y1-y0+1;
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, roi)));
        }
    }
#else
    if (ui.rangeX01->isEnabled())
    {
        int x0;
        int x1;
        ui.rangeX01->values(x0,x1);

        if ((m_currentParameters["x0"].getVal<int>() !=  x0))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, x0)));
        }
        if ((m_currentParameters["x1"].getVal<int>() !=  x1))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, x1)));
        }
    }

    if (ui.rangeY01->isEnabled())
    {
        int y0;
        int y1;
        ui.rangeY01->values(y0, y1);

        if ((m_currentParameters["y0"].getVal<int>() !=  y0))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, y0)));
        }
        if ((m_currentParameters["y1"].getVal<int>() !=  y1))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, y1)));
        }
    }
#endif

    int newBinH = ui.comboBinHor->isEnabled() ? ui.comboBinHor->itemData(ui.comboBinHor->currentIndex()).toInt() : 1;
    int newBinV = ui.comboBinHor->isEnabled() ? ui.comboBinVer->itemData(ui.comboBinVer->currentIndex()).toInt() : 1;
    if ((newBinH*100+newBinV) != m_currentParameters["binning"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, (newBinH*100+newBinV))));
    }

    if (m_currentParameters["trigger_mode"].getVal<int>() != (ui.checkTriggerMode->isChecked() ? 1 : 0))
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::Int, (ui.checkTriggerMode->isChecked() ? 1 : 0))));
    }

    QString newTriggerSource = ui.comboTriggerSource->itemData(ui.comboTriggerSource->currentIndex()).toString();
    if (newTriggerSource != m_currentParameters["trigger_source"].getVal<char*>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_source", ito::ParamBase::String, newTriggerSource.toLatin1().data())));
    }

    QString newTriggerActivation = ui.comboTriggerActivation->itemData(ui.comboTriggerActivation->currentIndex()).toString();
    if (newTriggerActivation != m_currentParameters["trigger_activation"].getVal<char*>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_activation", ito::ParamBase::String, newTriggerActivation.toLatin1().data())));
    }

    if (dblEq(m_currentParameters["offset"].getVal<double>(), ui.sliderOffset->value()) == 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, ui.sliderOffset->value())));
    }

    if (dblEq(m_currentParameters["gamma"].getVal<double>(), ui.sliderGamma->value()) == 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gamma", ito::ParamBase::Double, ui.sliderGamma->value())));
    }

    if (m_currentParameters["gain_auto"].getVal<int>() != (ui.checkGainAuto->isChecked() ? 1 : 0))
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain_auto", ito::ParamBase::Int, (ui.checkGainAuto->isChecked() ? 1 : 0))));
    }

    if (ui.checkGainAuto->isChecked() == false && dblEq(m_currentParameters["gain"].getVal<double>(), ui.sliderGain->value()) == 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, ui.sliderGain->value())));
    }

    if (dblEq(m_currentParameters["integration_time"].getVal<double>(), ui.sliderIntegrationTime->value()) == 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, ui.sliderIntegrationTime->value())));
    }

    if (dblEq(m_currentParameters["timeout"].getVal<double>(), ui.spinTimeout->value()) == 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, ui.spinTimeout->value())));
    }

    if (ui.groupGigE->isVisible() && m_currentParameters.contains("stream_bps") && m_currentParameters.contains("packet_size"))
    {
        if (m_currentParameters["stream_bps"].getVal<int>() != ui.spinStreamBpS->value())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stream_bps", ito::ParamBase::Int, ui.spinStreamBpS->value())));
        }

        if (m_currentParameters["packet_size"].getVal<int>() != ui.spinPacketSize->value())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("packet_size", ito::ParamBase::Int, ui.spinPacketSize->value())));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogAvtVimba::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogAvtVimba::enableDialog(bool enabled)
{
    //e.g.
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupTrigger->setEnabled(enabled);
    ui.groupGigE->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogAvtVimba::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogAvtVimba::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//------------------------------------------------------------------------------
void DialogAvtVimba::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}
