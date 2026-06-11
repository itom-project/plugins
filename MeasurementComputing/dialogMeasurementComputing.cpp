/* ********************************************************************
    Plugin "USB1208LS" for itom software
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

#include "dialogMeasurementComputing.h"
//#include "MeasurementComputing.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//#include "common/abstractAddInConfigDialog.h"

//#include <qmetaobject.h>
//#include <qdialogbuttonbox.h>
//#include <qmessagebox.h>

//#include "ui_dialogMeasurementComputing.h"

//#include <qdialog.h>
//#include <qstring.h>
//#include <qmap.h>
//#include <qabstractbutton.h>
//#include <qvector.h>
//#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogMeasurementComputing::DialogMeasurementComputing(ito::AddInBase *pluginInstance) :
    AbstractAddInConfigDialog(pluginInstance),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
void DialogMeasurementComputing::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;
    ito::IntMeta *minMeta;
    ito::IntMeta *maxMeta;
    ito::IntMeta *intMeta;
    int minValue;
    int maxValue;

    if (m_firstRun)
    {
        enableDialog(true);
        //use params (identical to m_params of the plugin)
        //__________________________________________________________________________________________________________
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        // general information group box
        ui.label_devicesName->setText(params["device_name"].getVal<char*>());
        ui.label_serialNumber->setText(params["serial_number"].getVal<char*>());

        //analog input group box
        ui.label_analogInputBpp->setText(QString::number(params["analog_input_bpp"].getVal<int>()));
        ui.label_analogNumberInputs->setText(QString::number(params["analog_number_inputs"].getVal<int>()));

        ui.lineEdit_inputRangeCode->setText(params["input_range_code"].getVal<char*>());
        ui.lineEdit_inputRangeCode->setEnabled(!(params["input_range_code"].getFlags() & ito::ParamBase::Readonly));

        intMeta = static_cast<ito::IntMeta*>(params["input_samples_per_second"].getMeta());
        ui.spinBox_inputSamplesPerSecond->setMinimum(intMeta->getMin());
        ui.spinBox_inputSamplesPerSecond->setMaximum(intMeta->getMax());
        ui.spinBox_inputSamplesPerSecond->setValue(params["input_samples_per_second"].getVal<int>());
        ui.spinBox_inputSamplesPerSecond->setEnabled(!(params["input_samples_per_second"].getFlags() & ito::ParamBase::Readonly));

        intMeta  = static_cast<ito::IntMeta*>(params["samples_per_input_channel"].getMeta());
        ui.spinBox_samplesPerInputChannel->setMinimum(intMeta->getMin());
        ui.spinBox_samplesPerInputChannel->setMaximum(intMeta->getMax());
        ui.spinBox_samplesPerInputChannel->setValue(params["samples_per_input_channel"].getVal<int>());
        ui.spinBox_samplesPerInputChannel->setEnabled(!(params["samples_per_input_channel"].getFlags() & ito::ParamBase::Readonly));

        intMeta = static_cast<ito::IntMeta*>(params["clock_frequency"].getMeta());
        ui.sliderWidget_clockFrequency->setMinimum(intMeta->getMin());
        ui.sliderWidget_clockFrequency->setMaximum(intMeta->getMax());
        ui.sliderWidget_clockFrequency->setValue(params["clock_frequency"].getVal<int>());
        ui.sliderWidget_clockFrequency->setEnabled(!(params["clock_frequency"].getFlags() & ito::ParamBase::Readonly));

        minMeta = static_cast<ito::IntMeta*>(params["analog_low_input_channel"].getMeta());
        maxMeta = static_cast<ito::IntMeta*>(params["analog_high_input_channel"].getMeta());
        minValue = params["analog_low_input_channel"].getVal<int>();
        maxValue = params["analog_high_input_channel"].getVal<int>();
        ui.rangeWidget_analogInputChannels->setMinimum(minMeta->getMin());
        ui.rangeWidget_analogInputChannels->setMaximum(maxMeta->getMax());
        ui.rangeWidget_analogInputChannels->setMinimumValue(minValue);
        ui.rangeWidget_analogInputChannels->setMaximumValue(maxValue);
        ui.rangeWidget_analogInputChannels->setEnabled(true);

        if (params["analog_voltage_input"].getVal<int>() == 1)
        {
            ui.checkBox_voltageInput->setChecked(true);
        }
        else
        {
            ui.checkBox_voltageInput->setChecked(false);
        }
        ui.checkBox_voltageInput->setEnabled(!(params["analog_voltage_input"].getFlags() & ito::ParamBase::Readonly));

        //analog output groupbox
        ui.label_analogOutputBpp->setText(QString::number(params["analog_output_bpp"].getVal<int>()));
        ui.label_analogNumberOutputs->setText(QString::number(params["analog_number_outputs"].getVal<int>()));

        ui.lineEdit_outputRangeCode->setText(params["output_range_code"].getVal<char*>());
        ui.lineEdit_outputRangeCode->setEnabled(!(params["output_range_code"].getFlags() & ito::ParamBase::Readonly));

        minMeta = static_cast<ito::IntMeta*>(params["analog_low_output_channel"].getMeta());
        maxMeta = static_cast<ito::IntMeta*>(params["analog_high_output_channel"].getMeta());
        minValue = params["analog_low_output_channel"].getVal<int>();
        maxValue = params["analog_high_output_channel"].getVal<int>();
        ui.rangeWidget_analogOutputChannels->setMinimum(minMeta->getMin());
        ui.rangeWidget_analogOutputChannels->setMaximum(maxMeta->getMax());
        ui.rangeWidget_analogOutputChannels->setMinimumValue(minValue);
        ui.rangeWidget_analogOutputChannels->setMaximumValue(maxValue);
        ui.rangeWidget_analogOutputChannels->setEnabled(true);

        if (params["analog_voltage_output"].getVal<int>() == 1)
        {
            ui.checkBox_voltageOutput->setChecked(true);
        }
        else
        {
            ui.checkBox_voltageOutput->setChecked(false);
        }
        ui.checkBox_voltageOutput->setEnabled(!(params["output_range_code"].getFlags() & ito::ParamBase::Readonly));

        //digital port settings
        ui.label_digitalPortNumber->setText(QString::number(params["digital_number_ports"].getVal<int>()));

        ui.lineEdit_inputRangeCode->setText(params["input_range_code"].getVal<char*>());
        ui.lineEdit_inputRangeCode->setEnabled(!(params["input_range_code"].getFlags() & ito::ParamBase::Readonly));

        ui.lineEdit_digitalPortName->setText(params["digital_port_name"].getVal<char*>());
        ui.lineEdit_digitalPortName->setEnabled(!(params["digital_port_name"].getFlags() & ito::ParamBase::Readonly));

        if (params["digital_port_mode"].getVal<int>() == 1)
        {
            ui.comboBox_digitalPortMode->setCurrentIndex(0);
            ui.comboBox_digitalPortMode->setCurrentText("output");
        }
        else if(params["digital_port_mode"].getVal<int>() == 2)
        {
            ui.comboBox_digitalPortMode->setCurrentIndex(1);
            ui.comboBox_digitalPortMode->setCurrentText("input");
        }
        ui.comboBox_digitalPortMode->setEnabled(!(params["digital_port_mode"].getFlags() & ito::ParamBase::Readonly));

        //temperature port settings
        ui.label_temperaturePorts->setText(QString::number(params["number_temperature_channel"].getVal<int>()));

        ui.lineEdit_temperatureScaling->setText(params["temperature_scale"].getVal<char*>());
        ui.lineEdit_temperatureScaling->setEnabled(!(params["temperature_scale"].getFlags() & ito::ParamBase::Readonly));

        //counter port settings
        ui.label_counterPorts->setText(QString::number(params["number_counter_channel"].getVal<int>()));

        m_firstRun = false;
    }

    m_currentParameters = params;
}

//---------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogMeasurementComputing::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    // clock frequency
    if (ui.sliderWidget_clockFrequency->isEnabled())
    {
        int value = ui.sliderWidget_clockFrequency->value();
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("clock_frequency", ito::ParamBase::Int, value)));
    }

    // range code
    if (ui.lineEdit_inputRangeCode->isEnabled())
    {
        QString rangeCode = ui.lineEdit_inputRangeCode->text();
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("input_range_code", ito::ParamBase::String, rangeCode.toLatin1().data())));
    }

    // input samples per second
    if (ui.spinBox_inputSamplesPerSecond->isEnabled())
    {
        int value = ui.spinBox_inputSamplesPerSecond->value();
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("input_samples_per_second", ito::ParamBase::Int, value)));
    }

    // samples per input channel
    if (ui.spinBox_samplesPerInputChannel->isEnabled())
    {
        int value = ui.spinBox_samplesPerInputChannel->value();
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("samples_per_input_channel", ito::ParamBase::Int, value)));
    }

    // analog input channels
    if (ui.rangeWidget_analogInputChannels->isEnabled())
    {
        int min = ui.rangeWidget_analogInputChannels->minimumValue();
        int max = ui.rangeWidget_analogInputChannels->maximumValue();
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_low_input_channel", ito::ParamBase::Int, min)));
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_high_input_channel", ito::ParamBase::Int, max)));
    }

    //analog voltage input flag
    if (ui.checkBox_voltageInput->isEnabled())
    {
        if (ui.checkBox_voltageInput->isChecked())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_voltage_input", ito::ParamBase::Int, 1)));
        }
        else
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_voltage_input", ito::ParamBase::Int, 0)));
        }
    }

    // analog output channels
    if (ui.rangeWidget_analogOutputChannels->isEnabled())
    {
        int min = ui.rangeWidget_analogOutputChannels->minimumValue();
        int max = ui.rangeWidget_analogOutputChannels->maximumValue();
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_low_output_channel", ito::ParamBase::Int, min)));
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_high_output_channel", ito::ParamBase::Int, max)));
    }

    //analog voltage output flag
    if (ui.checkBox_voltageOutput->isEnabled())
    {
        if (ui.checkBox_voltageOutput->isChecked())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_voltage_output", ito::ParamBase::Int, 1)));
        }
        else
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("analog_voltage_output", ito::ParamBase::Int, 0)));
        }
    }

    //digital port name
    if (ui.lineEdit_digitalPortName->isEnabled())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("digital_port_name", ito::ParamBase::String, ui.lineEdit_digitalPortName->text().toLatin1().data())));
    }

    //digital port mode
    if (ui.comboBox_digitalPortMode->isEnabled())
    {
        if(ui.comboBox_digitalPortMode->currentText() == "input")
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("digital_port_mode", ito::ParamBase::Int, 2)));
        }
        else if(ui.comboBox_digitalPortMode->currentText() == "output")
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("digital_port_mode", ito::ParamBase::Int, 1)));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogMeasurementComputing::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    //cancel button, emit reject() -> dialog is closed
    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    //ok button, emit accept() -> dialog is closed
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else //apply button, only call applyParameters
    {
        applyParameters(); //ApplyRole
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogMeasurementComputing::enableDialog(bool enabled)
{
    ui.groupBox_General->setEnabled(enabled);
    ui.groupBox_AnalogInput->setEnabled(enabled);
    ui.groupBox_AnalogOutput->setEnabled(enabled);
    ui.groupBox_digitalSettings->setEnabled(enabled);
}
