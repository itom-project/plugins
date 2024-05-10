/* ********************************************************************
    Plugin "SuperlumBS" for itom software
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

#include "dialogSuperlumBS.h"
//#include "SuperlumBS.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//#include "common/addInInterface.h"

//#include <qmetaobject.h>
//#include <qdialogbuttonbox.h>
//#include <qmessagebox.h>

//#include "ui_dialogSuperlumBS.h"

//#include <qdialog.h>
//#include <qstring.h>
//#include <qmap.h>
//#include <qabstractbutton.h>
//#include <qvector.h>
//#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogSuperlumBS::DialogSuperlumBS(ito::AddInBase *motor) :
    AbstractAddInConfigDialog(motor),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
void DialogSuperlumBS::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        //use params (identical to m_params of the plugin)
        //__________________________________________________________________________________________________________ enable Widget, if Master_Key, Remote Interlock and Remote is 1
        if (params["master_key"].getVal<int>() && params["remote_interlock"].getVal<int>() && params["local"].getVal<int>())
        {
            enableDialog(true);
        }
        else
        {
            enableDialog(false);
        }
        //__________________________________________________________________________________________________________ General Information
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        ui.label_Device->setText(params["serial_number"].getVal<char*>());
        ui.label_COM_Port->setText(QString::number(params["comPort"].getVal<int>()));
        ui.label_device_name->setText(QString((params)["name"].getVal<char*>()));

        //__________________________________________________________________________________________________________ Tab MANual
        ui.label_MAN_Start_Wavelength->setText(QString::number(params["wavelength"].getMax()) + " nm");
        ui.label_MAN_Stop_Wavelength->setText(QString::number(params["wavelength"].getMin()) + " nm");

        ui.doubleSpinBox_MAN_Wavelength->setMaximum(params["wavelength"].getMax());
        ui.doubleSpinBox_MAN_Wavelength->setMinimum(params["wavelength"].getMin());
        ui.doubleSpinBox_MAN_Wavelength->setValue(params["wavelength"].getVal<double>());

        //__________________________________________________________________________________________________________ Tab AUTOmatic
        ui.doubleSpinBox_AUTO_Speed->setMaximum(params["sweep_speed"].getMax());
        ui.doubleSpinBox_AUTO_Speed->setMinimum(params["sweep_speed"].getMin());
        ui.doubleSpinBox_AUTO_Speed->setValue(params["sweep_speed"].getVal<double>());

        ui.doubleSpinBox_AUTO_Stop_Wavelength->setMinimum(params["modification_end_wavelength"].getMin());
        ui.doubleSpinBox_AUTO_Stop_Wavelength->setMaximum(params["modification_end_wavelength"].getMax());
        ui.doubleSpinBox_AUTO_Stop_Wavelength->setValue(params["modification_end_wavelength"].getVal<double>());

        ui.doubleSpinBox_AUTO_Start_Wavelength->setMinimum(params["modification_start_wavelength"].getMin());
        ui.doubleSpinBox_AUTO_Start_Wavelength->setMaximum(params["modification_start_wavelength"].getMax());
        ui.doubleSpinBox_AUTO_Start_Wavelength->setValue(params["modification_start_wavelength"].getVal<double>());

        //__________________________________________________________________________________________________________ Tab EXTernal
        ui.doubleSpinBox_EXT_Speed->setMaximum(params["sweep_speed"].getMax());
        ui.doubleSpinBox_EXT_Speed->setMinimum(params["sweep_speed"].getMin());
        ui.doubleSpinBox_EXT_Speed->setValue(params["sweep_speed"].getVal<double>());

        ui.doubleSpinBox_EXT_Stop_Wavelength->setMinimum(params["modification_end_wavelength"].getMin());
        ui.doubleSpinBox_EXT_Stop_Wavelength->setMaximum(params["modification_end_wavelength"].getMax());
        ui.doubleSpinBox_EXT_Stop_Wavelength->setValue(params["modification_end_wavelength"].getVal<double>());

        ui.doubleSpinBox_EXT_Start_Wavelength->setMinimum(params["modification_start_wavelength"].getMin());
        ui.doubleSpinBox_EXT_Start_Wavelength->setMaximum(params["modification_start_wavelength"].getMax());
        ui.doubleSpinBox_EXT_Start_Wavelength->setValue(params["modification_start_wavelength"].getVal<double>());

        //__________________________________________________________________________________________________________ Tab MODulation
        ui.label_MOD_Start_Wavelength->setText(QString::number(params["wavelength"].getMax()) + " nm");
        ui.label_MOD_Stop_Wavelength->setText(QString::number(params["wavelength"].getMin()) + " nm");

        ui.doubleSpinBox_MOD_Wavelength1->setMaximum(params["wavelength_first"].getMax());
        ui.doubleSpinBox_MOD_Wavelength1->setMinimum(params["wavelength_first"].getMin());
        ui.doubleSpinBox_MOD_Wavelength1->setValue(params["wavelength_first"].getVal<double>());

        ui.doubleSpinBox_MOD_Wavelength2->setMaximum(params["wavelength_second"].getMax());
        ui.doubleSpinBox_MOD_Wavelength2->setMinimum(params["wavelength_second"].getMin());
        ui.doubleSpinBox_MOD_Wavelength2->setValue(params["wavelength_second"].getVal<double>());

        ui.comboBox_MOD_frequency->setObjectName(QString::fromUtf8("modulation_frequency"));

        //__________________________________________________________________________________________________________ Booster settings
        if (params["operation_booster"].getVal<int>() == -1) //booster is not installed
        {
            ui.label_Booster_Enable->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Enable->setText("Not installed");

            ui.label_Booster_Emission->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Emission->setText("Emission");

            ui.label_Booster_Limit->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Limit->setText("Limit");
        }
        else if (params["operation_booster"].getVal<int>() == 0) //booster disabled
        {
            ui.label_Booster_Enable->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Enable->setText("Disabled");

            ui.label_Booster_Emission->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Emission->setText("Emission");

            ui.label_Booster_Limit->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Limit->setText("Limit");
        }
        else if (params["operation_booster"].getVal<int>() == 1) //booster enabled
        {
            ui.label_Booster_Enable->setStyleSheet("QLabel { background-color : green; color : black }");
            ui.label_Booster_Enable->setText("Enabled");

            ui.label_Booster_Emission->setStyleSheet("QLabel { color : black }");
            ui.label_Booster_Emission->setText("Emission");

            ui.label_Booster_Limit->setStyleSheet("QLabel { color : black }");
            ui.label_Booster_Limit->setText("Limit");
        }
        else
        {
            ui.label_Booster_Enable->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Enable->setText("not installed");

            ui.label_Booster_Emission->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Emission->setText("Emission");

            ui.label_Booster_Limit->setStyleSheet("QLabel { color : grey }");
            ui.label_Booster_Limit->setText("Limit");
        }

        //__________________________________________________________________________________________________________ Master Key
        if (params["master_key"].getVal<int>() == 0)
        {
            ui.label_Master_Key->setStyleSheet("QLabel { background-color : red; color : black }");
            ui.label_Master_Key->setText("Master Key is O");
        }
        else if (params["master_key"].getVal<int>() == 1)
        {
            ui.label_Master_Key->setStyleSheet("QLabel { background-color : green; color : black }");
            ui.label_Master_Key->setText("Master Key is I");
        }

        //__________________________________________________________________________________________________________ Interlock
        if (params["remote_interlock"].getVal<int>() == 1)
        {
            ui.label_Interlock_Alarm->setStyleSheet("QLabel { background-color : green; color : grey }");
            ui.label_Interlock_Alarm->setText("Interlock closed");
        }
        else if (params["remote_interlock"].getVal<int>() == 0)
        {
            ui.label_Interlock_Alarm->setStyleSheet("QLabel { background-color : red; color : black }");
            ui.label_Interlock_Alarm->setText("ALARM! Interlock opened");
        }

        //__________________________________________________________________________________________________________ Remote
        ui.comboBox_Remote->setCurrentIndex( params["local"].getVal<int>());

        //__________________________________________________________________________________________________________ Optical Output
        if (params["optical_output"].getVal<int>() == 1)
        {
            ui.btn_Optical_Output->setChecked( 1 );
        }
        else if (params["optical_output"].getVal<int>() == 0)
        {
            ui.btn_Optical_Output->setChecked( 0 );
        }

        //__________________________________________________________________________________________________________ Modulation frequency
        int mod_f = params["modulation_frequency"].getVal<double>() * 10;
        switch (mod_f)
        {
            case 1:
                ui.comboBox_MOD_frequency->setCurrentIndex( 0 );
                break;
            case 2:
                ui.comboBox_MOD_frequency->setCurrentIndex( 1 );
                break;
            case 5:
                ui.comboBox_MOD_frequency->setCurrentIndex( 2 );
                break;
            case 10:
                ui.comboBox_MOD_frequency->setCurrentIndex( 3 );
                break;
            case 20:
                ui.comboBox_MOD_frequency->setCurrentIndex( 4 );
                break;
            case 50:
                ui.comboBox_MOD_frequency->setCurrentIndex( 5 );
                break;
            case 100:
                ui.comboBox_MOD_frequency->setCurrentIndex( 6 );
                break;
            case 200:
                ui.comboBox_MOD_frequency->setCurrentIndex( 7 );
                break;
            case 500:
                ui.comboBox_MOD_frequency->setCurrentIndex( 8 );
                break;
            case 1000:
                ui.comboBox_MOD_frequency->setCurrentIndex( 9 );
                break;
            case 2000:
                ui.comboBox_MOD_frequency->setCurrentIndex( 10 );
                break;
            case 5000:
                ui.comboBox_MOD_frequency->setCurrentIndex( 11 );
                break;
            case 10000:
                ui.comboBox_MOD_frequency->setCurrentIndex( 12 );
                break;
            default:

                break;
        }

        //__________________________________________________________________________________________________________ Power Mode
        ui.comboBox_Power_Mode->setCurrentIndex( params["power_mode"].getVal<int>());

        //__________________________________________________________________________________________________________ Tab of Mode
        ui.tabWidget_Operation_Mode->setCurrentIndex( params["operation_mode"].getVal<int>() - 1 );

        m_firstRun = false;
    }

    //__________________________________________________________________________________________________________ Editing
    if (!m_inEditing)
    {
        m_inEditing = true;

        //__________________________________________________________________________________________________________ Master Key
        if (params.contains("master_key"))
        {
            if (params["master_key"].getVal<int>() == 0)
            {
                ui.label_Master_Key->setStyleSheet("QLabel { background-color : red; color : black }");
                ui.label_Master_Key->setText("Master Key is O");
                enableDialog(false);
            }
            else if (params["master_key"].getVal<int>() == 1)
            {
                ui.label_Master_Key->setStyleSheet("QLabel { background-color : green; color : black }");
                ui.label_Master_Key->setText("Master Key is I");
                enableDialog(true);
            }
        }

        //__________________________________________________________________________________________________________ Remote Interlock
        if (params.contains("remote_interlock"))
        {
            if (params["remote_interlock"].getVal<int>() == 1)
            {
                ui.label_Interlock_Alarm->setStyleSheet("QLabel { background-color : green; color : grey }");
                ui.label_Interlock_Alarm->setText("Interlock closed");
                enableDialog(true);
            }
            else if (params["remote_interlock"].getVal<int>() == 0)
            {
                ui.label_Interlock_Alarm->setStyleSheet("QLabel { background-color : red; color : black }");
                ui.label_Interlock_Alarm->setText("ALARM! Interlock opened");
                enableDialog(false);
            }
        }

        //__________________________________________________________________________________________________________ Wavelength
        if (params.contains("wavelength"))
        {
            ui.doubleSpinBox_MAN_Wavelength->setValue(params["wavelength"].getVal<double>());
        }

        //__________________________________________________________________________________________________________ Sweep Speed
        if (params.contains("sweep_speed"))
        {
            ui.doubleSpinBox_AUTO_Speed->setValue(params["sweep_speed"].getVal<double>());
        }

        //__________________________________________________________________________________________________________ modification_end_wavelength
        if (params.contains("modification_end_wavelength"))
        {
            ui.doubleSpinBox_AUTO_Stop_Wavelength->setValue(params["modification_end_wavelength"].getVal<double>());
        }

        //__________________________________________________________________________________________________________ modification_start_wavelength
        if (params.contains("modification_start_wavelength"))
        {
            ui.doubleSpinBox_AUTO_Start_Wavelength->setValue(params["modification_start_wavelength"].getVal<double>());
        }

        //__________________________________________________________________________________________________________ wavelength_first
        if (params.contains("wavelength_first"))
        {
            ui.doubleSpinBox_MOD_Wavelength1->setValue(params["wavelength_first"].getVal<double>());
        }
        //__________________________________________________________________________________________________________ wavelength_second
        if (params.contains("wavelength_second"))
        {
            ui.doubleSpinBox_MOD_Wavelength2->setValue(params["wavelength_second"].getVal<double>());
        }

        //__________________________________________________________________________________________________________ Operation Mode
        if (params.contains("operation_mode"))
        {
            ui.tabWidget_Operation_Mode->setCurrentIndex( params["operation_mode"].getVal<int>() - 1 );
        }

        //__________________________________________________________________________________________________________ Power Mode
        if (params.contains("power_mode"))
        {
            ui.comboBox_Power_Mode->setCurrentIndex( params["power_mode"].getVal<int>());
        }

        //__________________________________________________________________________________________________________ Remote
        if (params.contains("local"))
        {
            if (params["local"].getVal<int>() == 1)
            {
                enableDialog(true);
            }
            else if (params["local"].getVal<int>() == 0)
            {
                enableDialog(false);
            }
            ui.comboBox_Remote->setCurrentIndex( params["local"].getVal<int>());
        }

        //__________________________________________________________________________________________________________ Optical Output
        if (params.contains("optical_output"))
        {
            if (params["optical_output"].getVal<int>() == 1)
            {
                ui.btn_Optical_Output->setChecked( 1 );
            }
            else if (params["optical_output"].getVal<int>() == 0)
            {
                ui.btn_Optical_Output->setChecked( 0 );
            }
        }

        //__________________________________________________________________________________________________________ Booster settings
        if (params.contains("operation_booster"))
        {
            if (params["operation_booster"].getVal<int>() == -1) //booster is not installed
            {
                ui.label_Booster_Enable->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Enable->setText("Not installed");

                ui.label_Booster_Emission->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Emission->setText("Emission");

                ui.label_Booster_Limit->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Limit->setText("Limit");
            }
            else if (params["operation_booster"].getVal<int>() == 0) //booster disabled
            {
                ui.label_Booster_Enable->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Enable->setText("Disabled");

                ui.label_Booster_Emission->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Emission->setText("Emission");

                ui.label_Booster_Limit->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Limit->setText("Limit");
            }
            else if (params["operation_booster"].getVal<int>() == 1) //booster enabled
            {
                ui.label_Booster_Enable->setStyleSheet("QLabel { background-color : green; color : black }");
                ui.label_Booster_Enable->setText("Enabled");

                ui.label_Booster_Emission->setStyleSheet("QLabel { color : black }");
                ui.label_Booster_Emission->setText("Emission");

                ui.label_Booster_Limit->setStyleSheet("QLabel { color : black }");
                ui.label_Booster_Limit->setText("Limit");
            }
            else
            {
                ui.label_Booster_Enable->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Enable->setText("not installed");

                ui.label_Booster_Emission->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Emission->setText("Emission");

                ui.label_Booster_Limit->setStyleSheet("QLabel { color : grey }");
                ui.label_Booster_Limit->setText("Limit");
            }
        }

        //__________________________________________________________________________________________________________ Modulation frequency
        if (params.contains("modulation_frequency"))
        {
            int mod_f = params["modulation_frequency"].getVal<double>() * 10;
            switch (mod_f)
            {
                case 1:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 0 );
                    break;
                case 2:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 1 );
                    break;
                case 5:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 2 );
                    break;
                case 10:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 3 );
                    break;
                case 20:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 4 );
                    break;
                case 50:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 5 );
                    break;
                case 100:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 6 );
                    break;
                case 200:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 7 );
                    break;
                case 500:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 8 );
                    break;
                case 1000:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 9 );
                    break;
                case 2000:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 10 );
                    break;
                case 5000:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 11 );
                    break;
                case 10000:
                    ui.comboBox_MOD_frequency->setCurrentIndex( 12 );
                    break;
            }
        }
        m_inEditing = false;
    }

    m_currentParameters = params;
}

//---------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogSuperlumBS::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;
    int val;
    double valdouble;
    int tabval;
    int valpower;
    bool valoptical;

    //__________________________________________________________________________________________________________ Power Mode
    valpower = ui.comboBox_Power_Mode->currentIndex();
    if (m_currentParameters["power_mode"].getVal<int>() != valpower)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("power_mode", ito::ParamBase::Int, valpower)));
    }

    //__________________________________________________________________________________________________________ Operation Mode
    tabval = ui.tabWidget_Operation_Mode->currentIndex() + 1 ;
    if (m_currentParameters["operation_mode"].getVal<int>() != tabval)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("operation_mode", ito::ParamBase::Int, tabval)));
    }

    //__________________________________________________________________________________________________________ MANual Mode
    if (tabval == 1)//MAN
    {
        //__________________________________________________________________________________________________________ Wavelength
        valdouble = ui.doubleSpinBox_MAN_Wavelength->value();
        if (m_currentParameters["wavelength"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("wavelength", ito::ParamBase::Double, valdouble)));
        }
    }

    //__________________________________________________________________________________________________________AUTOmatic Mode
    else if (tabval == 2)//AUTO
    {
        //__________________________________________________________________________________________________________End wavelength
        valdouble = ui.doubleSpinBox_AUTO_Stop_Wavelength->value();
        if (m_currentParameters["modification_end_wavelength"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("modification_end_wavelength", ito::ParamBase::Double, valdouble)));
        }

        //__________________________________________________________________________________________________________Start wavelength
        valdouble = ui.doubleSpinBox_AUTO_Start_Wavelength->value();
        if (m_currentParameters["modification_start_wavelength"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("modification_start_wavelength", ito::ParamBase::Double, valdouble)));

        }

        //__________________________________________________________________________________________________________Sweep speed
        valdouble = ui.doubleSpinBox_AUTO_Speed->value();
        if (m_currentParameters["sweep_speed"].getVal<int>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sweep_speed", ito::ParamBase::Int, valdouble)));
        }
    }

    //__________________________________________________________________________________________________________EXTernal Mode
    else if (tabval == 3)//EXT
    {
        //__________________________________________________________________________________________________________End wavelength
        valdouble = ui.doubleSpinBox_EXT_Stop_Wavelength->value();
        if (m_currentParameters["modification_end_wavelength"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("modification_end_wavelength", ito::ParamBase::Double, valdouble)));
        }
        //__________________________________________________________________________________________________________Start wavelength
        valdouble = ui.doubleSpinBox_EXT_Start_Wavelength->value();
        if (m_currentParameters["modification_start_wavelength"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("modification_start_wavelength", ito::ParamBase::Double, valdouble)));
        }

        //__________________________________________________________________________________________________________Sweep speed
        valdouble = ui.doubleSpinBox_EXT_Speed->value();
        if (m_currentParameters["sweep_speed"].getVal<int>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sweep_speed", ito::ParamBase::Int, valdouble)));
        }
    }

    //__________________________________________________________________________________________________________MODulation Mode
    else if (tabval == 4)//MOD
    {
        //__________________________________________________________________________________________________________Wavelength first
        valdouble = ui.doubleSpinBox_MOD_Wavelength1->value();
        if (m_currentParameters["wavelength_first"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("wavelength_first", ito::ParamBase::Double, valdouble)));
        }
        //__________________________________________________________________________________________________________Wavelength second
        valdouble = ui.doubleSpinBox_MOD_Wavelength2->value();
        if (m_currentParameters["wavelength_second"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("wavelength_second", ito::ParamBase::Double, valdouble)));
        }
        //__________________________________________________________________________________________________________Modulation frequency
        valdouble = ui.comboBox_MOD_frequency->currentText().remove(ui.comboBox_MOD_frequency->currentText().length()-3, 3).toDouble();
        if (m_currentParameters["modulation_frequency"].getVal<double>() != valdouble)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("modulation_frequency", ito::ParamBase::Double, valdouble)));
        }
    }

    //__________________________________________________________________________________________________________Remote/ Local
    val = ui.comboBox_Remote->currentIndex();
    if (m_currentParameters["local"].getVal<int>() != val)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("local", ito::ParamBase::Int, val)));
    }

    //__________________________________________________________________________________________________________Optical Output
    valoptical = ui.btn_Optical_Output->isChecked();
    val = int(valoptical);
    if (m_currentParameters["optical_output"].getVal<int>() != val)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("optical_output", ito::ParamBase::Int, val)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSuperlumBS::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogSuperlumBS::enableDialog(bool enabled)
{
    ui.groupBox_Control->setEnabled(enabled);
    ui.groupBox_Booster->setEnabled(enabled);
    ui.tabWidget_Operation_Mode->setEnabled(enabled);
}
