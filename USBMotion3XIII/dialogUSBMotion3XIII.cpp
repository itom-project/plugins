/* ********************************************************************
    Plugin "USBMotion3XIII" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

/**\file dialogUSBMotion3XIII.cpp
* \brief In this file the functions of the modal dialog for the DummyMotor are specified
*
*    This file defines the functions of the dialogDummyMotor-Class defined in the file "dialogDummyMotor.h"
*
*/

#include "dialogUSBMotion3XIII.h"

//----------------------------------------------------------------------------------------------------------------------------------
/** @detail This function changes the values of the different GUI-elements according to the input paramVals
*
*\param[in] motor    A handle to the motor attached to this dialog
*\param[in] axisnums The number of axis this attached motor offer
*
*\sa DummyMotor
*/
DialogUSBMotion3XIII::DialogUSBMotion3XIII(int uniqueID)
{
    ui.setupUi(this);

    ui.lblID->setText(QString::number(uniqueID));
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @detail This function changes the values of the different GUI-elements according to the input paramVals
*
*\param[in] paramVals    Parameterlist with Motorparamters (m_params)
*\warning If the Keywords (parameters) "speed" and "accel" do not exist in the Parameterlist and the find is not used , this will crash!!
*\sa DummyMotor
*/
int DialogUSBMotion3XIII::setVals(QMap<QString, ito::Param> *paramVals)
{
    double steps;
    int microSteps;
    char *temp = NULL;

    setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    // added by itobiege, Mar. 2013, but not tested!

    temp = (*paramVals)["name"].getVal<char*>(); //borrowed reference
    ui.lblName->setText( temp );

    if( (*paramVals)["connected"].getVal<int>() )
    {
        ui.lblConnected->setText( "connected" );
        ui.tab_motor1->setEnabled(true);
        ui.tab_motor2->setEnabled(true);
        ui.tab_motor3->setEnabled(true);
        ui.groupRunMode->setEnabled(true);
        ui.tabProperties->setCurrentWidget( ui.tab_general );
    }
    else
    {
        ui.lblConnected->setText( "disconnected" );
        ui.tab_motor1->setEnabled(false);
        ui.tab_motor2->setEnabled(false);
        ui.tab_motor3->setEnabled(false);
        ui.groupRunMode->setEnabled(false);
        ui.tabProperties->setCurrentWidget( ui.tab_general );
    }

    temp = (*paramVals)["serialNumber"].getVal<char*>();
    ui.lblSerialNumber->setText( temp );
    temp = (*paramVals)["productVersion"].getVal<char*>();
    ui.lblProductVersion->setText( temp );
    temp = (*paramVals)["vendorName"].getVal<char*>(); //borrowed reference
    ui.lblVendorName->setText( temp );
    temp = (*paramVals)["productName"].getVal<char*>(); //borrowed reference
    ui.lblProductName->setText( temp );

    ui.comboRunMode->setCurrentIndex( (*paramVals)["async"].getVal<int>() > 0 ? 1 : 0 );

    //Motor 1
    steps = (*paramVals)["axisSteps1"].getVal<double>();

    ui.lblM1AxisSteps->setText( QString::number(steps) );
    ui.lblM1Available->setText( steps <= 0 ? "No" : "Yes" );
    ui.checkM1Enabled->setChecked( (*paramVals)["axisEnabled1"].getVal<int>() );
    microSteps = (*paramVals)["microSteps1"].getVal<int>();

    for(int i=0;i<7;i++)
    {
        if( (1 << i) == microSteps)
        {
            ui.comboM1MicroSteps->setCurrentIndex( i );
            break;
        }
    }

    ui.spinM1AMax->setMaximum( (*paramVals)["aMax1"].getMax() );
    ui.spinM1AMax->setMinimum( (*paramVals)["aMax1"].getMin() );
    ui.spinM1AMax->setValue( (*paramVals)["aMax1"].getVal<double>() );
    ui.spinM1VMax->setMaximum( (*paramVals)["vMax1"].getMax() );
    ui.spinM1VMax->setMinimum( (*paramVals)["vMax1"].getMin() );
    ui.spinM1VMax->setValue( (*paramVals)["vMax1"].getVal<double>() );
    ui.spinM1VMin->setMaximum( (*paramVals)["vMin1"].getMax() );
    ui.spinM1VMin->setMinimum( (*paramVals)["vMin1"].getMin() );
    ui.spinM1VMin->setValue( (*paramVals)["vMin1"].getVal<double>() );
    ui.spinM1CoilThreshold->setMaximum( (*paramVals)["coilCurrentThreshold1"].getMax() );
    ui.spinM1CoilThreshold->setMinimum( (*paramVals)["coilCurrentThreshold1"].getMin() );
    ui.spinM1CoilThreshold->setValue( (*paramVals)["coilCurrentThreshold1"].getVal<double>() );

    ui.comboM1CoilHigh->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentHigh1"].getVal<double>() ));
    ui.comboM1CoilLow->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentLow1"].getVal<double>() ));
    ui.comboM1CoilRest->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentRest1"].getVal<double>() ));

    ui.groupM1CoilCurrent->setEnabled( steps > 0);
    ui.groupM1Info->setEnabled( steps > 0);
    ui.groupM1VelAcc->setEnabled( steps > 0);

    //Motor 2
    steps = (*paramVals)["axisSteps2"].getVal<double>();

    ui.lblM2AxisSteps->setText( QString::number(steps) );
    ui.lblM2Available->setText( steps <= 0 ? "No" : "Yes" );
    ui.checkM2Enabled->setChecked( (*paramVals)["axisEnabled2"].getVal<int>() );
    microSteps = (*paramVals)["microSteps2"].getVal<int>();

    for(int i=0;i<7;i++)
    {
        if( (2 << i) == microSteps)
        {
            ui.comboM2MicroSteps->setCurrentIndex( i );
            break;
        }
    }

    ui.spinM2AMax->setMaximum( (*paramVals)["aMax2"].getMax() );
    ui.spinM2AMax->setMinimum( (*paramVals)["aMax2"].getMin() );
    ui.spinM2AMax->setValue( (*paramVals)["aMax2"].getVal<double>() );
    ui.spinM2VMax->setMaximum( (*paramVals)["vMax2"].getMax() );
    ui.spinM2VMax->setMinimum( (*paramVals)["vMax2"].getMin() );
    ui.spinM2VMax->setValue( (*paramVals)["vMax2"].getVal<double>() );
    ui.spinM2VMin->setMaximum( (*paramVals)["vMin2"].getMax() );
    ui.spinM2VMin->setMinimum( (*paramVals)["vMin2"].getMin() );
    ui.spinM2VMin->setValue( (*paramVals)["vMin2"].getVal<double>() );
    ui.spinM2CoilThreshold->setMaximum( (*paramVals)["coilCurrentThreshold2"].getMax() );
    ui.spinM2CoilThreshold->setMinimum( (*paramVals)["coilCurrentThreshold2"].getMin() );
    ui.spinM2CoilThreshold->setValue( (*paramVals)["coilCurrentThreshold2"].getVal<double>() );

    ui.comboM2CoilHigh->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentHigh2"].getVal<double>() ));
    ui.comboM2CoilLow->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentLow2"].getVal<double>() ));
    ui.comboM2CoilRest->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentRest2"].getVal<double>() ));

    ui.groupM2CoilCurrent->setEnabled( steps > 0);
    ui.groupM2Info->setEnabled( steps > 0);
    ui.groupM2VelAcc->setEnabled( steps > 0);

    //Motor 3
    steps = (*paramVals)["axisSteps3"].getVal<double>();

    ui.lblM3AxisSteps->setText( QString::number(steps) );
    ui.lblM3Available->setText( steps <= 0 ? "No" : "Yes" );
    ui.checkM3Enabled->setChecked( (*paramVals)["axisEnabled3"].getVal<int>() );
    microSteps = (*paramVals)["microSteps3"].getVal<int>();

    for(int i=0;i<7;i++)
    {
        if( (3 << i) == microSteps)
        {
            ui.comboM3MicroSteps->setCurrentIndex( i );
            break;
        }
    }

    ui.spinM3AMax->setMaximum( (*paramVals)["aMax3"].getMax() );
    ui.spinM3AMax->setMinimum( (*paramVals)["aMax3"].getMin() );
    ui.spinM3AMax->setValue( (*paramVals)["aMax3"].getVal<double>() );
    ui.spinM3VMax->setMaximum( (*paramVals)["vMax3"].getMax() );
    ui.spinM3VMax->setMinimum( (*paramVals)["vMax3"].getMin() );
    ui.spinM3VMax->setValue( (*paramVals)["vMax3"].getVal<double>() );
    ui.spinM3VMin->setMaximum( (*paramVals)["vMin3"].getMax() );
    ui.spinM3VMin->setMinimum( (*paramVals)["vMin3"].getMin() );
    ui.spinM3VMin->setValue( (*paramVals)["vMin3"].getVal<double>() );
    ui.spinM3CoilThreshold->setMaximum( (*paramVals)["coilCurrentThreshold3"].getMax() );
    ui.spinM3CoilThreshold->setMinimum( (*paramVals)["coilCurrentThreshold3"].getMin() );
    ui.spinM3CoilThreshold->setValue( (*paramVals)["coilCurrentThreshold3"].getVal<double>() );

    ui.comboM3CoilHigh->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentHigh3"].getVal<double>() ));
    ui.comboM3CoilLow->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentLow3"].getVal<double>() ));
    ui.comboM3CoilRest->setCurrentIndex( coilCurrentIndex( (*paramVals)["coilCurrentRest3"].getVal<double>() ));

    ui.groupM3CoilCurrent->setEnabled( steps > 0);
    ui.groupM3Info->setEnabled( steps > 0);
    ui.groupM3VelAcc->setEnabled( steps > 0);

    return 0;
}

int DialogUSBMotion3XIII::getRunMode() //0: synchronous, 1: async
{
    return ui.comboRunMode->currentIndex() == 0 ? 0 : 1;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogUSBMotion3XIII::getAxisValues( int axis, int &enabled, int &microSteps, double &vMin, double &vMax, double &aMax, double &coilThreshold, double &coilHigh, double &coilLow, double &coilRest)
{
    switch(axis)
    {
    case 1:
        enabled = ui.checkM1Enabled->isChecked() ? 1 : 0;
        microSteps = ui.comboM1MicroSteps->currentText().toInt();
        vMin = ui.spinM1VMin->value();
        vMax = ui.spinM1VMax->value();
        aMax = ui.spinM1AMax->value();
        coilThreshold = ui.spinM1CoilThreshold->value();
        coilHigh = ui.comboM1CoilHigh->currentText().remove("%").toDouble();
        coilLow = ui.comboM1CoilLow->currentText().remove("%").toDouble();
        coilRest = ui.comboM1CoilRest->currentText().remove("%").toDouble();
        break;

    case 2:
        enabled = ui.checkM2Enabled->isChecked() ? 1 : 0;
        microSteps = ui.comboM2MicroSteps->currentText().toInt();
        vMin = ui.spinM2VMin->value();
        vMax = ui.spinM2VMax->value();
        aMax = ui.spinM2AMax->value();
        coilThreshold = ui.spinM2CoilThreshold->value();
        coilHigh = ui.comboM2CoilHigh->currentText().remove("%").toDouble();
        coilLow = ui.comboM2CoilLow->currentText().remove("%").toDouble();
        coilRest = ui.comboM2CoilRest->currentText().remove("%").toDouble();
        break;

    case 3:
        enabled = ui.checkM3Enabled->isChecked() ? 1 : 0;
        microSteps = ui.comboM3MicroSteps->currentText().toInt();
        vMin = ui.spinM3VMin->value();
        vMax = ui.spinM3VMax->value();
        aMax = ui.spinM3AMax->value();
        coilThreshold = ui.spinM3CoilThreshold->value();
        coilHigh = ui.comboM3CoilHigh->currentText().remove("%").toDouble();
        coilLow = ui.comboM3CoilLow->currentText().remove("%").toDouble();
        coilRest = ui.comboM3CoilRest->currentText().remove("%").toDouble();
        break;

    }
}

//---------------------------------------------------------------------------------------------------------------------------
