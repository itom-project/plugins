/**\file dialogDummyMotor.cpp
* \brief In this file the functions of the modal dialog for the DummyMotor are specified
*
*    This file defines the functions of the dialogDummyMotor-Class defined in the file "dialogDummyMotor.h"
* 
*\sa dialogDummyMotor, DummyMotor
*\author Wolfram Lyda
*\date    Oct2011
*/

#include "dialogDummyMotor.h"
#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @detail This function changes the values of the different GUI-elements according to the input paramVals
*
*\param[in] motor    A handle to the motor attached to this dialog
*\param[in] axisnums The number of axis this attached motor offer
*
*\sa DummyMotor
*/
dialogDummyMotor::dialogDummyMotor(ito::AddInActuator *motor, int axisnums) : m_pDummyMotor(motor), m_numaxis(axisnums)
{
    memset(m_enable,0,10*sizeof(int));
    ui.setupUi(this);
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @detail This function changes the values of the different GUI-elements according to the input paramVals
*
*\param[in] paramVals    Parameterlist with Motorparamters (m_params)
*\warning If the Keywords (parameters) "speed" and "accel" do not exist in the Parameterlist and the find is not used , this will crash!!
*\sa DummyMotor
*/
int dialogDummyMotor::setVals(QMap<QString, ito::Param> *paramVals)
{
    QVariant qvar;
    double dtemp  = 0.0;
    
    setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

    QMap<QString, ito::Param>::const_iterator paramIt = (*paramVals).constFind("speed");    // To check if this parameter exists

    if (paramIt != ((*paramVals).constEnd()))
    {
        dtemp = ((*paramVals)["speed"]).getMax(); 
        ui.doubleSpinBox_Speed->setMaximum(dtemp);
        dtemp = ((*paramVals)["speed"]).getMin(); 
        ui.doubleSpinBox_Speed->setMinimum(dtemp);
        dtemp = ((*paramVals)["speed"]).getVal<double>();   
        ui.doubleSpinBox_Speed->setValue(dtemp);
    }
    else
    {
        ui.doubleSpinBox_Speed->setEnabled(0);
    }

    paramIt = (*paramVals).constFind("accel");    // To check if this parameter exists

    if (paramIt != ((*paramVals).constEnd()))
    {
        dtemp = ((*paramVals)["accel"]).getMax(); 
        ui.doubleSpinBox_Accel->setMaximum(dtemp);
        dtemp = ((*paramVals)["accel"]).getMin(); 
        ui.doubleSpinBox_Accel->setMinimum(dtemp);
        dtemp = ((*paramVals)["accel"]).getVal<double>();   
        ui.doubleSpinBox_Accel->setValue(dtemp);
    }
    else
    {
        ui.doubleSpinBox_Accel->setEnabled(0);
    }

    if(m_numaxis>0)
    {
        ui.checkBox_EnableX->setEnabled(1);
        ui.checkBox_EnableX->setChecked(1);
    }
    if(m_numaxis>1)
    {
        ui.checkBox_EnableY->setEnabled(1);
        ui.checkBox_EnableY->setChecked(1);
    }
    if(m_numaxis>2)
    {
        ui.checkBox_EnableZ->setEnabled(1);
        ui.checkBox_EnableZ->setChecked(1);
    }
    if(m_numaxis>3)
    {
        ui.checkBox_EnableA->setEnabled(1);
        ui.checkBox_EnableA->setChecked(1);
    }
    if(m_numaxis>4)
    {
        ui.checkBox_EnableB->setEnabled(1);
        ui.checkBox_EnableB->setChecked(1);
    }
    if(m_numaxis>5)
    {
        ui.checkBox_EnableC->setEnabled(1);
        ui.checkBox_EnableC->setChecked(1);
    }
  
    // Futher axis not implementet yet

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
int dialogDummyMotor::getVals(QMap<QString, ito::Param> * /*paramVals*/)
{
    double dtemp = 0.0;
    QVariant qvar;

    if(ui.doubleSpinBox_Speed->isEnabled())    //If true than the getVal found the parameters during construction
    {
        dtemp = ui.doubleSpinBox_Speed->value();
        m_pDummyMotor->setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed",ito::ParamBase::Double, dtemp)));
    }

    if(ui.doubleSpinBox_Accel->isEnabled())
    {
        dtemp = ui.doubleSpinBox_Accel->value();
        m_pDummyMotor->setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("accel",ito::ParamBase::Double, dtemp)));
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogDummyMotor::on_pushButtonCalib_clicked()
{
    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    int i;
    QVector<int> axis;
    
    for(i=0;i<m_numaxis;i++)
    {
        if(m_enable[i])
            axis << i;
    }
    QMetaObject::invokeMethod(m_pDummyMotor,"calib",Q_ARG(QVector<int>,axis),Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));

    locker.getSemaphore()->wait(60000);

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void dialogDummyMotor::on_checkBox_EnableX_clicked()
{
    m_enable[0]=ui.checkBox_EnableX->isChecked();
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void dialogDummyMotor::on_checkBox_EnableY_clicked()
{
    m_enable[1]=ui.checkBox_EnableY->isChecked();
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
void dialogDummyMotor::on_checkBox_EnableZ_clicked()
{
    m_enable[2]=ui.checkBox_EnableZ->isChecked();
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
