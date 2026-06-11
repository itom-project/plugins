/* ********************************************************************
    Plugin "AerotechA3200" for itom software
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

/**\file dialogAerotechA3200.cpp
* \brief In this file the functions of the modal dialog for the AerotechA3200 are specified
*
*    This file defines the functions of the dialogAerotechA3200-Class defined in the file "dialogAerotechA3200.h"
*
*\sa dialogAerotechA3200, AerotechA3200
*\author Christof Pruss, Simon Chen
*\date   Apr2014
*/

#include "dialogAerotechA3200.h"
#include <qmetaobject.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @detail This function changes the values of the different GUI-elements according to the input paramVals
*
*\param[in] motor     A handle to the motor attached to this dialog
*\param[in] axisNames Names of axis this attached motor offer
*
*\sa AerotechA3200
*/
dialogAerotechA3200::dialogAerotechA3200(ito::AddInActuator *motor, QStringList axisNames) :
    m_pAerotechA3200(motor),
    m_numaxis(axisNames.count())
{
    ui.setupUi(this);

    QDoubleSpinBox* diaSpeed;
    QLabel* diaSpeedLabel;
    QCheckBox* diaEnabled;
    for (int i = 0; i < m_numaxis; ++i)
    {
        diaSpeedLabel = new QLabel(ui.groupProperties);
        diaSpeedLabel->setText(tr("Speed") + " " + axisNames[i]);
        m_pDialogSpeedLabel.append(diaSpeedLabel);
        ui.gridLayout->addWidget(diaSpeedLabel, i + 1, 0, 1, 1);

        diaSpeed = new QDoubleSpinBox(ui.groupProperties);
//        diaSpeed->setEnabled(false);
        QSizePolicy sizePolicy6(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy6.setHorizontalStretch(0);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(diaSpeed->sizePolicy().hasHeightForWidth());
        diaSpeed->setSizePolicy(sizePolicy6);
        diaSpeed->setMinimumSize(QSize(0, 0));
        diaSpeed->setMaximumSize(QSize(16777215, 16777215));
        diaSpeed->setDecimals(6);
        diaSpeed->setSingleStep(0.001);
        diaSpeed->setSuffix(" " + tr("mm/s"));
        m_pDialogSpeed.append(diaSpeed);
        ui.gridLayout->addWidget(diaSpeed, i + 1, 1, 1, 1);

        diaEnabled = new QCheckBox(ui.groupAxis);
        diaEnabled->setChecked(true);
        diaEnabled->setText(tr("Enable") + " " + axisNames[i]);
        m_pDialogEnabled.append(diaEnabled);
//        ui.gridLayout_2->addWidget(diaEnabled, (i / 2) + 1, (i % 2), 1, 1);
    }

    delete ui.labelSpeed;
    ui.labelSpeed = NULL;
    delete ui.doubleSpinBox_Speed;
    ui.doubleSpinBox_Speed = NULL;
    delete ui.checkBox_EnableA;
    ui.checkBox_EnableA = NULL;
    delete ui.checkBox_EnableX;
    ui.checkBox_EnableX = NULL;

    int newHeight = (2 * 29 * m_numaxis) + 102;
    setMinimumHeight(newHeight);
    setMaximumHeight(newHeight);

    ui.groupProperties->setMinimumHeight((29 * m_numaxis) + 26);
    ui.groupAxis->setMinimumHeight((29 * m_numaxis) + 52);
    ui.groupAxis->setGeometry(10, (29 * m_numaxis) + 41, 211, ui.groupAxis->minimumHeight());
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @detail This function changes the values of the different GUI-elements according to the input paramVals
*
*\param[in] paramVals    Parameterlist with Motorparameters (m_params)
*\warning If the Keywords (parameters) "speed" and "accel" do not exist in the Parameterlist and the find is not used , this will crash!!
*\sa AerotechA3200
*/
int dialogAerotechA3200::setVals(QMap<QString, ito::Param> *paramVals)
{
    setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

    QMap<QString, ito::Param>::const_iterator paramIt = (*paramVals).constFind("speed");    // To check if this parameter exists
    if (paramIt != ((*paramVals).constEnd()))
    {
        double *paramSpeed = ((*paramVals)["speed"].getVal<double*>()); //mm/s

        for (int i = 0; i < m_numaxis; i++)
        {
            m_pDialogSpeed[i]->setValue(paramSpeed[i]);
            m_pDialogSpeed[i]->setMinimum(0);
            m_pDialogSpeed[i]->setMaximum(1000);
        }
    }
    else
    {
        for (int i = 0; i < m_numaxis; i++)
        {
            m_pDialogSpeed[i]->setEnabled(false);
        }
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
int dialogAerotechA3200::getVals(QMap<QString, ito::Param> * /*paramVals*/)
{
    if (m_numaxis > 0 && m_pDialogSpeed[0]->isEnabled())    //If true than the getVal found the parameters during construction
    {
        double *speedArray = new double[m_numaxis];
        for (int i = 0; i < m_numaxis; i++)
        {
            speedArray[i] = m_pDialogSpeed[i]->value();
        }

        QSharedPointer<ito::ParamBase> val(new ito::ParamBase("speed", ito::ParamBase::DoubleArray, m_numaxis, speedArray));
        delete[] speedArray;

        m_pAerotechA3200->setParam(val);
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogAerotechA3200::on_pushButtonCalib_clicked()
{
    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    int i = 0;
    QVector<int> axis;

    for (i = 0; i < m_numaxis; i++)
    {
        if (m_pDialogEnabled[i]->isChecked())
        {
            axis << i;
        }
    }

    ui.cancelButton->setEnabled(false);
    ui.okButton->setEnabled(false);

    QMetaObject::invokeMethod(m_pAerotechA3200, "calib", Q_ARG(QVector<int>, axis), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

    if (locker.getSemaphore()->waitAndProcessEvents(60000))
    {
        QMessageBox::information(this, tr("Calibration (Homing)"), tr("Homing successfully executed."));
    }
    else
    {
        QMessageBox::critical(this, tr("Calibration (Homing)"), tr("Timeout while calibrating axes."));
    }

    ui.cancelButton->setEnabled(true);
    ui.okButton->setEnabled(true);


}
