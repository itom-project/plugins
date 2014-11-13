/* ********************************************************************
    Plugin "Newport SMC100" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2014, Institut f�r Technische Optik (ITO),
    Universit�t Stuttgart, Germany

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

#include "dialogSMC100.h"
#include "SMC100.h"


#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogSMC100::DialogSMC100(ito::AddInBase *actuator) :
    AbstractAddInConfigDialog(actuator),
    m_firstRun(true),
    m_pPlugin(actuator)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogSMC100::parametersChanged(QMap<QString, ito::Param> params)
{
////    QString temp;
//    setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
//    ui.lblDevice1->setText(params["ctrlType"].getVal<char*>());
//    //ui.lblDevice2->setText(params["ctrlName"].getVal<char*>());
//    ui.lblPiezo->setText(params["piezoName"].getVal<char*>());
//
//    bool hasMode = params["hasLocalRemote"].getVal<int>() > 0;
//    //ui.comboMode->setVisible(hasMode);
//    //ui.lblMode->setVisible(hasMode);
//
//    if (params["local"].getVal<int>() > 0)
//    {
//        //ui.comboMode->setCurrentIndex(1);
//    }
//    else
//    {
//        //ui.comboMode->setCurrentIndex(0);
//    }
//
//    ui.checkAsync->setChecked(params["async"].getVal<int>() > 0);
//    //ui.dblSpinPosLimitHigh->setValue(params["posLimitHigh"].getVal<double>() * 1000);
//    //ui.dblSpinPosLimitLow->setValue(params["posLimitLow"].getVal<double>() * 1000);
//
//    //ui.spinDelayOffset->setMinimum(0);
//    //ui.spinDelayOffset->setMaximum(params["delayOffset"].getMax() * 1000);
//    //ui.spinDelayOffset->setValue(params["delayOffset"].getVal<double>() * 1000);
//
//    //ui.spinDelayProp->setMinimum(0);
//    //ui.spinDelayProp->setMaximum(params["delayProp"].getMax() * 1000);
//    //ui.spinDelayProp->setValue(params["delayProp"].getVal<double>());
//
//    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
//    enableDialog(true);
//
//    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogSMC100::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    //QVector<QSharedPointer<ito::ParamBase> > values;
    //bool success = false;

    ////only send parameters which are changed

    ////int i = ui.comboMode->currentIndex() > 0 ? 1 : 0;
    //if (m_currentParameters["local"].getVal<int>() != i)
    //{
    //    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("local", ito::ParamBase::Int, i)));
    //}
    //
    //i = ui.checkAsync->isChecked() ? 1 : 0;
    //if (m_currentParameters["async"].getVal<int>() != i)
    //{
    //    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, i)));
    //}

    ////double v_high = ui.dblSpinPosLimitHigh->value() / 1000.0;
    ////double v_low = ui.dblSpinPosLimitLow->value() / 1000.0;
    //if (v_high < v_low)
    //{
    //    std::swap(v_high,v_low);
    //}
    //if (m_currentParameters["posLimitHigh"].getVal<double>() != v_high)
    //{
    //    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("posLimitHigh", ito::ParamBase::Double, v_high)));
    //}

    //if (m_currentParameters["posLimitLow"].getVal<double>() != v_low)
    //{
    //    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("posLimitLow", ito::ParamBase::Double, v_low)));
    //}

    //double v = ui.spinDelayOffset->value() / 1000.0;
    //if (m_currentParameters["delayOffset"].getVal<double>() != v)
    //{
    //    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("delayOffset", ito::ParamBase::Double, v)));
    //}

    //v = ui.spinDelayProp->value();
    //if (m_currentParameters["delayProp"].getVal<double>() != v)
    //{
    //    values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("delayProp", ito::ParamBase::Double, v)));
    //}

    //retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSMC100::on_buttonBox_clicked(QAbstractButton* btn)
{
    //ito::RetVal retValue(ito::retOk);

    //QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    //if (role == QDialogButtonBox::RejectRole)
    //{
    //    reject(); //close dialog with reject
    //}
    //else if (role == QDialogButtonBox::AcceptRole)
    //{
    //    accept(); //AcceptRole
    //}
    //else
    //{
    //    applyParameters(); //ApplyRole
    //}
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSMC100::on_calibrateBtn_clicked()
{
    ito::RetVal retval;
    QVector<int> axis;
    QStringList sList = ui.lineAxisToCalibrate->text().split(",", QString::SplitBehavior::SkipEmptyParts);
    QVariant v;
    ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

    // convert all string parts to int
    foreach(const QString &s, sList)
    {
        v.setValue(s);
        if (v.canConvert<int>())
        {
            axis.append(v.toInt());
        }
    }

    // Check which method should be used for calibration
    int homingType = 0;
    if (ui.radioType0->isChecked())
    {
        homingType = 0;
    }
    else if (ui.radioType1->isChecked())
    {
        homingType = 1;
    }
    else if (ui.radioType2->isChecked())
    {
        homingType = 2;
    }
    else if (ui.radioType3->isChecked())
    {
        homingType = 3;
    }
    else if (ui.radioType4->isChecked())
    {
        homingType = 4;
    }

    for (int i = 0; i < axis.size(); ++i)
    {
        if (QMetaObject::invokeMethod(m_pPlugin, "setCalibMode", Q_ARG(int, axis[i]), Q_ARG(int, homingType), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore())))
        {
            retval += observeInvocation(locker.getSemaphore(),msgLevelNo);
        }
        if (retval.containsError())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Couldn�t invoke slot of plugin"));
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.exec();
        }        
    }

    // go through and initialize them all
    for (int i = 0; i < axis.size(); ++i)
    {
        if (QMetaObject::invokeMethod(m_pPlugin, "calib", Q_ARG(int, axis[i]), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore())))
        {
            retval += observeInvocation(locker.getSemaphore(),msgLevelNo);
        }
        if (retval.containsError())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Couldn�t invoke slot of plugin"));
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.exec();
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSMC100::enableDialog(bool enabled)
{
   /* ui.group1->setEnabled(enabled);
    ui.group2->setEnabled(enabled);
    ui.group3->setEnabled(enabled);
    ui.group4->setEnabled(enabled);*/
}