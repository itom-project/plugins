/* ********************************************************************
    Plugin "LeicaMotorFocus" for itom software
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

#include "dialogLeicaMotorFocus.h"
#include "LeicaMotorFocus.h"
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogLeicaMotorFocus::DialogLeicaMotorFocus(QObject *pluginInstance) : m_pluginInstance(pluginInstance), m_unappliedChanges(false)
{
    ui.setupUi(this);
    enableDialog(true);
};

//----------------------------------------------------------------------------------------------------------------------------------
void DialogLeicaMotorFocus::enableDialog(bool enabled)
{
    ui.groupProperties->setEnabled(enabled);
    ui.groupCalibrate->setEnabled(enabled);
    ui.groupOrigin->setEnabled(enabled);
    ui.cmdOk->setEnabled(enabled);
    ui.cmdCancel->setEnabled(enabled);
    QCoreApplication::processEvents();
}


//----------------------------------------------------------------------------------------------------------------------------------
void DialogLeicaMotorFocus::parametersChanged(QMap<QString, ito::Param> params)
{
    const char* info = NULL;
    double dtemp = 0.0;
    int itemp = 0;

    setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    // added by itobiege, Mar. 2013, but not tested!

    if (params["inverseRefSwitch"].getVal<int>() == 0)
    {
        ui.radioRefUpper->setChecked(true);
    }
    else
    {
        ui.radioRefLower->setChecked(true);
    }

    ui.checkInvertAxis->setChecked( params["inverseAxis"].getVal<int>() );
    info = params["inverseAxis"].getInfo();
    if (info)
    {
        ui.checkInvertAxis->setToolTip( QString(info) );
    }

    dtemp = (params["speed"]).getVal<double>();
    ui.spinBoxSpeed->setValue(dtemp);
    dtemp = (params["speed"]).getMax();
    ui.spinBoxSpeed->setMaximum(dtemp);
    dtemp = (params["speed"]).getMin();
    ui.spinBoxSpeed->setMinimum(dtemp);
    info = params["speed"].getInfo();
    if (info)
    {
        ui.spinBoxSpeed->setToolTip( QString(info) );
    }

    itemp = (params["ratio"]).getVal<int>();
    ui.spinBoxRatio->setValue(itemp);
    itemp = (params["ratio"]).getMax();
    ui.spinBoxRatio->setMaximum(itemp);
    itemp = (params["ratio"]).getMin();
    ui.spinBoxRatio->setMinimum(itemp);
    info = params["ratio"].getInfo();
    if (info)
    {
        ui.spinBoxRatio->setToolTip( QString(info) );
    }

    m_unappliedChanges = false;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogLeicaMotorFocus::on_cmdHoming_clicked()
{
    ito::RetVal retval = ito::retOk;

    if (m_unappliedChanges)
    {
        int button = QMessageBox::question ( this, tr("unapplied changes"), tr("There are unapplied changes. You need to apply them before homing. Do you want to apply them now?"), QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Ok );
        if (button == QMessageBox::Cancel)
        {
            return;
        }

        retval += applyParameters();

        if (retval.containsError())
        {
            QMessageBox::critical( this, tr("error"), tr("error when applying parameters"));
            return;
        }
    }

    if (m_pluginInstance && !retval.containsError())
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

        enableDialog(false);
        QMetaObject::invokeMethod(m_pluginInstance,"calib",Q_ARG(int,0),Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));

        if (!locker.getSemaphore()->wait(60000))
        {
            QMessageBox::critical( this, tr("error"), tr("timeout when homing"));
        }
        else
        {
            QString msg, text;
            ito::RetVal retval = locker.getSemaphore()->returnValue;

            if (retval.containsWarningOrError())
            {
                if (retval.hasErrorMessage())
                {
                    msg = QLatin1String(retval.errorMessage());
                }
                else
                {
                    msg = tr("unknown");
                }
            }

            if (retval.containsError())
            {
                text = tr("While homing, an error occurred (%1)").arg(msg);
                QMessageBox::critical(this,tr("error when homing"),text);
            }
            else if (retval.containsWarning())
            {
                text = tr("While homing, a warning occurred (%1)").arg(msg);
                QMessageBox::warning(this,tr("warning when homing"),text);
            }
        }

        enableDialog(true);
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogLeicaMotorFocus::on_cmdOrigin_clicked()
{
    if (m_pluginInstance)
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

        enableDialog(false);
        QMetaObject::invokeMethod(m_pluginInstance,"setOrigin",Q_ARG(int,0),Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));

        if (!locker.getSemaphore()->wait(5000))
        {
            QMessageBox::critical( this, tr("error"), tr("timeout when setting origin"));
        }
        else
        {
            QString msg, text;
            ito::RetVal retval = locker.getSemaphore()->returnValue;

            if (retval.containsWarningOrError())
            {
                if (retval.hasErrorMessage())
                {
                    msg = QLatin1String(retval.errorMessage());
                }
                else
                {
                    msg = tr("unknown");
                }
            }

            if (retval.containsError())
            {
                text = tr("While setting to origin, an error occurred (%1)").arg(msg);
                QMessageBox::critical(this,tr("error when setting to origin"),text);
            }
            else if (retval.containsWarning())
            {
                text = tr("While setting to origin, a warning occurred (%1)").arg(msg);
                QMessageBox::warning(this,tr("warning when setting to origin"),text);
            }
        }

        enableDialog(true);
    }
}

//---------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogLeicaMotorFocus::applyParameters()
{
    QVector<QSharedPointer<ito::ParamBase> > outVector;
    ito::RetVal retVal = ito::retOk;

    if (m_unappliedChanges == false)
    {
        return ito::retOk;
    }

    outVector.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("speed", ito::ParamBase::Double, ui.spinBoxSpeed->value() ) ) );

    outVector.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("ratio", ito::ParamBase::Int, ui.spinBoxRatio->value() ) ) );

    if (ui.checkInvertAxis->isChecked())
    {
        outVector.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("inverseAxis", ito::ParamBase::Int, 1 ) ) );
    }
    else
    {
        outVector.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("inverseAxis", ito::ParamBase::Int, 0 ) ) );
    }

    if (ui.radioRefUpper->isChecked())
    {
        outVector.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("inverseRefSwitch", ito::ParamBase::Int, 0 ) ) );
    }
    else
    {
        outVector.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("inverseRefSwitch", ito::ParamBase::Int, 1 ) ) );
    }


    if (m_pluginInstance)   // Grabber exists
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        enableDialog(false);
        QMetaObject::invokeMethod(m_pluginInstance, "setParamVector", Q_ARG(const QVector<QSharedPointer<ito::ParamBase> >, outVector), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        if (!locker.getSemaphore()->wait(5000))
        {
            retVal +=  ito::RetVal(ito::retError,0,tr("timeout while applying parameters").toLatin1().data());
        }
        else
        {
            m_unappliedChanges = false;
            retVal += locker.getSemaphore()->returnValue;
        }

        enableDialog(true);
    }
    else
    {
        retVal += ito::RetVal(ito::retError,0,tr("no plugin available").toLatin1().data());
    }

    return retVal;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogLeicaMotorFocus::on_cmdOk_clicked()
{
    ito::RetVal retval;


    if (m_unappliedChanges)
    {
        retval += applyParameters();

        QString msg, text;

        if (retval.containsWarningOrError())
        {
            if (retval.hasErrorMessage())
            {
                msg = QLatin1String(retval.errorMessage());
            }
            else
            {
                msg = tr("unknown");
            }
        }

        if (retval.containsError())
        {
            text = tr("While applying the parameters, an error occurred (%1)").arg(msg);
            QMessageBox::critical(this,tr("error when applying parameters"),text);
            return;
        }
        else if (retval.containsWarning())
        {
            text = tr("While applying the parameters, a warning occurred (%1)").arg(msg);
            QMessageBox::warning(this,tr("warning when applying parameters"),text);
        }
    }

    this->accept();
}
