/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#include "dialogMSMediaFoundation.h"
#include "MSMediaFoundation.h"

#include "common/addInInterface.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogMSMediaFoundation::DialogMSMediaFoundation(ito::AddInGrabber *grabber) :
    m_pMSMediaFoundation(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogMSMediaFoundation::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.spinSizeX->setEnabled(false);  // readonly
        ui.spinSizeY->setEnabled(false);  // readonly

        ito::StringMeta* cm = (ito::StringMeta*)(params["colorMode"].getMeta());
        for (int x = 0; x < cm->getLen(); x++)
        {
            ui.comboColorMode->addItem(cm->getString());
        }
        m_firstRun = false;
    }

//    ui.comboColorMode->setcurrenttecurrentText((params["colorMode"].getVal<char*>()));

    ui.spinX0->setValue(params["x0"].getVal<int>());
    ui.spinY0->setValue(params["y0"].getVal<int>());
    ui.spinX1->setValue(params["x1"].getVal<int>());
    ui.spinY1->setValue(params["y1"].getVal<int>());
    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_actualParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogMSMediaFoundation::checkParameters()
{
    ito::RetVal retValue(ito::retOk);
/*    if (ui.dblSpinPosLimitHigh->value() < ui.dblSpinPosLimitLow->value())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("the upper position limit must be higher than the lower one").toLatin1().data());
    }*/

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogMSMediaFoundation::sendParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    //only send parameters which are changed

/*    double v = ui.comboMode->currentIndex() > 0 ? 1.0 : 0.0;
    if (m_actualParameters["local"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("local", ito::ParamBase::Int, v) ) );
    }
    
    v = ui.checkAsync->isChecked() ? 1.0 : 0.0;
    if (m_actualParameters["async"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("async", ito::ParamBase::Int, v) ) );
    }

    v = ui.dblSpinPosLimitHigh->value() / 1000.0;
    if (m_actualParameters["posLimitHigh"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("posLimitHigh", ito::ParamBase::Double, v) ) );
    }

    v = ui.dblSpinPosLimitLow->value() / 1000.0;
    if (m_actualParameters["posLimitLow"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("posLimitLow", ito::ParamBase::Double, v) ) );
    }

    v = ui.spinDelayOffset->value() / 1000.0;
    if (m_actualParameters["delayOffset"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("delayOffset", ito::ParamBase::Double, v) ) );
    }

    v = ui.spinDelayProp->value();
    if (m_actualParameters["delayProp"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("delayProp", ito::ParamBase::Double, v) ) );
    }

    if (m_pPIPiezo)
    {
        if (values.size() > 0)
        {
            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(m_pPIPiezo, "setParamVector", Q_ARG( const QVector< QSharedPointer<ito::ParamBase> >, values), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

            while(!success)
            {
                if (locker.getSemaphore()->wait(PLUGINWAIT) == true)
                {
                    success = true;
                }
                if (!m_pPIPiezo->isAlive())
                {
                    break;
                }
            }

            if (!success)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("timeout while setting parameters of plugin.").toLatin1().data());
            }
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("plugin instance not defined.").toLatin1().data());
    }*/

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogMSMediaFoundation::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    else //ApplyRole or AcceptRole
    {
        retValue += checkParameters();
        if (retValue.containsError())
        {
            QMessageBox msgBox(this);
            QString text = tr("Invalid parameter input");
            msgBox.setText( text );
            msgBox.setIcon(QMessageBox::Critical);
            if (retValue.errorMessage()) //if no error message indicates, this is NULL
            {
                msgBox.setInformativeText( retValue.errorMessage() );
            }

            msgBox.exec();
        }
        else
        {
            retValue += sendParameters();

            if (retValue.containsError())
            {
                QMessageBox msgBox(this);
                QString text = tr("Error while setting parameters of plugin.");
                msgBox.setText( text );
                msgBox.setIcon(QMessageBox::Critical);
                if (retValue.errorMessage()) //if no error message indicates, this is NULL
                {
                    msgBox.setInformativeText( retValue.errorMessage() );
                }

                msgBox.exec();
            }
            else if (role == QDialogButtonBox::AcceptRole)
            {
                accept(); //close dialog with accept
            }
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogMSMediaFoundation::enableDialog(bool enabled)
{
    ui.groupBox->setEnabled(enabled);
    ui.groupBox_3->setEnabled(enabled);
}