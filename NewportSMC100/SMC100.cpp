/* ********************************************************************
    Plugin "Newport SMC100" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2014, Institut für Technische Optik (ITO),
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "SMC100.h"

#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qtimer.h>
#include <qwaitcondition.h>
#include <qdatetime.h>

#define SMC_READTIMEOUT 256




//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//! 
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
    creates new instance of dialogSMC100, calls the method setVals of dialogSMC100, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogSMC100
*/
const ito::RetVal SMC100::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogSMC100(this));
}




//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the SMC100::init. The widged window is created at this position.
*/
SMC100::SMC100() :
    AddInActuator(),
    m_pSer(NULL),
    m_delayAfterSendCommandMS(0),
    m_getStatusInScan(true),
    m_getPosInScan(true),
    m_useOnTarget(true)
{
    ito::Param paramVal("name", ito::ParamBase::String, "SMC100", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    
    m_scale = 1e3; // SMC is programmed in µm, this evil Programm sents in mm
    m_async = 0;
    m_delayProp = 4; //s
    m_delayOffset = 0.08; //s
    m_hasHardwarePositionLimit = false;
    m_posLimitLow = -std::numeric_limits<double>::max();
    m_posLimitHigh = std::numeric_limits<double>::max();
    m_numAxis = 1;

    paramVal = ito::Param("ctrlType", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Current type of controller, e.g. E-662, E-665, E-753...").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("ctrlName", ito::ParamBase::String | ito::ParamBase::Readonly, "unknwon", tr("device information string").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("piezoName", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("piezo information string").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("posLimitLow", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), m_posLimitLow, tr("lower position limit [mm] of piezo (this can be supported by the device or by this plugin)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("posLimitHigh", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), m_posLimitHigh, tr("higher position limit [mm] of piezo (this can be supported by the device or by this plugin)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("delayProp", ito::ParamBase::Double, 0.0, 10.0, m_delayProp, tr("delay [s] per step size [mm] (e.g. value of 1 means that a delay of 100ms is set for a step of 100mu)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("delayOffset", ito::ParamBase::Double, 0.0, 10.0, m_delayOffset, tr("offset delay [s] per movement (independent on step size)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("local", ito::ParamBase::Int, 0, 1, 0, tr("defines whether system is in local (1) or remote (0) mode.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or sychronous (0) mode").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1, 1, tr("Number of axes (here always 1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("SMC_CMD", ito::ParamBase::String, "", tr("use this parameter followed by :YourCommand in order to read/write value from/to device (e.g. SMC_CMD:ERR?)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("hasLocalRemote", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 1, tr("defines whether the device has the ability to switch between local and remote control (1), else (0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("hasOnTargetFlag", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("defines whether the device has the ability to check the 'on-target'-status (1), else (0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("checkFlags", ito::ParamBase::Int, 0, 7, 5, tr("Check flags (or-combination possible): 0x0001: check position boundaries before positioning and actualize current position after positioning (default: on), 0x0010: check for errors when positioning (default: off), 0x1000: if device has a on-target flag, it is used for checking if the device is on target (default: on), else a simple time gap is used that lets the driver sleep after positioning").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    m_targetPos = QVector<double>(1,0.0);
    m_currentStatus = QVector<int>(1, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);
    m_currentPos = QVector<double>(1,0.0);

    //now create dock widget for this plugin
    QWidget *dockWidget = new DockWidgetSMC100(getID(), this);
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params and the corresponding member variabels. 
            This function is defined by the actuator class and overwritten at this position.

    \param[in] *name        Name of parameter
    \param[out] val            New parameter value as double
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal SMC100::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    QString key;
    bool hasIndex;
    int index;
    QString additionalTag;
    QByteArray answerString;
    double answerDouble;
    QVector<QPair<int, QByteArray> > lastError;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, additionalTag);

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (hasIndex && index != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("this motor only has one axis, therefore it is not allowed to get a parameter with index unequal to 0").toLatin1().data());
    }
    else if(!retValue.containsError())
    {
        if (key == "SMC_CMD")
        {
            if (additionalTag == "")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("parameter SMC_CMD requires the real command send to the motor after a colon-sign.").toLatin1().data());
            }
            else
            {
                retValue += SMCSendQuestionWithAnswerString(additionalTag.toLatin1(), answerString, 400);
                if (retValue.containsError())
                {
                    retValue += SMCGetLastErrors(lastError);
                    retValue += convertSMCErrorsToRetVal(lastError);
                }
                else
                {
                    val->setVal<char*>(answerString.data(), answerString.length());
                }
            }
        }
        else if (key == "local")
        {
            switch(m_ctrlType)
            {
            case E662Family:
                retValue += SMCSendQuestionWithAnswerString("SYST:DEV:CONT?", answerString, 400);
                break;
            case E625Family:
                retValue += ito::RetVal(ito::retError, 3, tr("this device has no local/remote switch").toLatin1().data());
                break;
            default:
                retValue += ito::RetVal(ito::retError, 2, tr("device type is unknown").toLatin1().data());
                break;
            }

            if (retValue.containsError())
            {
                retValue += SMCGetLastErrors(lastError);
                retValue += convertSMCErrorsToRetVal(lastError);
            }
            else
            {
                if (answerString.contains("loc") || answerString.contains("LOC"))
                {
                    it->setVal<int>(1);
                }
                else
                {
                    it->setVal<int>(0);
                }
                *val = it.value();
            }
        }
        else if (key == "posLimitLow")
        {
            switch(m_ctrlType)
            {
            case E662Family:
                retValue += SMCSendQuestionWithAnswerDouble("SOUR:POS:LIM:LOW?", answerDouble, 400);
                it->setVal<double>(answerDouble/1000.0);
                break;
            case E625Family:
                break;

            default:
                retValue += ito::RetVal(ito::retError, 2, tr("device type is unknown").toLatin1().data());
                break;
            }
            if (retValue.containsError())
            {
                retValue += SMCGetLastErrors(lastError);
                retValue += convertSMCErrorsToRetVal(lastError);
            }
            else
            {
                *val = it.value();
            }
        }
        else if (key == "posLimitHigh")
        {
            switch(m_ctrlType)
            {
            case E662Family:
                retValue += SMCSendQuestionWithAnswerDouble("SOUR:POS:LIM:HIGH?", answerDouble, 400);
                it->setVal<double>(answerDouble/1000.0);
                break;
            case E625Family:
                break;
            default:
                retValue += ito::RetVal(ito::retError, 2, tr("device type is unknown").toLatin1().data());
                break;
            }
            if (retValue.containsError())
            {
                retValue += SMCGetLastErrors(lastError);
                retValue += convertSMCErrorsToRetVal(lastError);
            }
            else
            {
                *val = it.value();
            }
        }
        else
        {
            *val = it.value();
        }
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

   return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type char* with key "name" stored in m_params and the corresponding member variabels. 
            This function is defined by the actuator class and overwritten at this position.
            If the "ctrl-type" is set, SMC100::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal SMC100::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    QVector<QPair<int, QByteArray> > lastError;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if(!retValue.containsError())
    {
        if (key == "SMC_CMD")
        {
            if (suffix == "")
            {
                if (it->getType() == (*val).getType())
                {
                    char *buf = (*val).getVal<char*>();
                    if (buf != NULL)
                    {
                        suffix = QByteArray(buf);
                    }
                }

                if (suffix == "")
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("parameter SMC_CMD requires the real command send to the motor after a colon-sign in the parameter name or as value (second parameter of setParam method).").toLatin1().data());
                }
            }

            if (!retValue.containsError())
            {
                retValue += SMCSendCommand(suffix.toLatin1());
            }
        }
        else if (key == "local")
        {
            if (m_params["hasLocalRemote"].getVal<int>() > 0)
            {
                retValue += SMCSetOperationMode(val->getVal<int>() != 0);

                if (retValue.containsError())
                {
                    retValue += SMCGetLastErrors(lastError);
                    retValue += convertSMCErrorsToRetVal(lastError);
                }
                else
                {
                    retValue += it->copyValueFrom( &(*val) );
                }
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("this device does not support the local/remote control mode switch").toLatin1().data());
            }
        }
        else if (key == "posLimitLow")
        {
            switch(m_ctrlType)
            {
            case E662Family:
                retValue += SMCSendCommand(QByteArray("SOUR:POS:LIM:LOW ").append(QByteArray::number(val->getVal<double>() * 1000)));
                if (retValue.containsError())
                {
                    retValue += SMCGetLastErrors(lastError);
                    retValue += convertSMCErrorsToRetVal(lastError);
                }
                else
                {
                    retValue += it->copyValueFrom(&(*val));
                }
                break;
            default:
                retValue += it->copyValueFrom(&(*val));
                break;
            }
        }
        else if (key == "posLimitHigh")
        {
            switch(m_ctrlType)
            {
            case E662Family:
                retValue += SMCSendCommand(QByteArray("SOUR:POS:LIM:HIGH ").append(QByteArray::number(val->getVal<double>() * 1000)));
                if (retValue.containsError())
                {
                    retValue += SMCGetLastErrors(lastError);
                    retValue += convertSMCErrorsToRetVal(lastError);
                }
                else
                {
                    retValue += it->copyValueFrom(&(*val));
                }
                break;
            default:
                retValue += it->copyValueFrom(&(*val));
                break;
            }
        }
        else if(key == "checkFlags")
        {
            retValue += it->copyValueFrom(&(*val));

            m_getPosInScan = val->getVal<int>() & 1 ? true : false;
            m_getStatusInScan = val->getVal<int>() & 2 ? true : false;
            m_useOnTarget = val->getVal<int>() & 4 ? true : false;
        }
        else if (key == "delayOffset")
        {
            m_delayOffset = val->getVal<double>();
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "delayProp")
        {
            m_delayProp = val->getVal<double>();
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "async")
        {
            m_async = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }
        else
        {
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if(!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \todo check if (*paramsMand)[0] is a serial port
    \return retOk
*/
ito::RetVal SMC100::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{   
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    int keepSerialConfig = paramsOpt->at(0).getVal<int>(); //0: SMCIdentifyAndInitializeSystem will overwrite the parameters of the serial device [default], else: let it like it is!

    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
    }
    else
    {
        retval += ito::RetVal(ito::retError, 1, tr("no serialIO instance given.").toLatin1().data());
    }

    if (!retval.containsError() && keepSerialConfig == 0)
    {
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud",ito::ParamBase::Int,57000)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits",ito::ParamBase::Int,8)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity",ito::ParamBase::Double,0.0)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits",ito::ParamBase::Int,1)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow",ito::ParamBase::Int,1)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline",ito::ParamBase::String,"\r\n")),NULL);
    }

    if (!retval.containsError())
    {
        retval += this->SMC
        retval += SMCIdentifyAndInitializeSystem(keepSerialConfig);
        if (!retval.containsError())
        {
            //retval += SMCSetOperationMode(false); //already done by identify system above
            retval += SMCCheckStatus();
            retval += requestStatusAndPosition(true, false);
        }

	    m_getPosInScan = m_params["checkFlags"].getVal<int>() & 1 ? true : false;
	    m_getStatusInScan = m_params["checkFlags"].getVal<int>() & 2 ? true : false;
	    m_useOnTarget = m_params["checkFlags"].getVal<int>() & 4 ? true : false;
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the SMC100Interface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal SMC100::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    retval += SMCSetOperationMode(true); //reset to local mode

#ifdef GCS2
    if (m_deviceID >= 0)
    {
        SMC_CloseConnection(m_deviceID);
        m_deviceID = -1;
    }
#endif

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal SMC100::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1,axis),waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for a set of axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Vector this numbers of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal SMC100::calib(const QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("not implemented").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function sets the zero position of a single axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    numbers of axis to set to zero
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal SMC100::setOrigin(const int axis, ItomSharedSemaphore * /*waitCond*/)
{
    return setOrigin(QVector<int>(1,axis), NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function sets the zero position of various axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Vector with numbers of axis to set to zero
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal SMC100::setOrigin(QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("not implemented").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The SMCStatus function is called internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \todo define the status value
*/
ito::RetVal SMC100::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = SMCCheckStatus();
    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a single axis spezified by axis. The value in device independet in mm.

    \param [in] axis        Axisnumber
    \param [out] pos        Current position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal SMC100::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    double axpos = 0.0;
    
    ito::RetVal retval = ito::retOk;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toLatin1().data());
    }
    else
    {
        retval += this->SMCSendQuestionWithAnswerDouble(m_PosQust, axpos, 200);
        *pos = (double)axpos / 1000;
        m_currentPos[0] = *pos;
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a set of axis spezified by "axis". The value in device independet in mm. 
            In this case if more than one axis is specified this function returns an error.

    \param [in] axis        Vector with axis numbers
    \param [out] pos        Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal SMC100::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);

    if ((axis.size() == 1) && (axis.value(0) == 0))
    {
        retval +=getPos(axis.value(0),sharedpos,0);
        (*pos)[0] = *sharedpos;
        m_currentPos[0] = *sharedpos;
    }
    else
    {
        retval +=ito::RetVal(ito::retError, 0, tr("Error. Too many Axis / wrong Axis").toLatin1().data());
    }
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
            This function calls SMC100::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal SMC100::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    double target_posMM = pos;
    retval = SMCSetPos(axis, target_posMM, false, waitCond);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls SMC100::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal SMC100::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    if (axis.size() > 1)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Too many axis. This is currently a single axis device").toLatin1().data());

        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
    }
    else
    {
        retval += setPosAbs(axis[0], pos[0], waitCond);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
            This function calls SMC100::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal SMC100::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;

    retval = SMCSetPos(axis, pos, true, waitCond);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls SMC100::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal SMC100::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);
    
    if (axis.size() > 1)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Too many axis. This is currently a single axis device.").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
    }
    else
    {
        retval += setPosRel(axis[0], pos[0], waitCond);
    }
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal SMC100::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
    *sharedpos = 0.0;

    retval += SMCCheckStatus();

    if (sendCurrentPos)
    {
        retval += getPos(0,sharedpos,0);
        m_currentPos[0] = *sharedpos;

        if (std::abs(*sharedpos-m_targetPos[0]) > 0.01)
        {
            m_targetPos[0] = *sharedpos;
        }
        
        sendStatusUpdate(false);
    }
    else
    {
        sendStatusUpdate(true);
    }

    if (sendTargetPos)
    {
        sendTargetUpdate();
    }

    return retval;

}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
updates the status flag (if possible). For E662 only the overflow status can be checked, the rest is determined by the movement
and a guess if the motor reached the target position
*/
ito::RetVal SMC100::SMCCheckStatus(void)
{
    ito::RetVal retVal = ito::retOk;
    double answerDbl;

    if (m_ctrlType == Unknown)
    {
        return ito::RetVal(ito::retError, 0, tr("controller device unknown").toLatin1().data());
    }
    else if (m_ctrlType == E662Family)
    {
        m_identifier = QString("E662 (%1)").arg(getID());
        retVal += SMCSendQuestionWithAnswerDouble("*STB?", answerDbl, 200);
        if (!retVal.containsError())
        {
            if (answerDbl == 0)
            {
                setStatus(m_currentStatus[0],0, ito::actMovingMask | ito::actStatusMask); //reset end switch flags
            }
            else
            {
                setStatus(m_currentStatus[0], ito::actuatorEndSwitch | ito::actuatorRightEndSwitch, ito::actMovingMask | ito::actStatusMask); //set end switch (right end switch, left end switch has no signal from the motor)
            }

            setStatus(m_currentStatus[0], ito::actuatorAvailable | ito::actuatorEnabled , ito::actMovingMask | ito::actSwitchesMask);
        }
        else
        {
            setStatus(m_currentStatus[0], 0, ito::actMovingMask | ito::actSwitchesMask);
        }
    }
    else if (m_ctrlType == E625Family)
    {
        m_identifier = QString("E625 (%1)").arg(getID());
        retVal += SMCSendQuestionWithAnswerDouble("OVF? A", answerDbl, 200);

        if (!retVal.containsError())
        {
            if (answerDbl == 0)
            {
                setStatus(m_currentStatus[0],0, ito::actMovingMask | ito::actStatusMask); //reset end switch flags
            }
            else
            {
                setStatus(m_currentStatus[0], ito::actuatorEndSwitch | ito::actuatorRightEndSwitch, ito::actMovingMask | ito::actStatusMask); //set end switch (right end switch, left end switch has no signal from the motor)
            }

            setStatus(m_currentStatus[0], ito::actuatorAvailable | ito::actuatorEnabled , ito::actMovingMask | ito::actSwitchesMask);
        }
        else
        {
            setStatus(m_currentStatus[0], 0, ito::actMovingMask | ito::actSwitchesMask);
        }
    }

    return retVal;
}



//----------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------
// 

/*!
    \detail Clear serial port before writing without any delay
    \return retOk
*/
ito::RetVal SMC100::SMCDummyRead(void) /*!< reads buffer of serial port without delay in order to clear it */
{
    int bufsize = 50;
    QSharedPointer<int> len(new int);
    *len = bufsize;
    QSharedPointer<char> buffer(new char[bufsize]);
    do
    {
        m_pSer->getVal(buffer, len, NULL);
    }
    while(*len > 0);

    return ito::retOk;
}

//--------------------------------------------------------------------------------
ito::RetVal SMC100::SMCReadString(QByteArray &result, int &len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    QTime timer;
    
    bool done = false;

    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";
    int curFrom = 0;
    int pos = 0;
    QByteArray endline;
    
    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retValue += m_pSer->getParam(param, NULL);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); //borrowed reference
        int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
        endline = QByteArray::fromRawData(temp,len);
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("could not read endline parameter from serial port").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        len = 0;
        timer.start();

        while(!done && !retValue.containsError())
        {
            *curBufLen = buflen;
            retValue += m_pSer->getVal(curBuf, curBufLen, NULL);

            if (!retValue.containsError())
            {
                result += QByteArray(curBuf.data(), *curBufLen);
                pos = result.indexOf(endline, curFrom);
                curFrom = qMax(0, result.length() - 3);

                if (pos >= 0) //found
                {
                    done = true;
                    result = result.left(pos);   
                }
            }

            if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
            {
                retValue += ito::RetVal(ito::retError, SMC_READTIMEOUT, tr("timeout").toLatin1().data());
            }
        }

        len = result.length();

    }

    return retValue;
}

//--------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCGetLastErrors(QVector<QPair<int,QByteArray> > &lastErrors)
{
    bool errorAvailable = true;
    ito::RetVal retValue(ito::retOk);
    int errorNo;
    lastErrors.clear();

    QByteArray buffer;
    QByteArray errorText;
    int readSigns;
    int pos;
    bool ok;

    while(errorAvailable && !retValue.containsError())
    {
        retValue += SMCSendCommand("ERR?");
        retValue += SMCReadString(buffer, readSigns, 250);

        if (!retValue.containsError())
        {
            if (m_ctrlType == E662Family)
            {
                //buffer has form ErrorCode, "ErrorMsg"
                pos = buffer.indexOf(",");
                if (pos >= 0)
                {
                    errorNo = buffer.left(pos).toInt();
                    if (errorNo != 0)
                    {
                        errorText = buffer.mid(pos+3);
                        errorText.chop(1); //remove " from the end
                        lastErrors.append(QPair<int,QByteArray>(errorNo, errorText));
                    }
                    else
                    {
                        errorAvailable = false;
                    }
                }
                else
                {
                    lastErrors.append(QPair<int,QByteArray>(-1000, tr("error could not be parsed").toLatin1().data()));
                }
            }
            else if (m_ctrlType == E625Family)
            {
                errorNo = buffer.toInt(&ok);
                if (ok)
                {
                    if (errorNo != 0)
                    {
                        switch(errorNo)
                        {
                        case 1: errorText = tr("Parameter syntax error").toLatin1(); break;
                        case 2: errorText = tr("Unknown command").toLatin1(); break;
                        case 3: errorText = tr("Command length out of limits or command buffer overrun").toLatin1(); break;
                        case 5: errorText = tr("Unallowable move attempted on unreferenced axis, or move attempted with servo off").toLatin1(); break;
                        case 10: errorText = tr("Controller was stopped by command").toLatin1(); break;
                        case 15: errorText = tr("Invalid axis identifier").toLatin1(); break;
                        case 17: errorText = tr("Parameter out of range").toLatin1(); break;
                        case 20: errorText = tr("Macro not found").toLatin1(); break;
                        case 54: errorText = tr("Unknown parameter").toLatin1(); break;
                        case 56: errorText = tr("Password invalid").toLatin1(); break;
                        case 60: errorText = tr("Protected Param: current Command Level (CCL) too low").toLatin1(); break;
                        case 73: errorText = tr("Motion commands are not allowed when wave generator is active").toLatin1(); break;
                        case 79: errorText = tr("Open-loop commands (SVA, SVR) are not allowed when servo is on").toLatin1(); break;
                        case 89: errorText = tr("Command not allowed in current motion mode").toLatin1(); break;
                        case 210: errorText = tr("Illegal file name (must be 8-0 format)").toLatin1(); break;
                        case 232: errorText = tr("Save system configuration failed").toLatin1(); break;
                        case 233: errorText = tr("Load system configuration failed").toLatin1(); break;
                        case 306: errorText = tr("Error on I2C bus").toLatin1(); break;
                        case 309: errorText = tr("Insufficient space to store macro").toLatin1(); break;
                        case 405: errorText = tr("Wave parameter out of range").toLatin1(); break;
                        }

                        lastErrors.append(QPair<int,QByteArray>(errorNo, errorText));
                    }
                    else
                    {
                        errorAvailable = false;
                    }
                }
                else
                {
                    lastErrors.append(QPair<int,QByteArray>(-1000, tr("error could not be parsed").toLatin1().data()));
                }
            }
            else //unknown controller
            {
                errorNo = buffer.left(1).toInt();
                if (errorNo != 0)
                {
                    lastErrors.append(QPair<int,QByteArray>(-1001,tr( "unknown error").toLatin1().data()));
                }
                else
                {
                    errorAvailable = false;
                }
            }
        }
    }

    return retValue;
}

//-----------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCSendCommandInt(const QByteArray &cmd, int value, int axis /*= -1*/)
{
    ito::RetVal retVal;
    QByteArray command(cmd);

    
    if (axis > m_numAxis)
    {
        retVal += ito::RetVal(ito::retError, 0, "number of axes exceeded");
    }
    else
    {
        if (axis >= 0)
        {
            command = QByteArray::number(m_addresses[axis]) + cmd;
        }

        command += QByteArray::number(value);

        retVal += m_pSer->setVal(command.data(), command.length(), NULL);
    
        if (m_delayAfterSendCommandMS > 0)
        {
            QMutex mutex;
            mutex.lock();
            QWaitCondition waitCondition;
            waitCondition.wait(&mutex,m_delayAfterSendCommandMS);
            mutex.unlock();
        }
    }

    return retVal;
}

//-----------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCSendCommandVoid(const QByteArray &cmd, int axis /*= -1*/)
{
    ito::RetVal retVal;
    QByteArray command(cmd);
    
    if (axis > m_numAxis)
    {
        retVal += ito::RetVal(ito::retError, 0, "number of axes exceeded");
    }
    else
    {
        if (axis >= 0)
        {
            command = QByteArray::number(m_addresses[axis]) + cmd;
        }

        retVal += m_pSer->setVal(command.data(), command.length(), NULL);
    
        if (m_delayAfterSendCommandMS > 0)
        {
            QMutex mutex;
            mutex.lock();
            QWaitCondition waitCondition;
            waitCondition.wait(&mutex,m_delayAfterSendCommandMS);
            mutex.unlock();
        }
    }

    return retVal;
}

//-----------------------------------------------------------------------
/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string. 
            If string is invalid, val is not set and error-message is reported
    \param[in] *buf        Answer-String
    \param[out] val        double Value
    \sa SMC100::SMCGetDouble
    \return retOk
*/
ito::RetVal SMC100::SMCSendQuestionWithAnswerDouble(QByteArray questionCommand, double &answer, int timeoutMS)
{
    int readSigns;
    QByteArray _answer;
    bool ok;
    ito::RetVal retValue = SMCSendCommand(questionCommand);
    retValue += SMCReadString(_answer, readSigns, timeoutMS);

    answer = _answer.toDouble(&ok);

    if (retValue.containsError() && retValue.errorCode() != SMC_READTIMEOUT)
    {
        QVector< QPair<int, QByteArray> > lastErrors;
        retValue += SMCGetLastErrors(lastErrors);
        retValue += convertSMCErrorsToRetVal(lastErrors);

    }
    else if (!ok)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("value could not be parsed to a double value").toLatin1().data());
    }

    return retValue;
}

//-------------------------------------------------------------------
/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string. 
            If string is invalid, val is not set and error-message is reported
    \param[out] *buf        Answer-String
    \param[in] bufsize        Number of signs to read
    \param[out] readsigns    Number of read signs
    \return retOk
*/
ito::RetVal SMC100::SMCSendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = SMCSendCommand(questionCommand);
    retValue += SMCReadString(answer, readSigns, timeoutMS);

    if (retValue.containsError() && retValue.errorCode() == SMC_READTIMEOUT)
    {
        QVector< QPair<int, QByteArray> > lastErrors;
        retValue += SMCGetLastErrors(lastErrors);
        retValue = convertSMCErrorsToRetVal(lastErrors);
    }

    return retValue;
}

//-----------------------------------------------------------------------------
ito::RetVal SMC100::convertSMCErrorsToRetVal(QVector<QPair<int,QByteArray> > &lastErrors)
{
    if (lastErrors.size() > 0)
    {
        QByteArray errorString;
        QByteArray codeString;
        for (int i=0;i<lastErrors.size();i++)
        {
            if (errorString != "") errorString.append(", ");
            codeString = QByteArray::number(lastErrors[i].first);
            errorString.append("[").append(codeString).append("] '").append(lastErrors[i].second).append("'");
        }

        return ito::RetVal(ito::retError,0,errorString.data());
    }
    return ito::retOk;
}

//------------------------------------------------------------------------------
ito::RetVal SMC100::SMCIdentifyAndInitializeSystem(int keepSerialConfig)
{
    ito::RetVal retval = ito::retOk;
    QByteArray answer;
    QVector<QPair<int,QByteArray> > lastErrors;
    double tempDbl;

//    retval += SMCDummyRead();

#ifndef GCS2
    //set serial settings
    if (keepSerialConfig == 0)
    {
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud",ito::ParamBase::Int,9600)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits",ito::ParamBase::Int,8)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity",ito::ParamBase::Double,0.0)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits",ito::ParamBase::Int,1)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow",ito::ParamBase::Int,108)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline",ito::ParamBase::String,"\n")),NULL);
    }
#endif

    //1. try to read *idn? in order to indentify device
    retval += SMCSendQuestionWithAnswerString("*idn?", answer, 500);
    if (retval.containsError() || answer.length() < 5)
    {
        //clear error-queue
        SMCGetLastErrors(lastErrors);
        lastErrors.clear();
        m_delayAfterSendCommandMS = 2; //small delay after sendCommands (else sometimes the controller does not understand the commands)
        retval = ito::retOk;
    }

    //2. try to read *idn? in order to indentify device
    retval += SMCSendQuestionWithAnswerString("*idn?", answer, 500);
    if (retval.containsError() || answer.length() < 5)
    {
        retval = ito::retOk;
#ifndef GCS2
        if (keepSerialConfig == 0)
        {
            //try to set baudrate to 115200
            retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 115200)), NULL);
			retval += SMCDummyRead();
            retval += SMCSendQuestionWithAnswerString("*idn?", answer, 1000);
        }
#endif
        m_delayAfterSendCommandMS = 5; //small delay after sendCommands (else sometimes the controller does not understand the commands)
    }

    if (retval.containsError() || answer.length() < 5)
    {
        retval = ito::RetVal(ito::retError, 0, tr("could not identify controller. No answer for command *idn?").toLatin1().data());
        return retval;
    }
    else
    {
#ifdef GCS2
        m_params["comPort"].setVal<int>(-1);
#else
        QSharedPointer<ito::Param> param(new ito::Param("port"));
        retval += m_pSer->getParam(param, NULL);
        if (retval.containsError() || param->getVal<int>() < 0)
        {
            retval = ito::RetVal(ito::retError, 0, tr("Could not read port number from serial port or port number invalid").toLatin1().data());
            return retval;
        }
        else
        {
            m_params["comPort"].setVal<int>(param->getVal<int>()); 
        }
#endif
    }

    //clear error-queue
    SMCGetLastErrors(lastErrors);
    lastErrors.clear();

    if (answer.contains("E-662"))
    {
        m_ctrlType = E662Family;

        m_AbsPosCmd = "POS";
        m_RelPosCmd = "POS:REL";
        m_PosQust = "POS?";

        m_params["hasLocalRemote"].setVal<int>(1.0);
        m_params["hasOnTargetFlag"].setVal<int>(0.0);
        m_params["ctrlType"].setVal<char*>("E662", (int)strlen("E662"));
        m_hasHardwarePositionLimit = true;

        //set remote mode
        SMCSetOperationMode(false);

        retval += SMCSendCommand("SOUR:POS:LIM:STATE ON"); //activate limits
        retval += SMCSendQuestionWithAnswerDouble("SOUR:POS:LIM:LOW?", tempDbl, 200);
        if (!retval.containsError())
        {
            m_posLimitLow = tempDbl;
            m_params["posLimitLow"].setVal<double>(m_posLimitLow / 1000.0);
        }
        retval += SMCSendQuestionWithAnswerDouble("SOUR:POS:LIM:HIGH?", tempDbl, 200);
        if (!retval.containsError())
        {
            m_posLimitHigh = tempDbl;
            m_params["posLimitHigh"].setVal<double>(m_posLimitHigh / 1000.0);
        }

        retval += SMCSendQuestionWithAnswerString("SYST:DEV?", answer, 200);
        if (!retval.containsError())
        {
            m_params["ctrlName"].setVal<char*>(answer.data(),answer.length());
        }

        retval += SMCSendQuestionWithAnswerString("SYST:PZT?", answer, 200);
        if (!retval.containsError())
        {
            m_params["piezoName"].setVal<char*>(answer.data(),answer.length());
        }

        retval += SMCGetLastErrors(lastErrors);
        retval += convertSMCErrorsToRetVal(lastErrors);
    }
    else if (answer.contains("E816") || answer.contains("E625"))
    {
        m_ctrlType = E625Family;

        if (answer.contains("E816"))
        {
            m_params["ctrlType"].setVal<char*>("E816", (int)strlen("E816"));
            m_params["hasOnTargetFlag"].setVal<int>(1.0);
        }
        else if (answer.contains("E625"))
        {
            m_params["ctrlType"].setVal<char*>("E625", (int)strlen("E625"));
            m_params["hasOnTargetFlag"].setVal<int>(1.0);
        }

        m_AbsPosCmd = "MOV A";
        m_RelPosCmd = "MVR A";
        m_PosQust = "POS? A";

        m_params["hasLocalRemote"].setVal<int>(0.0);
        m_hasHardwarePositionLimit = false;

        //set remote mode
        SMCSetOperationMode(false);

        retval += SMCSendCommand("SVO A 1"); //activates servo
        m_params["posLimitLow"].setVal<double>(0.0 / 1000.0);
        m_params["posLimitHigh"].setVal<double>(100.0 / 1000.0);

        m_params["ctrlName"].setVal<char*>(answer.data(),answer.length());
        answer = "unknown";
        m_params["piezoName"].setVal<char*>(answer.data(),answer.length());

        retval += SMCGetLastErrors(lastErrors);
        retval += convertSMCErrorsToRetVal(lastErrors);

    }
    else if (answer.contains("E-753"))
    {
        m_ctrlType = E625Family;

        m_params["ctrlType"].setVal<char*>("E753");
        m_params["hasOnTargetFlag"].setVal<int>(1.0);

        m_AbsPosCmd = "MOV 1";
        m_RelPosCmd = "MVR 1";
        m_PosQust = "POS? 1";

        m_params["hasLocalRemote"].setVal<int>(0.0);
        m_hasHardwarePositionLimit = false;

        //set remote mode
        SMCSetOperationMode(false);

        retval += SMCSendCommand("SVO 1 1"); //activates servo
        double minmaxpos;
        retval += this->SMCSendQuestionWithAnswerDouble("TMX? 1", minmaxpos, 1000); 
        m_params["posLimitHigh"].setVal<double>(minmaxpos / 1000.0);
        retval += this->SMCSendQuestionWithAnswerDouble("TMN? 1", minmaxpos, 1000);
        m_params["posLimitLow"].setVal<double>(0.0 / 1000.0);

        m_params["ctrlName"].setVal<char*>(answer.data(),answer.length());
        answer = "unknown";
        m_params["piezoName"].setVal<char*>(answer.data(),answer.length());

        retval += SMCGetLastErrors(lastErrors);
        retval += convertSMCErrorsToRetVal(lastErrors);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is used to switch between remote and local positioning, means handwheel or pc-command. 
    \return retOk
*/
ito::RetVal SMC100::SMCSetOperationMode(bool localNotRemote)
{
    ito::RetVal retValue(ito::retOk);

    switch (m_ctrlType)
    {
    case E662Family:
        if (localNotRemote)
        {
            m_params["local"].setVal<int>(1);
            retValue += SMCSendCommand("SYST:DEV:CONT LOC");
        }
        else
        {
            m_params["local"].setVal<int>(0);
            retValue += SMCSendCommand("SYST:DEV:CONT REM");
        }
        break;
    case E625Family:
    case E753Family:
        //not supported for this types
        break;
    default:
        retValue += ito::RetVal(ito::retError, 0, tr("controller device unknown").toLatin1().data());
    }

    return retValue;
}



//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the position (abs or rel) of a one axis spezified by "axis" to the position "dpos". The value in device independet in mm. 
            If the axisnumber is not 0, this function returns an error.

    \param [in] axis        axis number
    \param [in] dpos        target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal SMC100::SMCSetPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    double dpos_temp = posMM * 1e3;    // Round value by m_scale
    ito::RetVal retval = ito::retOk;
    bool released = false;
    bool outOfRange = false;
    int delayTimeMS = 0;
    QByteArray cmdTotal;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toLatin1().data());

        if (waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
    }
    else
    {
        retval += SMCDummyRead();

        delayTimeMS = m_delayOffset /*in seconds*/ * 1000.0 + abs(posMM) * m_delayProp /*in seconds/mm*/ * 1000.0;

        if (relNotAbs)
        {
            cmdTotal = m_RelPosCmd;
            cmdTotal = cmdTotal.append(" ").append(QByteArray::number(dpos_temp, 'g'));

            if (m_hasHardwarePositionLimit == false && m_getPosInScan == true)
            {
                if (m_currentPos[0] + posMM > m_params["posLimitHigh"].getVal<double>() || m_currentPos[0] + posMM < m_params["posLimitLow"].getVal<double>())
                {
                    retval += ito::RetVal(ito::retError, 0, tr("the new position (rel) seems to be out of the allowed position range (software check only). Please check params 'posLimitHigh' and 'posLimitLow'").toLatin1().data());
                    outOfRange = true;
                }
            }
            if (outOfRange == false) m_targetPos[0] += posMM;
        }
        else
        {
            cmdTotal = m_AbsPosCmd;
            cmdTotal = cmdTotal.append(" ").append(QByteArray::number(dpos_temp, 'g'));

            if (m_hasHardwarePositionLimit == false)
            {
                if (posMM > m_params["posLimitHigh"].getVal<double>() || posMM < m_params["posLimitLow"].getVal<double>())
                {
                    retval += ito::RetVal(ito::retError, 0, tr("the new position (abs) seems to be out of the allowed position range (software check only). Please check params 'posLimitHigh' and 'posLimitLow'").toLatin1().data());
                    outOfRange = true;
                }
            }
            if (outOfRange == false) m_targetPos[0] = posMM;
        }

        if (outOfRange == false)
        {
            setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate(false);

            sendTargetUpdate();
            retval += SMCSendCommand(cmdTotal);
            retval += SMCDummyRead();
        }

        QVector< QPair<int, QByteArray> > lastErrors;
        
        if (!retval.containsError())
        {
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }

            if (outOfRange == false)
            {
                retval += waitForDone(delayTimeMS, QVector<int>(1,axis)); //WaitForAnswer(60000, axis);
            }

			if(m_getStatusInScan)
			{
				retval += SMCGetLastErrors(lastErrors);
				retval += convertSMCErrorsToRetVal(lastErrors);
			
				if (retval.containsError() && retval.errorCode() == SMC_READTIMEOUT)
				{
					retval = ito::RetVal(ito::retOk);
					retval += SMCDummyRead();
					retval += SMCGetLastErrors(lastErrors);
					retval += convertSMCErrorsToRetVal(lastErrors);
				}
			}

            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
        else
        {
            replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate(true);

            if (waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::waitForDone(const int timeoutMS, const QVector<int> /*axis*/ /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retVal(ito::retOk);
    QMutex waitMutex;
    QWaitCondition waitCondition;
    bool atTarget = false;
    double answerDbl;
    int timeoutMS_ = timeoutMS;
	ito::RetVal ontRetVal;
	int ontIterations = 10;
    QSharedPointer<double> actPos = QSharedPointer<double>(new double);

    if (m_params["hasOnTargetFlag"].getVal<int>() > 0.0 && m_useOnTarget)
    {
        while(!atTarget && !retVal.containsError())
        {
            ontRetVal = SMCSendQuestionWithAnswerDouble("ONT? A", answerDbl, 50);
            if (!ontRetVal.containsError() && answerDbl > 0.0) 
            {
                atTarget = true;
            }
			else if (ontRetVal.containsError() && ontRetVal.errorCode() != SMC_READTIMEOUT)
			{
				retVal += ontRetVal;
			}
			else if (ontIterations == 0)
			{
				retVal += ontRetVal;
				break;
			}

			--ontIterations;
        }
    }

    while(timeoutMS_ > 0 && !atTarget)
    {
        //short delay
        waitMutex.lock();
        if (timeoutMS > 1000)
        {
            waitCondition.wait(&waitMutex, 1000);
            timeoutMS_ -= 1000;
        }
        else
        {
            waitCondition.wait(&waitMutex, timeoutMS);
            timeoutMS_ = 0;
        }
        waitMutex.unlock();
        setAlive();
    }

    replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorAtTarget);

    if(m_getStatusInScan)
    {
        retVal += SMCCheckStatus();
    }

    if(m_getPosInScan)
    {
        retVal += getPos(0, actPos, NULL);
    }

    m_targetPos[0] = *actPos;
    m_currentPos[0] = *actPos;
    sendStatusUpdate(false);
    sendTargetUpdate();

    return retVal;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal SMC100::checkError(const ito::RetVal &retval)
{
    ito::RetVal r = SMCSendCommandVoid("TE",1);
    QByteArray err;
    int len;

    r += SMCReadString(err, len, 200);

    if (!r.containsError())
    {
        if (err == "@")
        {
        }
        else if (err == "A")
        {
            r += ito::RetVal(ito::retError, 0, "Unknown message code or floating point controller address.");
        }
        else if (err == "B")
        {
            r += ito::RetVal(ito::retError, 0, "Controller address not correct.");
        }
        else if (err == "C")
        {
            r += ito::RetVal(ito::retError, 0, "Parameter missing or out of range.");
        }
        else if (err == "D")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed.");
        }
        else if (err == "E")
        {
            r += ito::RetVal(ito::retError, 0, "Home sequence already started.");
        }
        else if (err == "F")
        {
            r += ito::RetVal(ito::retError, 0, "ESP stage name unknown.");
        }
        else if (err == "G")
        {
            r += ito::RetVal(ito::retError, 0, "Displacement out of limits.");
        }
        else if (err == "H")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed in NOT REFERENCED state.");
        }
        else if (err == "I")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed in CONFIGURATION state.");
        }
        else if (err == "J")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed in DISABLE state.");
        }
        else if (err == "K")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed in READY state.");
        }
        else if (err == "L")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed in HOMING state.");
        }
        else if (err == "M")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed in MOVING state.");
        }
        else if (err == "N")
        {
            r += ito::RetVal(ito::retError, 0, "Current position out of software limit.");
        }
        else if (err == "S")
        {
            r += ito::RetVal(ito::retError, 0, "Communication Time Out.");
        }
        else if (err == "U")
        {
            r += ito::RetVal(ito::retError, 0, "Error during EEPROM access.");
        }
        else if (err == "V")
        {
            r += ito::RetVal(ito::retError, 0, "Error during command execution.");
        }
        else if (err == "W")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed for PP version.");
        }
        else if (err == "X")
        {
            r += ito::RetVal(ito::retError, 0, "Command not allowed for CC version.");
        }
    }

    r += retval;
    return r;
}



//---------------------------------------------------------------------------------------------------------------------------------- 
void SMC100::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        if (visible)
        {
            QObject::connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            QObject::connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::connect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
            emit parametersChanged(m_params);
            requestStatusAndPosition(true,true);
        }
        else
        {
            QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
        }
    }
}
