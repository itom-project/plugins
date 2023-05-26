/* ********************************************************************
    Plugin "Newport SMC100" for itom software
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


#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "SMC100.h"

#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qelapsedtimer.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>

#ifdef WIN32
    #define NOMINMAX //avoid inclusion of min,max macros in windows.h
    #include <Windows.h> //for sleep command
#endif
#include <qdebug.h>

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


//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the SMC100::init. The widged window is created at this position.
*/
SMC100::SMC100() :
    AddInActuator(),
    m_pSer(NULL)
{
    endlineParam = QSharedPointer<ito::Param>(new ito::Param("endline"));

    m_async = 0;
    m_numAxis = 1;

    ito::Param paramVal;

    // Read only - Parameters
    paramVal = ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("ctrlId", ito::ParamBase::String | ito::ParamBase::Readonly, "unknwon", tr("device information string").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("current_status", ito::ParamBase::IntArray, NULL, tr("Displays the current status in the Newport system.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "SMC100", tr("Name of the plugin").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 32, 0, tr("Number of Axis").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // Read/Write - Parameters
    paramVal = ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or sychronous (0) mode").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("accel", ito::ParamBase::DoubleArray, NULL, tr("Calibration / Homing mode for each axis for further information refer to datasheet command HT").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("calib_mode", ito::ParamBase::IntArray, NULL, tr("Calibration / Homing mode for each axis for further information refer to datasheet command HT").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("config_state", ito::ParamBase::IntArray, NULL, tr("Reset Controller and switch to configmode (1) or set it back to unreferenced (0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("speed", ito::ParamBase::DoubleArray, NULL, tr("Calibration / Homing mode for each axis for further information refer to datasheet command HT").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("exec_calib", ito::ParamBase::IntArray, NULL, tr("Expects an array with the axis that should be calibrated ([1,3,4])").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("calib_timeout", ito::ParamBase::Double, 0.0, std::numeric_limits<double>::max(), 16.0, tr("timeout when calibrating axes in seconds").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    m_currentStatus = QVector<int>(1, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetSMC100 *dockWidget = new DockWidgetSMC100(getID(), this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }
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
    int numAxes = paramsMand->at(1).getVal<int>();

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
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud",ito::ParamBase::Int,57600)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits",ito::ParamBase::Int,8)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity",ito::ParamBase::Double,0.0)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits",ito::ParamBase::Int,1)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow",ito::ParamBase::Int,1)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline",ito::ParamBase::String,"\r\n")),NULL);
    }

    if (!retval.containsError())
    {

        //test the number of connected axes
        m_numAxis = 0;
        ito::RetVal retval2;
        QByteArray answer;

        QSharedPointer<QVector<ito::ParamBase> > _dummy;
        m_pSer->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, NULL);
        m_pSer->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, NULL);

        for (int i = 1; i <= 31; ++i)
        {
            //try to clear error memory
            SMCCheckError(i);

            retval2 = SMCSendQuestionWithAnswerString("ID?", answer, 100, true, i);
            if (!retval2.containsError())
            {
                m_numAxis++;
                m_addresses.append(i);
                m_ids.append(answer);
                m_controllerState.append(ctrlStNotRef);

                if (m_numAxis == 1) //main controller
                {
                    setIdentifier(answer);
                }

                if (m_numAxis == numAxes)
                {
                    break;
                }
            }
        }

        if (m_numAxis != numAxes)
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("could not connect to %i axes").toLatin1().data(), numAxes);
        }
        else
        {
            m_params["async"].setVal<int>(m_async);

            int *acceleration = new int[m_numAxis];
            memset(acceleration, 0, m_numAxis * sizeof(int));
            m_params["accel"].setVal<int*>(acceleration, m_numAxis);
            delete[] acceleration;
            acceleration = NULL;

            int *calibMode = new int[m_numAxis];
            memset(calibMode, 0, m_numAxis * sizeof(int));
            m_params["calib_mode"].setVal<int*>(calibMode, m_numAxis);
            delete[] calibMode;
            calibMode = NULL;

            int *configMode = new int[m_numAxis];
            memset(configMode, 0, m_numAxis * sizeof(int));
            m_params["config_state"].setVal<int*>(configMode, m_numAxis);
            delete[] configMode;
            configMode = NULL;

            int *currentStatus = new int[m_numAxis];
            memset(currentStatus, 0, m_numAxis * sizeof(int));
            m_params["current_status"].setVal<int*>(currentStatus, m_numAxis);
            delete[] currentStatus;
            currentStatus = NULL;

            int *ids = new int[m_numAxis];
            memset(ids, 0, m_numAxis * sizeof(int));
            m_params["ctrlId"].setVal<int*>(ids, m_numAxis);
            delete[] ids;
            ids = NULL;

            int *velocity = new int[m_numAxis];
            memset(velocity, 0, m_numAxis * sizeof(int));
            m_params["speed"].setVal<int*>(velocity, m_numAxis);
            delete[] velocity;
            velocity = NULL;

            m_params["numaxis"].setVal<int>(m_numAxis);

            m_calibMode = QVector<int>(m_numAxis, 0);
            m_targetPos = QVector<double>(m_numAxis, 0.0);
            m_currentStatus = QVector<int>(m_numAxis, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);
            m_currentPos = QVector<double>(m_numAxis, 0.0);
            m_velocity = QVector<double>(m_numAxis, 0.0);
            m_acceleration = QVector<double>(m_numAxis, 0.0);

            QVector<int> allAxis;
            for (int i = 0; i < m_numAxis; ++i)
            {
                allAxis.append(i);
            }
            SMCCheckStatus(allAxis);
        }
    }

    if (!retval.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    getParam(QSharedPointer<ito::Param>(new ito::Param("async")));
    getParam(QSharedPointer<ito::Param>(new ito::Param("comPort")));
    getParam(QSharedPointer<ito::Param>(new ito::Param("accel")));
    getParam(QSharedPointer<ito::Param>(new ito::Param("calib_mode")));
    getParam(QSharedPointer<ito::Param>(new ito::Param("config_state")));
    getParam(QSharedPointer<ito::Param>(new ito::Param("current_status")));
    getParam(QSharedPointer<ito::Param>(new ito::Param("speed")));
    QVector<int> allAxis;
    for (int i = 0; i < m_numAxis; ++i)
    {
        allAxis.append(i);
    }
    SMCCheckStatus(allAxis);
    return retval;
}


const ito::RetVal SMC100::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogSMC100(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCResetController(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    retval += SMCCheckStatus(axis);

    if (!retval.containsError())
    {
        // Reset the given controller
        foreach(const int &a, axis)
        {
            if (m_controllerState.at(a) != ctrlStNotRef && m_controllerState.at(a) != ctrlStConfig)
            {
                SMCSendCommand("RS", false, m_addresses[a]);
                qDebug() << "Reset" << a;
            }
        }
        // Wait till all controller are ready
        foreach(const int &a, axis)
        {
            if (m_controllerState.at(a) != ctrlStNotRef && m_controllerState.at(a) != ctrlStConfig)
            {
                while (true)
                {
                    SMCCheckStatus(axis);
                    this->setAlive();
                    if (m_controllerState.at(a) == ctrlStNotRef)
                    {
                        qDebug() << "reset" << a << "done";
                        break;
                    }
                }
            }
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCEnterConfigMode(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    QElapsedTimer timer;

    SMCResetController(axis);
    foreach(const int &a, axis)
    {
        Sleep(200);
        retval += SMCSendCommand("PW1", false, m_addresses[a]);
        Sleep(1000);
        SMCCheckError(m_addresses[a]);
        retval += SMCCheckError(m_addresses[a]);
        qDebug() << "EnterConfig" << a;

        if (!retval.containsError())
        {
            timer.start();

            while (true)
            {
                retval += SMCCheckStatus(axis);

                if (retval.containsError())
                {
                    break;
                }

                this->setAlive();

                if (m_controllerState.at(a) == this->ctrlStConfig)
                {
                    break;
                }
                else if (timer.elapsed() > 5000)
                {
                    retval += ito::RetVal::format(ito::retError, 0, tr("timeout while entering config mode (axis %i)").toLatin1().data(), m_addresses[a]);
                    break;
                }
            }
        }
    }
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCLeaveConfigMode(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    // Sets the controllers to "not ref" state
    foreach(const int &a, axis)
    {
        this->SMCSendCommand("PW0", false, m_addresses[a]);
        qDebug() << "LeaveConfig" << a;
        while (true)
        {
            SMCCheckStatus(axis);
            this->setAlive();
            if (m_controllerState.at(a) == this->ctrlStNotRef)
            {
                break;
            }
        }
    }

    // Wait till movement is done and the release the semaphor
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
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

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
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
    QVector<QPair<int, QByteArray> > lastError;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, additionalTag);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (hasIndex && index != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("this motor only has one axis, therefore it is not allowed to get a parameter with index unequal to 0").toLatin1().data());
    }
    else if (!retValue.containsError())
    {
        if (key == "async")
        {
            it->setVal<int>(m_async);
            *val = it.value();
        }
        else if (key == "comPort")
        {
            QSharedPointer<ito::Param> val(new ito::Param("port"));
            retValue += m_pSer->getParam(val, NULL);
            //int comPort = 0;
            if (!retValue.containsError())
            {
                retValue += it->setVal<int>(val->getVal<int>());
            }
        }
        else if (key == "accel")
        {
            retValue += SMCGetVelocityAcceleration(false);
            retValue += it->setVal<double*>(m_acceleration.data(), m_acceleration.size());
            *val = it.value();
        }
        else if (key == "calib_mode")
        {
            retValue += SMCGetCalibMode(QVector<int>(m_addresses.size()));
            retValue += it->setVal<int*>(m_calibMode.data(), m_calibMode.size());
            *val = it.value();
        }
        else if (key == "config_state")
        {
            // only the controller status is checked. Every other status is displayed as 0
            // the setter method leaves every controller represented by a 0 in its state. It only changes the 1.
            QVector<int> axis;
            foreach(const int &a, m_addresses)
            {
                axis.append(a-1);
            }
            retValue += SMCCheckStatus(axis);
            axis.clear();
            foreach(const int &b, m_controllerState)
            {
                if (b == ctrlStConfig)
                {
                    axis.append(ctrlStConfig);
                }
                else
                {
                    axis.append(0);
                }
            }
            retValue += it->setVal<int*>(axis.data(), axis.size());
            *val = it.value();
        }
        else if (key == "current_status")
        {
            QVector<int> axis;
            foreach(const int &a, m_addresses)
            {
                axis.append(a-1);
            }
            retValue += SMCCheckStatus(axis);
            retValue += it->setVal<int*>(m_controllerState.data(), m_controllerState.size());
            *val = it.value();
        }
        else if (key == "ctrlId")
        {
            QString commaText = "";
            foreach(const QString &s, m_ids)
            {
                commaText.append(s + '\n');
            }
            commaText.remove(commaText.length() - 1, 1);
            retValue += it->setVal<char*>(commaText.toLatin1().data(), commaText.size());
            *val = it.value();
        }
        else if (key == "speed")
        {
            retValue += SMCGetVelocityAcceleration(true);
            retValue += it->setVal<double*>(m_velocity.data(), m_velocity.size());
            *val = it.value();
        }
        else if (key == "name")
        {
            retValue += it->setVal<char*>(getIdentifier().toLatin1().data(), getIdentifier().size());
            *val = it.value();
        }
        else if (key == "numaxis")
        {
            val->setVal<int>(m_numAxis);
            *val = it.value();
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
    QVector<QPair<int, QByteArray> > lastitError;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }
    if (!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        //---------------------------
        if (key == "calib_mode")
        {
            if (hasIndex)
            {
                //if (index < 0 || index >= m_numAxis)
                //{
                //    retValue += ito::RetVal::format(ito::retError, 0, "index must be in range [0,%i]", m_numAxis-1);
                //}
                //else
                //{
                //    it->getVal<int*>()[index] = val->getVal<int>();
                //    //setCalibMode
                //}
            }
            else
            {
                if (val->getLen() != m_numAxis)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("The given array must have %i entries").toLatin1().data(), m_numAxis);
                }
                else
                {
                    m_calibMode = QVector<int>(m_numAxis);
                    memcpy(m_calibMode.data(), val->getVal<int*>(), sizeof(int)*m_numAxis);
                    retValue += SMCSetCalibMode(m_calibMode);
                    if (!retValue.containsError())
                    {
                        it->copyValueFrom(&(*val));
                    }
                }
            }
        }
        //---------------------------
        else if (key == "exec_calib")
        {
            if (hasIndex)
            {
                if (val->getVal<int>() > 0)
                {
                    calib(QVector<int>(1, index));
                }
            }
            else
            {
                QVector<int> axes;
                for (int i = 0; i < val->getLen(); ++i)
                {
                    if (val->getVal<int*>()[i] > 0)
                    {
                        axes.append(i);
                    }
                }

                calib(axes);
            }
        }
        //---------------------------
        else if (key == "config_state")
        {
            if (hasIndex)
            {
                //if (index < 0 || index >= m_numAxis)
                //{
                //    retValue += ito::RetVal::format(ito::retError, 0, "index must be in range [0,%i]", m_numAxis-1);
                //}
                //else
                //{
                //    it->getVal<int*>()[index] = val->getVal<int>();
                //    SMCEnterConfigMode(QVector<int>(1, it->getVal<int*>()[0]));
                //    retValue += it->copyValueFrom(&(*val));
                //}
            }
            else
            {
                if (val->getLen() != m_numAxis)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("The given array must have %i entries").toLatin1().data(), m_numAxis);
                }
                else
                {
                    it->copyValueFrom(&(*val));
                    QVector<int> tempEnter;
                    QVector<int> tempLeave;
                    for (int i = 0; i < m_numAxis; ++i)
                    {
                        if (it->getVal<int*>()[i] == 1)
                        {
                            tempEnter.append(i);
                        }
                        else if (it->getVal<int*>()[i] == 0 && m_controllerState[i] == 1)
                        {
                            tempLeave.append(i);
                        }
                    }
                    SMCEnterConfigMode(tempEnter);
                    SMCLeaveConfigMode(tempLeave);
                }
            }
        }
        //---------------------------
        else if (key == "async")
        {
            m_async = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }
        //---------------------------
        else if (key == "speed")
        {
            if (hasIndex)
            {
                //if (index < 0 || index >= m_numAxis)
                //{
                //    retValue += ito::RetVal::format(ito::retError, 0, "index must be in range [0,%i]", m_numAxis-1);
                //}
                //else
                //{
                //    //SMCSetAcceleration
                //    it->getVal<int*>()[index] = val->getVal<int>();
                //}
            }
            else
            {
                if (val->getLen() != m_numAxis)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("The given array must have %i entries").toLatin1().data(), m_numAxis);
                }
                else
                {
                    QVector<double> speed = QVector<double>(val->getLen());
                    memcpy(speed.data(), val->getVal<int*>(), sizeof(double)*val->getLen());
                    SMCSetVelocityAcceleration(true, speed);
                    it->copyValueFrom(&(*val));
                }
            }
        }
        //---------------------------
        else if (key == "accel")
        {
            if (hasIndex)
            {
                //if (index < 0 || index >= m_numAxis)
                //{
                //    retValue += ito::RetVal::format(ito::retError, 0, "index must be in range [0,%i]", m_numAxis-1);
                //}
                //else
                //{
                //    //SMCSetAcceleration(QVector<int>(0, ));
                //    it->getVal<int*>()[index] = val->getVal<int>();
                //}
            }
            else
            {
                if (val->getLen() != m_numAxis)
                {
                    retValue += ito::RetVal::format(ito::retError, 0, tr("The given array must have %i entries").toLatin1().data(), m_numAxis);
                }
                else
                {
                    QVector<double> accel = QVector<double>(val->getLen());
                    memcpy(accel.data(), val->getVal<int*>(), sizeof(double)*val->getLen());
                    SMCSetVelocityAcceleration(false, accel);
                    it->copyValueFrom(&(*val));
                }
            }
        }
        else
        {
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
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
ito::RetVal SMC100::SMCGetCalibMode(const QVector<int> axis)
{
    ito::RetVal retval;
    for (int i = 0; i < axis.size(); ++i)
    {
        double v;
        retval += SMCSendQuestionWithAnswerDouble("HT?", v, SMC_READTIMEOUT, false, m_addresses[i]);
        m_calibMode[i] = v;
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCSetCalibMode(const QVector<int> axisAndMode)
{
    ito::RetVal retval = ito::retOk;
    QVector<int> axis;
    bool released = false;
    for (int i = 0; i < axisAndMode.size(); ++i)
    {
        if (m_addresses.size() > i)
        {
            if (axisAndMode[i] != m_calibMode[i])
            {
                if (m_controllerState[i] == ctrlStConfig)
                {
                    axis.append(i);
                    SMCSendCommand("HT"+QByteArray::number(axisAndMode[i]), false, m_addresses[i]);
                    m_calibMode[i] = axisAndMode[i];
                }
                else
                {
                    retval += ito::RetVal(ito::retWarning, 0, tr("The requested axis is not in config mode").toLatin1().data());
                    break;
                }
            }
            // otherwise it´s -1 and that means that the old status remains
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("The requested axis for setting homing mode could not be found").toLatin1().data());
            break;
        }
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
ito::RetVal SMC100::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    //for homing, we first need to go into not-reference mode (if not yet done)
    ito::RetVal retval = SMCResetController(axis, NULL);

    bool done = false;
    QElapsedTimer timer;

    foreach (const int &a, axis)
    {
        if (a < 0 || a >= m_numAxis)
        {
            retval += ito::RetVal(ito::retError, 0, tr("invalid axis number").toLatin1().data());
        }
        else
        {
            if (m_controllerState[a] == ctrlStNotRef)
            {
                SMCSendCommand("OR", false, m_addresses[a]);
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("The requested axis is not in notReferenced mode").toLatin1().data());
            }
        }
    }

    timer.start();

    while (!done)
    {
        retval += SMCCheckStatus(axis);

        qDebug() << "calib: " << m_controllerState << m_controllerState << m_currentStatus;

        done = true; //suggest we are done
        foreach (int a, axis)
        {
            if (m_controllerState[a] != ctrlStReady)
            {
                done = false;
            }
        }

        if (timer.elapsed() > m_params["calib_timeout"].getVal<double>() * 1000.0)
        {
            retval += ito::RetVal(ito::retError, 0, tr("timeout when calibrating axes").toLatin1().data());
            break;
        }

        if (retval.containsError())
        {
            break;
        }

        setAlive();
    }

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
    ito::RetVal retval;// = SMCCheckStatus();
    QVector<int> allAxis;
    for (int i = 0; i < m_numAxis; ++i)
    {
        allAxis.append(i);
    }
    SMCCheckStatus(allAxis);
    *status = m_controllerState;

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
    QSharedPointer<QVector<double> > pos_(new QVector<double>(1, 0.0));
    ito::RetVal ret = getPos(QVector<int>(1, axis), pos_, waitCond);
    *pos = (*pos_)[0];
    return ret;

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

    double *pos_ = pos->data();

    for (int idx = 0; idx < axis.size(); ++idx)
    {
        if (axis[idx] < 0 || axis[idx] >= m_numAxis)
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("invalid axis number (%i)").toLatin1().data(), axis[idx]);
        }
        else
        {
            retval += this->SMCSendQuestionWithAnswerDouble("TP", pos_[idx], 200, false, m_addresses[axis[idx]]);
            //(*pos)[0] = *sharedpos;
            m_currentPos[axis[idx]] = pos_[idx];
        }
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

    \param [in] axis     axis number
    \param [in] pos      absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal SMC100::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return SMCSetPos(QVector<int>(1, axis), QVector<double>(1, pos), false, waitCond);
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
    return SMCSetPos(axis, pos, false, waitCond);
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
    return SMCSetPos(QVector<int>(1, axis), QVector<double>(1, pos), true, waitCond);
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
    return SMCSetPos(axis, pos, true, waitCond);
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

    //retval += SMCCheckStatus();

    for (int i = 0; i < m_addresses.size(); ++i)
    {
        if (sendCurrentPos)
        {
            retval += getPos(i,sharedpos,0);
            m_currentPos[i] = *sharedpos;

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
    }

    return retval;

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCGetVelocityAcceleration(bool vNota)
{
    ito::RetVal retval;
    if (vNota == true)
    {
        for (int i = 0; i < m_numAxis; ++i)
        {
             retval += SMCSendQuestionWithAnswerDouble("VA?", m_velocity[i], SMC_READTIMEOUT, false, m_addresses[i]);
        }
    }
    else
    {
        for (int i = 0; i < m_numAxis; ++i)
        {
            retval += SMCSendQuestionWithAnswerDouble("AC?", m_acceleration[i], SMC_READTIMEOUT, false, m_addresses[i]);
        }
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCSetVelocityAcceleration(bool vNota, const QVector<double> value)
{
    ito::RetVal retval;
    if (vNota == true && value.size() == m_numAxis)
    { // Velocity
        for (int i = 0; i < m_numAxis; ++i)
        {
            if (m_controllerState[i] == ctrlStConfig)
            {
                QString cmd = "VA"+QString::number(value[i]);
                retval += SMCSendCommand(cmd.toLatin1(), false, m_addresses[i]);
            }
        }
    }
    else if (vNota == false && value.size() == m_numAxis)
    { // Acceleration
        for (int i = 0; i < m_numAxis; ++i)
        {
            if (m_controllerState[i] == ctrlStConfig)
            {
                QString cmd = "AC"+QString::number(value[i]);
                retval += SMCSendCommand(cmd.toLatin1(), false, m_addresses[i]);
            }
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("Array size doesn´t match the number of axis").toLatin1().data());
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
updates the status flag (if possible). For E662 only the overflow status can be checked, the rest is determined by the movement
and a guess if the motor reached the target position
*/
ito::RetVal SMC100::SMCCheckStatus(const QVector<int> axis)
{
    ito::RetVal retVal = ito::retOk;
    foreach(const int &a, axis)
    {
        if (a < 0 || a >= m_numAxis)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("invalid axis").toLatin1().data());
        }
        else
        {
            // This is a dummy read so that stage errors disappear
            SMCCheckError();
            ito::RetVal r = SMCSendCommand("TS?", false, m_addresses[a]);
            QByteArray ans;
            int len;
            r += SMCReadString(ans, len, 200, false);
            ans = ans.right(2);
            bool ok = false;
            int i = ans.toUInt(&ok, 16);
            if (ok == true)
            {
                if (i >= 10 && i < 19)
                {   // x0A - x11 =>NOT REFERENCED
                    setStatus(m_currentStatus[a], ito::actuatorAvailable, ito::actSwitchesMask | ito::actStatusMask);
                    m_controllerState[a] = ctrlStNotRef;
                }
                else if (i == 20)
                {   // x14 => CONFIG-Mode
                    setStatus(m_currentStatus[a], ito::actuatorAvailable, ito::actSwitchesMask | ito::actStatusMask);
                    m_controllerState[a] = this->ctrlStConfig;
                }
                else if (i == 40 || i== 0x001E || i == 0x001F) //moving, homing by rs232 or homing by smc-rc
                {   // x28 => MOVING
                    setStatus(m_currentStatus[a], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                    m_controllerState[a] = this->ctrlStMotion;
                }
                else if (i > 49 && i < 54)
                {   // x32 - x35 => READY
                    setStatus(m_currentStatus[a], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                    m_controllerState[a] = this->ctrlStReady;
                }
                else if (i > 59 && i < 63)
                {   // x3C - x3E => DISABLED
                    setStatus(m_currentStatus[a], !ito::actuatorEnabled, ito::actSwitchesMask | ito::actStatusMask);
                    m_controllerState[a] = this->ctrlStDisabled;
                }
                else
                {
                    m_controllerState[a] = -1;
                }
            }
        }
    }
    requestStatusAndPosition(true, true);
    sendStatusUpdate(false);
    return retVal;
}

//--------------------------------------------------------------------------------
ito::RetVal SMC100::SMCReadString(QByteArray &result, int &len, int timeoutMS, bool checkError, int axis /*= -1*/)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    bool done = false;

    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";
    int curFrom = 0;
    int pos = 0;
    QByteArray endline;

    retValue += m_pSer->getParam(endlineParam, NULL);

    if (endlineParam->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = endlineParam->getVal<char*>(); //borrowed reference
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

    if (checkError)
    {
        retValue += SMCCheckError(axis);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCCheckError(int axis /*= -1*/)
{
    if (axis == -1) axis = 1;
    ito::RetVal r = SMCSendCommand("TE", false, axis);
    QByteArray err;
    char errChar;
    int len;

    r += SMCReadString(err, len, 200, false);

    if (!r.containsError())
    {
        if (err.mid(1,2) != "TE" || err.size() < 4)
        {
            r += ito::RetVal(ito::retError, 0, tr("error while checking error.").toLatin1().data());
        }
        else
        {
            errChar = err.right(1).data()[0];

            switch (errChar)
            {
            case '@':
                break;
            case 'A':
                r += ito::RetVal(ito::retError, 0, tr("Unknown message code or floating point controller address.").toLatin1().data());
                break;
            case 'B':
            {
                r += SMCSendCommand("TE", false, axis);
                err = "";
                r += SMCReadString(err, len, 200, false);

                if (!r.containsError())
                {
                    errChar = err.right(1).data()[0];
                    if (errChar == 'B')
                    {
                        r += ito::RetVal(ito::retError, 0, tr("Controller address not correct.").toLatin1().data());
                    }
                }
                break;
            }
            case 'C':
            {
                r += ito::RetVal(ito::retError, 0, tr("Parameter missing or out of range.").toLatin1().data());
                break;
            }
            case 'D':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed.").toLatin1().data());
                break;
            }
            case 'E':
            {
                r += ito::RetVal(ito::retError, 0, tr("Home sequence already started.").toLatin1().data());
                break;
                break;
            }
            case 'F':
            {
                r += ito::RetVal(ito::retError, 0, tr("ESP stage name unknown.").toLatin1().data());
                break;
            }
            case 'G':
            {
                r += ito::RetVal(ito::retError, 0, tr("Displacement out of limits.").toLatin1().data());
                break;
            }
            case 'H':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed in NOT REFERENCED state.").toLatin1().data());
                break;
            }
            case 'I':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed in CONFIGURATION state.").toLatin1().data());
                break;
            }
            case 'J':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed in DISABLE state.").toLatin1().data());
                break;
            }
            case 'K':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed in READY state.").toLatin1().data());
                break;
            }
            case 'L':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed in HOMING state.").toLatin1().data());
                break;
            }
            case 'M':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed in MOVING state.").toLatin1().data());
                break;
            }
            case 'N':
            {
                r += ito::RetVal(ito::retError, 0, tr("Current position out of software limit.").toLatin1().data());
                break;
            }
            case 'S':
            {
                r += ito::RetVal(ito::retError, 0, tr("Communication Time Out.").toLatin1().data());
                break;
            }
            case 'U':
            {
                r += ito::RetVal(ito::retError, 0, tr("Error during EEPROM access.").toLatin1().data());
                break;
            }
            case 'V':
            {
                r += ito::RetVal(ito::retError, 0, tr("Error during command execution.").toLatin1().data());
                break;
            }
            case 'W':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed for PP version.").toLatin1().data());
                break;
            }
            case 'X':
            {
                r += ito::RetVal(ito::retError, 0, tr("Command not allowed for CC version.").toLatin1().data());
                break;
            }
            }
        }
    }

    return r;
}

//-----------------------------------------------------------------------------------------------
ito::RetVal SMC100::SMCSendCommand(const QByteArray &cmd, bool checkError, int axis /*= -1*/)
{
    ito::RetVal retVal;
    QByteArray command(cmd);

    if (axis > 0)
    {
        command = QByteArray::number(axis) + cmd;
    }

    retVal += m_pSer->setVal(command.data(), command.length(), NULL);

    if (checkError)
    {
        retVal += SMCCheckError(axis);
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
ito::RetVal SMC100::SMCSendQuestionWithAnswerDouble(const QByteArray &questionCommand, double &answer, int timeoutMS, bool checkError, int axis /*= -1*/)
{
    int readSigns;
    QByteArray _answer;
    bool ok;

    ito::RetVal retValue = SMCSendCommand(questionCommand, false, axis);
    retValue += SMCReadString(_answer, readSigns, timeoutMS, checkError, axis);

    answer = _answer.mid(3).toDouble(&ok);

    if (!ok)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("value could not be parsed to a double value").toLatin1().data());
    }

    return retValue;
}

//-------------------------------------------------------------------
ito::RetVal SMC100::SMCSendQuestionWithAnswerString(const QByteArray &questionCommand, QByteArray &answer, int timeoutMS, bool checkError, int axis /*= -1*/)
{
    int readSigns;
    ito::RetVal retValue = SMCSendCommand(questionCommand, false, axis);
    retValue += SMCReadString(answer, readSigns, timeoutMS, checkError, axis);
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
ito::RetVal SMC100::SMCSetPos(const QVector<int> axis, const QVector<double> posMM, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    bool released = false;
    QByteArray cmd;

    foreach (const int &a, axis)
    {
        if (a < 0 || a >= m_numAxis)
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("Axis %i does not exist").toLatin1().data(), a);
        }
    }

    if (!retval.containsError())
    {
        if (axis.size() == 1)
        { // Single Axis movement
            if (relNotAbs)
            {   // Relative movement
                cmd = QByteArray("PR") + QByteArray::number(posMM[0]);
            }
            else
            {   // Absolute movement
                cmd = QByteArray("PA") + QByteArray::number(posMM[0]);
            }

            setStatus(m_currentStatus[axis[0]], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate(false);

            if (relNotAbs)
            {
                m_targetPos[axis[0]] = m_currentPos[axis[0]] + posMM[0];
            }
            else
            {
                m_targetPos[axis[0]] = posMM[0];
            }
            sendTargetUpdate();

            retval += SMCSendCommand(cmd, true, m_addresses[axis[0]]);
        }
        else
        { // Multiple Axis simultaneous movement
            if (relNotAbs)
            { // Relative movement (the se command does not support relative movement)
                for (int i = 0; i < axis.size(); ++i)
                { // first do all the movement
                    cmd = QByteArray("PR") + QByteArray::number(posMM[i]);
                    m_targetPos[axis[0]] = m_currentPos[axis[0]] + posMM[0];
                    sendTargetUpdate();
                    SMCSendCommand(cmd, false, m_addresses[axis[i]]);
                }
                for (int i = 0; i < axis.size(); ++i)
                { // Check Errors afterwards, that the movements are nearly simultaneous
                    retval += SMCCheckError(m_addresses[axis[i]]);
                }
            }
            else
            { // Absolute movement
                for (int i = 0; i < axis.size(); ++i)
                {
                    cmd = QByteArray("SE") + QByteArray::number(posMM[i]);
                    m_targetPos[axis[i]] = posMM[i];
                    sendTargetUpdate();
                    retval += SMCSendCommand(cmd, true, m_addresses[axis[i]]);
                }
                SMCSendCommand("SE", true, -1);
            }
        }

        if (!retval.containsError())
        {
            // release semaphor immediately
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
            retval += waitForDone(100, axis);
            // Wait till movement is done and the release the semaphor
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
        else
        {
            foreach (const int &a, axis)
            {
                replaceStatus(m_currentStatus[a], ito::actuatorMoving, ito::actuatorInterrupted);
            }
            sendStatusUpdate(false);
        }
    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        released = true;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool stillMoving;

    while (!done && !retVal.containsWarningOrError())
    {
        foreach (const int &a, axis)
        {
            /*retVal += */SMCCheckError(m_addresses[a]);
        }

        if (isInterrupted())
        {
            SMCSendCommand("ST", false, -1);

            foreach (const int &a, axis)
            {
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);
            }

            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;

            sendStatusUpdate(true);
        }
        else
        {
            stillMoving = false;
            SMCCheckStatus(axis);
            foreach (const int &a, axis)
            {
                if (m_currentStatus[a] & ito::actuatorMoving)
                {
                    stillMoving = true;
                }
            }

            done = !stillMoving;

            if (done)
            {   // Position reached and movement done
                break;
            }

            if (!retVal.containsError())
            {
                setAlive();
            }
        }
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void SMC100::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            connect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
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

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
///////////////////////// NOT IMPLEMENTED FUNCTIONS /////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


ito::RetVal SMC100::setOrigin(const int axis, ItomSharedSemaphore * /*waitCond*/)
{
    return setOrigin(QVector<int>(1,axis), NULL);
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal SMC100::setOrigin(QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Not implemented, use calibmode to set actual position as zero.").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}



// TODO: Implement the hasIndex functionality in setParam
