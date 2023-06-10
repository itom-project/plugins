/* ********************************************************************
    Plugin "Standa NanotecStepMotor" for itom software
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

#define NOMINMAX

#include "NanotecStepMotor.h"

#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qtimer.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>
//#include <fcntl.h>

#include <qdebug.h>

#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

//! Error codes for error byte in EEPROM
#define ERROR_LOWVOLTAGE    0x01
#define ERROR_TEMP          0x02
#define ERROR_TMC           0x04
#define ERROR_EE            0x08
#define ERROR_QEI           0x10
#define ERROR_INTERNAL      0x20
#define ERROR_DRIVER        0x80

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//!
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
    creates new instance of dialogNanotecStepMotor, calls the method setVals of dialogNanotecStepMotor, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogNanotecStepMotor
*/

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NanotecStepMotor::NSMReadWrite(const int deviceIndex, const QString &command, QString *retAnswer /*= NULL*/)
{
    char buf[50];
    int deviceID = m_params["axisID"].getVal<int*>()[deviceIndex];
    QString compareCommand = command;
    QString commandStr = "#" + QString("%1").arg(deviceID) + compareCommand;
    QString answer = "";

    QSharedPointer<QVector<ito::ParamBase> > _dummy;
    ito::RetVal retValue = m_pSer->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, NULL);

    retValue += m_pSer->setVal(commandStr.toLatin1().data(), commandStr.length());

    if (!retValue.containsError())
    {
        QSharedPointer<int> len(new int);
        *len = sizeof(buf);

        QSharedPointer<char> tempBuf = QSharedPointer<char>(&buf[0], NanotecStepMotor::doNotDelSharedPtr); //trick to access part of buf using a shared pointer. the shared pointer is not allowed to delete the char-array, therefore the Deleter-method.
        retValue += m_pSer->getVal(tempBuf, len);

        if (!retValue.containsError())
        {
            answer = QString(buf).mid(0, *len);
            if (answer.size() == 0)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Missing answer").toLatin1().data());
            }
        }
    }

    if (answer.size() > 0)
    {
        if (answer.contains("?"))
        {
            if (answer.contains("?clock", Qt::CaseInsensitive))
            {
                retValue += ito::RetVal(ito::retError, 0, tr("The clock direction mode is currently active and the clock frequency is greater than 65 kHz. Please set the frequency less than 65 kHz.").toLatin1().data());
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Invalid command: '%1'").arg(commandStr).toLatin1().data());
            }
        }
        else
        {
            int commandPos = answer.indexOf(command, Qt::CaseInsensitive);
            int answerDeviceID = answer.mid(0, commandPos).toInt();
            if (answerDeviceID != deviceID || compareCommand.compare(answer.mid(commandPos, compareCommand.size()), Qt::CaseInsensitive) != 0)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Wrong answer: '%1'").arg(answer).toLatin1().data());
            }
            else if (retAnswer != NULL)
            {
                *retAnswer = answer.mid(commandPos + compareCommand.size());
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NanotecStepMotor::NSMReadWrite(const int deviceIndex, const QString &command, int *retAnswer)
{
    QString answer = "";
    ito::RetVal retValue = ito::retOk;
    retValue = NSMReadWrite(deviceIndex, command, &answer);
    if (retValue.containsError())
    {
        *retAnswer = -1;
    }
    else
    {
        bool ok = true;
        *retAnswer = answer.toInt(&ok, 10);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Value '%1' is not a number!").arg(answer).toLatin1().data());
            *retAnswer = -1;
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NanotecStepMotor::getState(const int deviceIndex, int &stateID, QString &stateText)
{
    ito::RetVal retValue = NSMReadWrite(deviceIndex, "$", &stateID);

    if (retValue.containsError())
    {
        stateID = -1;
    }
    else
    {
        stateID = stateID & 15;

        if (stateID & 8)
        {
            stateText = tr("Input 1 is set while the controller is ready again");
        }
        else if (stateID & 4)
        {
            QString errorText = "";
            getLastErrorText(deviceIndex, errorText);
            stateText = tr("Position error: ") + errorText;
        }
        else if (stateID & 2)
        {
            stateText = tr("Zero position reached");
        }
        else if (stateID & 1)
        {
            stateText = tr("Controller ready");
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal NanotecStepMotor::getLastErrorText(const int deviceIndex, QString &errorText)
{
    int errorPos = 0;
    int errorCode = 0;

    ito::RetVal retValue = NSMReadWrite(deviceIndex, "Z1E", &errorPos);
    if (!retValue.containsError())
    {
        ito::RetVal retValue = NSMReadWrite(deviceIndex, QString("%1E").arg(errorPos), &errorCode);
        if (!retValue.containsError())
        {
            switch (errorCode)
            {
                case ERROR_LOWVOLTAGE:
                    errorText = tr("Undervoltage");
                    break;
                case ERROR_TEMP:
                    errorText = tr("Temperature of the motor controller is outside of the specified range");
                    break;
                case ERROR_TMC:
                    errorText = tr("Overcurrent switch-off of the dspDrive was triggered");
                    break;
                case ERROR_EE:
                    errorText = tr("Incorrect data in the EEPROM");
                    break;
                case ERROR_QEI:
                    errorText = tr("Position error");
                    break;
                case ERROR_INTERNAL:
                    errorText = tr("Internal error");
                    break;
                case ERROR_DRIVER:
                    errorText = tr("Driver component returned one error");
                    break;
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
double NanotecStepMotor::stepsToUnit(const int deviceIndex, const double &steps)
{
    return steps / ((double)(m_params["microSteps"].getVal<int*>()[deviceIndex]) * m_params["axisSteps"].getVal<double*>()[deviceIndex]);
}

//----------------------------------------------------------------------------------------------------------------------------------
int NanotecStepMotor::unitToSteps(const int deviceIndex, double unitStep)
{
    return qRound(unitStep * (double)(m_params["microSteps"].getVal<int*>()[deviceIndex]) * m_params["axisSteps"].getVal<double*>()[deviceIndex]);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the NanotecStepMotor::init. The widged window is created at this position.
*/
NanotecStepMotor::NanotecStepMotor() :
    AddInActuator(),
    m_numAxes(0),
    m_pSer(NULL),
    m_async(0)
{
    // Read only - Parameters
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "NanotecStepMotor", tr("Name of the plugin").toLatin1().data()));
    m_params.insert("deviceID", ito::Param("deviceID", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Name of controller").toLatin1().data()));
    m_params.insert("devicePort", ito::Param("devicePort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 255, 0, tr("Serial port of device").toLatin1().data()));
    m_params.insert("numaxis", ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 255, 0, tr("Number of axis").toLatin1().data()));

    ito::Param param("axisID", ito::ParamBase::IntArray, NULL, tr("internal ID of axis").toLatin1().data());
    ito::IntArrayMeta idm(0, 255, 1, 1, 255, 1);
    param.setMeta(&idm, false);
    m_params.insert("axisID", param);

    param = ito::Param("axisSteps", ito::ParamBase::DoubleArray | ito::ParamBase::Readonly, NULL, tr("number of full steps per unit (deg or mm) of axis, 0: axis not connected [default]").toLatin1().data());
//    ito::IntArrayMeta asm(0.0, 255, 1, 1, 255, 1);
//    param.setMeta(&asm, false);
    m_params.insert("axisSteps", param);

    param = ito::Param("units", ito::ParamBase::IntArray | ito::ParamBase::Readonly, NULL, tr("unit of axis, 0: degree [default], 1: mm").toLatin1().data());
    ito::IntArrayMeta uim(0, 1, 1, 1, 255, 1);
    param.setMeta(&uim, false);
    m_params.insert("units", param);

    // Read/Write - Parameters
    m_params.insert("async", ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or sychronous (0) [default] mode").toLatin1().data()));

    param = ito::Param("microSteps", ito::ParamBase::IntArray, NULL, tr("micro steps for motor [1, 2, 4, 5, 8, 10, 16, 32, 64]").toLatin1().data());
    ito::IntArrayMeta msm(1, 64, 1, 1, 255, 1);
    param.setMeta(&msm, false);
    m_params.insert("microSteps", param);

    param = ito::Param("accel", ito::ParamBase::IntArray, NULL, tr("motor shaft acceleration, range: 1..65.535 [default 2.364]").toLatin1().data());
    ito::IntArrayMeta acm(1, 65535, 1, 1, 255, 1);
    param.setMeta(&acm, false);
    m_params.insert("accel", param);

    param = ito::Param("decel", ito::ParamBase::IntArray, NULL, tr("motor shaft deceleration, range: 1..65.535 [default 2.364]").toLatin1().data());
    ito::IntArrayMeta dcm(0, 65535, 1, 1, 255, 1);
    param.setMeta(&dcm, false);
    m_params.insert("decel", param);

    param = ito::Param("speed", ito::ParamBase::IntArray, NULL, tr("target speed range: 1..1.000.000 [default 1.000]").toLatin1().data());
    ito::IntArrayMeta spm(1, 65535, 1, 1, 255, 1);
    param.setMeta(&spm, false);
    m_params.insert("speed", param);

    param = ito::Param("coilCurrent", ito::ParamBase::IntArray, NULL, tr("coil current if motor is running, range: 0..100").toLatin1().data());
    ito::IntArrayMeta ccm(0, 100, 1, 1, 255, 1);
    param.setMeta(&ccm, false);
    m_params.insert("coilCurrent", param);

    param = ito::Param("coilCurrentRest", ito::ParamBase::IntArray, NULL, tr("coil current if motor is in rest, range: 0..100").toLatin1().data());
    ito::IntArrayMeta crm(0, 100, 1, 1, 255, 1);
    param.setMeta(&crm, false);
    m_params.insert("coilCurrentRest", param);

	param = ito::Param("endSwitchSettings", ito::ParamBase::IntArray, NULL, tr("end switch behaviour").toLatin1().data());
    crm = ito::IntArrayMeta(0, std::numeric_limits<int>::max(), 1, 1, 255, 1);
    param.setMeta(&crm, false);
    m_params.insert("endSwitchSettings", param);

    m_currentPos.fill(0.0, 1);
    m_currentStatus.fill(0, 1);
    m_targetPos.fill(0.0, 1);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetNanotecStepMotor *dockWidget = new DockWidgetNanotecStepMotor(this);
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
ito::RetVal NanotecStepMotor::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 115200)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r")), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("readline", ito::ParamBase::Int, 1)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endlineRead", ito::ParamBase::String, "\r")), NULL);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
    }

    m_numAxes = (*paramsMand)[1].getLen();
    retval += m_params["numaxis"].setVal<int>(m_numAxes);

    QSharedPointer<ito::Param> val(new ito::Param("port"));
    retval += m_pSer->getParam(val, NULL);
    if (!retval.containsError())
    {
        retval += m_params["devicePort"].setVal<int>(val->getVal<int>());
    }

    if (m_numAxes < 1)
    {
        retval += ito::RetVal(ito::retError, 0, tr("No device given!").toLatin1().data());
    }
    else
    {
        retval += m_params["axisID"].copyValueFrom(&(paramsMand->at(1)));

        if (paramsMand->at(2).getLen() != m_numAxes)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Number of given axisSteps is different to number of axisID!").toLatin1().data());
        }
        else
        {
            retval += m_params["axisSteps"].copyValueFrom(&(paramsMand->at(2)));
        }

        if (paramsOpt->at(0).getVal<int*>() == NULL)
        {
            int *units = new int[m_numAxes];
            for (int i = 0; i < m_numAxes; ++i)
            {
                units[i] = 1;
            }
            m_params["units"].setVal<int*>(units, m_numAxes);
            delete[] units;
        }
        else if (paramsOpt->at(0).getLen() == m_numAxes)
        {
            retval += m_params["units"].copyValueFrom(&(paramsOpt->at(0)));
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("Number of given units is different to number of axisID!").toLatin1().data());
        }
    }

    if (!retval.containsError())
    {
        QString vstr = "";
        retval += NSMReadWrite(0, "v", &vstr);
        vstr = vstr.mid(1, vstr.indexOf("_") - 1);
        retval += m_params["deviceID"].setVal<char*>(vstr.toLatin1().data());
    }

    if (!retval.containsError())
    {
        int value = 0;
        int *microSteps = new int[m_numAxes];
		int *endSwitchBehaviour = new int[m_numAxes];
        int *accel = new int[m_numAxes];
        int *decel = new int[m_numAxes];
        int *speed = new int[m_numAxes];
        int *coilCurrent = new int[m_numAxes];
        int *coilCurrentRest = new int[m_numAxes];

        m_currentPos.fill(0.0, m_numAxes);
        m_currentStatus.fill(0, m_numAxes);
        m_targetPos.fill(0.0, m_numAxes);

        int state = 0;
        QString messageText = "";

        for (int i = 0; i < m_numAxes; ++i)
        {
			//configure end switch behaviour for all motors
			retval += NSMReadWrite(i, QString("l5138")); //Bit0, Bit4, Bit10, Bit12 (default behaviour: set external references to move back from the reference switch if it is reached!)
			/*retval += NSMReadWrite(i, QString("=0"));
			retval += NSMReadWrite(i, QString("f0"));
			retval += NSMReadWrite(i, QString("R100"));
			retval += NSMReadWrite(i, QString("Q-100"));
			retval += NSMReadWrite(i, QString("L16712127"));
			retval += NSMReadWrite(i, QString("h16712127"));
			retval += NSMReadWrite(i, QString("Y0"));*/
			retval += NSMReadWrite(i, QString("K20")); //entprellzeit
			retval += NSMReadWrite(i, QString(":port_in_a7")); //configure all input ports to be sensitive for reference switch...
			retval += NSMReadWrite(i, QString(":port_in_b7"));
			retval += NSMReadWrite(i, QString(":port_in_c7"));
			retval += NSMReadWrite(i, QString(":port_in_d7"));
			retval += NSMReadWrite(i, QString(":port_in_e7"));
			retval += NSMReadWrite(i, QString(":port_in_f7"));
			retval += NSMReadWrite(i, QString(":port_in_g7"));
			retval += NSMReadWrite(i, QString(":port_in_h7"));

            retval += getState(i, state, messageText);
            if (retval.containsError() || state == 4)
            {
                m_currentStatus[i] = ito::actuatorUnknown;
            }
            else
            {
                if (state == 2)
                {
                    m_currentStatus[i] = ito::actuatorEndSwitch;
                }
                else
                {
                    m_currentStatus[i] = ito::actuatorAvailable;
                }
            }

            // microSteps
            retval += NSMReadWrite(i, "Zg", &value);
            microSteps[i] = value;

            // accel
            retval += NSMReadWrite(i, "Zb", &value);
            accel[i] = value;

            // decel
            retval += NSMReadWrite(i, "ZB", &value);
            decel[i] = value;

            // speed
            retval += NSMReadWrite(i, "Zo", &value);
            speed[i] = value;

            // Setting the direction of rotation to left (0)
            retval += NSMReadWrite(i, "d0");

            // Disable setting the change of direction (0)
            retval += NSMReadWrite(i, "t0");

            // Setting the repetitions to 1
            retval += NSMReadWrite(i, "W1");

            // Setting the positioning mode to 1
            retval += NSMReadWrite(i, "p1");

            // coil
            retval += NSMReadWrite(i, "Zi", &value);
            coilCurrent[i] = value;

            // coil rest
            retval += NSMReadWrite(i, "Zr", &value);
            coilCurrentRest[i] = value;

			// end switch behaviour
            retval += NSMReadWrite(i, "Zl", &value);
            endSwitchBehaviour[i] = value;

            if (retval.containsError())
            {
                break;
            }
        }

        m_params["microSteps"].setVal<int*>(microSteps, m_numAxes);
        m_params["accel"].setVal<int*>(accel, m_numAxes);
        m_params["decel"].setVal<int*>(decel, m_numAxes);
        m_params["speed"].setVal<int*>(speed, m_numAxes);
        m_params["coilCurrent"].setVal<int*>(coilCurrent, m_numAxes);
        m_params["coilCurrentRest"].setVal<int*>(coilCurrentRest, m_numAxes);
		m_params["endSwitchSettings"].setVal<int*>(endSwitchBehaviour, m_numAxes);

        delete[] microSteps;
        delete[] accel;
        delete[] decel;
        delete[] speed;
        delete[] coilCurrent;
        delete[] coilCurrentRest;
		delete[] endSwitchBehaviour;
    }

    if (!retval.containsError())
    {
        retval += requestStatusAndPosition(true, true);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal NanotecStepMotor::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogNanotecStepMotor(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the NanotecStepMotorInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal NanotecStepMotor::close(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;

    ItomSharedSemaphoreLocker locker(waitCond);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! returns parameter of m_params with key name.
/*!
    This method copies the string of the corresponding parameter to val with a maximum length of maxLen.

    \param [in] name is the key name of the parameter
    \param [in,out] val is a shared-pointer of type char*.
    \param [in] maxLen is the maximum length which is allowed for copying to val
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal NanotecStepMotor::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        //put your switch-case.. for getting the right value here

        //finally, save the desired value in the argument val (this is a shared pointer!)
        //if the requested parameter name has an index, e.g. roi[0], then the sub-value of the
        //array is split and returned using the api-function apiGetParam
        if (hasIndex)
        {
            *val = apiGetParam(*it, hasIndex, index, retValue);
        }
        else
        {
            *val = *it;
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
            If the "ctrl-type" is set, NanotecStepMotor::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal NanotecStepMotor::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (isMotorMoving()) //this if-case is for actuators only.
    {
        retValue += ito::RetVal(ito::retError, 0, tr("any axis is moving. Parameters cannot be set").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (key == "async")
        {
            //check the new value and if ok, assign it to the internal parameter
            m_async = val->getVal<int>();

			if (!retValue.containsError())
			{
				retValue += it->copyValueFrom(&(*val));
			}
        }
        else
        {
            QString command = "";

            if (key == "microSteps")
            {
                command = "g";
            }
            else if (key == "accel")
            {
                command = "b";
            }
            else if (key == "decel")
            {
                command = "B";
            }
            else if (key == "speed")
            {
                command = "o";
            }
			else if (key == "endSwitchSettings")
			{
				command = "l";
			}

			if (hasIndex)
			{
				if (index >= 0 && index < m_numAxes)
				{
					retValue += NSMReadWrite(index, command + QString("%1").arg(val->getVal<int>()));
				}
				else
				{
					retValue += ito::RetVal::format(ito::retError, 0, "invalid index %i", index);
				}

				if (!retValue.containsError())
				{
					//int value;
					//// end switch behaviour
					//NSMReadWrite(index, "Z" + command, &value);
					//qDebug() << command << value;

					it->getVal<int*>()[index] = val->getVal<int>();
				}
			}
			else
			{
				const int* value = val->getVal<const int*>();
				if (val->getLen() == m_numAxes)
				{
					for (int i = 0; i < m_numAxes; ++i)
					{
						retValue += NSMReadWrite(i, command + QString("%1").arg(value[i]));
					}
				}
				else
				{
					retValue += ito::RetVal::format(ito::retError, 0, "wrong number of values. %i required.", m_numAxes);
				}

				if (!retValue.containsError())
				{
					retValue += it->copyValueFrom(&(*val));
				}
			}
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
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal NanotecStepMotor::calib(const int axis, ItomSharedSemaphore *waitCond)
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
ito::RetVal NanotecStepMotor::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Not implemented, use calibmode to set actual position as zero.").toLatin1().data());

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
ito::RetVal NanotecStepMotor::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = NSMCheckStatus();

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
ito::RetVal NanotecStepMotor::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (axis < m_numAxes)
    {
        int tPos = 0;
        retval = NSMReadWrite(axis, "C", &tPos);
        *pos = stepsToUnit(axis, (double)tPos);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("Given asis (%1) out of range!").arg(axis).toLatin1().data());
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
ito::RetVal NanotecStepMotor::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QSharedPointer<double> dpos(new double);
    *dpos = 0.0;

    if (axis.size() > m_numAxes)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Too much axes given!").toLatin1().data());
    }
    else
    {
        for (int i = 0; i < axis.size(); ++i)
        {
            retval += getPos(axis.value(i), dpos, 0);
            (*pos)[i] = *dpos;
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
            This function calls NanotecStepMotor::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis     axis number
    \param [in] pos      absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal NanotecStepMotor::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return NSMSetPos(QVector<int>(1, axis), QVector<double>(1, pos), false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls NanotecStepMotor::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal NanotecStepMotor::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    return NSMSetPos(axis, pos, false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            This function calls NanotecStepMotor::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal NanotecStepMotor::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return NSMSetPos(QVector<int>(1, axis), QVector<double>(1, pos), true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls NanotecStepMotor::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal NanotecStepMotor::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    return NSMSetPos(axis, pos, true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal NanotecStepMotor::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
    *sharedpos = 0.0;

    if (sendCurrentPos)
    {
        for (int i = 0; i < m_numAxes; ++i)
        {
            retval += getPos(i, sharedpos, 0);
            m_currentPos[i] = *sharedpos;
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
*/
ito::RetVal NanotecStepMotor::NSMCheckStatus()
{
    ito::RetVal retVal = ito::retOk;

    int stateID = 0;
    QString stateText = "";
    for (int i = 0; i < m_numAxes; ++i)
    {
        retVal += getState(i, stateID, stateText);
        if (stateText != "")
        {
            retVal += ito::RetVal(ito::retError, 0, stateText.toLatin1().data());
            break;
        }
    }

    setStatus(m_currentStatus[0], ito::actuatorAvailable, ito::actSwitchesMask | ito::actMovingMask);

    requestStatusAndPosition(true, true);
    sendStatusUpdate(false);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal NanotecStepMotor::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    return setOrigin(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal NanotecStepMotor::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Not implemented, use calibmode to set actual position as zero.").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;

/*    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::RetVal(ito::retOk);
    result_t result;

    if ((result = command_zero(m_device)) != result_ok)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Error zeroing: %1").arg(getErrorString(result)).toLatin1().data());
    }
    retval += SMCCheckError(retval);

    SMCCheckStatus();

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;*/
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the position (abs or rel) of a one axis spezified by "axis" to the position "dpos". The value in device independet in mm.
            If the axisnumber is not 0, this function returns an error.

    \param [in] axis        axis number
    \param [in] dpos        target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal NanotecStepMotor::NSMSetPos(const QVector<int> axis, const QVector<double> posUnit, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    bool released = false;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor is running. Further action is not possible!").toLatin1().data());
    }
    else if (axis.size() > m_numAxes)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Too much axes given!").toLatin1().data());
    }
    else
    {
        for (int i = 0; i < axis.size(); ++i)
        {
            if (axis[i] < m_numAxes)
            {
                int steps = unitToSteps(axis[i], posUnit[i]);

				if (m_currentStatus[axis[i]] & ito::actuatorEndSwitch)
				{
					retval += NSMReadWrite(axis[i], "D");
					setStatus(m_currentStatus[axis[i]], 0, ito::actMovingMask | ito::actStatusMask);
				}

				setStatus(m_currentStatus[axis[i]], ito::actuatorMoving, ito::actStatusMask | ito::actSwitchesMask);
				sendStatusUpdate(false);

                if (relNotAbs)
                {   // Relative movement
                    m_targetPos[axis[i]] = m_currentPos[axis[i]] + posUnit[i];
                    sendTargetUpdate();

                    bool switchMotorDirection = (steps >= 0); //true for positive drive to the right, false for negative drive

                    if (switchMotorDirection)
                    {
                        retval += NSMReadWrite(axis[i], "d1");
                    }

					if (!retval.containsError())
					{
						retval += NSMReadWrite(axis[i], QString("s%1").arg(abs(steps)));
						retval += NSMReadWrite(axis[i], "A");
					}

                    if (switchMotorDirection)
                    {
                        retval += NSMReadWrite(axis[i], "d0");
                    }
                }
                else
                {   // Absolute movement
                    // setting absolute positioning mode
                    retval += NSMReadWrite(axis[i], "p2");

                    m_targetPos[axis[i]] = posUnit[i];
                    sendTargetUpdate();

                    retval += NSMReadWrite(axis[i], QString("s%1").arg(steps));
                    retval += NSMReadWrite(axis[i], "A");

                    // setting relative positioning mode
                    retval += NSMReadWrite(axis[i], "p1");
                }
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("Given axis (%1) out of range!").arg(axis[i]).toLatin1().data());
            }
        }
    }

    if (!retval.containsError())
    {
        // release semaphore immediately
        if (m_async && waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
        retval += waitForDone(-1, axis);
        // Wait till movement is done and the release the semaphore
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

    for (int i = 0; i < axis.size(); ++i)
    {
        QSharedPointer<double> pos(new double);
        if (!getPos(axis[i], pos, NULL).containsError())
			m_currentPos[axis[i]] = *pos;
    }
    sendStatusUpdate(false);

    if (waitCond && !released)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        released = true;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal NanotecStepMotor::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retval(ito::retOk);
    bool done = false;
	bool error = false;

    QVector<int> axis_ = axis;
	if (axis_.size() == 0)
	{
		for (int i = 0; i < m_numAxes; ++i)
		{
			axis_.append(i);
		}
	}

    while (!done && !retval.containsWarningOrError())
    {
        if (isInterrupted())
        {
            for (int i = 0; i < m_numAxes; ++i)
            {
                retval += NSMReadWrite(i, "S1");
            }

            replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);

            retval += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;

            sendStatusUpdate(true);
        }
        else
        {
            int statePos = 255;
            int stateNeg = 0;
            int state = 0;
            QString messageText = "";
            done = true;

            foreach (int currentAxis, axis_)
            {
                retval += NSMReadWrite(currentAxis, "$", &state);
				if (state & 0x04) //positioning error in this axis
				{
					replaceStatus(m_currentStatus[currentAxis], ito::actuatorMoving, ito::actuatorInterrupted);

					setStatus(m_currentStatus[currentAxis], ito::actuatorEndSwitch, ito::actMovingMask | ito::actStatusMask);
					error = true;
				}
                else if ((state & 1) == 0) //this axis is still moving
                {
                    done = false;
                }
                /*retval += getState(currentAxis, state, messageText);
                statePos = statePos & state;
                stateNeg = stateNeg | state;*/
            }

            //done = (statePos & 1 == 1) || (stateNeg & 6 > 0);

            if (done)
            {   // Position reached and movement done
                replaceStatus(axis_, ito::actuatorMoving, ito::actuatorAtTarget);
                sendStatusUpdate(true);
                break;
            }

            if (!retval.containsError())
            {
                setAlive();
            }
        }
    }

	if (error)
	{
		QMutex waitMutex;
		QWaitCondition waitCondition;
        //short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, 1000);
        waitMutex.unlock();
	}


    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void NanotecStepMotor::dockWidgetVisibilityChanged(bool visible)
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
