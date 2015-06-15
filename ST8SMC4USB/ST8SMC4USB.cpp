/* ********************************************************************
    Plugin "Standa ST8SMC4USB" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2015, Institut für Technische Optik (ITO),
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

#include "ST8SMC4USB.h"

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

//#define SMC_READTIMEOUT 256


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//! 
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
    creates new instance of dialogST8SMC4USB, calls the method setVals of dialogST8SMC4USB, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogST8SMC4USB
*/

//----------------------------------------------------------------------------------------------------------------------------------
QString ST8SMC4USB::getErrorString(const result_t result)
{
    switch (result)
    {
        case result_error:
        {
            return tr("Error");
        }
        case result_not_implemented:
        {
            return tr("Not implemented");
        }
        case result_nodevice:
        {
            return tr("no device");
        }
        default:
        {
            return tr("Success");
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
QString ST8SMC4USB::getLogLevelString(int loglevel)
{
    switch (loglevel)
    {
        case LOGLEVEL_ERROR:
        {
            return "ERROR";
        }
        case LOGLEVEL_WARNING:
        {
            return "WARN";
        }
        case LOGLEVEL_INFO:
        {
            return "INFO";
        }
        case LOGLEVEL_DEBUG:
        {
            return "DEBUG";
        }
        default:
        {
            return "UNKNOWN";
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int ST8SMC4USB::microStepsToMicrostepMode(const int microSteps)
{
    switch (microSteps)
    {
        case 2:
        {
            return MICROSTEP_MODE_FRAC_2;
        }
        case 4:
        {
            return MICROSTEP_MODE_FRAC_4;
        }
        case 8:
        {
            return MICROSTEP_MODE_FRAC_8;
        }
        case 16:
        {
            return MICROSTEP_MODE_FRAC_16;
        }
        case 32:
        {
            return MICROSTEP_MODE_FRAC_32;
        }
        case 64:
        {
            return MICROSTEP_MODE_FRAC_64;
        }
        case 128:
        {
            return MICROSTEP_MODE_FRAC_128;
        }
        case 256:
        {
            return MICROSTEP_MODE_FRAC_256;
        }
        default:
        {
            return MICROSTEP_MODE_FULL;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ST8SMC4USB::setMoveSettings(int accel /*= -1*/, int speed /*= -1*/, int microStepSpeed /*= -1*/)
{
    ito::RetVal retValue(ito::retOk);
    result_t result;
    move_settings_t move_settings;

    if ((result = get_move_settings(m_device, &move_settings)) != result_ok)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Error reading move settings: %1").arg(getErrorString(result)).toLatin1().data());
    }
    else
    {
        if (accel > -1)
        {
            move_settings.Accel = accel;
        }

        if (speed > -1)
        {
            move_settings.Speed = speed;
        }

        if (microStepSpeed > -1)
        {
            move_settings.uSpeed = microStepSpeed;
        }
    }

    retValue += SMCCheckError(retValue);

    if (!retValue.containsError())
    {
        if ((result = set_move_settings(m_device, &move_settings)) != result_ok)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Error setting move settings: %1").arg(getErrorString(result)).toLatin1().data());
        }
    }

    retValue += SMCCheckError(retValue);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the ST8SMC4USB::init. The widged window is created at this position.
*/
ito::RetVal ST8SMC4USB::SMCCheckError(ito::RetVal retval)
{
    result_t result;
    status_t state;

    if (!retval.containsError())
    {
        if ((result = get_status(m_device, &state)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error getting status: %1").arg(getErrorString(result)).toLatin1().data());
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the ST8SMC4USB::init. The widged window is created at this position.
*/
ST8SMC4USB::ST8SMC4USB() :
    AddInActuator(),
    m_device(-1),
    m_engine_settings(),
    m_pSer(NULL)
{
    m_async = 0;

    ito::Param paramVal;

    // Read only - Parameters
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ST8SMC4USB", NULL));
    m_params.insert("device_id", ito::Param("device_id", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Name of controller").toLatin1().data()));
    m_params.insert("units_per_step", ito::Param("units_per_step", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 100000.0, 200.0, tr("units (deg or mm) per step of axis, e.g. full step resolution of data sheet of actuator").toLatin1().data()));
    m_params.insert("deviceNum", ito::Param("deviceNum", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 10, 0, tr("The current number of this specific device, if there are more than one devices connected. (0 = first device)").toLatin1().data()));
    m_params.insert("device_port", ito::Param("device_port", ito::ParamBase::String | ito::ParamBase::Readonly, "unknwon", tr("Serial port of device").toLatin1().data()));
    m_params.insert("unit", ito::Param("unit", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("unit of axis, 0: degree (default), 1: mm").toLatin1().data()));

    // Read/Write - Parameters
    m_params.insert("micro_steps", ito::Param("micro_steps", ito::ParamBase::Int, 1, 256, 1, tr("micro steps for motor [1,2,4,8,16,32,64,128,256]").toLatin1().data()));
    m_params.insert("async", ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or sychronous (0) mode").toLatin1().data()));
    m_params.insert("accel", ito::Param("accel", ito::ParamBase::Int, 0, 65535, 0, tr("Motor shaft acceleration, steps/s^2(stepper motor) or RPM/s(DC); range: 0..65535").toLatin1().data()));
    m_params.insert("speed", ito::Param("speed", ito::ParamBase::Int, 0, 1000000, 0, tr("Target speed(for stepper motor: steps / c, for DC: rpm); range: 0..1000000").toLatin1().data()));
    m_params.insert("micro_step_speed", ito::Param("micro_step_speed", ito::ParamBase::Int, 0, 255, 0, tr("Target speed in 1/256 microsteps/s; range: 0..255").toLatin1().data()));
    m_params.insert(paramVal.getName(), paramVal);

    m_currentStatus = QVector<int>(1, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);

    m_currentPos.fill(0.0, 1);
    m_currentStatus.fill(0, 1);
    m_targetPos.fill(0.0, 1);

    //now create dock widget for this plugin
    DockWidgetST8SMC4USB *dockWidget = new DockWidgetST8SMC4USB(this);
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
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
ito::RetVal ST8SMC4USB::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{   
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    result_t result;
    QString serialStr;
    move_settings_t move_settings;
    const int probe_devices = 0;
    int names_count = 0;
    char device_name[256];
    bool deviceOpen = false;

    int deviceNum = paramsOpt->value(0).getVal<int>(); //0: parameter "deviceNum"
    retval += m_params["deviceNum"].setVal<int>(deviceNum);

    int microSteps = paramsOpt->value(1).getVal<int>(); //1: parameter "microSteps"
    if ((microSteps != 0) && !(microSteps & (microSteps - 1)) && microSteps <= 256) // check if an integer is a power of two
    {
        retval += m_params["micro_steps"].setVal<int>(microSteps);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("micro_steps value must be a power of two <= 256").toLatin1().data());
    }

    retval += m_params["unit"].setVal<int>(paramsMand->value(1).getVal<int>()); //1: parameter "unit"

    double unitsPerStep = paramsMand->value(0).getVal<double>(); //0: parameter "units_per_step"
    if (unitsPerStep <= 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Error enumerating devices").toLatin1().data());
    }
    else
    {
        retval += m_params["units_per_step"].setVal<double>(unitsPerStep);
    }
                    
    device_enumeration_t devenum;

    /* Inherit system locale */
    setlocale(LC_ALL, "");

    if (!retval.containsError())
    {
        devenum = enumerate_devices(probe_devices);
        if (!devenum)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error enumerating devices").toLatin1().data());
        }
        else
        {
            deviceOpen = true;
        }
    }

    if (!retval.containsError())
    {
        names_count = get_device_count(devenum);
        if (names_count == -1)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error enumerating devices").toLatin1().data());
            names_count = 0;
        }
        else if (names_count == 0)
        {
            retval += ito::RetVal(ito::retError, 0, tr("No device found").toLatin1().data());
        }
        else
        {
            strcpy(device_name, get_device_name(devenum, deviceNum));
        }
    }

    if (deviceOpen)
    {
        free_enumerate_devices(devenum);
    }

    //Opening device
    if (!retval.containsError())
    {
        m_device = open_device(device_name);
        if (m_device == device_undefined)
        {
            if (m_device && (result = close_device(&m_device)) != result_ok)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Error opening device: %1").arg(getErrorString(result)).toLatin1().data());
            }
            else
            {
                m_device = open_device(device_name);
                if (m_device == device_undefined)
                {
                    retval += ito::RetVal(ito::retError, 0, tr("Error opening device").toLatin1().data());
                }
            }
        }

        if (!retval.containsError())
        {
            QString deviceName(device_name);
            deviceName.replace('\\', ' ');
            deviceName.replace('.', ' ');
            deviceName.simplified();
            retval += m_params["device_port"].setVal<char*>(deviceName.toLatin1().data());

            retval += SMCCheckError(retval);
        }
    }

    if (!retval.containsError())
    {
        if ((result = command_zero(m_device)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error zeroing: %1").arg(getErrorString(result)).toLatin1().data());
        }
        retval += SMCCheckError(retval);
    }

    if (!retval.containsError())
    {
        if ((result = command_stop(m_device)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error sending stop").toLatin1().data());
        }
        retval += SMCCheckError(retval);
    }

    if (!retval.containsError())
    {
        unsigned int serial;
        if ((result = get_serial_number(m_device, &serial)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error reading serial number").toLatin1().data());
        }
        else
        {
            serialStr.setNum(serial);
        }
        retval += SMCCheckError(retval);
    }

    if (!retval.containsError())
    {
        controller_name_t controller_name;
        if ((result = get_controller_name(m_device, &controller_name)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error reading controller name").toLatin1().data());
        }
        else
        {
            m_identifier = controller_name.ControllerName;
            m_identifier.append('-');
            m_identifier.append(serialStr);
            setIdentifier(m_identifier);
        }
        retval += SMCCheckError(retval);
    }

    if (!retval.containsError())
    {
        if ((result = get_engine_settings(m_device, &m_engine_settings)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error getting engine settings: %1").arg(getErrorString(result)).toLatin1().data());
        }
        retval += SMCCheckError(retval);
    }

    if (!retval.containsError())
    {
        m_engine_settings.MicrostepMode = microStepsToMicrostepMode(microSteps);

        if ((result = set_engine_settings(m_device, &m_engine_settings)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error setting engine settings: %1").arg(getErrorString(result)).toLatin1().data());
        }
        retval += SMCCheckError(retval);
    }

    if (!retval.containsError())
    {
        if ((result = get_move_settings(m_device, &move_settings)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error reading move settings: %1").arg(getErrorString(result)).toLatin1().data());
        }
        else
        {
            retval += m_params["device_id"].setVal<char*>(m_identifier.toLatin1().data());
            retval += m_params["accel"].setVal<int>(move_settings.Accel);
            retval += m_params["speed"].setVal<int>(move_settings.Speed);
            retval += m_params["micro_step_speed"].setVal<int>(move_settings.uSpeed);
        }

        retval += SMCCheckError(retval);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}


const ito::RetVal ST8SMC4USB::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogST8SMC4USB(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the ST8SMC4USBInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ST8SMC4USB::close(ItomSharedSemaphore *waitCond)
{
    result_t result;

    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (m_device && (result = close_device(&m_device)) != result_ok)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Error closing device: %1").arg(getErrorString(result)).toLatin1().data());
    }

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
ito::RetVal ST8SMC4USB::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        *val = it.value();
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
            If the "ctrl-type" is set, ST8SMC4USB::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal ST8SMC4USB::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    result_t result;
    bool hasIndex;
    int index = 0;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    QVector<QPair<int, QByteArray> > lastitError;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

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
        if (key == "micro_steps")
        {
            int microSteps = val->getVal<int>();
            if ((microSteps != 0) && !(microSteps & (microSteps - 1)) && microSteps <= 256) // check if an integer is a power of two
            {
                m_engine_settings.MicrostepMode = microStepsToMicrostepMode(microSteps);

                if ((retValue = set_engine_settings(m_device, &m_engine_settings)) != result_ok)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("Error setting engine settings: %1").arg(getErrorString(result)).toLatin1().data());
                }
                retValue += SMCCheckError(retValue);
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("given value must be a power of two <= 256").toLatin1().data());
            }

            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
        }

        //---------------------------
        else if (key == "async")
        {
            m_async = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }

        //---------------------------
        else if (key == "accel")
        {
            retValue += setMoveSettings(val->getVal<int>());
            retValue += it->copyValueFrom(&(*val));
        }

        //---------------------------
        else if (key == "speed")
        {
            retValue += setMoveSettings(-1, val->getVal<int>());
            retValue += it->copyValueFrom(&(*val));
        }

        //---------------------------
        else if (key == "micro_step_speed")
        {
            retValue += setMoveSettings(-1, -1, val->getVal<int>());
            retValue += it->copyValueFrom(&(*val));
        }

        //---------------------------
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
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal ST8SMC4USB::calib(const int axis, ItomSharedSemaphore *waitCond)
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
ito::RetVal ST8SMC4USB::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
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
ito::RetVal ST8SMC4USB::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = SMCCheckStatus();

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
ito::RetVal ST8SMC4USB::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    QSharedPointer<QVector<double> > pos_(new QVector<double>(1, 0.0));
    ito::RetVal retval = getPos(QVector<int>(1, axis), pos_, NULL);
    *pos = (*pos_)[0];

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
ito::RetVal ST8SMC4USB::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    result_t result;
    int gPosition = 0;

    if (axis.size() != 1 || axis[0] != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("This device have only one axis").toLatin1().data());
    }
    else
    {
        get_position_t gPos;
        double unitsPerStep = m_params["units_per_step"].getVal<double>();
        int microSteps = m_params["units_per_step"].getVal<int>();

        if ((result = get_position(m_device, &gPos)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error getting status: %1").arg(getErrorString(result)).toLatin1().data());
        }
        else
        {
            m_currentPos[0] = ((gPos.Position + (gPos.uPosition * (1.0 / (double)microSteps))) * unitsPerStep);
            (*pos)[0] = m_currentPos[0];
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
            This function calls ST8SMC4USB::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis     axis number
    \param [in] pos      absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ST8SMC4USB::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return SMCSetPos(QVector<int>(1, axis), QVector<double>(1, pos), false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls ST8SMC4USB::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ST8SMC4USB::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    return SMCSetPos(axis, pos, false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
            This function calls ST8SMC4USB::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ST8SMC4USB::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return SMCSetPos(QVector<int>(1, axis), QVector<double>(1, pos), true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls ST8SMC4USB::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ST8SMC4USB::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    return SMCSetPos(axis, pos, true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal ST8SMC4USB::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
    *sharedpos = 0.0;

//    retval += SMCCheckStatus();

    if (sendCurrentPos)
    {
        retval += getPos(0, sharedpos, 0);
        m_currentPos[0] = *sharedpos;
        
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
ito::RetVal ST8SMC4USB::SMCCheckStatus()
{
    ito::RetVal retVal = SMCCheckError(ito::retOk);

    setStatus(m_currentStatus[0], ito::actuatorAvailable, ito::actSwitchesMask | ito::actMovingMask);

    requestStatusAndPosition(true, true);
    sendStatusUpdate(false);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ST8SMC4USB::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    return setOrigin(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal ST8SMC4USB::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
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
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the position (abs or rel) of a one axis spezified by "axis" to the position "dpos". The value in device independet in mm. 
            If the axisnumber is not 0, this function returns an error.

    \param [in] axis        axis number
    \param [in] dpos        target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal ST8SMC4USB::SMCSetPos(const QVector<int> axis, const QVector<double> posUnit, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    bool released = false;
    result_t result;
    int fullSteps = 0;
    int partSteps = 0;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toLatin1().data());
    }
    else if (axis.size() != 1 || axis[0] != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("This device have only one axis").toLatin1().data());
    }
    else
    {
        double unitsPerStep = m_params["units_per_step"].getVal<double>();
        int microSteps = m_params["micro_steps"].getVal<int>();

        double distanceSteps = qRound(posUnit[0] / unitsPerStep * (double)microSteps) / microSteps;
        fullSteps = (int)(distanceSteps);
        partSteps = (int)((distanceSteps - fullSteps) * microSteps);

        setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate(false);

        if (relNotAbs)
        {   // Relative movement
            if ((result = command_movr(m_device, fullSteps, partSteps)) != result_ok)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Error while moving: %1").arg(getErrorString(result)).toLatin1().data());
            }
        }
        else
        {   // Absolute movement
            if ((result = command_move(m_device, fullSteps, partSteps)) != result_ok)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Error while moving: %1").arg(getErrorString(result)).toLatin1().data());
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
        retval += waitForDone(100, axis); 
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

    if (!retval.containsError())
    {
        QSharedPointer<double> pos(new double);
        retval += getPos(0, pos, NULL);
        m_targetPos[0] = m_currentPos[0];

        qDebug() << m_targetPos << m_currentPos << m_currentStatus;
        sendTargetUpdate();
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
ito::RetVal ST8SMC4USB::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retval(ito::retOk);
    bool done = false;
    bool stillMoving;
    result_t result;
    status_t state;

    Sleep(10);
    
    while (!done && !retval.containsWarningOrError())
    {   

        if ((result = get_status(m_device, &state)) != result_ok)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Error getting status: %1").arg(getErrorString(result)).toLatin1().data());
        }

//        qDebug() << state.MoveSts << state.MvCmdSts << m_currentStatus;

        retval += SMCCheckStatus();

        if (isInterrupted())
        {
            if ((result = command_stop(m_device)) != result_ok)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Error while moving: %1").arg(getErrorString(result)).toLatin1().data());
            }
            
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);

            retval += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;

            sendStatusUpdate(true);
        }
        else
        {
            done = ((state.MoveSts & MOVE_STATE_MOVING) == 0) && (state.MvCmdSts & MVCMD_RUNNING) == 0;

//            qDebug() << "state.MoveSts " << state.MoveSts;
         
            if (done)
            {   // Position reached and movement done
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
                sendStatusUpdate(true);
                break;
            }

            if (!retval.containsError())
            {
                setAlive();
            }
        }
    }
    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void ST8SMC4USB::dockWidgetVisibilityChanged(bool visible)
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

