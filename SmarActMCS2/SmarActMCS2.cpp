/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "SmarActMCS2.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include <qdatetime.h>
#include <qwaitcondition.h>

#include "common/helperCommon.h"
#include "common/paramMeta.h"

#include "dockWidgetSmarActMCS2.h"

#include <iostream>

#include "SmarActControl.h"

QList<QString> SmarActMCS2::openedDevices = QList<QString>();

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
SmarActMCS2Interface::SmarActMCS2Interface()
{
    m_type = ito::typeActuator;
    setObjectName("SmarActMCS2");

    m_description = QObject::tr("SmarActMCS2");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This template can be used for implementing a new type of actuator plugin \n\
\n\
Put a detailed description about what the plugin is doing, what is needed to get it started, limitations...";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("The plugin's license string");
    m_aboutThis = QObject::tr(GITVERSION);

    //optional parameter
    m_initParamsOpt.append(ito::Param(
        "serialNo",
        ito::ParamBase::String,
        "",
        tr("Serial number of the device to be loaded. If empty, the first device that can be "
           "opened will be opened. (e.g.: network:sn:MCS2-00012345")
            .toLatin1()
            .data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!

*/
SmarActMCS2Interface::~SmarActMCS2Interface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SmarActMCS2Interface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(SmarActMCS2) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SmarActMCS2Interface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(SmarActMCS2) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(SmarActMCS2interface, SmarActMCS2Interface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif




//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
SmarActMCS2::SmarActMCS2() : AddInActuator(), m_async(0), m_nrOfAxes(1)
{
    ito::IntMeta* imeta;

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "SmarActMCS2", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    m_params.insert(
        "async",
        ito::Param(
            "async",
            ito::ParamBase::Int,
            0,
            1,
            m_async,
            tr("asynchronous move (1), synchronous (0) [default]").toLatin1().data()));

    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Serial number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Device Name.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "interfaceType",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Interface Type (USB or ETHERNET).").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "noOfBusModules",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        std::numeric_limits<ito::int32>::max(),
        0,
        tr("Number of Bus Modules.")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<ito::int32>::max(), 1, "Device info"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "noOfChannels",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        std::numeric_limits<ito::int32>::max(),
        0,
        tr("Number of Channels.")
            .toUtf8()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<ito::int32>::max(), 1, "Device info"));
    m_params.insert(paramVal.getName(), paramVal);

    // Control Parameters

    paramVal = ito::Param(
        "velocity",
        ito::ParamBase::DoubleArray,
        NULL,
        tr("Velocity of each axis, default 10 mm/s.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "acceleration",
        ito::ParamBase::DoubleArray,
        NULL,
        tr("Acceleration of each axis, default 100 mm/s^2.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "baseUnit",
        ito::ParamBase::IntArray | ito::ParamBase::Readonly,
        NULL,
        tr("Baseunit of Channel: (0) for none, (1) for millimeter and (2) for degree.")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "sensorPresent",
        ito::ParamBase::IntArray | ito::ParamBase::Readonly,
        NULL,
        tr("Show if sensor is present (1) or not (0).")
            .toLatin1()
            .data());
    m_params.insert(paramVal.getName(), paramVal);

    // Limits Parameters

    paramVal = ito::Param(
        "useLimits",
        ito::ParamBase::IntArray,
        NULL,
        tr("Use axes limits of axis (1) or not (0).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "limitUpper",
        ito::ParamBase::DoubleArray,
        NULL,
        tr("Upper limits of axes.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "limitLower",
        ito::ParamBase::DoubleArray,
        NULL,
        tr("Lower limits of axes.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //initialize the current position vector, the status vector and the target position vector
    m_currentPos.fill(0.0,m_nrOfAxes);
    m_currentStatus.fill(0,m_nrOfAxes);
    m_targetPos.fill(0.0,m_nrOfAxes);
    m_factor.fill(1, m_nrOfAxes);

    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetSmarActMCS2 *dw = new DockWidgetSmarActMCS2(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
SmarActMCS2::~SmarActMCS2()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal SmarActMCS2::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    SA_CTL_Result_t result;

    QString serialNo = paramsOpt->at(0).getVal<const char*>();

    if (openedDevices.contains(serialNo))
    {
        retValue += ito::RetVal(
            ito::retError,
            1,
            QObject::tr("Device at SerialNo %1 is already connected.")
                .arg(serialNo)
                .toLatin1()
                .data());
    }

    char deviceList[1024];
    size_t ioDeviceListLen = sizeof(deviceList);

    if (!retValue.containsError())
        {
        result = SA_CTL_FindDevices("", deviceList, &ioDeviceListLen);
        if (result != SA_CTL_ERROR_NONE)
        {
            retValue = ito::RetVal(
                ito::retError,
                0,
                tr("MCS2 failed to find devices.")
                    .toLatin1()
                    .data());
        }
        if (strlen(deviceList) == 0)
        {
            retValue =
                ito::RetVal(ito::retError, 0, tr("MCS2 no devices found.").toLatin1().data());
        }
    }

    char* ptr;
    char* snBuf = nullptr;
   
    if (!retValue.containsError())
    {
        snBuf = strtok_s(deviceList, "\n", &ptr);

        if (serialNo != "")
        {
            bool matched = false;
            while (snBuf != nullptr)
            {
                if (snBuf == serialNo)
                {
                    matched = true;
                    break;
                }
                snBuf = strtok_s(nullptr, "\n", &ptr);
            }

            if (matched == false)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("MCS2 could not find given serial-number: \"%1\".\n").arg(serialNo).toLatin1().data());
            }
        }
    }

    if (!retValue.containsError())
    {        
        result = SA_CTL_Open(&m_insrumentHdl, snBuf, "");
        if (result != SA_CTL_ERROR_NONE)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("MCS2 failed to open \"%1\".\n").arg(snBuf).toLatin1().data());
        }
    }

    int32_t type, noOfBusModules, noOfChannels;

    if (!retValue.containsError())
    {
        m_params["serialNumber"].setVal<char*>(snBuf);
        openedDevices.append(snBuf);

        char buf[SA_CTL_STRING_MAX_LENGTH + 1];
        size_t ioStringSize = sizeof(buf);
        
        result =
            SA_CTL_GetProperty_s(m_insrumentHdl, 0, SA_CTL_PKEY_DEVICE_NAME, buf, &ioStringSize);
        if (result != SA_CTL_ERROR_NONE)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("MCS2 error getting devive information.\n").toLatin1().data());
        }
        else
            m_params["deviceName"].setVal<char*>(buf);
    }

    if (!retValue.containsError())
    {
        result = SA_CTL_GetProperty_i32(m_insrumentHdl, 0, SA_CTL_PKEY_INTERFACE_TYPE, &type, 0);
        if (result != SA_CTL_ERROR_NONE)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("MCS2 error getting devive information.\n").toLatin1().data());
        }
        else
        {
            if (type == SA_CTL_INTERFACE_USB)
                m_params["interfaceType"].setVal<char*>(const_cast<char*>("USB"));
            else if (type == SA_CTL_INTERFACE_ETHERNET)
                m_params["interfaceType"].setVal<char*>(const_cast<char*>("ETHERNET"));
        }
    }

    if (!retValue.containsError())
    {
        result = SA_CTL_GetProperty_i32(
            m_insrumentHdl, 0, SA_CTL_PKEY_NUMBER_OF_BUS_MODULES, &noOfBusModules, 0);
        if (result != SA_CTL_ERROR_NONE)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("MCS2 error getting devive information.\n").toLatin1().data());
        }
        else
            m_params["noOfBusModules"].setVal<int>(noOfBusModules);
    }

    if (!retValue.containsError())
    {
        result = SA_CTL_GetProperty_i32(
            m_insrumentHdl, 0, SA_CTL_PKEY_NUMBER_OF_CHANNELS, &noOfChannels, 0);
        if (result != SA_CTL_ERROR_NONE)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("MCS2 error getting devive information.\n").toLatin1().data());
        }
        else
        {
            m_params["noOfChannels"].setVal<int>(noOfChannels);
            m_nrOfAxes = noOfChannels;

            m_currentPos.resize(m_nrOfAxes);
            m_currentStatus.resize(m_nrOfAxes);
            m_targetPos.resize(m_nrOfAxes);
            m_factor.resize(m_nrOfAxes);

            m_params["sensorPresent"].setMeta(
                new ito::IntArrayMeta(0, 1, 1, 0, m_nrOfAxes, 1, "Control"), true);
            int* sensorPresent = new int[m_nrOfAxes];

            for (int i = 0; i < m_currentPos.size(); i++)
            {
                SA_CTL_Result_t result;

                uint32_t channelState;
                result = SA_CTL_GetProperty_i32(
                    m_insrumentHdl, i, SA_CTL_PKEY_CHANNEL_STATE, (int32_t*)&channelState, 0);
                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to get channel state of axis \"%1\".\n")
                            .arg(i)
                            .toLatin1()
                            .data());
                    break;
                }
                else
                {
                    sensorPresent[i] = ((channelState & SA_CTL_CH_STATE_BIT_SENSOR_PRESENT) != 0);
                }
            }
            m_params["sensorPresent"].setVal<int*>(sensorPresent, m_nrOfAxes);
        }


        setIdentifier(snBuf);
    }

    // get base unit and factor of each channel:
    if (!retValue.containsError())
    {
        m_params["baseUnit"].setMeta(
            new ito::IntArrayMeta(0, 1, 1, 0, m_nrOfAxes, 1, "Control"), true);
        int32_t buf;
        int* baseUnit = new int[m_nrOfAxes];
        for (int i = 0; i < m_nrOfAxes; ++i)
        {
            result = SA_CTL_GetProperty_i32(m_insrumentHdl, i, SA_CTL_PKEY_POS_BASE_UNIT, &buf, 0);
            if (result != SA_CTL_ERROR_NONE)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("MCS2 failed to get base unit of axis \"%1\".\n").arg(i).toLatin1().data());
            }
            else
            {
                switch (buf)
                {
                case 0:
                    baseUnit[i] = 0;
                    m_factor[i] = 1;
                    break;
                case 2: //millimeter to pm
                    baseUnit[i] = 1;
                    m_factor[i] = 1e9;
                    break;
                case 3: //degree to nano degree
                    baseUnit[i] = 2;
                    m_factor[i] = 1e9;
                    break;
                default:
                    baseUnit[i] = 0;
                    m_factor[i] = 1;
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 got wrong baseUnit: \"%1\".\n")
                            .arg(buf)
                            .toLatin1()
                            .data());
                    break;
                }
                m_params["baseUnit"].setVal<int*>(baseUnit, m_nrOfAxes);
            }
        }
    }

    if (!retValue.containsError())
    {
        for (int i = 0; i < m_currentPos.size(); i++)
        {
            if (m_params["sensorPresent"].getVal<int*>()[i] != 0)
            {
                SA_CTL_Result_t result;

                int64_t position;
                result =
                    SA_CTL_GetProperty_i64(m_insrumentHdl, i, SA_CTL_PKEY_POSITION, &position, 0);
                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to get position of axis \"%1\".\n")
                            .arg(i)
                            .toLatin1()
                            .data());
                    break;
                }
                else
                {
                    m_currentPos[i] = static_cast<double>(position) / m_factor[i];
                    m_currentStatus[i] =
                        ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
                    m_targetPos[i] = static_cast<double>(position) / m_factor[i];
                }
            }
        }
    }

    if (!retValue.containsError())
    {
        m_params["velocity"].setMeta(
            new ito::DoubleArrayMeta(0, 100, 0.001, 0, m_nrOfAxes, 1, "Control"), true);
        m_params["acceleration"].setMeta(
            new ito::DoubleArrayMeta(0, 100, 0.001, 0, m_nrOfAxes, 1, "Control"), true);
        double* velocity = new double[m_nrOfAxes];
        double* acceleration = new double[m_nrOfAxes];
        for (int i = 0; i < m_nrOfAxes; ++i)
        {
            //set default value of velocity and acceleration
            velocity[i] = 10; // default value 10 mm/s (10 degree/s)
            result = SA_CTL_SetProperty_i64(
                m_insrumentHdl, i, SA_CTL_PKEY_MOVE_VELOCITY, velocity[i] * m_factor[i]);
            if (result != SA_CTL_ERROR_NONE)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("MCS2 error setting velocity.\n").toLatin1().data());
                break;
            }

            acceleration[i] = 100; //default value 100 mm/s^2 (100 degree/s^2)
            result = SA_CTL_SetProperty_i64(
                m_insrumentHdl, i, SA_CTL_PKEY_MOVE_ACCELERATION, acceleration[i] * m_factor[i]);
            if (result != SA_CTL_ERROR_NONE)
            {
                retValue += ito::RetVal(
                    ito::retError, 0, tr("MCS2 error setting acceleration.\n").toLatin1().data());
                break;
            }
        }
        m_params["velocity"].setVal<double*>(velocity, m_nrOfAxes);
        m_params["acceleration"].setVal<double*>(acceleration, m_nrOfAxes);
        DELETE_AND_SET_NULL_ARRAY(velocity);
        DELETE_AND_SET_NULL_ARRAY(acceleration);
    }

    if (!retValue.containsError())
    {
        m_params["useLimits"].setMeta(
            new ito::IntArrayMeta(0, 1, 1, 0, m_nrOfAxes, 1, "Limits"), true);
        m_params["limitUpper"].setMeta(
            new ito::DoubleArrayMeta(
                -std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                0.001,
                0,
                m_nrOfAxes,
                1,
                "Limits"),
            true);
        m_params["limitLower"].setMeta(
            new ito::DoubleArrayMeta(
                -std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                0.001,
                0,
                m_nrOfAxes,
                1,
                "Limits"),
            true);

        int* useLimits = new int[m_nrOfAxes]();
        double* limitUpper = new double[m_nrOfAxes];
        double* limitLower = new double[m_nrOfAxes];

        for (int i = 0; i < m_nrOfAxes; ++i)
        {
            limitUpper[i] = std::numeric_limits<double>::max();
            limitLower[i] = -std::numeric_limits<double>::max();
        }

        m_params["useLimits"].setVal<int*>(useLimits, m_nrOfAxes);
        m_params["limitUpper"].setVal<double*>(limitUpper, m_nrOfAxes);
        m_params["limitLower"].setVal<double*>(limitLower, m_nrOfAxes);
    }

    if (!retValue.containsError())
    {
        // register exec functions ------------------------------------

        QVector<ito::Param> pMand = QVector<ito::Param>();
        QVector<ito::Param> pOpt = QVector<ito::Param>();
        QVector<ito::Param> pOut = QVector<ito::Param>();

        pOpt << ito::Param(
            "axis",
            ito::ParamBase::Int,
            0,
            m_nrOfAxes - 1,
            -1,
            tr("axis to perform SmarAct calibration").toLatin1().data());

        registerExecFunc(
            "SmaractCalibrate", pMand, pOpt, pOut, tr("Perform the SmarAct calibration function."));
        pMand.clear();
        pOpt.clear();
        pOut.clear();

        // end register exec functions --------------------------------
    }


    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal SmarActMCS2::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    SA_CTL_Close(m_insrumentHdl);
    openedDevices.removeOne(m_params["serialNumber"].getVal<char*>());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SmarActMCS2::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
    {
        //put your switch-case.. for getting the right value here

        //finally, save the desired value in the argument val (this is a shared pointer!)
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
ito::RetVal SmarActMCS2::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    SA_CTL_Result_t result;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(isMotorMoving()) //this if-case is for actuators only.
    {
        retValue += ito::RetVal(ito::retError, 0, tr("any axis is moving. Parameters cannot be set.").toLatin1().data());
    }

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
        if(key == "async")
        {
            m_async = val->getVal<int>();
        }
        else if (key == "velocity")
        {
            if (val->getLen() == m_nrOfAxes)
            {
                double* data = val->getVal<double*>();
                for (int i = 0; i < m_nrOfAxes; ++i)
                {
                    result = SA_CTL_SetProperty_i64(
                        m_insrumentHdl,
                        i,
                        SA_CTL_PKEY_MOVE_VELOCITY, static_cast<int64_t>(data[i] * m_factor[i]));
                    if (result != SA_CTL_ERROR_NONE)
                    {
                        retValue += ito::RetVal(
                            ito::retError,
                            0,
                            tr("MCS2 error setting velocity.\n").toLatin1().data());
                        break;
                    }
                }
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("array (%1) must be the same size like the number of axis (%2).\n")
                        .arg(val->getLen())
                        .arg(m_nrOfAxes)
                        .toLatin1()
                        .data());
            }
        }
        else if (key == "acceleration")
        {
            if (val->getLen() == m_nrOfAxes)
            {
                double* data = val->getVal<double*>();
                for (int i = 0; i < m_nrOfAxes; ++i)
                {
                    result = SA_CTL_SetProperty_i64(
                        m_insrumentHdl,
                        i,
                        SA_CTL_PKEY_MOVE_ACCELERATION, static_cast<int64_t>(data[i] * m_factor[i]));
                    if (result != SA_CTL_ERROR_NONE)
                    {
                        retValue += ito::RetVal(
                            ito::retError,
                            0,
                            tr("MCS2 error setting acceleration.\n").toLatin1().data());
                        break;
                    }
                }
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("array (%1) must be the same size like the number of axis (%2).\n")
                        .arg(val->getLen())
                        .arg(m_nrOfAxes)
                        .toLatin1()
                        .data());
            }
        }
        else if (key == "limitLower")
        {
            if (val->getLen() == m_nrOfAxes)
            {
                double* data = val->getVal<double*>();
                for (int i = 0; i < m_nrOfAxes; ++i)
                {
                    if (data[i] > m_params["limitUpper"].getVal<double*>()[i])
                    {
                        retValue += ito::RetVal(
                            ito::retError,
                            0,
                            tr("Lower limit (%1) cannot be higher than upper limit (%2).\n")
                                .arg(data[i])
                                .arg(m_params["limitUpper"].getVal<double*>()[i])
                                .toLatin1()
                                .data());
                    }
                }
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("array (%1) must be the same size like the number of axis (%2).\n")
                        .arg(val->getLen())
                        .arg(m_nrOfAxes)
                        .toLatin1()
                        .data());
            }
        }
        else if (key == "limitUpper")
        {
            if (val->getLen() == m_nrOfAxes)
            {
                double* data = val->getVal<double*>();
                for (int i = 0; i < m_nrOfAxes; ++i)
                {
                    if (data[i] < m_params["limitLower"].getVal<double*>()[i])
                    {
                        retValue += ito::RetVal(
                            ito::retError,
                            0,
                            tr("Upper limit (%1) cannot be lower than upper limit (%2).\n")
                                .arg(data[i])
                                .arg(m_params["limitLower"].getVal<double*>()[i])
                                .toLatin1()
                                .data());
                    }
                }
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("array (%1) must be the same size like the number of axis (%2).\n")
                        .arg(val->getLen())
                        .arg(m_nrOfAxes)
                        .toLatin1()
                        .data());
            }
        }
        
        if (!retValue.containsError())
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom( &(*val) );
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
//! calib
/*!
    the given axis should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal SmarActMCS2::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! calib
/*!
    the given axes should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal SmarActMCS2::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    SA_CTL_Result_t result;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        for (int i = 0; i < axis.size(); i++)
        {
            if (axis[i] < 0 || axis[i] >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                //reference in MCS2 is the calibration function

                result = SA_CTL_SetProperty_i32(
                    m_insrumentHdl, axis[i], SA_CTL_PKEY_REFERENCING_OPTIONS, 0);

                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to do calibration of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }

                result = SA_CTL_Reference(m_insrumentHdl, axis[i], 0);

                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to do calibration of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }
            }
        }
    }

    if (!retValue.containsError())
    {
        bool done = false;
        bool timeout = false;
        QElapsedTimer timer;
        QMutex waitMutex;
        QWaitCondition waitCondition;
        long delay = 100; //[ms]
        const int timeoutMS = 60000; //Reference can take a lot of time

        timer.start();

        while (!done && !timeout)
        {
            done = true; // assume all axes at target

            for (int i = 0; i < axis.size(); i++)
            {
                SA_CTL_Result_t result;
                int32_t state;
                result = SA_CTL_GetProperty_i32(
                    m_insrumentHdl, axis[i], SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
                if (result == SA_CTL_ERROR_NONE)
                {
                    // usebitmaskingto determine thechannelsmovement state
                    if (!((state & SA_CTL_CH_STATE_BIT_REFERENCING) == 0))
                    {
                        setStatus(
                            m_currentStatus[axis[i]],
                            ito::actuatorMoving,
                            ito::actSwitchesMask | ito::actStatusMask);
                        done = false;
                    }
                    else
                    {
                        setStatus(
                            m_currentStatus[axis[i]],
                            ito::actuatorAtTarget,
                            ito::actSwitchesMask | ito::actStatusMask);
                    }
                }
            }

            // emit actuatorStatusChanged with both m_currentStatus and m_currentPos as arguments
            sendStatusUpdate(false);

            // short delay
            waitMutex.lock();
            waitCondition.wait(&waitMutex, delay);
            waitMutex.unlock();

            // raise the alive flag again, this is necessary such that itom does not drop into a
            // timeout if the positioning needs more time than the allowed timeout time.
            setAlive();

            if (timeoutMS > -1)
            {
                if (timer.elapsed() > timeoutMS)
                    timeout = true;
            }
        }

        if (timeout)
        {
            // timeout occurred, set the status of all currently moving axes to timeout
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retValue += ito::RetVal(ito::retError, 9999, "timeout occurred");
            sendStatusUpdate(true);
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
//! setOrigin
/*!
    the given axis should be set to origin. That means (if possible) its current position should be
    considered to be the new origin (zero-position). If this operation is not possible, return a
    warning.
*/
ito::RetVal SmarActMCS2::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    return setOrigin(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setOrigin
/*!
    the given axes should be set to origin. That means (if possible) their current position should be
    considered to be the new origin (zero-position). If this operation is not possible, return a
    warning.
*/
ito::RetVal SmarActMCS2::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += ito::RetVal(
        ito::retWarning,
        0,
        tr("Not needed and not implemented for this actuator.").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! getStatus
/*!
    re-checks the status (current position, available, end switch reached, moving, at target...) of all axes and
    returns the status of each axis as vector. Each status is an or-combination of the enumeration ito::tActuatorStatus.
*/
ito::RetVal SmarActMCS2::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += updateStatus();
    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! getPos
/*!
    returns the current position in pico meter [pm] for linear positioners or nano degree [ndeg] for rotatory positioners of the given axis
*/
ito::RetVal SmarActMCS2::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QSharedPointer<QVector<double> > pos2(new QVector<double>(1,0.0));
    ito::RetVal retValue = getPos(QVector<int>(1,axis), pos2, NULL); //forward to multi-axes version
    *pos = (*pos2)[0];

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! getPos
/*!
    returns the current position in meter for linear positioners or degree for rotatory positioners of all given axes
*/
ito::RetVal SmarActMCS2::getPos(QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    for (int i = 0; i < axis.size(); i++)
    {
        if (m_params["sensorPresent"].getVal<int*>()[axis[i]] == 0)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("MCS2 failed to get position of axis \"%1\": Sensor not present\n").arg(axis[i]).toLatin1().data());
            continue;
        }
        if (axis[i] >= 0 && axis[i] < m_nrOfAxes)
        {
            SA_CTL_Result_t result;

            int64_t position;
            result =
                SA_CTL_GetProperty_i64(m_insrumentHdl, axis[i], SA_CTL_PKEY_POSITION, &position, 0);
            if (result != SA_CTL_ERROR_NONE)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("MCS2 failed to get position of axis \"%1\".\n")
                        .arg(axis[i])
                        .toLatin1()
                        .data());
            }
            else
            {
                m_currentPos[axis[i]] = static_cast<double>(position) / m_factor[axis[i]];
                (*pos)[i] = m_currentPos[axis[i]];
            }
        }
        else
        {
            retValue += ito::RetVal::format(
                ito::retError,
                1,
                tr("axis %i not available. Only axis between 0 and %i").toLatin1().data(),
                i,
                m_nrOfAxes - 1);
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
//! setPosAbs
/*!
    starts moving the given axis to the desired absolute target position

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if the axis reached the given target position (async = 0)

    In some cases only relative movements are possible, then get the current position, determine the
    relative movement and call the method relatively move the axis.
*/
ito::RetVal SmarActMCS2::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosAbs(QVector<int>(1,axis), QVector<double>(1,pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosAbs
/*!
    starts moving all given axes to the desired absolute target positions

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if all axes reached their given target positions (async = 0)

    In some cases only relative movements are possible, then get the current position, determine the
    relative movement and call the method relatively move the axis.
*/
ito::RetVal SmarActMCS2::setPosAbs(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        for (int i = 0; i < axis.size(); i++)
        {
            if (axis[i] < 0 || axis[i] >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(), axis[i]);
            }
            else
            {
                if (m_params["useLimits"].getVal<int*>()[axis[i]] == 1)
                {
                    if (pos[i] < m_params["limitLower"].getVal<double*>()[axis[i]] ||
                        pos[i] > m_params["limitUpper"].getVal<double*>()[axis[i]])
                    {
                        retValue += ito::RetVal::format(
                            ito::retError,
                            1,
                            tr("axis %1 out of limit [%2, %3]: %4")
                                .arg(axis[i])
                                .arg(m_params["limitLower"].getVal<double*>()[axis[i]])
                                .arg(m_params["limitUpper"].getVal<double*>()[axis[i]])
                                .arg(pos[i])
                                .toLatin1()
                                .data());
                        break;
                    }
                }
                m_targetPos[axis[i]] = pos[i] * m_factor[axis[i]];
            }
        }

        if (!retValue.containsError())
        {
            //set status of all given axes to moving and keep all flags related to the status and switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

            for (int i = 0; i < axis.size(); i++)
            {
                SA_CTL_Result_t result;

                result = SA_CTL_SetProperty_i32(
                    m_insrumentHdl, axis[i], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to set absolute moving of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }
                else
                {
                    result = SA_CTL_Move(m_insrumentHdl, axis[i], m_targetPos[axis[i]], 0);
                }

                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to start moving of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }
            }

            //emit the signal targetChanged with m_targetPos as argument, such that all connected slots gets informed about new targets
            sendTargetUpdate();

            //emit the signal sendStatusUpdate such that all connected slots gets informed about changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();

            //release the wait condition now, if async is true (itom considers this method to be finished now due to the threaded call)
            if(m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            //call waitForDone in order to wait until all axes reached their target or a given timeout expired
            //the m_currentPos and m_currentStatus vectors are updated within this function
            retValue += waitForDone(60000, axis); //WaitForAnswer(60000, axis);

            //release the wait condition now, if async is false (itom waits until now if async is false, hence in the synchronous mode)
            if(!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    //if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosRel
/*!
    starts moving the given axis by the given relative distance

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if the axis reached the given target position (async = 0)

    In some cases only absolute movements are possible, then get the current position, determine the
    new absolute target position and call setPosAbs with this absolute target position.
*/
ito::RetVal SmarActMCS2::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosRel(QVector<int>(1,axis), QVector<double>(1,pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosRel
/*!
    starts moving the given axes by the given relative distances

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if all axes reached the given target positions (async = 0)

    In some cases only absolute movements are possible, then get the current positions, determine the
    new absolute target positions and call setPosAbs with these absolute target positions.
*/
ito::RetVal SmarActMCS2::setPosRel(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        for (int i = 0; i < axis.size(); i++)
        {
            if (axis[i] < 0 || axis[i] >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), axis[i]);
            }
            else
            {
                for (int i = 0; i < axis.size(); i++)
                {
                    if (axis[i] < 0 || axis[i] >= m_nrOfAxes)
                    {
                        retValue += ito::RetVal::format(
                            ito::retError,
                            1,
                            tr("axis %i not available").toLatin1().data(),
                            axis[i]);
                    }
                    else
                    {
                        if (m_params["useLimits"].getVal<int*>()[axis[i]] == 1)
                        {
                            if (m_currentPos[axis[i]] + pos[i] <
                                    m_params["limitLower"].getVal<double*>()[axis[i]] ||
                                m_currentPos[axis[i]] + pos[i] >
                                    m_params["limitUpper"].getVal<double*>()[axis[i]])
                            {
                                retValue += ito::RetVal::format(
                                    ito::retError,
                                    1,
                                    tr("axis %1 out of limit [%2, %3]: %4")
                                        .arg(axis[i])
                                        .arg(m_params["limitLower"].getVal<double*>()[axis[i]])
                                        .arg(m_params["limitUpper"].getVal<double*>()[axis[i]])
                                        .arg(m_currentPos[axis[i]] + pos[i])
                                        .toLatin1()
                                        .data());
                                break;
                            }
                        }
                        m_targetPos[axis[i]] = (m_currentPos[axis[i]] + pos[i]) * m_factor[axis[i]];
                    }
                }
            }
        }

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

            for (int i = 0; i < axis.size(); i++)
            {
                SA_CTL_Result_t result;

                result = SA_CTL_SetProperty_i32(
                    m_insrumentHdl, axis[i], SA_CTL_PKEY_MOVE_MODE, SA_CTL_MOVE_MODE_CL_ABSOLUTE);
                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to set absolute moving of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }
                else
                {
                    result = SA_CTL_Move(m_insrumentHdl, axis[i], m_targetPos[axis[i]], 0);
                }

                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to start moving of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }
            }

            //emit the signal targetChanged with m_targetPos as argument, such that all connected slots gets informed about new targets
            sendTargetUpdate();

            //emit the signal sendStatusUpdate such that all connected slots gets informed about changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();

            //release the wait condition now, if async is true (itom considers this method to be finished now due to the threaded call)
            if(m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            //call waitForDone in order to wait until all axes reached their target or a given timeout expired
            //the m_currentPos and m_currentStatus vectors are updated within this function
            retValue += waitForDone(60000, axis); //WaitForAnswer(60000, axis);

            //release the wait condition now, if async is false (itom waits until now if async is false, hence in the synchronous mode)
            if(!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    //if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method must be overwritten from ito::AddInActuator
/*!
    WaitForDone should wait for a moving motor until the indicated axes (or all axes of nothing is indicated) have stopped or a timeout or user interruption
    occurred. The timeout can be given in milliseconds, or -1 if no timeout should be considered. The flag-parameter can be used for your own purpose.
*/
ito::RetVal SmarActMCS2::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    char motor;
    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    long delay = 100; //[ms]

    timer.start();

    //if axis is empty, all axes should be observed by this method
    QVector<int> _axis = axis;
    if (_axis.size() == 0) //all axis
    {
        for (int i=0;i<m_nrOfAxes;i++)
        {
            _axis.append(i);
        }
    }

    while (!done && !timeout)
    {
        done = true; //assume all axes at target

        for (int i = 0; i < _axis.size(); i++)
        {
            SA_CTL_Result_t result;
            int32_t state;
            result = SA_CTL_GetProperty_i32(
                m_insrumentHdl, _axis[i], SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
            if (result == SA_CTL_ERROR_NONE)
            {
                //get current position
                SA_CTL_Result_t result;

                int64_t position;
                result = SA_CTL_GetProperty_i64(
                    m_insrumentHdl, axis[i], SA_CTL_PKEY_POSITION, &position, 0);
                if (result != SA_CTL_ERROR_NONE)
                {
                    retVal += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to get position of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }
                else
                {
                    m_currentPos[axis[i]] = static_cast<double>(position) / m_factor[axis[i]];
                }

                // use bit masking to determine thechannelsmovement state
                if (!((state & SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING) == 0))
                {
                    setStatus(
                        m_currentStatus[_axis[i]],
                        ito::actuatorMoving,
                        ito::actSwitchesMask | ito::actStatusMask);
                    done = false;
                }
                else
                {
                    setStatus(
                        m_currentStatus[_axis[i]],
                        ito::actuatorAtTarget,
                        ito::actSwitchesMask | ito::actStatusMask);
                }
            }
            else
            {
                retVal += ito::RetVal(
                    ito::retError,
                    0,
                    tr("MCS2 error occured during check state\n")
                        .toLatin1()
                        .data());
            }
        }

        //emit actuatorStatusChanged with both m_currentStatus and m_currentPos as arguments
        sendStatusUpdate(false);

        //now check if the interrupt flag has been set (e.g. by a button click on its dock widget)
        if (!done && isInterrupted())
        {
            //todo: force all axes to stop

            //set the status of all axes from moving to interrupted (only if moving was set before)
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);

            retVal += ito::RetVal(ito::retError,0,"interrupt occurred");
            done = true;
            return retVal;
        }

        //short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, delay);
        waitMutex.unlock();

        //raise the alive flag again, this is necessary such that itom does not drop into a timeout if the
        //positioning needs more time than the allowed timeout time.
        setAlive();

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS) timeout = true;
        }
    }

    if (timeout)
    {
        //timeout occurred, set the status of all currently moving axes to timeout
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError,9999,"timeout occurred");
        sendStatusUpdate(true);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method obtains the current position, status of all axes
/*!
    This is a helper function, it is not necessary to implement a function like this, but it might help.
*/
ito::RetVal SmarActMCS2::updateStatus()
{
    for (int i=0;i<m_nrOfAxes;i++)
    {
        m_currentStatus[i] = m_currentStatus[i] | ito::actuatorAvailable; //set this if the axis i is available, else use
        //m_currentStatus[i] = m_currentStatus[i] ^ ito::actuatorAvailable;

        m_currentPos[i] = 0.0; //todo fill in here the current position of axis i in mm or degree

        //if you know that the axis i is at its target position, change from moving to target if moving has been set, therefore:
        replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget);

        //if you know that the axis i is still moving, set this bit (all other moving-related bits are unchecked, but the status bits and switches bits
        //kept unchanged
        setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
    }

    //emit actuatorStatusChanged with m_currentStatus and m_currentPos in order to inform connected slots about the current status and position
    sendStatusUpdate();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void SmarActMCS2::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), widget, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            connect(this, SIGNAL(targetChanged(QVector<double>)), widget, SLOT(targetChanged(QVector<double>)));

            emit parametersChanged(m_params);
            sendTargetUpdate();
            sendStatusUpdate(false);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), widget, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            disconnect(this, SIGNAL(targetChanged(QVector<double>)), widget, SLOT(targetChanged(QVector<double>)));
        }
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SmarActMCS2::execFunc(
    const QString funcName,
    QSharedPointer<QVector<ito::ParamBase>> paramsMand,
    QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
    QSharedPointer<QVector<ito::ParamBase>> paramsOut,
    ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;

    ito::ParamBase* param1 = nullptr;

    if (funcName == "SmaractCalibrate")
    {
        param1 = ito::getParamByName(&(*paramsOpt), "axis", &retValue);

        int axisVal = param1->getVal<int>();

        QVector<int> axis;

        if (axisVal != -1)
        {
            axis.push_back(axisVal);
        }
        else
        {
            for (int i = 0; i < m_nrOfAxes; ++i)
            {
                axis.push_back(i);
            }
        }

        

        SA_CTL_Result_t result;

        for (int i = 0; i < axis.size(); i++)
        {
            if (axis[i] < 0 || axis[i] > m_nrOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                // reference in MCS2 is the calibration function

                result = SA_CTL_SetProperty_i32(
                    m_insrumentHdl, axis[i], SA_CTL_PKEY_CALIBRATION_OPTIONS, 0);

                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to do Smaract calibration of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }

                result = SA_CTL_Calibrate(m_insrumentHdl, axis[i], 0);

                if (result != SA_CTL_ERROR_NONE)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        tr("MCS2 failed to do Smaract calibration of axis \"%1\".\n")
                            .arg(axis[i])
                            .toLatin1()
                            .data());
                }
            }
        }
        if (!retValue.containsError())
        {
            bool done = false;
            bool timeout = false;
            QElapsedTimer timer;
            QMutex waitMutex;
            QWaitCondition waitCondition;
            long delay = 100; //[ms]
            const int timeoutMS = 60000; // Reference can take a lot of time

            timer.start();

            while (!done && !timeout)
            {
                done = true; // assume all axes at target

                for (int i = 0; i < axis.size(); i++)
                {
                    SA_CTL_Result_t result;
                    int32_t state;
                    result = SA_CTL_GetProperty_i32(
                        m_insrumentHdl, axis[i], SA_CTL_PKEY_CHANNEL_STATE, &state, 0);
                    if (result == SA_CTL_ERROR_NONE)
                    {
                        // usebitmaskingto determine thechannelsmovement state
                        if (!((state & SA_CTL_CH_STATE_BIT_CALIBRATING) == 0))
                        {
                            setStatus(
                                m_currentStatus[axis[i]],
                                ito::actuatorMoving,
                                ito::actSwitchesMask | ito::actStatusMask);
                            done = false;
                        }
                        else
                        {
                            setStatus(
                                m_currentStatus[axis[i]],
                                ito::actuatorAtTarget,
                                ito::actSwitchesMask | ito::actStatusMask);
                        }
                    }
                }

                // emit actuatorStatusChanged with both m_currentStatus and m_currentPos as
                // arguments
                sendStatusUpdate(false);

                // short delay
                waitMutex.lock();
                waitCondition.wait(&waitMutex, delay);
                waitMutex.unlock();

                // raise the alive flag again, this is necessary such that itom does not drop into a
                // timeout if the positioning needs more time than the allowed timeout time.
                setAlive();

                if (timeoutMS > -1)
                {
                    if (timer.elapsed() > timeoutMS)
                        timeout = true;
                }
            }

            if (timeout)
            {
                // timeout occurred, set the status of all currently moving axes to timeout
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
                retValue += ito::RetVal(ito::retError, 9999, "timeout occurred");
                sendStatusUpdate(true);
            }
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method called to show the configuration dialog
/*!
    This method is called from the main thread from itom and should show the configuration dialog of the plugin.
    If the instance of the configuration dialog has been created, its slot 'parametersChanged' is connected to the signal 'parametersChanged'
    of the plugin. By invoking the slot sendParameterRequest of the plugin, the plugin's signal parametersChanged is immediately emitted with
    m_params as argument. Therefore the configuration dialog obtains the current set of parameters and can be adjusted to its values.

    The configuration dialog should emit reject() or accept() depending if the user wanted to close the dialog using the ok or cancel button.
    If ok has been clicked (accept()), this method calls applyParameters of the configuration dialog in order to force the dialog to send
    all changed parameters to the plugin. If the user clicks an apply button, the configuration dialog itself must call applyParameters.

    If the configuration dialog is inherited from AbstractAddInConfigDialog, use the api-function apiShowConfigurationDialog that does all
    the things mentioned in this description.

    Remember that you need to implement hasConfDialog in your plugin and return 1 in order to signalize itom that the plugin
    has a configuration dialog.

    \sa hasConfDialog
*/
const ito::RetVal SmarActMCS2::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogSmarActMCS2(this));
}
