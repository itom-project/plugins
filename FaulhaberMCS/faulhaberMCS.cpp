/* ********************************************************************
    Plugin "FaulhaberMCS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#define NOMINMAX

#include "faulhaberMCS.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qdatetime.h>
#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#include "dockWidgetFaulhaberMCS.h"

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCSInterface::FaulhaberMCSInterface()
{
    m_type = ito::typeActuator;
    setObjectName("FaulhaberMCS");

    m_description = QObject::tr("FaulhaberMCS");

    m_detaildescription =
        QObject::tr("This plugin is an actuator plugin to control servo motors from Faulhaber.\n\
\n\
It was implemented for RS232 communication and tested with:\n\
\n\
* Serie MCS 3242: https://www.faulhaber.com/de/produkte/serie/mcs-3242bx4-et/");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal(
        "serialIOInstance",
        ito::ParamBase::HWRef | ito::ParamBase::In,
        nullptr,
        tr("An opened serial port of 'SerialIO' plugin instance.").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param(
        "Node",
        ito::ParamBase::Int | ito::ParamBase::In,
        1,
        new ito::IntMeta(1, std::numeric_limits<int>::max(), 1, "Communication"),
        tr("Node number of device.").toLatin1().data());
    m_initParamsMand.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCSInterface::~FaulhaberMCSInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCSInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(FaulhaberMCS)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCSInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(FaulhaberMCS)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCS::FaulhaberMCS() :
    AddInActuator(), m_delayAfterSendCommandMS(10), m_async(0), m_numOfAxes(1), m_node(1),
    m_statusWord(0), m_requestTimeOutMS(5000)
{
    ito::Param paramVal(
        "name", ito::ParamBase::String | ito::ParamBase::Readonly, "FaulhaberMCS", nullptr);
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category general device parameter
    //---------------------------//
    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Serial number of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceType",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Device type.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Name of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "vendorID",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Vendor ID of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "productCode",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Product code number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "revisionNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Revision number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "softwareVersion",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Manufacturer software version.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "ambientTemperature",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        tr("Ambient Temperature.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "General"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operationMode",
        ito::ParamBase::Int,
        -4,
        10,
        0,
        tr("Operation Mode. -4: ATC, -3: AVC, -2: APC, -1: Voltage mode, 0: Controller not "
           "activated, 1: PP, 3: PV, 6: Homing, 8: CSP, 9: CSV, 10: CST")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(-4, 10, 1, "General"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category movement ---------------------------//
    paramVal = ito::Param(
        "async",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("asynchronous move (1), synchronous (0) [default]").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operation",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Enable (1) or Disable (0) operation.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "power",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Enable (1) or Disable (0) device power.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "switch",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Enable (1) or Disable (0) switch.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "voltage",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Enable (1) or Disable (0) voltage.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "homed",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("homed (1) or not homed (0).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Movement"));
    m_params.insert(paramVal.getName(), paramVal);

    //------------------------------- category Statusword ---------------------------//
    paramVal = ito::Param(
        "statusWord",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        65535,
        0,
        tr("16bit statusWord to CiA 402. It indicates the status of the drive unit.")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 65535, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "readyToSwitchOn",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Ready to switch ON, 0: Not ready to switch ON (Bit 0).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "switchedOn",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Drive is in the 'Switched ON' state, 0: No voltage present (Bit 1).")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "operationEnabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Operation enabled, 0: Operation disabled (Bit 2).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "fault",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("1: Error present, 0: No error present (Bit 3).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "voltageEnabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Power supply enabled, 0: Power supply disabled (Bit 4).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "quickStop",
        ito::ParamBase::Int,
        0,
        1,
        0,
        tr("1: Quick stop enabled, Quick stop disabled (Bit 5).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "switchOnDisabled",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Switch on disabled, 0: Switch on enabled (Bit 6).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "warning",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: One of the monitored temperatures has exceeded at least the warning threshold, 0: "
           "No raised temperatures (Bit 7).")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "targetReached",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Target has reached, 0: is moving (Bit 10).").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "internalLimitActive",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Internal range limit (e.g. limit switch reached), 0: Internal range limit not "
           "reached (Bit 11).")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "setPointAcknowledged",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: New set-point has been loaded, 0: Previous set-point being changed or already "
           "reached (Bit 12).")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "followingError",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("1: Permissible range for the following error exceeded, 0: The actual position follows "
           "the instructions without a following error (Bit 13).")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::IntMeta(0, 1, 1, "Status"));
    m_params.insert(paramVal.getName(), paramVal);


    //------------------------------- category Motion control ---------------------------//
    paramVal = ito::Param(
        "maxMotorSpeed", ito::ParamBase::Int, 0, tr("Max motor speed in 1/min.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, 32767, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "profileVelocity",
        ito::ParamBase::Int,
        0,
        tr("Profile velocity in 1/min.").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, 32767, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "acceleration", ito::ParamBase::Int, 0, tr("Acceleration in 1/s².").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, 32750, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deceleration", ito::ParamBase::Int, 0, tr("Deceleration in 1/s².").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, 32750, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "quickStopDeceleration",
        ito::ParamBase::Int,
        0,
        tr("Quickstop deceleration in 1/s².").toLatin1().data());
    paramVal.setMeta(new ito::IntMeta(1, 32750, 1, "Motion control"));
    m_params.insert(paramVal.getName(), paramVal);

    int limits[] = {1000, 6000};
    paramVal = ito::Param(
        "torqueLimits",
        ito::ParamBase::IntArray,
        2,
        limits,
        tr("Torque limit values (negative, positive).").toLatin1().data());
    paramVal.setMeta(
        new ito::IntArrayMeta(0, std::numeric_limits<int>::max(), 1, "Torque control"), true);
    m_params.insert(paramVal.getName(), paramVal);

    // initialize the current position vector, the status vector and the target position vector
    m_currentPos.fill(0.0, m_numOfAxes);
    m_currentStatus.fill(0, m_numOfAxes);
    m_targetPos.fill(0.0, m_numOfAxes);

    // the following lines create and register the plugin's dock widget. Delete these lines if the
    // plugin does not have a dock widget.
    DockWidgetFaulhaberMCS* dw = new DockWidgetFaulhaberMCS(getID(), this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
        QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCS::~FaulhaberMCS()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (reinterpret_cast<ito::AddInBase*>((*paramsMand)[0].getVal<void*>())
            ->getBasePlugin()
            ->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        // Us given SerialIO instance
        m_pSerialIO = (ito::AddInDataIO*)(*paramsMand)[0].getVal<void*>();
        m_node = (uint8_t)paramsMand->at(1).getVal<int>();
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Input parameter is not a dataIO instance of the SerialIO Plugin!")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        QSharedPointer<QVector<ito::ParamBase>> _dummy;
        m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);
    }


    if (!retValue.containsError())
    {
        char* serialNumber = nullptr;
        retValue += getSerialNumber(serialNumber);
        if (!retValue.containsError())
        {
            m_params["serialNumber"].setVal<char*>(serialNumber);
        }
    }


    if (!retValue.containsError())
    {
        char* device = nullptr;
        retValue += getDeviceType(device);
        if (!retValue.containsError())
        {
            m_params["deviceType"].setVal<char*>(device);
        }
    }

    if (!retValue.containsError())
    {
        // TODO delete if status works well. User must execute the right command to start motor
        /*mmProtSendCommand(m_node, 0x0000, eMomancmd_start, 0, 0);
        mmProtSendCommand(m_node, 0x0000, eMomancmd_quickstop, 0, 0);
        retValue += waitForIntParam("quickStop", 0);

        mmProtSendCommand(m_node, 0x0000, eMomancmd_faultreset, 0, 0);
        retValue += waitForIntParam("fault", 0);

        mmProtSendCommand(m_node, 0x0000, eMomancmd_shutdown, 0, 0);
        retValue += waitForIntParam("switchedOn", 0);

        mmProtSendCommand(m_node, 0x0000, eMomancmd_switchon, 0, 0);
        retValue += waitForIntParam("switchedOn", 1);

        mmProtSendCommand(m_node, 0x0000, eMomancmd_EnOp, 0, 0);
        retValue += waitForIntParam("operationEnabled", 1);*/
    }

    if (!retValue.containsError())
    {
        /*int pos;
        retValue += getPosMCS(pos);
        m_currentPos[0] = pos;
        m_targetPos[0] = pos;
        m_currentStatus[0] = ito::actuatorAtTarget | ito::actuatorEnabled |
        ito::actuatorAvailable;*/
        sendStatusUpdate(false);
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        // gets the parameter key from m_params map (read-only is allowed, since we only want to get
        // the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (key == "operationMode")
        {
        }
        else if (key == "ambientTemperature")
        {
        }
        else if (key == "maxMotorSpeed")
        {
        }
        else if (key == "profileVelocity")
        {
        }
        else if (key == "acceleration")
        {
        }
        else if (key == "deceleration")
        {
        }
        else if (key == "quickStopDeceleration")
        {
        }
        else if (key == "statusWord")
        {
        }
        else if (key == "torqueLimits")
        {
        }

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
ito::RetVal FaulhaberMCS::setParam(
    QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (isMotorMoving()) // this if-case is for actuators only.
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("any axis is moving. Parameters cannot be set.").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        // gets the parameter key from m_params map (read-only is not allowed and leads to
        // ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        // here the new parameter is checked whether its type corresponds or can be cast into the
        //  value in m_params and whether the new type fits to the requirements of any possible
        //  meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "async")
        {
            m_async = val->getVal<int>();
            // check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "operationMode")
        {
        }
        else if (key == "operation")
        {
            int operation = val->getVal<int>();
            if (operation == 0)
            {
            }
            else if (operation == 1)
            {
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Value (%1) of parameter 'operation' must be 0 or 1.")
                        .arg(operation)
                        .toLatin1()
                        .data());
            }

            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "power")
        {
            int power = val->getVal<int>();
            if (power == 0)
            {
            }
            else if (power == 1)
            {
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Value (%1) of parameter 'power' must be 0 or 1.")
                        .arg(power)
                        .toLatin1()
                        .data());
            }
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "fault")
        {
            if (val->getVal<int>())
            {
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Set the parameter value to 1 for fault reset.").toLatin1().data());
            }
        }
        else if (key == "voltage")
        {
            if (val->getVal<int>())
            {
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Set the parameter value to 1 to disable voltage.").toLatin1().data());
            }
        }
        else if (key == "maxMotorSpeed")
        {
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "profileVelocity")
        {
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "acceleration")
        {
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "deceleration")
        {
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "quickStopDeceleration")
        {
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "torqueLimits")
        {
            retValue += it->copyValueFrom(&(*val));
        }
        else
        {
            // all parameters that don't need further checks can simply be assigned
            // to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(
            m_params); // send changed parameters to any connected dialogs or dock-widgets
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
ito::RetVal FaulhaberMCS::calib(const int axis, ItomSharedSemaphore* waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! calib
/*!
    the given axes should be calibrated (e.g. by moving to a reference switch).
*/
ito::RetVal FaulhaberMCS::calib(const QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    // check axis index
    foreach (const int& i, axis)
    {
        if (i >= m_numOfAxes)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("axis number is out of boundary").toLatin1().data());
        }
    }

    // homing routine if motor is not moving
    if (!retValue.containsError())
    {
        if (isMotorMoving())
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());

            if (waitCond)
            {
                waitCond->release();
                waitCond->returnValue = retValue;
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            for (const int& i : axis)
            {
                if (auto result = homingCurrentPosToZero(i); result.containsError())
                {
                    retValue += result;
                    break;
                }
            }

            if (!retValue.containsError())
            {
                m_params["homed"].setVal<int>(1);
            }

            if (waitCond)
            {
                waitCond->release();
                waitCond->returnValue = retValue;
            }

            sendStatusUpdate();
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegister(
    const uint16_t& address, const uint8_t& subindex, std::vector<uint8_t>& response)
{
    ito::RetVal retValue = ito::retOk;
    std::vector<uint8_t> command = {
        m_node,
        m_GET,
        static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>(address >> 8),
        subindex};

    std::vector<uint8_t> fullCommand = {static_cast<uint8_t>(command.size() + 2)};
    fullCommand.insert(fullCommand.end(), command.begin(), command.end());
    fullCommand.push_back(CRC(fullCommand));
    fullCommand.insert(fullCommand.begin(), m_S);
    fullCommand.push_back(m_E);

    QByteArray data(reinterpret_cast<char*>(fullCommand.data()), fullCommand.size());
    QByteArray answer;
    retValue += sendCommandAndGetResponse(data, answer);

    if (!retValue.containsError())
    {
        retValue += parseResponse(answer, response);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::parseResponse(
    const QByteArray& response, std::vector<uint8_t>& parsedResponse)
{
    ito::RetVal retValue = ito::retOk;
    std::vector<uint8_t> ansVector(response.begin(), response.end());

    if (CRC(std::vector<uint8_t>(ansVector.begin() + 1, ansVector.end() - 2)) !=
        ansVector[ansVector.size() - 2])
    {
        retValue += ito::RetVal(
            ito::retError, 0, tr("Checksum mismatch for %1").arg(response).toLatin1().data());
    }

    if (!retValue.containsError())
    {
        parsedResponse = std::vector<uint8_t>(ansVector.begin() + 7, ansVector.end() - 2);
    }
    else
    {
        parsedResponse = {0};
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::homingCurrentPosToZero(const int& axis)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;
    timer.start();

    // Set to homing mode
    uint8_t mode = 6;
    // retValue += setOperationMode(mode);
    if (retValue.containsError())
        return retValue;

    // Apply homing mode
    mode = 37;
    // retValue += setHomingMode(mode);
    if (retValue.containsError())
        return retValue;

    // Start homing
    mode = 0x000F;
    // retValue += setControlword(mode, 2);

    while (!retValue.containsWarningOrError())
    {
        isAlive();
        Sleep(1);
        // retValue += updateStatusMCS();

        if (!(m_statusWord & targetReached) &&
            !(m_statusWord & setPointAcknowledged)) // target still not reached
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("Target not reached during homing").toLatin1().data());
        }
        else if (m_statusWord & followingError) // error during homing
        {
            retValue +=
                ito::RetVal(ito::retError, 0, tr("Error occurs during homing").toLatin1().data());
        }
        else // target reached
        {
            m_targetPos[axis] = 0.0;
            sendTargetUpdate();
            break;
        }

        // Timeout during movement
        if (timer.hasExpired(10000))
        {
            retValue += ito::RetVal(
                ito::retError, 9999, QString("Timeout occurred during homing").toLatin1().data());
            break;
        }
    }


    mode = 0x001F;
    // retValue += setControlword(mode, 2);

    int currentPos;
    // retValue += getPosMCS(currentPos);

    m_currentPos[axis] = currentPos;
    setStatus(
        m_currentStatus[axis], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);

    if (retValue.containsError())
        return retValue;

    // Set to profile operation mode
    mode = 1;
    // retValue += setOperationMode(mode);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setOrigin
/*!
    the given axis should be set to origin. That means (if possible) its current position should be
    considered to be the new origin (zero-position). If this operation is not possible, return a
    warning.
*/
ito::RetVal FaulhaberMCS::setOrigin(const int axis, ItomSharedSemaphore* waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setOrigin
/*!
    the given axes should be set to origin. That means (if possible) their current position should
   be considered to be the new origin (zero-position). If this operation is not possible, return a
    warning.
*/
ito::RetVal FaulhaberMCS::setOrigin(QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach (const int& i, axis)
        {
            if (i >= 0 && i < m_numOfAxes)
            {
                // todo: set axis i to origin (current position is considered to be the 0-position).
            }
            else
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
        }

        retValue += updateStatus();
    }

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
    re-checks the status (current position, available, end switch reached, moving, at target...) of
   all axes and returns the status of each axis as vector. Each status is an or-combination of the
   enumeration ito::tActuatorStatus.
*/
ito::RetVal FaulhaberMCS::getStatus(
    QSharedPointer<QVector<int>> status, ItomSharedSemaphore* waitCond)
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
    returns the current position (in mm or degree) of the given axis
*/
ito::RetVal FaulhaberMCS::getPos(
    const int axis, QSharedPointer<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QSharedPointer<QVector<double>> pos2(new QVector<double>(1, 0.0));
    ito::RetVal retValue =
        getPos(QVector<int>(1, axis), pos2, nullptr); // forward to multi-axes version
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
    returns the current position (in mm or degree) of all given axes
*/
ito::RetVal FaulhaberMCS::getPos(
    QVector<int> axis, QSharedPointer<QVector<double>> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    foreach (const int i, axis)
    {
        if (i >= 0 && i < m_numOfAxes)
        {
            int intPos;
            // retValue += getPosMCS(intPos);
            if (!retValue.containsError())
            {
                m_currentPos[i] = intPos; // set m_currentPos[i] to the obtained position
                (*pos)[i] = m_currentPos[i];
            }
        }
        else
        {
            retValue += ito::RetVal::format(
                ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
        }
    }

    sendStatusUpdate();

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
ito::RetVal FaulhaberMCS::setPosAbs(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
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
ito::RetVal FaulhaberMCS::setPosAbs(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = pos[i];
            }
        }

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                setStatus(
                    m_currentStatus[naxis],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                sendStatusUpdate(false);
            }

            // emit the signal targetChanged with m_targetPos as argument, such that all connected
            // slots gets informed about new targets
            sendTargetUpdate();

            foreach (const int i, axis)
            {
                // retValue += setPosAbsMCS(m_targetPos[i]);
            }

            // emit the signal sendStatusUpdate such that all connected slots gets informed about
            // changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();
            foreach (const int& a, axis)
            {
                replaceStatus(m_currentStatus[a], ito::actuatorMoving, ito::actuatorInterrupted);
            }
            sendStatusUpdate(false);

            // release the wait condition now, if async is true (itom considers this method to be
            // finished now due to the threaded call)
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            // call waitForDone in order to wait until all axes reached their target or a given
            // timeout expired the m_currentPos and m_currentStatus vectors are updated within this
            // function
            retValue += waitForDone(m_waitForDoneTimeout, axis);

            // release the wait condition now, if async is false (itom waits until now if async is
            // false, hence in the synchronous mode)
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    // if the wait condition has not been released yet, do it now
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
ito::RetVal FaulhaberMCS::setPosRel(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setPosRel
/*!
    starts moving the given axes by the given relative distances

    depending on m_async this method directly returns after starting the movement (async = 1) or
    only returns if all axes reached the given target positions (async = 0)

    In some cases only absolute movements are possible, then get the current positions, determine
   the new absolute target positions and call setPosAbs with these absolute target positions.
*/
ito::RetVal FaulhaberMCS::setPosRel(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = m_currentPos[i] + pos[i];
            }
        }

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                setStatus(
                    m_currentStatus[naxis],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                sendStatusUpdate(false);
            }

            // emit the signal targetChanged with m_targetPos as argument, such that all connected
            // slots gets informed about new targets
            sendTargetUpdate();

            foreach (const int i, axis)
            {
                // retValue += setPosRelMCS(pos[i]);
            }

            // emit the signal sendStatusUpdate such that all connected slots gets informed about
            // changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();
            foreach (const int& a, axis)
            {
                replaceStatus(m_currentStatus[a], ito::actuatorMoving, ito::actuatorInterrupted);
            }
            sendStatusUpdate(false);

            // release the wait condition now, if async is true (itom considers this method to be
            // finished now due to the threaded call)
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            // call waitForDone in order to wait until all axes reached their target or a given
            // timeout expired the m_currentPos and m_currentStatus vectors are updated within this
            // function
            retValue += waitForDone(m_waitForDoneTimeout, axis);

            // release the wait condition now, if async is false (itom waits until now if async is
            // false, hence in the synchronous mode)
            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    // if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getSerialNumber(char*& serialNum)
{
    ito::RetVal retVal = readRegisterWithAnswerString(0x1018, 0x04, serialNum);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeviceType(char*& device)
{
    ito::RetVal retVal = readRegisterWithAnswerString(0x1000, 0x00, device);
    return retVal;
}

////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getVendorID(int& id)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x1018, 0x01, id);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getProductCode(int& code)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x1018, 0x02, code);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getRevisionNumber(int& num)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x1018, 0x03, num);
//
//     return convertErrorCode(error, __func__);
// }
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getDeviceName(const char*& name)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetStrObj(m_node, 0x1008, 0x00, &name);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getSoftwareVersion(const char*& version)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetStrObj(m_node, 0x100A, 0x00, &version);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getPosMCS(int& pos)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x6064, 0x00, pos);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getTargetPosMCS(int& pos)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x6062, 0x00, pos);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getAmbientTemperature(int& temp)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x232A, 0x08, temp);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setPosAbsMCS(double& pos)
//{
//     ito::RetVal retVal(ito::retOk);
//
//     unsigned int abortMessage;
//     eMomanprot error;
//
//     // Modes of Operation = Profile Position Mode (1):
//     error = mmProtSetObj(m_node, 0x6060, 0x00, 1, 1, abortMessage);
//     if (error == eMomanprot_ok)
//     {
//         int intPos = doubleToInteger(pos);
//         error = mmProtSetObj(m_node, 0x607A, 0x00, intPos, sizeof(intPos), abortMessage);
//         if (error == eMomanprot_ok)
//         {
//             // Enable Operation:
//             error = mmProtSetObj(m_node, 0x0000, eMomancmd_EnOp, 0, sizeof(0), abortMessage);
//             if (error == eMomanprot_ok)
//             {
//                 // Move absolute:
//                 retVal += setControlword(0x003F, 2);
//             }
//         }
//     }
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setPosRelMCS(const double& pos)
//{
//     ito::RetVal retVal(ito::retOk);
//
//     unsigned int abortMessage;
//     eMomanprot error;
//     // Modes of Operation = Profile Position Mode (1):
//     error = mmProtSetObj(m_node, 0x6060, 0x00, 1, 1, abortMessage);
//     if (error == eMomanprot_ok)
//     {
//         int intPos = doubleToInteger(pos);
//         error = mmProtSetObj(m_node, 0x607A, 0x00, intPos, sizeof(intPos), abortMessage);
//         if (error == eMomanprot_ok)
//         {
//             // Enable Operation:
//             error = mmProtSetObj(m_node, 0x0000, eMomancmd_EnOp, 0, 0, abortMessage);
//             if (error == eMomanprot_ok)
//             {
//                 // Move relative:
//                 retVal += setControlword(0x007F, 2);
//             }
//         }
//     }
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::updateStatusMCS()
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x6041, 0x0, m_statusWord);
//     if (error == eMomanprot_ok)
//     {
//         m_params["statusWord"].setVal<int>(m_statusWord);
//
//         m_params["readyToSwitchOn"].setVal<int>(m_statusWord & readyToSwitchOn ? 1 : 0);
//         m_params["switchedOn"].setVal<int>(m_statusWord & switchedOn ? 1 : 0);
//         m_params["operationEnabled"].setVal<int>(m_statusWord & operationEnabled ? 1 : 0);
//         m_params["fault"].setVal<int>(m_statusWord & fault ? 1 : 0);
//         m_params["voltageEnabled"].setVal<int>(m_statusWord & voltageEnabled ? 1 : 0);
//         m_params["quickStop"].setVal<int>(m_statusWord & quickStop ? 1 : 0);
//         m_params["switchOnDisabled"].setVal<int>(m_statusWord & switchOnDisabled ? 1 : 0);
//         m_params["warning"].setVal<int>(m_statusWord & warning ? 1 : 0);
//         m_params["targetReached"].setVal<int>(m_statusWord & targetReached ? 1 : 0);
//         m_params["internalLimitActive"].setVal<int>(m_statusWord & internalLimitActive ? 1 : 0);
//         m_params["setPointAcknowledged"].setVal<int>(m_statusWord & setPointAcknowledged ? 1 :
//         0); m_params["followingError"].setVal<int>(m_statusWord & followingError ? 1 : 0);
//
//         emit parametersChanged(m_params);
//     }
//
//     sendStatusUpdate();
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::convertErrorCode(
//     const eMomanprot& error,
//     const QString& functionName,
//     /*const*/ const unsigned int& abortMessage)
//{
//     ito::RetVal retVal;
//     if (error != eMomanprot_ok)
//     {
//         retVal += ito::RetVal(
//             ito::retError,
//             0,
//             tr("Error during function '%1' with error message: '%2' and abort "
//                "message: '%3'.")
//                 .arg(functionName)
//                 .arg(error)
//                 .arg(mmProtGetAbortMessage(abortMessage))
//                 .toLatin1()
//                 .data());
//     }
//     else
//     {
//         retVal = ito::retOk;
//     }
//     return retVal;
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getMaxMotorSpeed(int& speed)
//{
//     eMomanprot error = mmProtGetObj(m_node, 0x6080, 0x00, speed);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setMaxMotorSpeed(const int& speed)
//{
//     unsigned int abortMessage;
//     eMomanprot error = mmProtSetObj(m_node, 0x6080, 0x00, speed, sizeof(speed), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getProfileVelocity(int& speed)
//{
//     eMomanprot error = mmProtGetObj(m_node, 0x6081, 0x00, speed);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setProfileVelocity(const int& speed)
//{
//     unsigned int abortMessage;
//     eMomanprot error = mmProtSetObj(m_node, 0x6081, 0x00, speed, sizeof(speed), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getAcceleration(int& acceleration)
//{
//     eMomanprot error = mmProtGetObj(m_node, 0x6083, 0x00, acceleration);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setAcceleration(const int& acceleration)
//{
//     unsigned int abortMessage;
//     eMomanprot error =
//         mmProtSetObj(m_node, 0x6083, 0x00, acceleration, sizeof(acceleration), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getDeceleration(int& deceleration)
//{
//     eMomanprot error = mmProtGetObj(m_node, 0x6084, 0x00, deceleration);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setDeceleration(const int& deceleration)
//{
//     unsigned int abortMessage;
//     eMomanprot error =
//         mmProtSetObj(m_node, 0x6084, 0x00, deceleration, sizeof(deceleration), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getQuickStopDeceleration(int& deceleration)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x6085, 0x00, deceleration);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setQuickStopDeceleration(const int& deceleration)
//{
//     unsigned int abortMessage;
//     eMomanprot error =
//         mmProtSetObj(m_node, 0x6085, 0x00, deceleration, sizeof(deceleration), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setHomingMode(const uint8_t& mode)
//{
//     unsigned int abortMessage;
//     eMomanprot error = mmProtSetObj(m_node, 0x6098, 0x00, mode, sizeof(mode), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getTorqueLimits(int limits[])
//{
//     eMomanprot error = mmProtGetObj(m_node, 0x60E1, 0x00, limits[0]);
//     error = mmProtGetObj(m_node, 0x60E0, 0x00, limits[1]);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setTorqueLimits(const int limits[])
//{
//     unsigned int abortMessage;
//     eMomanprot error =
//         mmProtSetObj(m_node, 0x60E1, 0x00, limits[0], sizeof(limits[0]), abortMessage);
//     error = mmProtSetObj(m_node, 0x60E0, 0x00, limits[1], sizeof(limits[1]), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getOperationMode(int& mode)
//{
//     eMomanprot error = mmProtGetObj(m_node, 0x6060, 0x00, mode);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setOperationMode(const uint8_t& mode)
//{
//     unsigned int abortMessage;
//     eMomanprot error = mmProtSetObj(m_node, 0x6060, 0x00, mode, sizeof(mode), abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::getControlword(int& word)
//{
//     ito::RetVal retVal(ito::retOk);
//     eMomanprot error = mmProtGetObj(m_node, 0x6040, 0x00, word);
//
//     return convertErrorCode(error, __func__);
// }
//
////----------------------------------------------------------------------------------------------------------------------------------
// ito::RetVal FaulhaberMCS::setControlword(const uint8_t& word, const int& len)
//{
//     unsigned int abortMessage;
//     eMomanprot error = mmProtSetObj(m_node, 0x6040, 0x00, word, len, abortMessage);
//
//     return convertErrorCode(error, __func__, abortMessage);
// }

//----------------------------------------------------------------------------------------------------------------------------------
int FaulhaberMCS::doubleToInteger(const double& value)
{
    return int(std::round(value * 100) / 100);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    char motor;
    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;

    QVector<int> _axis = axis;
    if (_axis.size() == 0) // all axis
    {
        for (int i = 0; i < m_numOfAxes; i++)
        {
            _axis.append(i);
        }
    }

    timer.start();
    while (!done && !timeout && !retVal.containsWarningOrError())
    {
        if (!done && isInterrupted()) // movement interrupted
        {
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;
            sendStatusUpdate();
            return retVal;
        }

        if (!retVal.containsError()) // short delay to reduce CPU load
        {
            // short delay of 10ms
            waitMutex.lock();
            waitCondition.wait(&waitMutex, 10);
            waitMutex.unlock();
            setAlive();
        }

        for (int i = 0; i < axis.size(); ++i) // Check for completion
        {
            // retVal += updateStatusMCS();
            if ((m_statusWord & targetReached) && (m_statusWord & setPointAcknowledged))
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorAtTarget,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true;
                break;
            }
            else
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = false;
            }

            int currentPos;
            int targetPos;
            // retVal += getPosMCS(currentPos);
            m_currentPos[i] = double(currentPos);

            // retVal += getTargetPosMCS(targetPos);
            m_targetPos[i] = double(targetPos);
        }

        // retVal += updateStatusMCS();
        sendStatusUpdate(false);


        if (timer.hasExpired(timeoutMS)) // timeout during movement
        {
            timeout = true;
            // timeout occurred, set the status of all currently moving axes to timeout
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retVal += ito::RetVal(ito::retError, 9999, "timeout occurred during movement");
            sendStatusUpdate(true);
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method obtains the current position, status of all axes
/*!
    This is a helper function, it is not necessary to implement a function like this, but it might
   help.
*/
ito::RetVal FaulhaberMCS::updateStatus()
{
    ito::RetVal retVal = ito::retOk;
    for (int i = 0; i < m_numOfAxes; i++)
    {
        m_currentStatus[i] = m_currentStatus[i] |
            ito::actuatorAvailable; // set this if the axis i is available, else use
        // m_currentStatus[i] = m_currentStatus[i] ^ ito::actuatorAvailable;

        int intPos;
        // retVal += getPosMCS(intPos);

        m_currentPos[i] = double(intPos);

        // if you know that the axis i is at its target position, change from moving to
        // target if moving has been set, therefore:
        replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget);

        // if you know that the axis i is still moving, set this bit (all other moving-related bits
        // are unchecked, but the status bits and switches bits kept unchanged
        setStatus(
            m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
    }

    // emit actuatorStatusChanged with m_currentStatus and m_currentPos in order to inform connected
    // slots about the current status and position
    sendStatusUpdate();

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::sendCommandAndGetResponse(const QByteArray& command, QByteArray& response)
{
    ito::RetVal retVal;

    retVal += m_pSerialIO->setVal(command.data(), command.length(), nullptr);

    // delay
    if (m_delayAfterSendCommandMS > 0)
    {
        QMutex mutex;
        mutex.lock();
        QWaitCondition waitCondition;
        waitCondition.wait(&mutex, m_delayAfterSendCommandMS);
        mutex.unlock();
    }
    setAlive();

    if (!retVal.containsError())
    {
        ito::RetVal retValue = readResponse(response);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readResponse(QByteArray& result)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    bool done = false;

    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";

    timer.start();
    QThread::msleep(m_delayAfterSendCommandMS);

    while (!done && !retValue.containsError())
    {
        *curBufLen = buflen;
        retValue += m_pSerialIO->getVal(curBuf, curBufLen, nullptr);

        if (!retValue.containsError())
        {
            result += QByteArray(curBuf.data(), *curBufLen);
            done = true;
        }

        if (!done && timer.elapsed() > m_requestTimeOutMS && m_requestTimeOutMS >= 0)
        {
            retValue += ito::RetVal(
                ito::retError,
                m_delayAfterSendCommandMS,
                tr("timeout during read string.").toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::sendQuestionWithAnswerDouble(
    const QByteArray& questionCommand, double& answer)
{
    QByteArray questionCommand_ = QString::number(m_node).toUtf8() + questionCommand;
    int readSigns;
    QByteArray answerStr;
    bool ok;

    ito::RetVal retValue = sendCommandAndGetResponse(questionCommand_, answerStr);

    if (questionCommand_.contains("?"))
    {
        questionCommand_.replace("?", "");
    }

    answer = answerStr.toDouble(&ok);

    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerDouble, converting %1 to double value.")
                .arg(answerStr.constData())
                .toLatin1()
                .data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::sendQuestionWithAnswerDoubleArray(
    const QByteArray& questionCommand, double* answer, const int number)
{
    QByteArray questionCommand_ = QString::number(m_node).toUtf8() + questionCommand;
    int readSigns;
    QByteArray answerStr;
    bool ok = true;
    ito::RetVal retValue = sendCommandAndGetResponse(questionCommand_, answerStr);

    if (questionCommand_.contains("?"))
    {
        questionCommand_.replace("?", "");
    }


    QRegularExpression regex("-?\\d+(\\.\\d+)?");
    QRegularExpressionMatchIterator matchIterator = regex.globalMatch(answerStr);

    // Extract and print all matched values
    int n = 0;
    QRegularExpressionMatch match;
    QString matchedValue;
    while (matchIterator.hasNext() && ok)
    {
        match = matchIterator.next();
        matchedValue = match.captured();
        answer[n] = matchedValue.toDouble(&ok);
        n++;
    }

    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerDouble, converting %1 to double value.")
                .arg(answerStr.constData())
                .toLatin1()
                .data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegisterWithAnswerString(
    const uint16_t& address, const uint8_t& subindex, char*& answer)
{
    ito::RetVal retValue = ito::retOk;
    std::vector<uint8_t> registerValue;

    retValue += readRegister(address, subindex, registerValue);

    if (registerValue.size() > sizeof(int))
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during readRegisterWithAnswerString, converting to string value.")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        uint32_t num = 0;
        std::memcpy(&num, registerValue.data(), 4); // Assuming little-endian

        // Step 2: Convert integer to string
        std::ostringstream oss;
        oss << num;
        std::string str = oss.str();

        // Step 3: Convert string to char*
        answer = new char[str.size() + 1];
        std::strcpy(answer, str.c_str());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::readRegisterWithAnswerInteger(
    const uint16_t& address, const uint8_t& subindex, int& answer)
{
    ito::RetVal retValue = ito::retOk;
    std::vector<uint8_t> registerValue;

    retValue += readRegister(address, subindex, registerValue);

    if (registerValue.size() > sizeof(int))
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Error during SendQuestionWithAnswerInteger, converting to integer value.")
                .toLatin1()
                .data());
    }

    answer = 0;
    if (!retValue.containsError())
    {
        for (size_t i = 0; i < registerValue.size(); ++i)
        {
            answer |= static_cast<int>(registerValue[i]) << (8 * i);
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
uint8_t FaulhaberMCS::CRC(const std::vector<uint8_t>& msg)
{
    uint8_t poly = 0xD5;
    uint8_t crc = 0xFF;

    for (auto byte : msg)
    {
        crc ^= byte;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x01)
                crc = (crc >> 1) ^ poly;
            else
                crc >>= 1;
        }
    }

    return crc;
}
//----------------------------------------------------------------------------------------------------------------------------------
void FaulhaberMCS::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget* widget = getDockWidget()->widget();
        if (visible)
        {
            connect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(
                this,
                SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)),
                widget,
                SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            connect(
                this,
                SIGNAL(targetChanged(QVector<double>)),
                widget,
                SLOT(targetChanged(QVector<double>)));

            emit parametersChanged(m_params);
            sendTargetUpdate();
            sendStatusUpdate(false);
        }
        else
        {
            disconnect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            disconnect(
                this,
                SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)),
                widget,
                SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            disconnect(
                this,
                SIGNAL(targetChanged(QVector<double>)),
                widget,
                SLOT(targetChanged(QVector<double>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal FaulhaberMCS::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogFaulhaberMCS(this));
}
