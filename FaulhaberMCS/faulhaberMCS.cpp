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

#include "faulhaberMCS.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qdatetime.h>
#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

#include "dockWidgetFaulhaberMCS.h"

//----------------------------------------------------------------------------------------------------------------------------------
FaulhaberMCSInterface::FaulhaberMCSInterface()
{
    m_type = ito::typeActuator;
    setObjectName("FaulhaberMCS");

    m_description = QObject::tr("FaulhaberMCS");

    char docstring[] =
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

    ito::Param paramVal = ito::Param(
        "COMPort",
        ito::ParamBase::Int,
        1,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Communication"),
        tr("COM port of device.").toLatin1().data());
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param(
        "baudrate",
        ito::ParamBase::Int,
        112500,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "Communication"),
        tr("Baudrate in Bit/s of COM port.").toLatin1().data());
    m_initParamsOpt.append(paramVal);
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
FaulhaberMCS::FaulhaberMCS() : AddInActuator(), m_async(0), m_nrOfAxes(1), m_node(1)
{
    ito::Param paramVal(
        "name", ito::ParamBase::String | ito::ParamBase::Readonly, "FaulhaberMCS", NULL);
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
        "",
        tr("Serial number of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "deviceName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Name of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "vendorID",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Vendor ID of device.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "productCode",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Product code number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "revisionNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Revision number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "softwareVersion",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Manufacturer software version.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    // initialize the current position vector, the status vector and the target position vector
    m_currentPos.fill(0.0, m_nrOfAxes);
    m_currentStatus.fill(0, m_nrOfAxes);
    m_targetPos.fill(0.0, m_nrOfAxes);

    // the following lines create and register the plugin's dock widget. Delete these lines if the
    // plugin does not have a dock widget.
    DockWidgetFaulhaberMCS* dw = new DockWidgetFaulhaberMCS(this);

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
    eMomanprot error;

    m_hProtocolDll = LoadLibraryA("CO_RS232.dll");
    if (m_hProtocolDll == nullptr)
    {
        retValue +=
            ito::RetVal(ito::retError, 0, tr("CO_RS232.dll cannot be loaded!").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        // Needed functions of the communication library:
        bool ok = true;
        mmProtInitInterface =
            (tdmmProtInitInterface)GetProcAddress(m_hProtocolDll, "mmProtInitInterface");
        ok &= mmProtInitInterface != nullptr;
        mmProtCloseInterface =
            (tdmmProtCloseInterface)GetProcAddress(m_hProtocolDll, "mmProtCloseInterface");
        ok &= mmProtCloseInterface != nullptr;
        mmProtOpenCom = (tdmmProtOpenCom)GetProcAddress(m_hProtocolDll, "mmProtOpenCom");
        ok &= mmProtOpenCom != nullptr;
        mmProtCloseCom = (tdmmProtCloseCom)GetProcAddress(m_hProtocolDll, "mmProtCloseCom");
        ok &= mmProtCloseCom != nullptr;
        mmProtSendCommand =
            (tdmmProtSendCommand)GetProcAddress(m_hProtocolDll, "mmProtSendCommand");
        ok &= mmProtSendCommand != nullptr;
        mmProtReadAnswer = (tdmmProtReadAnswer)GetProcAddress(m_hProtocolDll, "mmProtReadAnswer");
        ok &= mmProtReadAnswer != nullptr;
        mmProtDecodeAnswStr =
            (tdmmProtDecodeAnswStr)GetProcAddress(m_hProtocolDll, "mmProtDecodeAnswStr");
        ok &= mmProtDecodeAnswStr != nullptr;
        mmProtGetStrObj = (tdmmProtGetStrObj)GetProcAddress(m_hProtocolDll, "mmProtGetStrObj");
        ok &= mmProtGetStrObj != nullptr;
        mmProtSetObj = (tdmmProtSetObj)GetProcAddress(m_hProtocolDll, "mmProtSetObj");
        ok &= mmProtSetObj != nullptr;
        mmProtGetAbortMessage =
            (tdmmProtGetAbortMessage)GetProcAddress(m_hProtocolDll, "mmProtGetAbortMessage");
        ok &= mmProtGetAbortMessage != nullptr;
        mmProtGetErrorMessage =
            (tdmmProtGetErrorMessage)GetProcAddress(m_hProtocolDll, "mmProtGetErrorMessage");
        ok &= mmProtGetErrorMessage != nullptr;
        mmProtGetObj = (tdmmProtGetObj)GetProcAddress(m_hProtocolDll, "mmProtGetObj");
        ok &= mmProtGetObj != nullptr;
        mmProtFindConnection =
            (tdmmProtFindConnection)GetProcAddress(m_hProtocolDll, "mmProtFindConnection");
        ok &= mmProtFindConnection != nullptr;
        mmProtSendMotionCommand =
            (tdmmProtSendMotionCommand)GetProcAddress(m_hProtocolDll, "mmProtSendMotionCommand");
        ok &= mmProtSendMotionCommand != nullptr;
        mmProtCheckMotionCommand =
            (tdmmProtCheckMotionCommand)GetProcAddress(m_hProtocolDll, "mmProtCheckMotionCommand");
        ok &= mmProtCheckMotionCommand != nullptr;

        if (!ok)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Error during definition of function forcommunication library!")
                    .toLatin1()
                    .data());
        }
    }

    if (!retValue.containsError())
    {
        error = mmProtInitInterface((char*)"Mocom.dll", nullptr, nullptr);
        if (error != eMomanprot_ok)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Error during loading MC3USB.dll with error message: '%1'!")
                    .arg(mmProtGetErrorMessage(error))
                    .toLatin1()
                    .data());
        }
    }

    if (!retValue.containsError())
    {
        int port = paramsMand->at(0).getVal<int>();
        int baud = paramsOpt->at(0).getVal<int>();

        error = mmProtOpenCom(m_node, port, baud);
        if (error != eMomanprot_ok)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Error during opening COM Port with error message: '%1'!")
                    .arg(mmProtGetErrorMessage(error))
                    .toLatin1()
                    .data());
        }
        if (!retValue.containsError())
        {
            m_COMPort = port;
        }
    }

    if (!retValue.containsError())
    {
        int serial;
        retValue += getSerialNumber(serial);
        if (!retValue.containsError())
        {
            m_params["serialNumber"].setVal<char*>(
                const_cast<char*>(std::to_string(serial).c_str()));
        }
    }

    if (!retValue.containsError())
    {
        const char* name = nullptr;
        retValue += getDeviceName(name);
        if (!retValue.containsError())
        {
            m_params["deviceName"].setVal<char*>(const_cast<char*>(name));
        }
    }

    if (!retValue.containsError())
    {
        const char* version = nullptr;
        retValue += getSoftwareVersion(version);
        if (!retValue.containsError())
        {
            m_params["softwareVersion"].setVal<char*>(const_cast<char*>(version));
        }
    }

    if (!retValue.containsError())
    {
        int id;
        retValue += getVendorID(id);
        if (!retValue.containsError())
        {
            m_params["vendorID"].setVal<char*>(const_cast<char*>(std::to_string(id).c_str()));
        }
    }

    if (!retValue.containsError())
    {
        int code;
        retValue += getProductCode(code);
        if (!retValue.containsError())
        {
            m_params["productCode"].setVal<char*>(const_cast<char*>(std::to_string(code).c_str()));
        }
    }

    if (!retValue.containsError())
    {
        int num;
        retValue += getRevisionNumber(num);
        if (!retValue.containsError())
        {
            m_params["revisionNumber"].setVal<char*>(
                const_cast<char*>(std::to_string(num).c_str()));
        }
    }

    // start device
    if (!retValue.containsError())
    {
        mmProtSendCommand(m_node, 0x0000, eMomancmd_start, 0, 0);
        mmProtSendCommand(m_node, 0x0000, eMomancmd_shutdown, 0, 0);
        Sleep(100);
        mmProtSendCommand(m_node, 0x0000, eMomancmd_switchon, 0, 0);
        mmProtSendCommand(m_node, 0x0000, eMomancmd_EnOp, 0, 0);
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

    setInitialized(true);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    mmProtSendCommand(m_node, 0x0000, eMomancmd_stop, 0, 0);
    mmProtSendCommand(m_node, 0x0000, eMomancmd_shutdown, 0, 0);

    mmProtCloseCom();
    mmProtCloseInterface();
    m_hProtocolDll = nullptr;

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
        // put your switch-case.. for getting the right value here

        // finally, save the desired value in the argument val (this is a shared pointer!)
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
        else if (key == "demoKey2")
        {
            // check the new value and if ok, assign it to the internal parameter
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
    ito::RetVal retValue =
        ito::RetVal(ito::retWarning, 0, tr("calibration not possible").toLatin1().data());

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Further action is not possible").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        retValue += homingMCS();
        // todo:
        // start calibrating the given axes and don't forget to regularily call setAlive().
        // this is important if the calibration needs more time than the timeout time of itom (e.g.
        // 5sec). itom regularily checks the alive flag and only drops to a timeout if setAlive() is
        // not regularily called (at least all 3-4 secs).
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
            if (i >= 0 && i < m_nrOfAxes)
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
        getPos(QVector<int>(1, axis), pos2, NULL); // forward to multi-axes version
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
        if (i >= 0 && i < m_nrOfAxes)
        {
            int intPos;
            retValue += getPosMCS(intPos);
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
            if (i < 0 || i >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = 0.0; // todo: set the absolute target position to the desired value
                                      // in mm or degree
            }
        }

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

            // emit the signal targetChanged with m_targetPos as argument, such that all connected
            // slots gets informed about new targets
            sendTargetUpdate();

            // emit the signal sendStatusUpdate such that all connected slots gets informed about
            // changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();

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
            retValue += waitForDone(100, axis); // WaitForAnswer(60000, axis);

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
            if (i < 0 || i >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = 0.0; // todo: set the absolute target position to the desired value
                                      // in mm or degree (obtain the absolute position with respect
                                      // to the given relative distances)
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
                m_targetPos[axis[naxis]] = m_currentPos[axis[naxis]] + pos[naxis];
            }

            // emit the signal targetChanged with m_targetPos as argument, such that all connected
            // slots gets informed about new targets
            sendTargetUpdate();

            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                int intPos;
                retValue += setPosRelMCS(intPos);
                mmProtSendCommand(m_node, 0x0000, eMomancmd_EnOp, 0, 0);
            }

            // emit the signal sendStatusUpdate such that all connected slots gets informed about
            // changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();

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
            // retValue += waitForDone(100, axis); // WaitForAnswer(60000, axis);

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
ito::RetVal FaulhaberMCS::getSerialNumber(int& serialNum)
{
    ito::RetVal retVal(ito::retOk);
    eMomanprot error = mmProtGetObj(m_node, 0x1018, 0x04, serialNum);
    if (error != eMomanprot_ok)
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during get serial number method with error message: '%1'!")
                .arg(mmProtGetErrorMessage(error))
                .toLatin1()
                .data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getVendorID(int& id)
{
    ito::RetVal retVal(ito::retOk);
    eMomanprot error = mmProtGetObj(m_node, 0x1018, 0x01, id);
    if (error != eMomanprot_ok)
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during get vendor id method with error message: '%1'!")
                .arg(mmProtGetErrorMessage(error))
                .toLatin1()
                .data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getProductCode(int& code)
{
    ito::RetVal retVal(ito::retOk);
    eMomanprot error = mmProtGetObj(m_node, 0x1018, 0x02, code);
    if (error != eMomanprot_ok)
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during get product code method with error message: '%1'!")
                .arg(mmProtGetErrorMessage(error))
                .toLatin1()
                .data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getRevisionNumber(int& num)
{
    ito::RetVal retVal(ito::retOk);
    eMomanprot error = mmProtGetObj(m_node, 0x1018, 0x03, num);
    if (error != eMomanprot_ok)
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during get revision number method with error message: '%1'!")
                .arg(mmProtGetErrorMessage(error))
                .toLatin1()
                .data());
    }
    return retVal;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getDeviceName(const char*& name)
{
    ito::RetVal retVal(ito::retOk);
    eMomanprot error = mmProtGetStrObj(m_node, 0x1008, 0x00, &name);
    if (error != eMomanprot_ok)
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during get device name method with error message: '%1'!")
                .arg(mmProtGetErrorMessage(error))
                .toLatin1()
                .data());
    }
    return retVal;
}

ito::RetVal FaulhaberMCS::getSoftwareVersion(const char*& version)
{
    ito::RetVal retVal(ito::retOk);
    eMomanprot error = mmProtGetStrObj(m_node, 0x100A, 0x00, &version);
    if (error != eMomanprot_ok)
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during get software version method with error message: '%1'!")
                .arg(mmProtGetErrorMessage(error))
                .toLatin1()
                .data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::getPosMCS(int& pos)
{
    ito::RetVal retVal(ito::retOk);
    eMomanprot error = mmProtGetObj(m_node, 0x6064, 0x00, pos);
    if (error != eMomanprot_ok)
    {
        retVal += ito::RetVal(
            ito::retError,
            0,
            tr("Error during get position method with error message: '%1'!")
                .arg(mmProtGetErrorMessage(error))
                .toLatin1()
                .data());
    }
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::setPosRelMCS(int& pos)
{
    ito::RetVal retVal(ito::retOk);
    mmProtSendCommand(m_node, 0x0000, eMomancmd_MR, sizeof(pos), pos);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FaulhaberMCS::homingMCS()
{
    ito::RetVal retVal(ito::retOk);
    mmProtSendCommand(m_node, 0x0000, eMomancmd_HS, 0, 0);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method must be overwritten from ito::AddInActuator
/*!
    WaitForDone should wait for a moving motor until the indicated axes (or all axes of nothing is
   indicated) have stopped or a timeout or user interruption occurred. The timeout can be given in
   milliseconds, or -1 if no timeout should be considered. The flag-parameter can be used for your
   own purpose.
*/
ito::RetVal FaulhaberMCS::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
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

    // if axis is empty, all axes should be observed by this method
    QVector<int> _axis = axis;
    if (_axis.size() == 0) // all axis
    {
        for (int i = 0; i < m_nrOfAxes; i++)
        {
            _axis.append(i);
        }
    }

    while (!done && !timeout)
    {
        // todo: obtain the current position, status... of all given axes

        done = true; // assume all axes at target
        motor = 0;
        foreach (const int& i, axis)
        {
            m_currentPos[i] = 0.0; // todo: if possible, set the current position if axis i to its
                                   // current position

            if (1 /*axis i is still moving*/)
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = false; // not done yet
            }
            else if (1 /*axis i is at target*/)
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorAtTarget,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = false; // not done yet
            }
        }

        // emit actuatorStatusChanged with both m_currentStatus and m_currentPos as arguments
        sendStatusUpdate(false);

        // now check if the interrupt flag has been set (e.g. by a button click on its dock widget)
        if (!done && isInterrupted())
        {
            // todo: force all axes to stop

            // set the status of all axes from moving to interrupted (only if moving was set before)
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);

            retVal += ito::RetVal(ito::retError, 0, "interrupt occurred");
            done = true;
            return retVal;
        }

        // short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, delay);
        waitMutex.unlock();

        // raise the alive flag again, this is necessary such that itom does not drop into a timeout
        // if the positioning needs more time than the allowed timeout time.
        setAlive();

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS)
                timeout = true;
        }
    }

    if (timeout)
    {
        // timeout occured, set the status of all currently moving axes to timeout
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError, 9999, "timeout occurred");
        sendStatusUpdate(true);
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
    for (int i = 0; i < m_nrOfAxes; i++)
    {
        m_currentStatus[i] = m_currentStatus[i] |
            ito::actuatorAvailable; // set this if the axis i is available, else use
        // m_currentStatus[i] = m_currentStatus[i] ^ ito::actuatorAvailable;

        m_currentPos[i] = 0.0; // todo fill in here the current position of axis i in mm or degree

        // if you know that the axis i is at its target position, change from moving to target if
        // moving has been set, therefore:
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
