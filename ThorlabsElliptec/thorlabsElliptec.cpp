/* ********************************************************************
    Plugin "Thorlabs Elliptec" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2025, Institut für Technische Optik (ITO),
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

#include "thorlabsElliptec.h"
#include "common/helperCommon.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>

#include "dockWidgetThorlabsElliptec.h"

QList<ito::uint8> ThorlabsElliptec::openedNodes = QList<ito::uint8>();

//------------------------------------------------------------------------------
ThorlabsElliptecInterface::ThorlabsElliptecInterface()
{
    m_type = ito::typeActuator;
    setObjectName("ThorlabsElliptec");

    m_description = QObject::tr("ThorlabsElliptec");

    m_detaildescription =
        QObject::tr("This plugin is an actuator plugin to control resonant piezoelectric motors from Elliptec / Thorlabs.");

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
        tr("An opened serial port of 'SerialIO' plugin instance.").toUtf8().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    auto meta = new ito::IntMeta(0x0, 0xF, 1, "Communication");
    meta->setRepresentation(ito::ParamMeta::HexNumber);
    paramVal = ito::Param(
        "address",
        ito::ParamBase::Int | ito::ParamBase::In,
        0x0,
        meta,
        tr("Address of stage, 0x0 - 0xF.").toUtf8().data());
    m_initParamsOpt.append(paramVal);
}

//------------------------------------------------------------------------------
ThorlabsElliptecInterface::~ThorlabsElliptecInterface()
{
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptecInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(ThorlabsElliptec)
        return ito::retOk;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptecInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(ThorlabsElliptec)
        return ito::retOk;
}

//------------------------------------------------------------------------------
ThorlabsElliptec::ThorlabsElliptec() :
    AddInActuator(), m_delayAfterSendCommandMS(50), m_async(0), m_numOfAxes(1), m_node(1),
    m_statusWord(0x0000), m_requestTimeOutMS(5000), m_waitForDoneTimeout(60000),
    m_waitForMCSTimeout(3000), m_nodeAppended(false), m_serialBufferSize(100)
{
    m_serialBuffer = QSharedPointer<char>(new char[m_serialBufferSize], [](char* ptr) {
        delete[] ptr; // Custom deleter to release the array properly
        });
    m_serialBufferLength = QSharedPointer<int>(new int(m_serialBufferSize));
    *m_serialBufferLength = m_serialBufferSize;
    // Clear the buffer initially
    std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);

    ito::Param paramVal(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        tr("ThorlabsElliptec").toUtf8().data(),
        nullptr);
    m_params.insert(paramVal.getName(), paramVal);





    registerExecFunc(
        "homing",
        pMand,
        pOpt,
        pOut,
        tr("In most of the cases before position control is to be used, the drive must perform a "
            "reference run to align the position used by the drive to the mechanic setup.")
        .toUtf8()
        .data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // initialize the current position vector, the status vector and the target position vector
    m_currentPos.fill(0.0, m_numOfAxes);
    m_currentStatus.fill(0, m_numOfAxes);
    m_targetPos.fill(0.0, m_numOfAxes);

    //------------------------------------------------- DOCK WIDGET
    DockWidgetThorlabsElliptec* dw = new DockWidgetThorlabsElliptec(getID(), this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
        QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);
}

//------------------------------------------------------------------------------
ThorlabsElliptec::~ThorlabsElliptec()
{
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    m_pSerialIO = paramsMand->at(0).getVal<ito::AddInDataIO*>();

    if (m_pSerialIO->getBasePlugin()->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        QSharedPointer<ito::Param> val(new ito::Param("port"));
        retValue += m_pSerialIO->getParam(val, NULL);

        if (!retValue.containsError())
        {
            m_port = val->getVal<int>();
        }
        m_node = (ito::uint8)paramsMand->at(1).getVal<int>();
        if (openedNodes.contains(m_node))
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("An instance of noder number '%1' is already open.")
                .arg(m_node)
                .toUtf8()
                .data());
        }
        else
        {
            ito::uint8 node;
            retValue += getNodeID(node);
            if (!retValue.containsError())
            {
                if (node != m_node)
                {
                    retValue = ito::RetVal(
                        ito::retError,
                        0,
                        tr("The node number of the device is '%1' and not '%2'.")
                        .arg(node)
                        .arg(m_node)
                        .toUtf8()
                        .data());
                }
                else
                {
                    openedNodes.append(m_node);
                    m_nodeAppended = true;
                    m_params["nodeID"].setVal<int>(m_node);
                }
            }
            else
            {
                retValue = ito::RetVal(
                    ito::retError,
                    999,
                    tr("No device found for serialIO port '%1' and node '%2'.")
                    .arg(m_port)
                    .arg(m_node)
                    .toUtf8()
                    .data());
            }
        }
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Input parameter is not a dataIO instance of the SerialIO Plugin!").toUtf8().data());
    }

    if (!retValue.containsError())
    {
        QSharedPointer<QVector<ito::ParamBase>> _dummy;
        m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_serialBuffer = QSharedPointer<char>(new char[m_serialBufferSize]);
        m_serialBufferLength = QSharedPointer<int>(new int(m_serialBufferSize));

        *m_serialBufferLength = m_serialBufferSize;
        std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);


        retValue += setCommunicationSettings(TRANSMIT_EMCY_VIA_RS232); // Transmit EMCYs via RS232
    }



    if (!retValue.containsError())
    {
        retValue += updateStatus();

        ito::int32 pos;

        for (int i = 0; i < m_numOfAxes; i++)
        {
            retValue += getPosMCS(pos);
            m_currentPos[i] = static_cast<double>(pos);

            retValue += getTargetPosMCS(pos);
            m_targetPos[i] = static_cast<double>(pos);
            m_currentStatus[i] =
                ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
        }

        retValue += updateStatus();
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

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_nodeAppended)
    {
        openedNodes.removeOne(m_node);
        if (openedNodes.isEmpty())
        {
            m_serialBuffer.clear();
            m_serialBufferLength.clear();
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
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

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::setParam(
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
            ito::retError, 0, tr("any axis is moving. Parameters cannot be set.").toUtf8().data());
    }

    if (!retValue.containsError())
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (!retValue.containsError())
        {
            retValue += it->copyValueFrom(&(*val));
        }
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

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::calib(const int axis, ItomSharedSemaphore* waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::calib(const QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    retValue +=
        ito::RetVal(ito::retError, 0, tr("'Calib' function is not implemented.").toUtf8().data());
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}



//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::setOrigin(const int axis, ItomSharedSemaphore* waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::setOrigin(QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue +=
            ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toUtf8().data());

        if (waitCond)
        {
            waitCond->release();
            waitCond->returnValue = retValue;
        }
    }
    else
    {
        foreach(const int& i, axis)
        {
            retValue += homingCurrentPosToZero(i);
            setStatus(
                m_currentStatus[i],
                ito::actuatorAtTarget,
                ito::actSwitchesMask | ito::actStatusMask);
            m_params["homed"].setVal<int>(1);
        }

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        sendStatusUpdate();
        sendTargetUpdate();
    }

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::getStatus(
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

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::getPos(
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

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::getPos(
    QVector<int> axis, QSharedPointer<QVector<double>> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    foreach(const int i, axis)
    {
        if (i >= 0 && i < m_numOfAxes)
        {
            ito::int32 intPos;
            retValue += getPosMCS(intPos);
            if (!retValue.containsError())
            {
                m_currentPos[i] =
                    static_cast<double>(intPos); // set m_currentPos[i] to the obtained position
                (*pos)[i] = m_currentPos[i];
            }
        }
        else
        {
            retValue += ito::RetVal::format(
                ito::retError, 1, tr("axis %i not available").toUtf8().data(), i);
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

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::setPosAbs(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::setPosAbs(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // check if pos in integer32 value range
    foreach(const auto i, axis)
    {
        if (pos[i] > std::numeric_limits<ito::int32>::max() ||
            pos[i] < std::numeric_limits<ito::int32>::min())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Target position %1 is out of range").arg(pos[i]).toUtf8().data());
            waitCond->returnValue = retValue;
            waitCond->release();
            return retValue;
        }
    }

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toUtf8().data());
    }
    else
    {
        foreach(const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toUtf8().data(), i);
            }
            else
            {
                m_targetPos[i] = pos[i];
            }
        }

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            foreach(const int i, axis)
            {
                ito::int32 newVal = static_cast<ito::int32>(pos[i]);
                retValue += setPosAbsMCS(newVal);
                m_targetPos[i] = static_cast<ito::float64>(newVal);
            }

            sendTargetUpdate();

            if (m_async && waitCond) // async disabled
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            retValue += waitForDone(m_waitForDoneTimeout, axis); // drops into timeout

            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }


    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::setPosRel(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::setPosRel(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // check if pos in integer32 value range
    foreach(const auto i, axis)
    {
        if (pos[i] > std::numeric_limits<ito::int32>::max() ||
            pos[i] < std::numeric_limits<ito::int32>::min())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Relative position %1 is out of range").arg(pos[i]).toUtf8().data());
            waitCond->returnValue = retValue;
            waitCond->release();
            return retValue;
        }
    }

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toUtf8().data());
    }
    else
    {
        foreach(const int i, axis)
        {
            if (i < 0 || i >= m_numOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("axis %i not available").toUtf8().data(), i);
            }
            else
            {
                m_targetPos[i] = m_currentPos[i] + pos[i];
            }
        }

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            foreach(const int i, axis)
            {
                ito::int32 newPos = static_cast<ito::int32>(pos[i]);
                retValue += setPosRelMCS(newPos);
                m_currentPos[i] = static_cast<ito::float64>(newPos);
            }

            sendTargetUpdate();

            if (m_async && waitCond) // async disabled
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            retValue += waitForDone(m_waitForDoneTimeout, axis); // drops into timeout

            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate();

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::execFunc(
    const QString funcName,
    QSharedPointer<QVector<ito::ParamBase>> paramsMand,
    QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
    QSharedPointer<QVector<ito::ParamBase>> paramsOut,
    ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;

    if (funcName == "homing")
    {
        retValue += updateStatus();

        if (!m_params["operationEnabled"].getVal<int>())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Operation is not enabled. Please enable operation first.").toUtf8().data());
        }

        if (!retValue.containsError())
        {
            ito::int8 method = static_cast<ito::int8>((*paramsMand)[0].getVal<int>());
            ito::int32 offset = static_cast<ito::int32>((*paramsOpt)[0].getVal<int>());
            ito::uint32 switchSeekVelocity =
                static_cast<ito::uint32>((*paramsOpt)[1].getVal<int>());
            ito::uint32 homingSpeed = static_cast<ito::uint32>((*paramsOpt)[2].getVal<int>());
            ito::uint32 acceleration = static_cast<ito::uint32>((*paramsOpt)[3].getVal<int>());
            ito::uint16 limitCheckDelayTime =
                static_cast<ito::uint16>((*paramsOpt)[4].getVal<int>());

            ito::uint16 negativeLimit = static_cast<ito::uint16>((*paramsOpt)[5].getVal<int*>()[0]);
            ito::uint16 positiveLimit = static_cast<ito::uint16>((*paramsOpt)[5].getVal<int*>()[1]);

            retValue += performHoming(
                method,
                offset,
                switchSeekVelocity,
                homingSpeed,
                acceleration,
                limitCheckDelayTime,
                negativeLimit,
                positiveLimit);
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



//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    char motor;
    int currentPos = 0;
    int targetPos = 0;
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
            quickStop();
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += startupSequence();
            done = true;
            sendStatusUpdate();
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toUtf8().data());
            return retVal;
        }

        if (!retVal.containsError()) // short delay to reduce CPU load
        {
            // short delay of 10ms
            waitMutex.lock();
            waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
            waitMutex.unlock();
            setAlive();
        }

        foreach(auto i, axis) // Check for completion
        {
            retVal += getPosMCS(currentPos);
            m_currentPos[i] = static_cast<double>(currentPos);

            retVal += getTargetPosMCS(targetPos);
            m_targetPos[i] = static_cast<double>(targetPos);

            retVal += updateStatus();
            if ((m_statusWord[10])) // target reached bit
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorAtTarget,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true;

                retVal += getPosMCS(currentPos);
                m_currentPos[i] = static_cast<double>(currentPos);

                retVal += getTargetPosMCS(targetPos);
                m_targetPos[i] = static_cast<double>(targetPos);

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
        }

        sendStatusUpdate(false);

        if (timer.hasExpired(timeoutMS)) // timeout during movement
        {
            timeout = true;
            // timeout occurred, set the status of all currently moving axes to timeout
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            retVal += ito::RetVal(
                ito::retError,
                9999,
                "timeout occurred during movement. If necessary increase the parameter "
                "'moveTimeout'.");
            sendStatusUpdate(true);

            quickStop();
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += startupSequence();
            sendStatusUpdate();
            return retVal;
        }
    }

    return retVal;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::updateStatus()
{
    ito::RetVal retVal = updateStatusMCS();

    for (int i = 0; i < m_numOfAxes; i++)
    {
        m_currentStatus[i] = m_currentStatus[i] | ito::actuatorAvailable;

        ito::int32 intPos;
        retVal += getPosMCS(intPos);

        m_currentPos[i] = static_cast<double>(intPos);

        if (m_params["targetReached"].getVal<int>())
        {
            replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget);
        }
        else
        {
            setStatus(
                m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        }
    }

    emit actuatorStatusChanged(m_currentStatus, m_currentPos);
    sendStatusUpdate();

    return retVal;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::sendCommand(const QByteArray& command)
{
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
    m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);

    ito::RetVal retVal = m_pSerialIO->setVal(command.data(), command.length(), nullptr);
    setAlive();
    return retVal;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::sendCommandAndGetResponse(const QByteArray& command, QByteArray& response)
{
    ito::RetVal retVal = ito::retOk;
    retVal += sendCommand(command);

    if (!retVal.containsError())
    {
        retVal += readResponse(response, command[3]);
    }

    return retVal;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::readResponse(QByteArray& response, const ito::uint8& command)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    *m_serialBufferLength = m_serialBufferSize;
    std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);

    ito::uint8 length;
    ito::uint8 recievedCommand;

    bool done = false;
    int offset = 0;
    int start = 0;
    int endIndex = 0;

    QMutex waitMutex;
    QWaitCondition waitCondition;

    timer.start();

    while (!done && !retValue.containsError())
    {
        waitMutex.lock();
        waitCondition.wait(&waitMutex, m_delayAfterSendCommandMS);
        waitMutex.unlock();
        setAlive();

        retValue += m_pSerialIO->getVal(m_serialBuffer, m_serialBufferLength, nullptr);
        response += QByteArray(m_serialBuffer.data(), *m_serialBufferLength);
        if (retValue.containsError())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Error occurred during reading the response from the serial port.")
                .toUtf8()
                .data());
            break;
        }
        else
        {
            // FOUND
            if ((response.contains(m_S) && response.contains(m_E)))
            {
                done = true;
            }
        }

        if (!done && timer.elapsed() > m_requestTimeOutMS && m_requestTimeOutMS >= 0)
        {
            retValue += ito::RetVal(
                ito::retError,
                m_delayAfterSendCommandMS,
                tr("timeout during read command.").toUtf8().data());
            return retValue;
        }
    }
    if (!(response.contains(m_S) && response.contains(m_E)))
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("The character 'S' or 'E' was not detected in the received bytearray.")
            .toUtf8()
            .data());
    }

    if (!retValue.containsError())
    {
        int start = response.indexOf(m_S);
        response = response.right(start + response.size() + 1);
        length = static_cast<ito::uint8>(response[1]);
        recievedCommand = static_cast<ito::uint8>(response[3]);

        if (recievedCommand == 0x03) // error
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            retValue += interpretCIA402Error(response.sliced(start, length + 1));
#else
            retValue += interpretCIA402Error(response.mid(start, length + 1));
#endif
        }
    }

    return retValue;
}

//------------------------------------------------------------------------------
template <typename T>
inline ito::RetVal ThorlabsElliptec::readRegisterWithParsedResponse(
    const ito::uint16& index, const ito::uint8& subindex, T& answer)
{
    QByteArray response;
    ito::RetVal retValue = readRegister(index, subindex, response);
    if (!retValue.containsError())
        retValue += parseResponse<T>(response, answer);
    return retValue;
}

//------------------------------------------------------------------------------
template <typename T>
ito::RetVal ThorlabsElliptec::parseResponse(QByteArray& response, T& parsedResponse)
{
    ito::RetVal retValue = ito::retOk;

    qsizetype startIndex = response.indexOf(m_S);

    // Extract basic components from the response
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ito::uint8 length = static_cast<ito::uint8>(response[startIndex + 1]);
    ito::uint8 nodeNumber = static_cast<ito::uint8>(response[startIndex + 2]);
    ito::uint8 command = static_cast<ito::uint8>(response[startIndex + 3]);

#else
    ito::uint8 length = static_cast<ito::uint8>(response.at(startIndex + 1));
    ito::uint8 nodeNumber = static_cast<ito::uint8>(response.at(startIndex + 2));
    ito::uint8 command = static_cast<ito::uint8>(response.at(startIndex + 3));
#endif

    // Verify node number
    if (nodeNumber != m_node)
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("The node number '%1' does not match the expected node number '%2'.")
            .arg(nodeNumber)
            .arg(m_node)
            .toUtf8()
            .data());
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ito::uint8 receivedCRC = static_cast<ito::uint8>(response[startIndex + length]);
#else
    ito::uint8 receivedCRC = static_cast<ito::uint8>(response.at(startIndex + length));
#endif

    ito::uint8 checkCRC = 0x00;
    QByteArray data = "";

    // Process the response based on command type
    switch (command)
    {
    case 0x01: // SDO read request/response
        if (length > 7)
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            checkCRC = calculateChecksum(response.sliced(1, startIndex + length - 1));
            data = response.sliced(7, startIndex + length - 7 + 1);
#else
            checkCRC = calculateChecksum(response.mid(1, startIndex + length - 1));
            data = response.mid(7, startIndex + length - 7 + 1);
#endif
        }
        else
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            checkCRC = calculateChecksum(response.sliced(1, startIndex + length));
#else
            checkCRC = calculateChecksum(response.mid(1, startIndex + length));
#endif
        }

        if (receivedCRC != checkCRC)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Checksum mismatch for SDO read request/response (received: '%1', "
                    "calculated: "
                    "'%2').")
                .arg(receivedCRC)
                .arg(checkCRC)
                .toUtf8());
        }

        if constexpr (std::is_same<T, QString>::value) // convert to QString
        {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
            parsedResponse = QString::fromUtf8(data.sliced(0, data.size() - 1));
#else
            parsedResponse = QString::fromUtf8(data.mid(0, data.size() - 1));
#endif
        }
        else if constexpr (

            std::is_integral<T>::value || std::is_floating_point<T>::value) // convert to integer
        {
            if (data.size() >= sizeof(T))
            {
                std::memcpy(&parsedResponse, data.constData(), sizeof(T));
            }
            else
            {
                return ito::RetVal(ito::retError, 0, tr("Data size mismatch").toUtf8().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Unsupported type").toUtf8().data());
        }
        break;

    case 0x02: // SDO write request/response
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        checkCRC = calculateChecksum(response.sliced(1, length - 1));
#else
        checkCRC = calculateChecksum(response.mid(1, length - 1));
#endif
        if (receivedCRC != checkCRC)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Checksum mismatch for SDO write request/response (received: '%1', "
                    "calculated: "
                    "'%2').")
                .arg(receivedCRC)
                .arg(checkCRC)
                .toUtf8());
        }
        break;
    case 0x05: // SDO write parameter request
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        checkCRC = calculateChecksum(response.sliced(1, length - 1));
#else
        checkCRC = calculateChecksum(response.mid(1, length - 1));
#endif
        if (receivedCRC != checkCRC)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Checksum mismatch for SDO write parameter request/response (received: "
                    "'%1', "
                    "calculated: "
                    "'%2').")
                .arg(receivedCRC)
                .arg(checkCRC)
                .toUtf8());
        }
        m_statusWord = (response[4] << 8) | response[5];

        updateStatusBits();
        emit parametersChanged(m_params);
        sendStatusUpdate();
        break;

    case 0x07: // EMCY notification
        retValue += getError();
        break;

    default:
        return ito::RetVal(
            ito::retError,
            0,
            tr("Unknown command received: %1")
            .arg(static_cast<ito::uint8>(command))
            .toUtf8()
            .data());
    }

    return retValue;
}


//------------------------------------------------------------------------------
template <typename T>
ito::RetVal ThorlabsElliptec::setRegister(
    const ito::uint16& index,
    const ito::uint8& subindex,
    const ito::uint32& value,
    const ito::uint8& length)
{
    ito::RetVal retVal = ito::retOk;
    QByteArray response;
    std::vector<ito::uint8> command = {
        m_node,
        m_SET,
        static_cast<ito::uint8>(index & 0xFF),
        static_cast<ito::uint8>(index >> 8),
        subindex };

    for (int i = 0; i < length; i++)
    {
        command.push_back((value >> (8 * i)) & 0xFF);
    }

    std::vector<ito::uint8> fullCommand = { static_cast<ito::uint8>(command.size() + 2) };
    fullCommand.insert(fullCommand.end(), command.begin(), command.end());
    QByteArray CRC(reinterpret_cast<const char*>(fullCommand.data()), fullCommand.size());
    fullCommand.push_back(calculateChecksum(CRC));
    fullCommand.insert(fullCommand.begin(), m_S);
    fullCommand.push_back(m_E);

    QByteArray data(reinterpret_cast<char*>(fullCommand.data()), fullCommand.size());

    retVal += sendCommandAndGetResponse(data, response);

    if (!retVal.containsError())
    {
        T parsedResponse;
        retVal += parseResponse<T>(response, parsedResponse);
    }

    return retVal;
}

//------------------------------------------------------------------------------
ito::uint8 ThorlabsElliptec::calculateChecksum(const QByteArray& message)
{
    ito::uint8 calcCRC = 0xFF;
    int len = message.size(); // Get the length of the QByteArray

    for (int i = 0; i < len; i++)
    {
        calcCRC = calcCRC ^
            static_cast<ito::uint8>(message[i]); // Access QByteArray data and cast to ito::uint8
        for (ito::uint8 j = 0; j < 8; j++)
        {
            if (calcCRC & 0x01)
                calcCRC = (calcCRC >> 1) ^ 0xd5;
            else
                calcCRC = (calcCRC >> 1);
        }
    }

    return calcCRC;
}

//------------------------------------------------------------------------------
bool ThorlabsElliptec::verifyChecksum(QByteArray& message, ito::uint8& receivedCRC)
{
    // Calculate the CRC of the message excluding SOF, CRC, and EOF
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    ito::uint8 calculatedCRC = calculateChecksum(message.sliced(1, message.size() - 3));
#else
    ito::uint8 calculatedCRC = calculateChecksum(message.mid(1, message.size() - 3));
#endif

    // Compare the calculated CRC with the received CRC
    if (calculatedCRC != receivedCRC)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
void ThorlabsElliptec::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetThorlabsElliptec* widget = (DockWidgetThorlabsElliptec*)(getDockWidget()->widget());

        if (visible)
        {
            connect(
                this, &ThorlabsElliptec::parametersChanged, widget, &DockWidgetThorlabsElliptec::parametersChanged);
            connect(
                this, &ThorlabsElliptec::actuatorStatusChanged, widget, &DockWidgetThorlabsElliptec::actuatorStatusChanged);
            connect(
                this, &ThorlabsElliptec::targetChanged, widget, &DockWidgetThorlabsElliptec::targetChanged);

            emit parametersChanged(m_params);
            sendTargetUpdate();
            sendStatusUpdate(false);
        }
        else
        {
            disconnect(
                this, &ThorlabsElliptec::parametersChanged, widget, &DockWidgetThorlabsElliptec::parametersChanged);
            disconnect(
                this, &ThorlabsElliptec::actuatorStatusChanged, widget, &DockWidgetThorlabsElliptec::actuatorStatusChanged);
            disconnect(
                this, &ThorlabsElliptec::targetChanged, widget, &DockWidgetThorlabsElliptec::targetChanged);
        }
    }
}

//------------------------------------------------------------------------------
const ito::RetVal ThorlabsElliptec::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsElliptec(this));
}
