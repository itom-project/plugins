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

// this plugin is also inspired by https://github.com/roesel/elliptec.

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

    // up to 16 devices can be connected to one Elliptec bus distributor
    auto meta = new ito::IntMeta(0x0, 0xF, 1, "Communication");
    int addresses[] = { 0x0, };
    meta->setRepresentation(ito::ParamMeta::HexNumber);
    paramVal = ito::Param(
        "addresses",
        ito::ParamBase::Int | ito::ParamBase::In,
        0x0,
        meta,
        tr("Address of stages, 0x0 - 0xF.").toUtf8().data());
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
    AddInActuator(),
    m_serialBufferSize(200),
    m_requestTimeOutMS(500),
    m_serialMutexLocked(false)
{
    ThorlabsElliptec::initElliptecModels();

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

    paramVal = ito::Param("comPort",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        1, nullptr, "");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("numaxis",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        1,
        1,
        1,
        tr("Number of axes attached to this stage: Here always 1. Multiple axes, connected to one bus driver, must init multiple objects with the same serial object.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("travelRange",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        INT_MIN,
        INT_MAX,
        1,
        tr("Travel range of the axis").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    QVector<ito::Param> pMand, pOpt, pOut;

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

    //------------------------------------------------- DOCK WIDGET
    DockWidgetThorlabsElliptec* dw = new DockWidgetThorlabsElliptec(getID(), this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
        QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<const char*>()), features, areas, dw);
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
    m_address = paramsOpt->at(0).getVal<int>();

    if (m_pSerialIO->getBasePlugin()->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSerialIO->getUserMutex().lock();

        QSharedPointer<ito::Param> val(new ito::Param("port"));
        retValue += m_pSerialIO->getParam(val, nullptr);

        if (!retValue.containsError())
        {
            m_params["comPort"].copyValueFrom(val.data());
        }

        if (!retValue.containsError())
        {
            retValue += m_pSerialIO->setParam(
                QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 9600)),
                nullptr);
            retValue += m_pSerialIO->setParam(
                QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)),
                nullptr);
            retValue += m_pSerialIO->setParam(
                QSharedPointer<ito::ParamBase>(
                    new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)),
                nullptr);
            retValue += m_pSerialIO->setParam(
                QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)),
                nullptr);
            retValue += m_pSerialIO->setParam(
                QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)),
                nullptr);
            retValue += m_pSerialIO->setParam(
                QSharedPointer<ito::ParamBase>(
                    new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")),
                nullptr);

            QSharedPointer<QVector<ito::ParamBase>> _dummy;
            m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
            m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);
        }

        m_pSerialIO->getUserMutex().unlock();
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
        retValue += identifyDevices();
    }

    if (!retValue.containsError())
    {
        retValue += updateStatus();

        /*ito::int32 pos;

        for (int i = 0; i < m_numOfAxes; i++)
        {
            retValue += getPosMCS(pos);
            m_currentPos[i] = static_cast<double>(pos);

            retValue += getTargetPosMCS(pos);
            m_targetPos[i] = static_cast<double>(pos);
            m_currentStatus[i] =
                ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
        }

        retValue += updateStatus();*/
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

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//------------------------------------------------------------------------------
/*static*/ void ThorlabsElliptec::initElliptecModels()
{
    if (elliptecModels.size() == 0)
    {
        elliptecModels << ElliptecDevice(
            6,
            "ELL6",
            "Dual-Position Slider",
            true,
            true,
            "mm",
            2,
            "in;gs;ho;ma;mr;gp"
        );

        elliptecModels << ElliptecDevice(
            9,
            "ELL9",
            "Four-Position Slider",
            true,
            true,
            "mm",
            4,
            "in;gs;ho;ma;mr;gp"
        );

        elliptecModels << ElliptecDevice(
            12,
            "ELL12",
            "Six-Position Slider",
            true,
            true,
            "mm",
            6,
            "in;gs;ho;ma;mr;gp"
        );

        elliptecModels << ElliptecDevice(
            14,
            "ELL14",
            "Rotation Mount",
            false,
            false,
            "°",
            0,
            "in;gs;ho;ma;mr;gp"
        );

        elliptecModels << ElliptecDevice(
            15,
            "ELL15",
            "Motorized Iris",
            false,
            true,
            "mm",
            0,
            "in;gs;ho;ma;mr;gp"
        );

        elliptecModels << ElliptecDevice(
            17,
            "ELL17",
            "Linear Stage",
            false,
            true,
            "mm",
            0,
            "in;gs;ho;ma;mr;gp"
        );

        elliptecModels << ElliptecDevice(
            18,
            "ELL18",
            "Rotation Stage",
            false,
            false,
            "°",
            0,
            "in;gs;ho;ma;mr;gp"
        );

        elliptecModels << ElliptecDevice(
            20,
            "ELL20",
            "Linear Stage",
            false,
            true,
            "mm",
            0,
            "in;gs;ho;ma;mr;gp"
        );
    }
}

//------------------------------------------------------------------------------
/*static*/ void ThorlabsElliptec::initSupportedCmds()
{
    if (supportedCmds.size() > 0)
    {
        return;
    }

    supportedCmds["in"] = CmdInfo("in", "IN", 0, 30, false); // identify
    supportedCmds["gs"] = CmdInfo("gs", "GS", 0, 2, false); // status
    supportedCmds["ho"] = CmdInfo("ho", "PO", 1, 8, true); // homing
    supportedCmds["ma"] = CmdInfo("ma", "PO", 8, 8, true); // move absolute
    supportedCmds["mr"] = CmdInfo("mr", "PO", 8, 8, true); // move relative
    supportedCmds["gp"] = CmdInfo("gp", "PO", 0, 8, false); // get position
}

//------------------------------------------------------------------------------
bool ThorlabsElliptec::getCmdInfo(const QByteArray& cmd, CmdInfo& info) const
{
    if (!m_model.m_supportedCmds.contains(cmd))
    {
        return false;
    }

    if (!supportedCmds.contains(cmd))
    {
        return false;
    }

    info = supportedCmds[cmd];
    return true;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::identifyDevices()
{
    ito::RetVal retValue;
    QByteArray response;
    bool ok = true;
    bool modelFound = false;

    unsigned char adr = (unsigned char)m_address;
    retValue += sendCommandAndGetResponse(adr, "in", "", response);

    if (!retValue.containsError())
    {
        int motorType = byteArrayToInt(response.mid(3 - 3, 2));
        QByteArray serial = response.mid(5 - 3, 8);
        QByteArray year = response.mid(13 - 3, 4);
        int fwRelease = byteArrayToInt(response.mid(17 - 3, 2));
        int hwRelease = byteArrayToInt(response.mid(19 - 3, 2));
        int travelRange = byteArrayToInt(response.mid(21 - 3, 4));
        int pulses = byteArrayToInt(response.mid(25 - 3, 8));

        m_params["travelRange"].setVal<int>(travelRange);

        foreach(const auto & model, elliptecModels)
        {
            if (model.m_modelId == motorType)
            {
                m_model = model;
                modelFound = true;
                break;
            }
        }

        if (!modelFound)
        {
            retValue += ito::RetVal::format(ito::retError, 0, "Unknown or unsupported model type for address %i", m_address);
        }
        else
        {
            setIdentifier(
                QString("Elliptec %1, Serial %2").arg(m_model.m_name).arg(serial)
            );
        }
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError,
            0,
            "Identification of device at address %i failed. Response format mismatch.",
            adr);
        return retValue;
    }
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

    if (axis.size() != 1 || pos->size() != 1 || axis[0] != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "Only one axis supported (index 0)");
    }
    else
    {
        QByteArray response;
        retValue += sendCommandAndGetResponse(m_address, "gp", "", response);

        if (!retValue.containsError())
        {
            double value = byteArrayToInt(response);

            if (m_model.m_indexed)
            {
                // convert value to indexed values 0, 1, 2, 3 ... (up to number of indexed positions)
                double travelRange = m_params["travelRange"].getVal<int>();
                double factor = travelRange / (m_model.m_numIndexedPositions - 1);
                (*pos)[0] = qRound(value / factor) + 1.;
            }
            else
            {
                (*pos)[0] = value;
            }
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

    if (axis.size() != 1 || pos.size() != 1 || axis[0] != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "Only one axis supported (index 0)");
    }
    else if (isMotorMoving())
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
            if (i < 0 || i >= )
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
ito::RetVal ThorlabsElliptec::sendCommand(unsigned char address, const QByteArray& cmdId, const QByteArray& data = QByteArray())
{
    // data must be a big-endian string representation of a number, if it is a number!
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
    m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);

    QByteArray cmd = QByteArray::number(address, 16) + cmdId + data;
    ito::RetVal retVal = m_pSerialIO->setVal(cmd.constData(), cmd.length(), nullptr);
    setAlive();
    return retVal;
}

//------------------------------------------------------------------------------
QByteArray ThorlabsElliptec::intToByteArray(int value, int numBytes) const
{
    QByteArray byteArray(numBytes, 0); // Erstelle ein QByteArray mit 4 Bytes

    // Erstelle einen QDataStream, um die Zahl in das QByteArray zu schreiben
    QDataStream stream(&byteArray, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian); // Setze die Byte-Reihenfolge auf Big-Endian
    stream << value; // Schreibe die Zahl in das Byte-Array

    // Ausgabe des Byte-Arrays zur Überprüfung
    return byteArray.toHex();
}

//------------------------------------------------------------------------------
int ThorlabsElliptec::byteArrayToInt(const QByteArray& value) const
{
    return value.toInt(nullptr, 16);
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::sendCommandAndGetResponse(
    unsigned char address, const QByteArray& cmdId, const QByteArray& data, QByteArray& response)
{
    ito::RetVal retVal = ito::retOk;
    bool serialMutexLockedLocal = false;

    if (!m_serialMutexLocked)
    {
        m_pSerialIO->getUserMutex().lock();
        m_serialMutexLocked = true;
        serialMutexLockedLocal = true;
    }

    CmdInfo info;

    if (getCmdInfo(cmdId, info))
    {
        if (data.size() != info.sendDataNumBytes)
        {
            retVal += ito::RetVal(ito::retError, 0, "required data of cmd has not the same length than the given data");
        }
        else
        {
            retVal += sendCommand(address, cmdId, data);
        }
    }
    else
    {
        retVal += ito::RetVal(ito::retError, 0, "invalid or unsupported cmd for this device");
    }

    if (!retVal.containsError())
    {
        retVal += readResponse(response);
    }

    if (!retVal.containsError())
    {
        if (response.size() != (3 + info.rcvDataNumBytes))
        {
            retVal += ito::RetVal(ito::retError, 0, "length of device response is wrong.");
        }
        else if (response[0] != address)
        {
            retVal += ito::RetVal(ito::retError, 0, "device response is assigned to wrong address.");
        }
        else if (response.mid(1, 2) == info.rcvCmd)
        {
            // correct answer
            response = response.mid(3);
        }
        else if (response.mid(1, 2) == "GS" && info.canReturnStatus)
        {
            retVal += parseStatusResponse(response);
        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, "invalid response");
        }
    }

    if (serialMutexLockedLocal)
    {
        m_pSerialIO->getUserMutex().unlock();
        m_serialMutexLocked = false;
        serialMutexLockedLocal = false;
    }
    return retVal;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::sendCommandAndGetResponse(unsigned char address, const QByteArray& cmdId, int data, QByteArray& response)
{
    CmdInfo info;

    if (getCmdInfo(cmdId, info))
    {
        QByteArray dataStr = intToByteArray(data, info.sendDataNumBytes);
        return sendCommandAndGetResponse(address, cmdId, dataStr, response);
    }
    else
    {
        return ito::RetVal(ito::retError, 0, "invalid or unsupported cmd for this device");
    }
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::readResponse(QByteArray& response)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    *m_serialBufferLength = m_serialBufferSize;
    std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);

    ito::uint8 length;
    ito::uint8 recievedCommand;
    response = "";

    bool done = false;
    int offset = 0;
    int start = 0;
    int endIndex = 0;

    timer.start();

    while (!done && !retValue.containsError())
    {
        QThread::msleep(10);
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
            if (response.endsWith("\r\n"))
            {
                done = true;
            }
        }

        if (!done && timer.elapsed() > m_requestTimeOutMS && m_requestTimeOutMS >= 0)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("timeout during read command.").toUtf8().data());
            return retValue;
        }
    }

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::parseStatusResponse(const QByteArray& response) const
{
    if (response.size() != 4)
    {
        return ito::RetVal(ito::retError, 0, "invalid response size");
    }
    else if (response.mid(1, 2) != "GS")
    {
        return ito::RetVal(ito::retError, 0, "invalid response command");
    }
    else
    {
        int status = response.mid(3, 1).toInt(nullptr, 16);

        switch (status)
        {
        case 0:
            return ito::retOk;
        case 1:
            return ito::RetVal(ito::retError, status, "Communication time out");
        case 2:
            return ito::RetVal(ito::retError, status, "Mechanical time out");
        case 3:
            return ito::RetVal(ito::retError, status, "Command error or not supported");
        case 4:
            return ito::RetVal(ito::retError, status, "Value out of range");
        case 5:
            return ito::RetVal(ito::retError, status, "Module isolated");
        case 6:
            return ito::RetVal(ito::retError, status, "Module out of isolation");
        case 7:
            return ito::RetVal(ito::retError, status, "Initializing error");
        case 8:
            return ito::RetVal(ito::retError, status, "Thermal error");
        case 9:
            return ito::RetVal(ito::retError, status, "Busy");
        case 10:
            return ito::RetVal(ito::retError, status, "Sensor Error(May appear during self - test.If code persists there is an error)");
        case 11:
            return ito::RetVal(ito::retError, status, "Motor Error(May appear during self - test.If code persists there is an error)");
        case 12:
            return ito::RetVal(ito::retError, status, "Out of Range(e.g., stage has been instructed to move beyond its travel range).");
        case 13:
            return ito::RetVal(ito::retError, status, "Over Current error");
        default:
            return ito::RetVal(ito::retError, status, "Reserved error code");
        }
    }
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
