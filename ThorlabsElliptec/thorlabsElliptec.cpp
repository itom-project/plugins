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

/*static*/ QList<ElliptecDevice> ThorlabsElliptec::elliptecModels = QList<ElliptecDevice>();
/*static*/ QMap<QByteArray, ThorlabsElliptec::CmdInfo> ThorlabsElliptec::supportedCmds =
    QMap<QByteArray, ThorlabsElliptec::CmdInfo>();

#define TIMEOUT_ID 1000

//------------------------------------------------------------------------------
ThorlabsElliptecInterface::ThorlabsElliptecInterface()
{
    m_type = ito::typeActuator;
    setObjectName("ThorlabsElliptec");

    m_description = QObject::tr("ThorlabsElliptec");

    m_detaildescription =
"This plugin is an actuator plugin to control resonant piezoelectric motors from Elliptec / Thorlabs. \n\
\n\
Currently, this plugin supports the following devices: \n\
ELL6, ELL9, ELL12, ELL14, ELL15, ELL17, ELL18, ELL20\n\
\n\
It could only be tested with the rotatory stage ELL18 and metric units (ELL18/M). Devices with imperal units are only \n\
partially supported.\n\
\n\
For properly adjusting the forward and backward frequencies, you can set them via the corresponding parameters \n\
or use the exec functions (see config dialog) for searching optimal frequencies, resetting to default or \n\
execute an optimization run.\n\
\n\
Sometimes, an axis might cause a mechanical timeout error, especially for longer movements. For ``calib`` and \n\
``setPosAbs`` the step is repeated one more time, if this timeout occurs. For ``setPosRel``, this is not possible. ";

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

    // up to 16 devices can be connected to one Elliptec bus distributor
    auto meta = new ito::IntMeta(0x0, 0xF, 1, "Communication");
    meta->setRepresentation(ito::ParamMeta::HexNumber);
    paramVal = ito::Param(
        "address",
        ito::ParamBase::Int | ito::ParamBase::In,
        0x0,
        meta,
        tr("Address of stage, 0x0 - 0xF. The address can be changed by setting the parameter later "
           "on.")
            .toLatin1()
            .data());
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
    m_requestTimeOutMS(1000),
    m_waitForDoneTimeoutMS(180000),
    m_serialMutexLocked(false), 
    m_async(0)
{
    // init some static data and LUTs.
    // executed in main thread, therefore thread-safe!
    ThorlabsElliptec::initElliptecModels();
    ThorlabsElliptec::initSupportedCmds();

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
        tr("ThorlabsElliptec").toLatin1().data(),
        nullptr);
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "serial",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        nullptr);
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("model", ito::ParamBase::String | ito::ParamBase::Readonly, "", nullptr);
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal =
        ito::Param("description", ito::ParamBase::String | ito::ParamBase::Readonly, "", nullptr);
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "numMotors",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        3,
        0,
        "number of piezo actuators to move the stage");
    paramVal.getMeta()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("comPort",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        1, nullptr, "");
    paramVal.setMeta(new ito::IntMeta(1, INT_MAX, 1, "Communication"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "axisType",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        -1,
        1,
        0,
        "Axis type: -1: indexed, 0: rotatory, 1: linear");
    paramVal.getMeta()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    auto meta = new ito::IntMeta(0x0, 0xF, 1, "Communication");
    meta->setRepresentation(ito::ParamMeta::HexNumber);
    paramVal = ito::Param(
        "address",
        ito::ParamBase::Int | ito::ParamBase::In,
        0x0,
        meta,
        tr("Address of stage, 0x0 - 0xF. The address can be changed by setting the parameter later "
           "on.")
            .toLatin1()
            .data());
    paramVal.getMeta()->setCategory("Communication");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("numaxis",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        1,
        1,
        1,
        tr("Number of axes attached to this stage: Here always 1. Multiple axes, connected to one bus driver, must init multiple objects with the same serial object.").toLatin1().data());
    paramVal.getMeta()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "autoSaveSettings",
        ito::ParamBase::Int,
        0,
        1,
        1,
        tr("If 1, motor frequency settings (search, optimization, manual adjustment...) are automatically stored to the device. See also the 'saveUserData' exec function.")
            .toLatin1()
            .data());
    paramVal.getMeta()->setCategory("MotorSettings");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("travelRange",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        INT_MIN,
        INT_MAX,
        1,
        tr("Travel range of the axis").toLatin1().data());
    paramVal.getMeta()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "pulsesPerUnit",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        -1,
        INT_MAX,
        1,
        tr("Pulses per unit (mm or deg) / -1 for indexed position devices").toLatin1().data());
    paramVal.getMeta()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("calibDirection",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        0,
        tr("The direction for the calib / homeing operation. 0: clockwise, 1: counter-clockwise. Only relevant for rotary stages.").toLatin1().data());
    paramVal.getMeta()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "async",
        ito::ParamBase::Int,
        0,
        1,
        m_async,
        tr("Toggles if motor has to wait until end of movement (0:sync) or not (1:async)")
            .toLatin1()
            .data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "forwardFrequency1",
        ito::ParamBase::Int,
        4,
        std::numeric_limits<int>::max(),
        4,
        tr("The forward frequency for the first motor in Hz")
            .toLatin1()
            .data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("MotorSettings");
    paramVal.getMetaT<ito::IntMeta>()->setUnit("Hz");
    
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "backwardFrequency1",
        ito::ParamBase::Int,
        4,
        std::numeric_limits<int>::max(),
        4,
        tr("The backward frequency for the first motor in Hz").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("MotorSettings");
    paramVal.getMetaT<ito::IntMeta>()->setUnit("Hz");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "forwardFrequency2",
        ito::ParamBase::Int,
        4,
        std::numeric_limits<int>::max(),
        4,
        tr("The forward frequency for the second motor in Hz (if available)").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("MotorSettings");
    paramVal.getMetaT<ito::IntMeta>()->setUnit("Hz");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "backwardFrequency2",
        ito::ParamBase::Int,
        4,
        std::numeric_limits<int>::max(),
        4,
        tr("The backward frequency for the second motor in Hz (if available)").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("MotorSettings");
    paramVal.getMetaT<ito::IntMeta>()->setUnit("Hz");
    m_params.insert(paramVal.getName(), paramVal);

    QVector<ito::Param> pMand, pOpt, pOut;

    registerExecFunc(
        "saveUserData",
        pMand,
        pOpt,
        pOut,
        tr("Save motor parameters like forward or backward frequency.")
        .toLatin1()
        .data());

    registerExecFunc(
        "resetDefaults",
        pMand,
        pOpt,
        pOut,
        tr("Reset all frequencies to their default values.").toLatin1().data());

    registerExecFunc(
        "optimizeMotors",
        pMand,
        pOpt,
        pOut,
        tr("Fine tunes the frequency search for forward and backward direction. At first applies a search frequency run for coarse optimization, then starts the fine tuning. This operation might take several minutes (e.g. 30min). It can be interrupted by the Keyboard Interrupt.")
        .toLatin1()
        .data());

    registerExecFunc(
        "cleanMechanics",
        pMand,
        pOpt,
        pOut,
        tr("Cleans the mechanics by applying forward and backward runs of the full travel range. This operation might take several minutes (e.g. 30min). It can be interrupted by the Keyboard Interrupt.")
        .toLatin1()
        .data());

    pMand.append(ito::Param("motorIndex", ito::ParamBase::Int, 0, 1, 0, "Motor Index (0 or 1)"));
    registerExecFunc(
        "searchFrequencies",
        pMand,
        pOpt,
        pOut,
        tr("Requests a frequency search to optimize the operating frequencies for backward and "
           "forward movement of the indicated motor.")
            .toLatin1()
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

    m_params["address"].setVal<int>(m_address);

    m_currentPos << 0.0;
    m_currentStatus << (ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable);
    m_targetPos << 0.0;
    
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
            tr("Input parameter is not a dataIO instance of the SerialIO Plugin!")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        retValue += identifyDevices();
    }

    if (!retValue.containsError())
    {
        QSharedPointer<QVector<int>> status(new QVector<int>(1, 0));

        // ignore any status errors, to finish the initialization, even if the stage is currently at
        // the end of its travel range (among others)
        getStatus(status, nullptr); 
        QSharedPointer<double> pos(new double);
        retValue += getPos(0, pos, nullptr);

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
            "in;gs;ho;ma;mr;gp;us;ca;i1;f1;b1",
            1,
            false,
            false
        );

        elliptecModels << ElliptecDevice(
            9,
            "ELL9",
            "Four-Position Slider",
            true,
            true,
            "mm",
            4,
            "in;gs;ho;ma;mr;gp;us;ca;i1;i2;f1;b1;f2;b2",
            2,
            false,
            false
        );

        elliptecModels << ElliptecDevice(
            12,
            "ELL12",
            "Six-Position Slider",
            true,
            true,
            "mm",
            6,
            "in;gs;ho;ma;mr;gp;us;ca;i1;i2;f1;b1;f2;b2",
            2,
            false,
            false
        );

        elliptecModels << ElliptecDevice(
            14,
            "ELL14",
            "Rotation Mount",
            false,
            false,
            "°",
            0,
            "in;gs;ho;ma;mr;gp;us;ca;cm;st;om;i1;i2;f1;b1;f2;b2;s1;s2",
            2,
            true,
            true
        );

        elliptecModels << ElliptecDevice(
            15,
            "ELL15",
            "Motorized Iris",
            false,
            true,
            "mm",
            0,
            "in;gs;ho;ma;mr;gp;us;ca;i1;i2;f1;b1;f2;b2",
            2,
            true,
            false
        );

        elliptecModels << ElliptecDevice(
            17,
            "ELL17",
            "Linear Stage",
            false,
            true,
            "mm",
            0,
            "in;gs;ho;ma;mr;gp;us;ca;cm;st;om;i1;i2;f1;b1;f2;b2;s1;s2",
            2,
            true,
            true
        );

        elliptecModels << ElliptecDevice(
            18,
            "ELL18",
            "Rotation Stage",
            false,
            false,
            "°",
            0,
            "in;gs;ho;ma;mr;gp;us;ca;cm;st;om;i1;i2;f1;b1;f2;b2;s1;s2",
            2,
            true,
            true
        );

        elliptecModels << ElliptecDevice(
            20,
            "ELL20",
            "Linear Stage",
            false,
            true,
            "mm",
            0,
            "in;gs;ho;ma;mr;gp;us;ca;cm;st;om;i1;i2;f1;b1;f2;b2;s1;s2",
            2,
            true,
            true
        );
    }
}

//------------------------------------------------------------------------------
/*static*/ void ThorlabsElliptec::initSupportedCmds()
{
    if (supportedCmds.size() > 0)
    {
        // already initialized
        return;
    }

    supportedCmds["in"] = CmdInfo("in", "IN", 0, 30, false); // identify
    supportedCmds["gs"] = CmdInfo("gs", "GS", 0, 2, true); // status
    supportedCmds["ho"] = CmdInfo("ho", "PO", 1, 8, true); // homing
    supportedCmds["ma"] = CmdInfo("ma", "PO", 8, 8, true); // move absolute
    supportedCmds["mr"] = CmdInfo("mr", "PO", 8, 8, true); // move relative
    supportedCmds["gp"] = CmdInfo("gp", "PO", 0, 8, false); // get position
    supportedCmds["us"] = CmdInfo("us", "GS", 0, 2, true); // save user data
    supportedCmds["ca"] = CmdInfo("ca", "GS", 1, 0, true); // change address
    supportedCmds["cm"] = CmdInfo("cm", "GS", 0, 2, true); // clean mechanics
    supportedCmds["st"] = CmdInfo("st", "GS", 0, 2, true); // motion stop
    supportedCmds["om"] = CmdInfo("om", "GS", 0, 2, true); // optimize motors
    supportedCmds["i1"] = CmdInfo("i1", "I1", 0, 22, false); // get motor1info
    supportedCmds["i2"] = CmdInfo("i2", "I2", 0, 22, false); // get motor2info
    supportedCmds["f1"] = CmdInfo("f1", "GS", 4, 2, true); // set forward frequency 1
    supportedCmds["f2"] = CmdInfo("f2", "GS", 4, 2, true); // set forward frequency 2
    supportedCmds["b1"] = CmdInfo("b1", "GS", 4, 2, true); // set backward frequency 1
    supportedCmds["b2"] = CmdInfo("b2", "GS", 4, 2, true); // set backward frequency 2
    supportedCmds["s1"] = CmdInfo("s1", "GS", 0, 2, true); // search optimal frequencies 1
    supportedCmds["s2"] = CmdInfo("s2", "GS", 0, 2, true); // search optimal frequencies 2
}

//------------------------------------------------------------------------------
bool ThorlabsElliptec::getCmdInfo(const QByteArray& cmd, CmdInfo& info) const
{
    if (m_model.m_supportedCmds.size() > 0)
    {
        if (m_model.m_supportedCmds.contains(cmd))
        {
            info = supportedCmds[cmd];
            return true;
        }
        else
        {
            return false;
        }
    }

    // model not yet identified -> only the 'in' command is available
    if (cmd == "in" && supportedCmds.contains(cmd))
    {
        info = supportedCmds[cmd];
        return true;
    }

    return false;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::identifyDevices()
{
    ito::RetVal retValue;
    QByteArray response;
    bool ok = true;
    bool modelFound = false;

    unsigned char adr = (unsigned char)m_address;
    retValue += sendCommandAndGetResponse(adr, "in", "", m_requestTimeOutMS, response);

    if (!retValue.containsError())
    {
        int motorType = byteArrayToInt(response.mid(3 - 3, 2));
        QByteArray serial = response.mid(5 - 3, 8);
        QByteArray year = response.mid(13 - 3, 4);
        float fwRelease = response.mid(17 - 3, 2).toFloat(nullptr) / 10.0;
        int hwRelease = byteArrayToInt(response.mid(19 - 3, 2));
        int travelRange = byteArrayToInt(response.mid(21 - 3, 4));

        // pulses are number of total pulses for the entire travel range
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
            if (m_model.m_indexed)
            {
                m_params["pulsesPerUnit"].setVal<int>(-1);
            }
            else
            {
                m_params["pulsesPerUnit"].setVal<int>(pulses / travelRange);
            }

            setIdentifier(
                QString("Elliptec %1 (%2), Serial %3").arg(m_model.m_description).arg(m_model.m_name).arg(serial)
            );

            m_params["serial"].setVal<const char*>(serial.data());
            m_params["model"].setVal<const char*>(m_model.m_name.toLatin1().data());
            m_params["description"].setVal<const char*>(m_model.m_description.toLatin1().data());
            m_params["numMotors"].setVal<int>(m_model.m_numMotors);

            if (m_model.m_indexed)
            {
                m_params["axisType"].setVal<int>(-1);
                m_params["travelRange"].getMetaT<ito::IntMeta>()->setUnit("");
            }
            else if (m_model.m_linear)
            {
                m_params["axisType"].setVal<int>(1);
                m_params["travelRange"].getMetaT<ito::IntMeta>()->setUnit("mm");
            }
            else
            {
                m_params["axisType"].setVal<int>(0);
                m_params["travelRange"].getMetaT<ito::IntMeta>()->setUnit("°");
            }
        }

        retValue += updateMotorFrequencies();
    }
    else if (retValue.errorCode() == TIMEOUT_ID)
    {
        retValue = ito::RetVal::format(
            ito::retError,
            TIMEOUT_ID,
            "No answer from identification request at address %i. Maybe try another address in the range 0x0 - 0xF.",
            adr);
    }
    else
    {
        retValue = ito::RetVal::format(ito::retError,
            0,
            "Identification of device at address %i failed. Response format mismatch.",
            adr);
    }

    return retValue;
}

//------------------------------------------------------------------------------
int ThorlabsElliptec::getFrequencyFromWord(const QByteArray& ba)
{
    int value = byteArrayToInt(ba); // period
    return qRound(14740000.0 / (double)value);
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::updateMotorFrequencies()
{
    ito::RetVal retValue;
    QByteArray response;

    retValue += sendCommandAndGetResponse(m_address, "i1", "", m_requestTimeOutMS, response);

    if (!retValue.containsError())
    {
        m_params["forwardFrequency1"].setVal<int>(getFrequencyFromWord(response.mid(14, 4)));
        m_params["backwardFrequency1"].setVal<int>(getFrequencyFromWord(response.mid(18, 4)));
    }

    if (m_model.m_numMotors < 2)
    {
        m_params["backwardFrequency2"].setFlags(
            m_params["backwardFrequency2"].getFlags() | ito::ParamBase::Readonly);
        m_params["forwardFrequency2"].setFlags(
            m_params["forwardFrequency2"].getFlags() | ito::ParamBase::Readonly);
    }
    else
    {
        retValue += sendCommandAndGetResponse(m_address, "i2", "", m_requestTimeOutMS, response);

        if (!retValue.containsError())
        {
            m_params["forwardFrequency2"].setVal<int>(getFrequencyFromWord(response.mid(14, 4)));
            ;
            m_params["backwardFrequency2"].setVal<int>(
                getFrequencyFromWord(response.mid(18, 4)));
            ;
        }
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
        if (key == "forwardFrequency1" || key == "forwardFrequency2" ||
            key == "backwardFrequency1" || key == "backwardFrequency2")
        {
            retValue += updateMotorFrequencies();
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
    bool valueAlreadyUpdated = false;

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
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "async")
        {
            // check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom(&(*val));
            m_async = m_params["async"].getVal<int>();
        }
        else if (key == "address")
        {
            // change the address of the device
            QByteArray response;
            retValue += sendCommandAndGetResponse(
                m_address, "ca", val->getVal<int>(), m_requestTimeOutMS, response);

            if (!retValue.containsError())
            {
                m_address = val->getVal<int>();
                retValue += saveUserData();
            }
        }
        else if (key == "forwardFrequency1")
        {
            int period = /*0x8000 | */0x0FFF & qRound(14740000.0 / (double)val->getVal<int>());
            QByteArray response;
            retValue += sendCommandAndGetResponse(m_address, "f1", period, m_requestTimeOutMS, response);
            retValue += updateMotorFrequencies();
            valueAlreadyUpdated = true;

            if (retValue == ito::retOk && m_params["autoSaveSettings"].getVal<int>())
            {
                retValue += saveUserData();
            }
        }
        else if (key == "forwardFrequency2")
        {
            int period = /*0x8000 | */0x0FFF & qRound(14740000.0 / (double)val->getVal<int>());
            QByteArray response;
            retValue +=
                sendCommandAndGetResponse(m_address, "f2", period, m_requestTimeOutMS, response);
            retValue += updateMotorFrequencies();
            valueAlreadyUpdated = true;

            if (retValue == ito::retOk && m_params["autoSaveSettings"].getVal<int>())
            {
                retValue += saveUserData();
            }
        }
        else if (key == "backwardFrequency1")
        {
            int period = /*0x8000 | */0x0FFF & qRound(14740000.0 / (double)val->getVal<int>());
            QByteArray response;
            retValue +=
                sendCommandAndGetResponse(m_address, "b1", period, m_requestTimeOutMS, response);
            retValue += updateMotorFrequencies();
            valueAlreadyUpdated = true;

            if (retValue == ito::retOk && m_params["autoSaveSettings"].getVal<int>())
            {
                retValue += saveUserData();
            }
        }
        else if (key == "backwardFrequency2")
        {
            int period = /*0x8000 | */0x0FFF & qRound(14740000.0 / (double)val->getVal<int>());
            QByteArray response;
            retValue +=
                sendCommandAndGetResponse(m_address, "b2", period, m_requestTimeOutMS, response);
            retValue += updateMotorFrequencies();
            valueAlreadyUpdated = true;

            if (retValue == ito::retOk && m_params["autoSaveSettings"].getVal<int>())
            {
                retValue += saveUserData();
            }
        }

        if (!valueAlreadyUpdated  && !retValue.containsError())
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

    if (axis.size() != 1 || axis[0] != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "Only implemented for one single axis with index 0");
    }
    else
    {
        setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate(true);

        int dir = m_params["calibDirection"].getVal<int>();
        QByteArray response;
        retValue += sendCommandAndGetResponse(m_address, "ho", 1, m_waitForDoneTimeoutMS, response);

        if (retValue.containsError() && retValue.errorCode() == 2)
        {
            // in case of a mechanical timeout retry one time
            retValue =
                sendCommandAndGetResponse(m_address, "ho", 1, m_waitForDoneTimeoutMS, response);
        }

        if (!retValue.containsError())
        {
            double value = positionFromPosResponse(response);

            m_currentPos[0] = value;
            m_targetPos[0] = value;
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
            sendTargetUpdate();
        }
        else
        {
            setStatus(axis, ito::actuatorError, ito::actSwitchesMask | ito::actStatusMask);
        }

        sendStatusUpdate();
    }

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
    ito::RetVal retValue(ito::retError, 0, "set origin not supported.");

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::getStatus(
    QSharedPointer<QVector<int>> status, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QByteArray response;
    retValue += sendCommandAndGetResponse(m_address, "gs", "", m_requestTimeOutMS, response);

    if (retValue.containsError())
    {
        setStatus(m_currentStatus[0],
            ito::actuatorError | ito::actuatorAvailable | ito::actuatorEnabled,
            ito::actSwitchesMask |ito::actMovingMask);
    }
    else
    {
        setStatus(m_currentStatus[0],
            ito::actuatorAvailable | ito::actuatorEnabled,
            ito::actSwitchesMask | ito::actMovingMask);
    }

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
        retValue += sendCommandAndGetResponse(m_address, "gp", "", m_requestTimeOutMS, response);

        if (!retValue.containsError())
        {
            double value = positionFromPosResponse(response);
            m_currentPos[0] = value;
            (*pos)[0] = value;
            setStatus(axis, ito::actuatorAvailable | ito::actuatorEnabled); // todo: at target? moving?
        }
        else
        {
            setStatus(axis, ito::actuatorAvailable | ito::actuatorEnabled | ito::actuatorError); // todo: at target? moving?
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
            tr("motor is running. Additional actions are not possible.").toLatin1().data());

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else
    {
        m_targetPos[0] = pos[0];

        setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate();
        sendTargetUpdate();

        if (m_async && waitCond) // async disabled
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        QByteArray data = positionTo8ByteArray(pos[0]);
        QByteArray response;
        retValue += sendCommandAndGetResponse(m_address, "ma", data, m_waitForDoneTimeoutMS, response);

        if (retValue.containsError() && retValue.errorCode() == 2)
        {
            // in case of a mechanical timeout retry one time
            retValue =
                sendCommandAndGetResponse(m_address, "ma", data, m_waitForDoneTimeoutMS, response);
        }

        if (retValue.containsError())
        {
            if (retValue.errorCode() == TIMEOUT_ID)
            {
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            }
            else
            {
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorError);
            }

            QSharedPointer<double> pos(new double);
            retValue += getPos(0, pos, nullptr);
        }
        else
        {
            m_currentPos[0] = positionFromPosResponse(response);
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
        }


        sendStatusUpdate();

        if (!m_async && waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
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

    if (axis.size() != 1 || pos.size() != 1 || axis[0] != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "Only one axis supported (index 0)");
    }
    else if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("motor is running. Additional actions are not possible.").toLatin1().data());

        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else
    {
        m_targetPos[0] = pos[0];

        setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate();
        sendTargetUpdate();

        if (m_async && waitCond) // async disabled
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }

        QByteArray data = positionTo8ByteArray(pos[0]);
        QByteArray response;
        retValue += sendCommandAndGetResponse(m_address, "mr", data, m_waitForDoneTimeoutMS, response);

        if (retValue.containsError())
        {
            if (retValue.errorCode() == TIMEOUT_ID)
            {
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorTimeout);
            }
            else
            {
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorError);
            }

            QSharedPointer<double> pos(new double);
            retValue += getPos(0, pos, nullptr);
        }
        else
        {
            m_currentPos[0] = positionFromPosResponse(response);
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
        }


        sendStatusUpdate();

        if (!m_async && waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::saveUserData()
{

    QByteArray response;
    return sendCommandAndGetResponse(m_address, "us", "", m_requestTimeOutMS, response);
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

    if (funcName == "saveUserData")
    {
        retValue += saveUserData();
    }
    else if (funcName == "searchFrequencies")
    {
        int motorIdx = paramsMand->at(0).getVal<int>();
        QByteArray response;

        switch (motorIdx)
        {
        case 0:
            retValue +=
                sendCommandAndGetResponse(m_address, "s1", "", m_requestTimeOutMS * 10, response);
            break;
        case 1:
            retValue +=
                sendCommandAndGetResponse(m_address, "s2", "", m_requestTimeOutMS * 10, response);
            break;
        default:
            retValue += ito::RetVal(ito::retError, 0, "unsupported motor index");
            break;
        }

        retValue += updateMotorFrequencies();

        if (retValue == ito::retOk && m_params["autoSaveSettings"].getVal<int>())
        {
            retValue += saveUserData();
        }

        emit parametersChanged(m_params);
    }
    else if (funcName == "resetDefaults")
    {
        int numMotors = m_params["numMotors"].getVal<int>();

        QByteArray response;
        int period = (ito::int16)(14740 / 100) | 0x8000;

        if (numMotors >= 1)
        {
            retValue +=
                sendCommandAndGetResponse(m_address, "f1", period, m_requestTimeOutMS, response);
            retValue +=
                sendCommandAndGetResponse(m_address, "b1", period, m_requestTimeOutMS, response);
        }

        if (numMotors >= 2)
        {
            retValue +=
                sendCommandAndGetResponse(m_address, "f2", period, m_requestTimeOutMS, response);
            retValue +=
                sendCommandAndGetResponse(m_address, "b2", period, m_requestTimeOutMS, response);
        }
        
        retValue += updateMotorFrequencies();
        retValue += saveUserData();

        emit parametersChanged(m_params);
    }
    else if (funcName == "optimizeMotors" || funcName == "cleanMechanics")
    {
        resetInterrupt();

        QByteArray response;

        if (funcName == "optimizeMotors")
        {
            if (!m_model.m_allowOptimization)
            {
                retValue += ito::RetVal(
                    ito::retError, 0, "This device does not support the clean & optimize feature.");
            }
            else
            {
                retValue += sendCommandAndGetResponse(
                    m_address, "om", "", 10 * m_waitForDoneTimeoutMS, response);
            }
            
        }
        else
        {
            if (!m_model.m_allowCleaning)
            {
                retValue += ito::RetVal(
                    ito::retError, 0, "This device does not support the cleaning feature.");
            }
            else
            {
                retValue += sendCommandAndGetResponse(
                    m_address, "cm", "", 10 * m_waitForDoneTimeoutMS, response);
            }
        }

        if (retValue != ito::retError)
        {
            if (retValue == ito::retWarning && retValue.errorCode() == 9)
            {
                // approach:
                // 1. request the status all 10sek (like the Thorlabs Elliptec software)
                // 2. put the alive signal every second
                // 3. maximum timeout of 10min
                // 4. check for interrupts every second

                int timeoutMs = 1000 * 50 * 60; // 50min
                QElapsedTimer elapsed;
                bool timeout = false;
                int getStatusCounter = 10;
                elapsed.start();

                while (!elapsed.hasExpired(timeoutMs))
                {
                    if (isInterrupted())
                    {
                        // it might take some seconds until st is finished
                        retValue = sendCommandAndGetResponse(
                            m_address, "st", "", m_requestTimeOutMS * 10, response);
                        break;
                    }

                    if (--getStatusCounter <= 0)
                    {
                        setAlive();
                        retValue = sendCommandAndGetResponse(
                            m_address, "gs", "", m_requestTimeOutMS * 20, response);

                        if (retValue != ito::retWarning && retValue.errorCode() != 9)
                        {
                            // there is another response than a busy response...
                            // e.g. the run is finished!
                            break;
                        }

                        getStatusCounter = 10;
                    }
                    else
                    {
                        QThread::sleep(1); // wait for one sec
                        setAlive();
                    }
                }

                if (elapsed.hasExpired(timeoutMs))
                {
                    retValue += ito::RetVal(
                        ito::retError, 0, "Timeout while executing operation. Stop it.");

                    // stop
                    // it might take some seconds until st is finished
                    retValue += sendCommandAndGetResponse(
                        m_address, "st", "", m_requestTimeOutMS * 10, response);
                }
            }

            retValue += updateMotorFrequencies();
            emit parametersChanged(m_params);
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
    // unused method.

    return ito::retOk;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::sendCommand(unsigned char address, const QByteArray& cmdId, const QByteArray& data /*= QByteArray()*/)
{
    // data must be a big-endian string representation of a number, if it is a number!
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    m_pSerialIO->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
    m_pSerialIO->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);

    QByteArray cmd = QByteArray::number(address, 16).toUpper() + cmdId + data;
    ito::RetVal retVal = m_pSerialIO->setVal(cmd.constData(), cmd.length(), nullptr);
    setAlive();
    return retVal;
}

//------------------------------------------------------------------------------
QByteArray ThorlabsElliptec::intToByteArray(int value, int numBytes) const
{
    QByteArray byteArray;

    // Creates an QDataStream, to write the number into the QByteArray
    QDataStream stream(&byteArray, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::BigEndian); // set the byte order to big-endian
    stream << value; // writes the number

    byteArray = byteArray.toHex();
    byteArray = byteArray.right(numBytes).toUpper();
    return byteArray;
}

//------------------------------------------------------------------------------
int ThorlabsElliptec::byteArrayToInt(const QByteArray& value) const
{
    return value.toInt(nullptr, 16);
}

//------------------------------------------------------------------------------
double ThorlabsElliptec::positionFromPosResponse(const QByteArray& response) const
{
    unsigned int value_unsigned;
    std::stringstream ss;
    ss << std::hex << response.constData();
    ss >> value_unsigned;
    // output it as a signed type
    auto value_signed = static_cast<int>(value_unsigned);
    double value = value_signed;

    if (m_model.m_indexed)
    {
        // convert value to indexed values 0, 1, 2, 3 ... (up to number of indexed positions)
        double travelRange = m_params["travelRange"].getVal<int>();
        double factor = travelRange / (m_model.m_numIndexedPositions - 1);
        return qRound(value / factor) + 1.;
    }
    else
    {
        double pulsesPerUnit = m_params["pulsesPerUnit"].getVal<int>();
        value /= pulsesPerUnit;
        return value;
    }
}

//------------------------------------------------------------------------------
QByteArray ThorlabsElliptec::positionTo8ByteArray(double position) const
{
    int value;

    if (m_model.m_indexed)
    {
        // convert indexed values 0, 1, 2, 3 ... (up to number of indexed positions) to value
        double travelRange = m_params["travelRange"].getVal<int>();
        double factor = travelRange / (m_model.m_numIndexedPositions - 1);
        value = qRound((position - 1.0) * factor);
    }
    else
    {
        double pulsesPerUnit = m_params["pulsesPerUnit"].getVal<int>();
        value = position * pulsesPerUnit;
    }

    return intToByteArray(value, 8);
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::sendCommandAndGetResponse(
    unsigned char address,
    const QByteArray& cmdId,
    const QByteArray& data,
    int timeoutMs,
    QByteArray& response)
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
        retVal += ito::RetVal::format(
            ito::retError,
            0,
            "invalid or unsupported command '%s' for this device",
            cmdId.constData());
    }

    if (!retVal.containsError())
    {
        retVal += readResponse(timeoutMs, response);
    }

    if (!retVal.containsError())
    {
        int response_address = response.left(1).toInt(nullptr, 16);

        if (response.mid(1, 2) == "GS" && info.canReturnStatus)
        {
            retVal += parseStatusResponse(response);
        }
        else if (response.size() != (3 + info.rcvDataNumBytes))
        {
            retVal += ito::RetVal(ito::retError, 0, "length of device response is wrong.");
        }
        else if (response_address != address)
        {
            retVal += ito::RetVal(ito::retError, 0, "device response is assigned to wrong address.");
        }
        else if (response.mid(1, 2) == info.rcvCmd)
        {
            // correct answer
            response = response.mid(3);
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
ito::RetVal ThorlabsElliptec::sendCommandAndGetResponse(
    unsigned char address, const QByteArray& cmdId, int data, int timeoutMs, QByteArray& response)
{
    CmdInfo info;

    if (getCmdInfo(cmdId, info))
    {
        QByteArray dataStr = intToByteArray(data, info.sendDataNumBytes);
        return sendCommandAndGetResponse(address, cmdId, dataStr, timeoutMs, response);
    }
    else
    {
        return ito::RetVal::format(ito::retError, 0, "invalid or unsupported command '%s' for this device", cmdId.constData());
    }
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::readResponse(int timeoutMs, QByteArray& response)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    *m_serialBufferLength = m_serialBufferSize;
    std::memset(m_serialBuffer.data(), '\0', m_serialBufferSize);

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

        *m_serialBufferLength = m_serialBufferSize;
        retValue += m_pSerialIO->getVal(m_serialBuffer, m_serialBufferLength, nullptr);

        if (*m_serialBufferLength > 0)
        {
            response += QByteArray(m_serialBuffer.data(), *m_serialBufferLength);
        }

        if (retValue.containsError())
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("Error occurred during reading the response from the serial port.")
                .toLatin1()
                .data());
            break;
        }
        else
        {
            // FOUND
            if (response.endsWith("\r\n"))
            {
                // remove endline characters
                response = response.left(response.size() - 2);
                done = true;
            }
        }

        if (!done && timer.elapsed() > timeoutMs && timeoutMs >= 0)
        {
            retValue += ito::RetVal(
                ito::retError, TIMEOUT_ID, tr("timeout while waiting for response from device.").toLatin1().data());
            return retValue;
        }
    }

    return retValue;
}

//------------------------------------------------------------------------------
ito::RetVal ThorlabsElliptec::parseStatusResponse(const QByteArray& response) const
{
    if (response.size() != 5)
    {
        // response e.g. 0GS00
        return ito::RetVal(ito::retError, 0, "invalid response size");
    }
    else if (response.mid(1, 2) != "GS")
    {
        return ito::RetVal(ito::retError, 0, "invalid response command");
    }
    else
    {
        int status = response.mid(3, 2).toInt(nullptr, 16);

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
            return ito::RetVal(ito::retWarning, status, "Busy");
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
    return apiShowConfigurationDialog(this, new DialogThorlabsElliptec(this, m_model.m_allowCleaning, m_model.m_allowOptimization));
}
