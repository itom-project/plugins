/* ********************************************************************
    Plugin "ThorlabsDMH" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, TRUMPF Laser- und Systemtechnik GmbH

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

#include "ThorlabsDMH.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qdatetime.h>
#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

#include "common/helperCommon.h"
#include "common/paramMeta.h"

#include "dockWidgetThorlabsDMH.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "TLDFMX.h"

QList<QString> ThorlabsDMH::openedDevices = QList<QString>();

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsDMHInterface::ThorlabsDMHInterface()
{
    m_type = ito::typeActuator;
    setObjectName("ThorlabsDMH");

    m_description = QObject::tr("ThorlabsDMH");
    char docstring[] =
        "The Thorlabs DMH deformable mirror has 40 segments, which can be used as axes in this plugin. \
A voltage of 0V to 300V can be set for each segment, with 150V representing a flat mirror. \
The Zernike coefficients 4 to 15 can also be set using an additional 'exec' function. \n\
The segment IDs of Thorlabs (1 to 40) correspond to the axes (0 to 39). \n\
The functionality for tip tilt actuator is not implemented.";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsOpt.append(ito::Param(
        "serialNo",
        ito::ParamBase::String,
        "",
        tr("Serial number of the device to be loaded. If empty, the first device that can be "
           "opened will be opened.")
            .toLatin1()
            .data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsDMHInterface::~ThorlabsDMHInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMHInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(ThorlabsDMH) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMHInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(ThorlabsDMH) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsDMH::ThorlabsDMH() : AddInActuator(), m_async(0)
{
    ito::IntMeta* imeta;
    ito::DoubleMeta* dmeta;

    // register exec functions
    int zernikeID[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    ito::float64 zernike[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    QVector<ito::Param> pMand = QVector<ito::Param>()
        << ito::Param("ZernikeIDs",
                      ito::ParamBase::IntArray,
                      12,
                      zernikeID,
                      new ito::IntArrayMeta(4, 15, 1, 0, 12),
                      tr("List of Zernike IDs.").toLatin1().data());
    pMand << ito::Param(
        "ZernikeValues",
        ito::ParamBase::DoubleArray,
        12,
        zernike,
        new ito::DoubleArrayMeta(-1.0, 1.0, 0, 0, 12),
        tr("List of Zernike values.").toLatin1().data());
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc(
        "setZernikes",
        pMand,
        pOpt,
        pOut,
        tr("Sets a list of Zernike coefficients on the entire mirror surface."));
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    registerExecFunc(
        "relaxMirror", pMand, pOpt, pOut, tr("Relax the mirror (hysteresis compensation)."));
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    // end register exec functions

    ito::Param paramVal(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        tr("ThorlabsDMH").toUtf8().data(),
        tr("Name of the plugin.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "async",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        1,
        m_async,
        tr("Toggles if motor has to wait until end of movement (0: sync) or not (1: async). Only "
           "sync supported.")
            .toLatin1()
            .data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("General");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "numaxis",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        40,
        40,
        40,
        tr("Number of axes attached to this stage").toLatin1().data());
    imeta = paramVal.getMetaT<ito::IntMeta>();
    imeta->setCategory("General");
    imeta->setRepresentation(ito::ParamMeta::PureNumber); // numaxis should be a spin box and no
                                                          // slider in any generic GUI
    m_params.insert(paramVal.getName(), paramVal);
    m_nrOfAxes = paramVal.getVal<int>();

    paramVal = ito::Param(
        "manufacturerName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Manufacturer name.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "instrumentName",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Instrument name.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Serial number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "extensionDriver",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Extension driver.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "firmware",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Firmware.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "", "Device info"), true);
    m_params.insert(paramVal.getName(), paramVal);

    // ----------------- Segments -----------------
    paramVal = ito::Param(
        "numSegments",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        new ito::IntMeta(0, 40, 1, "Device parameter"),
        tr("Number of segments.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "minVoltageMirror",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(0.0, 300.0, 0.0, "Device parameter"),
        tr("Min voltage of mirror segments.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "maxVoltageMirror",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(0.0, 300.0, 0.0, "Device parameter"),
        tr("Max voltage of mirror segments.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "commonVoltageMirror",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(0.0, 300.0, 0.0, "Device parameter"),
        tr("Common voltage of mirror segments.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "numTipTilt",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        new ito::IntMeta(0, 15, 1, "Device parameter"),
        tr("Number of tip/tilt elements.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "minVoltageTipTilt",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(0.0, 300.0, 0.0, "Device parameter"),
        tr("Min tip/tilt voltage.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "maxVoltageTipTilt",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(0.0, 300.0, 0.0, "Device parameter"),
        tr("Max tip/tilt voltage.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "commonVoltageTipTilt",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(0.0, 300.0, 0.0, "Device parameter"),
        tr("Common tip/tilt voltage.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // ----------------- Zernike -----------------
    paramVal = ito::Param(
        "minZernikeAmplitude",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(-1.0, 1.0, 0.0, "Device parameter"),
        tr("Min Zernike amplitude.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "maxZernikeAmplitude",
        ito::ParamBase::Double | ito::ParamBase::Readonly,
        0.0,
        new ito::DoubleMeta(-1.0, 1.0, 0.0, "Device parameter"),
        tr("Max Zernike amplitude.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "zernikeCount",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        new ito::IntMeta(0, 15, 1, "Device parameter"),
        tr("Zernike count.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "systemMeasurementSteps",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        new ito::IntMeta(0, 100, 1, "Device parameter"),
        tr("System measurement steps.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "relaxSteps",
        ito::ParamBase::Int | ito::ParamBase::Readonly,
        0,
        new ito::IntMeta(0, 100, 1, "Device parameter"),
        tr("Relax steps.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // initialize the current position vector, the status vector and the target position vector
    m_currentPos.fill(0.0, m_nrOfAxes);
    m_currentStatus.fill(0, m_nrOfAxes);
    m_targetPos.fill(0.0, m_nrOfAxes);

    // the following lines create and register the plugin's dock widget. Delete these lines if the
    // plugin does not have a dock widget.
    DockWidgetThorlabsDMH* dw = new DockWidgetThorlabsDMH(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
        QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
ThorlabsDMH::~ThorlabsDMH()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMH::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ViStatus err = VI_SUCCESS;


    QString serial = paramsOpt->at(0).getVal<const char*>();

    ViUInt32 deviceCount = 0;
    int choice = 0;

    ViChar manufacturer[TLDFM_BUFFER_SIZE];
    ViChar instrumentName[TLDFM_MAX_INSTR_NAME_LENGTH];
    ViChar serialNumber[TLDFM_MAX_SN_LENGTH];
    ViBoolean deviceAvailable;

    if (openedDevices.contains(serial))
    {
        retValue += ito::RetVal(
            ito::retError,
            1,
            QObject::tr("Device at SerialNo %1 is already connected.")
                .arg(serial)
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        err = TLDFM_get_device_count(VI_NULL, &deviceCount);
        if ((TL_ERROR_RSRC_NFOUND == err) || (0 == deviceCount))
        {
            retValue += ito::RetVal(
                ito::retError, 1, QObject::tr("No THORLABS instruments found.").toLatin1().data());
        }
    }

    if (!retValue.containsError())
    {
        bool gotDevice = false;
        for (ViUInt32 i = 0; i < deviceCount; i++)
        {
            err += TLDFM_get_device_information(
                VI_NULL,
                i,
                manufacturer,
                instrumentName,
                serialNumber,
                &deviceAvailable,
                m_resourceName);

            if (deviceAvailable && serial == "")
            {
                gotDevice = true;
                break;
            }
            else if (deviceAvailable && serial == serialNumber)
            {
                gotDevice = true;
                break;
            }
        }
        if (!gotDevice)
        {
            retValue += ito::RetVal(
                ito::retError, 1, QObject::tr("No available device found.").toLatin1().data());
        }
        if (err)
        {
            // get error msg
            ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
            TLDFMX_error_message(m_insrumentHdl, err, buf);

            retValue += ito::RetVal::format(
                ito::retError,
                1,
                tr("Error during initialisation: THORLABS error: '%1'.")
                    .arg(buf)
                    .toLatin1()
                    .data());
        }
    }


    if (!retValue.containsError())
    {
        if (!m_resourceName)
        {
            retValue += ito::RetVal(ito::retError, 1, tr("No device found.").toLatin1().data());
        }

        err = TLDFMX_init(m_resourceName, VI_TRUE, VI_TRUE, &m_insrumentHdl);
        if (err)
        {
            // get error msg
            ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
            TLDFMX_error_message(m_insrumentHdl, err, buf);

            retValue += ito::RetVal::format(
                ito::retError,
                1,
                tr("Error during initialisation: THORLABS error: '%s'.")
                    .arg(buf)
                    .toLatin1()
                    .data());
        }

        // get device info
        retValue = getDeviceInfo();
    }

    // get device configuration
    if (!retValue.containsError())
    {
        ViUInt32 Segments, TiltElements;
        ViReal64 VoltageMirrorMin, VoltageMirrorMax, VoltageMirrorCommon;
        ViReal64 VoltageTiltMin, VoltageTiltMax, VoltageTiltCommon;

        err = TLDFM_get_device_configuration(
            m_insrumentHdl,
            &Segments,
            &VoltageMirrorMin,
            &VoltageMirrorMax,
            &VoltageMirrorCommon,
            &TiltElements,
            &VoltageTiltMin,
            &VoltageTiltMax,
            &VoltageTiltCommon);

        if (err)
        {
            // get error msg
            ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
            TLDFMX_error_message(m_insrumentHdl, err, buf);

            retValue += ito::RetVal::format(
                ito::retError,
                1,
                tr("Error during getting parameter: THORLABS error: '%s'.")
                    .arg(buf)
                    .toLatin1()
                    .data());
        }
        else
        {
            m_params["numSegments"].setVal<int>(Segments);
            m_params["minVoltageMirror"].setVal<double>(VoltageMirrorMin);
            m_params["maxVoltageMirror"].setVal<double>(VoltageMirrorMax);
            m_params["commonVoltageMirror"].setVal<double>(VoltageMirrorCommon);
            m_params["numTipTilt"].setVal<int>(TiltElements);
            m_params["minVoltageTipTilt"].setVal<double>(VoltageTiltMin);
            m_params["maxVoltageTipTilt"].setVal<double>(VoltageTiltMax);
            m_params["commonVoltageTipTilt"].setVal<double>(VoltageTiltCommon);
        }
    }

    // get extension driver parameters
    if (!retValue.containsError())
    {
        ViInt32 zernikeCount, systemMeasurementSteps, relaxSteps;
        ViReal64 minZernikeAmplitude;
        ViReal64 maxZernikeAmplitude;

        err = TLDFMX_get_parameters(
            m_insrumentHdl,
            &minZernikeAmplitude,
            &maxZernikeAmplitude,
            &zernikeCount,
            &systemMeasurementSteps,
            &relaxSteps);

        if (err)
        {
            // get error msg
            ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
            TLDFMX_error_message(m_insrumentHdl, err, buf);

            retValue += ito::RetVal::format(
                ito::retError,
                1,
                tr("Error during getting parameter: THORLABS error: '%s'.")
                    .arg(buf)
                    .toLatin1()
                    .data());
        }
        else
        {
            m_params["minZernikeAmplitude"].setVal<double>(minZernikeAmplitude);
            m_params["maxZernikeAmplitude"].setVal<double>(maxZernikeAmplitude);
            m_params["zernikeCount"].setVal<int>(zernikeCount);
            m_params["systemMeasurementSteps"].setVal<int>(systemMeasurementSteps);
            m_params["relaxSteps"].setVal<int>(relaxSteps);
        }
    }

    if (!retValue.containsError())
    {
        err = TLDFMX_reset(m_insrumentHdl);
        if (err)
        {
            // get error msg
            ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
            TLDFMX_error_message(m_insrumentHdl, err, buf);

            retValue += ito::RetVal::format(
                ito::retError,
                1,
                tr("Error during getting parameter: THORLABS error: '%s'.")
                    .arg(buf)
                    .toLatin1()
                    .data());
        }
    }

    if (!retValue.containsError())
    {
        for (int i = 0; i < m_nrOfAxes; i++)
        {
            m_currentStatus[i] =
                ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
        }
        emit parametersChanged(m_params);
    }

    // An error occurs if the device has no power
    retValue += getError();


    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); // init method has been finished (independent on retval)

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMH::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // todo:
    //  - disconnect the device if not yet done
    //  - this function is considered to be the "inverse" of init.

    TLDFMX_close(m_insrumentHdl);
    openedDevices.removeOne(m_params["serialNumber"].getVal<char*>());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMH::execFunc(
    const QString funcName,
    QSharedPointer<QVector<ito::ParamBase>> paramsMand,
    QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
    QSharedPointer<QVector<ito::ParamBase>> paramsOut,
    ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase* param1 = nullptr;
    ito::ParamBase* param2 = nullptr;

    // check device for error
    retValue += getError();

    if (funcName == "setZernikes")
    {
        param1 = ito::getParamByName(&(*paramsMand), "ZernikeIDs", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "ZernikeValues", &retValue);

        int numIDs = param1->getLen();
        int numZernikes = param2->getLen();

        int* zernikeIDs = param1->getVal<int*>();
        double* zernikeValues = param2->getVal<double*>();

        if (numIDs != numZernikes)
        {
            retValue += ito::RetVal(
                ito::retError,
                1,
                tr("Not same amount of IDs (length: '%1') and values (length: '%1') given.")
                    .arg(numIDs)
                    .arg(numZernikes)
                    .toLatin1()
                    .data());
        }

        if (!retValue.containsError())
        {
            QVector<double> targetZernike;
            targetZernike.fill(0.0, 16);

            for (int i = 0; i < numIDs; i++)
            {
                targetZernike[zernikeIDs[i]] = zernikeValues[i];
            }

            ViStatus err;
            TLDFMX_zernike_flag_t zernike = Z_All_Flag;
            ViReal64 zernikePattern[MAX_SEGMENTS];

            for (int i = 0; i < TLDFMX_MAX_ZERNIKE_TERMS; i++)
            {
                m_ZernikeAmplitude[i] = targetZernike[i + 4];
            }

            // Calculate voltage pattern
            err = TLDFMX_calculate_zernike_pattern(
                m_insrumentHdl, zernike, m_ZernikeAmplitude, zernikePattern);
            if (err)
            {
                // get error msg
                ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                TLDFMX_error_message(m_insrumentHdl, err, buf);

                retValue += ito::RetVal::format(
                    ito::retError,
                    1,
                    tr("Error during calculate zernike pattern: THORLABS error: '%1'")
                        .arg(buf)
                        .toLatin1()
                        .data());
            }

            // Set voltages to device, the pattern is already range checked
            // [range checking is enabled by default]
            err = TLDFM_set_segment_voltages(m_insrumentHdl, zernikePattern);
            if (err)
            {
                // get error msg
                ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                TLDFMX_error_message(m_insrumentHdl, err, buf);

                retValue += ito::RetVal::format(
                    ito::retError,
                    1,
                    tr("Error during set segment voltage: THORLABS error: '%1'")
                        .arg(buf)
                        .toLatin1()
                        .data());
            }

            ViReal64 segmentVoltages[MAX_SEGMENTS];
            err = TLDFM_get_segment_voltages(m_insrumentHdl, segmentVoltages);

            for (int i = 0; i < m_nrOfAxes; i++)
            {
                if (i < 0 || i >= m_nrOfAxes)
                {
                    retValue += ito::RetVal::format(
                        ito::retError, 1, tr("Axis %i not available.").toLatin1().data(), i);
                }
                else
                {
                    m_targetPos[i] = segmentVoltages[i];
                    m_currentPos[i] = segmentVoltages[i];
                }
            }

            sendStatusUpdate(false);
            sendTargetUpdate();
            emit parametersChanged(m_params);
        }
    }
    else if (funcName == "relaxMirror")
    {
        if (!retValue.containsError())
        {
            ViStatus err;
            ViUInt32 relaxPart = T_MIRROR;
            ViBoolean isFirstStep = VI_TRUE, reload = VI_TRUE;
            ViReal64 relaxPattern[MAX_SEGMENTS];
            ViInt32 remainingRelaxSteps;

            QMutex waitMutex;
            QWaitCondition waitCondition;

            // First Step
            err = TLDFMX_relax(
                m_insrumentHdl,
                relaxPart,
                isFirstStep,
                reload,
                relaxPattern,
                VI_NULL,
                &remainingRelaxSteps);
            if (err)
            {
                // get error msg
                ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                TLDFMX_error_message(m_insrumentHdl, err, buf);

                retValue += ito::RetVal::format(
                    ito::retError,
                    1,
                    tr("Error during relaxing mirror: THORLABS error: '%1'.")
                        .arg(buf)
                        .toLatin1()
                        .data());
            }

            err = TLDFM_set_segment_voltages(m_insrumentHdl, relaxPattern);
            if (err)
            {
                // get error msg
                ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                TLDFMX_error_message(m_insrumentHdl, err, buf);

                retValue += ito::RetVal::format(
                    ito::retError,
                    1,
                    tr("Error during relaxing mirror: THORLABS error: '%1'.")
                        .arg(buf)
                        .toLatin1()
                        .data());
            }

            isFirstStep = VI_FALSE;

            // Loop until remaining relax steps are 0
            do
            {
                err = TLDFMX_relax(
                    m_insrumentHdl,
                    relaxPart,
                    isFirstStep,
                    reload,
                    relaxPattern,
                    VI_NULL,
                    &remainingRelaxSteps);
                if (err)
                {
                    // get error msg
                    ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                    TLDFMX_error_message(m_insrumentHdl, err, buf);

                    retValue += ito::RetVal::format(
                        ito::retError,
                        1,
                        tr("Error during relaxing mirror: THORLABS error: '%1'.")
                            .arg(buf)
                            .toLatin1()
                            .data());
                }

                err = TLDFM_set_segment_voltages(m_insrumentHdl, relaxPattern);
                if (err)
                {
                    // get error msg
                    ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                    TLDFMX_error_message(m_insrumentHdl, err, buf);

                    retValue += ito::RetVal::format(
                        ito::retError,
                        1,
                        tr("Error during relaxing mirror: THORLABS error: '%1'.")
                            .arg(buf)
                            .toLatin1()
                            .data());
                }

                // short delay
                waitMutex.lock();
                waitCondition.wait(&waitMutex, 5); // 5 ms
                waitMutex.unlock();

            } while (0 < remainingRelaxSteps);
        }
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Function name '%1' does not exist.")
                .arg(funcName.toLatin1().data())
                .toLatin1()
                .data());
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
ito::RetVal ThorlabsDMH::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
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
ito::RetVal ThorlabsDMH::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
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
        retValue += it->copyValueFrom(&(*val));
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
ito::RetVal ThorlabsDMH::calib(const int axis, ItomSharedSemaphore* waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMH::calib(const QVector<int> axis, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue =
        ito::RetVal(ito::retWarning, 0, tr("Calibration not possible.").toLatin1().data());

    // check device for error
    retValue += getError();

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Motor is running. Further action is not possible.").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        ViStatus err;

        err = TLDFM_reset(m_insrumentHdl);
        if (err)
        {
            retValue +=
                ito::RetVal(ito::retError, 0, tr("Error by resetting mirror.").toLatin1().data());
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
ito::RetVal ThorlabsDMH::setOrigin(const int axis, ItomSharedSemaphore* waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------

ito::RetVal ThorlabsDMH::setOrigin(QVector<int> axis, ItomSharedSemaphore* waitCond)
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
ito::RetVal ThorlabsDMH::getStatus(
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
ito::RetVal ThorlabsDMH::getPos(
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
ito::RetVal ThorlabsDMH::getPos(
    QVector<int> axis, QSharedPointer<QVector<double>> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // check device for error
    retValue += getError();

    ViReal64 segmentVoltages[MAX_SEGMENTS];
    ViStatus err;

    err = TLDFM_get_segment_voltages(m_insrumentHdl, segmentVoltages);

    if (err)
    {
        // get error msg
        ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
        TLDFMX_error_message(m_insrumentHdl, err, buf);

        retValue += ito::RetVal::format(
            ito::retError,
            1,
            tr("Error during get segment voltages: THORLABS error: '%1'.")
                .arg(buf)
                .toLatin1()
                .data());
    }
    else
    {
        for (int i = 0; i < axis.size(); i++)
        {
            if (axis[i] >= 0 && axis[i] < m_nrOfAxes)
            {
                m_currentPos[axis[i]] =
                    segmentVoltages[axis[i]]; // set m_currentPos[i] to the obtained position
                (*pos)[i] = m_currentPos[axis[i]];
            }
            else
            {
                retValue += ito::RetVal::format(
                    ito::retError,
                    1,
                    tr("Axis '%1' not available. Only segments between 0 and %2")
                        .arg(axis[i])
                        .arg(m_nrOfAxes - 1)
                        .toLatin1()
                        .data());
            }
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
ito::RetVal ThorlabsDMH::setPosAbs(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMH::setPosAbs(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    // check device for error
    retValue += getError();

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        int cntPos = 0;
        m_targetPos = m_currentPos;

        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("Axis %i not available.").toLatin1().data(), i);
            }
            else
            {
                double minVoltage = m_params["minVoltageMirror"].getVal<double>();
                double maxVoltage = m_params["maxVoltageMirror"].getVal<double>();
                if (pos[cntPos] < minVoltage || pos[cntPos] > maxVoltage)
                {
                    retValue += ito::RetVal::format(
                        ito::retError,
                        1,
                        tr("Voltage of axis %i outside the boundaries (%.2f - %.2f): %.2f ")
                            .toLatin1()
                            .data(),
                        i,
                        minVoltage,
                        maxVoltage,
                        pos[cntPos]);
                }
                m_targetPos[i] = pos[cntPos];
            }

            cntPos++;
        }

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

            ViStatus err;
            ViReal64 segmentVoltages[MAX_SEGMENTS];

            for (int i = 0; i < m_nrOfAxes; i++)
            {
                segmentVoltages[i] = m_targetPos[i];
            }

            err = TLDFM_set_segment_voltages(m_insrumentHdl, segmentVoltages);
            if (err)
            {
                // get error msg
                ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                TLDFMX_error_message(m_insrumentHdl, err, buf);

                retValue += ito::RetVal::format(
                    ito::retError,
                    1,
                    tr("Error during set segment voltage: THORLABS error: '%1'.")
                        .arg(buf)
                        .toLatin1()
                        .data());
            }

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

            // if an error occurred, reset position
            if (retValue.containsError())
            {
                m_targetPos = m_currentPos;
            }
            retValue += waitForDone(10000, axis); // WaitForAnswer(60000, axis);

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
ito::RetVal ThorlabsDMH::setPosRel(const int axis, const double pos, ItomSharedSemaphore* waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMH::setPosRel(
    QVector<int> axis, QVector<double> pos, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    // check device for error
    retValue += getError();

    if (isMotorMoving())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        int cntPos = 0;
        m_targetPos = m_currentPos;

        ViReal64 SegmentVoltages[MAX_SEGMENTS];
        ViStatus err;

        err = TLDFM_get_segment_voltages(m_insrumentHdl, SegmentVoltages);

        foreach (const int i, axis)
        {
            if (i < 0 || i >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(
                    ito::retError, 1, tr("Axis '%1' not available").arg(i).toLatin1().data());
            }
            else
            {
                m_targetPos[i] = pos[cntPos] + SegmentVoltages[i];

                double minVoltage = m_params["minVoltageMirror"].getVal<double>();
                double maxVoltage = m_params["maxVoltageMirror"].getVal<double>();

                if (m_targetPos[i] < minVoltage || m_targetPos[i] > maxVoltage)
                {
                    retValue += ito::RetVal::format(
                        ito::retError,
                        1,
                        tr("Voltage of axis %i outside the boundaries (%.2f - %.2f): %.2f ")
                            .toLatin1()
                            .data(),
                        i,
                        minVoltage,
                        maxVoltage,
                        m_targetPos[i]);
                }
            }

            cntPos++;
        }

        if (!retValue.containsError())
        {
            // set status of all given axes to moving and keep all flags related to the status and
            // switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

            ViStatus err;
            ViReal64 segmentVoltages[MAX_SEGMENTS];

            for (int i = 0; i < m_nrOfAxes; i++)
            {
                segmentVoltages[i] = m_targetPos[i];
            }

            err = TLDFM_set_segment_voltages(m_insrumentHdl, segmentVoltages);
            if (err)
            {
                // get error msg
                ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
                TLDFMX_error_message(m_insrumentHdl, err, buf);

                retValue += ito::RetVal::format(
                    ito::retError,
                    1,
                    tr("Error during set segment voltage: THORLABS error: '%1'.")
                        .arg(buf)
                        .toLatin1()
                        .data());
            }

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

            // if an error occurred, reset position
            if (retValue.containsError())
            {
                m_targetPos = m_currentPos;
            }
            retValue += waitForDone(10000, axis); // WaitForAnswer(60000, axis);

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
ito::RetVal ThorlabsDMH::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
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

    ViReal64 SegmentVoltages[MAX_SEGMENTS];
    ViStatus err;

    while (!done && !timeout)
    {
        err = TLDFM_get_segment_voltages(m_insrumentHdl, SegmentVoltages);

        if (err)
        {
            // get error msg
            ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
            TLDFMX_error_message(m_insrumentHdl, err, buf);

            retVal += ito::RetVal::format(
                ito::retError,
                1,
                tr("Error during get segment voltages: THORLABS error: '%1'.")
                    .arg(buf)
                    .toLatin1()
                    .data());
        }

        done = true; // assume all axes at target
        motor = 0;
        for (int i = 0; i < m_nrOfAxes; i++)
        {
            m_currentPos[i] = SegmentVoltages[i];

            if (m_currentPos[i] != m_targetPos[i])
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorMoving,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = false; // not done yet
            }
            else
            {
                setStatus(
                    m_currentStatus[i],
                    ito::actuatorAtTarget,
                    ito::actSwitchesMask | ito::actStatusMask);
                done = true; // done
            }
        }

        // emit actuatorStatusChanged with both m_currentStatus and m_currentPos as arguments
        sendStatusUpdate(false);

        // now check if the interrupt flag has been set (e.g. by a button click on its dock widget)
        if (!done && isInterrupted())
        {
            // todo: force all axes to stop --> not needed

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
        // timeout occurred, set the status of all currently moving axes to timeout
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
ito::RetVal ThorlabsDMH::updateStatus()
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
//! method to select instrument
ito::RetVal ThorlabsDMH::selectInstrument()
{
    ito::RetVal retValue = ito::retOk;

    ViStatus err;
    ViUInt32 deviceCount = 0;
    int choice = 0;

    ViChar manufacturer[TLDFM_BUFFER_SIZE];
    ViChar instrumentName[TLDFM_MAX_INSTR_NAME_LENGTH];
    ViChar serialNumber[TLDFM_MAX_SN_LENGTH];
    ViBoolean deviceAvailable;

    err = TLDFM_get_device_count(VI_NULL, &deviceCount);
    if ((TL_ERROR_RSRC_NFOUND == err) || (0 == deviceCount))
    {
        retValue += ito::RetVal(
            ito::retError,
            1,
            QObject::tr("No matching instruments found, maybe device is not connected.")
                .toLatin1()
                .data());
        return retValue;
    }

    // printf("Found %d matching instrument(s):\n\n", deviceCount);

    for (ViUInt32 i = 0; i < deviceCount; i++)
    {
        err = TLDFM_get_device_information(
            VI_NULL,
            i,
            manufacturer,
            instrumentName,
            serialNumber,
            &deviceAvailable,
            m_resourceName);
    }

    // just accept the first connected deformable mirror
    choice = 1;

    err = TLDFM_get_device_information(
        VI_NULL,
        (ViUInt32)(choice - 1),
        manufacturer,
        instrumentName,
        serialNumber,
        &deviceAvailable,
        m_resourceName);

    if (VI_SUCCESS != err)
    {
        // get error msg
        ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
        TLDFMX_error_message(m_insrumentHdl, err, buf);

        retValue += ito::RetVal::format(
            ito::retError,
            1,
            QObject::tr("Error in select instrument: THORLABS error: '%1'.")
                .arg(buf)
                .toLatin1()
                .data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method to get device Info
ito::RetVal ThorlabsDMH::getDeviceInfo()
{
    ito::RetVal retValue = ito::retOk;

    ViStatus err;
    ViChar manufNameBuf[TLDFM_BUFFER_SIZE];
    ViChar instrNameBuf[TLDFM_MAX_INSTR_NAME_LENGTH];
    ViChar snBuf[TLDFM_MAX_SN_LENGTH];
    ViChar drvRevBuf[TLDFM_MAX_STRING_LENGTH];
    ViChar fwRevBuf[TLDFM_MAX_STRING_LENGTH];

    err = TLDFM_get_manufacturer_name(m_insrumentHdl, manufNameBuf);
    if (err)
    {
        // get error msg
        ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
        TLDFMX_error_message(m_insrumentHdl, err, buf);

        retValue += ito::RetVal::format(
            ito::retError,
            1,
            QObject::tr("Did not get manufacturer name, maybe device is already connected with "
                        "THORLABS software.")
                .toLatin1()
                .data(),
            buf);
    }

    if (!retValue.containsError())
    {
        m_params["manufacturerName"].setVal<char*>(manufNameBuf);
    }

    err = TLDFM_get_instrument_name(m_insrumentHdl, instrNameBuf);
    if (err)
    {
        // get error msg
        ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
        TLDFMX_error_message(m_insrumentHdl, err, buf);

        retValue += ito::RetVal::format(
            ito::retError,
            1,
            QObject::tr("Did not get instrument name: THORLABS error: '%1'.")
                .arg(buf)
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        m_params["instrumentName"].setVal<char*>(instrNameBuf);
    }

    err = TLDFM_get_serial_Number(m_insrumentHdl, snBuf);
    if (err)
    {
        // get error msg
        ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
        TLDFMX_error_message(m_insrumentHdl, err, buf);

        retValue += ito::RetVal::format(
            ito::retError,
            1,
            QObject::tr("did not get serial number: THORLABS error: '%1'.")
                .arg(buf)
                .toLatin1()
                .data());
    }
    if (!retValue.containsError())
    {
        m_params["serialNumber"].setVal<char*>(snBuf);
        openedDevices.append(snBuf);
    }

    err = TLDFMX_revision_query(m_insrumentHdl, drvRevBuf, fwRevBuf);
    if (err)
    {
        // get error msg
        ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];
        TLDFMX_error_message(m_insrumentHdl, err, buf);

        retValue += ito::RetVal::format(
            ito::retError,
            1,
            QObject::tr("did not get firmware: THORLABS error: '%1'.").arg(buf).toLatin1().data());
    }

    if (!retValue.containsError())
    {
        m_params["extensionDriver"].setVal<char*>(drvRevBuf);
        m_params["firmware"].setVal<char*>(fwRevBuf);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method to get current errors on device
ito::RetVal ThorlabsDMH::getError()
{
    ito::RetVal retValue = ito::retOk;
    ViStatus err;
    ViChar buf[TLDFM_ERR_DESCR_BUFER_SIZE];

    TLDFM_error_query(m_insrumentHdl, &err, buf);

    if (err != TLDFM_NO_ERROR)
    {
        if (err == TLDFM_ERROR_INTERNAL_PWR)
        {
            retValue += ito::RetVal::format(
                ito::retError,
                1,
                QObject::tr("'%1', check power connection").arg(buf).toLatin1().data());
        }
        else
        {
            retValue += ito::RetVal::format(
                ito::retError,
                1,
                QObject::tr("Mirror has following error msg: '%1'.").arg(buf).toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void ThorlabsDMH::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal ThorlabsDMH::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogThorlabsDMH(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ThorlabsDMH::getZernikeAmplitude(QVector<double>& zernikeAmplitude)
{
    ito::RetVal retValue = ito::retOk;

    zernikeAmplitude.resize(TLDFMX_MAX_ZERNIKE_TERMS);
    for (int i = 0; i < TLDFMX_MAX_ZERNIKE_TERMS; ++i)
    {
        zernikeAmplitude[i] = m_ZernikeAmplitude[i];
    }

    return retValue;
}
