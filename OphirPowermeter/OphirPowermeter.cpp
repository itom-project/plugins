
/* ********************************************************************
itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2020, TRUMPF SE + Co. KG, Ditzingen

This file is part of itom and its software development toolkit (SDK).

itom is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

In addition, as a special exception, the Institut f�r Technische
Optik (ITO) gives you certain additional rights.
These rights are described in the ITO LGPL Exception version 1.0,
which can be found in the file LGPL_EXCEPTION.txt in this package.

itom is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "OphirPowermeter.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include <qelapsedtimer.h>
#include <qplugin.h>
#include <qregularexpression.h>
#include <qstring.h>
#include <qstringlist.h>

#include "common/apiFunctionsInc.h"
#include "common/helperCommon.h"

#include "dockWidgetOphirPowermeter.h"

#include <comdef.h>

#define BUFFER_SIZE 100

QList<QByteArray> OphirPowermeter::m_openedDevices = QList<QByteArray>();
QList<QPair<long, long>> OphirPowermeter::m_openedUSBHandlesAndChannels = QList<QPair<long, long>>();

struct CoInitializer
{
    CoInitializer()
    {
        CoInitialize(nullptr);
    }
    ~CoInitializer()
    {
        CoUninitialize();
    }
};

//----------------------------------------------------------------------------------------------------------------------------------
OphirPowermeterInterface::OphirPowermeterInterface()
{
    m_type = ito::typeDataIO | ito::typeADDA;
    setObjectName("OphirPowermeter");

    m_description = QObject::tr("Plugin for Ophir Powermeter.");
    m_detaildescription = QObject::tr(
        "The OphirPowermeter is an itom plugin which is used for Powermeters from Ophir. \n\
It supports the RS232 and USB connection types. To use the RS232 variante, you must initiate first a serialIO plugin and use it as an initParameter. \n\
\n\
Tested devices: 1-channel VEGA (USB, RS232) with a Thermopile head.\
For the USB type it is necessary to install the COM object which comes with StarLab form Ophir.\
Download page: https://www.ophiropt.com/laser--measurement/software/com-object");

    m_author = "J. Krauter, TRUMPF SE + Co. KG";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param p(
        "connection",
        ito::ParamBase::String,
        "RS232",
        "Type of the connection ('RS232', 'USB'). For the RS232 connection a serialIO plugin "
        "instance is needed!");
    ito::StringMeta* sm = new ito::StringMeta(ito::StringMeta::String, "RS232");
    sm->addItem("USB");
    p.setMeta(sm, true);
    m_initParamsMand.append(p);

    ito::Param paramVal(
        "serial",
        ito::ParamBase::HWRef | ito::ParamBase::In,
        NULL,
        tr("An opened serial port (the right communication parameters will be set by this plugin).")
            .toLatin1()
            .data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsOpt.append(paramVal);

    m_initParamsOpt.append(ito::Param(
        "serialNo",
        ito::ParamBase::String,
        "",
        tr("Serial number of the device to be connected. If empty, all found serial numbers will "
           "be printed and the first device that is detected will be opened.")
            .toLatin1()
            .data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeterInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(OphirPowermeter) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeterInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(
        OphirPowermeter) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PlugAndPlayCallback()
{
    std::cout << "Device has been removed from the USB. \n" << std::endl;
}

//----------------------------------------------------------------------------------------------------------------------------------
OphirPowermeter::OphirPowermeter() :
    AddInDataIO(), m_pSer(NULL), m_delayAfterSendCommandMS(0), m_dockWidget(NULL),
    m_isgrabbing(false), m_data(ito::DataObject()), m_handle(0), m_channel(0)
{
    ito::Param paramVal(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave,
        "OphirPowermeter",
        tr("Name of plugin.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    m_params.insert(
        "comPort",
        ito::Param(
            "comPort",
            ito::ParamBase::Int | ito::ParamBase::Readonly,
            0,
            65355,
            0,
            tr("The current com-port ID of this specific device. -1 means undefined.")
                .toLatin1()
                .data()));
    m_params.insert(
        "battery",
        ito::Param(
            "battery",
            ito::ParamBase::Int | ito::ParamBase::Readonly,
            0,
            1,
            0,
            tr("1 if battery is OK, 0 if battery is low.").toLatin1().data()));
    m_params.insert(
        "timeout",
        ito::Param(
            "timeout",
            ito::ParamBase::Int,
            0,
            100000,
            1000,
            tr("Request timeout, default 1000 ms.").toLatin1().data()));

    m_params.insert(
        "serialNumber",
        ito::Param(
            "serialNumber",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("Serial number of the device shown on display.").toLatin1().data()));
    m_params.insert(
        "ROMVersion",
        ito::Param(
            "ROMVersion",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("Version of ROM software.").toLatin1().data()));

    paramVal = ito::Param(
        "deviceType",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Device type (NOVA, VEGA, LASERSTAR-S (single channel), LASERSTAR-D (dual channel), "
           "Nova-II).")
            .toLatin1()
            .data());
    ito::StringMeta sm(ito::StringMeta::String);
    sm.addItem("NOVA");
    sm.addItem("VEGA");
    sm.addItem("LASERSTAR-S");
    sm.addItem("LASERSTAR-D channel A");
    sm.addItem("LASERSTAR-D channel B");
    sm.addItem("NOVA-II");
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    m_params.insert(
        "headSerialNumber",
        ito::Param(
            "headSerialNumber",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("Head serial number connected to the device.").toLatin1().data()));
    m_params.insert(
        "headName",
        ito::Param(
            "headName",
            ito::ParamBase::String | ito::ParamBase::Readonly,
            "",
            tr("Head name connected to the device.").toLatin1().data()));

    paramVal = ito::Param(
        "headType",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Head type (thermopile, BC20, temperature probe, photodiode, CIE head, RP head, "
           "pyroelectric, nanoJoule meter, no head connected.")
            .toLatin1()
            .data());
    sm.setCategory("headType");
    sm.clearItems();
    sm.addItem("BC20");
    sm.addItem("Beam track");
    sm.addItem("RM9");
    sm.addItem("Axial sensor");
    sm.addItem("PD300-CIE sensor");
    sm.addItem("NanoJoule meter");
    sm.addItem("Pyroelectric");
    sm.addItem("PD300RM");
    sm.addItem("Photodiode");
    sm.addItem("Thermopile");
    sm.addItem("Temperature probe");
    sm.addItem("No sensor connected");
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal =
        ito::Param("unit", ito::ParamBase::String, "", tr("Unit of device").toLatin1().data());
    sm.setCategory("unit");
    sm.clearItems();
    sm.addItem("dBm");
    sm.addItem("A");
    sm.addItem("J");
    sm.addItem("V");
    sm.addItem("W");
    sm.addItem("Lux");
    sm.addItem("fc");
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "measurementType",
        ito::ParamBase::String,
        "power",
        tr("Measurement type (energy or power).").toLatin1().data());
    sm.setCategory("measurementType");
    sm.clearItems();
    sm.addItem("energy");
    sm.addItem("power");
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "wavelengthSet",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "",
        tr("Setting of the measurement wavelength (DISCRETE or CONTINUOUS).").toLatin1().data());
    sm.setCategory("wavelengthSet");
    sm.clearItems();
    sm.addItem("DISCRETE");
    sm.addItem("CONTINUOUS");
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "calibrationDueDate",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "Date not available",
        tr("Calibration due date.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    sm.clearItems();
    paramVal = ito::Param(
        "connection",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "RS232",
        "type of the connection ('RS232', 'USB').");
    sm.addItem("RS232");
    sm.addItem("USB");
    paramVal.setMeta(&sm, false);
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        // now create dock widget for this plugin
        DockWidgetOphirPowermeter* m_dockWidget = new DockWidgetOphirPowermeter(getID(), this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, m_dockWidget);
    }

    m_charBuffer = (char*)malloc(BUFFER_SIZE);
}

//----------------------------------------------------------------------------------------------------------------------------------
OphirPowermeter::~OphirPowermeter()
{
    CoUninitialize();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal OphirPowermeter::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QByteArray answerStr;
    int answerInt;
    QByteArray request;

    QByteArray type = paramsMand->at(0).getVal<char*>(); // RS232 or USB

    QByteArray serialNoInput = paramsOpt->at(1).getVal<char*>();

    if (m_openedDevices.contains(
            serialNoInput)) // exit here if device with serial number already connected
    {
        retval += ito::RetVal(
            ito::retError,
            0,
            tr("The given serial number %1 is already connected.")
                .arg(serialNoInput.data())
                .toLatin1()
                .data());
    }

    if (!retval.containsError())
    {
        if (type == "RS232") // init as RS232 powermeter type
        {
            m_connection = connectionType::RS232;
            m_params["connection"].setVal<const char*>("RS232");

            if ((ito::AddInDataIO*)(*paramsOpt)[0].getVal<void*>())
            {
                // check serialIO and set parameters
                if (reinterpret_cast<ito::AddInBase*>((*paramsOpt)[0].getVal<void*>())
                        ->getBasePlugin()
                        ->getType() &
                    (ito::typeDataIO | ito::typeRawIO))
                {
                    m_pSer = (ito::AddInDataIO*)(*paramsOpt)[0].getVal<void*>();
                    retval += m_pSer->setParam(
                        QSharedPointer<ito::ParamBase>(
                            new ito::ParamBase("baud", ito::ParamBase::Int, 9600)),
                        NULL);
                    retval += m_pSer->setParam(
                        QSharedPointer<ito::ParamBase>(
                            new ito::ParamBase("bits", ito::ParamBase::Int, 8)),
                        NULL);
                    retval += m_pSer->setParam(
                        QSharedPointer<ito::ParamBase>(
                            new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)),
                        NULL);
                    retval += m_pSer->setParam(
                        QSharedPointer<ito::ParamBase>(
                            new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)),
                        NULL);
                    retval += m_pSer->setParam(
                        QSharedPointer<ito::ParamBase>(
                            new ito::ParamBase("flow", ito::ParamBase::Int, 0)),
                        NULL);
                    retval += m_pSer->setParam(
                        QSharedPointer<ito::ParamBase>(
                            new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")),
                        NULL);
                }
                else
                {
                    retval += ito::RetVal(
                        ito::retError,
                        1,
                        tr("No serialIO plugin instance given. A SerialIO instance is needed to "
                           "use the RS232 type of powermeter.")
                            .toLatin1()
                            .data());
                }
            }
            else
            {
                retval += ito::RetVal(
                    ito::retError,
                    1,
                    tr("No serialIO plugin instance given. A SerialIO instance is needed to use "
                       "the RS232 type of powermeter.")
                        .toLatin1()
                        .data());
            }

            if (!retval.containsError())
            {
                QSharedPointer<QVector<ito::ParamBase>> emptyParamVec(
                    new QVector<ito::ParamBase>());
                m_pSer->execFunc("clearInputBuffer", emptyParamVec, emptyParamVec, emptyParamVec);
                m_pSer->execFunc("clearOutputBuffer", emptyParamVec, emptyParamVec, emptyParamVec);

                QSharedPointer<ito::Param> param(new ito::Param("port"));
                retval += m_pSer->getParam(param, NULL);
                if (retval.containsError() || param->getVal<int>() < 1)
                {
                    retval += ito::RetVal(
                        ito::retError,
                        0,
                        tr("Could not read port number from serial port or port number invalid")
                            .toLatin1()
                            .data());
                }
                else
                {
                    m_params["comPort"].setVal<int>(param->getVal<int>());
                }
                Sleep(500);
            }

            if (!retval.containsError()) // get information
            {
                QByteArray headType = "";
                QByteArray serialNum = "";
                request = QByteArray("$HI");
                retval += SendQuestionWithAnswerString(
                    request,
                    answerStr,
                    m_params["timeout"].getVal<int>()); // optical output check query

                QRegularExpression reg("(\\S+)"); // matches numbers
                QRegularExpressionMatchIterator regMatchIt = reg.globalMatch(answerStr);

                QStringList list = QStringList();
                bool found = false;
                QByteArray headSerial;
                QByteArray headName;

                while (regMatchIt.hasNext())
                {
                    QRegularExpressionMatch regMatch = regMatchIt.next();
                    list << regMatch.captured(1);
                    found = true;

                }
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
                QStringList list2 = QString::fromStdString(answerStr.toStdString())
                                        .split(" ", Qt::SkipEmptyParts);
#else
                QStringList list2 = QString::fromStdString(answerStr.toStdString())
                                        .split(" ", QString::SkipEmptyParts);
#endif
                if (found)
                {
                    headSerial = list.at(1).toLatin1();
                    headName = list.at(2).toLatin1();
                }
                else
                {
                    retval += ito::RetVal(
                        ito::retError,
                        0,
                        tr("Could not read head serial number")
                            .toLatin1()
                            .data());
                }

                if (!retval.containsError())
                {
                    m_params["headSerialNumber"].setVal<char*>(headSerial.data());
                    m_params["headName"].setVal<char*>(headName.data());

                    if (answerStr.contains("BC"))
                    {
                        headType = "BC20";
                    }
                    else if (answerStr.contains("BT"))
                    {
                        headType = "Beam track";
                    }
                    else if (answerStr.contains("CR"))
                    {
                        headType = "RM9";
                    }
                    else if (answerStr.contains("CP") || answerStr.contains("PY"))
                    {
                        headType = "Pyroelectric";
                    }
                    else if (answerStr.contains("FX"))
                    {
                        headType = "Axial sensor";
                    }
                    else if (answerStr.contains("LX"))
                    {
                        headType = "PD300-CIE sensor";
                    }
                    else if (answerStr.contains("NJ"))
                    {
                        headType = "NanoJoule meter";
                    }
                    else if (answerStr.contains("RM"))
                    {
                        headType = "PD300RM";
                    }
                    else if (answerStr.contains("SI"))
                    {
                        headType = "photodiode";
                    }
                    else if (answerStr.contains("TH"))
                    {
                        headType = "Thermopile";
                    }
                    else if (answerStr.contains("TP"))
                    {
                        headType = "Temperature probe";
                    }
                    else if (answerStr.contains("XX"))
                    {
                        headType = "No sensor connected";
                    }
                    else
                    {
                        retval += ito::RetVal(
                            ito::retError,
                            0,
                            tr("return answer %1 for rquest $HT not found.")
                                .arg(answerStr.data())
                                .toLatin1()
                                .data());
                    }

                    m_params["headType"].setVal<char*>(headType.data());
                }
            }

            if (!retval.containsError())
            {
                request = QByteArray("$VE");
                retval += SendQuestionWithAnswerString(
                    request,
                    answerStr,
                    m_params["timeout"].getVal<int>()); // optical output check query
                m_params["ROMVersion"].setVal<char*>(answerStr.data());
            }

            if (!retval.containsError()) // instrument information
            {
                bool found = false;
                QByteArray type;
                request = QByteArray("$II");
                retval += SendQuestionWithAnswerString(
                    request,
                    answerStr,
                    m_params["timeout"].getVal<int>()); // optical output check query

                QRegularExpression reg("(\\S+)"); // matches numbers
                QRegularExpressionMatchIterator regMatchIt = reg.globalMatch(answerStr);

                QStringList list = QStringList();
                QByteArray foundSerialNo;
                found = false;


                while (regMatchIt.hasNext())
                {
                    QRegularExpressionMatch regMatch = regMatchIt.next();
                    list << regMatch.captured(1);
                    found = true;
                }

                if (found)
                {
                    foundSerialNo = list.at(1).toLatin1();
                }
                else
                {
                    retval += ito::RetVal(
                        ito::retError,
                        0,
                        tr("Cound not read instrument serial number.").toLatin1().data());
                }

                if (!retval.containsError())
                {

                    if (serialNoInput == "")
                    {
                        found = true;
                    }
                    else if (
                        serialNoInput.contains(foundSerialNo) &&
                        serialNoInput.length() == foundSerialNo.length())
                    {
                        found = true;
                    }
                    else
                    {
                        retval += ito::RetVal(
                            ito::retError,
                            0,
                            tr("Given serial number %1 does not match the received number %2")
                                .arg(serialNoInput.data())
                                .arg(foundSerialNo.data())
                                .toLatin1()
                                .data());
                    }
                }

                if (!retval.containsError() && found)
                {
                    m_params["serialNumber"].setVal<char*>(foundSerialNo.data());

                    if (answerStr.contains("NOVA"))
                    {
                        type = "NOVA";
                    }
                    else if (answerStr.contains("VEGA"))
                    {
                        type = "VEGA";
                    }
                    else if (answerStr.contains("LS-A 54545"))
                    {
                        type = "LASERSTAR-S";
                    }
                    else if (answerStr.contains("LS-A 23452"))
                    {
                        type = "LASERSTAR-D channel A";
                    }
                    else if (answerStr.contains("LS-B 23453"))
                    {
                        type = "LASERSTAR-D channel B";
                    }
                    else
                    {
                        retval += ito::RetVal(
                            ito::retError,
                            0,
                            tr("return answer %1 for rquest $HT not found.")
                                .arg(answerStr.data())
                                .toLatin1()
                                .data());
                    }

                    m_params["deviceType"].setVal<char*>(type.data());
                }
            }

            if (!retval.containsError()) // get unit of measurement
            {
                QByteArray unit;
                request = QByteArray("$SI");
                retval += SendQuestionWithAnswerString(
                    request,
                    answerStr,
                    m_params["timeout"].getVal<int>()); // optical output check query

                if (!retval.containsError())
                {
                    if (answerStr.contains("W"))
                    {
                        unit = "W";
                    }
                    else if (answerStr.contains("V"))
                    {
                        unit = "V";
                    }
                    else if (answerStr.contains("A"))
                    {
                        unit = "A";
                    }
                    else if (answerStr.contains("d"))
                    {
                        unit = "dBm";
                    }
                    else if (answerStr.contains("l"))
                    {
                        unit = "Lux";
                    }
                    else if (answerStr.contains("c"))
                    {
                        unit = "fc";
                    }
                    else if (answerStr.contains("J"))
                    {
                        unit = "J";
                    }
                    else
                    {
                        retval += ito::RetVal(
                            ito::retError,
                            0,
                            tr("return answer %1 for rquest $HT not found.")
                                .arg(answerStr.data())
                                .toLatin1()
                                .data());
                    }

                    m_params["unit"].setVal<char*>(unit.data());
                }
            }

            if (!retval.containsError()) // get the wavelength settings
            {
                request = QByteArray("$AW");
                retval += SendQuestionWithAnswerString(
                    request,
                    answerStr,
                    m_params["timeout"].getVal<int>()); // optical output check query


                QRegularExpression reg("(\\S+)"); // matches numbers
                QRegularExpressionMatchIterator regMatchIt = reg.globalMatch(answerStr);

                QStringList list;


                while (regMatchIt.hasNext())
                {
                    QRegularExpressionMatch regMatch = regMatchIt.next();
                    list << regMatch.captured(1);
                }

                m_params["wavelengthSet"].setVal<char*>(list.at(0).toLatin1().data());

                if (list.at(0).contains("DISCRETE"))
                {
                    ito::StringMeta sm(ito::StringMeta::String);

                    QString discreteWavelengths;
                    int currentWaveIdx = list.at(1).toInt();
                    for (int idx = 2; idx < list.size(); idx++)
                    {
                        discreteWavelengths += " "; // space
                        if (!list.at(idx).contains("NONE"))
                        {
                            sm.addItem(list.at(idx).toLatin1().data());
                            discreteWavelengths += list.at(idx).toLatin1().data();
                        }
                    }

                    ito::Param paramVal = ito::Param(
                        "wavelength",
                        ito::ParamBase::String,
                        list.at(currentWaveIdx + 1).toLatin1().data(),
                        tr("Available discrete wavelengths:%1.")
                            .arg(discreteWavelengths)
                            .toLatin1()
                            .data());

                    paramVal.setMeta(&sm, false);
                    m_params.insert(paramVal.getName(), paramVal);
                }
                else if (list.at(0).contains("CONTINUOUS"))
                {
                    int waveMin = list.at(1).toInt();
                    int waveMax = list.at(2).toInt();
                    int waveCur = list.at(list.at(3).toInt() + 3).toInt();

                    ito::Param paramVal = ito::Param(
                        "wavelength",
                        ito::ParamBase::Int,
                        0,
                        new ito::IntMeta(waveMin, waveMax),
                        tr("Set Wavelengths [nm] continuous between: %1 - %2.")
                            .arg(waveMin)
                            .arg(waveMax)
                            .toLatin1()
                            .data());
                    m_params.insert(paramVal.getName(), paramVal);
                    m_params["wavelength"].setVal<int>(waveCur);
                }
                else
                {
                    retval += ito::RetVal(
                        ito::retError, 0, tr("Wavelength is not available.").toLatin1().data());
                }
            }

            if (!retval.containsError())
            {
                request = QByteArray("$RN");
                retval += SendQuestionWithAnswerInt(
                    request,
                    answerInt,
                    m_params["timeout"].getVal<int>()); // optical output check query

                ito::Param paramVal = ito::Param(
                    "range",
                    ito::ParamBase::Int,
                    -2,
                    2,
                    0,
                    tr("Measurement range (-2: dBm autoranging, -1: autoranging, 0: highest range, "
                       "1: second range, 2: next highest range).")
                        .toLatin1()
                        .data());
                m_params.insert(paramVal.getName(), paramVal);
                m_params["range"].setVal<int>(answerInt);
            }

            if (!retval.containsError()) // request if energy or power measurement
            {
                request = QByteArray("$FP");
                retval += SerialSendCommand(request); // does not return a value

                m_params["measurementType"].setVal<const char*>("power");

                Sleep(1000); // give the device some time

                // dummy read
                double answer;
                QByteArray request;
                request = QByteArray("$SP");
                retval += SendQuestionWithAnswerDouble(
                    request,
                    answer,
                    m_params["timeout"].getVal<int>()); // optical output check query
            }
        }
        else // connect to USB powermeter type
        {
            m_connection = connectionType::USB;
            m_params["connection"].setVal<const char*>("USB");

            // creates instance of OphirLMMeasurement
            try
            {
                CoInitializer initializer; // must call for COM initialization and deinitialization
                m_OphirLM = QSharedPointer<OphirLMMeasurement>(new OphirLMMeasurement);
                
                if (m_OphirLM.isNull())
                {
                    m_OphirLM.clear();

                    retval += ito::RetVal(
                        ito::retError,
                        0,
                        tr("No USB device was found.").arg(serialNoInput.data()).toLatin1().data());
                }
            }
            catch (const _com_error& e)
            {
                retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
            }


            // scan for serial numbers
            if (!retval.containsError())
            {
                bool found = false;
                std::vector<std::wstring> serialsFound;

                // Scan for connected Devices
                try
                {
                    m_OphirLM->ScanUSB(serialsFound);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }

                if (serialsFound.size() > 0) // found some connected devices
                {
                    QByteArray newSerial;
                    for (int idx = 0; idx < serialsFound.size();
                         idx++) // iterate through all serialnumbers
                    {
                        char* foundTemp = wCharToChar(serialsFound[idx].c_str());

                        if (m_openedDevices.contains(wCharToChar(serialsFound[idx].c_str())) &&
                            (serialNoInput == serialsFound[idx].c_str())) // already connected
                        {
                            retval += ito::RetVal(
                                ito::retError,
                                0,
                                tr("The given input serial %1 is already connected.")
                                    .arg(wCharToChar(serialsFound[idx].c_str()))
                                    .toLatin1()
                                    .data());
                        }
                        else if (
                            serialNoInput.size() > 0 &&
                            serialNoInput.contains(wCharToChar(
                                serialsFound[idx].c_str()))) // connected to optional parameter/
                                                             // input serial number
                        {
                            m_serialNo = serialsFound[idx];
                            found = true;
                            retval = ito::retOk;
                            break;
                        }
                    }
                }
                else
                {
                    retval += ito::RetVal(ito::retError, 0, "no connected Ophir device detected");
                }

                if (!found && !retval.containsError()) // serials found but not
                {
                    std::cout << "Detected serial numbers of USB "
                                 "devices:\n------------------------------\n"
                              << std::endl;
                    std::wstring deviceSerial;
                    foreach (deviceSerial, serialsFound)
                    {
                        std::cout << wCharToChar(deviceSerial.c_str()) << "\n" << std::endl;
                    }

                    retval += ito::RetVal(
                        ito::retError,
                        0,
                        tr("More like one device was found. Initialization only possible with the "
                           "corresponding serial number for the plugin input parameter *serialNo*.")
                            .toLatin1()
                            .data());
                    found = true;
                }
            }


            // open device
            if (!retval.containsError())
            {
                try
                {
                    m_OphirLM->OpenUSBDevice(m_serialNo, m_handle);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }
            }

            // check if sensor exists
            if (!retval.containsError())
            {
                m_openedDevices.append(wCharToChar(m_serialNo.c_str()));
                QPair<long, long> pair = QPair<long, long>(m_handle, m_channel);
                m_openedUSBHandlesAndChannels.append(pair);

                bool exists;
                try
                {
                    m_OphirLM->IsSensorExists(m_handle, m_channel, exists);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }


                if (!exists)
                {
                    retval += ito::RetVal(ito::retError, 0, "no sensor connected to the device");
                }
            }

            // get device infos
            if (!retval.containsError())
            {
                std::wstring deviceName, romVersion, serialNumber;
                std::wstring info, headSN, headType, headName, version;
                try
                {
                    m_OphirLM->GetDeviceInfo(m_handle, deviceName, romVersion, serialNumber);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }


                m_params["deviceType"].setVal<char*>(wCharToChar(deviceName.c_str()));
                m_params["ROMVersion"].setVal<char*>(wCharToChar(romVersion.c_str()));
                m_params["serialNumber"].setVal<char*>(wCharToChar(serialNumber.c_str()));

                try
                {
                    m_OphirLM->GetSensorInfo(m_handle, 0, headSN, headType, headName);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }

                m_params["headSerialNumber"].setVal<char*>(wCharToChar(headSN.c_str()));
                m_params["headType"].setVal<char*>(wCharToChar(headType.c_str()));
                m_params["headName"].setVal<char*>(wCharToChar(headName.c_str()));
            }

            if (!retval.containsError()) // get wavelengths
            {
                bool modifiable;
                long waveMin;
                long waveMax;
                try
                {
                    m_OphirLM->GetWavelengthsExtra(
                        m_handle, m_channel, modifiable, waveMin, waveMax);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }


                if (modifiable) // continuous
                {
                    ito::Param paramVal = ito::Param(
                        "wavelength",
                        ito::ParamBase::Int,
                        0,
                        new ito::IntMeta(waveMin, waveMax),
                        tr("Set Wavelengths [nm] continuous between: %1 - %2.")
                            .arg(waveMin)
                            .arg(waveMax)
                            .toLatin1()
                            .data());
                    m_params.insert(paramVal.getName(), paramVal);

                    try
                    {
                        m_OphirLM->ModifyWavelength(m_handle, m_channel, 0, waveMin);
                    }
                    catch (const _com_error& e)
                    {
                        retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                    }


                    m_params["wavelength"].setVal<int>(waveMin);
                    m_params["wavelengthSet"].setVal<const char*>("CONTINUOUS");
                }
                else // discrete
                {
                    LONG index;
                    std::vector<std::wstring> options;
                    m_OphirLM->GetWavelengths(m_handle, m_channel, index, options);

                    ito::StringMeta sm(ito::StringMeta::String);

                    QString discreteWavelengths;
                    for (int idx = 0; idx < options.size(); idx++)
                    {
                        m_discreteWavelengths.insert(wCharToChar(options[idx].c_str()), idx);
                        discreteWavelengths += " "; // space
                        sm.addItem(wCharToChar(options[idx].c_str()));
                        discreteWavelengths += wCharToChar(options[idx].c_str());
                    }

                    ito::Param paramVal = ito::Param(
                        "wavelength",
                        ito::ParamBase::String,
                        wCharToChar(options.at(0).c_str()),
                        tr("Available discrete wavelengths:%1.")
                            .arg(discreteWavelengths)
                            .toLatin1()
                            .data());
                    paramVal.setMeta(&sm, false);
                    m_params.insert(paramVal.getName(), paramVal);

                    try
                    {
                        m_OphirLM->SetWavelength(m_handle, m_channel, 0); // set first wavelength
                    }
                    catch (const _com_error& e)
                    {
                        retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                    }


                    m_params["wavelengthSet"].setVal<const char*>("DISCRETE");
                }
            }

            // get calibration due date
            if (!retval.containsError())
            {
                std::tm dueDate;

                try
                {
                    m_OphirLM->GetDeviceCalibrationDueDate(m_handle, dueDate);
                    m_params["calibrationDueDate"].setVal<char*>(QString("%1:%L2:%L3")
                                                                     .arg(dueDate.tm_year)
                                                                     .arg(dueDate.tm_mon)
                                                                     .arg(dueDate.tm_mday)
                                                                     .toLatin1()
                                                                     .data());
                }
                catch (const _com_error& /*e*/)
                {
                    m_params["calibrationDueDate"].setVal<const char*>("not available");
                }
            }

            // get ranges
            if (!retval.containsError())
            {
                long index;
                std::vector<std::wstring> options;
                try
                {
                    m_OphirLM->GetRanges(m_handle, m_channel, index, options);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }


                std::wstring docu = L"Measurement range (";
                std::vector<std::wstring>::iterator it;

                int idx = 0;
                for (it = options.begin(); it != options.end(); ++it)
                {
                    docu += std::to_wstring(idx);
                    docu += L": ";
                    docu += *it;
                    docu += L", ";
                    idx += 1;
                }
                docu.pop_back(); // delete "; " again
                docu.pop_back();

                docu += L").";

                ito::Param paramVal = ito::Param(
                    "range",
                    ito::ParamBase::Int,
                    0,
                    idx,
                    index,
                    tr(wCharToChar(docu.c_str())).toLatin1().data());
                m_params.insert(paramVal.getName(), paramVal);

                // there is no OphirLM function to get the unit
                // here is some workaround
                QByteArray unit;
                QByteArray opt = wCharToChar(options[idx - 1].c_str());
                if (opt.contains("W"))
                {
                    unit = "W";
                }
                else if (opt.contains("V"))
                {
                    unit = "V";
                }
                else if (opt.contains("A"))
                {
                    unit = "A";
                }
                else if (opt.contains("d"))
                {
                    unit = "dBm";
                }
                else if (opt.contains("l"))
                {
                    unit = "Lux";
                }
                else if (opt.contains("c"))
                {
                    unit = "fc";
                }
                else if (opt.contains("J"))
                {
                    unit = "J";
                }
                else
                {
                    retval += ito::RetVal(
                        ito::retError,
                        0,
                        tr("return answer %1 not found.").arg(opt.data()).toLatin1().data());
                }

                m_params["unit"].setVal<char*>(unit.data());
            }

            // get measurement mode
            if (!retval.containsError())
            {
                long index;
                std::vector<std::wstring> options;
                try
                {
                    m_OphirLM->GetMeasurementMode(m_handle, m_channel, index, options);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }


                for (int idx = 0; idx < options.size(); idx++)
                {
                    m_measurementModes.insert(wCharToChar(options[idx].c_str()), idx);
                }
                try
                {
                    m_OphirLM->SetMeasurementMode(m_handle, m_channel, 0);
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }
            }

            if (!retval.containsError())
            {
                try
                {
                    // start measuring on first device
                    m_OphirLM->RegisterPlugAndPlay(PlugAndPlayCallback);
                    m_OphirLM->ConfigureStreamMode(m_handle, m_channel, 0, 0); // turbo off
                    m_OphirLM->ConfigureStreamMode(m_handle, m_channel, 2, 1); // immediate on
                    m_isgrabbing = false;
                }
                catch (const _com_error& e)
                {
                    retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }
            }

            if (!retval.containsError()) // set some default values
            {
                QSharedPointer<ito::Param> paramVal = QSharedPointer<ito::Param>(new ito::Param(
                    "range",
                    ito::ParamBase::Int,
                    -2,
                    2,
                    0,
                    tr("Measurement range (-2: dBm autoranging, -1: autoranging, 0: highest range, "
                       "1: second range, 2: next highest range).")
                        .toLatin1()
                        .data()));
                ItomSharedSemaphore* dummyWait = new ItomSharedSemaphore();
                retval += setParam(paramVal, dummyWait);

                if (dummyWait)
                {
                    dummyWait->returnValue = retval;
                    dummyWait->release();
                    dummyWait = NULL;
                }
            }

            if (!retval.containsError()) // set RS232 params to readonly
            {
                m_params["comPort"].setFlags(ito::ParamBase::Readonly);
                m_params["comPort"].setVal<int>(-1);
            }
        }
    }


    if (!retval.containsError()) // set the size of the m_data object
    {
        m_data = ito::DataObject(1, 1, ito::tFloat64);
        m_data.setValueUnit(m_params["unit"].getVal<char*>());
        m_data.setValueDescription("power");
    }


    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }


    setInitialized(true); // init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    visibilityChanged(false);
    if (m_connection == connectionType::USB && !m_OphirLM.isNull())
    {
        try
        {
            //m_OphirLM->StopStream(m_handle, m_channel);
            m_OphirLM->Close(m_handle);
        }
        catch (const _com_error& e)
        {
            retValue += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
        }

        m_openedDevices.removeOne(wCharToChar(m_serialNo.c_str()));
        QPair<long, long> pair = QPair<long, long>(m_handle, m_channel);
        m_openedUSBHandlesAndChannels.removeOne(pair);
    }

    // Free multibyte character buffer
    if (m_charBuffer)
    {
        free(m_charBuffer);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    QByteArray request;

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
        if (key.compare("") == 0)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("name of requested parameter is empty.").toLatin1().data());
        }
        else if (key.compare("battery") == 0)
        {
            int answer;
            request = QByteArray("$BC");
            retValue += SendQuestionWithAnswerInt(
                request, answer, m_params["timeout"].getVal<int>()); // optical output check query

            if (!retValue.containsError())
            {
                val->setVal<int>(answer);
            }
        }
        else
        {
            QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(key);
            if (paramIt != m_params.constEnd())
            {
                *val = paramIt.value();
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
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
ito::RetVal OphirPowermeter::setParam(
    QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QByteArray request;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        // gets the parameter key from m_params map (read-only is not allowed and leads to
        // ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        // here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key.compare("measurementType") == 0)
        {
            if (m_connection == connectionType::RS232)
            {
                QString type = QLatin1String(val->getVal<char*>());
                if (type.compare("energy") == 0)
                {
                    request = "$FE";
                    retValue += SerialSendCommand(request);

                    if (!retValue.containsError())
                    {
                        it->setVal<const char*>("energy");

                        m_data.setValueDescription("energy");
                    }
                }
                else if (type.compare("power") == 0)
                {
                    request = "$FP";
                    retValue += SerialSendCommand(request);

                    if (!retValue.containsError())
                    {
                        it->setVal<const char*>("power");
                        m_data.setValueDescription("power");
                    }
                }

                else
                {
                    return ito::RetVal(
                        ito::retError,
                        0,
                        tr("Parameter value: %1 is unknown.")
                            .arg(val->getVal<char*>())
                            .toLatin1()
                            .data());
                }

                if (!retValue.containsError())
                {
                    QSharedPointer<ito::Param> param(new ito::Param("unit"));
                    retValue += this->getParam(param, NULL);
                    m_data.setValueUnit(param->getVal<char*>());
                }
            }
            else
            {
                try
                {
                    m_OphirLM->SetMeasurementMode(
                        m_handle, m_channel, m_measurementModes.value(val->getVal<char*>()));
                }
                catch (const _com_error& e)
                {
                    retValue += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }

                Sleep(0.2);
            }
        }
        else if (key.compare("wavelength") == 0)
        {
            if (m_connection == connectionType::RS232)
            {
                QString setWave = QString::fromLatin1(m_params["wavelengthSet"].getVal<char*>());

                if (setWave.compare("DISCRETE") == 0)
                {
                    QByteArray request = "$WW ";
                    request.append(val->getVal<char*>());

                    retValue += SerialSendCommand(request);

                    if (!retValue.containsError())
                    {
                        retValue += it->copyValueFrom(&(*val));
                    }
                }
                else // CONTINUOUS
                {
                    QByteArray request = "$WL ";

                    int wave = val->getVal<int>();
                    QByteArray setVal;
                    setVal.setNum(wave);
                    request.append(setVal);

                    retValue += SerialSendCommand(request);

                    if (!retValue.containsError())
                    {
                        retValue += it->copyValueFrom(&(*val));
                    }
                }
            }
            else
            {
                QString setWave = QString::fromLatin1(m_params["wavelengthSet"].getVal<char*>());
                try
                {
                    if (setWave.compare("DISCRETE") == 0)
                    {
                        m_OphirLM->SetWavelength(
                            m_handle, m_channel, m_discreteWavelengths.value(val->getVal<char*>()));
                    }
                    else
                    {
                        m_OphirLM->ModifyWavelength(m_handle, m_channel, 0, val->getVal<int>());
                    }
                }
                catch (const _com_error& e)
                {
                    retValue += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }
            }
        }
        else if (key.compare("range") == 0)
        {
            if (m_connection == connectionType::RS232)
            {
                QByteArray request = "$WN ";

                int idx = val->getVal<int>();
                QByteArray setVal;
                setVal.setNum(idx);
                request.append(setVal);

                retValue += SerialSendCommand(request);
            }
            else
            {
                try
                {
                    m_OphirLM->SetRange(m_handle, m_channel, val->getVal<int>());
                }
                catch (const _com_error& e)
                {
                    retValue += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
                }
            }

            if (!retValue.containsError())
            {
                retValue += it->copyValueFrom(&(*val));
            }
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
ito::RetVal OphirPowermeter::startDevice(ItomSharedSemaphore* waitCond)
{
    ito::RetVal retval =
        ito::RetVal(ito::retWarning, 0, tr("StartDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::stopDevice(ItomSharedSemaphore* waitCond)
{
    ito::RetVal retval =
        ito::RetVal(ito::retWarning, 0, tr("StopDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
bool OphirPowermeter::definitelyGreaterThan(const double& a, const double& b)
{
    return (a - b) > ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) *
                      std::numeric_limits<double>::epsilon());
}

//----------------------------------------------------------------------------------------------------------------------------------
bool OphirPowermeter::definitelyLessThan(const double& a, const double& b)
{
    return (b - a) > ((std::abs(a) < std::abs(b) ? std::abs(b) : std::abs(a)) *
                      std::numeric_limits<double>::epsilon());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::acquire(const int trigger, ItomSharedSemaphore* waitCond)
{
    ito::RetVal retval(ito::retOk);

    QByteArray request;
    QTime timer;
    bool done = false;

    double value;


    m_isgrabbing = true;
    if (m_connection == connectionType::RS232)
    {
        QString type = QLatin1String(m_params["measurementType"].getVal<char*>());

        if (type.compare("energy") == 0)
        {
            request = QByteArray("$SE");
        }
        else if (type.compare("power") == 0)
        {
            request = QByteArray("$SP");
        }
        else
        {
            retval += ito::RetVal(
                ito::retError,
                0,
                tr("given measurement type %1 is unknown.").arg(type).toLatin1().data());
        }

        retval += SendQuestionWithAnswerDouble(
            request, value, m_params["timeout"].getVal<int>()); // optical output check query
    }
    else
    {
        try
        {
            m_OphirLM->StartStream(m_handle, m_channel);
        }
        catch (const _com_error& e)
        {
            retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
        }

        std::vector<double> values;
        std::vector<double> timestamps;
        std::vector<OphirLMMeasurement::Status> statuses;
        bool hasNewValue = false;
        QElapsedTimer timer;
        int timeout = m_params["timeout"].getVal<int>();
        timer.start();

        while (!timer.hasExpired(timeout))
        {

            try
            {
                m_OphirLM->GetData(m_handle, m_channel, values, timestamps, statuses);
            }
            catch (const _com_error& e)
            {
                retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
            }

            if (values.size() > 0)
            {
                value = values[0];
                hasNewValue = true;
                break;
            }
        }

        try
        {
            m_OphirLM->StopStream(m_handle, m_channel);
        }
        catch (const _com_error& e)
        {
            retval += ito::RetVal(ito::retError, 0, TCharToChar(e.ErrorMessage()));
        }

        if (!hasNewValue)
        {
            retval += ito::RetVal(ito::retError, 0, "timeout during getData.");
        }
    }

    if (!retval.containsError())
    {
        if (definitelyLessThan(std::abs(value), 1E-30))
        {
            m_data.at<ito::float64>(0, 0) = 0.0;
        }
        else if (definitelyGreaterThan(std::abs(value), 1E30))
        {
            m_data.at<ito::float64>(0, 0) = std::numeric_limits<ito::float64>::infinity();
        }
        else
        {
            m_data.at<ito::float64>(0, 0) = value;
        }

        m_data.setValueUnit(m_params["unit"].getVal<char*>());
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

ito::RetVal OphirPowermeter::acquireAutograbbing(
    QSharedPointer<double> value, QSharedPointer<QString> unit, ItomSharedSemaphore* waitCond)
{
    ito::RetVal retval(ito::retOk);
    retval = acquire(0, waitCond);

    *value = m_data.at<ito::float64>(0, 0);
    *unit = QString::fromStdString(m_data.getValueUnit());

    m_isgrabbing = false;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::retrieveData(ito::DataObject* externalDataObject)
{
    // todo: this is just a basic example for getting the buffered image to m_data or the externally
    // given data object enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(
            ito::retWarning,
            0,
            tr("Tried to get picture without triggering exposure").toLatin1().data());
        return retValue;
    }


    if (externalDataObject == NULL)
    {
        return retValue;
    }
    else
    {
        retValue += checkData(externalDataObject);

        if (!retValue.containsError())
        {
            retValue += m_data.deepCopyPartial(*externalDataObject);
        }
        m_isgrabbing = false;
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::checkData(ito::DataObject* externalDataObject)
{
    int futureHeight = 1;
    int futureWidth = 1;
    int futureType = ito::tFloat64;
    ito::RetVal retval;

    if (externalDataObject == NULL)
    {
        if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight ||
            m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight, futureWidth, futureType);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(futureHeight, futureWidth, futureType);
        }
        else if (externalDataObject->calcNumMats() != 1)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Error during check data, external dataObject invalid. Object has more than 1 "
                   "plane or zero planes. It must be of right size and type or an uninitialized "
                   "image.")
                    .toLatin1()
                    .data());
        }
        else if (
            externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight ||
            externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth ||
            externalDataObject->getType() != futureType)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Error during check data, external dataObject invalid. Object must be of right "
                   "size and type or a uninitialized image.")
                    .toLatin1()
                    .data());
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as reference.
ito::RetVal OphirPowermeter::getVal(void* vpdObj, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject* dObj = reinterpret_cast<ito::DataObject*>(vpdObj);

    // call retrieveData without argument. Retrieve data should then put the currently acquired
    // image into the dataObject m_data of the camera.
    retValue += retrieveData();

    if (!retValue.containsError())
    {
        if (dObj)
        {
            (*dObj) = m_data; // copy reference to externally given object
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
ito::RetVal OphirPowermeter::copyVal(void* vpdObj, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject* dObj = reinterpret_cast<ito::DataObject*>(vpdObj);

    if (!dObj)
    {
        retValue += ito::RetVal(
            ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        // this method calls retrieveData with the passed dataObject as argument such that
        // retrieveData is able to copy the image obtained by the camera directly into the given,
        // external dataObject
        retValue += retrieveData(dObj);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void OphirPowermeter::dockWidgetVisibilityChanged(bool visible)
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
            connect(this, SIGNAL(visibilityChanged(bool)), widget, SLOT(manageTimer(bool)));

            emit visibilityChanged(visible);
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit visibilityChanged(visible);
            disconnect(this, SIGNAL(visibilityChanged(bool)), widget, SLOT(manageTimer(bool)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::SendQuestionWithAnswerInt(
    QByteArray questionCommand, int& answer, int timeoutMS)
{
    int readSigns;
    bool ok;
    QByteArray _answer;
    ito::RetVal retValue = SerialSendCommand(questionCommand);
    retValue += readString(_answer, readSigns, timeoutMS);

    if (_answer[0] == '*')
    {
        _answer.remove(0, 1);
    }

    if (_answer.length() > 0)
    {
        answer = _answer.toInt(&ok);
    }
    else
    {
        std::cout << "empty answer\n" << std::endl;
        answer = 0;
        ok = true;
    }

    if (retValue.containsError() || !ok)
    {
        retValue += ito::RetVal(
            ito::retError, 0, tr("value could not be parsed to a double value").toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::SendQuestionWithAnswerDouble(
    QByteArray questionCommand, double& answer, int timeoutMS)
{
    int readSigns;
    bool ok;
    QByteArray _answer;
    ito::RetVal retValue = SerialSendCommand(questionCommand);
    retValue += readString(_answer, readSigns, timeoutMS);

    if (_answer[0] == '*')
    {
        _answer.remove(0, 1);
    }

    if (_answer.length() > 0)
    {
        answer = _answer.toDouble(&ok);
        if (!ok) // search for whitespace character
        {
            if (_answer[0] == ' ')
            {
                _answer.remove(0, 1);
            }

            answer = _answer.toDouble(&ok);
        }
    }
    else
    {
        answer = std::numeric_limits<double>::quiet_NaN();
        ok = true;
    }


    if (retValue.containsError() || !ok)
    {
        if (answer != 0.0)
        {
            retValue += ito::RetVal(
                ito::retError,
                0,
                tr("value could not be parsed to a double value").toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::SendQuestionWithAnswerString(
    QByteArray questionCommand, QByteArray& answer, int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = SerialSendCommand(questionCommand);
    retValue += readString(answer, readSigns, timeoutMS);

    if (answer[0] == '*')
    {
        answer.remove(0, 1);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::readString(QByteArray& result, int& len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    bool done = false;
    QElapsedTimer timer;
    QByteArray endline;
    int curFrom = 0;
    int pos = 0;
    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";
    timer.start();

    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retValue += m_pSer->getParam(param, NULL);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); // borrowed reference
        int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
        endline = QByteArray::fromRawData(temp, len);
        //
        // endline[0] = temp[0];
        // endline[1] = temp[1];
        // endline[2] = temp[2];
        // endline = endline.trimmed();
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("could not read endline parameter from serial port").toLatin1().data());
    }

    while (!done && !retValue.containsError())
    {
        *curBufLen = buflen;
        retValue += m_pSer->getVal(curBuf, curBufLen, NULL);

        if (!retValue.containsError())
        {
            result += QByteArray(curBuf.data(), *curBufLen);
            pos = result.indexOf(endline, curFrom);
            curFrom = qMax(0, result.length() - 3);

            if (pos >= 0) // found
            {
                done = true;
                result = result.left(pos);
            }
        }

        if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("timeout").toLatin1().data());
        }

        len = result.length();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::SerialSendCommand(QByteArray command)
{
    ito::RetVal retVal = m_pSer->setVal(command.data(), command.length(), NULL);

    if (m_delayAfterSendCommandMS > 0)
    {
        QMutex mutex;
        mutex.lock();
        QWaitCondition waitCondition;
        waitCondition.wait(&mutex, m_delayAfterSendCommandMS);
        mutex.unlock();
    }

    return retVal;
}


//----------------------------------------------------------------------------------------------------------------------------------
char* OphirPowermeter::wCharToChar(const wchar_t* input)
{
    size_t i;
    // Conversion
    wcstombs_s(&i, m_charBuffer, (size_t)BUFFER_SIZE, input, (size_t)BUFFER_SIZE);
    return m_charBuffer;
}

//----------------------------------------------------------------------------------------------------------------------------------
char* OphirPowermeter::TCharToChar(const TCHAR* message)
{
    wcstombs(m_charBuffer, message, wcslen(message) + 1);
    return m_charBuffer;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OphirPowermeter::checkError(const int& e, const char* message)
{
    switch (e)
    {
    case 0x00000000:
        return ito::retOk;
        break;
    case 0x80070057:
        return ito::RetVal::format(ito::retError, e, "%s: Invalid argument %i.", message, e);
        break;
    case 0x80040201:
        return ito::RetVal::format(ito::retError, e, "%s: Device already opened %i.", message, e);
        break;
    case 0x80040300:
        return ito::RetVal::format(ito::retError, e, "%s: Device failed %i.", message, e);
        break;
    case 0x80040301:
        return ito::RetVal::format(
            ito::retError, e, "%s: Device firmware is incorrect %i.", message, e);
        break;
    case 0x80040302:
        return ito::RetVal::format(ito::retError, e, "%s: Sensor failed %i.", message, e);
        break;
    case 0x80040303:
        return ito::RetVal::format(
            ito::retError, e, "%s: Sensor firmware is incorrect %i.", message, e);
        break;
    case 0x80040306:
        return ito::RetVal::format(
            ito::retError, e, "%s: This sensor is not supported %i.", message, e);
        break;
    case 0x80040308:
        return ito::RetVal::format(
            ito::retError, e, "%s: The device is no longer available %i.", message, e);
        break;
    case 0x80040405:
        return ito::RetVal::format(ito::retError, e, "%s: command failed %i.", message, e);
        break;
    case 0x80040501:
        return ito::RetVal::format(
            ito::retError, e, "%s: A channel is in stream mode %i.", message, e);
        break;

    default:
        return ito::RetVal::format(ito::retError, e, "%s: unknown error %i.", message, e);
    }
}
