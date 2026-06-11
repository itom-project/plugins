/* ********************************************************************
Plugin "DummyMotor" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#include "MeasurementComputing.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant

#include <QtCore/QtPlugin>
#include <qset.h>

#include "common/helperCommon.h"
#include "DataObject/dataobj.h"
#include "common/typeDefs.h"

static QSet<int> InitializedBoards = QSet<int>(); /*!< a list with all board numbers that are currently initialized. */

//----------------------------------------------------------------------------------------------------------------------------------
MeasurementComputingInterface::MeasurementComputingInterface() :
    m_numberOfInstances(0)
{
    m_type = ito::typeDataIO | ito::typeADDA;
    setObjectName(PLUGIN_NAME);
    m_description = tr("MeasurementComputing Digital-Analog Converter Plugin.");
    m_detaildescription = tr(
"The MeasurementComputing is a itom-Plugin to give a direct access to the MeasurementComputing USB digital to analog converter (e. g. USB-1208LS). \n\
\n\
Measurement Computing devices are available with analog input and output, digital I/O, counter and temperature ports. \
Before you can use this itom-Plugin, you must configure the D/A board using the software 'InstaCal'. \n\
First define a board number in the software 'InstaCal' (e.g. 0). \n\
This board number is a mandatory itom plugin initiation parameter. \n\
\n\
The analog input channels are acquire by using the **acquire**-function, then the data are returned to python by the **getVal**- or **copyVal**-function. \
Returned data are a dataobject of the size m x n, where m are the number of channels and n the number of acquired samples. \
The analog output values are set by the **setVal**-function. \
Digital I/O, counter and temperature ports can be used by the **exec**-functions, see in the description below. \
\n\
Refer to http://www.mccdaq.com/execteam.html for the names, titles, and contact information of each key executive at Measurement Computing. \n").toLatin1().data();

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param param = ito::Param("board_number", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 254, 0, tr("board number of the connected device. This number must be defined by the software 'InstaCal'").toLatin1().data());
    m_initParamsMand.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
MeasurementComputingInterface::~MeasurementComputingInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputingInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(MeasurementComputing)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputingInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(MeasurementComputing)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal MeasurementComputing::showConfDialog(void)
{
   return apiShowConfigurationDialog(this, new DialogMeasurementComputing(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
// Constructor  -  Define all parameters here
MeasurementComputing::MeasurementComputing() : AddInDataIO(),
    m_boardNum(0),
    m_devNum(0),
    m_input_range(BIP10VOLTS),
    m_output_range(UNI5VOLTS),
    m_pBuffer(NULL),
    m_dataAcquired(false),
    m_ad_resolution(0),
    m_da_resolution(0),
    m_vxdrevnum(0),
    m_revision(0)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "MeasurementComputing", tr("name of itom plugin.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("device_name", ito::ParamBase::String | ito::ParamBase::Readonly, "devices_name", tr("name of connected device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("serial_number", ito::ParamBase::String | ito::ParamBase::Readonly, NULL, tr("serial number of connected device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_input_bpp", ito::ParamBase::Int, 0, 32, 12, tr("bit resolution of the analog input.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_output_bpp", ito::ParamBase::Int, 0, 32, 12, tr("bit resolution of the analog output.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_low_input_channel", ito::ParamBase::Int, 0, 7, 0, tr("first analog input channel (See pin description of your device).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_high_input_channel", ito::ParamBase::Int, 0, 7, 7, tr("last analog input channel (See pin description of your device).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_number_inputs", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("number of input channels of this device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_low_output_channel", ito::ParamBase::Int, 0, 7, 0, tr("first analog output channel (See pin description of your device).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_high_output_channel", ito::ParamBase::Int, 0, 7, 7, tr("last analog output channel (See pin description of your device).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_number_outputs", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("number of output channels of this device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("samples_per_input_channel", ito::ParamBase::Int, 1, std::numeric_limits<int>::max(), 1, tr("number of samples that are acquired per channel after each acquisition.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("input_samples_per_second", ito::ParamBase::Int, 100, 8000, 1200, tr("analog input samples per second. The samples are distributed over all channels. The effective rate per channel is this parameter divided by the number of channels. USB1208LS: 100 Hz - 1200 Hz, for higher rates a fast acquisition with 8000 Hz is executed where only 4096 samples can be acquired in one run.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("number_temperature_channel", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("number of temperature channels of this device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("number_counter_channel", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("number of counter channels of this device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("clock_frequency", ito::ParamBase::Int, 0, 40, 0, tr("clock frequency in megahertz (MHz) (40, 10, 8, 6, 5, 4, 3, 2, 1) or 0 for not supported.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_voltage_input", ito::ParamBase::Int, 0, 1, 0, tr("if parameter is set to 1, the A/D value is returned as a voltage value.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("analog_voltage_output", ito::ParamBase::Int, 0, 1, 0, tr("if parameter is set to 1, the D/A value is set as a voltage value.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("input_range_code", ito::ParamBase::String, "BIP10VOLTS", tr("A/D range code, if board has a programmable gain. Refer to board specific information for a list of the supported A/D ranges.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("output_range_code", ito::ParamBase::String, "UNI5VOLTS", tr("D/A range code, if board has a programmable gain. Refer to board specific information for a list of the supported D/A ranges.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("digital_number_ports", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("number of digital I/O ports of this device.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("digital_port_name", ito::ParamBase::String, "FIRSTPORTA", tr("digital devices type.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("digital_port_mode", ito::ParamBase::Int, DIGITALOUT, DIGITALIN, DIGITALIN, tr("sets the digital port defined by the parameter ""digital_port_name"" as input (2) or output(1).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("temperature_scale", ito::ParamBase::String, "CELSIUS", tr("scale value of the temperature input. Coises are CELSIUS, FAHRENHEIT, KELVIN, VOLTS and NOSCALE. default = CELSIUS.").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String);
    sm->addItem("CELSIUS");
    sm->addItem("FAHRENHEIT");
    sm->addItem("KELVIN");
    sm->addItem("VOLTS");
    sm->addItem("NOSCALE");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    //register exec functions
    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    paramVal = ito::Param("voltage_input_channel", ito::ParamBase::Int | ito::ParamBase::In, 0, 8, 0, tr("voltage input channel").toLatin1().data());
    pMand.append(paramVal);
    paramVal = ito::Param("voltage_input", ito::ParamBase::Double | ito::ParamBase::Out, -100.0, 100.0, 0.0, tr("voltage value of defined input channel").toLatin1().data());
    pOut.append(paramVal);

    registerExecFunc("getVIn", pMand, pOpt, pOut, tr("reads and returns the voltage value of the specified input channel in the parameter voltage_input_channel.").toLatin1().data());
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register setVOut func
    paramVal = ito::Param("voltage_output_channel", ito::ParamBase::Int | ito::ParamBase::In, 0, 8, 0, tr("voltage output channel").toLatin1().data());
    pMand.append(paramVal);
    paramVal = ito::Param("voltage_output", ito::ParamBase::Double | ito::ParamBase::In, -100.0, 100.0, 0.0, tr("voltage value to set at the output channel").toLatin1().data());
    pMand.append(paramVal);

    registerExecFunc("setVOut", pMand, pOpt, pOut, tr("set the voltage value at the specific analog output channel defined by the parameter voltage_output_channel.").toLatin1().data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register getTIn func
    paramVal = ito::Param("temperature_input_channel", ito::ParamBase::Int | ito::ParamBase::In, 0, 8, 0, tr("temperature input channel").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("temperature_input", ito::ParamBase::Double | ito::ParamBase::Out, -100.0, 100.0, 0.0, tr("temperature value of defined input channel").toLatin1().data());
    pOut.append(paramVal);

    registerExecFunc("getTIn", pMand, pOpt, pOut, tr("reads and returns the temperature value of the specific input channel defined by the temperature_input_channel. Use the parameter temperature_scale the define the temperature scaling value.").toLatin1().data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register getCIn func
    paramVal = ito::Param("counter_input_channel", ito::ParamBase::Int | ito::ParamBase::In, 0, 8, 0, tr("counter input channel").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("counter_input", ito::ParamBase::Int | ito::ParamBase::Out, 0, std::numeric_limits<int>::max(), 0, tr("counter value of defined input channel").toLatin1().data());
    pOut.append(paramVal);

    paramVal = ito::Param("counter_set_value", ito::ParamBase::Int | ito::ParamBase::In, 0, 0, -1, tr("counter value to load into the counter's register. To reset the counter, load the value zero").toLatin1().data());
    pOpt.append(paramVal);

    registerExecFunc("getCIn", pMand, pOpt, pOut, tr("reads and returns the current count of the specified counter input channel. Use the parameter counter_set_value to reset the counter.").toLatin1().data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register getDIn func
    paramVal = ito::Param("digital_port_number", ito::ParamBase::String | ito::ParamBase::In, m_params["digital_port_name"].getVal<char*>(), tr("digital I/O port to read").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("digital_port_value", ito::ParamBase::Int | ito::ParamBase::Out, 0, 255, 0, tr("digital input value of the specific I/O port").toLatin1().data());
    pOut.append(paramVal);

    registerExecFunc("getDIn", pMand, pOpt, pOut, tr("reads the digital I/O port value. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a input port.").toLatin1().data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register setDOut func
    paramVal = ito::Param("digital_port_number", ito::ParamBase::String | ito::ParamBase::In, m_params["digital_port_name"].getVal<char*>(), tr("digital I/O port to read").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("digital_port_value", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("digital output value of the specific I/O port").toLatin1().data());
    pMand.append(paramVal);

    registerExecFunc("setDOut", pMand, pOpt, pOut, tr("sets the digital I/O port value. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a output port.").toLatin1().data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register getBitIn func
    paramVal = ito::Param("digital_port_number", ito::ParamBase::String | ito::ParamBase::In, m_params["digital_port_name"].getVal<char*>(), tr("digital I/O port to read").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("digital_port_bit_number", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("digital port bit number of the specific I/O port").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("digital_port_value", ito::ParamBase::Int | ito::ParamBase::Out, 0, 255, 0, tr("digital input value of the specific I/O port-bit").toLatin1().data());
    pOut.append(paramVal);

    registerExecFunc("getBitIn", pMand, pOpt, pOut, tr("reads a single bit of the specified I/O port. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a input port.").toLatin1().data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //register setBitOut func
    paramVal = ito::Param("digital_port_number", ito::ParamBase::String | ito::ParamBase::In, m_params["digital_port_name"].getVal<char*>(), tr("digital I/O port to read").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("digital_port_bit_number", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("digital port bit number of the specific I/O port").toLatin1().data());
    pMand.append(paramVal);

    paramVal = ito::Param("digital_port_value", ito::ParamBase::Int | ito::ParamBase::In, 0, 255, 0, tr("digital output value of the specific I/O port-bit").toLatin1().data());
    pMand.append(paramVal);

    registerExecFunc("setBitOut", pMand, pOpt, pOut, tr("sets a single bit of the specified I/O port. Use the parameters digital_port_name to define the port you want to use. Use the parameter digital_port_mode to define the port as a output port.").toLatin1().data());

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    //since the cbw32.dll or cbw64.dll are delay loaded, their first startup at the initialization of the first instance
    //will need a lot of time such that timeouts may occur when calling the first cb... command in init. Therefore the
    //following dummy command from the library is called here (in the thread of the caller without timeout constraints).
    cbGetRevision(&m_revision, &m_vxdrevnum);
}

//----------------------------------------------------------------------------------------------------------------------------------
MeasurementComputing::~MeasurementComputing()
{
    m_params.clear();

    cbStopBackground(m_boardNum, AIFUNCTION);

    //delete buffer if not yet done
    if (m_pBuffer != NULL)
    {
        cbWinBufFree(m_pBuffer);
        m_pBuffer = NULL;
    }

    if (m_pOutputBuffer != NULL)
    {
        cbWinBufFree(m_pOutputBuffer);
        m_pOutputBuffer = NULL;
    }
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QMap<QString, ito::Param>::iterator it;
    ito::RetVal retValue(ito::retOk);

    m_boardNum = paramsMand->at(0).getVal<int>();

    if (InitializedBoards.contains(m_boardNum))
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("Instance with board number %i already initialized").toLatin1().data(), m_boardNum);
    }
    else
    {
        InitializedBoards.insert(m_boardNum);
    }

    if (!retValue.containsError())
    {
        if (int ret = cbDeclareRevision(&m_revision))
        {
            retValue += getErrStr(ret, QString::number(m_revision));
        }
    }

    if (!retValue.containsError())
    {
        it = m_params.find("digital_port_name");
        int type;
        if (int ret = cbGetConfig(DIGITALINFO, m_boardNum, m_devNum, DIDEVTYPE, &type))
        {
            retValue += getErrStr(ret, QString::number(type));
        }
        it->setVal<char*>(digDevTypeIntToString(type).toLocal8Bit().data());
    }

    if(!retValue.containsError())
    {
        it = m_params.find("digital_port_mode");
        if(int ret = cbDConfigPort(m_boardNum, digDevTypeStringToInt(m_params["digital_port_name"].getVal<char*>()), it->getVal<int>()))
        {
            retValue += getErrStr(ret, QString::number(it->getVal<int>()));
        }

    }

    if (!retValue.containsError())
    {
        // get board name
        it = m_params.find("device_name");

        char name[BOARDNAMELEN];

        if (int ret = cbGetBoardName(m_boardNum, name))
        {
            retValue += getErrStr(ret, name);
        }

        it->setVal<char*>(name);
    }

    if (!retValue.containsError())
    {
        // factory serial number
        it = m_params.find("serial_number");
        int serial;
        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BIFACTORYID, &serial))
        {
            retValue += getErrStr(ret, QString::number(serial));
        }
        it->setVal<char*>(QString::number(serial).toLatin1().data());
    }

    if (!retValue.containsError())// init analog inputs
    {
        it = m_params.find("analog_number_inputs");
        int number;
        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BINUMADCHANS, &number))
        {
            retValue += getErrStr(ret, QString::number(number));
        }

        it->setMeta(new ito::IntMeta(number, number, 1), true);
        it->setVal<int>(number);

        it = m_params.find("analog_low_input_channel");
        it->setMeta(new ito::IntMeta(0, number - 1, 1), true);
        it->setVal<int>(0);

        it = m_params.find("analog_high_input_channel");
        it->setMeta(new ito::IntMeta(0, number - 1, 1), true);
        it->setVal<int>(number - 1);
    }

    if (!retValue.containsError())// init analog outputs
    {
        it = m_params.find("analog_number_outputs");
        int number;
        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BINUMDACHANS, &number))
        {
            retValue += getErrStr(ret, QString::number(number));
        }

        it->setMeta(new ito::IntMeta(number, number, 1), true);
        it->setVal<int>(number);

        it = m_params.find("analog_low_output_channel");
        it->setMeta(new ito::IntMeta(0, number - 1, 1), true);
        it->setVal<int>(0);

        it = m_params.find("analog_high_output_channel");
        it->setMeta(new ito::IntMeta(0, number - 1, 1), true);
        it->setVal<int>(number - 1);
    }

    if (!retValue.containsError())//init digital IO ports
    {
        it = m_params.find("digital_number_ports");
        int number;

        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BINUMIOPORTS, &number))
        {
            retValue += getErrStr(ret, QString::number(number));
        }

        it->setMeta(new ito::IntMeta(number, number, 1), true);
        it->setVal(number);
    }

    if (!retValue.containsError())// init temperature inputs
    {
        it = m_params.find("number_temperature_channel");
        int number;

        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BINUMTEMPCHANS, &number))
        {
            retValue += getErrStr(ret, QString::number(number));
        }

        it->setMeta(new ito::IntMeta(number, number, 1), true);
        it->setVal(number);
    }

    if (!retValue.containsError())// init counter inputs
    {
        it = m_params.find("number_counter_channel");
        int number;

        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BICINUMDEVS, &number))
        {
            retValue += getErrStr(ret, QString::number(number));
        }

        it->setMeta(new ito::IntMeta(number, number, 1), true);
        it->setVal(number);
    }

    if (!retValue.containsError())// init clock frequency
    {
        it = m_params.find("clock_frequency");
        int number;

        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BICLOCK, &number))
        {
            retValue += getErrStr(ret, QString::number(number));
        }

        it->setMeta(new ito::IntMeta(0, number, 1), true);
        it->setVal(number);
    }

    if (!retValue.containsError())
    {
        it = m_params.find("analog_output_bpp"); //default bpp = 10
        /* Get the resolution of D/A */
        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BIDACRES, &m_da_resolution))
        {
            retValue += getErrStr(ret, QString::number(m_da_resolution));
        }

        it->setMeta(new ito::IntMeta(m_da_resolution, m_da_resolution, 1), true);
        it->setVal<int>(m_da_resolution);
        it->setFlags(ito::ParamBase::Readonly);
    }

    if (!retValue.containsError())
    {
        it = m_params.find("analog_input_bpp"); //default bpp = 12
        /* Get the resolution of A/D */
        if (int ret = cbGetConfig(BOARDINFO, m_boardNum, m_devNum, BIADRES, &m_ad_resolution))
        {
            retValue += getErrStr(ret, QString::number(m_ad_resolution));
        }

        it->setMeta(new ito::IntMeta(m_ad_resolution,m_ad_resolution, 1), true);
        it->setVal<int>(m_ad_resolution);
        it->setFlags(ito::ParamBase::Readonly);
    }

    if (!retValue.containsError())
    {
        //successful initialization will flash the LED on device.
        if (int ret = cbFlashLED(m_boardNum))
        {
            retValue += getErrStr(ret, QString::number(0));
        }
    }

    // synchronize settings of plugin
    retValue += synchronizeSettings(sAll);

    if (!retValue.containsError())
    {    // emit signal about changed parameters
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retValue)

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    InitializedBoards.remove(m_boardNum);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
//! sets parameter of m_params with key name.
/*!
    This method copies the given value  to the m_params-parameter.

    \param [in] name is the key name of the parameter
    \param [in] val is the double value to set.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal MeasurementComputing::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //if you program for itom 1.4.0 or higher (Interface version >= 1.3.1) you should use this
        //API method instead of the one above: The difference is, that incoming parameters that are
        //compatible but do not have the same type than the corresponding m_params value are cast
        //to the type of the internal parameter and incoming double values are rounded to the
        //next value (depending on a possible step size, if different than 0.0)
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        if (key == "input_range_code")
        {
            m_input_range = rangeCodeStringToInt(val->getVal<char*>());
            retValue += it->copyValueFrom(&(*val));
            retValue += synchronizeSettings(sMData);
        }

        else if(key == "output_range_code")
        {
            m_output_range = rangeCodeStringToInt(val->getVal<char*>());
            retValue += it->copyValueFrom(&(*val));
            retValue += synchronizeSettings(sMData);
        }

        else if (key == "temperature_scale")
        {
            m_temperature_scale = tempScaleStringToInt(val->getVal<char*>());
            retValue += it->copyValueFrom(&(*val));
        }

        else if (key == "analog_low_input_channel" || \
                 key == "analog_high_input_channel" || \
                 key == "samples_per_input_channel" || \
                 key == "analog_voltage_input")
        {
            retValue += it->copyValueFrom(&(*val));
            retValue += synchronizeSettings(sMData);
        }

        else if (key == "analog_high_output_channel" || \
                 key == "analog_low_output_channel" || \
                 key == "samples_per_output_channel" || \
                 key == "analog_voltage_output")
        {
            retValue += it->copyValueFrom(&(*val));
            retValue += synchronizeSettings(sOutputData);
        }

        else if (key == "digital_port_mode")
        {
            if (int ret = cbDConfigPort(m_boardNum, digDevTypeStringToInt(m_params["digital_port_name"].getVal<char*>()), val->getVal<int>()))
            {
                retValue += getErrStr(ret, "error during stetting digital port direstion.");
            }
            retValue += it->copyValueFrom(&(*val));
        }

        else if (key == "clock_frequency")
        {
            if (int ret = cbSetConfig(BOARDINFO, m_boardNum, m_devNum, BICLOCK, val->getVal<int>()))
            {
                retValue += getErrStr(ret, "error during stetting clock frequency.");
            }
            retValue += it->copyValueFrom(&(*val));
        }

        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
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
//! init method which is called by the addInManager after the initiation of a new instance of MeasurementComputing.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::startDevice(ItomSharedSemaphore *waitCond)
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
ito::RetVal MeasurementComputing::stopDevice(ItomSharedSemaphore *waitCond)
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
ito::RetVal MeasurementComputing::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;

    int lowChannel = m_params["analog_low_input_channel"].getVal<int>();
    int highChannel = m_params["analog_high_input_channel"].getVal<int>();
    long count = m_numInputSamples * m_numInputChannels;
    long rate = m_params["input_samples_per_second"].getVal<int>();
    int voltageInput = m_params["analog_voltage_input"].getVal<int>();
    int ret;

    m_acquisitionResult = ito::retOk;

    unsigned int Options = BACKGROUND | CONVERTDATA;
    if (rate > 1200)
    {
        rate = 8000 / m_numInputChannels;
        Options |= BURSTIO;

        if (count > 4096)
        {
            retVal += ito::RetVal(ito::retError, 0, "For a high-speed sampling rate of > 1200 Hz, only 4096 samples can be acquired in total");
        }
    }
    else
    {
        rate /= m_numInputChannels;
    }

    if (ret = cbAInScan(m_boardNum, lowChannel, highChannel, count, &rate, m_input_range, m_pBuffer, Options))
    {
        retVal += getErrStr(ret, "Error during acquisition!");
    }


    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    if (!retVal.containsError())
    {
        //wait until acquisition has been finished
        SHORT status;
        LONG curCount, curIndex;
        bool done = false;

        while (!done)
        {
            if (ret = cbGetStatus(m_boardNum, &status, &curCount, &curIndex, AIFUNCTION))
            {
                retVal += getErrStr(ret, "Error during status request!");
                done = true;
                cbStopBackground(m_boardNum, AIFUNCTION);
            }
            else if (status == 0)
            {
                done = true;
                m_dataAcquired = true;

                if (m_ad_resolution <= 16 && !voltageInput)
                {
                    //copy buffer to m_data
                    ito::uint16 *channelPtr = m_data.rowPtr<ito::uint16>(0,0);

                    for (int channel = 0; channel < m_numInputChannels; ++channel)
                    {
                        for (int sample = 0; sample < m_numInputSamples; ++ sample)
                        {
                            *channelPtr = ((ito::uint16*)m_pBuffer)[sample * m_numInputChannels + channel];
                            channelPtr++;
                        }
                    }
                }
                else if (m_ad_resolution <= 32 && !voltageInput)
                {
                    //copy buffer to m_data
                    ito::uint32 *channelPtr = m_data.rowPtr<ito::uint32>(0,0);

                    for (int channel = 0; channel < m_numInputChannels; ++channel)
                    {
                        for (int sample = 0; sample < m_numInputSamples; ++ sample)
                        {
                            *channelPtr = ((ito::uint32*)m_pBuffer)[sample * m_numInputChannels + channel];
                            channelPtr++;
                        }
                    }
                }
                else if (m_ad_resolution <= 16 && voltageInput)
                {
                    //copy buffer to m_data
                    ito::float64 *channelPtr = m_data.rowPtr<ito::float64>(0,0);

                    for (int channel = 0; channel < m_numInputChannels; ++channel)
                    {
                        for (int sample = 0; sample < m_numInputSamples; ++ sample)
                        {
                            //*channelPtr = calcInputVoltage(((ito::uint16*)m_pBuffer)[sample * m_numInputChannels + channel], scalingFactor, scalingOffset);
                            if (int ret = cbToEngUnits32(m_boardNum, m_input_range, ((ito::uint16*)m_pBuffer)[sample * m_numInputChannels + channel], channelPtr))
                            {
                                retVal += getErrStr(ret, "Error during datavalue conversion");
                            }
                            channelPtr++;
                        }
                    }
                }
                else if (m_ad_resolution <= 32 && voltageInput)
                {
                    //copy buffer to m_data
                    ito::float64 *channelPtr = m_data.rowPtr<ito::float64>(0,0);

                    for (int channel = 0; channel < m_numInputChannels; ++channel)
                    {
                        for (int sample = 0; sample < m_numInputSamples; ++ sample)
                        {
                            //*channelPtr = calcInputVoltage32(((ito::uint32*)m_pBuffer)[sample * m_numInputChannels + channel], scalingFactor, scalingOffset);
                            if (int ret = cbToEngUnits32(m_boardNum, m_input_range, ((ito::uint32*)m_pBuffer)[sample * m_numInputChannels + channel], channelPtr))
                            {
                                retVal += getErrStr(ret, "Error during datavalue conversion");
                            }

                            channelPtr++;
                        }
                    }
                }

                if (voltageInput)
                {
                    m_data.setValueDescription("voltage");
                    m_data.setValueUnit("V");
                }
                m_data.setAxisDescription(0, "A/D channel");
                m_data.setAxisDescription(1, "number of sample");
                m_data.setTag("analog_input_rate", rate); //set a dataobject tag with the analog input sampling rate
            }
        }

        m_acquisitionResult = retVal;
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getVal(void *data, ItomSharedSemaphore *waitCond = NULL)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = m_acquisitionResult;
    ito::DataObject *dataObj = reinterpret_cast<ito::DataObject *>(data);

    if (!m_dataAcquired)
    {
        retval += ito::RetVal(ito::retError, 0, tr("No data acquired").toLatin1().data());
    }

    if (!retval.containsError())
    {
        if (dataObj == NULL)
        {
            retval += ito::RetVal(ito::retError, 0, tr("DataObject was NULL-Pointer").toLatin1().data());
        }
        else
        {
            *dataObj = m_data;
        }
    }

    m_dataAcquired = false;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
};

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::copyVal(void *dObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = m_acquisitionResult;
    ito::DataObject *dataObj = reinterpret_cast<ito::DataObject *>(dObj);

    if (!m_dataAcquired)
    {
        retval += ito::RetVal(ito::retError, 0, tr("No data acquired").toLatin1().data());
    }

    if (!retval.containsError())
    {
        if (dataObj == NULL)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
        }
        else if (dataObj->getDims() == 0)
        {
            //create new object and put the buffer data inside
            retval += m_data.copyTo(*dataObj);
        }
        else if (m_ad_resolution <= 16 && dataObj->getType() == ito::tUInt16 && dataObj->getSize(0) == m_numInputChannels && dataObj->getSize(1) == m_numInputSamples)
        {
            //copy buffer into given data object
            m_data.deepCopyPartial(*dataObj);
        }
        else if (m_ad_resolution <= 32 && dataObj->getType() == ito::tUInt32 && dataObj->getSize(0) == m_numInputChannels && dataObj->getSize(1) == m_numInputSamples)
        {
            //copy buffer into given data object
            m_data.deepCopyPartial(*dataObj);
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("Given dataObject has an invalid type (uint16 required) or size (channels x samples).").toLatin1().data());
        }
    }

    m_dataAcquired = false;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
};

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::setVal(const char *dObj, const int length, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    const ito::DataObject *dataObj = reinterpret_cast<const ito::DataObject*>(dObj);

    int lowChannel = m_params["analog_low_output_channel"].getVal<int>();
    int highChannel = m_params["analog_high_output_channel"].getVal<int>();
    int voltageOutput = m_params["analog_voltage_output"].getVal<int>();
    int numChannels = highChannel - lowChannel + 1;
    int numSamples = 1;
    int inputDataType = dataObj->getType();

    long count = numChannels * numSamples;
    long rate = NOTUSED;
    int ret;

    ito::int16 m_pOutputBuffer[64];

    unsigned int Options = SIMULTANEOUS | BACKGROUND | SCALEDATA;

    if (voltageOutput) //Check dataobject datatype
    {
        if (inputDataType != ito::tFloat32 && inputDataType != ito::tFloat64)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("If analog voltage output is used, dataobject datatype must be float32 or float64.").toLatin1().data());
        }
    }
    else
    {
        if ((inputDataType != ito::tInt16 && m_da_resolution <=12) || (inputDataType != ito::tInt32 && m_da_resolution > 12))
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Wrong dataobject datatype.").toLatin1().data());
        }
    }

    if ((dataObj->getSize(0) != numChannels) && (dataObj->getSize(1) != numSamples)) //check if dimensions are right
    {
        retValue += ito::RetVal(ito::retError, 0, tr("DataObject has wrong size. Size must be numberOfChannels x numberOfSamples.").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        if (voltageOutput && (inputDataType == ito::tFloat32))
        {
            int cnt = 0;
            for (int channel = 0; channel < numChannels; channel++)
            {
                for (int sample = 0; sample < numSamples; sample++)
                {
                    USHORT dataval;
                    float val = dataObj->at<ito::float32>(channel,sample);
                    if (int ret = cbFromEngUnits(m_boardNum, m_output_range, val, &dataval))
                    {
                        retValue += getErrStr(ret, "Error during datavalue conversion");
                    }
                    m_pOutputBuffer[cnt] = *reinterpret_cast<unsigned short *>(&dataval);

                    cnt++;
                }
            }
        }
        else if(voltageOutput && (inputDataType == ito::tFloat64))
        {
            int cnt = 0;
            for (int channel = 0; channel < numChannels; channel++)
            {
                for (int sample = 0; sample < numSamples; sample++)
                {
                    USHORT dataval;
                    float val = dataObj->at<ito::float64>(channel,sample);
                    if (int ret = cbFromEngUnits(m_boardNum, m_output_range, val, &dataval))
                    {
                        retValue += getErrStr(ret, "Error during datavalue conversion");
                    }
                    m_pOutputBuffer[cnt] = *reinterpret_cast<unsigned short *>(&dataval);
                    cnt++;
                }
            }
        }
        else
        {
            int cnt = 0;
            for (int channel = 0; channel < numChannels; channel++)
            {
                for (int sample = 0; sample < numSamples; sample++)
                {
                    m_pOutputBuffer[cnt] = dataObj->at<ito::int16>(channel,sample);
                    cnt++;
                }
            }
        }

    }

    if (!retValue.containsError())
    {
        // the range of the analog output is not programmable, fixed to UNI5VOLTS (0 to 5 volts) (USB1208LS).
        // all rate settings are ignored
        if (ret = cbAOutScan(m_boardNum, lowChannel, highChannel, count, &rate, m_output_range, m_pOutputBuffer, Options))
        {
            retValue += getErrStr(ret, "Error during setting of analog output channel.");
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//--------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = NULL;
    ito::ParamBase *param2 = NULL;
    ito::ParamBase *param3 = NULL;

    if(funcName == "getVIn")
    {
        param1 = ito::getParamByName(&(*paramsMand), "voltage_input_channel", &retValue);
        param2 = ito::getParamByName(&(*paramsOut), "voltage_input", &retValue);

        if(!retValue.containsError())
        {
            retValue += MeasurementComputing::getVIn( *param1, *param2);
        }
    }
    else if(funcName == "setVOut")
    {
        param1 = ito::getParamByName(&(*paramsMand), "voltage_output_channel", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "voltage_output", &retValue);

        if(!retValue.containsError())
        {
            retValue += MeasurementComputing::setVOut( *param1, *param2);
        }
    }
    else if(funcName == "getTIn")
    {
        param1 = ito::getParamByName(&(*paramsMand), "temperature_input_channel", &retValue);
        param2 = ito::getParamByName(&(*paramsOut), "temperature_input", &retValue);

        if (!retValue.containsError())
        {
            retValue += MeasurementComputing::getTIn(*param1, *param2);
        }
    }
    else if(funcName == "getCIn")
    {
        param1 = ito::getParamByName(&(*paramsMand), "counter_input_channel", &retValue);
        param2 = ito::getParamByName(&(*paramsOpt), "counter_set_value", &retValue);
        param3 = ito::getParamByName(&(*paramsOut), "counter_input", &retValue);

        if (!retValue.containsError())
        {
            retValue += MeasurementComputing::getCIn(*param1, *param2, *param3);
        }
    }
    else if(funcName == "getDIn")
    {
        param1 = ito::getParamByName(&(*paramsMand), "digital_port_number", &retValue);
        param2 = ito::getParamByName(&(*paramsOut), "digital_port_value", &retValue);

        if (!retValue.containsError())
        {
            retValue += MeasurementComputing::getDIn(*param1, *param2);
        }
    }
    else if(funcName == "setDOut")
    {
        param1 = ito::getParamByName(&(*paramsMand), "digital_port_number", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "digital_port_value", &retValue);

        if (!retValue.containsError())
        {
            retValue += MeasurementComputing::setDOut(*param1, *param2);
        }
    }
    else if(funcName == "getBitIn")
    {
        param1 = ito::getParamByName(&(*paramsMand), "digital_port_number", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "digital_port_bit_number", &retValue);
        param3 = ito::getParamByName(&(*paramsOut), "digital_port_value", &retValue);

        if (!retValue.containsError())
        {
            retValue += MeasurementComputing::getBitIn(*param1, *param2, *param3);
        }
    }
    else if(funcName == "setBitOut")
    {
        param1 = ito::getParamByName(&(*paramsMand), "digital_port_number", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "digital_port_bit_number", &retValue);
        param3 = ito::getParamByName(&(*paramsMand), "digital_port_value", &retValue);

        if (!retValue.containsError())
        {
            retValue += MeasurementComputing::setBitOut(*param1, *param2, *param3);
        }
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError,0,tr("function name '%s' does not exist").toLatin1().data(), funcName.toLatin1().data());
    }

    if(waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::setBitOut(ito::ParamBase &port, ito::ParamBase &bit, ito::ParamBase &value)
{
    ito::RetVal retValue(ito::retOk);

    if (int ret = cbDBitOut(m_boardNum, digDevTypeStringToInt(port.getVal<char*>()), bit.getVal<int>(), value.getVal<int>()))
    {
        retValue += getErrStr(ret, "error while setting digital output value.");
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getBitIn(ito::ParamBase &port, ito::ParamBase &bit, ito::ParamBase &value)
{
    ito::RetVal retValue(ito::retOk);
    USHORT val;

    if (int ret = cbDBitIn(m_boardNum, digDevTypeStringToInt(port.getVal<char*>()), bit.getVal<int>(), &val))
    {
        retValue += getErrStr(ret, "error while getting digital input value.");
    }

    value.setVal<int>((USHORT)val);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::setDOut(ito::ParamBase &digitalPort, ito::ParamBase &digitalValue)
{
    ito::RetVal retValue(ito::retOk);

    if (int ret = cbDOut(m_boardNum, digDevTypeStringToInt(digitalPort.getVal<char*>()), digitalValue.getVal<int>()))
    {
        retValue += getErrStr(ret, "error while setting digital output value.");
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getDIn(ito::ParamBase &digitalPort, ito::ParamBase &digitalValue)
{
    ito::RetVal retValue(ito::retOk);

    USHORT val;
    int port = digDevTypeStringToInt(digitalPort.getVal<char*>());

    if (int ret = cbDIn(m_boardNum, port, &val))
    {
        retValue += getErrStr(ret, "error while getting digital input value.");
    }

    digitalValue.setVal<int>((int)val);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getCIn(ito::ParamBase &channel, ito::ParamBase &counterSet, ito::ParamBase &counter)
{
    ito::RetVal retValue(ito::retOk);

    unsigned long val;
    int chan = channel.getVal<int>();

    //int test = counterSet.getVal<int>();
    if (counterSet.getVal<int>() != -1)
    {
        if (int ret = cbCLoad(m_boardNum, chan, (unsigned int)counterSet.getVal<int>()))
        {
            retValue += getErrStr(ret, "error while setting counter value.");
        }
    }

    if (int ret = cbCIn32(m_boardNum, chan, &val))
    {
        retValue += getErrStr(ret, "error while getting counter input value.");
    }

    counter.setVal<int>((int)val);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getTIn(ito::ParamBase &channel, ito::ParamBase &temperature)
{
    ito::RetVal retValue(ito::retOk);

    int Options = FILTER; //a smoothing function is applied to temperature readings. When selected 10 samples are read from the specified channel and averaged.
    float val = -1;
    int chan = channel.getVal<int>();

    if (int ret = cbTIn(m_boardNum, chan, m_temperature_scale, &val, Options))
    {
        retValue += getErrStr(ret, "error while getting temperature input value.");
    }

    temperature.setVal<double>((double)val);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::setVOut(ito::ParamBase &channel, ito::ParamBase &voltage)
{
    ito::RetVal retValue(ito::retOk);

    int Options = 0;
    float val = voltage.getVal<double>();
    int chan = channel.getVal<int>();

    if (int ret = cbVOut(m_boardNum, chan, m_output_range, val, Options))
    {
        retValue += getErrStr(ret, "error while setting voltage output value.");
    }

    voltage.setVal<double>((double)val);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getVIn(ito::ParamBase &channel, ito::ParamBase &voltage)
{
    ito::RetVal retValue(ito::retOk);

    int Options = 0;
    float val;
    int chan = channel.getVal<int>();

    if (int ret = cbVIn(m_boardNum, chan, m_input_range, &val, Options))
    {
        retValue += getErrStr(ret, "error while getting voltage input value.");
    }

    voltage.setVal<double>((double)val);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::getErrStr(const int error, const QString &value)
{
    char message[ERRSTRLEN];

    int err = cbGetErrMsg(error, message);
    return ito::RetVal(ito::retError, error, tr("MeasurementComputing error message: %1 (value %2)").arg(message).arg(value).toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MeasurementComputing::synchronizeSettings(int what)
{
    if (what & sMData)
    {
        int lowChannel = m_params["analog_low_input_channel"].getVal<int>();
        int highChannel = m_params["analog_high_input_channel"].getVal<int>();
        int voltageInput = m_params["analog_voltage_input"].getVal<int>();
        if (lowChannel > highChannel)
        {
            std::swap(lowChannel, highChannel);
        }

        m_numInputSamples = m_params["samples_per_input_channel"].getVal<int>();
        m_numInputChannels = highChannel - lowChannel + 1;

        if (m_pBuffer != NULL)
        {
            cbWinBufFree(m_pBuffer);
            m_pBuffer = NULL;
        }

        if (m_ad_resolution <= 16 && !voltageInput)
        {
            m_data = ito::DataObject(m_numInputChannels, m_numInputSamples, ito::tUInt16);

            //alloc new output buffer (uint16)
            m_pBuffer = cbWinBufAlloc(m_numInputChannels * m_numInputSamples);
        }
        else if (m_ad_resolution <= 32 && !voltageInput)
        {
            m_data = ito::DataObject(m_numInputChannels, m_numInputSamples, ito::tUInt32);

            //alloc new output buffer (uint32)
            m_pBuffer = cbWinBufAlloc32(m_numInputChannels * m_numInputSamples);
        }
        else if (m_ad_resolution <= 16 && voltageInput)
        {
            m_data = ito::DataObject(m_numInputChannels, m_numInputSamples, ito::tFloat64);

            //alloc new output buffer (float64)
            m_pBuffer = cbWinBufAlloc(m_numInputChannels * m_numInputSamples);
        }
        else if (m_ad_resolution <= 32 && voltageInput)
        {
            m_data = ito::DataObject(m_numInputChannels, m_numInputSamples, ito::tFloat64);

            //alloc new output buffer (float64)
            m_pBuffer = cbWinBufAlloc32(m_numInputChannels * m_numInputSamples);
        }

    }
    else if (what & sOutputData)
    {
        int lowChannel = m_params["analog_low_output_channel"].getVal<int>();
        int highChannel = m_params["analog_high_output_channel"].getVal<int>();
        int voltageOutput = m_params["analog_voltage_output"].getVal<int>();

        if (lowChannel > highChannel)
        {
            std::swap(lowChannel, highChannel);
        }
        int OutputSamples = 1;
        int OutputChannels = highChannel - lowChannel + 1;

        if (m_pOutputBuffer != NULL)
        {
            cbWinBufFree(m_pOutputBuffer);
            m_pOutputBuffer = NULL;
        }

        if (m_ad_resolution <= 16 && !voltageOutput)
        {
            //alloc new output buffer (uint16)
            m_pOutputBuffer = cbWinBufAlloc(OutputChannels * OutputSamples);
        }
        else if (m_ad_resolution <= 32 || voltageOutput)
        {
            //alloc new output buffer (uint32)
            m_pOutputBuffer = cbWinBufAlloc32(OutputChannels * OutputSamples);
        }
        else if (m_ad_resolution <= 16 && voltageOutput)
        {
            //alloc new output buffer (uint32)
            m_pOutputBuffer = cbWinBufAlloc(OutputChannels * OutputSamples);
        }
        else if (m_ad_resolution <= 32 && voltageOutput)
        {
            //alloc new output buffer (uint32)
            m_pOutputBuffer = cbWinBufAlloc32(OutputChannels * OutputSamples);
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MeasurementComputing::rangeCodeIntToString(int rangeCodeInt)
{
    QString rangeCodeString;

    switch (rangeCodeInt)
    {
    case 20:  //BIP60VOLTS
        rangeCodeString = "BIP60VOLTS";
        break;
    case 23: //BIP30VOLTS
        rangeCodeString = "BIP30VOLTS";
        break;
    case 15: //BIP20VOLTS
        rangeCodeString = "BIP20VOLTS";
        break;
    case 21: //BIP15VOLTS
        rangeCodeString = "BIP15VOLTS";
        break;
    case 1: //BIP10VOLTS
        rangeCodeString = "BIP10VOLTS";
        break;
    case 0: //BIP5VOLTS
        rangeCodeString = "BIP5VOLTS";
        break;
    case 16: //BIP4VOLTS
        rangeCodeString = "BIP4VOLTS";
        break;
    case 2: //BIP2PT5VOLTS
        rangeCodeString = "BIP2PT5VOLTS";
        break;
    case 14: //BIP2VOLTS
        rangeCodeString = "BIP2VOLTS";
        break;
    case 3: //BIP1PT25VOLTS
        rangeCodeString = "BIP1PT25VOLTS";
        break;
    case 4: //BIP1VOLTS
        rangeCodeString = "BIP1VOLTS";
        break;
    case 5: //BIPPT625VOLTS
        rangeCodeString = "BIPPT625VOLTS";
        break;
    case 6: //BIPPT5VOLTS
        rangeCodeString = "BIPPT5VOLTS";
        break;
    case 12: //BIPPT25VOLTS
        rangeCodeString = "BIPPT25VOLTS";
        break;
    case 13: //BIPPT2VOLTS
        rangeCodeString = "BIPPT2VOLTS";
        break;
    case 7: //BIPPT1VOLTS
        rangeCodeString = "BIPPT1VOLTS";
        break;
    case 8: //BIPPT05VOLTS
        rangeCodeString = "BIPPT05VOLTS";
        break;
    case 9: //BIPPT01VOLTS
        rangeCodeString = "BIPPT01VOLTS";
        break;
    case 10: //BIPPT005VOLTS
        rangeCodeString = "BIPPT005VOLTS";
        break;
    case 11: //BIP1PT67VOLTS
        rangeCodeString = "BIP1PT67VOLTS";
        break;
    case 17: //BIPPT312VOLTS
        rangeCodeString = "BIPPT312VOLTS";
        break;
    case 18: //BIPPT156VOLTS
        rangeCodeString = "BIPPT156VOLTS";
        break;
    case 22: //BIPPT125VOLTS
        rangeCodeString = "BIPPT125VOLTS";
        break;
    case 19: //BIPPT078VOLTS
        rangeCodeString = "BIPPT078VOLTS";
        break;
    case 100: //UNI10VOLTS
        rangeCodeString = "UNI10VOLTS";
        break;
    case 101: //UNI5VOLTS
        rangeCodeString = "UNI5VOLTS";
        break;
    case 114: //UNI4VOLTS
        rangeCodeString = "UNI4VOLTS";
        break;
    case 102: //UNI2PT5VOLTS
        rangeCodeString = "UNI2PT5VOLTS";
        break;
    case 103: //UNI2VOLTS
        rangeCodeString = "UNI2VOLTS";
        break;
    case 109: //UNI1PT67VOLTS
        rangeCodeString = "UNI1PT67VOLTS";
        break;
    case 104:  //UNI1PT25VOLTS
        rangeCodeString = "UNI1PT25VOLTS";
        break;
    case 105: //UNI1VOLTS
        rangeCodeString = "UNI1VOLTS";
        break;
    case 110: //UNIPT5VOLTS
        rangeCodeString = "UNIPT5VOLTS";
        break;
    case 111: //UNIPT25VOLTS
        rangeCodeString = "UNIPT25VOLTS";
        break;
    case 112: //UNIPT2VOLTS
        rangeCodeString = "UNIPT2VOLTS";
        break;
    case 106: //UNIPT1VOLTS
        rangeCodeString = "UNIPT1VOLTS";
        break;
    case 113: //UNIPT05VOLTS
        rangeCodeString = "UNIPT05VOLTS";
        break;
    case 108: //UNIPT02VOLTS
        rangeCodeString = "UNIPT02VOLTS";
        break;
    case 107: //UNIPT01VOLTS
        rangeCodeString = "UNIPT01VOLTS";
        break;
    case 200: //MA4TO20
        rangeCodeString = "MA4TO20";
        break;
    case 201:  //MA2TO10
        rangeCodeString = "MA2TO10";
        break;
    case 202: //MA1TO5
        rangeCodeString = "MA1TO5";
        break;
    case 203: //MAPT5TO2PT5
        rangeCodeString = "MAPT5TO2PT5";
        break;
    case 204: //MA0TO20
        rangeCodeString = "MA0TO20";
        break;
    case 205: //BIPPT025AMPS
        rangeCodeString = "BIPPT025AMPS";
        break;
    case 400: //BIPPT025VOLTSPERVOLT
        rangeCodeString = "BIPPT025VOLTSPERVOLT";
        break;
    default:
        rangeCodeString = "";
        break;
    }

    return rangeCodeString;
}
//----------------------------------------------------------------------------------------------------------------------------------
int MeasurementComputing::rangeCodeStringToInt(char* rangeCodeString)
{
    int rangeCodeInt;

    if (!strcmp(rangeCodeString, "BIP60VOLTS"))
    {
        rangeCodeInt = BIP60VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP30VOLTS"))
    {
        rangeCodeInt = BIP30VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP20VOLTS"))
    {
        rangeCodeInt = BIP20VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP15VOLTS"))
    {
        rangeCodeInt = BIP15VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP10VOLTS"))
    {
        rangeCodeInt = BIP10VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP5VOLTS"))
    {
        rangeCodeInt = BIP5VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP4VOLTS"))
    {
        rangeCodeInt = BIP4VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP2PT5VOLTS"))
    {
        rangeCodeInt = BIP2PT5VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP2VOLTS"))
    {
        rangeCodeInt = BIP2VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP1PT25VOLTS"))
    {
        rangeCodeInt = BIP1PT25VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP1VOLTS"))
    {
        rangeCodeInt = BIP1VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT625VOLTS"))
    {
        rangeCodeInt = BIPPT625VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT5VOLTS"))
    {
        rangeCodeInt = BIPPT5VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT25VOLTS"))
    {
        rangeCodeInt = BIPPT25VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT2VOLTS"))
    {
        rangeCodeInt = BIPPT2VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT1VOLTS"))
    {
        rangeCodeInt = BIPPT1VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT05VOLTS"))
    {
        rangeCodeInt = BIPPT05VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT01VOLTS"))
    {
        rangeCodeInt = BIPPT01VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT005VOLTS"))
    {
        rangeCodeInt = BIPPT005VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIP1PT67VOLTS"))
    {
        rangeCodeInt = BIP1PT67VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT312VOLTS"))
    {
        rangeCodeInt = BIPPT312VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT156VOLTS"))
    {
        rangeCodeInt = BIPPT156VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT125VOLTS"))
    {
        rangeCodeInt = BIPPT125VOLTS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT078VOLTS"))
    {
        rangeCodeInt = BIPPT078VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI10VOLTS"))
    {
        rangeCodeInt = UNI10VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI5VOLTS"))
    {
        rangeCodeInt = UNI5VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI4VOLTS"))
    {
        rangeCodeInt = UNI4VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI2PT5VOLTS"))
    {
        rangeCodeInt = UNI2PT5VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI2VOLTS"))
    {
        rangeCodeInt = UNI2VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI1PT67VOLTS"))
    {
        rangeCodeInt = UNI1PT67VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI1PT25VOLTS"))
    {
        rangeCodeInt = UNI1PT25VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNI1VOLTS"))
    {
        rangeCodeInt = UNI1VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNIPT5VOLTS"))
    {
        rangeCodeInt = UNIPT5VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNIPT25VOLTS"))
    {
        rangeCodeInt = UNIPT25VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNIPT2VOLTS"))
    {
        rangeCodeInt = UNIPT2VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNIPT1VOLTS"))
    {
        rangeCodeInt = UNIPT1VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNIPT05VOLTS"))
    {
        rangeCodeInt = UNIPT05VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNIPT02VOLTS"))
    {
        rangeCodeInt = UNIPT02VOLTS;
    }
    else if (!strcmp(rangeCodeString, "UNIPT01VOLTS"))
    {
        rangeCodeInt = UNIPT01VOLTS;
    }
    else if (!strcmp(rangeCodeString, "MA4TO20"))
    {
        rangeCodeInt = MA4TO20;
    }
    else if (!strcmp(rangeCodeString, "MA2TO10"))
    {
        rangeCodeInt = MA2TO10;
    }
    else if (!strcmp(rangeCodeString, "MA1TO5"))
    {
        rangeCodeInt = MA1TO5;
    }
    else if (!strcmp(rangeCodeString, "MAPT5TO2PT5"))
    {
        rangeCodeInt = MAPT5TO2PT5;
    }
    else if (!strcmp(rangeCodeString, "MA0TO20"))
    {
        rangeCodeInt = MA0TO20;
    }
    else if (!strcmp(rangeCodeString, "BIPPT025AMPS"))
    {
        rangeCodeInt = BIPPT025AMPS;
    }
    else if (!strcmp(rangeCodeString, "BIPPT025VOLTSPERVOLT"))
    {
        rangeCodeInt = BIPPT025VOLTSPERVOLT;
    }
    else
    {
        rangeCodeInt = NOTUSED;
    }

    return rangeCodeInt;
}

//----------------------------------------------------------------------------------------------------------------------------------
int MeasurementComputing::tempScaleStringToInt(char* tempScaleString)
{
    int tempScaleInt;
    if (!strcmp(tempScaleString, "CELSIUS"))
    {
        tempScaleInt = CELSIUS;
    }
    else if(!strcmp(tempScaleString, "FAHRENHEIT"))
    {
        tempScaleInt = FAHRENHEIT;
    }
    else if(!strcmp(tempScaleString, "KELVIN"))
    {
        tempScaleInt = KELVIN;
    }
    else if(!strcmp(tempScaleString, "VOLTS"))
    {
        tempScaleInt = VOLTS;
    }
    else if(!strcmp(tempScaleString, "NOSCALE"))
    {
        tempScaleInt = NOSCALE;
    }
    else
    {
        tempScaleInt = NOSCALE;
    }

    return tempScaleInt;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MeasurementComputing::tempScaleIntToString(int tempScaleInt)
{
    QString tempScaleString;

    switch (tempScaleInt)
    {
    case 0:  //CELSIUS
        tempScaleString = "CELSIUS";
        break;
    case 1:    //FAHRENHEIT
        tempScaleString = "FAHRENHEIT";
        break;
    case 2: //KELVIN
        tempScaleString = "KELVIN";
        break;
    case 4: //VOLTS
        tempScaleString = "VOLTS";
        break;
    case 5: //NOSCALE
        tempScaleString = "NOSCALE";
        break;
    default:
        tempScaleString = "NOSCALE";
        break;
    }

    return tempScaleString;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString MeasurementComputing::digDevTypeIntToString(int digDevTypeInt)
{
    QString digDevTypeString;

    switch (digDevTypeInt)
    {
    case 1: //AUXPORT
        digDevTypeString = "AUXPORT";
        break;
    case 10: //FIRSTPORTA
        digDevTypeString = "FIRSTPORTA";
        break;
    case 11: //FIRSTPORTB
        digDevTypeString = "FIRSTPORTB";
        break;
    case 12: //FIRSTPORTC
        digDevTypeString = "FIRSTPORTC";
        break;
    case 13: //FIRSTPORTCH
        digDevTypeString = "FIRSTPORTCH";
        break;
    case 14: //SECONDPORTA
        digDevTypeString = "SECONDPORTA";
        break;
    case 15: //SECONDPORTB
        digDevTypeString = "SECONDPORTB";
        break;
    case 16: //SECONDPORTCL
        digDevTypeString = "SECONDPORTCL";
        break;
    case 17: //SECONDPORTCH
        digDevTypeString = "SECONDPORTCH";
        break;
    case 18: //THIRDPORTA
        digDevTypeString = "THIRDPORTA";
        break;
    case 19: //THIRDPORTB
        digDevTypeString = "THIRDPORTB";
        break;
    case 20: //THIRDPORTCL
        digDevTypeString = "THIRDPORTCL";
        break;
    case 21: //THIRDPORTCH
        digDevTypeString = "THIRDPORTCH";
        break;
    case 22: //FOURTHPORTA
        digDevTypeString = "FOURTHPORTA";
        break;
    case 23: //FOURTHPORTB
        digDevTypeString = "FOURTHPORTB";
        break;
    case 24: //FOURTHPORTCL
        digDevTypeString = "FOURTHPORTCL";
        break;
    case 25: //FOURTHPORTCH
        digDevTypeString = "FOURTHPORTCH";
        break;
    case 26: //FIFTHPORTA
        digDevTypeString = "FIFTHPORTA";
        break;
    case 27: //FIFTHPORTB
        digDevTypeString = "FIFTHPORTB";
        break;
    case 28: //FIFTHPORTCL
        digDevTypeString = "FIFTHPORTCL";
        break;
    case 29: //FIFTHPORTCH
        digDevTypeString = "FIFTHPORTCH";
        break;
    case 30: //SIXTHPORTA
        digDevTypeString = "SIXTHPORTA";
        break;
    case 31: //SIXTHPORTB
        digDevTypeString = "SIXTHPORTB";
        break;
    case 32: //SIXTHPORTCL
        digDevTypeString = "SIXTHPORTCL";
        break;
    case 33: //SIXTHPORTCH
        digDevTypeString = "SIXTHPORTCH";
        break;
    case 34: //SEVENTHPORTA
        digDevTypeString = "SEVENTHPORTA";
        break;
    case 35: //SEVENTHPORTB
        digDevTypeString = "SEVENTHPORTB";
        break;
    case 36: //SEVENTHPORTCL
        digDevTypeString = "SEVENTHPORTCL";
        break;
    case 37: //SEVENTHPORTCH
        digDevTypeString = "SEVENTHPORTCH";
        break;
    case 38: //EIGHTHPORTA
        digDevTypeString = "EIGHTHPORTA";
        break;
    case 39: //EIGHTHPORTB
        digDevTypeString = "EIGHTHPORTB";
        break;
    case 40: //EIGHTHPORTCL
        digDevTypeString = "EIGHTHPORTCL";
        break;
    case 41: //EIGHTHPORTCH
        digDevTypeString = "EIGHTHPORTCH";
        break;
    default:
        digDevTypeString = "FIRSTPORTA";
        break;
    }

    return digDevTypeString;
}

//----------------------------------------------------------------------------------------------------------------------------------
int MeasurementComputing::digDevTypeStringToInt(char* digDevTypeString)
{
    int digDevTypeInt;
    if (!strcmp(digDevTypeString, "AUXPORT"))
    {
        digDevTypeInt = AUXPORT;
    }
    else if(!strcmp(digDevTypeString, "FIRSTPORTA"))
    {
        digDevTypeInt = FIRSTPORTA;
    }
    else if(!strcmp(digDevTypeString, "FIRSTPORTB"))
    {
        digDevTypeInt = FIRSTPORTB;
    }
    else if(!strcmp(digDevTypeString, "FIRSTPORTCL"))
    {
        digDevTypeInt = FIRSTPORTCL;
    }
    else if(!strcmp(digDevTypeString, "FIRSTPORTCH"))
    {
        digDevTypeInt = FIRSTPORTCH;
    }
    else if(!strcmp(digDevTypeString, "FIRSTPORTC"))
    {
        digDevTypeInt = FIRSTPORTC;
    }
    else if(!strcmp(digDevTypeString, "SECONDPORTA"))
    {
        digDevTypeInt = SECONDPORTA;
    }
    else if(!strcmp(digDevTypeString, "SECONDPORTB"))
    {
        digDevTypeInt = SECONDPORTB;
    }
    else if(!strcmp(digDevTypeString, "SECONDPORTCL"))
    {
        digDevTypeInt = SECONDPORTCL;
    }
    else if(!strcmp(digDevTypeString, "SECONDPORTCH"))
    {
        digDevTypeInt = SECONDPORTCH;
    }
    else if(!strcmp(digDevTypeString, "THIRDPORTA"))
    {
        digDevTypeInt = THIRDPORTA;
    }
    else if(!strcmp(digDevTypeString, "THIRDPORTB"))
    {
        digDevTypeInt = THIRDPORTB;
    }
    else if(!strcmp(digDevTypeString, "THIRDPORTCL"))
    {
        digDevTypeInt = THIRDPORTCL;
    }
    else if(!strcmp(digDevTypeString, "THIRDPORTCH"))
    {
        digDevTypeInt = THIRDPORTCH;
    }
    else if(!strcmp(digDevTypeString, "FOURTHPORTA"))
    {
        digDevTypeInt = FOURTHPORTA;
    }
    else if(!strcmp(digDevTypeString, "FOURTHPORTB"))
    {
        digDevTypeInt = FOURTHPORTB;
    }
    else if(!strcmp(digDevTypeString, "FOURTHPORTCL"))
    {
        digDevTypeInt = FOURTHPORTCL;
    }
    else if(!strcmp(digDevTypeString, "FOURTHPORTCH"))
    {
        digDevTypeInt = FOURTHPORTCH;
    }
    else if(!strcmp(digDevTypeString, "FIFTHPORTA"))
    {
        digDevTypeInt = FIFTHPORTA;
    }
    else if(!strcmp(digDevTypeString, "FIFTHPORTB"))
    {
        digDevTypeInt = FIFTHPORTB;
    }
    else if(!strcmp(digDevTypeString, "FIFTHPORTCL"))
    {
        digDevTypeInt = FIFTHPORTCL;
    }
    else if(!strcmp(digDevTypeString, "FIFTHPORTCH"))
    {
        digDevTypeInt = FIFTHPORTCH;
    }
    else if(!strcmp(digDevTypeString, "SIXTHPORTA"))
    {
        digDevTypeInt = SIXTHPORTA;
    }
    else if(!strcmp(digDevTypeString, "SIXTHPORTB"))
    {
        digDevTypeInt = SIXTHPORTB;
    }
    else if(!strcmp(digDevTypeString, "SIXTHPORTCL"))
    {
        digDevTypeInt = SIXTHPORTCL;
    }
    else if(!strcmp(digDevTypeString, "SIXTHPORTCH"))
    {
        digDevTypeInt = SIXTHPORTCH;
    }
    else if(!strcmp(digDevTypeString, "SEVENTHPORTA"))
    {
        digDevTypeInt = SEVENTHPORTA;
    }
    else if(!strcmp(digDevTypeString, "SEVENTHPORTB"))
    {
        digDevTypeInt = SEVENTHPORTB;
    }
    else if(!strcmp(digDevTypeString, "SEVENTHPORTCL"))
    {
        digDevTypeInt = SEVENTHPORTCL;
    }
    else if(!strcmp(digDevTypeString, "SEVENTHPORTCH"))
    {
        digDevTypeInt = SEVENTHPORTCH;
    }
    else if(!strcmp(digDevTypeString, "EIGHTHPORTA"))
    {
        digDevTypeInt = EIGHTHPORTA;
    }
    else if(!strcmp(digDevTypeString, "EIGHTHPORTB"))
    {
        digDevTypeInt = EIGHTHPORTB;
    }
    else if(!strcmp(digDevTypeString, "EIGHTHPORTCL"))
    {
        digDevTypeInt = EIGHTHPORTCL;
    }
    else if(!strcmp(digDevTypeString, "EIGHTHPORTCH"))
    {
        digDevTypeInt = EIGHTHPORTCH;
    }
    else
    {
        digDevTypeInt = FIRSTPORTA;
    }

    return digDevTypeInt;
}
