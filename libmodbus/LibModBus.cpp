/* ********************************************************************
    Plugin "LibModBus" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#include "LibModBus.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qbytearray.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qregularexpression.h>

#include "pluginVersion.h"
#include "gitVersion.h"

//#include "dockWidgetLibModBus.h"

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBusInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(LibModBus) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBusInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(LibModBus) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
LibModBusInterface::LibModBusInterface()
{
    m_type = ito::typeDataIO | ito::typeADDA;
    setObjectName("LibModBus");

    m_description = tr("itom-plugin for a modbus communication");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"LibModBus is a itom-Plugin which provides modbusTCP and modbusRTU communication.\n\
The plugin is based on libmodbus v3.1.1 library and tested under Windows only atm.\n\
Registers are addressed using the modbus_read_registers (0x03) and modbus_write_registers (0x10) functions of libmodbus, coils are addressed using the modbus_read_bits (0x01) and modbus_write_bits (0x0F) functions. \n\
The plugin-functions used are getVal(dObj) and setVal(dObj) with a data object of the size 1xN with N the number of registers to be read/written. \n\
The content of the registers is expected as data in the uint16 data object for registers or uint8 data object for coils, the addressing of the registers is performed by a dObj-MetaTag 'registers' containing a string with address and number of consecutive registers seperated by ',' and different registers seperated by ';' i.e.: '10,2;34,1;77,4' to address registers 10,11;34;77..80. Number 1 of consecutive registers can be left out i.e.:'10,2;34;77,4' \n\
If no MetaTag is set, values of m_params['registers'] is tried to be used for addressing.";
    m_detaildescription = tr(docstring);*/
    m_detaildescription = tr(
"LibModBus is a itom-Plugin which provides modbusTCP and modbusRTU communication.\n\
The plugin is based on libmodbus v3.1.1 library and tested under Windows only atm.\n\
Registers are addressed using the modbus_read_registers (0x03) and modbus_write_registers (0x10) functions of libmodbus, coils are addressed using the modbus_read_bits (0x01) and modbus_write_bits (0x0F) functions. \n\
The plugin-functions used are getVal(dObj) and setVal(dObj) with a data object of the size 1xN with N the number of registers to be read/written. \n\
The content of the registers is expected as data in the uint16 data object for registers or uint8 data object for coils, the addressing of the registers is performed by a dObj-MetaTag 'registers' containing a string with address and number of consecutive registers seperated by ',' and different registers seperated by ';' i.e.: '10,2;34,1;77,4' to address registers 10,11;34;77..80. Number 1 of consecutive registers can be left out i.e.:'10,2;34;77,4' \n\
If no MetaTag is set, values of m_params['registers'] is tried to be used for addressing.");

    m_author = "J.Nitsche, IPROM, TU Braunschweig";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under GPL, since the libmodbus is also licensed under GPL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("target", ito::ParamBase::String, "127.0.0.1", tr("Adress of the target device. IP-Adress for ModbusTCP (i.e. 127.0.0.1) or COM-Port for ModbusRTU (i.e. COM1)").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::RegExp, "[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}||COM[1-9]||/dev/ttyS[0-9]{1,3}||/dev/ttyUSB[0-9]{1,3}"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("port", ito::ParamBase::Int, 0, 1024, 502, tr("The number of the TCP port for ModBusTCP (default 502) or slave ID for ModbusRTU").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("baud", ito::ParamBase::Int, 50, 4000000, 9600, tr("The baudrate of the port for RTU communication").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::String, "N", tr("Parity for RTU communication (N,E,O)").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::RegExp, "[N,P,O]{1}"), true);
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("databit", ito::ParamBase::Int, 5, 8, 8, tr("Number of bits to be written in line for RTU communication").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("stopbit", ito::ParamBase::Int, 1, 2, 1, tr("Stop bits after every n bits for RTU communication").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("output_mode", ito::ParamBase::Int, 0, 1, 0, tr("Enables command-line output of different readouts (e.g. register values of getVal)").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
LibModBusInterface::~LibModBusInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LibModBus::showConfDialog(void)
{
//    dialogLibModBus *confDialog = new dialogLibModBus((void*)this);
//    QVariant qvar = m_params["port"].getVal<double>();
//    confDialog->setVals(&m_params);
//    if (confDialog->exec())
//    {
//        confDialog->getVals(&m_paramNames);
//    }
//    delete confDialog;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
LibModBus::LibModBus() : AddInDataIO(), m_pCTX(NULL), m_connected(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "LibModBus", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("target", ito::ParamBase::String | ito::ParamBase::In | ito::ParamBase::Readonly, "127.0.0.1", tr("IP Adress or COM-Port of the target device").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("port", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, 0, 1024, 502, tr("TCP Port for ModbusTCP or slave ID for ModbusRTU").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("baud", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, 50, 4000000, 9600, tr("The baudrate of the port for RTU communication").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::String | ito::ParamBase::In | ito::ParamBase::Readonly, "N", tr("Parity for RTU communication (N,E,O)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("databit", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, 5, 8, 8, tr("Number of bits to be written in line for RTU communication").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stopbit", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, 1, 2, 1, tr("Stop bits after every n bits for RTU communication").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("output_mode", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("Enables command-line output of different readouts (e.g. register values of getVal)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("registers",ito::ParamBase::String | ito::ParamBase::In,"0,10",tr("Default string for register addressing. Coding is 'Reg1Address,Reg1Size;Reg2Address,Reg2Size...'").toLatin1().data());
    m_params.insert(paramVal.getName(),paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
LibModBus::~LibModBus()
{
    m_params.clear();
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
ito::RetVal LibModBus::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal LibModBus::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

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

        //if you program for itom 1.4.0 or higher (Interface version >= 1.3.1) you should use this
        //API method instead of the one above: The difference is, that incoming parameters that are
        //compatible but do not have the same type than the corresponding m_params value are cast
        //to the type of the internal parameter and incoming double values are rounded to the
        //next value (depending on a possible step size, if different than 0.0)
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        //all parameters that don't need further checks can simply be assigned
        //to the value in m_params (the rest is already checked above)
        retValue += it->copyValueFrom( &(*val) );
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
ito::RetVal LibModBus::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval;
    int port = 0;
    int baud = 9600;
    int databit = 8;
    int stopbit = 1;
    char parity;
    char *target;
    bool IP = false;
    bool output_mode=false;

    retval += m_params["target"].copyValueFrom(&((*paramsMand)[0]));
    target = m_params["target"].getVal<char *>(); //borrowed reference
    retval += m_params["port"].copyValueFrom(&((*paramsOpt)[0]));
    port = m_params["port"].getVal<int>();
    retval += m_params["baud"].copyValueFrom(&((*paramsOpt)[1]));
    baud = m_params["baud"].getVal<int>();
    retval += m_params["parity"].copyValueFrom(&((*paramsOpt)[2]));
    parity = *m_params["parity"].getVal<char *>();
    retval += m_params["databit"].copyValueFrom(&((*paramsOpt)[3]));
    databit = m_params["databit"].getVal<int>();
    retval += m_params["stopbit"].copyValueFrom(&((*paramsOpt)[4]));
    stopbit = m_params["stopbit"].getVal<int>();
    retval += m_params["output_mode"].copyValueFrom(&((*paramsOpt)[5]));
    output_mode = m_params["output_mode"].getVal<int>();

    QString target_ = target;
    QRegularExpression rx_ip("^[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}$");
    QRegularExpression rx_com("^COM[1-9]$");
    QRegularExpression rx_tty("^/dev/ttyS[0-9]{1,3}||/dev/ttyUSB[0-9]{1,3}$");

    if (!retval.containsError())
    {
        auto match1 = rx_ip.match(target_);
        auto match2 = rx_com.match(target_);
        auto match3 = rx_tty.match(target_);

        if (match1.hasMatch())
        {
            //std::cout << "IP found \n" << std::endl;
            m_pCTX = modbus_new_tcp(target,port);
            IP = true;
        }
        else if (match2.hasMatch() || match3.hasMatch())
        {
            //std::cout << "Serial found \n" << std::endl;
            m_pCTX = modbus_new_rtu(target,baud,parity,databit,stopbit);
            modbus_set_slave(m_pCTX,port);
            IP = false;
            //retval += ito::RetVal(ito::retError, 0, tr("COM found").toLatin1().data());
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("invalid target device").toLatin1().data());
        }
        if (m_pCTX == NULL)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Unable to allocate libmodbus context").toLatin1().data());
        }
        else if ( modbus_connect(m_pCTX) == -1)
        {
            retval += ito::RetVal(ito::retError,0,QObject::tr("Modbus-connect failed!").toLatin1().data());
        }
        else
        {
            m_connected = true;
        }

        if (!retval.containsError())
        {
           std::cout << "Connect to Device at: " << target << "; Port/ID: " << port << " success" << std::endl;
        }
    }

    if (!retval.containsError())
    {
        emit parametersChanged(m_params);
        setIdentifier(QString("Target %1 @ Port/ID %2").arg(target).arg(port));
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (m_pCTX && m_connected)
    {
        modbus_close(m_pCTX);
        m_connected = false;
    }

    if (m_pCTX)
    {
        modbus_free(m_pCTX);
        m_pCTX = NULL;
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::startDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StartDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::stopDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StopDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("Acquire not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    bool validOp=true;
    bool output_mode=false;
    uint16_t tab_reg[64];
    uint8_t coil_reg[64];
    ito::DataObjectTagType registers;
    int listcounter,registercounter,i,j,tmpInt;
    int regNumbers = 0;
    int dObjPos=0;
    std::vector<int> regAddr,regNb;
    QString regContent;
    QStringList regList,addrList;
    output_mode = m_params["output_mode"].getVal<int>();
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    int inputDataType= dObj->getType();

    if (inputDataType!=3 && inputDataType!=1)
    {
            retval += ito::RetVal(ito::retError,0,QObject::tr("Data type of input object must be uint16 for registers or uint8 for coils").toLatin1().data());
    }

    if (!retval.containsError())
    {
        if (dObj->getSize(0)>0)
        {
            registers = dObj->getTag("registers",validOp);
            if (validOp)                                            // Register Address transmitted in dObj-Meta-Data
            {
                regContent = registers.getVal_ToString().data();
            }
            else                                                    // Register Adress taken from m_params as default fallback
            {
                char* regchar = m_params["registers"].getVal<char*>();
                regContent = QString(QLatin1String(regchar));
            }
            regList = regContent.split(";");
            listcounter=regList.size();

            for ( i=0; i<listcounter; i++)
            {
                addrList = regList.at(i).split(",");
                regAddr.push_back(addrList.at(0).toInt(&validOp));
                tmpInt=addrList.size();
                if (tmpInt>1)
                {
                    tmpInt = addrList.at(1).toInt(&validOp);
                }
                else
                {
                    tmpInt=1;
                }
                regNb.push_back(tmpInt);
                regNumbers += tmpInt;
            }

            if (regNumbers == dObj->getSize(1))
            {
                for (i=0;i<regAddr.size();i++)
                {
                    if (inputDataType==1)
                    {
                        registercounter = modbus_read_bits(m_pCTX, regAddr.at(i), regNb.at(i), coil_reg);
                    }
                    else
                    {
                        registercounter = modbus_read_registers(m_pCTX, regAddr.at(i), regNb.at(i), tab_reg);
                    }
                    for (j=0; j < registercounter; j++)
                    {
                        if (output_mode)
                        {
                            if (inputDataType==1)
                            {
                                std::cout << "coil[" << regAddr.at(i)+j << "]=" << coil_reg[j] << "\n" << std::endl;
                            }
                            else
                            {
                                std::cout << "reg[" << regAddr.at(i)+j << "]=" << tab_reg[j] << "\n" << std::endl;
                            }
                        }
                        if (inputDataType==1)
                        {
                            dObj->at<ito::uint8>(0,dObjPos)=coil_reg[j];
                        }
                        else
                        {
                            dObj->at<ito::uint16>(0,dObjPos)=tab_reg[j];
                        }
                        dObjPos++;
                    }
                }
            }
            else
            {
                retval += ito::RetVal(ito::retError,0,QObject::tr("Size of given data object does not match number of requested registers").toLatin1().data());
            }
        }
    }
    //std::cout << val << std::endl;
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond)
{
//    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    //retval = m_serport.sread(data.data(), length.data(), 0);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::setVal(const char *data, const int datalength, ItomSharedSemaphore *waitCond)
{
    bool validOp=true;
    bool output_mode=false;
    uint16_t tab_reg[64];
    uint8_t coil_reg[64];
    ito::DataObjectTagType registers;
    int listcounter,registercounter,i,tmpInt;
    int regNumbers = 0;
    int dObjPos=0;
    std::vector<int> regAddr,regNb;
    QString regContent;
    QStringList regList,addrList;
    output_mode = m_params["output_mode"].getVal<int>();
    ItomSharedSemaphoreLocker locker(waitCond);
    const ito::DataObject *dObj = reinterpret_cast<const ito::DataObject*>(data);
    //const char *buf = data;
    char endline[3] = {0, 0, 0};
    ito::RetVal retval(ito::retOk);
    int inputDataType=dObj->getType();

    if (inputDataType!=3 && inputDataType!=1)
    {
            retval += ito::RetVal(ito::retError,0,QObject::tr("Data type of input object must be uint16 for registers or uint8 for coils").toLatin1().data());
    }

    if (!retval.containsError())
    {
        if (dObj->getSize(0)>0)
        {
            registers = dObj->getTag("registers",validOp);
            if (validOp)                                            // Register Address transmitted in dObj-Meta-Data
            {
                regContent = registers.getVal_ToString().data();
            }
            else                                                    // Register Adress taken from m_params as default fallback
            {
                char* regchar = m_params["registers"].getVal<char*>();
                regContent = QString(QLatin1String(regchar));
            }
            regList = regContent.split(";");
            listcounter=regList.size();

            for ( i=0; i<listcounter; i++)
            {
                addrList = regList.at(i).split(",");
                regAddr.push_back(addrList.at(0).toInt(&validOp));
                tmpInt=addrList.size();
                if (tmpInt>1)
                {
                    tmpInt = addrList.at(1).toInt(&validOp);
                }
                else
                {
                    tmpInt=1;
                }
                regNb.push_back(tmpInt);
                regNumbers += tmpInt;
            }

            if (regNumbers == dObj->getSize(1))
            {
                for (i=0;i<regNumbers;i++)
                {
                    if (inputDataType==1)
                    {
                        coil_reg[i]=dObj->at<ito::uint8>(0,i);
                    }
                    else
                    {
                        tab_reg[i]=dObj->at<ito::uint16>(0,i);
                    }
                }
                for (i=0;i<regAddr.size();i++)
                {
                    if (inputDataType==1)
                    {
                        uint8_t *coil_reg_nb = coil_reg + dObjPos;
                        registercounter = modbus_write_bits(m_pCTX, regAddr.at(i), regNb.at(i), coil_reg_nb);
                    }
                    else
                    {
                        uint16_t *tab_reg_nb = tab_reg + dObjPos;
                        registercounter = modbus_write_registers(m_pCTX, regAddr.at(i), regNb.at(i), tab_reg_nb);
                    }
                    if (registercounter == regNb.at(i))
                    {
                        if (output_mode)
                        {
                            std::cout << "Write at Reg. " << regAddr.at(i) << " success! \n " << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "Write at Reg. " << regAddr.at(i) << " failed! \n " << std::endl;
                    }
                    dObjPos=dObjPos+regNb.at(i);
                    /*registercounter = modbus_read_registers(m_pCTX, regAddr.at(i), regNb.at(i), tab_reg);
                    for (j=0; j < registercounter; j++)
                    {
                        std::cout << "reg[" << regAddr.at(i)+j << "]=" << tab_reg[j] << "\n" << std::endl;
                        incomingObject->at<ito::uint16>(0,dObjPos)=tab_reg[j];
                        dObjPos++;
                    }*/
                }
            }
            else
            {
                retval += ito::RetVal(ito::retError,0,QObject::tr("Size of given data object does not match number of transmitted registers").toLatin1().data());
            }
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
ito::RetVal LibModBus::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond /*= NULL*/)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*void LibModBus::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetLibModBus *dw = qobject_cast<DockWidgetLibModBus*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), dw, SLOT(serialLog(QByteArray, QByteArray, const char)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            disconnect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), dw, SLOT(serialLog(QByteArray, QByteArray, const char)));
        }
    }
}*/

//----------------------------------------------------------------------------------------------------------------------------------
