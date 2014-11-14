/* ********************************************************************
    Plugin "LibModBus" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#include "LibModBus.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qbytearray.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

//#ifndef linux
//    #include <Windows.h>
////#define CCTS_OFLOW      0x00010000      /* CTS flow control of output */
////#define CRTSCTS         (CCTS_OFLOW | CRTS_IFLOW)
////#define CRTS_IFLOW      0x00020000      /* RTS flow control of input */
//#else
//    #include <unistd.h>
////    #include <stdio.h>
//    #include <termios.h>
////    #include <sys/stat.h>
//    #include <fcntl.h>
//    #include <sys/ioctl.h>
//#endif

//#include "dockWidgetLibModBus.h"
modbus_t *ctx;
int initnum=0;


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
    char docstring[] = \
	"LibModBus is a itom-Plugin which provides modbusTCP communication.\n\
	The plugin is based on libmodbus v3.1.1 library and under development for Windows only atm.\n\
	Registers are addressed using the modbus_read_registers (0x03) and modbus_write_registers (0x10) functions of libmodbus. \n\
	The plugin-functions used are getVal(dObj) and setVal(dObj) with a data object of the size 1xN with N the number of registers to be read/written. \n\
	The content of the registers is expected as data in the uint16 data object, the addressing of the registers is performed by a dObj-MetaTag 'registers' containing a string with address and number of consecutive registers seperated by ',' and different registers seperated by ';' i.e.: '10,2;34,1;77,4' to address registers 10,11;34;77..80. Number 1 of consecutive registers can be left out i.e.:'10,2;34;77,4' \n\
	If no MetaTag is set, values of m_params['registers'] is tried to be used for addressing.";

    m_detaildescription = tr(docstring);
    m_author = "J.Nitsche, IPROM, University Braunschweig";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");  

    ito::Param paramVal("IP", ito::ParamBase::String, "127.0.0.1", tr("IP-Adress of the target device (for ModBus TCP)").toLatin1().data());
    m_initParamsMand.append(paramVal);
	paramVal = ito::Param("port", ito::ParamBase::Int, 0, 1024, 502, tr("The number of the TCP port for ModBus Communication (default 502)").toLatin1().data());
    m_initParamsMand.append(paramVal);


    /*paramVal = ito::Param("bits", ito::ParamBase::Int, 5, 8, 8, tr("Number of bits to be written in line").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("stopbits", ito::ParamBase::Int, 1, 2, 1, tr("Stop bits after every n bits").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::Int, 0, 2, 0, tr("Parity: 0 -> no parity, 1 -> odd parity, 2 -> even parity").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("flow", ito::ParamBase::Int, 0, 127, 0, tr("Bitmask for flow control (see docstring for more information)").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("sendDelay", ito::ParamBase::Int, 0, 65000, 0, tr("0 -> write output buffer as block or single characters with delay (1..65000)").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("enableDebug", ito::ParamBase::Int, 0, 1, 0, tr("Initialised 'debug'-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_initParamsOpt.append(paramVal);*/

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
LibModBusInterface::~LibModBusInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(LibModBusinterface, LibModBusInterface)
#endif

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
LibModBus::LibModBus() : AddInDataIO(), m_debugMode(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "LibModBus", NULL);
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("IP", ito::ParamBase::String | ito::ParamBase::In, "127.0.0.1", tr("IP Adress of the target device").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("port", ito::ParamBase::Int | ito::ParamBase::In, 0, 1024, 502, tr("TCP Port for ModBus TCP Communication").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("registers",ito::ParamBase::String | ito::ParamBase::In,"1,10",tr("Default string for register addressing. Coding is 'Reg1Address,Reg1Size;Reg2Address,Reg2Size...'").toLatin1().data());
	m_params.insert(paramVal.getName(),paramVal);
    /*paramVal = ito::Param("baud", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 50, 4000000, 9600, tr("Current baudrate in bits/s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bits", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 5, 8, 8, tr("Number of bits to be written in line").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stopbits", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 1, 2, 1, tr("Stop bits after every n bits").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 2, 0, tr("Toggle parity check").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("flow", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 127, 0, tr("Bitmask for flow control as integer").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endline", ito::ParamBase::String | ito::ParamBase::NoAutosave, "\n", tr("Endline character, will be added automatically during setVal").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sendDelay", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 65000, 0, tr("0 -> write output buffer as block at once or single characters with delay (1..65000)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::NoAutosave, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 1, 0, tr("If true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);*/

    //register exec functions
    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    /*registerExecFunc("clearInputBuffer", pMand, pOpt, pOut, tr("Clears the input buffer of serial port"));
    registerExecFunc("clearOutputBuffer", pMand, pOpt, pOut, tr("Clears the output buffer of serial port"));

    pMand << ito::Param("bufferType", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("Clears input (0) or output (1) buffer").toLatin1().data());
    registerExecFunc("clearBuffer", pMand, pOpt, pOut, tr("Clears the input or output buffer of serial port"));*/

/*    //now create dock widget for this plugin
    DockWidgetLibModBus *LibModBusWidget = new DockWidgetLibModBus(m_params, m_uniqueID);
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), LibModBusWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(this, SIGNAL(uniqueIDChanged(const int)), LibModBusWidget, SLOT(uniqueIDChanged(const int)));
    connect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), LibModBusWidget, SLOT(serialLog(QByteArray, QByteArray, const char)));*/

    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

    //now create dock widget for this plugin
    /*DockWidgetLibModBus *dw = new DockWidgetLibModBus(m_params, getID() );
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);*/
}

//----------------------------------------------------------------------------------------------------------------------------------
LibModBus::~LibModBus()
{
   m_pThread->quit();
   m_pThread->wait(5000);
   delete m_pThread;
   m_pThread = NULL;

   m_params.clear();

   return;
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
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toLatin1().data());
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
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
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
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toLatin1().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
        {

            int flow = 0;
            char *endline = NULL;


            if (paramIt->getFlags() & ito::ParamBase::Readonly)    //check read-only
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toLatin1().data());
                goto end;
            }
            else if (val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if (curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else if (curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else
                {
                    paramIt.value().setVal<double>(curval);
                }
            }
            else if (paramIt->getType() == val->getType())
            {
                retValue += paramIt.value().copyValueFrom(&(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toLatin1().data());
                goto end;
            }

            
            flow = m_params["flow"].getVal<int>();
            endline = m_params["endline"].getVal<char*>(); //borrowed reference
            //retValue += m_serport.setparams(baud, endline, bits, stopbits, parity, flow, sendDelay, timeout);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Parameter not found").toLatin1().data());
        }
    }
    emit parametersChanged(m_params);

end:
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
    int port = paramsMand->at(1).getVal<int>();
	int connect = 0;
	m_params["port"].setVal<int>(port);
    char *IP;
    //char * ip = paramsMand->at(0).getVal<char>();;

    // mandatory parameters
    if (paramsMand == NULL)
    {
        retval = ito::RetVal(ito::retError, 0, QObject::tr("Mandatory paramers are NULL").toLatin1().data());
        goto end;
    }


    retval += m_params["IP"].copyValueFrom(&((*paramsMand)[0]));
    IP = m_params["IP"].getVal<char *>(); //borrowed reference
    //strncpy(IP, ip, 3);
//    sprintf(endline, "%s", tendline);

    // optional parameters
    if (paramsOpt == NULL)
    {
        retval = ito::RetVal(ito::retError, 0, QObject::tr("Optinal paramers are NULL").toLatin1().data());
        goto end;
    }
	retval += m_params["port"].copyValueFrom(&((*paramsMand)[1]));
    port = m_params["port"].getVal<int>();


	ctx = modbus_new_tcp(IP,port);
    if ( modbus_connect(ctx)==0)
	{
		retval+=ito::RetVal(ito::retOk);
	}
	else
	{
		retval+=ito::RetVal(ito::retError,0,QObject::tr("ModbusTCP-connect failed!").toLatin1().data());
	}
	
    if (!retval.containsError())
    {
       std::cout << "Connect to Device at IP: " << IP << "; Port: " << port << " success" << std::endl;
	   ++initnum;
    }


    //retval += m_params["debug"].copyValueFrom(&((*paramsOpt)[6]));
    //m_debugMode = (bool)(m_params["debug"].getVal<int>());

    emit parametersChanged(m_params);

end:

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
	modbus_close(ctx);
	--initnum;
	if ( initnum==0)
	{
		modbus_free(ctx);
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

ito::RetVal LibModBus::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
	bool validOp=true;
	uint16_t tab_reg[64];
	ito::DataObjectTagType registers;
	int listcounter,registercounter,i,j,tmpInt;
	int regNumbers = 0;
	int dObjPos=0;
	std::vector<int> regAddr,regNb;
	QString regContent;
	QStringList regList,addrList;
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
	if (dObj->getSize(0)>0)
	{
		registers = dObj->getTag("registers",validOp);
		if (validOp)											// Register Address transmitted in dObj-Meta-Data
		{
			regContent = registers.getVal_ToString().data();
		}
		else													// Register Adress taken from m_params as default fallback
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
				registercounter = modbus_read_registers(ctx, regAddr.at(i), regNb.at(i), tab_reg);
				for (j=0; j < registercounter; j++) 
				{
					std::cout << "reg[" << regAddr.at(i)+j << "]=" << tab_reg[j] << "\n" << std::endl;
					dObj->at<ito::uint16>(0,dObjPos)=tab_reg[j];
					dObjPos++;
				}
			}
			
		}
		else
		{
			std::cout << "Size of given data object does not match number of requested registers \n" << std::endl;
		}
	}
	//std::cout << val << std::endl;
	if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LibModBus::getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond)
{
//    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    //retval = m_serport.sread(data.data(), length.data(), 0);

    if (m_debugMode)
    {
        //emit serialLog(QByteArray(data.data(),*length), "", '<');
    }

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
	uint16_t tab_reg[64];
	ito::DataObjectTagType registers;
	int listcounter,registercounter,i,j,tmpInt;
	int regNumbers = 0;
	int dObjPos=0;
	std::vector<int> regAddr,regNb;
	QString regContent;
	QStringList regList,addrList;


    ItomSharedSemaphoreLocker locker(waitCond);
	const ito::DataObject *incomingObject = reinterpret_cast<const ito::DataObject*>(data);
    //const char *buf = data;
    char endline[3] = {0, 0, 0};
    ito::RetVal retval(ito::retOk);

    //m_serport.getendline(endline);
    if (m_debugMode)
    {
        //emit serialLog(QByteArray(buf,datalength), QByteArray(endline, (int)strlen(endline)), '>');
    }
    //retval = m_serport.swrite(buf, datalength, m_params["sendDelay"].getVal<int>());

	if (incomingObject->getSize(0)>0)
	{
		registers = incomingObject->getTag("registers",validOp);
		if (validOp)											// Register Address transmitted in dObj-Meta-Data
		{
			regContent = registers.getVal_ToString().data();
		}
		else													// Register Adress taken from m_params as default fallback
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
		if (regNumbers == incomingObject->getSize(1))
		{
			for (i=0;i<regNumbers;i++)
			{
				tab_reg[i]=incomingObject->at<ito::uint16>(0,i);
			}
			for (i=0;i<regAddr.size();i++)
			{
				uint16_t *tab_reg_nb = tab_reg + dObjPos;
				registercounter = modbus_write_registers(ctx, regAddr.at(i), regNb.at(i), tab_reg_nb);
				if (registercounter == regNb.at(i))
				{
					std::cout << "Write at Reg. " << regAddr.at(i) << " success! \n " << std::endl; 
				}
				else
				{
					std::cout << "Write at Reg. " << regAddr.at(i) << " failed! \n " << std::endl; 
				}
				dObjPos=dObjPos+regNb.at(i);
				/*registercounter = modbus_read_registers(ctx, regAddr.at(i), regNb.at(i), tab_reg);
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
			std::cout << "Size of given data object does not match number of transmitted registers \n" << std::endl;
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

    if (funcName == "clearInputBuffer")
    {
        //retval = m_serport.sclearbuffer(0);
    }
    else if (funcName == "clearOutputBuffer")
    {
        //retval = m_serport.sclearbuffer(1);
    }
    else if (funcName == "clearBuffer")
    {
        ito::ParamBase *bufferType = NULL;
        bufferType = &((*paramsMand)[0]);
        if (!retval.containsError())
        {
            //retval = m_serport.sclearbuffer(static_cast<bool>(bufferType->getVal<int>()));
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
