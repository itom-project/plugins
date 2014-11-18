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
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::RegExp, "[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}"), true);
    m_initParamsMand.append(paramVal);
	paramVal = ito::Param("port", ito::ParamBase::Int, 0, 1024, 502, tr("The number of the TCP port for ModBus Communication (default 502)").toLatin1().data());
    m_initParamsMand.append(paramVal);
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
LibModBus::LibModBus() : AddInDataIO(), m_pCTX(NULL), m_connected(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "LibModBus", NULL);
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("IP", ito::ParamBase::String | ito::ParamBase::In | ito::ParamBase::Readonly, "127.0.0.1", tr("IP Adress of the target device").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("port", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, 0, 1024, 502, tr("TCP Port for ModBus TCP Communication").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("registers",ito::ParamBase::String | ito::ParamBase::In,"1,10",tr("Default string for register addressing. Coding is 'Reg1Address,Reg1Size;Reg2Address,Reg2Size...'").toLatin1().data());
	m_params.insert(paramVal.getName(),paramVal);

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
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
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

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
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

    if(!retValue.containsError())
    {
        //all parameters that don't need further checks can simply be assigned
        //to the value in m_params (the rest is already checked above)
        retValue += it->copyValueFrom( &(*val) );
    }

    if(!retValue.containsError())
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
    char *IP;

    retval += m_params["IP"].copyValueFrom(&((*paramsMand)[0]));
    IP = m_params["IP"].getVal<char *>(); //borrowed reference
	retval += m_params["port"].copyValueFrom(&((*paramsMand)[1]));
    port = m_params["port"].getVal<int>();

    if (!retval.containsError())
    {
	    m_pCTX = modbus_new_tcp(IP,port);
        if (m_pCTX == NULL)
        {
            retval += ito::RetVal(ito::retError, 0, tr("Unable to allocate libmodbus context").toLatin1().data());
        }
        else if ( modbus_connect(m_pCTX) == -1)
	    {
		    retval += ito::RetVal(ito::retError,0,QObject::tr("ModbusTCP-connect failed!").toLatin1().data());
	    }
        else
        {
            m_connected = true;
        }
	
        if (!retval.containsError())
        {
           std::cout << "Connect to Device at IP: " << IP << "; Port: " << port << " success" << std::endl;
        }
    }

    if (!retval.containsError())
    {
        emit parametersChanged(m_params);
        setIdentifier(QString("IP %1 @ Port %2").arg(IP).arg(port));
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
				registercounter = modbus_read_registers(m_pCTX, regAddr.at(i), regNb.at(i), tab_reg);
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
				registercounter = modbus_write_registers(m_pCTX, regAddr.at(i), regNb.at(i), tab_reg_nb);
				if (registercounter == regNb.at(i))
				{
					std::cout << "Write at Reg. " << regAddr.at(i) << " success! \n " << std::endl; 
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
