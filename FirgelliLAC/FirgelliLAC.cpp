/* ********************************************************************
    Plugin "FirgelliLAC" for itom software
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

#include "FirgelliLAC.h"

#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qtimer.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>
#include <qdatetime.h>
//#include <fcntl.h>

#include <qdebug.h>

#define NOMINMAX
#include "mpusbapi.h"

typedef enum
{
	SET_ACCURACY                = 0x01,
	SET_RETRACT_LIMIT           = 0x02,
	SET_EXTEND_LIMT             = 0x03,
	SET_MOVEMENT_THRESHOLD      = 0x04,
	SET_STALL_TIME              = 0x05,
	SET_PWM_THRESHOLD           = 0x06,
	SET_DERIVATIVE_THRESHOLD    = 0x07,
	SET_DERIVATIVE_MAXIMUM      = 0x08,
	SET_DERIVATIVE_MINIMUM      = 0x09,
	SET_PWM_MAXIMUM             = 0x0A,
	SET_PWM_MINIMUM             = 0x0B,
	SET_PROPORTIONAL_GAIN       = 0x0C,
	SET_DERIVATIVE_GAIN         = 0x0D,
	SET_AVERAGE_RC              = 0x0E,
	SET_AVERAGE_ADC             = 0x0F,
	GET_FEEDBACK                = 0x10,
	SET_POSITION                = 0x20,
	SET_SPEED                   = 0x21,
	DISABLE_MANUAL              = 0x30,
	RESET                       = 0xFF,
}TYPE_CMD;

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of FirgelliLACInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created FirgelliLACInterface-instance is stored in *addInInst
    \return retOk
    \sa FirgelliLAC
*/
ito::RetVal FirgelliLACInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ito::RetVal retValue;
    if (m_InstList.size() <= 0) //the first instance is instantiated
    {
        retValue += loadDLL();
    }


    NEW_PLUGININSTANCE(FirgelliLAC)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of FirgelliLACInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa FirgelliLAC
*/
ito::RetVal FirgelliLACInterface::closeThisInst(ito::AddInBase **addInInst)
{

    ito::RetVal retValue;
    REMOVE_PLUGININSTANCE(FirgelliLAC)

    if (m_InstList.size() <= 0)
    {
        retValue += unloadDLL(); //the unloading with a windows machine needs a lot of time, therefore we don't do it here
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
FirgelliLACInterface::FirgelliLACInterface()
{
    m_type = ito::typeActuator;

    setObjectName("FirgelliLAC");

    m_description = QObject::tr("FirgelliLAC");
    m_detaildescription = QObject::tr("FirgelliLAC is an itom-plugin, which can be used to communicate with the Firgelli USB controllers.\
\n\
It has been tested with one connected motor PQ12-SS-GG-VV-P with one axis.\n\
\n\
Please indicate the correct working distance of the connected motor (e.g. 20mm) at startup, else the\n\
plugin might deliver wrong values. At startup, the motor is always moved to its zero-position\n\
in order to be able to subsequently provide right position values. This behaviour is mandatory\n\
and cannot be changed (due to the driver of the controller board).");

    m_author = "H. Bieger, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);    
    
    m_initParamsMand.append(ito::Param("spoolMax", ito::ParamBase::Double, 20.0, new ito::DoubleMeta(0.0,100000.0), tr("Maximum length of spool (mm) [default 20.0 mm]").toLatin1().data()));

    m_initParamsOpt.append(ito::Param("deviceNum", ito::ParamBase::Int, -1, 127, -1, tr("The current number of this specific device, if there are more than one devices connected. (0 = first device)").toLatin1().data()));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FirgelliLACInterface::loadDLL()
{
    ito::RetVal retValue(ito::retOk);
    if (mLib.isLoaded())
    {
        return ito::retOk;
    }

#ifdef _WIN64
    mLib.setFileName("E:\\itom\\sources\\plugins\\FirgelliLAC\\Mpusbapi\\win64\\mpusbapi.dll");
#elif defined WIN32
    mLib.setFileName("E:\\itom\\sources\\plugins\\FirgelliLAC\\Mpusbapi\\win32\\mpusbapi.dll");
#endif

    if (!mLib.load())
    {
        retValue += ito::RetVal(ito::retError, 1, tr("%1").arg(mLib.errorString()).toLatin1().data());
    }
    else
    {
        MPUSBGetDLLVersion = (DWORD(*)(void)) mLib.resolve("_MPUSBGetDLLVersion");
        MPUSBGetDeviceCount = (DWORD(*)(PCHAR)) mLib.resolve("_MPUSBGetDeviceCount");
        MPUSBOpen = (HANDLE(*)(DWORD, PCHAR, PCHAR, DWORD, DWORD)) mLib.resolve("_MPUSBOpen");
        MPUSBRead = (DWORD(*)(HANDLE, PVOID, DWORD, PDWORD, DWORD)) mLib.resolve("_MPUSBRead");
        MPUSBWrite = (DWORD(*)(HANDLE, PVOID, DWORD, PDWORD, DWORD)) mLib.resolve("_MPUSBWrite");
        MPUSBReadInt = (DWORD(*)(HANDLE, PVOID, DWORD, PDWORD, DWORD)) mLib.resolve("_MPUSBReadInt");
        MPUSBClose = (BOOL(*)(HANDLE)) mLib.resolve("_MPUSBClose");
        MPUSBGetDeviceDescriptor = (DWORD(*)(HANDLE, PVOID, DWORD, PDWORD)) mLib.resolve("_MPUSBGetDeviceDescriptor");
        MPUSBGetConfigurationDescriptor = (DWORD(*)(HANDLE, UCHAR, PVOID, DWORD, PDWORD)) mLib.resolve("_MPUSBGetConfigurationDescriptor");
        MPUSBGetStringDescriptor = (DWORD(*)(HANDLE, UCHAR, USHORT, PVOID, DWORD, PDWORD)) mLib.resolve("_MPUSBGetStringDescriptor");
        MPUSBSetConfiguration = (DWORD(*)(HANDLE, USHORT)) mLib.resolve("_MPUSBSetConfiguration");
        
        //check (some) for success:
        if (!(MPUSBGetDLLVersion))
        {
            retValue += ito::RetVal(ito::retError, 1, tr("error while loading DLL 'mpusbapi.dll': %1").arg(mLib.errorString()).toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FirgelliLACInterface::unloadDLL()
{
    if (mLib.isLoaded())
    {
        if (!mLib.unload())
        {
            return ito::RetVal(ito::retWarning, 2, tr("DLL could not be unloaded").toLatin1().data());
        }

        MPUSBGetDLLVersion = 0;
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class FirgelliLACInterface with the name FirgelliLACInterface as plugin for the Qt-System (see Qt-DOC)

        
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//! 
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
    creates new instance of dialogFirgelliLAC, calls the method setVals of dialogFirgelliLAC, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogFirgelliLAC
*/

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FirgelliLAC::LACWrite(const BYTE command, const int value, const bool checkAnswer /*= false*/)
{
    ito::RetVal retValue = ito::retOk;
	BYTE bufData[3];
	bufData[0] = command;
	bufData[1] = value & 0xff; //(BYTE)value;
	bufData[2] = (value >> 8) & 0xff; //(BYTE)(value >> 8);
	DWORD bufLen = sizeof(bufData);
	DWORD bufProcessed;

    if (!MPUSBWrite(LACOutpipe, bufData, bufLen, &bufProcessed, 1000))
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Error while writing").toLatin1().data());
    }
    else if (checkAnswer)
    {
        int val;
        BYTE cmd;
        LACRead(cmd, val);

//qDebug() << cmd << val;

        if (cmd != command)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Communication error: Wrong command").toLatin1().data());
        }
        else if (val != value)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Communication error: Wrong answer").toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FirgelliLAC::LACRead(BYTE &retCommand, int &retAnswer)
{
    ito::RetVal retValue = ito::retOk;
	BYTE bufData[3];
	DWORD bufLen = sizeof(bufData);
	DWORD bufProcessed;

    if (!MPUSBRead(LACInpipe, bufData, bufLen, &bufProcessed, 1000))
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Error while reading").toLatin1().data());
    }
    else
    {
        retCommand = bufData[0];
        retAnswer = bufData[1] + bufData[2] * 256;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the FirgelliLAC::init. The widged window is created at this position.
*/
ito::RetVal FirgelliLAC::LACCheckError(ito::RetVal retval)
{
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the FirgelliLAC::init. The widged window is created at this position.
*/
FirgelliLAC::FirgelliLAC() :
    AddInActuator(),
    m_pSer(NULL),
    LACOutpipe(NULL),
    LACInpipe(NULL),
    m_pos(0.0),
    m_targetSteps(0),
    m_async(0)
{
    // Read only - Parameters
    m_params.insert("name", ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "FirgelliLAC", tr("Name of the plugin").toLatin1().data()));
    m_params.insert("deviceID", ito::Param("device_id", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Name of controller").toLatin1().data()));
    m_params.insert("dllVersion", ito::Param("dll_version", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Version of DLL file").toLatin1().data()));
    m_params.insert("spoolMax", ito::Param("spoolMax", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 100000.0, 20.0, tr("Maximum length of spool (mm)").toLatin1().data()));
    m_params.insert("deviceNum", ito::Param("device_num", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 10, 0, tr("The current number of this specific device, if there are more than one devices connected. (0 = first device)").toLatin1().data()));

    // Read/Write - Parameters
    m_params.insert("async", ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or sychronous (0) mode").toLatin1().data()));
    m_params.insert("speed", ito::Param("speed", ito::ParamBase::Double, 0.0, 100.0, 0.0, tr("Target speed in %; range: 0.0..100.0 %").toLatin1().data()));
    m_params.insert("accuracy", ito::Param("accuracy", ito::ParamBase::Double, 0.0, 100.0, 0.0, tr("Accuracy in %; range: 0.0..100.0 %").toLatin1().data()));

    m_currentStatus = QVector<int>(1, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);

    m_currentPos.fill(0.0, 1);
    m_currentStatus.fill(0, 1);
    m_targetPos.fill(0.0, 1);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetFirgelliLAC *dockWidget = new DockWidgetFirgelliLAC(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \todo check if (*paramsMand)[0] is a serial port
    \return retOk
*/
ito::RetVal FirgelliLAC::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{   
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    bool deviceOpen = false;
	PCHAR pipename = const_cast<char*>("\\MCHP_EP1");
    PCHAR VidPid = const_cast<char*>("vid_04d8&pid_fc5f");

    double spoolMax = paramsMand->value(0).getVal<double>(); // 0: mand parameter "spoolMax"
    int deviceNum = paramsOpt->value(0).getVal<int>(); //  0: opt parameter "deviceNum"

    int dllVersionInt = MPUSBGetDLLVersion();
    QString dllVersion = QString("v%1.%2.%3.%4").arg((dllVersionInt >> 24) & 255).arg((dllVersionInt >> 16) & 255).arg((dllVersionInt >> 8) & 255).arg(dllVersionInt & 255);
    retval += m_params["dllVersion"].setVal<char*>(dllVersion.toLatin1().data());

    int deviceCount = MPUSBGetDeviceCount(VidPid);

    if (deviceCount == 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("No device connected").toLatin1().data());
    }

    if (!retval.containsError())
    {
        if (deviceNum == -1)
        {
            do
            {
                deviceNum++;
                LACOutpipe = MPUSBOpen(deviceNum, VidPid, pipename, MP_WRITE, 0);
            }
            while ((LACOutpipe == INVALID_HANDLE_VALUE) && (deviceNum < 128));
        }
        else
        {
        	LACOutpipe = MPUSBOpen(deviceNum, VidPid, pipename, MP_WRITE, 0);
        }

        if (LACOutpipe == INVALID_HANDLE_VALUE)
        {
            retval += ito::RetVal(ito::retError, 0, tr("No device found").toLatin1().data());
        }
        else
        {
            retval += m_params["deviceNum"].setVal<int>(deviceNum);
	        LACInpipe = MPUSBOpen(deviceNum, VidPid, pipename, MP_READ, 0);

            if (LACInpipe == INVALID_HANDLE_VALUE)
            {
                retval += ito::RetVal(ito::retError, 0, tr("InPipe is not initialized").toLatin1().data());
            }
            else
            {
                deviceOpen = true;
            }
        }
    }

    if (deviceOpen)
    {
        retval += m_params["spoolMax"].setVal<double>(spoolMax);
        retval += m_params["deviceID"].setVal<const char*>("FirgelliLAC");  // ToDo!
        retval += m_params["speed"].setVal<double>(50.0);
        retval += m_params["accuracy"].setVal<double>(98.0);
        
        retval += LACWrite(SET_SPEED, qRound(1023 * m_params["speed"].getVal<double>() / 100.0), true);
        retval += LACWrite(SET_ACCURACY, 1024 - qRound(1023 * m_params["accuracy"].getVal<double>() / 100.0), true);

        retval += LACSetPos(0.0, false);
    }
    
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal FirgelliLAC::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogFirgelliLAC(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail close method which is called before that this instance is deleted by the FirgelliLACInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal FirgelliLAC::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if ((LACOutpipe != INVALID_HANDLE_VALUE) && (!MPUSBClose(LACOutpipe)))
    {
        retval += ito::RetVal(ito::retError, 0, tr("Error closing device").toLatin1().data());
    }

    if ((LACInpipe != INVALID_HANDLE_VALUE) && (!MPUSBClose(LACInpipe)))
    {
        retval += ito::RetVal(ito::retError, 0, tr("Error closing device").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params and the corresponding member variabels. 
            This function is defined by the actuator class and overwritten at this position.

    \param[in] *name        Name of parameter
    \param[out] val            New parameter value as double
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal FirgelliLAC::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

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

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type char* with key "name" stored in m_params and the corresponding member variabels. 
            This function is defined by the actuator class and overwritten at this position.
            If the "ctrl-type" is set, FirgelliLAC::SMCSwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal FirgelliLAC::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index = 0;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;
    QVector<QPair<int, QByteArray> > lastitError;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

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
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
        //---------------------------
        if (key == "async")
        {
            m_async = val->getVal<int>();
            retValue += it->copyValueFrom(&(*val));
        }

        //---------------------------
        else if (key == "speed")
        {
            retValue += LACWrite(SET_SPEED, qRound(1023 * val->getVal<double>() / 100.0), true);
            retValue += it->copyValueFrom(&(*val));
        }
        //---------------------------

        else if (key == "accuracy")
        {
            retValue += LACWrite(SET_ACCURACY, 1024 - qRound(1023 * val->getVal<double>() / 100.0), true);
            retValue += it->copyValueFrom(&(*val));
        }

        //---------------------------
        else
        {
            retValue += it->copyValueFrom(&(*val));
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
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal FirgelliLAC::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1,axis),waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for a set of axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Vector this numbers of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal FirgelliLAC::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Not implemented, use calibmode to set actual position as zero.").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The SMCStatus function is called internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \todo define the status value
*/
ito::RetVal FirgelliLAC::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = LACCheckStatus();

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a single axis spezified by axis. The value in device independet in mm.

    \param [in] axis        Axisnumber
    \param [out] pos        Current position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal FirgelliLAC::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    QSharedPointer<QVector<double> > pos_(new QVector<double>(1, 0.0));
    ito::RetVal retval = getPos(QVector<int>(1, axis), pos_, NULL);
    *pos = (*pos_)[0];

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a set of axis spezified by "axis". The value in device independet in mm. 
            In this case if more than one axis is specified this function returns an error.

    \param [in] axis        Vector with axis numbers
    \param [out] pos        Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal FirgelliLAC::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
//    result_t result;
    int gPosition = 0;

    if (axis.size() != 1 || axis[0] != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("This device have only one axis").toLatin1().data());
    }
    else
    {
        m_currentPos[0] = m_pos;
        (*pos)[0] = m_currentPos[0];
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
            This function calls FirgelliLAC::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis     axis number
    \param [in] pos      absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal FirgelliLAC::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return LACSetPos(pos, false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls FirgelliLAC::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal FirgelliLAC::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    if (axis.size() > 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Too much axes given!").toLatin1().data());
    }
    else
    {
        return LACSetPos(pos[0], false, waitCond);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
            This function calls FirgelliLAC::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal FirgelliLAC::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return LACSetPos(pos, true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls FirgelliLAC::SMCSetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa SMCSetPos
    \return retOk
*/
ito::RetVal FirgelliLAC::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    if (axis.size() > 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Too much axes given!").toLatin1().data());
    }
    else
    {
        return LACSetPos(pos[0], true, waitCond);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa SMCSetPos
    \return retOk
*/
ito::RetVal FirgelliLAC::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
    *sharedpos = 0.0;

    retval += LACCheckStatus();

    if (sendCurrentPos)
    {
        retval += getPos(0, sharedpos, 0);
        m_currentPos[0] = *sharedpos;
        
        sendStatusUpdate(false);
    }
    else
    {
        sendStatusUpdate(true);
    }

    if (sendTargetPos)
    {
        sendTargetUpdate();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
*/
ito::RetVal FirgelliLAC::LACCheckStatus()
{
    ito::RetVal retVal = LACCheckError(ito::retOk);

    setStatus(m_currentStatus[0], ito::actuatorAvailable, ito::actSwitchesMask | ito::actMovingMask);

//    requestStatusAndPosition(true, true);
    sendStatusUpdate(false);

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal FirgelliLAC::setOrigin(const int axis, ItomSharedSemaphore * waitCond)
{
    return setOrigin(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal FirgelliLAC::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Not implemented, use calibmode to set actual position as zero.").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the position (abs or rel) of a one axis spezified by "axis" to the position "dpos". The value in device independet in mm. 
            If the axisnumber is not 0, this function returns an error.

    \param [in] axis        axis number
    \param [in] dpos        target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal FirgelliLAC::LACSetPos(const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
    bool released = false;
    double newPos = 0.0;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("The motor is running. Further action is not possible!").toLatin1().data());
    }
    else
    {
        setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate(false);

        if (relNotAbs)
        {   // Relative movement
            newPos = m_targetPos[0] + posMM;
        }
        else
        {   // Absolute movement
            newPos = posMM;
        }

        if (newPos > m_params["spoolMax"].getVal<double>())
        {
            retval += ito::RetVal(ito::retError, 0, tr("The target position %1 is higher than spool maximum %2").arg(newPos).arg(m_params["spoolMax"].getVal<double>()).toLatin1().data());
        }
        else
        {
            m_targetPos[0] = newPos;
            sendTargetUpdate();

            m_targetSteps = qRound(m_targetPos[0] / m_params["spoolMax"].getVal<double>() * 1023);
            retval += LACWrite(SET_POSITION, m_targetSteps);
        }
    }
        
    if (!retval.containsError())
    {
        // release semaphore immediately
        if (m_async && waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
        
        retval += waitForDone(-1, QVector<int>(1, 0)); 
        // Wait till movement is done and the release the semaphore
        if (!m_async && waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
    }
    else
    {
        replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorInterrupted);
        sendStatusUpdate(false);
    }

    if (!retval.containsError())
    {
            m_currentPos[0] = m_pos;
            sendStatusUpdate(false);
    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        released = true;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------
ito::RetVal FirgelliLAC::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retval(ito::retOk);
    bool done = false;
    BYTE cmd;
    int answer = 0;
    int stepActually = 0;
    int sameStep = 0;

    while (!done && !retval.containsWarningOrError())
    {   
        retval += LACRead(cmd, answer);

        if (!retval.containsError())
        {
            if (cmd == SET_POSITION)
            {
                m_pos = answer * m_params["spoolMax"].getVal<double>() / 1023;
                done = (answer == m_targetSteps);

                if (!done)
                {
                    // check if the motor is still running
                    if (stepActually != answer)
                    {
                        // motor is running
                        stepActually = answer;
                        sameStep = 0;
                    }
                    else
                    {
                        // motor isn't running
                        sameStep++;

                        // after 3 times the motor will running no longer
                        done = (sameStep == 3);
                    }
                }
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("Error while setting position").toLatin1().data());
            }
        }

        retval += LACCheckStatus();

        if (isInterrupted())
        {
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);

            retval += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;

            sendStatusUpdate(true);
        }
        else
        {
            if (done)
            {   // Position reached and movement done
                replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget);
                sendStatusUpdate(true);
                break;
            }
            else
            {
                Sleep(50);
                retval += LACWrite(SET_POSITION, m_targetSteps);
            }

            if (!retval.containsError())
            {
                setAlive();
            }
        }
    }

    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void FirgelliLAC::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            connect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
            emit parametersChanged(m_params);
            requestStatusAndPosition(true, true);
        }
        else
        {
            QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
        }
    }
}

