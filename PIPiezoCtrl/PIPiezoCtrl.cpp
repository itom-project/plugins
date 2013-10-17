#include "PIPiezoCtrl.h"
#include "pluginVersion.h"
//#ifdef __linux__
//    #include <unistd.h>
//#else
//    #include <windows.h>
//#endif

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#define PIDELAY 2
#define PI_READTIMEOUT 256


//----------------------------------------------------------------------------------------------------------------------------------
/*!
	\detail creates new instance of PIPiezoCtrlInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created PIPiezoCtrlInterface-instance is stored in *addInInst
    \return retOk
    \sa PIPiezoCtrl
*/
ito::RetVal PIPiezoCtrlInterface::getAddInInst(ito::AddInBase **addInInst)
{
    PIPiezoCtrl* newInst = new PIPiezoCtrl();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
	\detail Closes an instance of of PIPiezoCtrlInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa PIPiezoCtrl
*/
ito::RetVal PIPiezoCtrlInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
        delete ((PIPiezoCtrl *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call) 
	with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
PIPiezoCtrlInterface::PIPiezoCtrlInterface()
{
    m_type = ito::typeActuator;
    setObjectName("PIPiezoCtrl");

    m_description = QObject::tr("PI Piezos E662, E-816, E-621, E-625, E665");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"The PIPiezoCtrl is an itom-plugin, which can be used to communicate with PI piezo-controllers.\
Different PI-Piezo Controller (E-816, E-621, E-625, E-665 or E662) are implemented.\n\
\n\
It has been tested with different Piefocs and Piezo-stages. This system needs a serial port, which differs depending on the controller type. \
The parameters of the serial port (besides port number) are set automatically during initialization. \n\
\n\
WARNING: The calibration between applied voltage and desired position is depending on every single PI device and is stored in the corresponding \
PI controller. Therefore don't mix stages and controllers but only use the original, calibrated combination.";
	m_detaildescription = QObject::tr(docstring);

    m_author = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");    
    
    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An opened serial port (the right communcation parameters will be set by this piezo-controller).").toAscii().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class PIPiezoCtrlInterface with the name PIPiezoCtrlInterface as plugin for the Qt-System (see Qt-DOC)
Q_EXPORT_PLUGIN2(PIPiezoCtrlInterface, PIPiezoCtrlInterface)

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//! 
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
	creates new instance of dialogPIPiezoCtrl, calls the method setVals of dialogPIPiezoCtrl, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogPIPiezoCtrl
*/
const ito::RetVal PIPiezoCtrl::showConfDialog(void)
{
    DialogPIPiezoCtrl *confDialog = new DialogPIPiezoCtrl(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(parametersChanged(QMap<QString, ito::Param>)));
    //connect(confDialog, SIGNAL(sendParamVector(const QVector< QSharedPointer<ito::tParam> >,ItomSharedSemaphore*)), this, SLOT(setParamVector(const QVector<QSharedPointer<ito::tParam> >,ItomSharedSemaphore*)));

    QMetaObject::invokeMethod(this, "sendParameterRequest"); //requests plugin to send recent parameter map to dialog
    
    confDialog->exec();
    delete confDialog;
    confDialog = NULL;
    return ito::retOk;
}




//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
	with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the PIPiezoCtrl::init. The widged window is created at this position.
*/
PIPiezoCtrl::PIPiezoCtrl() :
    AddInActuator(),
    m_pSer(NULL),
    m_delayAfterSendCommandMS(0),
    m_dockWidget(NULL),
    m_getStatusInScan(true),
    m_getPosInScan(true),
    m_useOnTarget(true)
{
    ito::Param paramVal("name", ito::ParamBase::String, "PIPiezoCtrl", NULL);
    m_params.insert(paramVal.getName(), paramVal);
	
	m_scale = 1e3; // PI is programmed in µm, this evil Programm sents in mm
    m_async = 0;
    m_delayProp = 0.5; //s
    m_delayOffset = 0.02; //s
    m_hasHardwarePositionLimit = false;
    m_posLimitLow = -std::numeric_limits<double>::max();
    m_posLimitHigh = std::numeric_limits<double>::max();
    m_ctrlType = Unknown;
    m_numAxis = 1;

    paramVal = ito::Param("ctrlType", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Current type of controller, e.g. E-662, E-665, ...").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("ctrlName", ito::ParamBase::String | ito::ParamBase::Readonly, "unknwon", tr("device information string").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("piezoName", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("piezo information string").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("posLimitLow", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), m_posLimitLow, tr("lower position limit [mm] of piezo (this can be supported by the device or by this plugin)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("posLimitHigh", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), m_posLimitHigh, tr("higher position limit [mm] of piezo (this can be supported by the device or by this plugin)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("delayProp", ito::ParamBase::Double, 0.0, 10.0, m_delayProp, tr("delay [s] per step size [mm] (e.g. value of 1 means that a delay of 100ms is set for a step of 100mu)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("delayOffset", ito::ParamBase::Double, 0.0, 10.0, m_delayOffset, tr("offset delay [s] per movement (independent on step size)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("local", ito::ParamBase::Int, 0, 1, 0, tr("defines whether system is in local (1.0) or remote (0.0) mode.").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1.0) or sychronous (0.0) mode").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1, 1, tr("Number of axes (here always 1)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("PI_CMD", ito::ParamBase::String, "", tr("use this parameter followed by :YourCommand in order to read/write value from/to device (e.g. PI_CMD:ERR?)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("hasLocalRemote", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 1, tr("defines whether the device has the ability to switch between local and remote control (1), else (0)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("hasOnTargetFlag", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 1, 0, tr("defines whether the device has the ability to check the 'on-target'-status (1), else (0)").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("fastMode", ito::ParamBase::Int, 0, 3, 0, tr("Disabled position and end switch control during positioning for fast scans").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    m_targetPos = QVector<double>(1,0.0);
    m_currentStatus = QVector<int>(1, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);
    m_currentPos = QVector<double>(1,0.0);

    //now create dock widget for this plugin
    m_dockWidget = new DockWidgetPIPiezoCtrl(getID(), this);
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_dockWidget);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail It is used to set the parameter of type int/double with key "name" stored in m_params and the corresponding member variabels. 
			This function is defined by the actuator class and overwritten at this position.

	\param[in] *name		Name of parameter
	\param[out] val			New parameter value as double
	\param[in/out] *waitCond	Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal PIPiezoCtrl::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    QString paramName;
    bool hasIndex;
    int index;
    QString additionalTag;
    QByteArray answerString;
    double answerDouble;
    QVector<QPair<int, QByteArray> > lastError;

    retValue += apiParseParamName(key, paramName, hasIndex, index, additionalTag);

    if (hasIndex && index != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("this motor only has one axis, therefore it is not allowed to get a parameter with index unequal to 0").toAscii().data());
    }
    else if (paramName == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toAscii().data());
    }
    else if (paramName == "PI_CMD")
    {
        if (additionalTag == "")
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter PI_CMD requires the real command send to the motor after a colon-sign.").toAscii().data());
        }
        else
        {
            retValue += PISendQuestionWithAnswerString(additionalTag.toAscii(), answerString, 400);
            if (retValue.containsError())
            {
                retValue += PIGetLastErrors(lastError);
                retValue += convertPIErrorsToRetVal(lastError);
            }
            else
            {
                (*val).setVal<char*>(answerString.data(), answerString.length());
            }
        }
    }
    else if (paramName == "local")
    {
        switch(m_ctrlType)
        {
        case E662Family:
            retValue += PISendQuestionWithAnswerString("SYST:DEV:CONT?", answerString, 400);
            break;
		case E625Family:
			retValue += ito::RetVal(ito::retError, 3, tr("this device has no local/remote switch").toAscii().data());
			break;
        default:
            retValue += ito::RetVal(ito::retError, 2, tr("device type is unknown").toAscii().data());
            break;
        }

        if (retValue.containsError())
        {
            retValue += PIGetLastErrors(lastError);
            retValue += convertPIErrorsToRetVal(lastError);
        }
        else
        {
            if (answerString.contains("loc") || answerString.contains("LOC"))
            {
                (*val).setVal<int>(1);
            }
            else
            {
                (*val).setVal<int>(0);
            }
        }
    }
    else if (paramName == "posLimitLow")
    {
        switch(m_ctrlType)
        {
        case E662Family:
            retValue += PISendQuestionWithAnswerDouble("SOUR:POS:LIM:LOW?", answerDouble, 400);
            m_params["posLimitLow"].setVal<double>(answerDouble/1000.0);
            break;
		case E625Family:
			break;

        default:
            retValue += ito::RetVal(ito::retError, 2, tr("device type is unknown").toAscii().data());
            break;
        }
        if (retValue.containsError())
        {
            retValue += PIGetLastErrors(lastError);
            retValue += convertPIErrorsToRetVal(lastError);
        }
        else
        {
            *val = m_params["posLimitLow"];
        }
    }
    else if (paramName == "posLimitHigh")
    {
        switch(m_ctrlType)
        {
        case E662Family:
            retValue += PISendQuestionWithAnswerDouble("SOUR:POS:LIM:HIGH?", answerDouble, 400);
            m_params["posLimitHigh"].setVal<double>(answerDouble/1000.0);
            break;
		case E625Family:
			break;
        default:
            retValue += ito::RetVal(ito::retError, 2, tr("device type is unknown").toAscii().data());
            break;
        }
        if (retValue.containsError())
        {
            retValue += PIGetLastErrors(lastError);
            retValue += convertPIErrorsToRetVal(lastError);
        }
        else
        {
            *val = m_params["posLimitHigh"];
        }
    }
    else
    {
        QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(paramName);
        if (paramIt != m_params.constEnd())
        {
            *val = paramIt.value();
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
/*!
    \detail It is used to set the parameter of type char* with key "name" stored in m_params and the corresponding member variabels. 
			This function is defined by the actuator class and overwritten at this position.
			If the "ctrl-type" is set, PIPiezoCtrl::PISwitchType is executed.

	\param[in] *name		Name of parameter
	\param[in] *val			String with parameter
	\param[in] len			Length of the string
	\param[in/out] *waitCond	Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal PIPiezoCtrl::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    QString paramName;
    bool hasIndex;
    int index;
    int value;
    QString additionalTag;
    QVector<QPair<int, QByteArray> > lastError;

    retValue += apiParseParamName(key, paramName, hasIndex, index, additionalTag);

    if (paramName == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toAscii().data());
    }
    else if (paramName == "PI_CMD")
    {
        if (additionalTag == "")
        {
            if (m_params["PI_CMD"].getType() == (*val).getType())
            {
                char *buf = (*val).getVal<char*>();
                if (buf != NULL)
                {
                    additionalTag = QByteArray(buf);
                }
            }

            if (additionalTag == "")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("parameter PI_CMD requires the real command send to the motor after a colon-sign in the parameter name or as value (second parameter of setParam method).").toAscii().data());
            }
        }

        if (!retValue.containsError())
        {
            retValue += PISendCommand(additionalTag.toAscii());
        }
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(paramName);
        if (paramIt != m_params.end())
        {

            if (paramIt->getFlags() & ito::ParamBase::Readonly)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toAscii().data());
                goto end;
            }
			else if (val->isNumeric() && paramIt->isNumeric())
			{
				double curval = val->getVal<double>();
				if (curval > paramIt->getMax())
				{
				    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toAscii().data());
                    goto end;
				}
				else if (curval < paramIt->getMin())
				{
				    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toAscii().data());
                    goto end;
				}
				else if (paramName == "local")
                {
                    if (m_params["hasLocalRemote"].getVal<int>() > 0)
                    {
                        value = (*val).getVal<int>();

                        retValue += PISetOperationMode(value != 0);

                        if (retValue.containsError())
                        {
                            retValue += PIGetLastErrors(lastError);
                            retValue += convertPIErrorsToRetVal(lastError);
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("this device does not support the local/remote control mode switch").toAscii().data());
                    }
                }
                else if (paramName == "posLimitLow")
                {
                    switch(m_ctrlType)
                    {
                    case E662Family:
                        retValue += PISendCommand(QByteArray("SOUR:POS:LIM:LOW ").append(QByteArray::number((*val).getVal<double>() * 1000)));
                        if (retValue.containsError())
                        {
                            retValue += PIGetLastErrors(lastError);
                            retValue += convertPIErrorsToRetVal(lastError);
                        }
                        else
                        {
                            retValue += paramIt.value().copyValueFrom(&(*val));
                        }
                        break;
                    default:
                        retValue += paramIt.value().copyValueFrom(&(*val));
                        break;
                    }
                }
                else if (paramName == "posLimitHigh")
                {
                    switch(m_ctrlType)
                    {
                    case E662Family:
                        retValue += PISendCommand(QByteArray("SOUR:POS:LIM:HIGH ").append(QByteArray::number((*val).getVal<double>() * 1000)));
                        if (retValue.containsError())
                        {
                            retValue += PIGetLastErrors(lastError);
                            retValue += convertPIErrorsToRetVal(lastError);
                        }
                        else
                        {
                            retValue += paramIt.value().copyValueFrom(&(*val));
                        }
                        break;
                    default:
                        retValue += paramIt.value().copyValueFrom(&(*val));
                        break;
                    }
                }
                else if(paramName == "fastMode")
                {
                    paramIt.value().setVal<double>(curval);

                    m_getPosInScan = m_params["fastMode"].getVal<int>() > 1 ? false : true;
                    m_getStatusInScan = m_params["fastMode"].getVal<int>() > 0 ? false : true;
                    m_useOnTarget = m_params["fastMode"].getVal<int>() > 2 ? false : true;
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
				retValue += ito::RetVal(ito::retError, 0, tr("Given parameter and m_param do not have the same type").toAscii().data());
				goto end;
			}
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toAscii().data());
        }
    }
end:
    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    emit parametersChanged(m_params);
    return retValue;
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
ito::RetVal PIPiezoCtrl::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore *waitCond)
{   
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

//    m_pSer = qobject_cast<ito::AddInDataIO*>(reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>()));
//    if (m_pSer)
//    {
    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();

        retval += PIIdentifyAndInitializeSystem();
        if (!retval.containsError())
        {
            //retval += PISetOperationMode(false); //already done by identify system above
            retval += PICheckStatus();
            retval += RequestStatusAndPosition(true, false);
        }

    }
    else
    {
        retval += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toAscii().data());
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
/*! \detail close method which is called before that this instance is deleted by the PIPiezoCtrlInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIPiezoCtrl::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

	retval += PISetOperationMode(true); //reset to local mode

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

	\param [in] axis	Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIPiezoCtrl::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1,axis),waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for a set of axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

	\param [in] axis	Vector this numbers of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIPiezoCtrl::calib(const QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
{
	ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("not implemented").toAscii().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function sets the zero position of a single axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

	\param [in] axis	numbers of axis to set to zero
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIPiezoCtrl::setOrigin(const int axis, ItomSharedSemaphore * /*waitCond*/)
{
    return setOrigin(QVector<int>(1,axis), NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function sets the zero position of various axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

	\param [in] axis	Vector with numbers of axis to set to zero
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIPiezoCtrl::setOrigin(QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("not implemented").toAscii().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The PIStatus function is called internally.

	\param [out] status		Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
	\todo define the status value
*/
ito::RetVal PIPiezoCtrl::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = PICheckStatus();
    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Get the Position of a single axis spezified by axis. The value in device independet in mm.

	\param [in] axis		Axisnumber
	\param [out] pos		Current position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PIPiezoCtrl::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    double axpos = 0.0;
	
    ito::RetVal retval = ito::retOk;

	if (axis != 0)
	{
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toAscii().data());
    }
    else
	{
        retval += this->PISendQuestionWithAnswerDouble(m_PosQust, axpos, 200);
		*pos = (double)axpos / 1000;
        m_currentPos[0] = *pos;
	}

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

	\param [in] axis		Vector with axis numbers
	\param [out] pos		Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PIPiezoCtrl::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
	ito::RetVal retval = ito::retOk;
	QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);

	if ((axis.size() == 1) && (axis.value(0) == 0))
	{
		retval +=getPos(axis.value(0),sharedpos,0);
		(*pos)[0] = *sharedpos;
        m_currentPos[0] = *sharedpos;
	}
	else
	{
        retval +=ito::RetVal(ito::retError, 0, tr("Error. Too many Axis / wrong Axis").toAscii().data());
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
			This function calls PIPiezoCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

	\param [in] axis	axis number
	\param [in] pos		absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
	\sa PISetPos
    \return retOk
*/
ito::RetVal PIPiezoCtrl::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;
	double target_posMM = pos;
    retval = PISetPos(axis, target_posMM, false, waitCond);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
			If the size of the vector is more then 1 element, this function returns an error.
			This function calls PIPiezoCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

	\param [in] axis	1 Element Vector with axis numbers
	\param [in] pos		1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
	\sa PISetPos
    \return retOk
*/
ito::RetVal PIPiezoCtrl::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

	if (axis.size() > 1)
	{
		retval += ito::RetVal(ito::retError, 0, tr("Too many axis. This is currently a single axis device").toAscii().data());

        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
	}
	else
	{
		retval += setPosAbs(axis[0], pos[0], waitCond);
	}

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm. 
			This function calls PIPiezoCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

	\param [in] axis	axis number
	\param [in] pos		relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
	\sa PISetPos
    \return retOk
*/
ito::RetVal PIPiezoCtrl::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::retOk;

    retval = PISetPos(axis, pos, true, waitCond);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
			If the size of the vector is more then 1 element, this function returns an error.
			This function calls PIPiezoCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

	\param [in] axis	1 Element Vector with axis numbers
	\param [in] pos		1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
	\sa PISetPos
    \return retOk
*/
ito::RetVal PIPiezoCtrl::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);
	
	if (axis.size() > 1)
	{
		retval += ito::RetVal(ito::retError, 0, tr("Too many axis. This is currently a single axis device.").toAscii().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
	}
	else
	{
		retval += setPosRel(axis[0], pos[0], waitCond);
	}
    
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
			It sends the current postion and the status to the world.

	\sa PISetPos
    \return retOk
*/
ito::RetVal PIPiezoCtrl::RequestStatusAndPosition(bool sendActPosition, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

	QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
	*sharedpos = 0.0;

	retval += PICheckStatus();

    if (sendActPosition)
    {
	    retval += getPos(0,sharedpos,0);
		m_currentPos[0] = *sharedpos;

		if (std::abs(*sharedpos-m_targetPos[0]) > 0.01)
		{
			m_targetPos[0] = *sharedpos;
		}
		
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

/*!
updates the status flag (if possible). For E662 only the overflow status can be checked, the rest is determined by the movement
and a guess if the motor reached the target position
*/
ito::RetVal PIPiezoCtrl::PICheckStatus(void)
{
    ito::RetVal retVal = ito::retOk;
    double answerDbl;

    if (m_ctrlType == Unknown)
    {
        return ito::RetVal(ito::retError, 0, tr("controller device unknown").toAscii().data());
    }
    else if (m_ctrlType == E662Family)
    {
        m_identifier = QString("E662 (%1)").arg(getID());
        retVal += PISendQuestionWithAnswerDouble("*STB?", answerDbl, 200);
        if (!retVal.containsError())
        {
            if (answerDbl == 0)
            {
                setStatus(m_currentStatus[0],0, ito::actMovingMask | ito::actStatusMask); //reset end switch flags
            }
            else
            {
                setStatus(m_currentStatus[0], ito::actuatorEndSwitch | ito::actuatorRightEndSwitch, ito::actMovingMask | ito::actStatusMask); //set end switch (right end switch, left end switch has no signal from the motor)
            }

            setStatus(m_currentStatus[0], ito::actuatorAvailable | ito::actuatorEnabled , ito::actMovingMask | ito::actSwitchesMask);
        }
        else
        {
            setStatus(m_currentStatus[0], 0, ito::actMovingMask | ito::actSwitchesMask);
        }
    }
    else if (m_ctrlType == E625Family)
    {
        m_identifier = QString("E625 (%1)").arg(getID());
        retVal += PISendQuestionWithAnswerDouble("OVF? A", answerDbl, 200);

        if (!retVal.containsError())
        {
            if (answerDbl == 0)
            {
                setStatus(m_currentStatus[0],0, ito::actMovingMask | ito::actStatusMask); //reset end switch flags
            }
            else
            {
                setStatus(m_currentStatus[0], ito::actuatorEndSwitch | ito::actuatorRightEndSwitch, ito::actMovingMask | ito::actStatusMask); //set end switch (right end switch, left end switch has no signal from the motor)
            }

            setStatus(m_currentStatus[0], ito::actuatorAvailable | ito::actuatorEnabled , ito::actMovingMask | ito::actSwitchesMask);
        }
        else
        {
            setStatus(m_currentStatus[0], 0, ito::actMovingMask | ito::actSwitchesMask);
        }
    }

    return retVal;
}



//----------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------
// 

/*!
    \detail Clear serial port before writing without any delay
    \return retOk
*/
ito::RetVal PIPiezoCtrl::PIDummyRead(void) /*!< reads buffer of serial port without delay in order to clear it */
{
    int bufsize = 50;
    QSharedPointer<int> len(new int);
    *len = bufsize;
    QSharedPointer<char> buffer(new char[bufsize]);
    do
    {
        m_pSer->getVal(buffer, len, NULL);
    }
    while(*len > 0);
    

    //QSharedPointer<QVector<ito::ParamBase> > pMand(new QVector<ito::ParamBase>());
    //m_pSer->execFunc("clearInputBuffer", pMand, pMand, pMand, NULL);

    return ito::retOk;
}

ito::RetVal PIPiezoCtrl::PIReadString(QByteArray &result, int &len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    QTime timer;
    QByteArray endline;
    bool done = false;
    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";
    int curFrom = 0;
    int pos = 0;
    
    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retValue += m_pSer->getParam(param, NULL);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); //borrowed reference
        int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
        endline = QByteArray::fromRawData(temp,len);
        //
        //endline[0] = temp[0];
        //endline[1] = temp[1];
        //endline[2] = temp[2];
        //endline = endline.trimmed();
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("could not read endline parameter from serial port").toAscii().data());
    }

    if (!retValue.containsError())
    {
        len = 0;
        timer.start();

        while(!done && !retValue.containsError())
        {
            *curBufLen = buflen;
            retValue += m_pSer->getVal(curBuf, curBufLen, NULL);

            if (!retValue.containsError())
            {
                result += QByteArray(curBuf.data(), *curBufLen);
                pos = result.indexOf(endline, curFrom);
                curFrom = qMax(0, result.length() - 3);

                if (pos >= 0) //found
                {
                    done = true;
                    result = result.left(pos);   
                }
            }

			//qDebug() << "readString. done: "<<done<<", tempResult: "<<result;

            if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
            {
                retValue += ito::RetVal(ito::retError, PI_READTIMEOUT, tr("timeout").toAscii().data());
            }
        }

        len = result.length();

    }

    return retValue;
}

ito::RetVal PIPiezoCtrl::PIGetLastErrors(QVector<QPair<int,QByteArray> > &lastErrors)
{
    bool errorAvailable = true;
    ito::RetVal retValue(ito::retOk);
    QByteArray buffer;
    QByteArray errorText;
    int readSigns;
    int pos;
    int errorNo;
    lastErrors.clear();
    bool ok;

    while(errorAvailable && !retValue.containsError())
    {
        retValue += PISendCommand("ERR?");
        retValue += PIReadString(buffer, readSigns, 250);

        if (!retValue.containsError())
        {
            if (m_ctrlType == E662Family)
            {
                //buffer has form ErrorCode, "ErrorMsg"
                pos = buffer.indexOf(",");
                if (pos >= 0)
                {
                    errorNo = buffer.left(pos).toInt();
                    if (errorNo != 0)
                    {
                        errorText = buffer.mid(pos+3);
                        errorText.chop(1); //remove " from the end
                        lastErrors.append(QPair<int,QByteArray>(errorNo, errorText));
                    }
                    else
                    {
                        errorAvailable = false;
                    }
                }
                else
                {
                    lastErrors.append(QPair<int,QByteArray>(-1000, tr("error could not be parsed").toAscii().data()));
                }
            }
            else if (m_ctrlType == E625Family)
            {
                errorNo = buffer.toInt(&ok);
                if (ok)
                {
                    if (errorNo != 0)
                    {
                        switch(errorNo)
                        {
                        case 1: errorText = tr("Parameter syntax error").toAscii(); break;
                        case 2: errorText = tr("Unknown command").toAscii(); break;
                        case 3: errorText = tr("Command length out of limits or command buffer overrun").toAscii(); break;
                        case 5: errorText = tr("Unallowable move attempted on unreferenced axis, or move attempted with servo off").toAscii(); break;
                        case 10: errorText = tr("Controller was stopped by command").toAscii(); break;
                        case 15: errorText = tr("Invalid axis identifier").toAscii(); break;
                        case 17: errorText = tr("Parameter out of range").toAscii(); break;
                        case 20: errorText = tr("Macro not found").toAscii(); break;
                        case 54: errorText = tr("Unknown parameter").toAscii(); break;
                        case 56: errorText = tr("Password invalid").toAscii(); break;
                        case 60: errorText = tr("Protected Param: current Command Level (CCL) too low").toAscii(); break;
                        case 73: errorText = tr("Motion commands are not allowed when wave generator is active").toAscii(); break;
                        case 79: errorText = tr("Open-loop commands (SVA, SVR) are not allowed when servo is on").toAscii(); break;
                        case 89: errorText = tr("Command not allowed in current motion mode").toAscii(); break;
                        case 210: errorText = tr("Illegal file name (must be 8-0 format)").toAscii(); break;
                        case 232: errorText = tr("Save system configuration failed").toAscii(); break;
                        case 233: errorText = tr("Load system configuration failed").toAscii(); break;
                        case 306: errorText = tr("Error on I2C bus").toAscii(); break;
                        case 309: errorText = tr("Insufficient space to store macro").toAscii(); break;
                        case 405: errorText = tr("Wave parameter out of range").toAscii(); break;
                        }

                        lastErrors.append(QPair<int,QByteArray>(errorNo, errorText));
                    }
                    else
                    {
                        errorAvailable = false;
                    }
                }
                else
                {
                    lastErrors.append(QPair<int,QByteArray>(-1000, tr("error could not be parsed").toAscii().data()));
                }
            }
            else //unknown controller
            {
                errorNo = buffer.left(1).toInt();
                if (errorNo != 0)
                {
                    lastErrors.append(QPair<int,QByteArray>(-1001,tr( "unknown error").toAscii().data()));
                }
                else
                {
                    errorAvailable = false;
                }
            }
        }
    }

    return retValue;
}

ito::RetVal PIPiezoCtrl::PISendCommand(QByteArray command)
{
	ito::RetVal retVal = m_pSer->setVal(command.data(), command.length(), NULL);
	if (m_delayAfterSendCommandMS > 0)
	{
		QMutex mutex;
		mutex.lock();
		QWaitCondition waitCondition;
		waitCondition.wait(&mutex,m_delayAfterSendCommandMS);
		mutex.unlock();
	}
	return retVal;
}

/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string. 
			If string is invalid, val is not set and error-message is reported
	\param[in] *buf		Answer-String
	\param[out] val		double Value
	\sa PIPiezoCtrl::PIGetDouble
    \return retOk
*/
ito::RetVal PIPiezoCtrl::PISendQuestionWithAnswerDouble(QByteArray questionCommand, double &answer, int timeoutMS)
{
    int readSigns;
    QByteArray _answer;
    bool ok;
    ito::RetVal retValue = PISendCommand(questionCommand);
    retValue += PIReadString(_answer, readSigns, timeoutMS);

    answer = _answer.toDouble(&ok);

    if (retValue.containsError() && retValue.errorCode() == PI_READTIMEOUT)
    {
        QVector< QPair<int, QByteArray> > lastErrors;
        retValue += PIGetLastErrors(lastErrors);
        retValue = convertPIErrorsToRetVal(lastErrors);

    }
    else if (!ok)
    {
        retValue = ito::RetVal(ito::retError, 0, tr("value could not be parsed to a double value").toAscii().data());
    }

    return retValue;
}

/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string. 
			If string is invalid, val is not set and error-message is reported
	\param[out] *buf		Answer-String
	\param[in] bufsize		Number of signs to read
	\param[out] readsigns	Number of read signs
    \return retOk
*/
ito::RetVal PIPiezoCtrl::PISendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = PISendCommand(questionCommand);
    retValue += PIReadString(answer, readSigns, timeoutMS);

    if (retValue.containsError() && retValue.errorCode() == PI_READTIMEOUT)
    {
        QVector< QPair<int, QByteArray> > lastErrors;
        retValue += PIGetLastErrors(lastErrors);
        retValue = convertPIErrorsToRetVal(lastErrors);
    }

    return retValue;
}

ito::RetVal PIPiezoCtrl::convertPIErrorsToRetVal(QVector<QPair<int,QByteArray> > &lastErrors)
{
    if (lastErrors.size() > 0)
    {
        QByteArray errorString;
        QByteArray codeString;
        for (int i=0;i<lastErrors.size();i++)
        {
            if (errorString != "") errorString.append(", ");
            codeString = QByteArray::number(lastErrors[i].first);
            errorString.append("[").append(codeString).append("] '").append(lastErrors[i].second).append("'");
        }

        return ito::RetVal(ito::retError,0,errorString.data());
    }
    return ito::retOk;
}

ito::RetVal PIPiezoCtrl::PIIdentifyAndInitializeSystem(void)
{
    ito::RetVal retval = ito::retOk;
    QByteArray answer;
    QVector<QPair<int,QByteArray> > lastErrors;
    double tempDbl;

//    retval += PIDummyRead();

    //set serial settings
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud",ito::ParamBase::Int,9600)),NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits",ito::ParamBase::Int,8)),NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity",ito::ParamBase::Double,0.0)),NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits",ito::ParamBase::Int,1)),NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow",ito::ParamBase::Int,108)),NULL);
    retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline",ito::ParamBase::String,"\n")),NULL);

    //1. try to read *idn? in order to indentify device
    retval += PISendQuestionWithAnswerString("*idn?", answer, 2000);
    if (retval.containsError() || answer.length() < 5)
    {
		//clear error-queue
		PIGetLastErrors(lastErrors);
		lastErrors.clear();
		m_delayAfterSendCommandMS = 2; //small delay after sendCommands (else sometimes the controller didn't not understand the commands)
		retval = ito::retOk;
	}

    //2. try to read *idn? in order to indentify device
    retval += PISendQuestionWithAnswerString("*idn?", answer, 2000);
    if (retval.containsError() || answer.length() < 5)
    {
        retval = ito::retOk;
        //try to set baudrate to 115200
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 115200)), NULL);
        retval += PISendQuestionWithAnswerString("*idn?", answer, 2000);
		m_delayAfterSendCommandMS = 5; //small delay after sendCommands (else sometimes the controller didn't not understand the commands)
    }

    if (retval.containsError() || answer.length() < 5)
    {
        retval = ito::RetVal(ito::retError, 0, tr("could not identify controller. No answer for command *idn?").toAscii().data());
        return retval;
    }
	else
	{
		QSharedPointer<ito::Param> param(new ito::Param("port"));
		retval += m_pSer->getParam(param, NULL);
		if (retval.containsError() || param->getVal<int>() < 1)
		{
			retval = ito::RetVal(ito::retError, 0, tr("Could not read port number from serial port or port number invalid").toAscii().data());
			return retval;
		}
		else
		{
			m_params["comPort"].setVal<int>(param->getVal<int>()); 
		}
	}

    //clear error-queue
    PIGetLastErrors(lastErrors);
    lastErrors.clear();

    if (answer.contains("E-662"))
    {
        m_ctrlType = E662Family;

        m_AbsPosCmd = "POS";
        m_RelPosCmd = "POS:REL";
        m_PosQust = "POS?";

        m_params["hasLocalRemote"].setVal<int>(1.0);
        m_params["hasOnTargetFlag"].setVal<int>(0.0);
        m_params["ctrlType"].setVal<char*>("E662",strlen("E662"));
        m_hasHardwarePositionLimit = true;

        //set remote mode
        PISetOperationMode(false);

        retval += PISendCommand("SOUR:POS:LIM:STATE ON"); //activate limits
        retval += PISendQuestionWithAnswerDouble("SOUR:POS:LIM:LOW?", tempDbl, 200);
        if (!retval.containsError())
        {
            m_posLimitLow = tempDbl;
            m_params["posLimitLow"].setVal<double>(m_posLimitLow / 1000.0);
        }
        retval += PISendQuestionWithAnswerDouble("SOUR:POS:LIM:HIGH?", tempDbl, 200);
        if (!retval.containsError())
        {
            m_posLimitHigh = tempDbl;
            m_params["posLimitHigh"].setVal<double>(m_posLimitHigh / 1000.0);
        }

        retval += PISendQuestionWithAnswerString("SYST:DEV?", answer, 200);
        if (!retval.containsError())
        {
            m_params["ctrlName"].setVal<char*>(answer.data(),answer.length());
        }

        retval += PISendQuestionWithAnswerString("SYST:PZT?", answer, 200);
        if (!retval.containsError())
        {
            m_params["piezoName"].setVal<char*>(answer.data(),answer.length());
        }

        retval += PIGetLastErrors(lastErrors);
        retval += convertPIErrorsToRetVal(lastErrors);
    }
    else if (answer.contains("E816") || answer.contains("E625"))
    {
        m_ctrlType = E625Family;

        if (answer.contains("E816"))
        {
            m_params["ctrlType"].setVal<char*>("E816",strlen("E816"));
            m_params["hasOnTargetFlag"].setVal<int>(1.0);
        }
        else if (answer.contains("E625"))
        {
            m_params["ctrlType"].setVal<char*>("E625",strlen("E625"));
            m_params["hasOnTargetFlag"].setVal<int>(1.0);
        }

        m_AbsPosCmd = "MOV A";
		m_RelPosCmd = "MVR A";
		m_PosQust = "POS? A";

        m_params["hasLocalRemote"].setVal<int>(0.0);
        m_hasHardwarePositionLimit = false;

        //set remote mode
        PISetOperationMode(false);

        retval += PISendCommand("SVO A 1"); //activates servo
        m_params["posLimitLow"].setVal<double>(0.0 / 1000.0);
        m_params["posLimitHigh"].setVal<double>(100.0 / 1000.0);

        m_params["ctrlName"].setVal<char*>(answer.data(),answer.length());
        answer = "unknown";
        m_params["piezoName"].setVal<char*>(answer.data(),answer.length());

        retval += PIGetLastErrors(lastErrors);
        retval += convertPIErrorsToRetVal(lastErrors);

    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is used to switch between remote and local positioning, means handwheel or pc-command. 
    \return retOk
*/
ito::RetVal PIPiezoCtrl::PISetOperationMode(bool localNotRemote)
{
    ito::RetVal retValue(ito::retOk);

    if (m_ctrlType == Unknown)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("controller device unknown").toAscii().data());
    }
    else if (m_ctrlType == E662Family)
    {
        if (localNotRemote)
        {
            m_params["local"].setVal<int>(1);
            retValue += PISendCommand("SYST:DEV:CONT LOC");
        }
        else
        {
            m_params["local"].setVal<int>(0);
            retValue += PISendCommand("SYST:DEV:CONT REM");
        }
    }
    else if (m_ctrlType == E625Family)
    {
        //no support for that
    }

    return retValue;
}



//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the position (abs or rel) of a one axis spezified by "axis" to the position "dpos". The value in device independet in mm. 
			If the axisnumber is not 0, this function returns an error.

	\param [in] axis		axis number
	\param [in] dpos		target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PIPiezoCtrl::PISetPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    double dpos_temp = posMM * 1e3;	// Round value by m_scale
	ito::RetVal retval = ito::retOk;
    bool released = false;
    bool outOfRange = false;
    int delayTimeMS = 0;
    QByteArray cmdTotal;

	if (axis != 0)
	{
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toAscii().data());

        if (waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
    }
    else
	{
		retval += PIDummyRead();

        delayTimeMS = m_delayOffset /*in seconds*/ * 1000.0 + abs(posMM) * m_delayProp /*in seconds/mm*/ * 1000.0;

        if (relNotAbs)
        {
            cmdTotal = m_RelPosCmd;
            cmdTotal = cmdTotal.append(" ").append(QByteArray::number(dpos_temp, 'g'));

            if (m_hasHardwarePositionLimit == false && m_getPosInScan == true)
            {
                if (m_currentPos[0] + posMM > m_params["posLimitHigh"].getVal<double>() || m_currentPos[0] + posMM < m_params["posLimitLow"].getVal<double>())
                {
                    retval += ito::RetVal(ito::retError, 0, tr("the new position (rel) seems to be out of the allowed position range (software check only). Please check params 'posLimitHigh' and 'posLimitLow'").toAscii().data());
                    outOfRange = true;
                }
            }
            if (outOfRange == false) m_targetPos[0] += posMM;
        }
        else
        {
            cmdTotal = m_AbsPosCmd;
            cmdTotal = cmdTotal.append(" ").append(QByteArray::number(dpos_temp, 'g'));

            if (m_hasHardwarePositionLimit == false)
            {
                if (posMM > m_params["posLimitHigh"].getVal<double>() || posMM < m_params["posLimitLow"].getVal<double>())
                {
                    retval += ito::RetVal(ito::retError, 0, tr("the new position (abs) seems to be out of the allowed position range (software check only). Please check params 'posLimitHigh' and 'posLimitLow'").toAscii().data());
                    outOfRange = true;
                }
            }
            if (outOfRange == false) m_targetPos[0] = posMM;
        }

        if (outOfRange == false)
        {
            setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate(false);

            sendTargetUpdate();
            retval += PISendCommand(cmdTotal);
			retval += PIDummyRead();
        }

        QVector< QPair<int, QByteArray> > lastErrors;

        if(m_getStatusInScan)
        {
            retval += PIGetLastErrors(lastErrors);
            retval += convertPIErrorsToRetVal(lastErrors);
        }

		if (retval.containsError() && retval.errorCode() == PI_READTIMEOUT)
		{
			retval = ito::RetVal(ito::retOk);
			retval += PIDummyRead();
			retval += PIGetLastErrors(lastErrors);
			retval += convertPIErrorsToRetVal(lastErrors);
		}
        
        if (!retval.containsError())
        {
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }

            if (outOfRange == false)
            {
                retval += waitForDone(delayTimeMS, QVector<int>(1,axis)); //WaitForAnswer(60000, axis);
            }

            if (!m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
        else
        {
            replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorAtTarget);
            sendStatusUpdate(true);

            if (waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
	}
    return retval;
}


ito::RetVal PIPiezoCtrl::waitForDone(const int timeoutMS, const QVector<int> /*axis*/ /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retVal(ito::retOk);
    QMutex waitMutex;
    QWaitCondition waitCondition;
    bool atTarget = false;
    double answerDbl;
	int timeoutMS_ = timeoutMS;
    QSharedPointer<double> actPos = QSharedPointer<double>(new double);

    if (m_params["hasOnTargetFlag"].getVal<int>() > 0.0 && m_useOnTarget)
    {
        while(!atTarget && !retVal.containsError())
        {
            retVal += PISendQuestionWithAnswerDouble("ONT? A", answerDbl, 50);
            if (!retVal.containsError() && answerDbl > 0.0) 
            {
                atTarget = true;
            }
        }
    }

    while(timeoutMS_ > 0 && !atTarget)
    {
        //short delay
        waitMutex.lock();
        if (timeoutMS > 1000)
        {
            waitCondition.wait(&waitMutex, 1000);
            timeoutMS_ -= 1000;
        }
        else
        {
            waitCondition.wait(&waitMutex, timeoutMS);
            timeoutMS_ = 0;
        }
        waitMutex.unlock();
        setAlive();
    }

    replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorAtTarget);

    if(m_getStatusInScan)
    {
        retVal += PICheckStatus();
    }

    if(m_getPosInScan)
    {
        retVal += getPos(0, actPos, NULL);
    }

    m_targetPos[0] = *actPos;
    m_currentPos[0] = *actPos;
    sendStatusUpdate(false);
    sendTargetUpdate();

    return retVal;
}



//---------------------------------------------------------------------------------------------------------------------------------- 
void PIPiezoCtrl::dockWidgetVisibilityChanged(bool visible)
{
    if (m_dockWidget)
    {
        if (visible)
        {
            connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)), m_dockWidget, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            connect(this, SIGNAL(targetChanged(QVector<double>)), m_dockWidget, SLOT(targetChanged(QVector<double>)));
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), m_dockWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)), m_dockWidget, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            disconnect(this, SIGNAL(targetChanged(QVector<double>)), m_dockWidget, SLOT(targetChanged(QVector<double>)));
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), m_dockWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        }
    }
}
