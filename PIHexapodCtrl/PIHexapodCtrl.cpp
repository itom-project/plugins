#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "PIHexapodCtrl.h"

#define NOMINMAX

#include "pluginVersion.h"
#include "gitVersion.h"
#include <QHostAddress>

#ifndef WIN32
    #include <unistd.h>
#else
    #include <windows.h>
#endif

#include <qmath.h>

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <QElapsedtimer>
#include <qtimer.h>
#include <qwaitcondition.h>

#include "common/apiFunctionsInc.h"
#include "common/helperCommon.h"
#include "common/sharedStructuresQt.h"

#define PI_READTIMEOUT 256
#define PI_INTERRUPT 512

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail creates new instance of PIHexapodCtrlInterface and returns the instance-pointer
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created PIHexapodCtrlInterface-instance is stored in *addInInst
    \return retOk
    \sa PIHexapodCtrl
*/
ito::RetVal PIHexapodCtrlInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PIHexapodCtrl)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Closes an instance of of PIHexapodCtrlInterface. This instance is given by parameter addInInst.
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa PIHexapodCtrl
*/
ito::RetVal PIHexapodCtrlInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(PIHexapodCtrl)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
PIHexapodCtrlInterface::PIHexapodCtrlInterface()
{
    m_type = ito::typeActuator;
    setObjectName("PIHexapodCtrl");

    m_description = tr("PI Hexapods H810, H824, H840, H850");
    m_detaildescription = tr("The PIHexapodCtrl is an itom-plugin, which can be used to communicate with PI Hexapod-controllers.\n\
Different PI-Hexapod Controller (E-816, E-621, E-625, E-665 or E662) are implemented,\n\
It is used to work with Piefocs and Hexapod-Stages. The ITO-Controllers have only one axis with axis number 0.\n\
This system needs a serial port, which differs depending on the controller type. The parameter are set automatically during initialization.\n\
It is initialized by actuator(\"PIHexapodCtrl\", SerialIO, Controller Type (e.g. 'E662')).\n\
Stageparamters can be set directly by setParam(\"STAGEPARAMETER\", value).\n\
WARNING: The calibration of voltage to position are hardcoded into the controller according to its corresponding stage.\n\
Hence, stages should not be mixed up.\n\
\n\
If the device is newly started, all axes have to be initialized first. This is done using the command myMotor.calib(0,1,2,3,4,5) in order \n\
to initialize the x,y,z,u,v and w axis, respectively.");
    m_author = "W. Lyda, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("LGPL");
    m_aboutThis = tr(GITVERSION);

    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An opened serial port (If connected via Serial-Port).").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("IP", ito::ParamBase::String, NULL, tr("The IP-address of the PI-Controller (If connected via TCP-IP).").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("Port", ito::ParamBase::Int, 0, 65000, 50000, tr("The TCP/IP-Port of the PI-Controller (If connected via TCP-IP).").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class PIHexapodCtrlInterface with the name PIHexapodCtrlInterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//!
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
    creates new instance of dialogPIHexapodCtrl, calls the method setVals of dialogPIHexapodCtrl, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogPIHexapodCtrl
*/
const ito::RetVal PIHexapodCtrl::showConfDialog(void)
{
    //DialogPIHexapodCtrl *confDialog = new DialogPIHexapodCtrl(this);

    //connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(parametersChanged(QMap<QString, ito::Param>)));
    ////connect(confDialog, SIGNAL(sendParamVector(const QVector< QSharedPointer<ito::tParam> >,ItomSharedSemaphore*)), this, SLOT(setParamVector(const QVector<QSharedPointer<ito::tParam> >,ItomSharedSemaphore*)));

    //QMetaObject::invokeMethod(this, "sendParameterRequest"); //requests plugin to send recent parameter map to dialog
    //
    //confDialog->exec();
    //delete confDialog;
    //confDialog = NULL;
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call)
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the PIHexapodCtrl::init. The widged window is created at this position.
*/
PIHexapodCtrl::PIHexapodCtrl() :
    AddInActuator(),
    m_pSer(NULL),
    m_delayAfterSendCommandMS(0),
    m_dockWidget(NULL),
    m_useTCPIP(false),
    m_connection(NULL),
    m_tcpAddr(""),
    m_tcpPort(0),
    m_closing(false),
    m_numAxis(6),
    m_isInit(false),
    m_doWait(true)
{
    //register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>() << ito::Param("xPosition", ito::ParamBase::Double | ito::ParamBase::In, 0.0, new ito::DoubleMeta(-1e308,1e308), tr("Position of the Pivot-Point in x").toLatin1().data())
                                                      << ito::Param("yPosition", ito::ParamBase::Double | ito::ParamBase::In, 0.0, new ito::DoubleMeta(-1e308,1e308), tr("Position of the Pivot-Point in y").toLatin1().data())
                                                      << ito::Param("zPosition", ito::ParamBase::Double | ito::ParamBase::In, 0.0, new ito::DoubleMeta(-1e308,1e308), tr("Position of the Pivot-Point in z").toLatin1().data());
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc("setPivotPoint", pMand, pOpt, pOut, tr("Set the system Pivot-Point (origin of rotation)"));
    pMand.clear();
    pOpt.clear();
    pOut.clear();

    pOut = QVector<ito::Param>() << ito::Param("xPosition", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, new ito::DoubleMeta(-1.0e308,1.0e308), tr("Position of the Pivot-Point in x").toLatin1().data())
                                                     << ito::Param("yPosition", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, new ito::DoubleMeta(-1.0e308,1.0e308), tr("Position of the Pivot-Point in y").toLatin1().data())
                                                     << ito::Param("zPosition", ito::ParamBase::Double | ito::ParamBase::Out, 0.0, new ito::DoubleMeta(-1.0e308,1.0e308), tr("Position of the Pivot-Point in z").toLatin1().data());
    registerExecFunc("getPivotPoint", pMand, pOpt, pOut, tr("Get the system Pivot-Point (origin of rotation)"));

    pMand.clear();
    pOpt.clear();
    pOut.clear();

    pMand = QVector<ito::Param>() << ito::Param("cycles", ito::ParamBase::Int | ito::ParamBase::In, 0.0, new ito::IntMeta(0,10), tr("Cycle to iterate").toLatin1().data())
                                                      << ito::Param("amplitude", ito::ParamBase::Double | ito::ParamBase::In, 0.0, new ito::DoubleMeta(0.1,20), tr("Amplitude in mm").toLatin1().data())
                                                      << ito::Param("timeconstant", ito::ParamBase::Double | ito::ParamBase::In, 0.0, new ito::DoubleMeta(0.0001,20), tr("Wait between to command in seconds").toLatin1().data());
    registerExecFunc("beFunny", pMand, pOpt, pOut, tr("Print the current positions of the specified axis to the command line"));

    pMand.clear();
    pOpt.clear();
    pOut.clear();


    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "PIHexapodCtrl", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    m_scale = 1; // PI is programmed in µm, this evil Programm sents in mm
    m_async = 0;
    m_hasHardwarePositionLimit = false;

    paramVal = ito::Param("identifier", ito::ParamBase::String | ito::ParamBase::Readonly, "unknown", tr("Identifier string of device (*idn?)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1.0) or sychronous (0.0) mode").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 6, 6, 6, tr("Number of axes (here always 1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("speed", ito::ParamBase::Double, 0.0, 100.0, 2.0, tr("speed of every axis in physical units / s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("PI_CMD", ito::ParamBase::String, "", tr("use this parameter followed by :YourCommand in order to read/write value from/to device (e.g. PI_CMD:ERR?)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 11, 0, tr("Number of axes attached to this stage (see axesNames for names of axes)").toLatin1().data());
    ito::IntMeta *imeta = paramVal.getMetaT<ito::IntMeta>();
    imeta->setCategory("General");
    imeta->setRepresentation(ito::ParamMeta::PureNumber); //numaxis should be a spin box and no slider in any generic GUI
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("axesNames", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("semicolon-separated list of the names of all connected axes").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

	double pivotPoint[] = { 0.0, 0.0, 0.0 };
	paramVal = ito::Param("pivotPoint", ito::ParamBase::DoubleArray, 3, pivotPoint, tr("current pivot point (x,y,z)").toLatin1().data());
	paramVal.setMeta(new ito::DoubleArrayMeta(-500.0, 500.0, 0.0, 3, 3, 1), true);
	m_params.insert(paramVal.getName(), paramVal);

    m_targetPos = QVector<double>(11,0.0);
    m_currentStatus = QVector<int>(11, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);
    m_currentPos = QVector<double>(11,0.0);

    if (hasGuiSupport())
    {
        // This is for the docking widged
        //now create dock widget for this plugin
        DockWidgetPIHexapodCtrl *dockWidget = new DockWidgetPIHexapodCtrl(getID(), this);    // Create a new non-modal dialog

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dockWidget);    // Give the widget a name ..)

        // till here
    }
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
ito::RetVal PIHexapodCtrl::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        if (key == "PI_CMD")
        {
            QByteArray answerString;

            if (suffix == "")
            {
                retValue += ito::RetVal(ito::retError, 0, tr("parameter PI_CMD requires the real command send to the motor after a colon-sign.").toLatin1().data());
            }
            else
            {
                retValue += PISendQuestionWithAnswerString(suffix.toLatin1(), answerString, 400);
                if (retValue.containsError())
                {
                    QVector<QPair<int, QByteArray> > lastError;
                    retValue += PIGetLastErrors(lastError);
                    retValue += convertPIErrorsToRetVal(lastError);
                }
                else
                {
                    *val = ito::Param("PI_CMD", ito::ParamBase::String, "", "");
                    val->setVal<const char*>(answerString.constData(), answerString.length());
                }
            }
        }
        else
        {
            if (key == "speed")
            {
                double speed = 0.0;
                retValue += PISendQuestionWithAnswerDouble("VEL?", speed, 100);
                if (!retValue.containsError())
                {
                    m_params["speed"].setVal<double>(speed);
                }
            }

            retValue += val->copyValueFrom(&(*it));
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
            If the "ctrl-type" is set, PIHexapodCtrl::PISwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal PIHexapodCtrl::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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

    if (isMotorMoving()) //this if-case is for actuators only.
    {
        retValue += ito::RetVal(ito::retError, 0, tr("any axis is moving. Parameters cannot be set").toLatin1().data());
    }

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
    }

    if (!retValue.containsError())
    {
        if (key == "PI_CMD")
        {
            if (suffix == "")
            {
                if (m_params["PI_CMD"].getType() == (*val).getType())
                {
                    char *buf = (*val).getVal<char*>();
                    if (buf != NULL)
                    {
                        suffix = QByteArray(buf);
                    }
                }

                if (suffix == "")
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("parameter PI_CMD requires the real command send to the motor after a colon-sign in the parameter name or as value (second parameter of setParam method).").toLatin1().data());
                }
            }

            if (!retValue.containsError())
            {
                retValue += PISendCommand(suffix.toLatin1());
            }
        }
        else if (key == "speed")
        {
            double speed = val->getVal<double>();
            QByteArray cmd = "VEL ";
            cmd.append(QByteArray::number(speed, 'f'));
            retValue += PISendCommand(cmd);
            retValue += PISendQuestionWithAnswerDouble("VEL?", speed, 100);
            if (!retValue.containsError())
            {
                m_params["speed"].setVal<double>(speed);
            }
        }
		else if (key == "pivotPoint")
		{
			if (hasIndex)
			{
				retValue += ito::RetVal(ito::retError, 0, "indexed based setting of pivotPoint not implemented. Pass an array with three values (x,y,z).");
			}
			else
			{
				retValue += setPivotPoint(val->getVal<const double*>());
			}
		}
        else
        {
            retValue += it->copyValueFrom(&(*val));
        }

    }

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
ito::RetVal PIHexapodCtrl::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

	m_threadID = QThread::currentThreadId();

    ito::AddInBase * pSer = reinterpret_cast<ito::AddInBase *>((*paramsOpt)[0].getVal<void *>());
    char *ipchar = (*paramsOpt)[1].getVal<char *>();
    m_tcpPort = (*paramsOpt)[2].getVal<int>();

    if (pSer != NULL)
    {
        if (pSer->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
        {
            m_pSer = (ito::AddInDataIO *)pSer;

            retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 57600)), NULL);
            retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
            retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
            retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
            retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 108)), NULL);
            retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\n")), NULL);
            m_useTCPIP = false;

            if (!retval.containsError())
            {
      //          retval += PICheckStatus();
      //          retval += RequestStatusAndPosition(true, false);
            }
        }
        else
        {
            m_pSer = NULL;
            retval += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
        }
    }
    else if (ipchar != NULL)
    {
        m_pSer = NULL;
        m_tcpAddr = QString::fromLatin1(ipchar);
        m_useTCPIP = true;
        retval += tcpOpenConnection(m_tcpAddr, m_tcpPort, 0);

    }
    else
    {
        m_pSer = NULL;
    }

    ipchar = NULL;

    if (!retval.containsError())
    {
        retval += PIIdentifyAndInitializeSystem();
    }

    if (!retval.containsError())
    {
        m_targetPos = QVector<double>(m_numAxis, 0.0);
        m_currentStatus = QVector<int>(m_numAxis, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);
        m_currentPos = QVector<double>(m_numAxis, 0.0);
    }

    if (!retval.containsError())
    {
        retval += requestStatusAndPosition(true, false);

        double speed = 0.0;
        retval += PISendQuestionWithAnswerDouble("VEL?", speed, 100);
        m_params["speed"].setVal<double>(speed);

		retval += updatePivotPoint();
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
/*! \detail close method which is called before that this instance is deleted by the PIHexapodCtrlInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIHexapodCtrl::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    m_closing = true;

    if (m_connection && m_connection->isOpen())
    {
		if (m_threadID == QThread::currentThreadId())
		{
			//this is a workaround if the unwanted case occurs, that the close method is directly called from itom whereas init has been called in the plugin thread.
			//sending events over TCP/IP is not allowed from different threads!
			PIDummyRead();
			PISendCommand("#24");// Stop
			PISendCommand("#27"); // ESC
			m_connection->disconnectFromHost();
			m_connection->waitForDisconnected(3000);
			DELETE_AND_SET_NULL(m_connection);
		}
		else
		{
			QMetaObject::invokeMethod(this, "threadSafeClose");
			Sleep(1000);
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
void PIHexapodCtrl::threadSafeClose()
{
	//this is a workaround if the unwanted case occurs, that the close method is directly called from itom whereas init has been called in the plugin thread.
	//sending events over TCP/IP is not allowed from different threads!
	PIDummyRead();
	PISendCommand("#24");// Stop
	PISendCommand("#27"); // ESC
	m_connection->disconnectFromHost();
	m_connection->waitForDisconnected(3000);
	DELETE_AND_SET_NULL(m_connection);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIHexapodCtrl::calib(const int axis, ItomSharedSemaphore *waitCond)
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
ito::RetVal PIHexapodCtrl::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval(ito::retOk);

    QByteArray cmd("INI");
    for (int i = 0; i <  axis.size(); i++)
    {
        cmd.append(' ');
        cmd.append(m_axesNames[axis[i]]);
    }

	PIDummyRead();

	for (int i = 0; i < axis.size(); i++)
	{
		setStatus(m_currentStatus[axis[i]], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
	}
	sendStatusUpdate(true);

    retval += PISendCommand(cmd);

	for (int i = 0; i < 10; ++i)
	{
		PICheckStatus(10000); //it seems that the stage "freezes" during init.
		PIDummyRead();
	}

    if (!retval.containsError())
    {
        retval += waitForDone(3000000, axis); //WaitForAnswer(60000, axis);
    }

	for (int i = 0; i < axis.size(); i++)
	{
		replaceStatus(m_currentStatus[axis[i]], ito::actuatorMoving, ito::actuatorAtTarget);
	}
	sendStatusUpdate(true);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function sets the zero position of a single axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    numbers of axis to set to zero
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIHexapodCtrl::setOrigin(const int axis, ItomSharedSemaphore * /*waitCond*/)
{
    return setOrigin(QVector<int>(1,axis), NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function sets the zero position of various axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Vector with numbers of axis to set to zero
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PIHexapodCtrl::setOrigin(QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("not implemented").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function gets the status of the device. The PIStatus function is called internally.

    \param [out] status        Status of System. 0: okay, 1: error
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \todo define the status value
*/
ito::RetVal PIHexapodCtrl::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = PICheckStatus();
    *status = m_currentStatus;
    sendStatusUpdate(true);

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
ito::RetVal PIHexapodCtrl::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    double axpos = 0.0;

    ito::RetVal retval = ito::retOk;

    if (axis < 0 || axis > 5)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toLatin1().data());
    }
    else
    {
        QByteArray PosQust("POS? ");
        PosQust.append(m_axesNames[axis]);
        QByteArray answer;
        retval += PISendQuestionWithAnswerString(PosQust, answer, 200);

        if (!retval.containsError() && answer.contains(m_axesNames[axis]))
        {
            int fpos = answer.indexOf(m_axesNames[axis]);
            int len = 1;
            if (answer.contains(" \n"))
            {
                int len = answer.indexOf(" \n") - fpos;
                *pos = answer.mid(fpos + 2,  len).toDouble();
            }
            else
            {
                int len = answer.length() - fpos - 2;
                *pos = answer.mid(fpos + 2,  len).toDouble();
            }
            m_currentPos[axis] = *pos;
        }

        sendStatusUpdate(false);
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

    \param [in] axis        Vector with axis numbers
    \param [out] pos        Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PIHexapodCtrl::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval = checkAxisVector(axis);
	ito::RetVal retval2;
    QByteArray answer;

    if (!retval.containsError())
    {
        QByteArray question = "POS?";
        for (int i = 0; i < axis.size(); ++i)
        {
            question += " ";
            question += m_axesNames[axis[i]];

        }
		retval2  = PISendQuestionWithAnswerString(question, answer, 200);

		if (retval2.containsError() && retval2.errorCode() == PI_READTIMEOUT)
		{
			Sleep(100);
			int readSigns = 0;
			retval2 = PIReadString(answer, readSigns, 200);
		}

		retval += retval2;
    }

    if (!retval.containsError())
    {
        for (int i = 0; i < axis.size(); i++)
        {
            if (answer.contains(m_axesNames[axis[i]]))
            {
                int fpos = answer.indexOf(m_axesNames[axis[i]]);
                int lpos = answer.indexOf('\n', fpos);
                if (lpos == -1)
                {
                    lpos = answer.length();
                }

                int lpos1 = answer.indexOf(" \n", fpos);
                if (lpos1 != -1 && lpos1 < lpos)
                {
                    lpos = lpos1;
                }
                int len = lpos - fpos - 2;
                if (len > 0)
                {
                    (*pos)[i] = answer.mid(fpos + 2,  len).toDouble();
                    m_currentPos[axis[i]] = (*pos)[i];
                }
            }
        }

        sendStatusUpdate(false);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------
inline ito::RetVal PIHexapodCtrl::checkAxisVector(const QVector<int> &axis)
{
    ito::RetVal retval = ito::retOk;
    unsigned char flag = 0;
    if (axis.size() > m_numAxis)
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("Too many axes given. Only %i axes available.").toLatin1().data(), m_numAxis);
    }
    else
    {
        for (int i = 0; i < axis.size(); i++)
        {
            if (axis[i] < 0 || axis[i] >= m_numAxis)
            {
                retval += ito::RetVal::format(ito::retError, 0, tr("Index of %ith axis out of range [0,%i].").toLatin1().data(), i + 1, m_numAxis - 1);
                break;
            }
            if ((1 << axis.value(i)) & flag)
            {
                retval += ito::RetVal(ito::retError, 0, tr("Error. Addressed axis twice").toLatin1().data());
                break;
            }
            else
            {
                flag |= 1 << axis.value(i);
            }
        }
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            This function calls PIHexapodCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa PISetPos
    \return retOk
*/
ito::RetVal PIHexapodCtrl::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    QVector<int> axisV(1, axis);
    QVector<double> posV(1, pos);

    return PISetPos(axisV, posV, false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls PIHexapodCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa PISetPos
    \return retOk
*/
ito::RetVal PIHexapodCtrl::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    return this->PISetPos(axis, pos, false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the relativ position of a one axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            This function calls PIHexapodCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa PISetPos
    \return retOk
*/
ito::RetVal PIHexapodCtrl::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    QVector<int> axisV(1, axis);
    QVector<double> posV(1, pos);

    return PISetPos(axisV, posV, true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls PIHexapodCtrl::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa PISetPos
    \return retOk
*/
ito::RetVal PIHexapodCtrl::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    return PISetPos(axis, pos, true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa PISetPos
    \return retOk
*/
ito::RetVal PIHexapodCtrl::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    QSharedPointer<QVector<double> > sharedpos = QSharedPointer<QVector<double> >(new QVector<double>);
    sharedpos->resize(m_numAxis);

    retval += PICheckStatus();

    if (sendCurrentPos)
    {
        QVector<int> axis(m_numAxis);
        for (int i = 0; i < m_numAxis; i++)
        {
            axis[i] = i;
            (*sharedpos)[0] = 0.0;
        }
        retval += getPos(axis, sharedpos, NULL);

        for (int i = 0; i < m_numAxis; i++)
        {
            if (std::abs((*sharedpos)[i]-m_targetPos[i]) > 0.01)
            {
                m_targetPos[i] = (*sharedpos)[i];
            }
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

//----------------------------------------------------------------------------------------------------------------------------------
/*!
updates the status flag (if possible). For E662 only the overflow status can be checked, the rest is determined by the movement
and a guess if the motor reached the target position
*/
ito::RetVal PIHexapodCtrl::PICheckStatus(int timeoutMS /*= 200*/)
{
    ito::RetVal retVal = ito::retOk;
    QByteArray answer(0);
    int a;
    int flag;

	retVal += PISendQuestionWithAnswerString("STA?", answer, timeoutMS);
    qDebug() << "Status: " << answer;

	if (retVal.containsError() && retVal.errorCode() == PI_READTIMEOUT)
	{
		Sleep(100);
		int readSigns = 0;
		retVal = PIReadString(answer, readSigns, timeoutMS);
	}

    if (!retVal.containsError())
    {
        bool ok;
        a = answer.toInt(&ok);
        if (!ok)
        {
            retVal += ito::RetVal(ito::retError, 0, "Error getting status. Response is not numeric.");
        }
    }

    if (!retVal.containsError())
    {
        //x:
        int idx = m_axesNames.indexOf("X");
        if (idx >= 0)
        {
            flag = (a & 0x00000001) ? ito::actuatorUnknown : ((a & 0x00000100) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }

        //y:
        idx = m_axesNames.indexOf("Y");
        if (idx >= 0)
        {
            flag = (a & 0x00000002) ? ito::actuatorUnknown : ((a & 0x00000200) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }

        //z:
        idx = m_axesNames.indexOf("Z");
        if (idx >= 0)
        {
            flag = (a & 0x00000004) ? ito::actuatorUnknown : ((a & 0x00000400) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }

        //u:
        idx = m_axesNames.indexOf("U");
        if (idx >= 0)
        {
            flag = (a & 0x00000008) ? ito::actuatorUnknown : ((a & 0x00000800) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }

        //v:
        idx = m_axesNames.indexOf("V");
        if (idx >= 0)
        {
            flag = (a & 0x00000010) ? ito::actuatorUnknown : ((a & 0x00001000) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }

        //w:
        idx = m_axesNames.indexOf("W");
        if (idx >= 0)
        {
            flag = (a & 0x00000020) ? ito::actuatorUnknown : ((a & 0x00002000) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }

        //a:
        idx = m_axesNames.indexOf("A");
        if (idx >= 0)
        {
            flag = (a & 0x00000040) ? ito::actuatorUnknown : ((a & 0x00004000) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }

        //b:
        idx = m_axesNames.indexOf("B");
        if (idx >= 0)
        {
            flag = (a & 0x00000080) ? ito::actuatorUnknown : ((a & 0x00008000) ? ito::actuatorMoving : ito::actuatorAtTarget);
            setStatus(m_currentStatus[idx], ito::actuatorEnabled | ito::actuatorAvailable | flag);
        }
    }


    //if (!retVal.containsError() && answer.length() > 4)
    //{
    //    int val = answer.left(4).toInt();
    //    bool failure = false;
    //    bool moving = false;
    //    for (int i = 0; i < m_numAxis; i++)
    //    {
    //        failure = val & (1 << i);
    //        moving = val & (1 << (i + 8));
    //        if (failure)
    //        {
    //            setStatus(m_currentStatus[i], ito::actuatorEndSwitch | ito::actuatorRightEndSwitch, ito::actMovingMask | ito::actStatusMask);
    //        }
    //        else
    //        {
    //            setStatus(m_currentStatus[i],0, ito::actMovingMask | ito::actStatusMask); //reset end switch flags
    //        }

    //        if (moving)
    //        {
    //            setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
    //        }
    //        else
    //        {
    //            if (failure) setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
    //            else setStatus(m_currentStatus[i], ito::actuatorInterrupted, ito::actSwitchesMask | ito::actStatusMask);
    //        }
    //    }
    //}
    //else
    //{
    //    for (int i = 0; i < m_numAxis; i++)
    //    {
    //        setStatus(m_currentStatus[i], ito::actuatorUnknown, ito::actSwitchesMask | ito::actStatusMask);
    //    }
    //}

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
//

/*!
    \detail Clear serial port before writing without any delay
    \return retOk
*/
ito::RetVal PIHexapodCtrl::PIDummyRead(void) /*!< reads buffer of serial port without delay in order to clear it */
{
    if (m_useTCPIP)
    {
        if (m_connection)
        {
            int buffersize = m_connection->bytesAvailable();
            if (buffersize > 0)
            {
                QByteArray bdarray(buffersize, 0);
                bdarray = m_connection->read(buffersize);
				qDebug() << "unused answer from device: " << bdarray;
            }
        }
    }
    else
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
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::PIReadString(QByteArray &result, int &len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    result.resize(0);
    bool done = false;
    QElapsedTimer timer;

    QByteArray endline("\n");

    if (m_tcpPort)
    {
        QByteArray bdarray;
        bdarray.resize(1);
        int buffersize = 1;
        int count = 0;

        if (len)
        {
            result.reserve(len);
        }

        if (!m_connection)
        {
            bdarray.resize(1);
            bdarray[0] = 0;
            buffersize = 0;
            tcpReconnect();
            QString errString = tr("Error reading from device!\n%1\n").arg(m_connection->errorString());
            retValue += ito::RetVal(ito::retError, 0, errString.toLatin1().data());
        }

        if (!retValue.containsError())
        {
            len = 0;
            timer.start();

            //waitForReadyRead is not safe on Windows, therefore we implemented our own check and timeout mechanism
            QTimer timer2;
            timer2.setSingleShot(true);
            QEventLoop loop;
            connect(m_connection, SIGNAL(readyRead()), &loop, SLOT(quit()));
            connect(&timer2, SIGNAL(timeout()), &loop, SLOT(quit()));

            while(!done && !retValue.containsError())
            {
                setAlive();

                timer2.start(10);
                loop.exec();

                if (timer2.isActive())
                {
                    buffersize = m_connection->bytesAvailable();
                    if (buffersize > bdarray.capacity())
                    {
                        bdarray.reserve(buffersize);
                    }
                    bdarray = m_connection->read(buffersize);
                    setAlive();
                    result.append(bdarray);

                    count++;
                }
                else
                {
					count++;
                    //qDebug("timeout");
                }

                if (result.length() > 1 && (result.at(result.length() - 1) == '\n') && (result.at(result.length() - 2) != ' '))
                {
                    done = true;
                }

                if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
                {
                    retValue += ito::RetVal(ito::retError, PI_READTIMEOUT, tr("timeout").toLatin1().data());
                }
            }
        }

        int pos = result.lastIndexOf('\n');

        if (pos >= 0) //found
        {
            result = result.left(pos);
        }
    }
    else
    {
        if (m_pSer)
        {
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
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("could not read endline parameter from serial port").toLatin1().data());
            }

            if (!retValue.containsError())
            {
                len = 0;
                timer.start();

                while(!done && !retValue.containsError())
                {
                    setAlive();
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

                    if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
                    {
                        retValue += ito::RetVal(ito::retError, PI_READTIMEOUT, tr("timeout").toLatin1().data());
                    }
                }
            }
        }
    }
    len = result.length();
    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::PIGetLastErrors(QVector<QPair<int,QByteArray> > &lastErrors)
{
    bool errorAvailable = true;
    ito::RetVal retValue(ito::retOk);
	ito::RetVal retVal2;
    QByteArray buffer;
    QByteArray errorText;
    int readSigns = 0;
    int pos;
    int errorNo;
    lastErrors.clear();

    while(errorAvailable && !retValue.containsError())
    {
        retValue += PISendCommand("ERR?");
        retVal2 =  PIReadString(buffer, readSigns, 200);

		if (retVal2.containsError() && retVal2.errorCode() == PI_READTIMEOUT)
		{
			Sleep(200);
			retVal2 = PIReadString(buffer, readSigns, 500);
		}

		retValue += retVal2;

        if (!retValue.containsError())
        {
            //buffer has form ErrorCode, "ErrorMsg"
            pos = buffer.indexOf(' ');

            if (buffer.length() >= 0)
            {
                if (pos > 0)
                {
                    errorNo = buffer.left(pos).toInt();
                }
                else
                {
                    errorNo = buffer.toInt();
                }
                switch(errorNo)
                {
                    case 0:
                        errorAvailable = false;
                    break;
                    case 1:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Parameter Syntax Error").toLatin1().data()));
                    break;
                    case 2:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Unknown command").toLatin1().data()));
                    break;
                    case 3:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Command too long").toLatin1().data()));
                    break;
                    case 5:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("INI or Servo ON required before move at this time").toLatin1().data()));
                    break;
                    case 6:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("SGA-Parameter out of range").toLatin1().data()));
                    break;
                    case 7:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Motion range exceeded").toLatin1().data()));
                    break;
                    case 8:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Velocity range exceeded").toLatin1().data()));
                    break;
                    case 9:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Attempt to set pivot point while U, V or W not all equal to 0").toLatin1().data()));
                    break;
                    case 10:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Last command was STOP or DRV").toLatin1().data()));
                    break;
                    case 11:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("SST parameter out of range").toLatin1().data()));
                    break;
                    case 13:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("NAV parameter out of range").toLatin1().data()));
                    break;
                    case 14:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Invalid analog channel").toLatin1().data()));
                    break;
                    case 15:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Invalid axis identifier").toLatin1().data()));
                    break;
                    case 17:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Command parameter out of range").toLatin1().data()));
                    break;
                    case 23:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Invalid Axis").toLatin1().data()));
                    break;
                    case 25:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Invalid Real Number").toLatin1().data()));
                    break;
                    case 26:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Missing Parameter").toLatin1().data()));
                    break;
                    case 27:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Soft Limit out of Range").toLatin1().data()));
                    break;
                    case 46:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("F-361 (Optical Power Meter) missing").toLatin1().data()));
                    break;
                    case 47:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("F-361 cannot be initialized/is not initialized").toLatin1().data()));
                    break;
                    case 48:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("F-361 communications error").toLatin1().data()));
                    break;
                    case 53:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("MOV! command motion in progress").toLatin1().data()));
                    break;
                    case 54:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Unknown parameter").toLatin1().data()));
                    break;
                    case 57:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Data Record Table does not exist").toLatin1().data()));
                    break;
                    case 63:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("INI is running").toLatin1().data()));
                    break;
                    case 200:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("No stage connected").toLatin1().data()));
                    break;
                    case 201:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("File with stage/axis parameters not found").toLatin1().data()));
                    break;
                    case 202:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Invalid Axis Parameter File").toLatin1().data()));
                    break;
                    case 210:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Illegal File Name").toLatin1().data()));
                    break;
                    case 211:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("File not found").toLatin1().data()));
                    break;
                    case 212:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("File write Error").toLatin1().data()));
                    break;
                    case 213:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("DTR hinders Velocity Change").toLatin1().data()));
                    break;
                    case 214:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Position Unknown").toLatin1().data()));
                    break;
                    case 217:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Unexpected Strut Stop").toLatin1().data()));
                    break;
                    case 218:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Reported Position is based on Interpolation (MOV, MWG or MAR! is running)").toLatin1().data()));
                    break;
                    case 219:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Reported Position is based on Estimation (MOV! is running)").toLatin1().data()));
                    break;
                    case 301:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Buffer overflow").toLatin1().data()));
                    break;
                    case 333:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Hardware Error, motion deviation limit exceeded").toLatin1().data()));
                    break;
                    default:
                        lastErrors.append(QPair<int,QByteArray>(errorNo, tr("Undefined error code").toLatin1().data()));
                        break;
                }
            }
            else
            {
                lastErrors.append(QPair<int,QByteArray>(-666, tr("error could not be parsed").toLatin1().data()));
            }
        }
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::PISendCommand(const QByteArray &command)
{
    ito::RetVal retVal = ito::retOk;
    if (m_useTCPIP)
    {
        if (m_connection)
        {
            if (command.length() > 450)
            {
                int chunks = ceil(command.length() / 450.0);
                int remain = command.length();
                for (int pak = 0; pak < chunks; pak++)
                {
                    QString packet(command.mid(pak * 450, remain>450?450:remain));
                    m_connection->write(packet.toLatin1().data(), remain>450?450:remain);
                    remain -= 450;
                    if (!m_connection->waitForBytesWritten())
                    {
                        retVal += tcpReconnect();
                        QString errString = QString("Error writing to device!\n%1\n").arg(m_connection->errorString());
                        return ito::RetVal(ito::retError, 0, errString.toLatin1().data());
                    }
                }
            }
            else
            {
                QByteArray fullcommand = command + "\n";
                m_connection->write(fullcommand.data(), fullcommand.length());

                //waitForReadyRead is not safe on Windows, therefore we implemented our own check and timeout mechanism
                QTimer timer;
                timer.setSingleShot(true);
                QEventLoop loop;
                connect(m_connection, SIGNAL(bytesWritten(qint64)), &loop, SLOT(quit()));
                connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));

                timer.start(400);
                loop.exec();

                if (!timer.isActive())
                {
                    retVal += tcpReconnect();
                    QString errString = QString("Error writing to device!\n%1\n").arg(m_connection->errorString());
                    retVal += ito::RetVal(ito::retError, 0, errString.toLatin1().data());
                }
            }
        }
    }
    else
    {
        if (m_pSer)
        {
            retVal += m_pSer->setVal(command.data(), command.length(), NULL);
            if (m_delayAfterSendCommandMS > 0)
            {
                QMutex mutex;
                mutex.lock();
                QWaitCondition waitCondition;
                waitCondition.wait(&mutex,m_delayAfterSendCommandMS);
                mutex.unlock();
            }
        }
        else retVal += ito::RetVal(ito::retError, 0, tr("No serial port open, pointer is NULL").toLatin1().data());
    }

    return retVal;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string.
            If string is invalid, val is not set and error-message is reported
    \param[in] *buf        Answer-String
    \param[out] val        double Value
    \sa PIHexapodCtrl::PIGetDouble
    \return retOk
*/
ito::RetVal PIHexapodCtrl::PISendQuestionWithAnswerDouble(const QByteArray &questionCommand, double &answer, int timeoutMS)
{
    int readSigns = 0;
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
        retValue = ito::RetVal(ito::retError, 0, tr("value could not be parsed to a double value").toLatin1().data());
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string.
            If string is invalid, val is not set and error-message is reported
    \param[out] *buf        Answer-String
    \param[in] bufsize        Number of signs to read
    \param[out] readsigns    Number of read signs
    \return retOk
*/
ito::RetVal PIHexapodCtrl::PISendQuestionWithAnswerString(const QByteArray &questionCommand, QByteArray &answer, int timeoutMS)
{
    int readSigns = 0;
    ito::RetVal retValue = PISendCommand(questionCommand);
    retValue += PIReadString(answer, readSigns, timeoutMS);

    if (retValue.containsError())
    {
        QVector< QPair<int, QByteArray> > lastErrors;
        retValue += PIGetLastErrors(lastErrors);
        retValue += convertPIErrorsToRetVal(lastErrors);
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::convertPIErrorsToRetVal(QVector<QPair<int,QByteArray> > &lastErrors)
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

//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::PIIdentifyAndInitializeSystem(void)
{
    ito::RetVal retval = ito::retOk;
    QByteArray answer;
    QVector<QPair<int,QByteArray> > lastErrors;

    PIDummyRead();

    retval += PISendQuestionWithAnswerString("*idn?", answer, 400);
    if (retval.containsError() || answer.length() < 5)
    {
        retval = ito::RetVal(ito::retError, 0, tr("could not identify controller. No answer for command *idn?").toLatin1().data());
    }
    else
    {
        m_params["identifier"].setVal<const char*>(answer);
        setIdentifier(QLatin1String(answer));
    }

    //clear error-queue
    PIGetLastErrors(lastErrors);
    lastErrors.clear();

    PIDummyRead();

    //get axes
    if (!retval.containsError())
    {
        retval += PISendQuestionWithAnswerString("CST?", answer, 400);
        if (retval.containsError() || answer.length() < 5)
        {
            retval = ito::RetVal(ito::retError, 0, tr("could not identify axes. No answer for command CST?").toLatin1().data());
        }
        else
        {
            QString axes = QLatin1String(answer);
            //'X=HEXAPOD_AXIS_X \nY=HEXAPOD_AXIS_Y \nZ=HEXAPOD_AXIS_Z \nU=HEXAPOD_AXIS_U \nV=HEXAPOD_AXIS_V \nW=HEXAPOD_AXIS_W'
            QStringList axes2 = axes.split(" \n");
            for (int i = 0; i < axes2.size(); ++i)
            {
                m_axesNames.append(axes2[i].split("=")[0].toLatin1());
            }

            m_numAxis = m_axesNames.size();
            m_params["numaxis"].setVal<int>(m_numAxis);
#if QTVERSION >= 0x050400
            m_params["axesNames"].setVal<const char*>(m_axesNames.join(";"));
#else
            QByteArray axesNamesJoined;
            if (m_axesNames.size() >= 1)
            {
                axesNamesJoined = m_axesNames[0];
            }
            for (int i = 1; i < m_axesNames.size(); ++i)
            {
                axesNamesJoined.append(";");
                axesNamesJoined.append(m_axesNames[i]);
            }

            m_params["axesNames"].setVal<const char*>(axesNamesJoined);
#endif
        }
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
ito::RetVal PIHexapodCtrl::PISetPos(const QVector<int> &axis,  const QVector<double> &posMM, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = checkAxisVector(axis);
    bool released = false;
    bool outOfRange = false;
    QByteArray cmdTotal;
	isInterrupted(); //reset interrupt

    if (!retval.containsError())
    {
        PIDummyRead();
        if (relNotAbs)
        {
            QSharedPointer<QVector<double> > actPos = QSharedPointer<QVector<double> >(new QVector<double>(axis.size(), 0.0));
            retval += getPos(axis, actPos, NULL);
            for (int i = 0; i < axis.size(); i++)
            {
				qDebug() << m_targetPos[axis[i]] << posMM[i] << (*actPos)[i];
                m_targetPos[axis[i]] = (posMM[i] + (*actPos)[i]);
            }
        }
        else
        {
            for (int i = 0; i < axis.size(); i++)
            {
                m_targetPos[axis[i]] = posMM[i];
            }
        }

        if (!retval.containsError())
        {
            cmdTotal = "MOV!";
            for (int i = 0; i < axis.size(); i++)
            {
                cmdTotal.append(' ');
                cmdTotal.append(m_axesNames[axis[i]]);
                cmdTotal.append(QByteArray::number(m_targetPos[axis[i]], 'g'));
            }

			for (int i = 0; i < axis.size(); i++)
			{
				setStatus(m_currentStatus[axis[i]], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
			}
			sendStatusUpdate(false);

			sendTargetUpdate();
			retval += PISendCommand(cmdTotal);

			QVector<QPair<int, QByteArray> > lastError;
			retval += PIGetLastErrors(lastError);
			retval += convertPIErrorsToRetVal(lastError);
		}

        if (!retval.containsError())
        {
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }

            if (m_doWait == true)
            {
                retval += waitForDone(3000000, axis); //WaitForAnswer(60000, axis);
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
            for (int i = 0; i < axis.size(); i++)
            {
                replaceStatus(m_currentStatus[axis[i]], ito::actuatorMoving, ito::actuatorAtTarget);
            }
            sendStatusUpdate(true);

            if (waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }
        }
    }

	if (waitCond && !released)
	{
		waitCond->returnValue = retval;
		waitCond->release();
	}

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retVal(ito::retOk);
    QMutex waitMutex;
    QWaitCondition waitCondition;
    bool atTarget = false;
	int timeouts = 0;

    QElapsedTimer timer;
    timer.start();

    while (!atTarget && !retVal.containsError() && timer.elapsed() < timeoutMS)
    {
		if (isInterrupted())
		{
			PIDummyRead();
			PISendCommand("#24");// Stop
			PISendCommand("#27"); // ESC
			PISendCommand("STP");
			PIDummyRead();
			replaceStatus(axis, ito::actuatorMoving, ito::actuatorInterrupted);
			retVal += ito::RetVal(ito::retError, PI_INTERRUPT, tr("interrupt occurred").toLatin1().data());
		}
		else
		{
			retVal += PICheckStatus();

			if (retVal.containsError() && retVal.errorCode() == PI_READTIMEOUT)
			{
				timeouts++;
				if (timeouts < 20)
				{
					retVal = ito::retOk;
				}
				else
				{
					//qDebug() << "timeout in waitForDone";
				}
			}
			else
			{
				timeouts = 0;
			}

			atTarget = true;
			for (int i = 0; i < m_numAxis; ++i)
			{
				if (m_currentStatus[i] & ito::actuatorMoving)
				{
					atTarget = false;
					break;
				}
			}
		}
    }

	PIDummyRead();

    /*QByteArray answer;
    retVal += PISendQuestionWithAnswerString("STA?", answer, timeoutMS);

    retVal += PISendQuestionWithAnswerDouble("MOV?", answerDbl, timeoutMS);
    if (answerDbl > 0.0)
    {
        atTarget = true;
    }

    retVal += PICheckStatus();*/

	qDebug() << retVal.containsError() << retVal.errorCode() << "vor getPos";

    QSharedPointer<QVector<double> > sharedpos = QSharedPointer<QVector<double> >(new QVector<double>);
    sharedpos->resize(m_numAxis);

    QVector<int> axes(m_numAxis);
    for (int i = 0; i < m_numAxis; i++)
    {
		axes[i] = i;
        (*sharedpos)[0] = 0.0;
    }
	retVal += getPos(axes, sharedpos, NULL);

    for (int i = 0; i < m_numAxis; i++)
    {
        if (std::abs((*sharedpos)[i]-m_targetPos[i]) > 0.01)
        {
            m_targetPos[i] = (*sharedpos)[i];
        }
    }

	PIDummyRead();

    sendStatusUpdate(false);
    sendTargetUpdate();

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PIHexapodCtrl::tcpChanged(QAbstractSocket::SocketState socketState)
{
    if (((socketState == QAbstractSocket::ClosingState) || (socketState == QAbstractSocket::NotOpen)) && !m_closing)
    {
        tcpReconnect();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::tcpReconnect(void)
{
    ito::RetVal retval(ito::retOk);

    if (m_connection)
    {
        setAlive();
        m_connection->disconnectFromHost();
        if (m_connection->state() == QAbstractSocket::ConnectedState)
            m_connection->waitForDisconnected(3000);
        setAlive();
        retval += tcpOpenConnection(m_tcpAddr, m_tcpPort, 0);
        setAlive();
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("no connection opened!\n").toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::tcpOpenConnection(const QString &ipAddress, const long port, const char /*proto*/)
{
    ito::RetVal retval(ito::retOk);
    QHostAddress address;

    if (!address.setAddress(ipAddress))
    {
        return ito::RetVal(ito::retError, 0, "Invalid IP Address!\n");
    }

    if (!m_connection)
    {
        m_connection = new QTcpSocket();
    }
//    bool ret = connect(m_connection, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(tcpChanged(QAbstractSocket::SocketState)));
    m_connection->connectToHost(address, port);
    if (!m_connection->waitForConnected(3000))
    {
        QString errString = QString("Connecting device on IP: %1:%2 failed!\n%3\n").arg(ipAddress).arg(port).arg(m_connection->errorString());
        return ito::RetVal(ito::retError, 0, errString.toLatin1().data());
    }
    m_connection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    m_connection->setSocketOption(QAbstractSocket::KeepAliveOption, 1);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PIHexapodCtrl::dockWidgetVisibilityChanged(bool visible)
{
    if(qobject_cast<QApplication*>(QCoreApplication::instance()) && getDockWidget())
    {
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            emit parametersChanged(m_params);
            //requestStatusAndPosition(true, true); //not necessary since called by motorAxisController widget of dock widget
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::updatePivotPoint()
{
	QByteArray answer(0);
	ito::RetVal retValue = PISendQuestionWithAnswerString("SPI?", answer, 1000);
	if (!retValue.containsError() && answer.length() > 0)
	{
		const char names[3] = { 'R', 'S', 'T' };
		double result[3] = { 0.0, 0.0, 0.0 };
		for (int i = 0; i < 3; i++)
		{
			if (answer.contains(names[i]))
			{
				int fpos = answer.indexOf(names[i]);
				int lpos = answer.indexOf('\n', fpos);
				if (lpos == -1)
				{
					lpos = answer.length();
				}

				int lpos1 = answer.indexOf(" \n", fpos);
				if (lpos1 != -1 && lpos1 < lpos)
				{
					lpos = lpos1;
				}
				int len = lpos - fpos - 2;
				if (len > 0)
				{
					result[i] = answer.mid(fpos + 2, len).toDouble();
				}
			}
			else
			{
				retValue += ito::RetVal::format(ito::retError, 0, tr("getting pivot point: coordinate name '%c' does not exist").toLatin1().data(), names[i]);
			}
		}

		double* pivotPoint = m_params["pivotPoint"].getVal<double*>();
		memcpy(pivotPoint, result, 3 * sizeof(double));
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::setPivotPoint(const double *values)
{
	ito::RetVal retValue;

	QByteArray command("SPI R");
	command.reserve(40);
	command.append(QByteArray::number(values[0], 'g', 6));
	command.append(" S");
	command.append(QByteArray::number(values[1], 'g', 6));
	command.append(" T");
	command.append(QByteArray::number(values[2], 'g', 6));

	retValue += PISendCommand(command);

	QVector<QPair<int, QByteArray> > lastError;
	retValue += PIGetLastErrors(lastError);
	retValue += convertPIErrorsToRetVal(lastError);

	updatePivotPoint();

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PIHexapodCtrl::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = NULL;
    ito::ParamBase *param2 = NULL;
    ito::ParamBase *param3 = NULL;
    QVector<QPair<int, QByteArray> > lastError;

    if (funcName == "setPivotPoint")
    {
        param1 = ito::getParamByName(&(*paramsMand), "xPosition", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "yPosition", &retValue);
        param3 = ito::getParamByName(&(*paramsMand), "zPosition", &retValue);

        if (!retValue.containsError())
        {
			double values[] = { param1->getVal<double>(), param2->getVal<double>(), param3->getVal<double>() };
			retValue += setPivotPoint(values);
        }
    }
    else if (funcName == "getPivotPoint")
    {
        param1 = ito::getParamByName(&(*paramsOut), "xPosition", &retValue);
        param2 = ito::getParamByName(&(*paramsOut), "yPosition", &retValue);
        param3 = ito::getParamByName(&(*paramsOut), "zPosition", &retValue);

        if (!retValue.containsError())
        {
            QByteArray answer(0);
			retValue += updatePivotPoint();
			if (!retValue.containsError())
			{
				const double *result = m_params["pivotPoint"].getVal<const double*>();
                param1->setVal<double>(result[0]);
                param2->setVal<double>(result[1]);
                param3->setVal<double>(result[2]);
            }
        }
    }
    else if (funcName == "beFunny")
    {
        param1 = ito::getParamByName(&(*paramsMand), "cycles", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "amplitude", &retValue);
        param3 = ito::getParamByName(&(*paramsMand), "timeconstant", &retValue);

        if (!retValue.containsError())
        {
            m_doWait = false;
            double amp = param2->getVal<double>();
            QVector<int> axis(6);
            axis[0] = 0;
            axis[1] = 1;
            axis[2] = 2;
            axis[3] = 3;
            axis[4] = 4;
            axis[5] = 5;

            QVector<double> tarPos(6);
            int delay = (int)(param3->getVal<double>()*1000 + 0.5);
            for (int cylce = 0; cylce < param1->getVal<int>(); cylce++)
            {
                tarPos[0] = amp * qCos(cylce/10.0);
                tarPos[1] = amp * qSin(cylce/10.0);
                tarPos[2] = amp/3 * qSin(cylce/3.0);
                tarPos[3] = amp/2 * qSin(cylce/5.0);
                tarPos[4] = amp/2 * qCos(cylce/5.0);
                tarPos[5] = 0.0;
                retValue += PISetPos(axis, tarPos, false, NULL);
                setAlive();
                Sleep(delay);
            }
        }
        m_doWait = true;
    }
    else
    {
        retValue += ito::RetVal::format(ito::retError,0,tr("function name '%s' does not exist").toLatin1().data(), funcName.toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retValue;
}
