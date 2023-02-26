#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "piezosystemJena_NV40_1.h"
#include "dockWidgetPiezosystemJena_NV40_1.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qregularexpression.h>
#include <qwaitcondition.h>
#include <qelapsedtimer.h>

#include "common/apiFunctionsInc.h"

#define PI_READTIMEOUT 256

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the plugin type (typeActuator) and sets the plugins object name. Theplugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt).
*/
PiezosystemJena_NV40_1Interface::PiezosystemJena_NV40_1Interface()
{
    m_type = ito::typeActuator;
    setObjectName("PiezosystemJena_NV40_1");

    m_description = QObject::tr("Piezosystem Jena NV40/1 CLE");

/*    char docstring[] = \
"The PiezosystemJena is an itom-plugin, which can be used to communicate with the one-axis piezo controller Piezosystem Jena NV40/1 CLE.\n\
This system needs a serial port. The parameters are set automatically during initialization.\n\
It is initialized by actuator(\"PiezosystemJena_NV40_1\", serialInstance).";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("The PiezosystemJena is an itom-plugin, which can be used to communicate with the one-axis piezo controller Piezosystem Jena NV40/1 CLE.\n\
This system needs a serial port. The parameters are set automatically during initialization.\n\
It is initialized by actuator(\"PiezosystemJena_NV40_1\", serialInstance).");

    m_author = "V. Ferreras-Paz, M. Gronle, W. Lyda, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL");
    m_aboutThis = QObject::tr(GITVERSION);    
    
    ito::Param paramVal("serial", ito::ParamBase::HWRef | ito::ParamBase::In, NULL, tr("An opened serial port (the right communcation parameters will be set by this piezo-controller).").toLatin1().data());
    m_initParamsMand.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PiezosystemJena_NV40_1Interface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PiezosystemJena_NV40_1)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PiezosystemJena_NV40_1Interface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(PiezosystemJena_NV40_1)
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
//! 
/*!
    \detail This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
    creates new instance of dialogPiezosystemJena, calls the method setVals of dialogPiezosystemJena, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogPiezosystemJena
*/
const ito::RetVal PiezosystemJena_NV40_1::showConfDialog(void)
{
    DialogPiezosystemJena_NV40_1 *dialog = new DialogPiezosystemJena_NV40_1(this);
    return apiShowConfigurationDialog(this, dialog);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail defines the name and sets the plugins parameters (m_parans). The plugin is initialized (e.g. by a Python call) 
    with mandatory or optional parameters (m_initParamsMand and m_initParamsOpt) by the PiezosystemJena_NV40_1::init. The widged window is created at this position.
*/
PiezosystemJena_NV40_1::PiezosystemJena_NV40_1() :
    AddInActuator(),
    m_pSer(NULL),
    m_delayAfterSendCommandMS(0),
    m_dockWidget(NULL),
    m_async(0),
    m_closedLoop(true)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "PiezosystemJena", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("async", ito::ParamBase::Int, 0, 1, m_async, tr("asychronous (1) or synchronous (0) mode").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numAxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 1, 1, tr("number of axes (here always 1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("closedLoop", ito::ParamBase::Int, 0, 1, 0, tr("open loop (0, unit is voltage) or closed loop (1, unit is millimeters)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("delayMode", ito::ParamBase::Int, 0, 1, 0, tr("0: target position is continuously read after each positioning to check for end of movement (default), 1: sleep for a constant delayTime after each movement").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("delayTime", ito::ParamBase::Double, 0.0, 2.0, 0.02, tr("offset delay [s] after each movement (only considered for delayMode: 1)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("remote", ito::ParamBase::Int, 0, 1, 1, tr("defines whether device is in local (0) or remote (1) mode (default).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    m_targetPos = QVector<double>(1,0.0);
    m_currentStatus = QVector<int>(1, ito::actuatorAtTarget | ito::actuatorAvailable | ito::actuatorEnabled);
    m_currentPos = QVector<double>(1,0.0);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        m_dockWidget = new DockWidgetPiezosystemJena_NV40_1(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_dockWidget);
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
ito::RetVal PiezosystemJena_NV40_1::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

    if (hasIndex)
    {
        retValue += ito::RetVal(ito::retError,0,"index based parameter name not supported by this plugin");
    }

    if(!retValue.containsError())
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
            If the "ctrl-type" is set, PiezosystemJena_NV40_1::PISwitchType is executed.

    \param[in] *name        Name of parameter
    \param[in] *val            String with parameter
    \param[in] len            Length of the string
    \param[in/out] *waitCond    Waitcondition between this thread and the callers tread

    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(isMotorMoving()) //this if-case is for actuators only.
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Axis is moving. Parameters cannot be set.").toLatin1().data());
    }

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
        //here the new parameter is checked whether it's type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if(!retValue.containsError())
    {
        if (key == "closedLoop")
        {
            if (val->getVal<int>() > 0)
            {
                retValue += serialSendCommand("cl");
                QByteArray answer;
                int len;
                readString(answer, len, 1500);
                if (answer == "only OL mode\r\n")
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("closed loop not possible. No sensor connected.").toLatin1().data());
                    it->setVal<int>(0);
                }
                else
                {
                    m_closedLoop = true;
                }
            }
            else
            {
                retValue += serialSendCommand("ol");
                QByteArray answer;
                int len;
                readString(answer, len, 1500);
                if (answer == "only OL mode\r\n")
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("closed loop not possible. No sensor connected.").toLatin1().data());
                    it->setVal<int>(0);
                }
                else
                {
                    m_closedLoop = false;
                }
            }

            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
        }
        else if (key == "remote")
        {
            if (val->getVal<int>() > 0)
            {
                retValue += serialSendCommand("i1");
            }
            else
            {
                retValue += serialSendCommand("i0");
            }
            
            if (!retValue.containsError())
            {
                it->copyValueFrom(&(*val));
            }
        }
        else if (key == "async")
        {
            m_async = val->getVal<int>();
            it->copyValueFrom(&(*val));
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom( &(*val) );
        }
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
/*! \detail Init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \todo check if (*paramsMand)[0] is a serial port
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{   
    QByteArray answer;
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();

        //set serial settings
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 9600)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Int, 0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 33)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r")), NULL);
        m_receiveEndline = "\r\n";
    }

    if (!retval.containsError())
    {
        m_delayAfterSendCommandMS = 5; //small delay after sendCommands (else sometimes the controller didn't not understand the commands)

        QString name;
        retval += identifyAndInitializeSystem(name);
        QSharedPointer<ito::Param> port(new ito::Param("port", ito::ParamBase::Int));
        m_pSer->getParam(port);
        setIdentifier(QString("%1 @ COM%2").arg(name).arg(port->getVal<int>()));

        if (!retval.containsError())
        {
            retval += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("remote", ito::ParamBase::Int, 1)));

            ito::RetVal retval2 = setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("closedLoop", ito::ParamBase::Int, 1)));
            if (retval2.containsError())
            {
                //closed loop not possible, goto open loop
                retval += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("closedLoop", ito::ParamBase::Int, 0)));
            }

            retval += requestStatusAndPosition(true, false);
        }
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
/*! \detail close method which is called before that this instance is deleted by the PiezosystemJenaInterface
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PiezosystemJena_NV40_1::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This function executes a calibration routine for one axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    Number of axis to calibrate
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PiezosystemJena_NV40_1::calib(const int axis, ItomSharedSemaphore *waitCond)
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
ito::RetVal PiezosystemJena_NV40_1::calib(const QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
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
/*! \detail This function sets the zero position of a single axis spezified by "axis". In the case of this device the function body is nearly empty and has no effect.

    \param [in] axis    numbers of axis to set to zero
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PiezosystemJena_NV40_1::setOrigin(const int axis, ItomSharedSemaphore * /*waitCond*/)
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
ito::RetVal PiezosystemJena_NV40_1::setOrigin(QVector<int> /*axis*/, ItomSharedSemaphore *waitCond)
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
ito::RetVal PiezosystemJena_NV40_1::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval;

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

    \param [in] axis        Axisnumber
    \param [out] pos        Current position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    double axpos = 0.0;
    
    ito::RetVal retval = ito::retOk;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis %i does not exist").arg(axis).toLatin1().data());
    }
    else
    {
        retval += sendQuestionWithAnswerDouble("rd,", axpos, 1500);
        if (m_closedLoop)
        {
            *pos = axpos / 1000.0; // mu m to mm
        }
        else
        {
            *pos = axpos; //Volt
        }
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

    \param [in] axis        Vector with axis numbers
    \param [out] pos        Current positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);

    if (axis.size() == 1)
    {
        retval += getPos(axis.at(0), sharedpos, NULL);
        if (!retval.containsError())
        {
            (*pos)[0] = *sharedpos;
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("Error: controller only supports one axis.").toLatin1().data());
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
            This function calls PiezosystemJena_NV40_1::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        absolute target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa setPos
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPos(axis, pos, false, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls PiezosystemJena_NV40_1::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with absolute positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa PISetPos
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    if (axis.size() != 1)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Too many axes. This is a single axis device.").toLatin1().data());

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
            This function calls PiezosystemJena_NV40_1::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    axis number
    \param [in] pos        relative target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa setPos
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPos(axis, pos, true, waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail Set the absolute position of a number of axis spezified by "axis" to the position "pos" . The value in device independet in mm.
            If the size of the vector is more then 1 element, this function returns an error.
            This function calls PiezosystemJena_NV40_1::PISetPos(axis, pos, "ABSOLUTCOMMAND")

    \param [in] axis    1 Element Vector with axis numbers
    \param [in] pos        1 Element Vector with relative positions in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \sa setPos
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);

    if (axis.size() != 1)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Too many axes. This is a single axis device.").toLatin1().data());

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
/*! \detail Set the position (abs or rel) of a one axis spezified by "axis" to the position "dpos". The value in device independet in mm. 
            If the axisnumber is not 0, this function returns an error.

    \param [in] axis        axis number
    \param [in] dpos        target position in mm
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::setPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond)
{
    double dpos_temp;
    ito::RetVal retval = ito::retOk;
    bool released = false;
    int delayTimeMS = 0;
    QByteArray cmdTotal;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toLatin1().data());

        if (waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
    }
    else if (m_params["remote"].getVal<int>() == 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Remote mode must be enabled for movement").toLatin1().data());
    }
    else
    {
        retval += serialDummyRead();

        if (relNotAbs)
        {
            QSharedPointer<double> actPos = QSharedPointer<double>(new double);

            retval += getPos(0, actPos, NULL);
            if (!retval.containsError())
            {
                m_targetPos[0] = m_currentPos[0] + posMM;
            }

            if (m_closedLoop)
            {
                dpos_temp = m_targetPos[0] * 1000;
            }
            else
            {
                dpos_temp = m_targetPos[0];
            }

            cmdTotal = "wr,";
            cmdTotal = cmdTotal.append( QByteArray::number(dpos_temp, 'g') );
        }
        else
        {
            if (m_closedLoop)
            {
                dpos_temp = posMM * 1000;
            }
            else
            {
                dpos_temp = posMM;
            }

            cmdTotal = "wr,";
            cmdTotal = cmdTotal.append( QByteArray::number(dpos_temp, 'g') );
            m_targetPos[0] = posMM;
        }

        if (!retval.containsError())
        {
            setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate(false);
            sendTargetUpdate();

            retval += serialSendCommand(cmdTotal);
            retval += serialDummyRead();

            if (retval.containsError())
            {
                setStatus(m_currentStatus[0], ito::actuatorUnknown, ito::actSwitchesMask | ito::actStatusMask);
                sendStatusUpdate(false);
            }
        }

        if (!retval.containsError())
        {
            if (m_async && waitCond && !released)
            {
                waitCond->returnValue = retval;
                waitCond->release();
                released = true;
            }

            retval += waitForDone(5000, QVector<int>(1,axis));

            if (!m_async && waitCond && !released)
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
        released = true;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PiezosystemJena_NV40_1::waitForDone(const int timeoutMS, const QVector<int> /*axis*/ /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retVal;
    QSharedPointer<double> actPos = QSharedPointer<double>(new double);

    if (m_params["delayMode"].getVal<int>() > 0) //sleep
    {
        sleep(m_params["delayTime"].getVal<double>() * 1000); //0..2sec, no need to call setAlive
        replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorAtTarget);

        retVal += getPos(0, actPos, NULL);
        sendStatusUpdate(false);
    }
    else //wait for end of placement
    {
        QElapsedTimer timer;
        bool done = false;
        ito::RetVal retval2;
        timer.start();

        while (!done && !retVal.containsError() && timer.elapsed() < timeoutMS)
        {
            sleep(5);
            retval2 = getPos(0, actPos, NULL);
            if (retval2.errorCode() == PI_READTIMEOUT)
            {
                serialDummyRead();
                //ignore timeout when obtaining the current position
            }
            else
            {
                retVal += retval2;
            }

            if (!retVal.containsError())
            {
                if (std::abs(m_currentPos[0] - m_targetPos[0]) < (m_closedLoop ? 0.001 : 0.1))
                {
                    m_targetPos[0] = m_currentPos[0];
                    replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorAtTarget);
                    done = true;
                }
            }
            sendStatusUpdate(false);
            setAlive();
        }

        if (!done && timer.elapsed() > timeoutMS)
        {
            retVal += ito::RetVal(ito::retError, 0, tr("timeout while waiting for end of movement").toLatin1().data());
            replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorTimeout);
            sendStatusUpdate(true);
        }
        else if (retVal.containsError())
        {
            replaceStatus(m_currentStatus[0], ito::actuatorMoving, ito::actuatorUnknown);
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \detail This slot is triggerd by the request signal from the dockingwidged dialog to update the position after ever positioning command.
            It sends the current postion and the status to the world.

    \sa setPos
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);

    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
    *sharedpos = 0.0;

    if (sendCurrentPos)
    {
        retval += getPos(0, sharedpos, 0);
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
    \detail Clear serial port before writing without any delay
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::serialDummyRead(QByteArray *content /*= NULL*/) /*!< reads buffer of serial port without delay in order to clear it */
{
    int bufsize = 50;
    QSharedPointer<int> len(new int);
    *len = bufsize;
    QSharedPointer<char> buffer(new char[bufsize]);

    if (content)
    {
        QByteArray result;
        do
        {
            m_pSer->getVal(buffer, len, NULL);
            if (*len > 0)
            {
                result += QByteArray(&(*buffer), *len);
            }
        }
        while( *len > 0 );
        *content = result;
    }
    else
    {
        do
        {
            m_pSer->getVal(buffer, len, NULL);
        }
        while( *len > 0 );
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PiezosystemJena_NV40_1::readString(QByteArray &result, int &len, int timeoutMS, const QByteArray endline /*= ""*/)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;
    QByteArray endl = m_receiveEndline;
    if (!endline.isEmpty())
    {
        endl = endline;
    }

    bool done = false;
    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";
    int curFrom = 0;
    int pos = 0;

    if (!retValue.containsError())
    {
        len = 0;
        timer.start();

        while(!done && !retValue.containsError())
        {
            *curBufLen = buflen;
            retValue += m_pSer->getVal(curBuf, curBufLen, NULL);
            //qDebug() << *curBufLen;

            if (!retValue.containsError())
            {
                result += QByteArray(curBuf.data(), *curBufLen);
                pos = result.indexOf( endl, curFrom );
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
                retValue += ito::RetVal(ito::retError, PI_READTIMEOUT, tr("timeout").toLatin1().data());
            }
        }

        len = result.length();
    }

    if (!retValue.containsError())
    {
        QRegularExpression reg("^err,(\\d)$");
        QRegularExpressionMatch match = reg.match(result);
        if (match.hasMatch())
        {
            switch (match.captured(1).toInt())
            {
            case 1:
                retValue += ito::RetVal(ito::retError, 0, tr("unknown command").toLatin1().data());
                break;
            case 2:
                retValue += ito::RetVal(ito::retError, 0, tr("to many characters in the command").toLatin1().data());
                break;
            case 3:
                retValue += ito::RetVal(ito::retError, 0, tr("to many characters in the parameter").toLatin1().data());
                break;
            case 4:
                retValue += ito::RetVal(ito::retError, 0, tr("to many parameter").toLatin1().data());
                break;
            case 5:
                retValue += ito::RetVal(ito::retError, 0, tr("wrong character in parameter").toLatin1().data());
                break;
            case 6:
                retValue += ito::RetVal(ito::retError, 0, tr("wrong separator").toLatin1().data());
                break;
            case 7:
                retValue += ito::RetVal(ito::retError, 0, tr("overload").toLatin1().data());
                break;
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PiezosystemJena_NV40_1::sleep(int ms)
{
    QMutex mutex;
    mutex.lock();
    QWaitCondition waitCondition;
    waitCondition.wait(&mutex,ms);
    mutex.unlock();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PiezosystemJena_NV40_1::serialSendCommand(const QByteArray &command)
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

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string. 
            If string is invalid, val is not set and error-message is reported
    \param[in] *buf        Answer-String
    \param[out] val        double Value
    \sa PiezosystemJena_NV40_1::PIGetDouble
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::sendQuestionWithAnswerDouble( const QByteArray &questionCommand, double &answer, int timeoutMS )
{
    int readSigns;
    QByteArray _answer;
    bool ok = true;
    ito::RetVal retValue = serialSendCommand(questionCommand);
    retValue += readString(_answer, readSigns, timeoutMS);

    if (!retValue.containsError())
    {
        if(_answer.length() > questionCommand.length()+1)
        {
            int idx = _answer.indexOf(questionCommand);

            if (idx == -1)
            {
                ok = false;
            }
            else
            {
                _answer.remove(idx, questionCommand.length());
            }

            answer = _answer.toDouble(&ok);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("answer is too short").toLatin1().data());
        }

        if (retValue.errorCode() == PI_READTIMEOUT)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("timeout").toLatin1().data());
        }
        else if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("value '%1' could not be parsed to a double value").arg(QString(_answer)).toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail Returns a double value from the device answer stored in buffer. Tries to read an integer value and if this fails a double value from the string. 
            If string is invalid, val is not set and error-message is reported
    \param[out] *buf        Answer-String
    \param[in] bufsize        Number of signs to read
    \param[out] readsigns    Number of read signs
    \return retOk
*/
ito::RetVal PiezosystemJena_NV40_1::sendQuestionWithAnswerString(const QByteArray &questionCommand, QByteArray &answer, int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = serialSendCommand(questionCommand);

    if (questionCommand == "")
    {
        retValue += readString(answer, readSigns, timeoutMS, ">");
    }
    else
    {
        retValue += readString(answer, readSigns, timeoutMS);
    }

    if (retValue.errorCode() == PI_READTIMEOUT)
    {
        retValue = ito::RetVal(ito::retError, PI_READTIMEOUT, tr("timeout").toLatin1().data());
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal PiezosystemJena_NV40_1::identifyAndInitializeSystem(QString &identifier)
{
    ito::RetVal retval;

    serialDummyRead();

    //send an empty command (with \r at the end let the device answer with its name)
    QByteArray answer;
    retval += sendQuestionWithAnswerString("", answer, 2000); //this will lead to a timeout since the version does not answer with \r\n but with >

    if (!retval.containsError() || retval.errorCode() == PI_READTIMEOUT)
    {
        retval = ito::retOk;
        QRegularExpression reg("^NV1CL V(.*)$");
        QRegularExpressionMatch match= reg.match(answer);

        if (match.hasMatch())
        {
            identifier = match.captured(1);
        }
        else
        {
            retval += ito::RetVal::format(ito::retError, 0, tr("identification %s does not fit to required identification string NV1CL Vxxxx").toLatin1().data(), answer.data());
        }
    }

    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void PiezosystemJena_NV40_1::dockWidgetVisibilityChanged( bool visible )
{
    if (getDockWidget())
    {
        if (visible)
        {
            QObject::connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            QObject::connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::connect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
            emit parametersChanged(m_params);
            requestStatusAndPosition(true,true);
        }
        else
        {
            QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), getDockWidget()->widget(), SLOT(parametersChanged(QMap<QString, ito::Param>)));
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
        }
    }
}
