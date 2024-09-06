#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "MycobotControl.h"
#include <math.h>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qelapsedtimer.h>
#include <qwaitcondition.h>
#include "pluginVersion.h"
#include "gitVersion.h"

#include "common/helperCommon.h"
#include "common/paramMeta.h"

#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControlInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(MycobotControl)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControlInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(MycobotControl)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
MycobotControlInterface::MycobotControlInterface(QObject * /*parent*/)
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeActuator;
    setObjectName("MycobotControl");

    m_description = QObject::tr("A plugin to control MyCobot robots via TCP.");
    m_detaildescription = QObject::tr(
        "The MycobotControl plugin allows for controlling MyCobot robots over a TCP connection.\n"
        "It supports up to 6 axes and provides methods for absolute and relative positioning.\n");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal = ito::Param("numAxis", ito::ParamBase::Int, 6, new ito::IntMeta(1, 6), tr("Number of axes for this motor").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("host", ito::ParamBase::String, "127.0.0.1", tr("Hostname or IP address of the robot server").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("port", ito::ParamBase::Int, 9999, new ito::IntMeta(1, 65535), tr("Port number of the robot server").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
MycobotControlInterface::~MycobotControlInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal MycobotControl::showConfDialog(void)
{
    if (qobject_cast<QApplication*>(QCoreApplication::instance()))
        return apiShowConfigurationDialog(this, new DialogMycobotControl(this));
    else
        return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
MycobotControl::MycobotControl() :
    AddInActuator(),
    m_socket(nullptr),
    m_host("129.69.65.242"),
    m_port(9999)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "MycobotControl", "Name of the plugin");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 6, 6, tr("Number of axes attached to this stage").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("host", ito::ParamBase::String, m_host.toLatin1().data(), tr("Hostname or IP address of the robot server").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("port", ito::ParamBase::Int, m_port, tr("Port number of the robot server").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);


    m_targetPos = QVector<double>(6, 0.0);
 

    if (hasGuiSupport())
    {
        // Create dock widget for this plugin
        DockWidgetMycobotControl *mycobotWid = new DockWidgetMycobotControl(getID(), this);

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, mycobotWid);
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
MycobotControl::~MycobotControl()
{
    if (m_socket)
    {
        m_socket->disconnectFromHost();
        delete m_socket;
        m_socket = nullptr;
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal MycobotControl::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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

    if(isMotorMoving()) //this if-case is for actuators only.
    {
        retValue += ito::RetVal(ito::retError, 0, tr("any axis is moving. Parameters cannot be set.").toLatin1().data());
    }

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
    }

    if(!retValue.containsError())
    {
        if(key == "async")
        {
            m_async = val->getVal<int>();
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
        }
        else if(key == "demoKey2")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
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
ito::RetVal MycobotControl::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //steps todo:
    // - get all initialization parameters
    // - try to detect your device
    // - establish a connection to the device
    // - synchronize the current parameters of the device with the current values of parameters inserted in m_params
    // - if an identifier string of the device is available, set it via setIdentifier("yourIdentifier")
    // - set m_nrOfAxes to the number of axes
    // - resize and refill m_currentStatus, m_currentPos and m_targetPos with the corresponding values
    // - call emit parametersChanged(m_params) in order to propagate the current set of parameters in m_params to connected dock widgets...
    // - call setInitialized(true) to confirm the end of the initialization (even if it failed)


    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //todo:
    // - disconnect the device if not yet done
    // - this function is considered to be the "inverse" of init.

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::connectToSocket()
{
    // Check if the socket object has been created
    if (!m_socket)
    {
        m_socket = new QTcpSocket(this);  // Create a new QTcpSocket object, with 'this' as its parent
    }

    // If the socket is already connected, disconnect first
    if (m_socket->state() == QAbstractSocket::ConnectedState)
    {
        m_socket->disconnectFromHost();
        if (m_socket->state() != QAbstractSocket::UnconnectedState)
        {
            m_socket->waitForDisconnected();  // Wait for the socket to fully disconnect
        }
    }

    // Attempt to connect to the specified host and port
    m_socket->connectToHost(m_host, m_port);

    // Wait for the connection to complete, with a timeout of 5 seconds
    if (!m_socket->waitForConnected(5000))
    {
        // If the connection fails, return an error
        return ito::RetVal(ito::retError, 0, tr("Failed to connect to host %1 on port %2: %3")
            .arg(m_host).arg(m_port).arg(m_socket->errorString()).toLatin1().data());
    }

    // If the connection is successful, return a success status
    return ito::RetVal(ito::retOk, 0, tr("Successfully connected to host %1 on port %2").arg(m_host).arg(m_port).toLatin1().data());
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::sendSocketData(const QString &data)
{
    if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
        qint64 bytesWritten = m_socket->write(data.toUtf8());
        
        // 检查是否成功写入数据
        if (bytesWritten == -1) {
            return ito::RetVal(ito::retError, 0, QString("Failed to send data: %1").arg(m_socket->errorString()).toLatin1().data());
        }

        // 确保数据被完全发送
        if (!m_socket->flush()) {
            return ito::RetVal(ito::retError, 0, "Failed to flush data to the socket");
        }

        return ito::RetVal(ito::retOk);
    } else {
        return ito::RetVal(ito::retError, 0, "Socket not connected");
    }
}



//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosAbs(QVector<int>(1,axis), QVector<double>(1,pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach(const int i, axis)
        {
            if (i < 0 || i >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = 0.0; //todo: set the absolute target position to the desired value in mm or degree
            }
        }

        if (!retValue.containsError())
        {
            //set status of all given axes to moving and keep all flags related to the status and switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

            //todo: start the movement

            //emit the signal targetChanged with m_targetPos as argument, such that all connected slots gets informed about new targets
            sendTargetUpdate();

            //emit the signal sendStatusUpdate such that all connected slots gets informed about changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();

            //release the wait condition now, if async is true (itom considers this method to be finished now due to the threaded call)
            if(m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            //call waitForDone in order to wait until all axes reached their target or a given timeout expired
            //the m_currentPos and m_currentStatus vectors are updated within this function
            retValue += waitForDone(60000, axis); //WaitForAnswer(60000, axis);

            //release the wait condition now, if async is false (itom waits until now if async is false, hence in the synchronous mode)
            if(!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    //if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}



ito::RetVal MycobotControl::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    return setOrigin(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! setOrigin
/*!
    the given axes should be set to origin. That means (if possible) their current position should be
    considered to be the new origin (zero-position). If this operation is not possible, return a
    warning.
*/
ito::RetVal MycobotControl::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach(const int &i,axis)
        {
            if (i >= 0 && i < m_nrOfAxes)
            {
                //todo: set axis i to origin (current position is considered to be the 0-position).
            }
            else
            {
                retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
        }

        retValue += updateStatus();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosRel(QVector<int>(1,axis), QVector<double>(1,pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Additional actions are not possible.").toLatin1().data());
    }
    else
    {
        foreach(const int i, axis)
        {
            if (i < 0 || i >= m_nrOfAxes)
            {
                retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(), i);
            }
            else
            {
                m_targetPos[i] = 0.0; //todo: set the absolute target position to the desired value in mm or degree
                                      //(obtain the absolute position with respect to the given relative distances)
            }
        }

        if (!retValue.containsError())
        {
            //set status of all given axes to moving and keep all flags related to the status and switches
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);

            //todo: start the movement

            //emit the signal targetChanged with m_targetPos as argument, such that all connected slots gets informed about new targets
            sendTargetUpdate();

            //emit the signal sendStatusUpdate such that all connected slots gets informed about changes in m_currentStatus and m_currentPos.
            sendStatusUpdate();

            //release the wait condition now, if async is true (itom considers this method to be finished now due to the threaded call)
            if(m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            //call waitForDone in order to wait until all axes reached their target or a given timeout expired
            //the m_currentPos and m_currentStatus vectors are updated within this function
            retValue += waitForDone(60000, axis); //WaitForAnswer(60000, axis);

            //release the wait condition now, if async is false (itom waits until now if async is false, hence in the synchronous mode)
            if(!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    //if the wait condition has not been released yet, do it now
    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::calib(const int axis, ItomSharedSemaphore *waitCond)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
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
ito::RetVal MycobotControl::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QSharedPointer<QVector<double> > pos2(new QVector<double>(1,0.0));
    ito::RetVal retValue = getPos(QVector<int>(1,axis), pos2, NULL); //forward to multi-axes version
    *pos = (*pos2)[0];

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    foreach (const int i, axis)
    {
        if (i >= 0 && i < m_nrOfAxes)
        {
            //obtain current position of axis i
            //transform tempPos to angle
            m_currentPos[i] = 0.0; //set m_currentPos[i] to the obtained position
            (*pos)[i] = m_currentPos[i];
        }
        else
        {
            retValue += ito::RetVal::format(ito::retError, 1, tr("axis %i not available").toLatin1().data(),i);
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
// ito::RetVal MycobotControl::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
// {

// }

//----------------------------------------------------------------------------------------------------------------------------------
void MycobotControl::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), widget, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            connect(this, SIGNAL(targetChanged(QVector<double>)), widget, SLOT(targetChanged(QVector<double>)));

            emit parametersChanged(m_params);
            sendTargetUpdate();
            sendStatusUpdate(false);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), widget, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            disconnect(this, SIGNAL(targetChanged(QVector<double>)), widget, SLOT(targetChanged(QVector<double>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MycobotControl::waitForDone(const int timeoutMS, const QVector<int> axis, const int /*flags*/)
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

    //if axis is empty, all axes should be observed by this method
    QVector<int> _axis = axis;
    if (_axis.size() == 0) //all axis
    {
        for (int i=0;i<m_nrOfAxes;i++)
        {
            _axis.append(i);
        }
    }

    while (!done && !timeout)
    {
        //todo: obtain the current position, status... of all given axes

        done = true; //assume all axes at target
        motor = 0;
        foreach(const int &i,axis)
        {
            m_currentPos[i] = 0.0; //todo: if possible, set the current position if axis i to its current position

            if (1 /*axis i is still moving*/)
            {
                setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                done = false; //not done yet
            }
            else if (1 /*axis i is at target*/)
            {
                setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                done = false; //not done yet
            }
        }

        //emit actuatorStatusChanged with both m_currentStatus and m_currentPos as arguments
        sendStatusUpdate(false);

        //now check if the interrupt flag has been set (e.g. by a button click on its dock widget)
        if (!done && isInterrupted())
        {
            //todo: force all axes to stop

            //set the status of all axes from moving to interrupted (only if moving was set before)
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);

            retVal += ito::RetVal(ito::retError,0,"interrupt occurred");
            done = true;
            return retVal;
        }

        //short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, delay);
        waitMutex.unlock();

        //raise the alive flag again, this is necessary such that itom does not drop into a timeout if the
        //positioning needs more time than the allowed timeout time.
        setAlive();

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS) timeout = true;
        }
    }

    if (timeout)
    {
        //timeout occurred, set the status of all currently moving axes to timeout
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError,9999,"timeout occurred");
        sendStatusUpdate(true);
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
