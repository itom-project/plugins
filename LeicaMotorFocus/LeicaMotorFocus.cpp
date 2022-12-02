/* ********************************************************************
    Plugin "LeicaMotorFocus" for itom software
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

#include "LeicaMotorFocus.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qelapsedtimer.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

enum lmfUnit {
       UNIT_PRESENT        = 1,
       GET_VERSION        = 2,
       MOVE_UP            = 10,
       MOVE_DOWN        = 11,
       MOVE_STOP        = 12,
       SET_REFERENCE    =     13,
       MOVE_ABSOLUTE    =     30,
       MOVE_RELATIVE    =     31,
       GET_POSITION        = 32,
       SET_VELOCITY        = 33,
       GET_VELOCITY        = 34,
       GET_STATUS_BYTE    =     35,
       GET_STATUS_BITS    =     80,
       SET_UPPER_LIMIT    =     36,
       SET_LOWER_LIMIT    =     37,
       GET_UPPER_LIMIT    =     38,
       GET_LOWER_LIMIT    =     39,
       MOVE_REL_INC        = 49,
       BEEP            = 60,
       SET_BEEP_ENABLE    =     92,
       GET_BEEP_ENABLE    =     93,
       TOGGLE_BEEP_ENABLE =     94
};

enum lmfManCMD {
      CLEAR_POS_1        = 20,
      CLEAR_POS_2        = 21,
      CLEAR_POS_3        = 22,
      CLEAR_POS_4        = 23,
      CLEAR_POS_5        = 24,
      SET_RATIO            = 33,
      GET_RATIO            = 34,
      GET_KEY_STATE        = 35,
      SAVE_POS_1        = 40,
      SAVE_POS_2        = 41,
      SAVE_POS_3        = 42,
      SAVE_POS_4        = 43,
      SAVE_POS_5        = 44,
      GET_POS_1        = 45,
      GET_POS_2        = 46,
      GET_POS_3        = 47,
      GET_POS_4        = 48,
      GET_POS_5        = 49,
      SET_ENABLE_REPORT_KEY    = 91
};

enum lmfStatus {
    STATUS_OK       = 0x0000,
    STATUS_ACTIVE   = 0x0001, //in every moving operation the first bit is set
    STATUS_HARDWARE_ERROR1 = 0x0002,
    STATUS_HARDWARE_ERROR2 = 0x0004,
    STATUS_MOVING   = 0x0020, //bit 6: any moving operation is going on
    STATUS_UPPER_REF_SWITCH = 0x0040, //bit 7: upper reference switch is reached
    STATUS_LOWER_REF_SWITCH = 0x0080 //bit 8: lower reference switch is reached
};

#define FULLSPEED    (140/6.0)    /* 140mm in 6s */
#define LMFDELAY 10

/*static*/ QSharedPointer<QVector<ito::ParamBase> > LeicaMotorFocus::emptySharedParamBaseVec(new QVector<ito::ParamBase>());

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocusInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(LeicaMotorFocus)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocusInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(LeicaMotorFocus)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
LeicaMotorFocusInterface::LeicaMotorFocusInterface()
{
    m_type = ito::typeActuator;
    setObjectName("LeicaMotorFocus");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The LeiceMotorFocus is an itom-plugin, which can be used to control the z-stage of Leica MZ12 or MZ12.5 stereo-microscopes.\n\
\n\
For the initialization of this plugin you already need an opened serial IO port (using the plugin 'SerialIO'). Give the handle of \
the opened serial port to the constructor of this plugin. This plugin will keep a reference of the serial port until the actuator \
is closed again.";
*/
    m_description = QObject::tr("Plugin for Leica MZ12.X focus actuator");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"The LeiceMotorFocus is an itom-plugin, which can be used to control the z-stage of Leica MZ12 or MZ12.5 stereo-microscopes.\n\
\n\
For the initialization of this plugin you already need an opened serial IO port (using the plugin 'SerialIO'). Give the handle of \
the opened serial port to the constructor of this plugin. This plugin will keep a reference of the serial port until the actuator \
is closed again.");

    m_author = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);     

    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An initialized SerialIO").toLatin1().data());
    paramVal.setMeta( new ito::HWMeta("SerialIO"), true );
    m_initParamsMand.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
LeicaMotorFocusInterface::~LeicaMotorFocusInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------

const ito::RetVal LeicaMotorFocus::showConfDialog(void)
{
    DialogLeicaMotorFocus *confDialog = new DialogLeicaMotorFocus(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(parametersChanged(QMap<QString, ito::Param>)));
    QMetaObject::invokeMethod(this, "sendParameterRequest");

    confDialog->exec(); //wait until done
    disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(parametersChanged(QMap<QString, ito::Param>)));  
    delete confDialog;

    return ito::retOk;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// read buffer without delay
const ito::RetVal LeicaMotorFocus::LMFDummyRead()
{
	m_pSer->execFunc("clearInputBuffer", emptySharedParamBaseVec, emptySharedParamBaseVec, emptySharedParamBaseVec);
	m_pSer->execFunc("clearOutputBuffer", emptySharedParamBaseVec, emptySharedParamBaseVec, emptySharedParamBaseVec);
    return ito::retOk;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LeicaMotorFocus::LMFReadString(char *buf, const int bufsize, int * readsigns)
{
    ito::RetVal retval = ito::retOk;
    QSharedPointer<int> len(new int);

	*len = bufsize;
    QSharedPointer<char> tempBuf(buf, LeicaMotorFocus::doNotDelSharedPtr); //trick to access part of buf using a shared pointer. the shared pointer is not allowed to delete the char-array, therefore the Deleter-method.
    retval += m_pSer->getVal(tempBuf, len);
	*readsigns = *len;
	return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LeicaMotorFocus::LMFWriteCmd(int id, int cmd)
{    
    ito::RetVal retval = ito::retOk;
    char query[20];
    memset(&query, 0, 20*sizeof(char));

    sprintf(query, "%d %03d", id, cmd);

    retval += m_pSer->setVal(&query[0], (int)strlen(query),0);

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LeicaMotorFocus::LMFWriteCmdArg(int id, int cmd, long arg)
{    
    ito::RetVal retval = ito::retOk;
    char query[20];
    memset(&query, 0, 20*sizeof(char));

    sprintf(query, "%d %03d %ld", id, cmd, arg);

    retval += m_pSer->setVal(&query[0], (int)strlen(query),0);

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LeicaMotorFocus::LMFQueryS(int id, int cmd, char *buf, int bufsize)
{
    ito::RetVal retval = ito::retOk;

    char answer[50];
    char *copy;
    char *ptr;
    int got_id=0, got_cmd=0;
    int buflen=50;
    int redsigns=0;

    if (bufsize < 1)
        return ito::retError;

    retval += this->LMFDummyRead();

    retval += this->LMFWriteCmd(id, cmd);
    if (retval == ito::retError)
    {
        return retval;
    }
    memset(answer, 0, 50*sizeof(char));
    retval += this->LMFReadString(&answer[0], buflen, &redsigns);
    if (retval == ito::retError)
    {
        return retval;
    }
    if (redsigns > 0)
    {
        if (answer[0] == 17)
        {
            retval += ito::RetVal(ito::retWarning, 0, tr("Unexspected #(17) at first sign in buffer. Sign deleted").toLatin1().data());
            copy = _strdup(&answer[1]);
        }
        else
        {
            copy = _strdup(answer);
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("No answer read").toLatin1().data());
        return retval;
    }
    /* check if response fits query */
    
    if (!copy)
    {
        return ito::retError;
    }
    if (NULL!=(ptr = strtok(copy, " ")))
    {
        got_id = atol(ptr);
    }
    if (NULL!=(ptr = strtok(NULL, " ")))
    {
        got_cmd = atol(ptr);
    }
    if (id != got_id || cmd != got_cmd) 
    {
        retval += ito::RetVal(ito::retError, 0, tr("Answer \"%1\" to command '%2 %3' does not match query").arg(answer).arg(id).arg(cmd).replace('\r', "\\r").replace('\n', "\\n").toLatin1().data());
        free(copy);
        return retval;
    }

    ptr = strtok(NULL, " ");
    free(copy);
    if (!ptr) 
    {
        retval += ito::RetVal(ito::retError, 0, tr("Malformed answer '%1'").arg(answer).toLatin1().data());
        return retval;
    }

    /* only return part after returned id and cmd */
    //_snprintf(buf, bufsize, "%s", answer + (ptr-copy));
    sprintf(buf, "%s", answer + (ptr-copy));

    /* discard any pending input (?) */
    retval += this->LMFDummyRead();

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LeicaMotorFocus::LMFQueryL(int id, int cmd, long *plval)
{
    ito::RetVal retval = ito::retOk;
    char answer[50];

    answer[0] = '\0';
    retval += this->LMFQueryS(id, cmd, answer, sizeof(answer));
    if (retval!=ito::retError)
    {
        *plval = atol(answer);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocus::waitForDone(const int timeoutMS, const QVector<int> axis, const int /*flags*/)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    long delay = LMFDELAY; //[ms]
    QSharedPointer<double> actPos = QSharedPointer<double>(new double);
    int status = 0;
    int counter = 0;

    timer.start();

    QVector<int> _axis = axis;
    if (_axis.size() == 0) //all axis
    {
        _axis.append(0);
    }

    while (!done && !timeout)
    {
        retVal += LMFStatus(status);
        if (retVal.containsError())
        {
            done = true;
        }

        if ( status == STATUS_OK)
        {
            done = true;
        }
        else if (status & STATUS_UPPER_REF_SWITCH)
        {
            //retry to verify
            setAlive();
            Sleep(100);
            retVal += LMFStatus(status);

            if (status & STATUS_UPPER_REF_SWITCH)
            {
                qDebug() << "LeicaMotorFocus Status: " << status << " UPPER_REF:" << STATUS_UPPER_REF_SWITCH;
                retVal += ito::RetVal(ito::retError, STATUS_UPPER_REF_SWITCH, tr("upper reference switch reached").toLatin1().data());
                done = true;
            }
        }
        else if (status & STATUS_LOWER_REF_SWITCH)
        {
            //retry to verify
            setAlive();
            Sleep(100);
            retVal += LMFStatus(status);

            if (status & STATUS_LOWER_REF_SWITCH)
            {
                qDebug() << "LeicaMotorFocus Status: " << status << " LOWER_REF:" << STATUS_LOWER_REF_SWITCH;
                retVal += ito::RetVal(ito::retError, STATUS_LOWER_REF_SWITCH, tr("lower reference switch reached").toLatin1().data());
                done = true;
            }
        }
        
        setAlive();

        if (counter == 10) //actualize position every 10. round
        {
            counter = 0;
            retVal += getPos(0, actPos, NULL);
            sendStatusUpdate(false);
        }
        else
        {
            counter ++;
            sendStatusUpdate(true);
        }

        if (!done && isInterrupted())
        {
            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);

            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;
            return retVal;
        }

        QCoreApplication::processEvents();

        //short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, delay);
        waitMutex.unlock();
        setAlive();

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS) timeout = true;
        }
    }

    retVal += LMFStatus(status);
    retVal += getPos(0, actPos, NULL);
    sendStatusUpdate(false);

    if (timeout)
    {
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError, 9999, tr("timeout occurred").toLatin1().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LeicaMotorFocus::LMFStatus(int &status)
{
    ito::RetVal retval = ito::retOk;
    long lval=0;
    retval += LMFQueryL(70, GET_STATUS_BYTE, &lval);

    if (retval!=ito::retOk)
    {
        status = -1;
    }
    else
    {
        status = static_cast<int>(lval);

        if (status == STATUS_OK)
        {
            if (!(m_currentStatus[0] & ito::actuatorInterrupted)) //if interrupted, keep that status
            {
                setStatus(m_currentStatus[0], ito::actuatorAtTarget, ito::actStatusMask);
            }
        }
        else if (lval & STATUS_HARDWARE_ERROR1 || lval & STATUS_HARDWARE_ERROR2)
        {
            setStatus(m_currentStatus[0], ito::actuatorUnknown, ito::actStatusMask);
            retval += ito::RetVal(ito::retError, 0, tr("Hardware error during status check").toLatin1().data());
        }
        else if (status == (STATUS_MOVING | STATUS_ACTIVE))
        {
            setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actStatusMask);
        }
        else if (status & STATUS_UPPER_REF_SWITCH)
        {
            setStatus(m_currentStatus[0], ito::actuatorRefSwitch | ito::actuatorRightEndSwitch | ito::actuatorEndSwitch | ito::actuatorRightRefSwitch, ito::actStatusMask);
        }
        else if (status & STATUS_LOWER_REF_SWITCH)
        {
            setStatus(m_currentStatus[0], ito::actuatorRefSwitch | ito::actuatorLeftEndSwitch | ito::actuatorEndSwitch | ito::actuatorLeftRefSwitch, ito::actStatusMask);
        }
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
LeicaMotorFocus::LeicaMotorFocus() : AddInActuator(), m_async(0), m_direction(1)/*, m_posrequestlisteners(0)*/
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "LeicaMotorFocus", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    
    m_scale = 1e3; // Leica is Programmes in mu m, this evil Programm sent in mm

    paramVal = ito::Param("speed", ito::ParamBase::Double, FULLSPEED/1000, FULLSPEED, FULLSPEED, tr("Speed in m/s (Default=Maximum: 23,33 mm/s)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("ratio", ito::ParamBase::Int, 0, 32, 8, tr("Sensitivity of Handwheel. From 1 (fine) to 32 (coarse) (Default: 8)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inverseAxis", ito::ParamBase::Int, 0, 1, 0, tr("0: actuator moves upwards for positive relative position, 1: actuator moves downwards. (default: 0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inverseRefSwitch", ito::ParamBase::Int, 0, 1, 0, tr("0: actuator uses upper reference switch, 1: actuator uses lower reference switch for calibration. (default: 0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 1, 1, 1, tr("Number of axis @ device in ito-version is always 1").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    m_currentStatus.fill(0,1);
    m_currentStatus[0] = ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
    m_currentPos.fill(0,1);
    m_targetPos.fill(0,1);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetLeicaMotorFocus *LMFWid = new DockWidgetLeicaMotorFocus(m_params, getID(), this);
        connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), LMFWid, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        connect(LMFWid, SIGNAL(MoveRelative(const int, const double, ItomSharedSemaphore *)), this, SLOT(setPosRel(const int, const double, ItomSharedSemaphore *)));
        connect(LMFWid, SIGNAL(MoveAbsolute(const int, const double, ItomSharedSemaphore *)), this, SLOT(setPosAbs(const int, const double, ItomSharedSemaphore *)));
        connect(LMFWid, SIGNAL(MotorTriggerStatusRequest(bool, bool)), this, SLOT(requestStatusAndPosition(bool, bool)));

        connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), LMFWid, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
        connect(this, SIGNAL(targetChanged(QVector<double>)), LMFWid, SLOT(targetChanged(QVector<double>)));

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, LMFWid);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete Param of the corresponding parameter to val

    \param [in,out] val  is a input of type::Param containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal LeicaMotorFocus::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
/*!
    \detail This method copies the value of val to to the m_params-parameter and sets the corresponding camera parameters.

    \param [in] val  is a input of type::ParamBase containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal LeicaMotorFocus::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    long stepperspeed = 1000;
    long ratio = 8;

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toLatin1().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
        {

            if (paramIt->getFlags() & ito::ParamBase::Readonly)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toLatin1().data());
            }
            else if (val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if ( curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toLatin1().data());
                }
                else if (curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toLatin1().data());
                }
            }
            else if (paramIt->getType() == val->getType())
            {
                //retValue += paramIt.value().copyValueFrom( &(*val) );
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toLatin1().data());
            }

            if (!retValue.containsWarningOrError())
            {

                retValue += paramIt.value().copyValueFrom( &(*val) );

                if (key == "speed")
                {
                    stepperspeed = (long)(m_params["speed"].getVal<double>()/ FULLSPEED * 1000 + 0.5); // Speed is something between 1 and 1000
                    retValue += this->LMFWriteCmdArg(70,SET_VELOCITY,stepperspeed);
                }
                else if (key == "ratio")
                {
                    ratio = (long)m_params["ratio"].getVal<int>();
                    retValue += this->LMFWriteCmdArg(71, SET_RATIO, ratio);
                }
                else if (key == "inverseAxis")
                {
                    m_direction = m_params["inverseAxis"].getVal<int>()>0?-1:1;
                }
            }
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
        }
    }

    if (!retValue.containsWarningOrError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocus::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore *waitCond)
{   
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    long speed;
    char unit70[30], version70[20], unit71[30], version71[20];

// TODO: Pruefen ob uebergebener Parameter tatsaechlich eine serielle Schnittstelle ist!
//    m_pSer = qobject_cast<ito::AddInDataIO*>( reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>()) );
//    if (m_pSer)
//    {
    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud",ito::ParamBase::Int,9600)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits",ito::ParamBase::Int,8)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity",ito::ParamBase::Double,0.0)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits",ito::ParamBase::Int,1)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow",ito::ParamBase::Int,1)),NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline",ito::ParamBase::String,"\r\n")),NULL);
		retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endlineRead",ito::ParamBase::String,"\r\n")),NULL);
		retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("readline",ito::ParamBase::Int,1)),NULL);
        retval += this->LMFDummyRead();
    }
    else
    {
        retval += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
    }

    if (retval!=ito::retError)
    {
        retval += this->LMFQueryS(70, UNIT_PRESENT, unit70, sizeof(unit70));
    }
    if (retval != ito::retError)
    {
        retval += this->LMFQueryS(70, GET_VERSION, version70, sizeof(version70));
    }
    if (retval != ito::retError)
    {
        retval += this->LMFQueryS(70, UNIT_PRESENT, unit71, sizeof(unit71));
    }
    if (retval != ito::retError)
    {
        retval += this->LMFQueryS(70, GET_VERSION, version71, sizeof(version71));
    }

    /* Get the startup velocity */
    if (retval != ito::retError)
    {
        retval += LMFQueryL(70, GET_VELOCITY, &speed);
        m_params["speed"].setVal<double>(speed / 1000 * FULLSPEED); // Fullspeed is 140mm / 6s and answer is in Promill of Fullspeed
    }

    if (retval != ito::retError)
    {
        requestStatusAndPosition(true,true); //initial position check
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
ito::RetVal LeicaMotorFocus::close(ItomSharedSemaphore *waitCond)
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
ito::RetVal LeicaMotorFocus::calib(const int /*axis*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    int status;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toLatin1().data());
    }
    else
    {
        retval += LMFStatus(status);
        sendStatusUpdate(true);    

        setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate(true);    
    
        long oldspeed;
        QSharedPointer<double> oldPos(new double);
        QSharedPointer<double> endSwitchPos(new double);
    
        /* Referenz goes up (!) */
        /* temporarily set speed to maximum */
        retval += LMFQueryL(70, GET_VELOCITY, &oldspeed);
        if (retval != ito::retError)
        {
            retval += LMFWriteCmdArg(70, SET_VELOCITY, 1000);    // Ugly SET_VELOCITY is something between 1 and 1000
        }
        // insert wait

        retval += getPos(0, oldPos, NULL); //get current, uncalibrated position

        int refSwitchDirection = m_params["inverseRefSwitch"].getVal<int>();

        if (retval != ito::retError)
        {
            if (refSwitchDirection == 0)
            {
                retval += LMFWriteCmd(70, MOVE_UP);
            }
            else
            {
                retval += LMFWriteCmd(70, MOVE_DOWN);
            }
        }
        if (retval != ito::retError)
        {
            retval += waitForDone(10000);
            if (retval.containsError())
            {
                if (retval.errorCode() == STATUS_UPPER_REF_SWITCH && refSwitchDirection == 0)
                {
                    retval = ito::retOk;

                }
                else if (retval.errorCode() == STATUS_LOWER_REF_SWITCH && refSwitchDirection != 0)
                {
                    retval = ito::retOk;
                }

                retval += getPos(0, endSwitchPos, NULL); //get current, uncalibrated position at the end switch
            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, tr("error: no reference switch reached").toLatin1().data());
                *oldPos = 0.0;
                *endSwitchPos = 0.0;
            }
        }
        if (retval != ito::retError)
        {
            retval += LMFWriteCmd(70, SET_REFERENCE);
        }

        if (retval != ito::retError)
        {
            retval += LMFWriteCmdArg(70, SET_VELOCITY, oldspeed);
        }

        //now move stage to position, where it has been at startup
        retval += setPosRel(0, *oldPos - *endSwitchPos ,NULL);

        retval += LMFStatus(status);
        sendStatusUpdate(true);    
    }

    if (waitCond)
    {
        waitCond->release();
        waitCond->returnValue = retval;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocus::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    if (axis.size() == 1)
    {
        return calib(axis[0], waitCond);
    }
    else
    {
        ItomSharedSemaphoreLocker locker(waitCond);
        ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Error. Number of axis must be 1.").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
        return retval;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocus::setOrigin(const int /*axis*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toLatin1().data());
    }
    else
    {
        retval = LMFWriteCmd(70,SET_REFERENCE);
        m_currentPos[0] = 0.0;
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
ito::RetVal LeicaMotorFocus::setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    if (axis.size() == 1)
    {
        return setOrigin(axis[0], waitCond);
    }
    else
    {
        ItomSharedSemaphoreLocker locker(waitCond);
        ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Error. Number of axes must be 1.").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
        return retval;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocus::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    int s;
    ito::RetVal retval = LMFStatus(s);
    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal LeicaMotorFocus::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
//    double axpos = 0.0;
    long retPos = 0;
    ito::RetVal retval = ito::retOk;

    if (axis != 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toLatin1().data());
    }
    else
    {
        retval += LMFDummyRead();
        retval += LMFQueryL(70, GET_POSITION, &retPos);

        if (retval != ito::retError)
        {
                *pos = (double)retPos / m_scale * m_direction;
                m_currentPos[0] = *pos;
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
ito::RetVal LeicaMotorFocus::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);

    if ((axis.size() == 1) && (axis.value(0) == 0))
    {
        retval +=getPos(axis.value(0),sharedpos,0);
        (*pos)[0] = *sharedpos;
    }
    else
    {
        retval +=ito::RetVal(ito::retError, 0, tr("Error. Too many Axis / wrong Axis").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal LeicaMotorFocus::LMFSetPos(QVector<int> axis, const double dpos, const int absrelflag, ItomSharedSemaphore *waitCond)
{
    long lpos = 0;
    double dpos_temp = dpos * m_direction * m_scale;    // Mirror the new pos and set this to m_scale
    ito::RetVal retValue = ito::retOk;
    bool released = false;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toLatin1().data());
    }
    else
    {
        setStatus(m_currentStatus[0], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        sendStatusUpdate(false);    

        if (absrelflag == MOVE_RELATIVE)
        {
            m_targetPos[0] += dpos;
        }
        else
        {
            m_targetPos[0] = dpos;
        }

        sendTargetUpdate();
    
        retValue += LMFDummyRead();

        if (dpos_temp < 0)
        {
            lpos=((long)(dpos_temp - .5));
        }
        else
        {
            lpos=((long)(dpos_temp + .5));
        }
        retValue += LMFWriteCmdArg(70, absrelflag, lpos);

        if (m_async && waitCond && !released)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
            released = true;
        }

        retValue += waitForDone(10000, axis );

        if (!m_async && waitCond && !released)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
            released = true;
        }
/*
        if (!m_async && retval == ito::retOk)
        {
            retval += LMFWaitForAnswer(10000);
        }

        emit PositioningStatusChanged(0);
        if (m_posrequestlisteners && !m_async)
        {
            QSharedPointer<QVector<double> > sharedpos = QSharedPointer<QVector<double> >(new QVector<double>);

            for(int i=0;i<axis.size();i++)
            {
                *sharedpos << 0;
            }
            retval+=getPos(axis,sharedpos,0);
            emit SentPositionChanged(axis,*sharedpos);    
        }*/
    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        released = true;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------        
ito::RetVal LeicaMotorFocus::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    double target_pos = pos;
    
    if (axis!=0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
    }
    else
    {
        retval = LMFSetPos(QVector<int>(1, axis), target_pos, MOVE_ABSOLUTE, waitCond);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
ito::RetVal LeicaMotorFocus::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    if (axis.size() == 1)
    {
        return setPosAbs(axis[0], pos[0], waitCond);
    }
    else
    {
        ItomSharedSemaphoreLocker locker(waitCond);
        ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Error. Number of axis must be 1.").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
        return retval;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------        
ito::RetVal LeicaMotorFocus::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    double target_pos = pos;

    if (axis!=0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis does not exist").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }
    }
    else
    {
        retval = LMFSetPos(QVector<int>(1, axis), target_pos, MOVE_RELATIVE, waitCond);
    }
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------        
ito::RetVal LeicaMotorFocus::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    if (axis.size() == 1)
    {
        return setPosRel(axis[0], pos[0], waitCond);
    }
    else
    {
        ItomSharedSemaphoreLocker locker(waitCond);
        ito::RetVal retval = ito::RetVal(ito::retError, 0, tr("Error. Too many Axis").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }

        return retval;
    }
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal LeicaMotorFocus::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);
    int status;

    QSharedPointer<double> sharedpos = QSharedPointer<double>(new double);
    *sharedpos = 0.0;

    retval += LMFStatus(status);

    if (sendCurrentPos)
    {
        retval += getPos(0,sharedpos,0);
        m_currentPos[0] = *sharedpos;

        if (status == 0 && std::abs(*sharedpos-m_targetPos[0]) > 0.01)
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

//---------------------------------------------------------------------------------------------------------------------------------- 
void LeicaMotorFocus::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        if (visible)
        {
            QObject::connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::connect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
            requestStatusAndPosition(true,true);
        }
        else
        {
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
        }
    }
}

