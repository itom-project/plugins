#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "UhlRegister.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

//#include "common/helperCommon.h"
#define NOMINMAX

#ifdef __linux__
    #include <unistd.h>
#else
    #include <windows.h>
#endif

enum uhlCmdReg {
    XPOS        =   0,      // traverse paths
//    YPOS        =   1,
//    ZPOS        =   2,
    XABSPOS     =   3,
    STATUS      =   6,
    COMMAND     =   7,      // command register
    RAMP        =   8,      // acceleration [0..99]
    REVSPEED    =   9,      // revolution speed in .1 rps [0..90]
//    CURRENT     =   10,     // electrical current in idle mode [0..10]
    MASK        =   11,     // active axis
    SDELAY      =   12,     // delay of answers of control the serial interface by 2ms steps, 0..18ms
    XPERIOD     =   27,     // partition period X
    READ        =   64,     // offset write register -> read register
    START       =   80,
//    SCALEREG    =   103,
    POSREG      =   107
};

#define UHLDELAY 30

enum uhlCmd {
    ABORT       =   'a',
    CALIBRATE   =   'c',
    GOABSEXTCLK =   'e',
    GORELEXTCLK =   'g',
    JOYON       =   'j',
//    JOYON2      =   's',
    GOEND       =   'l',
    GOABSINTCLK =   'r',
    GORELINTCLK =   'v'
};

enum uhlStatus {
    STATUS_UPPER_REF_SWITCH = 0x0040, //bit 7: upper reference switch is reached
    STATUS_LOWER_REF_SWITCH = 0x0080 //bit 8: lower reference switch is reached
};

#define SPITCH 10000L // spindle pitch

//----------------------------------------------------------------------------------------------------------------------------------
int GetAxisDir(const int value)
{
    if (value == 1)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegisterInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(UhlRegister)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegisterInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(UhlRegister)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
UhlRegisterInterface::UhlRegisterInterface()
{
    m_type = ito::typeActuator;
    setObjectName("UhlRegister");

    m_description = tr("DLL for old 2-3 axis Uhl-Controller");
    m_detaildescription = tr("The UhlRegister is a plugin, which can be used to control the 2-3 axis\n\
stepper motor devices from Uhl (F9S-x)\n\
\n\
It is initialized by actuator(\"UhlRegister\", SerialIO, ...).\n\
\n\
WARNING: There are different controller versions with different\n\
command languages. This DLL is for F9S Register devices.\n\
WARNING: The calibration direction of the stages differs according to motor / controller.\n\
Check calibration direction before usage. \n\
\n\
This plugin was published with the kind permission of company Walter Uhl, technische Mikroskopie GmbH & Co. KG");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An initialized SerialIO").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("calibration", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("1 -> calibration during initialization").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("inversex", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("Invert axis direction for x").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("inversey", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("Invert axis direction for y").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("inversez", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("Invert axis direction for z").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("joyenabled", ito::ParamBase::Int, 1, new ito::IntMeta(0,1), tr("Enabled/disabled Joystick. Default: enabled").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
UhlRegisterInterface::~UhlRegisterInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------


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
const ito::RetVal UhlRegister::showConfDialog(void)
{
    dialogUhl *confDialog = new dialogUhl(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(parametersChanged(QMap<QString, ito::Param>)));
    //connect(confDialog, SIGNAL(sendParamVector(const QVector< QSharedPointer<ito::tParam> >,ItomSharedSemaphore*)), this, SLOT(setParamVector(const QVector<QSharedPointer<ito::tParam> >,ItomSharedSemaphore*)));

    QMetaObject::invokeMethod(this, "sendParameterRequest"); //requests plugin to send recent parameter map to dialog

    confDialog->exec();
    delete confDialog;
    confDialog = NULL;
    return ito::retOk;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::AnalyseAnswer(char *bufData)
{
    ito::RetVal retval(ito::retOk);

    if (!strncmp(bufData, "ERR", 3))
    {
        QString ErrMsg = tr("UHL errormessage: ");
//        QString ErrCode = bufData[4];
//        switch (ErrCode.toInt())
        switch (atoi(&bufData[4]))
        {
            case 1:
            {
                ErrMsg += tr("Wrong command!");
                break;
            }
            case 2:
            {
                ErrMsg += tr("Wrong register to read!");
                break;
            }
            case 3:
            {
                ErrMsg += tr("Wrong data!");
                break;
            }
            case 4:
            {
                ErrMsg += tr("Wrong register to write!");
                break;
            }
            case 5:
            {
                ErrMsg += tr("State message in status register!");
                break;
            }
            case 6:
            {
                ErrMsg += tr("Too much axes in register!");
                break;
            }
            default:
            {
                ErrMsg += tr("Undefined answer from serial port!");
            }
        }
        retval = ito::RetVal(ito::retWarning, 0, ErrMsg.toLatin1().data());
    }
    else if ((bufData[0] == 'D') || (bufData[1] == 'D') || (bufData[2]=='D'))
    {
        retval = ito::RetVal(ito::retWarning, 0, tr("E-Strike detected in serial buffer").toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
// to synchronize required position = current position
const ito::RetVal UhlRegister::UhlAxisSync()
{
    ito::RetVal ret = ito::retOk;
    int axis = 0;
    long pos = 0;

    // for (axis = 0; axis < m_numAxis; axis++)
    for (axis = 0; axis < 3; axis++)
    {
        ret += UhlReadRegL(XABSPOS + READ + axis, &pos);
        if (ret.containsError())
        {
            return ret;
        }
        ret += UhlWriteRegL(XPOS + axis, pos);
        if (ret.containsError())
        {
            return ret;
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
// read buffer without delay
const ito::RetVal UhlRegister::DummyRead()
{
    ito::RetVal retval(ito::retOk);

    QSharedPointer<char> buf(new char[50]);
    *buf = '0';
    char* bufData = buf.data();
    QSharedPointer<int> bufsize(new int);
    *bufsize = 49;
    bufData[*bufsize] = 0;
    m_pSer->getVal(buf, bufsize);

    if (*bufsize > 0)
    {
        bufData[*bufsize] = 0;
        retval = AnalyseAnswer(bufData);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlReadString(char *buf, const int bufsize)
{
    ito::RetVal retval = ito::retOk;
    QSharedPointer<int> len(new int);
    *len = 0;
    QSharedPointer<char> tempBuf;
    int totlen = 0;
    static char endline[3] = {0, 0, 0};

    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retval += m_pSer->getParam(param, NULL);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); //borrowed reference
        endline[0] = temp[0];
        endline[1] = temp[1];
        endline[2] = temp[2];
    }

    totlen = 0;
    do
    {
        *len = bufsize - totlen;
        tempBuf = QSharedPointer<char>(&buf[totlen], UhlRegister::doNotDelSharedPtr); //trick to access part of buf using a shared pointer. the shared pointer is not allowed to delete the char-array, therefore the Deleter-method.
        retval += m_pSer->getVal(tempBuf, len);
        totlen += *len;
        Sleep(2);
    }
    while ((totlen > 0) && (buf[totlen - 1] != endline[0]) && (totlen < bufsize));

    if (totlen > 0)
    {
        buf[totlen - 1] = 0;
    }
    else
    {
        buf[0] = 0;
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlReadRegL(unsigned char reg, long *plval)
{
    char buf[50];
    ito::RetVal ret = ito::retOk;

    ret = UhlWriteReg(reg);
    if (ret == ito::retError)
    {
        return ret;
    }

    Sleep(3*UHLDELAY);
    ret = UhlReadString(buf, sizeof(buf));

    if (ret == ito::retError)
    {
        return ret;
    }

    *plval = atol(buf);
    return ito::retOk;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlWriteReg(unsigned char reg)
{
    char buf[2];

    buf[0] = 'U';
    buf[1] = reg;
    return m_pSer->setVal(buf, 2);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlWriteRegB(unsigned char reg, unsigned char ch)
{
    char buf[3];

    buf[0] = 'U';
    buf[1] = reg;
    buf[2] = ch;
    return m_pSer->setVal(buf, 3);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlWriteRegL(unsigned char reg, long lval)
{
    char buf[20];

    int len = sprintf(buf, "U%c%ld", reg, lval);
    return m_pSer->setVal(buf, len);
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlStatus()
{
    char buf[20];
    ito::RetVal ret = ito::retOk;

    ret = UhlWriteReg(STATUS + READ);
    if (ret == ito::retError)
    {
        return ret;
    }

    Sleep(UHLDELAY);
    ret = UhlReadString(buf, sizeof(buf));
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlJoystickOn()
{
    ito::RetVal ret = ito::retOk;

    if (m_joyEnabled)
    {
        ret += UhlWriteRegB(COMMAND, JOYON);
        Sleep(UHLDELAY);
        ret += DummyRead();
        ret += UhlWriteReg(START);
        Sleep(UHLDELAY);
        ret += DummyRead();
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlJoystickOff()
{
    ito::RetVal ret = ito::retOk;

    if (m_joyEnabled)
    {
        char buf[1];

        // not command 'j' ("U\007j"), but only 'j'
        buf[0] = 'j';
        ret += m_pSer->setVal(buf, 1);

        if (ret.containsError())
        {
            return ret;
        }
        Sleep(UHLDELAY);     // it's necessary!!!
        ret += DummyRead();
    }

    if (!ret.containsError())
    {
        ret += UhlAxisSync();
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    char buf[50];
    unsigned long bufcnt = 0;
    int axiscnt = 0;
    long pos = 0;
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    QElapsedTimer timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    long delay = UHLDELAY; //[ms]
    QSharedPointer<double> actPos = QSharedPointer<double>(new double);
//    int status = 0;

    timer.start();

    QVector<int> _axis = axis;
    if (_axis.size() == 0) //all axis
    {
        for (int i = 0; i < m_numAxis; i++)
        {
            _axis.append(i);
        }
    }

    //set all moving axis to status "moving", any ref or end switch flag is reset
    setStatus(_axis, ito::actuatorMoving, ito::actStatusMask);

    while (!done && !timeout)
    {
        memset(buf, 0, 50);
        retVal += UhlReadString(buf, 6);
        if (strlen(buf) != 0)
        {
            bufcnt = (unsigned long)strlen(buf);
            if (bufcnt == 5) // Okay, five characters + 0
            {
                for (axiscnt = 0; axiscnt < _axis.size(); axiscnt++)
                {
                    // Check if axis has moved and is where it should be (@)
                    if (buf[axiscnt] == 0x40 /*@-character*/)
                    {
                        replaceStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorMoving, ito::actuatorAtTarget);
                    }
                    else // Okay wrong character (not @). Axis movement failed for this axis.
                    {
                        (void)UhlReadRegL(XABSPOS + READ + _axis.value(axiscnt), &pos);
                        Sleep(UHLDELAY);
                        (void)UhlWriteRegL(XPOS + axis.value(axiscnt), pos);

                        if (buf[axiscnt] == 0x41) // A -> axis is end switch at null position
                        {
                            retVal += ito::RetVal(ito::retError, STATUS_UPPER_REF_SWITCH, tr("E-Strike hit null position").toLatin1().data());
                            setStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorAtTarget | ito::actuatorRefSwitch | ito::actuatorRightEndSwitch | ito::actuatorEndSwitch | ito::actuatorRightRefSwitch, ito::actStatusMask);
                        }
                        else if (buf[axiscnt] == 0x44) // D -> axis is end switch at end position
                        {
                            retVal += ito::RetVal(ito::retError, STATUS_LOWER_REF_SWITCH, tr("E-Strike hit end position").toLatin1().data());
                            setStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorAtTarget | ito::actuatorRefSwitch | ito::actuatorLeftEndSwitch | ito::actuatorEndSwitch | ito::actuatorLeftRefSwitch, ito::actStatusMask);
                        }
                        else
                        {
                            retVal += ito::RetVal(ito::retError, 0, tr("Undefined answer in serial port buffer").toLatin1().data());
                            setStatus(_axis, ito::actuatorAtTarget, ito::actStatusMask);
                            return retVal;
                        }
                    }
                }

                done = true;
            }
            else
            {
                retVal += ito::RetVal(ito::retError, 0, tr("Undefined answer in serial port buffer").toLatin1().data());
                setStatus(_axis, ito::actuatorAtTarget, ito::actStatusMask);
                return retVal;
            }
        }

        sendStatusUpdate(true);

        if (!done && isInterrupted())
        {
            buf[0] = 'a';
            retVal += m_pSer->setVal(buf, 1);
            Sleep(UHLDELAY);
            retVal += DummyRead();

            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;
            return retVal;
        }

        if (!done)
        {
            QCoreApplication::processEvents();

            //short delay
            waitMutex.lock();
            waitCondition.wait(&waitMutex, delay);
            waitMutex.unlock();
            setAlive();
        }

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS)
            {
                timeout = true;
            }
        }
    }

    foreach(const int &i, _axis)
    {
        retVal += getPos(i, actPos, NULL);
    }
    sendStatusUpdate(false);

    if (timeout)
    {
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError, 9999, tr("timeout occurred").toLatin1().data());
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void UhlRegister::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *dock = getDockWidget()->widget();
        if (visible)
        {
            QObject::connect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),dock, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::connect(this, SIGNAL(targetChanged(QVector<double>)), dock, SLOT(targetChanged(QVector<double>)));
            QObject::connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dock, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            requestStatusAndPosition(true, true);
            emit parametersChanged(m_params);
        }
        else
        {
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>,QVector<double>)),dock, SLOT(actuatorStatusChanged(QVector<int>,QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), dock, SLOT(targetChanged(QVector<double>)));
            QObject::disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dock, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlCheckAxisNumber(const int axis)
{
    if (axis < m_numAxis && axis > -1)
    {
        return ito::retOk;
    }
    else
    {
        return ito::retError;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlCheckAxisNumber(QVector<int> axis)
{
    long axiscnt = 0;

    for (axiscnt = 0; axiscnt < axis.size(); axiscnt++)
    {
        if (axis[axiscnt] >= m_numAxis && axis[axiscnt] < 0)
        {
            return ito::retError;
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
UhlRegister::UhlRegister() : AddInActuator(), m_spitchx(0), m_resolution(0), m_pSer(NULL), m_numAxis(0),
    m_scale(0), m_stepperspeed(50), m_accel(50), m_async(0), m_posrequestlisteners(0)
{
//    qRegisterMetaType< QMap<QString,ito::tParam> >("QMap<QString,ito::tParam>");

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "UhlRegister", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("async", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("accel", ito::ParamBase::Double, 0.0, 99.0, 0.0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("speed", ito::ParamBase::Double, 0.0, 90.0, 0.0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inversex", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inversey", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inversez", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 3, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("joyenabled", ito::ParamBase::Int, 0, 1, 1, tr("Enabled/disabled Joystick. Default: enabled").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::In, 0.0, std::numeric_limits<double>::max(), 20.0, tr("timeout for axes movements in seconds").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetUhl *UhlWid = new DockWidgetUhl(getID(), this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, UhlWid);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
UhlRegister::~UhlRegister()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further information
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal UhlRegister::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
    }
    else if (key == "")
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

    \param [in] val  is a input of type::tparam containing name, value and further information
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal UhlRegister::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

//    long stepperspeed = 1000;
//    long ratio = 8;

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

            if (key == "speed")
            {
                retValue += UhlJoystickOff();
                // Value in [mm/s] * mscale = Steps / s -> Steps / s * SPITCH = Turns /s and *10 due to fucking uhl-manuel
                m_stepperspeed = (long)(m_params["speed"].getVal<double>() * m_scale / SPITCH);
                retValue += UhlWriteRegL(REVSPEED, m_stepperspeed);
                if (!retValue.containsError())
                {
                    long speed = 0;
                    retValue += UhlReadRegL(REVSPEED + READ, &speed);
                    if (!retValue.containsError())
                    {
                        m_stepperspeed = speed;
                        m_params.find("speed").value().setVal<double>((double)(m_stepperspeed));
                    }
                }
                retValue += UhlJoystickOn();
            }
            else if (key == "async")
            {
                m_async = m_params["async"].getVal<int>();
            }
            else if (key == "accel")
            {
                retValue += UhlJoystickOff();
                m_accel = m_params["accel"].getVal<int>();
                retValue += UhlWriteRegL(RAMP, m_accel);
                if (!retValue.containsError())
                {
                    long accel = 0;
                    retValue += UhlReadRegL(RAMP + READ, &accel);
                    if (!retValue.containsError())
                    {
                        m_accel = accel;
                        m_params["accel"].setVal(m_accel);
                    }
                }
                retValue += UhlJoystickOn();
            }
            else if (key == "inversex")
            {
                m_inverse[0] = GetAxisDir(m_params["inversex"].getVal<int>());
            }
            else if (key == "inversey")
            {
                m_inverse[1] = GetAxisDir(m_params["inversey"].getVal<int>());
            }
            else if (key == "inversez")
            {
                m_inverse[2] = GetAxisDir(m_params["inversez"].getVal<int>());
            }
            else if (key == "joyenabled")
            {
                retValue += UhlJoystickOff();
                m_joyEnabled = m_params["joyenabled"].getVal<int>();
                retValue += UhlJoystickOn();
            }
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
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
ito::RetVal UhlRegister::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    char buf[20];
    int n = 0, axis = 0, axiscnt = 0;
    double stepperspeed_mm = 0.0;

    m_stepperspeed = 60;
    m_accel = 50;
    m_async = 0;
    m_scale = 1e4;
    m_joyEnabled = 1;  // It's necessary to switching off Joystick!

    for (n = 0; n < 3; n++)
    {
        m_inverse[n] = GetAxisDir((*paramsOpt)[n + 1].getVal<int>());
        m_params.find((*paramsOpt)[n + 1].getName()).value().setVal<double>((double)(*paramsOpt)[n + 1].getVal<int>());
    }

    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 9600)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 2)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 12)), NULL);
        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r")), NULL);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
        goto end;
    }

    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT").toLatin1().data());
        goto end;
    }
    else
    {
        QSharedPointer<ito::Param> paramSerial(new ito::Param("port"));
        retval += m_pSer->getParam(paramSerial, NULL);
        if (retval.containsError() || paramSerial->getVal<int>() < 1)
        {
            retval.appendRetMessage(tr(" during port number read out").toLatin1().data());
            m_identifier = QString("Failed(%1)").arg(getID());
            return retval;
        }
        else
        {
            m_params["comPort"].setVal<int>(paramSerial->getVal<int>());
            m_identifier = QString("Uhl@Com%1").arg(m_params["comPort"].getVal<int>());
        }
    }

    // clean the world - otherwise the Suuper Uhl will fuck you ;-)
    retval += UhlWriteReg(COMMAND);
    if (retval.containsError())
    {
        goto end;
    }

    setAlive();

    Sleep(UHLDELAY * 2);
    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY * 2);
    retval += UhlWriteReg(START);
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY * 2);
    memset(&buf, 0, 20);
    retval += UhlReadString(buf, 6);
    if (retval.containsError())
    {
        goto end;
    }

    if (!strncmp(buf, "ERR 1", 5))
    {
        // if this table is an UhlText controller this command set the Uhl interpreter to text command
        int len = sprintf(buf, "U%cmb", COMMAND);
        retval += m_pSer->setVal(buf, len);
        if (retval.containsError())
        {
            goto end;
        }

        retval += DummyRead();
        if (retval.containsError())
        {
            goto end;
        }

        // check if this table is an UhlText controller
        sprintf(buf, "?status");
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval.containsError())
        {
            goto end;
        }

        Sleep(UHLDELAY * 2);
        memset(&buf, 0, 20);
        retval += UhlReadString(buf, 6);
        if (retval.containsError())
        {
            goto end;
        }

        if (strncmp(buf, "ERR", 3))
        {
            retval += ito::RetVal(ito::retError, 0, tr("Wrong command! You should try to run this UHL table by using the UhlText driver!").toLatin1().data());
            goto end;
        }

        retval += UhlJoystickOff();
        if (retval.containsError())
        {
            goto end;
        }

        retval += UhlJoystickOn();
        if (retval.containsError())
        {
            goto end;
        }

        Sleep(UHLDELAY * 2);
        retval += UhlWriteReg(START);
        if (retval.containsError())
        {
            goto end;
        }

        Sleep(UHLDELAY * 2);
        memset(&buf, 0, 20);
        retval += UhlReadString(buf, 6);
        if (retval.containsError())
        {
            goto end;
        }

        if (!strncmp(buf, "ERR 1", 5))
        {
            retval += ito::RetVal(ito::retError, 0, tr("Wrong command! You should try to run this UHL table by using the UhlText driver!").toLatin1().data());
            goto end;
        }
    }

    if (strlen(buf) == 0)  // Sometimes START command don't get an answer. In that case UHL table will answer after sending 'j'!
    {
        buf[0] = 'j';
        retval += m_pSer->setVal(buf, 1);
        Sleep(UHLDELAY * 2);
        memset(&buf, 0, 20);
        retval += UhlReadString(buf, 6);
        if (retval.containsError())
        {
            goto end;
        }
        m_joyEnabled = 0;  // Disable Joystick temporary!
    }

    // count axes
    if (strlen(buf) == 0)
    {
        retval += ito::RetVal(ito::retError, 0, tr("No answer from serial port. Initialisation canceld.\nIf this is really an UHL table, you should try an other joystick!").toLatin1().data());
        goto end;
    }

    axis = 0;
    for (n = 0 ;n < (int)strlen(buf); n++)
    {
        if ((buf[n] == 0x40) || (buf[n] == 'D') || (buf[n] == 'A'))
        {
            axis++;
        }
        else if ((buf[n] == '-') || (buf[n] == '.')|| (buf[n] == '\r'))
        {
            continue;
        }
        else
        {
            axis = -1;
        }
    }

    if (axis < 2)
    {
        retval += ito::RetVal(ito::retError, 0, tr("Undefined answer from serial port. Initialisation canceld").toLatin1().data());
        goto end;
    }

    if (axis == 2)
    {
        m_numAxis = 2;
        m_spitchx = 13;
        m_resolution = 15;
        //m_params.find("speed").value().setMax((double)(120.0));
        static_cast<ito::DoubleMeta*>(m_params["speed"].getMeta())->setMax(120.0);
    }
    else
    {
        m_numAxis = 3;
        m_spitchx = 21;
        m_resolution = 25;
    }

    m_currentStatus.fill(0, m_numAxis);
    m_currentPos.fill(0, m_numAxis);
    m_targetPos.fill(0, m_numAxis);
    for (n = 0; n < m_numAxis; n++)
    {
        m_currentStatus[n] = ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
    }

    m_params.find("numaxis").value().setVal<double>((double)(axis));

    if (m_joyEnabled)
    {
        retval += UhlJoystickOff();
        m_joyEnabled = 0;  // Disable Joystick temporary!
    }
    else
    {
        retval += UhlAxisSync();
    }
    if (retval.containsError())
    {
        goto end;
    }

    retval += UhlWriteRegL(SDELAY, 0);
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY);
    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    retval += UhlWriteRegL(m_resolution, 1);
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY);
    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    // really useful???
    retval += UhlWriteRegB(COMMAND, GORELEXTCLK);
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY);
    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    // set speed
    retval += UhlWriteRegL(REVSPEED, m_stepperspeed);
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY);
    if (m_stepperspeed == 0)
    {
        stepperspeed_mm = 0.01;
    }
    else
    {
        stepperspeed_mm = m_stepperspeed;
    }
    m_params.find("speed").value().setVal<double>((double)(stepperspeed_mm / 10 * SPITCH / m_scale));

    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    // set ramp
    retval += UhlWriteRegL(RAMP, m_accel);
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY);
    m_params.find("accel").value().setVal<double>((double)(m_accel));
    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    for (axiscnt = 0; axiscnt < m_numAxis; axiscnt++)
    {
        retval += UhlWriteRegL(m_spitchx + axiscnt, SPITCH);
        Sleep(UHLDELAY);
        retval += DummyRead();
        Sleep(UHLDELAY);
    }

    if (retval.containsError())
    {
        goto end;
    }

    retval += UhlWriteRegL(MASK, 7);
    if (retval.containsError())
    {
        goto end;
    }

    Sleep(UHLDELAY);
    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    m_joyEnabled = (*paramsOpt)[4].getVal<int>();
    m_params.find((*paramsOpt)[4].getName()).value().setVal<double>((double) m_joyEnabled);
    retval += UhlJoystickOn();
    if (retval.containsError())
    {
        goto end;
    }

    if ((*paramsOpt)[0].getVal<int>() == 1)
    {
        QVector<int> axisvektor;

        for (n = 0; n < m_numAxis; n++)
        {
            axisvektor.append(n);
        }
        calib(axisvektor, 0);
    }

end:
    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    emit parametersChanged(m_params);
    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::close(ItomSharedSemaphore *waitCond)
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
ito::RetVal UhlRegister::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    retval += calib(QVector<int>(1, axis), 0);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::calib(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    long positions[4];
    int axis_cnt = 0;
    long mask_calib = 0;
    long mask_revcalib = 0;
    long mask_old = 0;
    ito::RetVal retval = ito::retOk;

//    emit PositioningStatusChanged(1);

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());

        if (waitCond)
        {
            waitCond->release();
            waitCond->returnValue = retval;
        }
    }
    else
    {
        retval += UhlCheckAxisNumber(axis);

        if (retval != ito::retError)
        {
            retval += UhlJoystickOff();
        }

        if (retval != ito::retError)
        {
            // Workaround for Bug in der F9S-3-O: At Reset of one axis
            // some Controller forget the positions of the other axis

            for (axis_cnt = 0; axis_cnt < m_numAxis; axis_cnt++)
            {
                retval += UhlReadRegL(XABSPOS + axis_cnt + READ, &positions[axis_cnt]);
            }

            if (retval != ito::retError)
            {
                // Get axis mask
                (void)UhlReadRegL(75, &mask_old);

                // Masked all axis expect the active one (bit-mask)
                for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
                {
                    mask_calib += (m_inverse[axis.value(axis_cnt)]>0?1:0)<< axis.value(axis_cnt);
                    mask_revcalib += (m_inverse[axis.value(axis_cnt)]<0?1:0)<< axis.value(axis_cnt);
                }

                // Write mask for calibration command
                if (mask_calib) // because mask_calib=0 sometimes behaves like all axis on
                {
                    retval  += UhlWriteRegL(MASK, mask_calib);
                    if (retval != ito::retError)
                    {
                        // Write calibration command
                        retval += UhlWriteRegB(COMMAND, CALIBRATE);
        //                GOEND
                    }

                    retval  += DummyRead();
                    if (retval != ito::retError)
                    {
                        retval += UhlWriteReg(START);
                        retval += waitForDone(60000, axis);
                        if (retval.containsError())
                        {
                            if ((retval.errorCode() == STATUS_UPPER_REF_SWITCH) || (retval.errorCode() == STATUS_LOWER_REF_SWITCH))
                            {
                                retval = ito::retOk;
                            }
                        }
/*                        else
                        {
                            retval += ito::RetVal(ito::retError, 0, tr("error: no reference switch reached").toLatin1().data());
                        }*/
                    }
                    if (retval != ito::retError)
                    {
                        retval = UhlStatus();
                    }
                }
                // Write mask for reverse calibration command
                if (mask_revcalib) // because mask_calib=0 sometimes behaves like all axis on
                {
                    retval  += UhlWriteRegL(MASK, mask_revcalib);
                    if (retval != ito::retError)
                    {
                        // Write reverse calibration command
                        retval += UhlWriteRegB(COMMAND, GOEND);
        //                GOEND
                    }

                    retval  += DummyRead();
                    if (retval != ito::retError)
                    {
                        retval += UhlWriteReg(START);
                        retval += waitForDone(60000, axis);
                        if (retval.containsError())
                        {
                            if ((retval.errorCode() == STATUS_UPPER_REF_SWITCH) || (retval.errorCode() == STATUS_LOWER_REF_SWITCH))
                            {
                                retval = ito::retOk;
                            }
                        }
/*                        else
                        {
                            retval += ito::RetVal(ito::retError, 0, tr("error: no reference switch reached").toLatin1().data());
                        }*/
                    }
                    if (retval != ito::retError)
                    {
                        retval = UhlStatus();
                    }
                }
                // Write back absolute positions of other axis
                for (axis_cnt = 0; axis_cnt < m_numAxis; axis_cnt++)
                {
                    (void)UhlWriteRegL(XABSPOS + axis_cnt, positions[axis_cnt]);
                }

                // Set absolute position und target register of calibrated axis to zero
                Sleep(UHLDELAY);
                for (axis_cnt = 0; axis_cnt < m_numAxis; axis_cnt++)
                {
                    (void)UhlWriteRegL(XABSPOS + axis_cnt, 0);
                    (void)UhlWriteRegL(XPOS + axis_cnt, 0);
                    m_currentPos[axis_cnt] = 0.0;
                }

                // Write back old mask
                (void)UhlWriteRegL(MASK, mask_old);
            }
        }

        (void)UhlJoystickOn();

        if (waitCond)
        {
            waitCond->returnValue = retval;
            waitCond->release();
        }

        if (m_posrequestlisteners)
        {
            QSharedPointer<QVector<double> > sharedpos = QSharedPointer<QVector<double> >(new QVector<double>);

            for (int i = 0; i < axis.size(); i++)
            {
                *sharedpos << 0;
            }

            retval += getPos(axis, sharedpos, 0);
//            emit SentPositionChanged(axis, *sharedpos);
        }
    }
    sendStatusUpdate(false);
//    emit PositioningStatusChanged(0);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    retval += UhlJoystickOff();
    Sleep(UHLDELAY);
    retval = UhlCheckAxisNumber(axis);
    Sleep(UHLDELAY);
    retval = DummyRead();

    if (retval != ito::retError)
    {
        if (axis < m_numAxis)
        {
            retval += UhlWriteRegL(XABSPOS + axis, 0);
            retval += UhlWriteRegL(XPOS + axis, 0);
            if (retval != ito::retError)
            {
                 m_currentPos[axis] = 0.0;
            }
            Sleep(UHLDELAY);
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("Axis not exist").toLatin1().data());
        }
    }
    retval += UhlJoystickOn();
    sendStatusUpdate(false);

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    int axis_cnt = 0;

    for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
    {
        retval += setOrigin(axis.value(axis_cnt), 0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);
//    ito::RetVal retval = UhlStatus();
    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    long retPos = 0;

    retval = UhlCheckAxisNumber(axis);
    retval = DummyRead();

    if (axis < m_numAxis)
    {
        retval += UhlReadRegL(XABSPOS + READ + axis, &retPos);
        if (retval != ito::retError)
        {
             *pos = (double)retPos / m_scale * m_inverse[axis];
             m_currentPos[axis] = *pos;
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis not exist").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QSharedPointer<double> dpos(new double);
    *dpos = 0.0;
    int axis_cnt = 0;

    for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
    {
        retval += getPos(axis.value(axis_cnt), dpos, 0);
        (*pos)[axis_cnt] = *dpos; // \TODO what about m_scale (like getPos above)
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlRegister::UhlSetPos(QVector<int> axis, QVector<double> pos, const unsigned char absrelflag, ItomSharedSemaphore *waitCond)
{
    int axis_cnt = 0;
    long lpos = 0;
    double dpos = 0.0;
    ito::RetVal retval = ito::retOk;
    long mask_move = 0;
    bool released = false;
    int timeoutMS = m_params["timeout"].getVal<ito::float64>() * 1000;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());

        if (waitCond)
        {
            waitCond->release();
            waitCond->returnValue = retval;
            released = true;
        }
    }
    else
    {
        retval += UhlCheckAxisNumber(axis);
        retval += DummyRead();
        retval += UhlJoystickOff();
        retval += DummyRead();
        retval += UhlWriteRegB(COMMAND, absrelflag);
        Sleep(UHLDELAY);

        for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
        {
            //update target position
            if (absrelflag == GOABSINTCLK) //move absolute
            {
                m_targetPos[ axis[axis_cnt] ] = pos.value(axis_cnt);
            }
            else //move relative
            {
                m_targetPos[ axis[axis_cnt] ] = m_currentPos[ axis[axis_cnt]] + pos.value(axis_cnt);
            }

            mask_move += 1<< axis.value(axis_cnt);
            dpos = pos.value(axis_cnt) * m_scale;
            if (dpos < 0)
                lpos=((long)(dpos - .5));
            else
                lpos=((long)(dpos + .5));
            retval += UhlWriteRegL(XPOS + axis.value(axis_cnt), lpos * m_inverse[axis.value(axis_cnt)]);
            Sleep(UHLDELAY);
        }
        sendTargetUpdate();

        retval  += UhlWriteRegL(MASK, mask_move);

        retval += UhlWriteReg(START);

        if (m_async && waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
        retval += waitForDone(timeoutMS, axis);

        if (!m_async && waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
        if (retval != ito::retError)
        {
            retval += UhlJoystickOn();
        }

        if (retval != ito::retError)
        {
            retval += UhlWriteRegL(MASK, 7);
        }

        if (retval != ito::retError)
        {
            Sleep(UHLDELAY);
            retval += DummyRead();
        }

        if (retval != ito::retError)
        {
            retval += UhlWriteReg(START);
        }

        if (retval != ito::retError)
        {
            Sleep(UHLDELAY);
            retval += DummyRead();
        }

        if (waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    retval = UhlSetPos(QVector<int>(1, axis), QVector<double>(1, pos), GOABSINTCLK, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    retval = UhlSetPos(axis, pos, GOABSINTCLK, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    retval = UhlSetPos(QVector<int>(1, axis), QVector<double>(1, pos), GORELINTCLK, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    retval = UhlSetPos(axis, pos, GORELINTCLK, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlRegister::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retval(ito::retOk);
    int i;
    QVector<int> axis;
    QSharedPointer<QVector<double> > sharedpos = QSharedPointer<QVector<double> >(new QVector<double>);

    for (i = 0; i < m_numAxis; i++)
    {
        axis << i;
        *sharedpos << 0;
    }

    retval += UhlStatus();
    retval += UhlStatus();
    if (sendCurrentPos)
    {
        retval += getPos(axis, sharedpos, 0);
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
