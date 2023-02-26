#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "UhlText.h"
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

#include <qnumeric.h>

enum uhlCmd {
    GOREL        =   0,
    GOABS        =   1
};

enum uhlStatus {
    STATUS_UPPER_REF_SWITCH = 0x0040, //bit 7: upper reference switch is reached
    STATUS_LOWER_REF_SWITCH = 0x0080 //bit 8: lower reference switch is reached
};

#define UHLDELAY 20

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
ito::RetVal UhlTextInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(UhlText)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlTextInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(UhlText)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
UhlTextInterface::UhlTextInterface()
{
    m_type = ito::typeActuator;
    setObjectName("UhlText");

    m_description = tr("DLL for 2-4 axis Uhl / Lang LStep-Controller");
    m_detaildescription = tr("The UhlText is a plugin, which can be used to control the 2-4 axis\n\
stepper motor devices from Uhl (F9S-x) and Lang LSTEP 2x\n\
It is initialized by actuator(\"UhlText\", SerialIO, ...).\n\
\n\
WARNING: There are different controller versions with different\n\
command languages. This DLL is for devices that are controlled by ASCII commands via the RS232 port.\n\
WARNING: The calibration direction of the stages differs according to motor / controller.\n\
Check calibration direction before usage. \n\
\n\
This plugin was published with the kind permission of company Walter Uhl, technische Mikroskopie GmbH & Co. KG.");
    m_author = "W. Lyda, H. Bieger, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("LGPL");
    m_aboutThis = tr(GITVERSION);

//    m_initParamsMand;
    ito::Param paramVal("serial", ito::ParamBase::HWRef, NULL, tr("An initialized SerialIO").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("calibration", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("1 -> calibration during initilization").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("inversex", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("Invert axis direction for x").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("inversey", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("Invert axis direction for y").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("inversez", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("Invert axis direction for z").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("inversea", ito::ParamBase::Int, 0, new ito::IntMeta(0,1), tr("Invert axis direction for a").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("joyenabled", ito::ParamBase::Int, 1, new ito::IntMeta(0,1), tr("Enabled/disabled Joystick. Default: enabled").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("targetBaud", ito::ParamBase::Int, 9600, new ito::IntMeta(9600,115200), tr("New baud rate for uhl table").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
UhlTextInterface::~UhlTextInterface()
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
const ito::RetVal UhlText::showConfDialog(void)
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

//----------------------------------------------------------------------------------------------------------------------------------
/*const ito::RetVal UhlText::showConfDialog(void)
{

    dialogUhl *confDialog = new dialogUhl(qobject_cast<ito::AddInActuator*>(this), m_numAxis);
    confDialog->setVals(&m_params);
    if (confDialog->exec())
    {
        confDialog->getVals(&m_params);
    }
    delete confDialog;
    return ito::retOk;
}*/

//-------------------------------------------------------------------------------------------------------------------------------------------------
// read buffer without delay
const ito::RetVal UhlText::DummyRead()
{
    ito::RetVal retval(ito::retOk);

    //char buf[100] = {0};
    QSharedPointer<int> bufsize(new int);
    *bufsize = 100;
    QSharedPointer<char> buf(new char[100]);
    memset((void*)buf.data(), 0, 100);
    char* bufData = buf.data();
    //int bufsize = 100;
    int i;

    m_pSer->getVal(buf, bufsize);
    if (*bufsize > 0)
    {
        if (!strncmp(bufData, "ERR", 3))
        {
            QString ErrMsg = tr("UHL errormessage: ");
            QString CodeCode(bufData[4]);
            if (strlen(bufData) == 7)
            {
                CodeCode.append(bufData[5]);
            }

            QString ErrCode(bufData[4]);
            switch (ErrCode.toInt())
            {
                case 1:
                {
                    ErrMsg += tr("No valid axis name!");
                    break;
                }
                case 2:
                {
                    ErrMsg += tr("No executable function!");
                    break;
                }
                case 3:
                {
                    ErrMsg += tr("Too many signs in the command string!");
                    break;
                }
                case 4:
                {
                    ErrMsg += tr("No valid command!");
                    break;
                }
                case 5:
                {
                    ErrMsg += tr("Out of valid number range!");
                    break;
                }
                case 6:
                {
                    ErrMsg += tr("Wrong number of parameters!");
                    break;
                }
                case 7:
                {
                    ErrMsg += tr("Missing '!' or '?'!");
                    break;
                }
                case 8:
                {
                    ErrMsg += tr("No trigger mode possible since this axis is active!");
                    break;
                }
                case 9:
                {
                    ErrMsg += tr("Axis cannot be en- or disabled since trigger mode is active!");
                    break;
                }
                case 10:
                {
                    ErrMsg += tr("Function is not configured!");
                    break;
                }
                case 11:
                {
                    ErrMsg += tr("No move-command allowed since joystick is enabled!");
                    break;
                }
                case 12:
                {
                    ErrMsg += tr("End-switch triggered!");
                    break;
                }
                default:
                {
                    ErrMsg += tr("Undefined answer from serial port!");
                }
            }
            retval = ito::RetVal(ito::retWarning, 0, ErrMsg.toLatin1().data());
//            retval = ito::RetVal(ito::retWarning, 0, tr("Errormessage in Serial-Buffer code %1").arg(CodeMsg).toLatin1().data());
        }
        for (i=0; i<*bufsize; i++)
        {
            if (bufData[i] == 'S')
            {
                retval = ito::RetVal(ito::retWarning, 0, tr("'S' detected in answer from device (end-switch reached by any axis???)").toLatin1().data());
            }
            if (bufData[i] == 'J')
            {
                retval = ito::RetVal(ito::retWarning, 0, tr("'J' detected in answer from device (joystick mode of any axis)").toLatin1().data());
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlText::UhlReadString(char *buf, const int bufsize, const int tries, int *signcnt)
{
    ito::RetVal retval = ito::retOk;
    QSharedPointer<int> len(new int);
    *len = 0;
    QSharedPointer<char> tempBuf;
    int totlen = 0;
    static char endline[3] = { 0, 0, 0 };
    int tried =0;

    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retval += m_pSer->getParam(param, NULL);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>();
        endline[0] = temp[0];
        endline[1] = temp[1];
        endline[2] = temp[2];
    }

    totlen = 0;
    do
    {
        Sleep(UHLDELAY);
        do
        {
            *len = bufsize - totlen;
            tempBuf = QSharedPointer<char>(&buf[totlen], UhlText::doNotDelSharedPtr); //trick to access part of buf using a shared pointer. the shared pointer is not allowed to delete the char-array, therefore the Deleter-method.
            retval += m_pSer->getVal(tempBuf, len);
            totlen += *len;
            Sleep(2);
        }
        while ((totlen > 0) && (buf[totlen - 1] != endline[0]) && (totlen < bufsize));
    }
    while ((totlen == 0) && (++tried < tries));

    if (totlen > 0)
    {
        buf[totlen - 1] = 0;
    }
    else
    {
        buf[0] = 0;
    }

    *signcnt = totlen;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlText::UhlReadFloat(float *value)
{
    ito::RetVal retval = ito::retOk;
    char buf[50];
    int bufsize = 50;
    int count;

    retval += UhlReadString(buf, bufsize, 10, &count);
    if (!count)
    {
        retval += ito::RetVal(ito::retError, 9999, tr("No signs read after 10 tries").toLatin1().data());
    }

    *value = atof(buf);
    if (qIsNaN(*value) || qIsInf(*value))
    {
        retval += ito::RetVal(ito::retError, 666, tr("Tried to read float-value but got a string-value").toLatin1().data());
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlText::UhlReadLong(long *value)
{
    ito::RetVal retval = ito::retOk;
    char buf[50];
    int bufsize = 50;
    int count;

    retval += UhlReadString(buf, bufsize, 10, &count);
    if (!count)
    {
        retval += ito::RetVal(ito::retError, 9999, tr("No signs read after 10 tries").toLatin1().data());
    }
    *value = atol(buf);

    float temp = atof(buf);
    if (qIsNaN(temp) || qIsInf(temp))
    {
        retval += ito::RetVal(ito::retError, 666, tr("Tried to read long-value but got a string-value").toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlText::UhlStatus()
{
    char buf[20];
    ito::RetVal ret = ito::retOk;
    int count;

    memset(buf, 0, 20);
    sprintf(buf, "?status");
    ret += m_pSer->setVal(buf, (int)strlen(buf));
    if (ret == ito::retError)
    {
        return ret;
    }
    memset(buf, 0, 20);
    ret += UhlReadString(buf, sizeof(buf), 2, &count);
    if (!count)
    {
        ret += ito::RetVal(ito::retError, 0, tr("No answer during status request").toLatin1().data());
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlText::UhlJoystickOn()
{
    ito::RetVal ret = ito::retOk;

    if (m_joyEnabled)
    {
        char buf[20];

        memset(buf, 0, 20);
        sprintf(buf, "!joy 4");
        ret += m_pSer->setVal(buf, (int)strlen(buf));

        if (!ret.containsError())
        {
            Sleep(80);
            ret += DummyRead();

            if (ret.containsError())
            {
                ret.appendRetMessage(tr(" in UhlJoystickOn!").toLatin1().data());
            }
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal UhlText::UhlJoystickOff(bool waitforanswer)
{
    ito::RetVal ret = ito::retOk;

    if (m_joyEnabled)
    {
        char buf[20];
        int bytes = 0;

        memset(buf, 0, 20);
        sprintf(buf, "!joy 0");
        ret += m_pSer->setVal(buf, (int)strlen(buf));

        if ((!ret.containsError()) && waitforanswer)
        {
            memset(buf, 0, 20*sizeof(char));
            ret += this->UhlReadString(buf, 6, 30, &bytes);

            if (!ret.containsError() && bytes == 0)
            {
                //if you set !joy 0 twice, no answer is returned. Therefore check now, if the current
                //status is really 0.
                memset(buf, 0, 20);
                sprintf(buf, "?joy");
                ret += m_pSer->setVal(buf, (int)strlen(buf));

                if (!ret.containsError())
                {
                    memset(buf, 0, 20*sizeof(char));
                    ret += UhlReadString(buf, 6, 30, &bytes);

                    if (!ret.containsError())
                    {
                        if (bytes == 0 || buf[0] != '0' /*char 48*/)
                        {
                            ret += ito::RetVal(ito::retError, 0, tr("Error while switching joystick off").toLatin1().data());
                        }
                    }
                }
            }
        }

        if (ret.containsError())
        {
            ret.appendRetMessage(tr(" in UhlJoystickOff!").toLatin1().data());
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    char buf[100];
    unsigned long axiscnt = 0;
    unsigned long bufcnt = 0;
//    unsigned long chkcycles = 0;
    int cnts = 0;
    char status;
    float physpos;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    long delay = UHLDELAY; //[ms]
    bool done = false;
    bool timeout = false;
    QElapsedTimer timer;
    ito::RetVal retVal = ito::retOk;
    QSharedPointer<double> actPos = QSharedPointer<double>(new double);

    timer.start();

    QVector<int> _axis = axis;
    if (_axis.size() == 0) //all axis
    {
        for (int i=0;i<m_numAxis;i++)
        {
            _axis.append(i);
        }
    }

    //set all moving axis to status "moving", any ref or end switch flag is resetted
    setStatus(_axis, ito::actuatorMoving, ito::actStatusMask);

    while (!done && !timeout)
    {
        memset(buf, 0, 50);
        retVal += UhlReadString(buf, 6, 1, &cnts);
        if (cnts != 0)
        {
            bufcnt = strlen(buf);
            for (axiscnt = 0; axiscnt < (unsigned long)_axis.size(); axiscnt++)
            {
                // Check if axis has moved and is where it should be (@) or E-Strike (S) or Joystick on (J) or Error (else)
                status = buf[axiscnt];
                switch (status)
                {
                    case 'D':
                    case 'A':
                    case '@':
                    {
                        if (bufcnt < 5)
                        {
                            retVal += ito::RetVal(ito::retError, 0, tr("Undefined answer in serial port buffer, not enough characters").toLatin1().data());
                            setStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorAtTarget | ito::actuatorRefSwitch | ito::actuatorLeftEndSwitch | ito::actuatorEndSwitch | ito::actuatorLeftRefSwitch, ito::actStatusMask);
                        }
                        else
                        {
                            replaceStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorMoving, ito::actuatorAtTarget);
                        }
                        break;
                    }

                    case 'S':
                    {
                        //Eventuell Zielregister neu setzen
                        memset(buf, 0, 100);
                        sprintf(buf, "?pos %c", m_nameAxis[_axis[axiscnt]]);
                        retVal += m_pSer->setVal(buf, (int)strlen(buf));
                        Sleep(80);
                        memset(buf, 0, 100);
                        retVal += UhlReadFloat(&physpos);
                        if (retVal == ito::retOk)
                        {
                            memset(buf, 0, 100);
                            sprintf(buf, "!moa %c %f.1", m_nameAxis[_axis[axiscnt]], physpos);
                            retVal += m_pSer->setVal(buf, (int)strlen(buf));
                            retVal += UhlJoystickOn();
//                            Sleep(80);
                        }
/*
                        memset(wbuf, 0, sizeof(wbuf));
#if (defined __MINGW32__ || defined __MINGW64__)
                        swprintf(wbuf, L"Uhl Setpos: Endshwitch of Axis %c hit", m_nameAxis[axiscnt]);
#else
                        swprintf(wbuf, 100, L"Uhl Setpos: Endshwitch of Axis %c hit", m_nameAxis[axiscnt]);
#endif
                        ret += ito::RetVal(ito::retError, 0, wbuf);
*/
                        retVal += ito::RetVal(ito::retError, STATUS_UPPER_REF_SWITCH, tr("Uhl Setpos: End switch of axis %1 hit").arg(m_nameAxis[axiscnt]).toLatin1().data());
//                        setStatus(m_currentStatus[0], ito::actuatorRefSwitch | ito::actuatorRightEndSwitch | ito::actuatorEndSwitch, ito::actStatusMask);
                        setStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorAtTarget | ito::actuatorRefSwitch | ito::actuatorLeftEndSwitch | ito::actuatorEndSwitch | ito::actuatorLeftRefSwitch, ito::actStatusMask);
                        break;
                    }

                    case 'J':
                    {
                        retVal += ito::RetVal(ito::retError, 0, tr("Joystick is activ. Programming error?").toLatin1().data());
//                        setStatus(_axis, ito::actuatorAtTarget, ito::actStatusMask);
                        setStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorAtTarget | ito::actuatorRefSwitch | ito::actuatorLeftEndSwitch | ito::actuatorEndSwitch | ito::actuatorLeftRefSwitch, ito::actStatusMask);
                        return retVal;
                    }

                    default:
                    {
/*
                        memset(wbuf, 0, sizeof(wbuf));
#if (defined __MINGW32__ || defined __MINGW64__)
                        swprintf(wbuf, L"Unexpected Answer. S or @ expected, but got chr(%i)", status);
#else
                        swprintf(wbuf, 100, L"Unexpected Answer. S or @ expected, but got chr(%i)", status);
#endif
                        ret += ito::RetVal(ito::retError, 0, wbuf);
*/
                        retVal += ito::RetVal(ito::retError, 0, tr("Unexpected Answer. S or @ expected, but got chr(%1)").arg(status).toLatin1().data());
                        setStatus(_axis, ito::actuatorAtTarget, ito::actStatusMask);
                        Sleep(80);
                        retVal += UhlJoystickOn();
                        setStatus(m_currentStatus[_axis[axiscnt]], ito::actuatorAtTarget | ito::actuatorRefSwitch | ito::actuatorLeftEndSwitch | ito::actuatorEndSwitch | ito::actuatorLeftRefSwitch, ito::actStatusMask);
                        return retVal;
                    }
                }
            }
            done = true;
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
            if (timer.elapsed() > timeoutMS) timeout = true;
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
/*    chkcycles = (long)ceil((double)timeoutMS / UHLDELAY);

    for (cycle = 0; cycle < chkcycles; cycle++)
    {
        //Sleep(UHLDELAY); Copied to UhlReadString;
        memset(buf, 0, 100);
        ret += UhlReadString(buf, 6, 1, &cnts);
        if (cnts != 0)
        {
            bufcnt = strlen(buf);
            for (axiscnt = 0; axiscnt < axis.size(); axiscnt++)
            {
                // Check if axis has moved and is where it should be (@) or E-Strike (S) or Joystick on (J) or Error (else)
                status = buf[axiscnt];
                switch (status)
                {
                    case 'D':
                    case 'A':
                    case '@':
                    {
                        if (bufcnt < 5)
                        {
                            ret += ito::RetVal(ito::retError, 0, tr("Undefined answer in serial port buffer, not enough characters").toLatin1().data());
                        }
                        break;
                    }

                    case 'S':
                    {
                        //Eventuell Zielregister neu setzen

                        memset(buf, 100, 0);
                        sprintf(buf, "?pos %c", m_nameAxis[axiscnt]);
                        ret += m_pSer->setVal(buf, strlen(buf));
                        Sleep(80);
                        memset(buf, 0, 100);
                        ret += UhlReadFloat(&physpos);
                        if (ret == ito::retOk)
                        {
                            memset(buf, 0, 100);
                            sprintf(buf, "!moa %c %f.1", m_nameAxis[axiscnt], physpos);
                            ret += m_pSer->setVal(buf, strlen(buf));
                            ret += UhlJoystickOn();
//                            Sleep(80);
                        }
//                        memset(wbuf, 0, sizeof(wbuf));
//#if (defined __MINGW32__ || defined __MINGW64__)
//                        swprintf(wbuf, L"Uhl Setpos: Endshwitch of Axis %c hit", m_nameAxis[axiscnt]);
//#else
//                        swprintf(wbuf, 100, L"Uhl Setpos: Endshwitch of Axis %c hit", m_nameAxis[axiscnt]);
//#endif
//                        ret += ito::RetVal(ito::retError, 0, wbuf);
/*                        ret += ito::RetVal(ito::retError, STATUS_UPPER_REF_SWITCH, tr("Uhl Setpos: End switch of axis %1 hit").arg(m_nameAxis[axiscnt]).toLatin1().data());
                        setStatus(m_currentStatus[0], ito::actuatorRefSwitch | ito::actuatorRightEndSwitch | ito::actuatorEndSwitch, ito::actStatusMask);
                        break;
                    }

                    case 'J':
                    {
                        ret += ito::RetVal(ito::retError, 0, tr("Joystick is activ. Programming error?").toLatin1().data());
                        return ret;
                    }

                    default:
                    {
/*
                        memset(wbuf, 0, sizeof(wbuf));
#if (defined __MINGW32__ || defined __MINGW64__)
                        swprintf(wbuf, L"Unexpected Answer. S or @ expected, but got chr(%i)", status);
#else
                        swprintf(wbuf, 100, L"Unexpected Answer. S or @ expected, but got chr(%i)", status);
#endif
                        ret += ito::RetVal(ito::retError, 0, wbuf);
*/
/*                        ret += ito::RetVal(ito::retError, 0, tr("Unexpected Answer. S or @ expected, but got chr(%1)").arg(status).toLatin1().data());
                        Sleep(80);
                        ret += UhlJoystickOn();
                        return ret;
                    }
                }
            }
            return ret;
        }
        setAlive();

        if (counter == 10) // actualize position every 11. round
        {
            counter = 0;
            ret += getPos(_axis, sharedpos, NULL);
            sendStatusUpdate(false);
        }
        else
        {
            counter ++;
            sendStatusUpdate(true);
        }

        if (isInterrupted())
        {
            buf[0] = 'a';
            ret += m_pSer->setVal(buf, 1);
            Sleep(UHLDELAY);
            ret += DummyRead();

            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);

            ret += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            return ret;
        }

        QCoreApplication::processEvents();
    }
    replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
    ret += ito::RetVal(ito::retError, 0, tr("Dropped to time-out").toLatin1().data());
    return ret;
*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void UhlText::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal UhlText::UhlCheckAxisNumber(const int axis)
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
const ito::RetVal UhlText::UhlCheckAxisNumber(QVector<int> axis)
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
UhlText::UhlText() : AddInActuator(), m_spitchx(0), m_resolution(0), m_pSer(NULL), m_numAxis(0),
    m_turnspeed(40), m_accel(50), m_async(0), m_posrequestlisteners(0)
{
    m_scale = 1e3; // Uhl in \B5m the ugly itom in mm
    m_nameAxis[0] = 'x';
    m_nameAxis[1] = 'y';
    m_nameAxis[2] = 'z';
    m_nameAxis[3] = 'a';
//    qRegisterMetaType< QMap<QString,ito::tParam> >("QMap<QString,ito::tParam>");

    m_pitch[0] = 0.0;
    m_pitch[1] = 0.0;
    m_pitch[2] = 0.0;
    m_pitch[3] = 0.0;

    m_jogging[0] = 0;
    m_jogging[1] = 0;
    m_jogging[2] = 0;
    m_jogging[3] = 0;

    m_jogTimer.setInterval(1000);
    m_jogTimer.setParent(this);
    m_jogTimer.setSingleShot(false);
    connect(&m_jogTimer, SIGNAL(timeout()), this, SLOT(triggerJogIncrement()));

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "UhlText", NULL);
    m_params.insert(paramVal.getName(), paramVal);
//    paramVal = ito::tParam("getrangex", ito::ParamBase::Int, 0, 1, 0);
//    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("accel", ito::ParamBase::Double, 10.0, new ito::DoubleMeta(10.0,10000.0), tr("Accelaration of all axis in mm/s^2").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("speed", ito::ParamBase::Double, 0.0, 90.0, 0.0, tr("Speed of all axis in mm/s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inversex", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inversey", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inversez", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("inversea", ito::ParamBase::Int, 0, 1, 0, NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numaxis", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 3, 0, tr("Number of axis at controller").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("joyenabled", ito::ParamBase::Int, 0, 1, 1, tr("Enabled/disabled Joystick. Default: enabled").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("comPort", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 65355, 0, tr("The current com-port ID of this specific device. -1 means undefined").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("async", ito::ParamBase::Int, 0, 1, 0, tr("Toggle asynchrone mode of this device").toLatin1().data());
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
UhlText::~UhlText()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal UhlText::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

    \param [in] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::ParamBase, ItomSharedSemaphore
*/
ito::RetVal UhlText::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    char buf[50];
    float speed = 0.0;

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
                if (!retValue.containsError())
                {
                    Sleep(80);
                    retValue += DummyRead();
                    for (int i = 0; i < m_numAxis; i++)
                    {
                        /*
                        memset(buf, 0, 50);
                        sprintf(buf, "?pitch %c", m_nameAxis[i]);
                        retValue += m_pSer->setVal(buf, strlen(buf));
                        memset(buf, 0, 50);
                        retValue += UhlReadFloat(&m_pitch[i]);
                        if (retValue.containsWarningOrError())
                        {
                            retValue.appendRetMessage(tr(". Expected answer from serial port missed!").toLatin1().data());
                            Sleep(80);
                            retValue += DummyRead();
                            break;
                        }
                        else*/
                        {
                            speed = m_params["speed"].getVal<double>() / m_pitch[i];  // Transform mm/s to U/s by pitch [mm]
                            memset(buf, 0, 50);
                            sprintf(buf, "!vel %c %f", m_nameAxis[i], speed);
                            retValue += m_pSer->setVal(buf, (int)strlen(buf));
                        }
                        Sleep(80);
                        retValue += DummyRead();
                    }
                }
                retValue += UhlJoystickOn();
//                Sleep(80);
//                retValue += DummyRead();
            }
            else if (key == "async")
            {
                m_async = m_params["async"].getVal<int>();
                /*QVector<int> axis;
                axis << 0 << 1;
                QVector<double> vel;
                vel << 0.0 << 0.5;
                
                
                startJoyStickMovement(axis, vel);*/
/*
                memset(buf, 0, 50);
                sprintf(buf, "mor 2500 2500");
                retValue += m_pSer->setVal(buf, strlen(buf));
                sprintf(buf, "a");
                retValue += m_pSer->setVal(buf, strlen(buf));
                memset(buf, 0, 50);
                sprintf(buf, "mor -2500 0");
                retValue += m_pSer->setVal(buf, strlen(buf));
                */
            }
            else if (key == "accel")
            {
                m_accel = m_params["accel"].getVal<double>() / 1000.0;// from [10-10000] mm/s^2 to [0.01-10] m/s^2

                if (m_accel < 0.01)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("Too small acceleration").toLatin1().data());
                }
                if (m_accel > 10)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("Too high acceleration").toLatin1().data());
                }

                if (!retValue.containsError())
                {
                    retValue += UhlJoystickOff();
                }

                if (!retValue.containsError())
                {
//                    Sleep(80);
//                    retValue += DummyRead();

                    memset(buf, 0, 50);
                    sprintf(buf, "!accel");
                    for (int i = 0; i < m_numAxis; i++)
                    {
                        char buf2[20] = "";                    
                        sprintf(buf2, " %f", (float)m_accel);
                        strcat(buf, buf2);
//                        sprintf(buf, "%s %f", buf, (float)m_accel);    // check value !!!
                    }
                    retValue += m_pSer->setVal(buf, (int)strlen(buf));

                    Sleep(80);
                    retValue += DummyRead();
                }
                retValue += UhlJoystickOn();
//                Sleep(80);
//                retValue += DummyRead();
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
            else if (key == "inversea")
            {
                m_inverse[3] = GetAxisDir(m_params["inversea"].getVal<int>());
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
    if (retValue.containsWarningOrError())
    {
        char errBuf[128] = {0};
        _snprintf(errBuf, 128, tr(" in Uhl::SetParam for %s").toLatin1().data(), key.toLatin1().data());
        retValue.appendRetMessage(errBuf);
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
//ito::RetVal UhlText::setParam(const char *name, const double val, ItomSharedSemaphore *waitCond)
//{
//    ItomSharedSemaphoreLocker locker(waitCond);
//    ito::RetVal retval = ito::retError;
//    int i = 0, count = 0;
//    char buf[50];
//    float speed = 0.0;
//
//    QMap<QString, ito::Param>::iterator paramIt = m_params.find(name);
//    if (paramIt != m_params.end())
//    {
//        if (strcmp(paramIt.value().getName(), "numaxis") == 0)
//        {
//            retval = ito::RetVal(ito::retError, 0, L"Numaxis is readonly!");
//        }
//        else
//        {
//            paramIt.value().setVal<double>(val);
//            retval = ito::retOk;
//
//            if (strcmp(paramIt.value().getName(), "speed") == 0)
//            {
//                retval += UhlJoystickOff();
//                if (!retval.containsError())
//                {
//                    Sleep(80);
//                    retval += DummyRead();
//                    for (i = 0; i < m_numAxis; i++)
//                    {
//                        memset(buf, 0, 50);
//                        sprintf(buf, "?pitch %c", m_nameAxis[i]);
//                        retval += m_pSer->setVal(buf, strlen(buf));
//                        memset(buf, 0, 50);
//                        retval += UhlReadFloat(&m_pitch[i]);
//                        if (retval.containsWarningOrError())
//                        {
//                            retval += ito::RetVal(ito::retError, 0, L"Expected answer from serial port missed");
//                        }
//                        else
//                        {
//                            pitch = atof(buf);
//                            speed = m_params["speed"].getVal<double>() / pitch;  // Transform mm/s to U/s by pitch [mm]
//                            memset(buf, 0, 50);
//                            sprintf(buf, "!vel %c %f", m_nameAxis[i], speed);
//                            retval += m_pSer->setVal(buf, strlen(buf));
//                        }
//                        Sleep(80);
//                        retval += DummyRead();
//                    }
//                }
//                retval += UhlJoystickOn();
//                Sleep(80);
//                retval += DummyRead();
//            }
//            else if (strcmp(paramIt.value().getName(), "accel") == 0)
//            {
//                m_accel = m_params["accel"].getVal<double>() / 1000.0;// from [10-10000] mm/s^2 to [0.01-10] m/s^2
//
//                if (m_accel < 0.01)
//                {
//                    retval += ito::RetVal(ito::retError, 0, L"Too small acceleration");
//                }
//                if (m_accel > 10)
//                {
//                    retval += ito::RetVal(ito::retError, 0, L"Too high acceleration");
//                }
//
//                if (!retval.containsError())
//                {
//                    retval += UhlJoystickOff();
//                }
//
//                if (!retval.containsError())
//                {
//                    Sleep(80);
//                    retval += DummyRead();
//
//                    memset(buf, 0, 50);
//                    sprintf(buf, "!accel");
//
//                    for (i = 0; i < m_numAxis; i++)
//                    {
//                        sprintf(buf, "%s %f", buf, (float)m_accel);    // check value !!!
//                    }
//                    retval += m_pSer->setVal(buf, strlen(buf));
//
//                    Sleep(80);
//                    retval += DummyRead();
//                }
//                retval += UhlJoystickOn();
//
//            }
//            else if (strcmp(paramIt.value().getName(), "inversex") == 0)
//            {
//                m_inverse[0] = GetAxisDir(m_params["inversex"].getVal<int>());
//            }
//            else if (strcmp(paramIt.value().getName(), "inversey") == 0)
//            {
//                m_inverse[1] = GetAxisDir(m_params["inversey"].getVal<int>());
//            }
//            else if (strcmp(paramIt.value().getName(), "inversez") == 0)
//            {
//                m_inverse[2] = GetAxisDir(m_params["inversez"].getVal<int>());
//            }
//            else if (strcmp(paramIt.value().getName(), "inversea") == 0)
//            {
//                m_inverse[3] = GetAxisDir(m_params["inversea"].getVal<int>());
//            }
//        }
//    }
//
//    if (waitCond)
//    {
//        waitCond->release();
//        waitCond->returnValue = retval;
//
//    }
//    emit parametersChanged(m_params);
//    return retval;
//}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
QSharedPointer<double> tempdpos(new double);
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    char buf[50];
    int len = 0, n = 0;
    bool savedPos = false;
    float savePos[4] = {0.0, 0.0, 0.0, 0.0};
    float biggest_pitch = 0;
    float smallest_pitch = 1000;
    float speed = 0;
    float smallest_speed = 40.0;    //! Uhlspeed must be between 0.01 and 40.0 [Turns / s]
    float accel = 0;
    float smallest_accel = 10000;
    long axiscnt = 0;

    m_turnspeed = 60;
    m_accel = 50;
    m_async = 0;
    m_joyEnabled = 1;  // It's necessary to switching off Joystick!

    int bautRate = (*paramsOpt)[6].getVal<int>();

    for (n = 0; n < (int)sizeof(m_nameAxis); n++)
    {
        m_inverse[n] = GetAxisDir((*paramsOpt)[n + 1].getVal<int>());
        m_params.find((*paramsOpt)[n + 1].getName()).value().setVal<double>((double)(*paramsOpt)[n + 1].getVal<int>());
    }

//    m_pSer = qobject_cast<ito::AddInDataIO*>(reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>()));
//    if (m_pSer)
//    {
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

    setAlive();

    memset(buf, 0, 50);
    sprintf(buf, "?status");
    retval += m_pSer->setVal(buf, (int)strlen(buf));
    if (retval.containsError())
    {
        retval = m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, bautRate)), NULL);
        memset(buf, 0, 50);
        sprintf(buf, "?status");
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval.containsError())
        {

            retval.appendRetMessage(tr(" in UHL::INIT for sending status").toLatin1().data());
            goto end;
        }
    }

    Sleep(80);
    memset(&buf, 0, 50);
    retval += UhlReadString(buf, 6, 1, &n);
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT").toLatin1().data());
        goto end;
    }

    if (!strncmp(buf, "OK...", 5))  // UHL table is still initialized
    {
        // saving old position
        for (n = 0; n < 4; n++)
        {
            memset(buf, 0, 50);
            sprintf(buf, "?pos %c", m_nameAxis[n]);
            retval += m_pSer->setVal(buf, (int)strlen(buf));
            if (retval.containsError())
            {
                break;
            }

            Sleep(UHLDELAY);
            /*retval +=*/ UhlReadFloat(&savePos[n]);
            /*if (retval.containsError())
            {
                break;
            }*/
            //the retval is not evaluated here, since Uhl-tables with less than 4 axes
            //should not raise an error here. If the position of all 4 axes cannot be
            //read properly, no bad behaviour occurs and we can simply continue.

            Sleep(UHLDELAY);
        }

        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT for saving old position").toLatin1().data());
            goto end;
        }

        savedPos = true;

        // clean the world - otherwise the Suuper Uhl will fuck you ;-)
        memset(buf, 0, 50);
        sprintf(buf, "Reset");
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT for sending reset").toLatin1().data());
            goto end;
        }

        Sleep(500);
        retval += DummyRead();
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT").toLatin1().data());
            goto end;
        }
    }

    memset(buf, 0, 50);
    sprintf(buf, "U7mb");
    retval += m_pSer->setVal(buf, (int)strlen(buf));
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT sending U7mb").toLatin1().data());
        goto end;
    }

    Sleep(200);
    retval += DummyRead();
    if (retval.containsError())
    {
        goto end;
    }

    memset(buf, 0, 50);
    sprintf(buf, "?det");
    retval += m_pSer->setVal(buf, (int)strlen(buf));
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT sending ?det").toLatin1().data());
        goto end;
    }

    Sleep(100);
    retval += UhlReadLong(&axiscnt);
    if (retval.containsError())
    {
        if (retval.errorCode() == 9999)  // Could it be an UhlRegister?
        {
            ito::RetVal ret = ito::retOk;
            char buf[20];
            int count;

            buf[0] = 'U';
            buf[1] = 7;
            ret += m_pSer->setVal(buf, 2);
            if (!ret.containsError())
            {
                Sleep(80);
                memset(&buf, 0, 20);
                ret += UhlReadString(buf, 6, 1, &count);
            }

            if (!ret.containsError() && (strlen(buf) == 0))
            {
                buf[0] = 'U';
                buf[1] = 80;
                ret += m_pSer->setVal(buf, 2);
                if (!ret.containsError())
                {
                    Sleep(80);
                    memset(&buf, 0, 20);
                    ret += UhlReadString(buf, 6, 1, &count);
                }

                if (!ret.containsError() && (strlen(buf) == 0))
                {
                    buf[0] = 'j';
                    ret += m_pSer->setVal(buf, 1);
                    if (!ret.containsError())
                    {
                        Sleep(80);
                        memset(&buf, 0, 20);
                        ret += UhlReadString(buf, 6, 1, &count);
                    }
                }
            }

            if (!ret.containsError() && (strlen(buf) == 5) && (buf[4] == '.') && ((buf[0] == 'A') || (buf[0] == 'D') || (buf[0] == '@')))
            {
                retval = ito::RetVal(ito::retError, 0, tr("For this UHL table use the UhlRegister driver!").toLatin1().data());
            }
        }
        goto end;
    }

    m_numAxis = (0x000000F0 & (int)axiscnt) >> 4;
    if ((m_numAxis < 2) || (m_numAxis > 4))
    {
        retval += ito::RetVal(ito::retError, 0, tr("Stupid number of axis specified from uhltable").toLatin1().data());
        goto end;
    }
    m_params.find("numaxis").value().setVal<double>((double)(m_numAxis));

    m_currentStatus.fill(0,m_numAxis);
    m_currentPos.fill(0,m_numAxis);
    m_targetPos.fill(0,m_numAxis);
    for (n = 0; n<m_numAxis; n++)
    {
        m_currentStatus[n] = ito::actuatorAtTarget | ito::actuatorEnabled | ito::actuatorAvailable;
    }

    memset(buf, 0, 50);
    sprintf(buf, "!joydir");
    for (n = 0; n < m_numAxis; n++)
    {
        char buf2[20] = "";                    
        sprintf(buf2, " %i", m_inverse[n]);
        strcat(buf, buf2);
//        sprintf(buf, "%s %i", buf, m_inverse[n]);  // Axisdirections of joystick
    }

    retval += m_pSer->setVal(buf, (int)strlen(buf));
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT sending !!joydir").toLatin1().data());
        goto end;
    }

    memset(buf, 0, 50);
    sprintf(buf, "!dim");
    for (n = 0; n < m_numAxis; n++)
    {
        char buf2[20] = "";
        sprintf(buf2, " 1");
        strcat(buf, buf2);
//        sprintf(buf, "%s 1", buf);  // Axis in Micrometer
    }

    retval += m_pSer->setVal(buf, (int)strlen(buf));
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT sending !dim").toLatin1().data());
        goto end;
    }

    memset(buf, 0, 50);
    sprintf(buf, "?err");
    retval += m_pSer->setVal(buf, (int)strlen(buf));
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT sending ?err").toLatin1().data());
        goto end;
    }

    memset(buf, 0, 50);
    retval += UhlReadString(buf, 50, 5, &len);
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT reading ?err").toLatin1().data());
        goto end;
    }

    if (buf[0] != '0')
    {
        retval += ito::RetVal(ito::retError, 0, tr("uhlTextController::Init uhlcommand @ dim crashed").toLatin1().data());
        goto end;
    }

    if (bautRate != 9600)
    {
        memset(buf, 0, 50);
        sprintf(buf, "!baud %i", bautRate);
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT sending ?err").toLatin1().data());
            goto end;
        }

        retval += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud",ito::ParamBase::Int,115200)),NULL);
        /*
        retval += UhlReadString(buf, 50, 5, &len);
        if (buf[0] != '0')
        {
            retval += ito::RetVal(ito::retError, 0, tr("uhlTextController::Init uhlcommand @ !baud115200 crashed").toLatin1().data());
            goto end;
        }
        */
        DummyRead();
    }

    retval += UhlJoystickOff(false);  // "false" because UHL table will never answer at this time - but I don't know why...
    m_joyEnabled = 0;  // Disable Joystick temporary!

    // get the Stepperspeed of all axis, take the smallest one and write axis speed back to all axis
    for (n = 0; n < m_numAxis; n++)
    {
        memset(buf, 0, 50);
        sprintf(buf, "?pitch %c", m_nameAxis[n]);

        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT sending ?pitch").toLatin1().data());
            break;
        }

        Sleep(100);
        retval += UhlReadFloat(&m_pitch[n]);
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT reading ?pitch").toLatin1().data());
            break;
        }

        memset(buf, 0, 50);
        sprintf(buf, "?vel %c", m_nameAxis[n]);
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT sending ?vel").toLatin1().data());
            break;
        }

        Sleep(100);
        retval += UhlReadFloat(&speed);
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT reading ?vel").toLatin1().data());
            break;
        }

        speed = speed * m_pitch[n]; // to transform speed [U/s] to [mm/s]

        if ((speed < smallest_speed * m_pitch[n]) && (speed > 0))
        {
            smallest_speed = speed;  // set smallest speed in [mm/s] ToDO Turningaxis??
        }
        if (m_pitch[n] < smallest_pitch)
        {
            smallest_pitch = m_pitch[n];
        }
        if (m_pitch[n] > biggest_pitch)
        {
            biggest_pitch = m_pitch[n];
        }

        memset(buf, 0, 50);
        sprintf(buf, "?accel %c", m_nameAxis[n]);
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT sending ?accel").toLatin1().data());
            break;
        }

        Sleep(100);
        retval += UhlReadFloat(&accel);
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT reading ?accel").toLatin1().data());
            break;
        }

        if (accel < smallest_accel)
        {
            smallest_accel = accel; // Warning accel is m/s^2, itom uses mm/s^2, transformed before writen back
        }
    }
    
    m_params["speed"].setMeta(new ito::DoubleMeta((double)(0.01 * smallest_pitch) , (double)(40.0 * biggest_pitch)), true);
    //m_params.find("speed").value().setMax((double)(40.0 * biggest_pitch));
    //m_params.find("speed").value().setMin((double)(0.01 * smallest_pitch));

    retval += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed",ito::ParamBase::Double,smallest_speed)),NULL);// Speedback speed. Here mm/s transformed in setparam to [U/s] = [mm/s] / Pitch
    retval += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("accel",ito::ParamBase::Double,smallest_accel * 1000)),NULL);// Switch m/s^2 to mm/s^2

    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" in UHL::INIT").toLatin1().data());
        goto end;
    }

    // write the saved position back
    if (savedPos)
    {
        memset(buf, 0, 50);
        sprintf(buf, "!pos");
        for (n = 0; n < m_numAxis; n++)
        {
            char buf2[20] = "";
            sprintf(buf2, " %i", (int)savePos[n]);
            strcat(buf, buf2);        
//            sprintf(buf, "%s %i", buf, (int)savePos[n]);
        }

        retval += m_pSer->setVal(buf, (int)strlen(buf));
        Sleep(UHLDELAY);

        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT for writing old position").toLatin1().data());
            goto end;
        }

        retval  += DummyRead();
        if (retval.containsError())
        {
            retval.appendRetMessage(tr(" in UHL::INIT for writing old position").toLatin1().data());
            goto end;
        }
    }

    m_joyEnabled = (*paramsOpt)[5].getVal<int>();
    m_params.find((*paramsOpt)[5].getName()).value().setVal<double>((double) m_joyEnabled);
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
ito::RetVal UhlText::close(ItomSharedSemaphore *waitCond)
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
ito::RetVal UhlText::calib(const int axis, ItomSharedSemaphore *waitCond)
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
ito::RetVal UhlText::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    char buf[100];
    char maskbufcal[20];
    char maskcal[20];
    char maskbufrevcal[20];
    float positions[4] = {0.0, 0.0, 0.0, 0.0};
    int axis_cnt = 0;
    bool enable[4] = { 0, 0, 0, 0 };
    bool do_calib = false;
    bool do_revcalib = false;
    ito::RetVal retval = UhlCheckAxisNumber(axis);

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError, 0, tr("Any motor axis is already moving").toLatin1().data());

        if (waitCond)
        {
            waitCond->release();
            waitCond->returnValue = retval;
        }
    }
    else if (!retval.containsError())
    {
        retval += UhlJoystickOff();

        //first read all available positions
        for (axis_cnt = 0; axis_cnt < m_numAxis; ++axis_cnt)
        {
            memset(buf, 0, 50);
            sprintf(buf, "?pos %c", m_nameAxis[axis_cnt]);
            retval += m_pSer->setVal(buf, (int)strlen(buf));
            retval += UhlReadFloat(&positions[axis_cnt]);
        }

        if (!retval.containsError())
        {
            //calibrate every desired axis (single axis calibration)
            foreach(const int a, axis)
            {
                memset(buf, 0, 50);

                if (m_inverse[a] > 0)
                {
                    sprintf(buf, "!rm %c", m_nameAxis[a]);
                }
                else
                {
                    sprintf(buf, "!cal %c", m_nameAxis[a]);
                }

                retval  += DummyRead();
                retval += m_pSer->setVal(buf, (int)strlen(buf));

                if (!retval.containsError())
                {
                    retval += waitForDone(60000, QVector<int>(1, a));
                    if (retval.containsError())
                    {
                        if ((retval.errorCode() == STATUS_UPPER_REF_SWITCH) || (retval.errorCode() == STATUS_LOWER_REF_SWITCH))
                        {
                            retval = ito::retOk;
                        }
                        else
                        {
                            retval.appendRetMessage(tr(" in UHL::CALIB").toLatin1().data());
                        }
                    }
                }
            }
        }

        if (!retval.containsError())
        {
            //set the positions of all axes that have not been calibrated to their original position
            //set the position of all calibrated axes to 0.0
            for (axis_cnt = 0; axis_cnt < m_numAxis; ++axis_cnt)
            {
                memset(buf, 0, 50);

                if (axis.contains(axis_cnt))
                {
                    sprintf(buf, "!pos %c %.1f", m_nameAxis[axis_cnt], (float)0.0);
                    m_currentPos[axis_cnt] = 0.0;
                }
                else
                {
                    sprintf(buf, "!pos %c %.1f", m_nameAxis[axis_cnt], positions[axis_cnt]);
                    m_currentPos[axis_cnt] = positions[axis_cnt];
                }
                retval += m_pSer->setVal(buf, (int)strlen(buf));
            }
        }

        retval += UhlJoystickOn();

//        memset(maskcal, 0, sizeof(maskcal));
//        sprintf(maskcal, "!axis 0 0 0 0");
//        for (axis_cnt = 0; axis_cnt < (unsigned long)m_numAxis; axis_cnt++)
//        {
//            maskcal[6 + (axis_cnt * 2)] = '1';
//        }
//        // Aktivate all existing axis
//        retval += m_pSer->setVal(maskcal, (int)strlen(maskcal));
//
//        if (retval != ito::retError)
//        {
//            retval += UhlJoystickOff();
//        }
//
//        if (retval != ito::retError)
//        {
//            // Workaround for Bug in der F9S-3-O: At Reset of one axis
//            // some Controller forget the positions of the other axis
//
//            for (axis_cnt = 0; axis_cnt < (unsigned long)m_numAxis; axis_cnt++)
//            {
//                memset(buf, 0, 50);
//                sprintf(buf, "?pos %c", m_nameAxis[axis_cnt]);
//                retval += m_pSer->setVal(buf, (int)strlen(buf));
//                retval += UhlReadFloat(&positions[axis_cnt]);
//            }
//
//            if (retval != ito::retError)
//            {
//                // Masked all axis expect the active one (bit-mask)
//                memset(maskbufcal, 0, sizeof(maskbufcal));
//                memset(maskbufrevcal, 0, sizeof(maskbufrevcal));
//                sprintf(maskbufcal, "!axis");
//                sprintf(maskbufrevcal, "!axis");
//
//                for (axis_cnt = 0; axis_cnt < (unsigned long)axis.size(); axis_cnt++)
//                {
//                    enable[axis[axis_cnt]] = true;
//                }
//
//                for (axis_cnt = 0; axis_cnt < (unsigned long)axis.size(); axis_cnt++)
//                {
//                    char buf2[20] = "";
//                    sprintf(buf2, " %i", (m_inverse[axis_cnt] > 0 ? 1 : 0) && enable[axis_cnt]);
//                    strcat(maskbufcal, buf2);
////                    sprintf(maskbufcal, "%s %i", maskbufcal, (m_inverse[axis_cnt] > 0 ? 1 : 0) && enable[axis_cnt]);
//                    buf2[0] = 0;
//                    sprintf(buf2, " %i", (m_inverse[axis_cnt] > 0 ? 0 : 1) && enable[axis_cnt]);
//                    strcat(maskbufrevcal, buf2);                    
////                    sprintf(maskbufrevcal, "%s %i", maskbufrevcal, (m_inverse[axis_cnt] > 0 ? 0 : 1) && enable[axis_cnt]);
//                    if (m_inverse[axis[axis_cnt]] > 0)
//                    {
//                        do_calib = true;
//                    }
//                    if (m_inverse[axis[axis_cnt]] < 0)
//                    {
//                        do_revcalib = true;
//                    }
//                }
//                // Write mask for calibration command
//                if (do_calib) // because mask_calib=0 sometimes behaves like all axis on
//                {
//                    retval += m_pSer->setVal(maskbufcal, (int)strlen(maskbufcal));
//                    Sleep(80);
//                    retval  += DummyRead();
//
//                    memset(buf, 0, 50);
//                    sprintf(buf, "!cal"); // Start
//                    if (retval != ito::retError)
//                    {
//                        retval += m_pSer->setVal(buf, (int)strlen(buf));
//                    }
//
//                    if (retval != ito::retError)
//                    {
//                        retval += waitForDone(60000, axis);
//                        if (retval.containsError())
//                        {
//                            if ((retval.errorCode() == STATUS_UPPER_REF_SWITCH) || (retval.errorCode() == STATUS_LOWER_REF_SWITCH))
//                            {
//                                retval = ito::retOk;
//                            }
//                            else
//                            {
//                                retval.appendRetMessage(tr(" in UHL::CALIB").toLatin1().data());
//                            }
//                        }
//    /*                        else
//                        {
//                            retval += ito::RetVal(ito::retError, 0, tr("error: no reference switch reached").toLatin1().data());
//                        }*/
//                    }
//                }
//
//                // Aktivate all existing axis
//                retval += m_pSer->setVal(maskcal, (int)strlen(maskcal));
//
//                retval += UhlJoystickOff();
//
//                // Write mask for calibration command
//                if (retval != ito::retError)
//                {
//                    if (do_revcalib) // because mask_calib=0 sometimes behaves like all axis on
//                    {
//                        retval += m_pSer->setVal(maskbufrevcal, (int)strlen(maskbufrevcal));
//                        Sleep(80);
//                        retval  += DummyRead();
//
//                        memset(buf, 0, 50);
//                        sprintf(buf, "!rm"); // Start
//                        if (retval != ito::retError)
//                        {
//                            retval += m_pSer->setVal(buf, (int)strlen(buf));
//                        }
//
//                        if (retval != ito::retError)
//                        {
//                            retval += waitForDone(60000, axis);
//                            if (retval.containsError())
//                            {
//                                if ((retval.errorCode() == STATUS_UPPER_REF_SWITCH) || (retval.errorCode() == STATUS_LOWER_REF_SWITCH))
//                                {
//                                    retval = ito::retOk;
//                                }
//                                else
//                                {
//                                    retval.appendRetMessage(tr(" in UHL::CALIB").toLatin1().data());
//                                }
//                            }
//        /*                        else
//                            {
//                                retval += ito::RetVal(ito::retError, 0, tr("error: no reference switch reached").toLatin1().data());
//                            }*/
//                        }
//                    }
//                }
//                // Write back absolut positions of other axis
//                for (axis_cnt = 0; axis_cnt < (unsigned long)m_numAxis; axis_cnt++)
//                {
//                    memset(buf, 0, 50);
//                    sprintf(buf, "!pos %c %.1f", m_nameAxis[axis_cnt], (float)positions[axis_cnt]);
//                    retval += m_pSer->setVal(buf, (int)strlen(buf));
//                }
//
//                // Set absolutposition und target register of calibrated axis to zero
//                for (axis_cnt = 0; axis_cnt < (unsigned long)axis.size(); axis_cnt++)
//                {
//                    if (enable[axis_cnt])
//                    {
//                        memset(buf, 0, 50);
//                        sprintf(buf, "!pos %c %.1f", m_nameAxis[axis_cnt], (float)0);
//                        retval += m_pSer->setVal(buf, (int)strlen(buf));
//                        m_currentPos[axis_cnt] = (float)0;
//                    }
//                }
//
//                // Activate all existing axis
//                retval += m_pSer->setVal(maskcal, (int)strlen(maskcal));
//            }
//        }
//
//        (void)UhlJoystickOn();

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
    //        emit SentPositionChanged(axis, *sharedpos);
        }
    }

    sendStatusUpdate(false);
//    emit PositioningStatusChanged(0);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    char buf[50];

    retval = UhlCheckAxisNumber(axis);
    retval = DummyRead();

    if (axis < m_numAxis)
    {
        memset(buf, 0, 50);
        sprintf(buf, "!pos %c %i", m_nameAxis[axis], 0);
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        if (retval != ito::retError)
        {
             m_currentPos[axis] = 0.0;
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("Axis not exist").toLatin1().data());
    }
    sendStatusUpdate(false);    

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond)
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
ito::RetVal UhlText::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
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
ito::RetVal UhlText::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    float retPos = 0.0;
    char buf[50];

    retval = UhlCheckAxisNumber(axis);
    retval = DummyRead();

    if (axis < m_numAxis)
    {
        memset(buf, 0, 50);
        sprintf(buf, "?pos %c", m_nameAxis[axis]);
        retval += m_pSer->setVal(buf, (int)strlen(buf));
        Sleep(UHLDELAY);
        retval += UhlReadFloat(&retPos);
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
ito::RetVal UhlText::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    QSharedPointer<double> dpos(new double);
    *dpos = 0.0;
    int axis_cnt = 0;

    for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
    {
        retval += getPos(axis.value(axis_cnt), dpos, 0); // Faster if we use the multiaxis command but less robust
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
const ito::RetVal UhlText::UhlSetPos(QVector<int> axis, QVector<double> pos, const unsigned char absrelflag, ItomSharedSemaphore *waitCond)
{
    char buf[100];
    double dpos[4] = { 0.0, 0.0, 0.0, 0.0 };
    int axisfound[4] = { 0, 0, 0, 0 };
    int axis_cnt;
    QSharedPointer<double> tempdpos(new double);
    *tempdpos = 0.0;
    ito::RetVal retval = ito::retOk;
    bool released = false;

	int timeoutMS = m_params["timeout"].getVal<ito::float64>() * 1000;

    if (isMotorMoving())
    {
        retval += ito::RetVal(ito::retError,0,"Any motor axis is already moving");

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
//        retval += DummyRead();
//        Sleep(UHLDELAY);

        memset(buf, 0, sizeof(buf));

        for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
        {
            //update target position
            if (absrelflag == GOABS) //move absolute
            {
                m_targetPos[ axis[axis_cnt] ] = pos.value(axis_cnt);
            }
            else //move relative
            {
                m_targetPos[ axis[axis_cnt] ] = m_currentPos[ axis[axis_cnt]] + pos.value(axis_cnt);
            }
        }

//        sendTargetUpdate();

        if (absrelflag == GOABS)
        {
            sprintf(buf, "!moa");
        }
        else
        {
            sprintf(buf, "!mor");
        }

        if ((axis.size() == 1)) // okay single axis is easy
        {
            dpos[0] = pos[0] * m_scale;
            char buf2[20] = "";
            sprintf(buf2, " %c %f.1", m_nameAxis[axis[0]], dpos[0] * m_inverse[axis.value(0)]);
            strcat(buf, buf2);            
//            sprintf(buf, "%s %c %f.1", buf, m_nameAxis[axis[0]], dpos[0] * m_inverse[axis.value(0)]);
        }
        else if (axis.size() == m_numAxis)  // All attached axis is still easy
        {
            // resort the positions ascending
/*            dpos[axis[0]] = pos[axis[0]] * m_scale;
            dpos[axis[1]] = pos[axis[1]] * m_scale;

            if (m_numAxis == 3)
            {
                dpos[axis[2]] = pos[axis[2]] * m_scale;
                sprintf(buf, "%s %f.1 %f.1 %f.1", buf, dpos[0] * m_inverse[axis.value(0)], dpos[1] * m_inverse[axis.value(1)], dpos[2] * m_inverse[axis.value(2)]);
            }
            else if (m_numAxis == 2)
            {
                sprintf(buf, "%s %f.1 %f.1", buf, dpos[0] * m_inverse[axis.value(0)], dpos[1] * m_inverse[axis.value(1)]);
            }
            else
            {
                dpos[axis[2]] = pos[axis[2]] * m_scale;
                dpos[axis[3]] = pos[axis[3]] * m_scale;
                sprintf(buf, "%s %f.1 %f.1 %f.1 %f.1", buf, dpos[0] * m_inverse[axis.value(0)], dpos[1] * m_inverse[axis.value(1)], dpos[2] * m_inverse[axis.value(2)], dpos[3] * m_inverse[axis.value(3)]);
            }*/

            for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
            {
                dpos[axis[axis_cnt]] = pos[axis[axis_cnt]] * m_scale;
                char buf2[20] = "";
                sprintf(buf2, " %f.1", dpos[axis_cnt] * m_inverse[axis.value(axis_cnt)]);
                strcat(buf, buf2);                    
//                sprintf(buf, "%s %f.1", buf, dpos[axis_cnt] * m_inverse[axis.value(axis_cnt)]);
            }
        }
        else    // now its getting complex!! We have to find the missing axis
        {
            for (axis_cnt = 0; axis_cnt < axis.size(); axis_cnt++)
            {
                dpos[axis[axis_cnt]] = pos[axis[axis_cnt]] * m_scale;
                axisfound[axis[axis_cnt]]++;
            }
            for (axis_cnt = 0; axis_cnt < m_numAxis; axis_cnt++)
            {
                if (axisfound[axis_cnt] > 1)
                {
                     retval += ito::RetVal(ito::retError, 0, tr("Axis was specified twice").toLatin1().data());
                }

                if (axisfound[axis_cnt] == 0)
                {
                    if (absrelflag == GOABS)
                    {
                        retval += getPos(axis_cnt, tempdpos, 0);
                        dpos[axis_cnt] = *tempdpos * m_scale;
                    }
                    else
                    {
                        dpos[axis_cnt] = 0.0;
                    }
                }
                char buf2[20] = "";
                sprintf(buf2, " %f.1", dpos[axis_cnt] * m_inverse[axis_cnt]);
                strcat(buf, buf2);                
//                sprintf(buf, "%s %f.1", buf, dpos[axis_cnt] * m_inverse[axis_cnt]);
            }
        }

        sendTargetUpdate();

        retval += m_pSer->setVal(buf, (int)strlen(buf));

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

        if (waitCond && !released)
        {
            waitCond->returnValue = retval;
            waitCond->release();
            released = true;
        }
/*
        if (!m_async && retval == ito::retOk)
        {
            retval += waitForDone(timeoutMS, axis);
        }

        if (retval != ito::retError)
        {
            retval += UhlJoystickOn();
        }

        if (m_posrequestlisteners && !m_async)
        {
            QSharedPointer<QVector<double> > sharedpos = QSharedPointer<QVector<double> >(new QVector<double>);

            for (int i=0; i<axis.size(); i++)
            {
                *sharedpos << 0;
            }

            retval += getPos(axis, sharedpos, 0);
    //        emit SentPositionChanged(axis, *sharedpos);
        }
*/
    }
//    emit PositioningStatusChanged(0);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    retval = UhlSetPos(QVector<int>(1, axis), QVector<double>(1, pos), GOABS, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    retval = UhlSetPos(axis, pos, GOABS, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;
    retval = UhlSetPos(QVector<int>(1, axis), QVector<double>(1, pos), GOREL, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval = ito::retOk;

    retval = UhlSetPos(axis, pos,GOREL, waitCond);

/*    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }*/

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::startJoyStickMovement(QVector<int> axis, QVector<double> vel)
{
    if (axis.size() != vel.size())
        return ito::retError;

    int status = 0;
    float speed;

    if (axis.size() == 2)
    {

        char buf[50];
        memset(buf, 0, 50);

        bool xFast = qAbs(vel[0]) > qAbs(vel[1]);
        m_jogDir[0] = vel[0] < 0.0 ? false : true;
        m_jogDir[1] = vel[1] < 0.0 ? false : true;
        if (qAbs(vel[1]) < 0.001 && qAbs(vel[0]) < 0.001)
        { 
            buf[0] = 'a';
            m_pSer->setVal(buf, 1);

            requestStatusAndPosition(true, false);

            speed = m_params["speed"].getVal<double>();  // Transform mm/s to U/s by pitch [mm]
            sprintf(buf, "!vel %f.1 %f.1", speed / m_pitch[0], speed / m_pitch[1]);
            m_pSer->setVal(buf, (int)strlen(buf));

            if (m_params["joyenabled"].getVal<int>())
            {
                (void)UhlJoystickOff();
                m_joyEnabled = m_params["joyenabled"].getVal<int>();
                (void)UhlJoystickOn();
            }
            m_jogging[0] = 0;
            m_jogging[1] = 0; 
            m_jogTimer.stop();
            return ito::retOk;
        }

        (void)UhlJoystickOff();
        m_joyEnabled = 0;

        int timing = 50;
        /*int iVel0 = 1;
        int iVel1 = 1;

        if (abs(vel[1]) < 0.001)    // just x
        {
            m_jogging[0] = 1;
            m_jogging[1] = 0;
         
            iVel0 = vel[0] * timing + 0.5;

            while (iVel0 < 1 && timing < 300)
            {
                timing++;
                iVel0 = vel[0] * timing + 0.5;
            }
            
        }
        else if (abs(vel[0]) < 0.001) // just y
        {
            m_jogging[0] = 0;
            m_jogging[1] = 1;
         
            iVel1 = vel[1] * timing + 0.5;

            while (iVel1 < 1 && timing < 300)
            {
                timing++;
                iVel1 = vel[1] * timing + 0.5;
            }       
        }
        else
        {
            if (xFast)
            {
                iVel0 = vel[0] * timing + 0.5;
                while (iVel0 < 1 && timing < 300)
                {
                    timing++;
                    iVel0 = vel[0] * timing + 0.5;
                    iVel1 = vel[1] * timing + 0.5;
                }

            }
            else
            {
                iVel1 = vel[1] * timing + 0.5;
                while (iVel1 < 1 && timing < 300)
                {
                    timing++;
                    iVel0 = vel[0] * timing + 0.5;
                    iVel1 = vel[1] * timing + 0.5;
                }
            }
            m_jogging[0] = 1;
            m_jogging[1] = 1; 
        }
        */

        if (qAbs(vel[1]) < 0.001)    // just x
        {        
            m_jogging[0] = vel[0] * timing + 0.5;
            m_jogging[1] = 0;

            while (m_jogging[0] < 1 && timing < 300)
            {
                timing++;
                m_jogging[0] = vel[0] * timing + 0.5;
            }
            speed = vel[0];
        }
        else if (qAbs(vel[0]) < 0.001) // just y
        {
            m_jogging[0] = 0;
            m_jogging[1] = vel[1] * timing + 0.5;

            while (m_jogging[1] < 1 && timing < 300)
            {
                timing++;
                m_jogging[1] = vel[1] * timing + 0.5;
            }
            speed = vel[1];
        }
        else
        {
            if (xFast)
            {
                m_jogging[0]  = vel[0] * timing + 0.5;
                while (m_jogging[0]  < 1 && timing < 300)
                {
                    timing++;
                    m_jogging[0] = vel[0] * timing + 0.5;
                    m_jogging[1] = vel[1] * timing + 0.5;
                }
                speed = vel[0];
            }
            else
            {
                m_jogging[1] = vel[1] * timing + 0.5;
                while (m_jogging[1] < 1 && timing < 300)
                {
                    timing++;
                    m_jogging[0] = vel[0] * timing + 0.5;
                    m_jogging[1] = vel[1] * timing + 0.5;
                }
                speed = vel[1];
            }
        }

        sprintf(buf, "!vel %f.1 %f.1", speed / m_pitch[0], speed / m_pitch[1]);
        m_pSer->setVal(buf, (int)strlen(buf));

        m_jogTimer.setInterval(timing);

        //sprintf(buf, "!tvrf %i %i", iVel0, iVel1); 
        //m_pSer->setVal(buf, strlen(buf));
        /*
        if (timing  > 0)
        {
            m_jogTimer.setInterval(timing);
        }
        else
        {
        
        }
        */
        if (!m_jogTimer.isActive())
        {
            m_jogTimer.start();
        }
        
    }
    else
    {
        return ito::retError;
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void UhlText::triggerJogIncrement()
{ 
    char buf[30];
    memset(buf, 0, 30);

    //buf[0] = 'a';
    //m_pSer->setVal(buf, 1);

    sprintf(buf, "mor %i %i", m_jogging[0], m_jogging[1]);

    m_pSer->setVal(buf, (int)strlen(buf));
    /*
    char buf[3] = {0, 0, 0};
    for (int i = 0; i < m_numAxis; i++)
    {
        
        if (m_jogging[i])
        {
            
            buf[0] = m_jogDir ? 'p' : 'n'; 
            buf[1] = m_nameAxis[i];
            for (int t = 0; t < m_jogging[i]; t++)
            {
                m_pSer->setVal(buf, 2);
            }

            

        }
    }*/
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal UhlText::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
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

/*    if (retval == ito::retError)
    {
        emit SentStatusChanged(2);
    }
    else
    {
        emit SentStatusChanged(0);
    }

    retval += getPos(axis, sharedpos, 0);
    emit SentPositionChanged(axis, *sharedpos);
*/

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
void UhlText::connectNotify (const char * signal)
{
    if (QLatin1String(signal) == SIGNAL(SentPositionChanged(QVector<int>,QVector<double>)))
    {
        m_posrequestlisteners++;
    }

    AddInActuator::connectNotify(signal);
}*/

//----------------------------------------------------------------------------------------------------------------------------------
/*
void UhlText::disconnectNotify (const char * signal)
{
    if (QLatin1String(signal) == SIGNAL(SentPositionChanged(QVector<int>,QVector<double>)))
    {
        m_posrequestlisteners--;
    }

    AddInActuator::disconnectNotify(signal);
}*/

//----------------------------------------------------------------------------------------------------------------------------------
