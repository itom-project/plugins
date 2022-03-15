/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "quantumComposer.h"
#include "gitVersion.h"
#include "pluginVersion.h"

#include <qmessagebox.h>
#include <qplugin.h>
#include <qstring.h>
#include <qstringlist.h>
#include <qwaitcondition.h>

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
QuantumComposerInterface::QuantumComposerInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO; // any grabber is a dataIO device AND its subtype
                                               // grabber (bitmask -> therefore the OR-combination).
    setObjectName("QuantumComposer");

    m_description = QObject::tr("QuantumComposer");

    // for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = "";
    m_detaildescription = QObject::tr(
        "QuantumComposer is an itom-plugin to communicate with the pulse generator 9520 series. \n\
\n\
This plugin has been developed for the 9520 series via a RS232 interface. So you first have to create an instance of the SerialIO plugin \n\
which is a mandatory input argument of the QuantumComposer plugin. \n\
The plugin sets the right RS232 parameter during initialization. \n\
\n\
The default parameters are: \n\
* Baud Rate: 38400 (default for USB), 115200 (default for RS232)\n\
* Data Bits: 8 \n\
* Parity: None \n\
* Stop Bits: 1\n\
* endline: \\r\\n");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal(
        "serialIOInstance",
        ito::ParamBase::HWRef | ito::ParamBase::In,
        nullptr,
        tr("An opened serial port.").toLatin1().data());
    paramVal.setMeta(new ito::HWMeta("SerialIO"), true);
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param(
        "connection",
        ito::ParamBase::String,
        "USB",
        "Type of connection ('USB', 'RS232'). The Baud Rate for the USB connection will be set "
        "to 38400 and for RS232 to 115200.");
    ito::StringMeta* sm = new ito::StringMeta(ito::StringMeta::String, "USB");
    sm->addItem("RS232");
    paramVal.setMeta(sm, true);
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
QuantumComposerInterface::~QuantumComposerInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposerInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(QuantumComposer) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposerInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(
        QuantumComposer) // the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(
    quantumcomposerinterface,
    QuantumComposerInterface) // the second parameter must correspond to the class-name of the
                              // interface class, the first parameter is arbitrary (usually the same
                              // with small letters only)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or
   remove entries from m_params in this constructor or later in the init method
*/
QuantumComposer::QuantumComposer() :
    AddInDataIO(), m_pSer(nullptr), m_delayAfterSendCommandMS(100), m_requestTimeOutMS(500)
{
    ito::Param paramVal = ito::Param(
        "name",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "QuantumComposer",
        tr("Plugin name.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "manufacturer",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Manufacturer identification.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "model",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Model identification.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "serialNumber",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Serial number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "version",
        ito::ParamBase::String | ito::ParamBase::Readonly,
        "unknown",
        tr("Version number.").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "Device parameter"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "requestTimeout",
        ito::ParamBase::Int,
        m_requestTimeOutMS,
        new ito::IntMeta(0, std::numeric_limits<int>::max(), 1, "SerialIO parameter"),
        tr("Request timeout for the SerialIO interface.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "mode",
        ito::ParamBase::String,
        "",
        tr("Mode of the system output. (NORM: normal, SING: single shot, BURST: burst, DCYC: duty "
           "cycle).")
            .toLatin1()
            .data());
    ito::StringMeta* sm = new ito::StringMeta(ito::StringMeta::String, "NORM", "Device parameter");
    sm->addItem("NORM");
    sm->addItem("SING");
    sm->addItem("BURST");
    sm->addItem("DCYC");
    sm->setCategory("Device parameter");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "gateMode",
        ito::ParamBase::String,
        "",
        tr("Global gate mode of the system output. (DIS: diabled, PULS: pulse inhibit, OUTP: "
           "output inhibit, CHAN: channel)."
           "cycle).")
            .toLatin1()
            .data());
    sm = new ito::StringMeta(ito::StringMeta::String, "DIS", "Device parameter");
    sm->addItem("DIS");
    sm->addItem("PULS");
    sm->addItem("OUTP");
    sm->addItem("CHAN");
    sm->setCategory("Device parameter");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
QuantumComposer::~QuantumComposer()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal QuantumComposer::init(
    QVector<ito::ParamBase>* paramsMand,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QByteArray answer;

    if (reinterpret_cast<ito::AddInBase*>((*paramsMand)[0].getVal<void*>())
            ->getBasePlugin()
            ->getType() &
        (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO*)(*paramsMand)[0].getVal<void*>();
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            1,
            tr("Input parameter is not a dataIO instance of ther SerialIO Plugin!")
                .toLatin1()
                .data());
    }

    if (!retValue.containsError())
    {
        QByteArray connectionType = paramsOpt->at(0).getVal<char*>();
        int baud;
        if (connectionType == "USB")
        {
            baud = 38400;
        }
        else if (connectionType == "RS232")
        {
            baud = 115200;
        }
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, baud)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)),
            nullptr);
        retValue += m_pSer->setParam(
            QSharedPointer<ito::ParamBase>(
                new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")),
            nullptr);

        QSharedPointer<QVector<ito::ParamBase>> _dummy;
        m_pSer->execFunc("clearInputBuffer", _dummy, _dummy, _dummy, nullptr);
        m_pSer->execFunc("clearOutputBuffer", _dummy, _dummy, _dummy, nullptr);

        retValue += SendQuestionWithAnswerString("*IDN?", answer, 1000);

        if (!retValue.containsError())
        {
            QByteArrayList idn =
                answer.split(','); // split identification answer in lines and by comma
            if (idn.length() == 4)
            {
                m_params["manufacturer"].setVal<char*>(idn[0].data());
                m_params["model"].setVal<char*>(idn[1].data());
                m_params["serialNumber"].setVal<char*>(idn[2].data());
                m_params["version"].setVal<char*>(idn[3].data());
            }
            else
            {
                retValue += ito::RetVal(
                    ito::retError,
                    1,
                    tr("Answer of the identification request is not valid!").toLatin1().data());
            }
        }

        if (!retValue.containsError())
        {
            retValue += SendQuestionWithAnswerString(":PULSE0:MODE?", answer, 1000);
            m_params["mode"].setVal<char*>(answer.data());

            retValue += SendQuestionWithAnswerString(":PULSE0:GAT:MOD?", answer, 1000);
            m_params["gateMode"].setVal<char*>(answer.data());
        }


        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); // init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal QuantumComposer::close(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // todo:
    //  - disconnect the device if not yet done
    //  - this funtion is considered to be the "inverse" of init.

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        // gets the parameter key from m_params map (read-only is allowed, since we only want to get
        // the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        // put your switch-case.. for getting the right value here

        // finally, save the desired value in the argument val (this is a shared pointer!)
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
ito::RetVal QuantumComposer::setParam(
    QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    // parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        // gets the parameter key from m_params map (read-only is not allowed and leads to
        // ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        // here the new parameter is checked whether its type corresponds or can be cast into the
        //  value in m_params and whether the new type fits to the requirements of any possible
        //  meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "mode")
        {
            retValue += SendCommand(
                QString(":PULSE0:MODE %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "gateMode")
        {
            retValue += SendCommand(
                QString(":PULSE0:GAT:MOD %1").arg(val->getVal<char*>()).toStdString().c_str());
            retValue += it->copyValueFrom(&(*val));
        }
        else if (key == "requestTimeOut")
        {
            m_requestTimeOutMS = val->getVal<int>();
            m_params["requestTimeOut"].setVal<int>(m_requestTimeOutMS);
            retValue += it->copyValueFrom(&(*val));
        }
        else
        {
            // all parameters that don't need further checks can simply be assigned
            // to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom(&(*val));
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(
            m_params); // send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::startDevice(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    // todo:
    //  if this function has been called for the first time (grabberStartedCount() == 1),
    //  start the camera, allocate necessary buffers or do other work that is necessary
    //  to prepare the camera for image acquisitions.

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::stopDevice(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::getVal(void* vpdObj, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject* dObj = reinterpret_cast<ito::DataObject*>(vpdObj);

    // call retrieveData without argument. Retrieve data should then put the currently acquired
    // image into the dataObject m_data of the camera. retValue += retrieveData();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::copyVal(void* vpdObj, ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject* dObj = reinterpret_cast<ito::DataObject*>(vpdObj);

    if (!dObj)
    {
        retValue += ito::RetVal(
            ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        // this method calls retrieveData with the passed dataObject as argument such that
        // retrieveData is able to copy the image obtained by the camera directly into the given,
        // external dataObject retValue += retrieveData(dObj);  //checkData is executed inside of
        // retrieveData
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::SendCommand(const QByteArray& command)
{
    ito::RetVal retVal;
    retVal += m_pSer->setVal(command.data(), command.length(), nullptr);
    if (m_delayAfterSendCommandMS > 0)
    {
        QMutex mutex;
        mutex.lock();
        QWaitCondition waitCondition;
        waitCondition.wait(&mutex, m_delayAfterSendCommandMS);
        mutex.unlock();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::ReadString(QByteArray& result, int& len, int timeoutMS)
{
    ito::RetVal retValue = ito::retOk;
    QElapsedTimer timer;

    bool done = false;
    int curFrom = 0;
    int pos = 0;

    int buflen = 100;
    QSharedPointer<int> curBufLen(new int);
    QSharedPointer<char> curBuf(new char[buflen]);
    result = "";

    QByteArray endline;

    QSharedPointer<ito::Param> param(new ito::Param("endline"));
    retValue += m_pSer->getParam(param, nullptr);

    if (param->getType() == (ito::ParamBase::String & ito::paramTypeMask))
    {
        char* temp = param->getVal<char*>(); // borrowed reference
        int len = temp[0] == 0 ? 0 : (temp[1] == 0 ? 1 : (temp[2] == 0 ? 2 : 3));
        endline = QByteArray::fromRawData(temp, len);
    }
    else
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("could not read endline parameter from serial port").toLatin1().data());
    }

    if (!retValue.containsError())
    {
        len = 0;
        timer.start();
        _sleep(m_delayAfterSendCommandMS); // The amount of time required to receive, process, and
                                           // repond to a command is
                     // approximately 10ms.

        while (!done && !retValue.containsError())
        {
            *curBufLen = buflen;
            retValue += m_pSer->getVal(curBuf, curBufLen, nullptr);

            if (!retValue.containsError())
            {
                result += QByteArray(curBuf.data(), *curBufLen);
                pos = result.indexOf(endline, curFrom);
                curFrom = qMax(0, result.length() - 3);

                if (pos >= 0) // found
                {
                    done = true;
                    result = result.mid(pos + endline.length(), curFrom);
                }

                if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        timeoutMS,
                        tr("timeout during read string from SerialIO").toLatin1().data());
                }
            }
        }
        result = result.trimmed();
        len = result.length();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal QuantumComposer::SendQuestionWithAnswerString(
    const QByteArray& questionCommand, QByteArray& answer, const int timeoutMS)
{
    int readSigns;
    ito::RetVal retValue = SendCommand(questionCommand);
    retValue += ReadString(answer, readSigns, timeoutMS);

    if (retValue.containsError())
    {
        std::cout << "Error during SendQuestionWithAnswerString" << std::endl;
    }

    return retValue;
}
