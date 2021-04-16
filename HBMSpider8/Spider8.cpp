/* ********************************************************************
    Plugin "Spider8" for itom software
    URL: http://lccv.ufal.br/
    Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil

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

#include "Spider8.h"
#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include "DataObject/dataobj.h"
#include <qvarlengtharray.h>

#ifdef WIN32
#include <Windows.h>
#endif

int NTHREADS = 2;

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(spider8interface, Spider8Interface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif

//#include "dockWidgetniDAQmx.h"


//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/
Spider8Interface::Spider8Interface()
{
    m_type = ito::typeDataIO | ito::typeADDA; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("HBMSpider8");

    m_description = QObject::tr("HBMSpider8");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"The plugin implements the Spider8 functions for the analog-digital-converter from HBM. \n\
The installation needs an initialized serial port";
    m_detaildescription = QObject::tr(docstring);

    m_author = "Christian Kohler, Uniersidade Federal de Alagoas (UFAL)";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(""); 
    
    m_initParamsMand.clear();
    ito::Param paramVal("SerialIO", ito::ParamBase::HWRef, NULL, tr("Open com-port where Spider8 device is connected").toLatin1().data());
    m_initParamsMand.append(paramVal);
   
//    ito::Param paramVal("device", ito::ParamBase::String, "Dev1", tr("Name of the target Device, Dev1 as default, cDAQ1Mod1 for compactDAQ single module Device. Other names see device description in NI MAX").toLatin1().data());
//    m_initParamsOpt.append(paramVal);
    m_initParamsOpt.clear();
    NTHREADS = QThread::idealThreadCount();
    paramVal = ito::Param("baud", ito::ParamBase::Int, 9600, 76800, 57600, tr("baud rate for hbm").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
    Spider8Interface::~Spider8Interface()
{

}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Interface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(Spider8) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Interface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(Spider8) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetErrStr(const int errNum, QString &errMsg)
{
    ito::RetVal retval(ito::retOk);

    switch (errNum)
    {
        case 0:
            errMsg.sprintf(QObject::tr("ok (0)").toLatin1().data());
        break;
        case 10000:
            errMsg.sprintf(QObject::tr("reset (10000)").toLatin1().data());
        break;
        case 10001:
            errMsg.sprintf(QObject::tr("parity error (10001)").toLatin1().data());
        break;
        case 10002:
            errMsg.sprintf(QObject::tr("system error (10002)").toLatin1().data());
        break;
        case 10003:
            errMsg.sprintf(QObject::tr("unknown command (10003)").toLatin1().data());
        break;
        case 10004:
            errMsg.sprintf(QObject::tr("wrong number of parameters (10004)").toLatin1().data());
        break;
        case 10005:
            errMsg.sprintf(QObject::tr("wrong parameter value (10005)").toLatin1().data());
        break;
        case 10006:
            errMsg.sprintf(QObject::tr("error with filter frequency (10006)").toLatin1().data());
        break;
        case 10007:
            errMsg.sprintf(QObject::tr("amplifier error (10007)").toLatin1().data());
        break;
        case 10008:
            errMsg.sprintf(QObject::tr("command (currently) not executable (10008)").toLatin1().data());
        break;
        case 10009:
            errMsg.sprintf(QObject::tr("error with OMB (10009)").toLatin1().data());
        break;
        case 10010:
            errMsg.sprintf(QObject::tr("error with channel selection (10010)").toLatin1().data());
        break;
        case 10011:
            errMsg.sprintf(QObject::tr("error when measuring (10011)").toLatin1().data());
        break;
        case 10012:
            errMsg.sprintf(QObject::tr("error when triggering (10012)").toLatin1().data());
        break;
        case 10013:
            errMsg.sprintf(QObject::tr("error with measuring range (10013)").toLatin1().data());
        break;
        case 10014:
            errMsg.sprintf(QObject::tr("error when taring (10014)").toLatin1().data());
        break;
        case 10015:
            errMsg.sprintf(QObject::tr("master device has lost contact to slave (10015)").toLatin1().data());
        break;
        case 10020:
            errMsg.sprintf(QObject::tr("Spider8 is in flash update-mode (10020)").toLatin1().data());
        break;
        case 10021:
            errMsg.sprintf(QObject::tr("Warning due to filter frequency (10021)").toLatin1().data());
        break;
        case 10022:
            errMsg.sprintf(QObject::tr("Warning due to tare status (10022)").toLatin1().data());
        break;
        default:
            errMsg.sprintf(QObject::tr("unknown error (%d)").toLatin1().data(), errNum);
            retval = ito::RetVal(ito::retError, 0, QObject::tr("unknown error %1").arg(errNum).toLatin1().data());            
        break;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetError(QString &err, int &errNum)
{
    // retrieving Spider8 error. 
    ito::RetVal retValue(ito::retOk);
    int maxlen = 255;
    char cmdBuf[20] = "";

    retValue += m_pSer->setVal("EST?", 4, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, 0);
    errNum = atoi(m_readBuf.data());
    retValue += hbmGetErrStr(errNum, err);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
* Read hbm status. Either of the global status if no channel is passed or the specific channel status
*
* status can be one of the following values:
* 0  no valid measured values available
* 1  Spider8 is in a waiting phase, i.e. transients caused by switching to a different measuring range have not yet decayed.
* 2  the pre-trigger buffer is being filled
* 3  the pre-trigger buffer is full; waiting for trigger event
* 4  the post-trigger buffer is being filled
* 5  the acquisition terminated with error
* 6  the acquisition terminated without error
* 7  channel is passive
* 8  channel measure enabled
* 9  channel tare enabled
* 10 channel measure and tare enabled
*/
ito::RetVal Spider8Funcs::hbmGetStatus(int &status, const int channel = -1)
{
    // retrieving Spider8 status. If no channel number is passed the general status
    // is queried otherwise we check the specified channel using either MSV or ACT 
    // command
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[20] = "";

    if (channel < 0 || channel > 99)
    {
        retValue += m_pSer->setVal("MSV?", 4, NULL);
        Sleep(m_rdSleep);
        *m_readBufLen = 255;
        memset(m_readBuf.data(), 0, 255);
        retValue += m_pSer->getVal(m_readBuf, m_readBufLen, 0);
        QStringList resLst = QString::fromLatin1(m_readBuf.data()).split(",");
        status = resLst[0].toInt();

        if (status < 0 || status > 6)
        {
            QString err;
            int errNum;
            retValue += hbmGetError(err, errNum);
            retValue += ito::RetVal(ito::retError, 0, QObject::tr("Received unknown status: %1, %2").arg(QString::number(status), err).toLatin1().data());
        }
    }
    else
    {
        sprintf(cmdBuf, "ACT?%d", channel);
        retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
        Sleep(m_rdSleep);
        *m_readBufLen = 255;
        memset(m_readBuf.data(), 0, 255);
        retValue += m_pSer->getVal(m_readBuf, m_readBufLen, 0);
        switch (atoi(m_readBuf.data()))
        {
            // channel is passive
            case 6200:
                status = 7;
            break;
            // channel measure enabled
            case 6201:
                status = 8;
            break;
            // channel tare enabled
            case 6202:
                status = 9;
            break;
            // channel measure and tare enabled
            case 6203:
                status = 10;
            break;
            // we do not recognize the result so return an error
            default:
            {
                QString err;
                int errNum;
                retValue += hbmGetError(err, errNum);
                retValue += ito::RetVal(ito::retError, 0, QObject::tr("Received unknown status: %1, %2").arg(QString(m_readBuf.data()), err).toLatin1().data());
            }
            break;
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmSetBaud(const int baud)
{
    ito::RetVal retValue(ito::retOk);
    QString err;
    int errNum;
    char cmdBuf[20] = "";
    int baudSet = 1408;

    switch (baud)
    {
        case 600:
            baudSet = 1404;
        break;
        case 1200:
            baudSet = 1405;
        break;
        case 2400:
            baudSet = 1406;
        break;
        case 4800:
            baudSet = 1407;
        break;
        case 9600:
            baudSet = 1408;
        break;
        case 19200:
            baudSet = 1410;
        break;
        case 38400:
            baudSet = 1412;
        break;
        case 57600:
            baudSet = 1414;
        break;
        case 76800:
            baudSet = 1415;
        break;
        default:
            baudSet = 1408;
            retValue += ito::RetVal(ito::retWarning, 0, QObject::tr("unknown baud rate %1 passed, defaulting to 9600 baud").arg(baud).toLatin1().data());
        break;
    }
    sprintf(cmdBuf, "BDR%d", baudSet);
    retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    // added some extra delay here as well due to random startup errors
    Sleep(m_rdSleep * 3);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) != 0)
    {
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Received unknown status: %1, %2").arg(m_readBuf.data(), err).toLatin1().data());
    }
    else
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, baud)), NULL);

    //after setting baud rate a reset is performed and we need to run EST?
    // we loop a little here as the answers returning from the Spider8 are little bit confusion.
    // first we get a ?, then a 10001 (parity error) then everyting is fine ...
    for (int nt = 0; nt < 3; nt++)
    {
        retValue += m_pSer->setVal("EST?", 4, NULL);
        Sleep(m_lRdSleep);
        *m_readBufLen = 255;
        memset(m_readBuf.data(), 0, 255);
        retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    }
    // something really went wrong so what is the matter?
    if (atoi(m_readBuf.data()) != 10000 && atoi(m_readBuf.data()) != 0)
    {
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Received unknown status: %1, %2").arg(m_readBuf.data(), err).toLatin1().data());
    }
    else if (baud >= 57600)
        m_rdSleep = m_sRdSleep;

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetBaud(int &baud)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[20] = "";
    int baudSet = 1408;

    retValue += m_pSer->setVal("BDR?", 4, NULL);
    Sleep(m_lRdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) < 1404 || atoi(m_readBuf.data()) > 1415)
    {
        QString err;
        int errNum;
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Received unknown status: %1, %2").arg(m_readBuf.data(), err).toLatin1().data());
    }

    switch (atoi(m_readBuf.data()))
    {
        case 1404:
            baudSet = 600;
        break;
        case 1405:
            baudSet = 1200;
        break;
        case 1406:
            baudSet = 2400;
        break;
        case 1407:
            baudSet = 4800;
        break;
        case 1408:
            baudSet = 9600;
        break;
        case 1410:
            baudSet = 19200;
        break;
        case 1412:
            baudSet = 38400;
        break;
        case 1414:
            baudSet = 57600;
        break;
        case 1415:
            baudSet = 76800;
        break;
        default:
            retValue += ito::RetVal(ito::retWarning, 0, QObject::tr("unknown baud rate %1 returned").arg(baud).toLatin1().data());
        break;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmStop(void)
{
    ito::RetVal retValue(ito::retOk);

    retValue += m_pSer->setVal("STP", 3, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) != 0)
    {
        QString err;
        int errNum;
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Received unknown status: %1, %2").arg(m_readBuf.data(), err).toLatin1().data());
    }
    m_activeChs.clear();

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmListDevices(QVector<int> &devices, QVector<QString> *devNames = NULL)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[10] = "";
    devices.clear();
    for (int n = 0; n < 8; n++)
    {
        sprintf(cmdBuf, "IDN?%d", n);
        retValue += m_pSer->setVal(cmdBuf, 5, NULL);
        Sleep(m_rdSleep);
        *m_readBufLen = 255;
        memset(m_readBuf.data(), 0, 255);
        retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
        
        // TODO: check what is returned when device does not exist
        QStringList ident = QString::fromLatin1(m_readBuf.data()).split(",");
        if (ident.length() > 0)
        {
            devices.append(n);
            if (devNames)
            {
                devNames->append(m_readBuf.data());
            }
        }
    }
    return(retValue);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmListChannels(QMap<int, Spider8Channel> &channels)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[20] = "";
    channels.clear();
    
    retValue += m_pSer->setVal("AID?", 4, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (retValue.containsWarningOrError())
    {
        QString err;
        int errNum;
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Error reading channel list %1").arg(err).toLatin1().data());
        return retValue;
    }
    QStringList chs = QString::fromLatin1(m_readBuf.data()).split(",");
    if (chs.length() < 2)
    {
        QString err;
        int errNum;
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Error reading channel list %1").arg(err).toLatin1().data());
        return retValue;

    }
    for (int n = 0; n < chs.length(); n = n + 2)
    {
        // read channel types
        int chaNum = atoi(chs[n].toLatin1().data());
//        sprintf(cmdBuf, "AID?%d", chaNum);
//        retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
//        Sleep(m_rdSleep);
//        *m_readBufLen = 255;
//        memset(m_readBuf.data(), 0, 255);
//        retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
//        if (retValue.containsWarningOrError())
//        {
//            QString err;
//            retValue += hbmGetError(err);
//            retValue += ito::RetVal(ito::retError, 0, QObject::tr("Error reading channel %1, %2").arg(chs[n], err).toLatin1().data());
//            return retValue;
//        }
        int type = atoi(chs[n + 1].toLatin1().data());
        if (type > 5050 && type <= 5058)
        {
            sprintf(cmdBuf, "ASA?%d,?", chaNum);
            retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
            Sleep(m_rdSleep);
            *m_readBufLen = 255;
            memset(m_readBuf.data(), 0, 255);
            retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
            if (retValue.containsWarningOrError())
            {
                QString err;
                int errNum;
                retValue += hbmGetError(err, errNum);
                retValue += ito::RetVal(ito::retError, 0, QObject::tr("Error reading channel %1, %2").arg(chs[n], err).toLatin1().data());
                return retValue;
            }
            QStringList modes = QString(m_readBuf.data()).split(",");
            QMap<int, QVector<int> > mrVec;
            for (int nm = 0; nm < modes.length(); nm++)
            {
                int mode = atoi(modes[nm].toLatin1().data());
                sprintf(cmdBuf, "ASA?%d,%d,?", chaNum, mode);
                retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
                Sleep(m_rdSleep);
                *m_readBufLen = 255;
                memset(m_readBuf.data(), 0, 255);
                retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
                if (retValue.containsWarningOrError())
                {
                    QString err;
                    int errNum;
                    retValue += hbmGetError(err, errNum);
                    retValue += ito::RetVal(ito::retError, 0, QObject::tr("Error reading channel %1, %2").arg(chs[n], err).toLatin1().data());
                    return retValue;
                }
                QStringList ranges = QString(m_readBuf.data()).split(",");
                QVector<int> rVec;
                for (int nr = 0; nr < ranges.length(); nr++)
                {
                    rVec.append(atoi(ranges[nr].toLatin1().data()));
                }
                mrVec.insert(mode, rVec);
            }
            channels.insert(chaNum, Spider8Channel(chaNum, type, mrVec));
        }
        else
        {
            QString err;
            int errNum;
            retValue += hbmGetError(err, errNum);
            retValue += ito::RetVal(ito::retError, 0, QObject::tr("Error reading channel %1, %2").arg(chs[n], err).toLatin1().data());
        }
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetShunt(const int channel, bool &value)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[10] = "";

    sprintf(cmdBuf, "ASS?%d", channel);
    retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) == 42)
        value = 0;
    else if (atoi(m_readBuf.data()) == 43)
        value = 1;
    else
    {
        QString err;
        int errNum;
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Error reading channel shunt status, %1, %2").arg(QString(m_readBuf.data()), err).toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmSetShunt(const int channel, const bool value)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[10] = "";

    if (value)
        sprintf(cmdBuf, "ASS%d,43", channel); //!< shunt on
    else
        sprintf(cmdBuf, "ASS%d,42", channel); //!< shunt off
    retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    
    if (atoi(m_readBuf.data()) != 0)
    {
        QString errMsg;
        retValue += hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
        retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmSetMemMngmnt(const bool firstReadClear)
{
    char cmdBuf[10] = "";
    ito::RetVal retValue(ito::retOk);

    sprintf(cmdBuf, "FMB%d", firstReadClear);
    retValue += m_pSer->setVal(cmdBuf, 4, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) != 0)
    {
        QString errMsg;
        retValue += hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
        retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetMemMngmnt(bool &firstReadClear)
{
    ito::RetVal retValue(ito::retOk);
    retValue += m_pSer->setVal("FMB?", 4, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) < 0 || atoi(m_readBuf.data()) >1)
    {
        QString err;
        int errNum;
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("got unknown memory management setting: %1, %2").arg(m_readBuf.data(), err).toLatin1().data());
    }
    else
        firstReadClear = atoi(m_readBuf.data());

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmSetSamplingRate(double &rate)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[10] = "";
    int rateVal = 6315;

    // 1 Hz
    if (rate <= 1.0) { rate = 1.0; rateVal = 6300; }
    // 2 Hz
    else if (rate <= 2.0) { rate = 2.0; rateVal = 6301; }
    // 5 Hz
    else if (rate <= 5.0) { rate = 5.0; rateVal = 6302; }
    // 10 Hz
    else if (rate <= 10.0) { rate = 10.0; rateVal = 6303; }
    // 25 Hz
    else if (rate <= 25.0) { rate = 25.0; rateVal = 6304; }
    // 50 Hz
    else if (rate <= 50.0) { rate = 50.0; rateVal = 6305; }
    // 60 Hz
    else if (rate <= 60.0) { rate = 60.0; rateVal = 6306; }
    // 75 Hz
    else if (rate <= 75.0) { rate = 75.0; rateVal = 6307; }
    // 100 Hz
    else if (rate <= 100.0) { rate = 100.0; rateVal = 6308; }
    // 150 Hz
    else if (rate <= 150.0) { rate = 150.0; rateVal = 6309; }
    // 200 Hz
    else if (rate <= 200.0) { rate = 200.0; rateVal = 6310; }
    // 300 Hz
    else if (rate <= 300.0) { rate = 300.0; rateVal = 6311; }
    // 400 Hz
    else if (rate <= 400.0) { rate = 400.0; rateVal = 6312; }
    // 600 Hz
    else if (rate <= 600.0) { rate = 600.0; rateVal = 6313; }
    // 800 Hz
    else if (rate <= 800.0) { rate = 800.0; rateVal = 6314; }
    // 1200 Hz (default)
    else if (rate <= 1200.0) { rate = 1200.0; rateVal = 6315; }
    // 1600 Hz
    else if (rate <= 1600.0) { rate = 1600.0; rateVal = 6316; }
    // 2400 Hz
    else if (rate <= 2400.0) { rate = 2400.0; rateVal = 6317; }
    // 3200 Hz
    else if (rate <= 3200.0) { rate = 3200.0; rateVal = 6318; }
    // 4800 Hz
    else if (rate <= 4800.0) { rate = 4800.0; rateVal = 6319; }
    // 9600 Hz
    else { rate = 9600; rateVal = 6320; }
    sprintf(cmdBuf, "ICR%d", rateVal);
    retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);

    if (atoi(m_readBuf.data()) != 0)
    {
        QString errMsg;
        QString err;
        int errNum;
        retValue += hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetSamplingRate(double &rate)
{
    ito::RetVal retValue(ito::retOk);
    retValue += m_pSer->setVal("ICR?", 4, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);

    switch (atoi(m_readBuf.data()))
    {
        case 6300:
            rate = 1.0;
        break;
        case 6301:
            rate = 2.0;
        break;
        case 6302:
            rate = 5.0;
        break;
        case 6303:
            rate = 10.0;
        break;
        case 6304:
            rate = 25.0;
        break;
        case 6305:
            rate = 50.0;
        break;
        case 6306:
            rate = 60.0;
        break;
        case 6307:
            rate = 75.0;
        break;
        case 6308:
            rate = 100.0;
        break;
        case 6309:
            rate = 150.0;
        break;
        case 6310:
            rate = 200.0;
        break;
        case 6311:
            rate = 300.0;
        break;
        case 6312:
            rate = 400.0;
        break;
        case 6313:
            rate = 600.0;
        break;
        case 6314:
            rate = 800.0;
        break;
        case 6315:
            rate = 1200.0;
        break;
        case 6316:
            rate = 1600.0;
        break;
        case 6317:
            rate = 2400.0;
        break;
        case 6318:
            rate = 4800.0;
        break;
        case 6319:
            rate = 9600.0;
        break;
        default:
        {
            rate = -1.0;
            QString err;
            int errNum;
            retValue += hbmGetError(err, errNum);
            retValue += ito::RetVal(ito::retError, 0, QObject::tr("Received unknown rate value: %1, %2").arg(m_readBuf.data(), err).toLatin1().data());
        }
        break;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetTrigger(int &channel, int &mode, int &threshold)
{
    ito::RetVal retValue(ito::retOk);

    retValue += m_pSer->setVal("DTR?", 4, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    QStringList trgRes = QString::fromLatin1(m_readBuf.data()).split(",");
    if (trgRes.length() < 3)
    {
        QString err;
        int errNum;
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unexpected result reading trigger mode: %1, %2").arg(m_readBuf.data(), err).toLatin1().data());
    }
    else
    {
        channel = trgRes[0].toInt();
        mode = trgRes[1].toInt() - 6000;
        threshold = trgRes[2].toInt();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmSetTrigger(const int channel, const int mode, const int threshold = 0)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[20] = "";

    if (channel < 0 || channel > 7 || mode < 0 || mode > 3 || threshold < -32769 || threshold > 32767)
    {
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Parameter value out of range:\n\tchannel: %1 [0 ... 7]\n\tmode: %2 [0 ... 3]\n\tlevel: %3 [-32769 ... 32767]\n").arg(channel, mode, threshold).toLatin1().data());
    }
    else
    {
        sprintf(cmdBuf, "DTR%d,%d,%d", channel, mode, threshold);
        retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
        Sleep(m_rdSleep);
        *m_readBufLen = 255;
        memset(m_readBuf.data(), 0, 255);
        retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
        if (atoi(m_readBuf.data()) != 0)
        {
            QString err;
            int errNum;
            QString errMsg;
            retValue += hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
            retValue += hbmGetError(err, errNum);
            retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmCheckChModeRange(Spider8Channel &channel, const int mode, const int range, bool &check)
{
    ito::RetVal retval(ito::retError);
    check = false;

    if (channel.m_allowedModRanges.contains(mode + 350))
    {
        QVector<int> modes = channel.m_allowedModRanges[mode + 350];
        if (modes.contains(range + 700))
        {
            check = true;
            retval = ito::retOk;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmSetChConfig(Spider8Channel &channel, const int num, const int mode, const int range)
{
    ito::RetVal retValue(ito::retOk);
    char cmdBuf[20];

    sprintf(cmdBuf, "ASA%d,%d,%d", num, mode + 350, range + 700);
    retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) != 0)
    {
        QString err;
        QString errMsg;
        int errNum;
        retValue += hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }
    else
    {
        channel.m_mode = mode;
        channel.m_range = range;
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmStopAcq(QStringList &channels)
{
    ito::RetVal retval(ito::retOk);

    if (channels.length() < 1)
    {
        retval += m_pSer->setVal("ACT100,6200", 11, NULL);
        Sleep(m_rdSleep);
        *m_readBufLen = 255;
        memset(m_readBuf.data(), 0, 255);
        retval += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
        if (atoi(m_readBuf.data()) != 0)
        {
            QString errMsg;
            QString err;
            int errNum;
            retval += hbmGetError(err, errNum);
            hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
            retval += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
        }
    }
    else
    {
        char cmdBuf[15];

        for (int nc = 0; nc < channels.length(); nc++)
        {
            sprintf(cmdBuf, "ACT%d,6200", m_activeChs[nc]);
            retval += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
            Sleep(m_rdSleep);
            *m_readBufLen = 255;
            memset(m_readBuf.data(), 0, 255);
            retval += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
            if (atoi(m_readBuf.data()) != 0)
            {
                QString errMsg;
                QString err;
                int errNum;
                retval += hbmGetError(err, errNum);
                hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
                retval += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
            }
        }
    }
    m_activeChs.clear();

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmActivateCh(const QMap<int, Spider8Channel> &channels, const int &chnum, const int &chactmode)
{
    ito::RetVal retval(ito::retOk);

    if (chnum == -1)
    {
        char cmdBuf[20];
        for (int nc = 0; nc < m_activeChs.size(); nc++)
        {
//            int chactmode = channels[nc].m_measMode;
            sprintf(cmdBuf, "ACT%d,%d", m_activeChs[nc], chactmode);
            retval += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
            Sleep(m_rdSleep);
            *m_readBufLen = 255;
            memset(m_readBuf.data(), 0, 255);
            retval += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
            if (atoi(m_readBuf.data()) != 0)
            {
                QString errMsg;
                QString err;
                int errNum;
                retval += hbmGetError(err, errNum);
                hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
                retval += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
            }
        }
    }
    else
    {
        char cmdBuf[20];
        sprintf(cmdBuf, "ACT%d,%d", chnum, chactmode);
        retval += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
        Sleep(m_rdSleep);
        *m_readBufLen = 255;
        memset(m_readBuf.data(), 0, 255);
        retval += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
        if (atoi(m_readBuf.data()) != 0)
        {
            QString errMsg;
            QString err;
            int errNum;
            retval += hbmGetError(err, errNum);
            hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
            retval += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmStartMeas(const int trigger, const int numSamp, const int cycles, const int preTrgSamples)
{
    ito::RetVal retval(ito::retOk);

    char cmdBuf[30];
    int runcycles = cycles;
    // according to hdm documentation cycles must be 1 for a continuous measurement
    if (numSamp == 0)
        runcycles = 1;
    sprintf(cmdBuf, "MSV%d,%d,%d,%d", numSamp, preTrgSamples, trigger, runcycles);
    retval += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retval += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    if (atoi(m_readBuf.data()) != 0)
    {
        QString errMsg;
        QString err;
        int errNum;
        retval += hbmGetError(err, errNum);
        hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
        retval += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmGetNumActChs(int &numChs)
{
    ito::RetVal retValue(ito::retOk);

    retValue += m_pSer->setVal("CCT?6210", 8, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    numChs = atoi(m_readBuf.data());
    if (numChs > 8 || numChs < 0)
    {
        QString err;
        QString errMsg;
        int errNum;
        retValue += hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
* status can be one of the following values:
* 0 no valid measured values available
* 1 Spider8 is in a waiting phase, i.e. transients caused by switching to a different measuring range have not yet decayed.
* 2 the pre-trigger buffer is being filled
* 3 the pre-trigger buffer is full; waiting for trigger event
* 4 the post-trigger buffer is being filled
* 5 the acquisition terminated with error
* 6 the acquisition terminated without error
*/
ito::RetVal Spider8Funcs::hbmGetNumSamples(int &status, int &numSamples)
{
    ito::RetVal retValue(ito::retOk);

    retValue += m_pSer->setVal("MSV?", 4, NULL);
    Sleep(m_rdSleep);
    *m_readBufLen = 255;
    memset(m_readBuf.data(), 0, 255);
    retValue += m_pSer->getVal(m_readBuf, m_readBufLen, NULL);
    QStringList statusStr = QString(m_readBuf.data()).split(",");
    if (atoi(statusStr[0].toLatin1().data()) < 0 || atoi(statusStr[0].toLatin1().data()) > 6 || statusStr.length() < 2)
    {
        QString err;
        QString errMsg;
        int errNum;
        retValue += hbmGetErrStr(atoi(m_readBuf.data()), errMsg);
        retValue += hbmGetError(err, errNum);
        retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }
    else
    {
        numSamples = atoi(statusStr[1].toLatin1().data());
        status = atoi(statusStr[0].toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmReadData(QSharedPointer<char> dPtr, QSharedPointer<int> dSize, const int numValues)
{
    ito::RetVal retValue(ito::retOk);

    char cmdBuf[15];
    sprintf(cmdBuf, "OMB?%d", numValues);
    retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    Sleep(m_rdSleep);
    retValue += m_pSer->getVal(dPtr, dSize, NULL);
    if (dPtr.data()[0] != '#' || dPtr.data()[1] != '0')
    {
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("received wrong identifier while reading data").toLatin1().data());
        QString errMsg;
        int errNum;
        retValue += hbmGetError(errMsg, errNum);
        retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8Funcs::hbmReadNScale(const int numChs, const int samples, double *iscales, double *voffsets, double *vscales, ito::DataObject *dataObj)
{
    ito::RetVal retValue(ito::retOk);
    int numValues = samples * numChs * 2 + 4;
    int values2Read = numValues;
    int samplesRead = 0;
    int samplesCnvrt = 0;
    QSharedPointer<char> dataBuf(new char[numValues]);
    QSharedPointer<int> dataBufSize(new int);
    // we double buffer here, I tried messing around with incomplete word reads, but the thing gets quite messy as 
    // Spider is really slooow. So I opted for a double buffering here. For the sake of less messy code and a little
    // higher memory use.
    ito::uint8 *localBuf = (ito::uint8*)malloc(numValues);
    ito::uint8 *localBufPtr = localBuf;
    ito::uint16 *localBufCnvrtPtr = (ito::uint16*)(localBuf + 2);
    // for first reading user data starts after preamble (#0) so we have remove that for value calculation
    ito::uint16 *dPtr = (ito::uint16*)&(dataBuf.data()[2]);
    double *doPtr[8];
    char cmdBuf[25], readRep = 0, totRep = 0;
    int dStartIdx = m_lastIdxRead;

    for (int nc = 0; nc < numChs; nc++)
    {
        doPtr[nc] = (double*)dataObj->rowPtr(0, nc);
    }
    sprintf(cmdBuf, "OMB?%d", samples);
    retValue += m_pSer->setVal(cmdBuf, strlen(cmdBuf), NULL);
    Sleep(m_rdSleep);
    while (values2Read > 0 && totRep < 100)
    {
        *dataBufSize = numValues;
        retValue += m_pSer->getVal(dataBuf, dataBufSize, NULL);
/*
        readRep = 0;
        do
        {
            *dataBufSize = numValues;
            retValue += m_pSer->getVal(dataBuf, dataBufSize, NULL);
            if (*dataBufSize == 0)
            {
                Sleep(m_rdSleep);
            }
            readRep++;
        } while (*dataBufSize == 0 && readRep < 10);
        memcpy((ito::uint8*)localBufPtr, dataBuf.data(), *dataBufSize);
        localBufPtr += *dataBufSize;
*/
        if (*dataBufSize > 0)
        {
            memcpy((ito::uint8*)localBufPtr, dataBuf.data(), *dataBufSize);
            localBufPtr += *dataBufSize;

            // on first read check if we are receiving a binary data stream opened by #0
            if ((numValues == values2Read) && (dataBuf.data()[0] != '#' || dataBuf.data()[1] != '0'))
            {
                retValue += ito::RetVal(ito::retError, 0, QObject::tr("received wrong identifier while reading data").toLatin1().data());
                QString errMsg;
                int errNum;
                retValue += hbmGetError(errMsg, errNum);
                retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
                goto end;
            }
            // did we read more data than expected, than return an error
            if (values2Read - *dataBufSize < 0)
            {
                retValue += ito::RetVal(ito::retError, 0, QObject::tr("received more data than requested").toLatin1().data());
                QString errMsg;
                int errNum;
                retValue += hbmGetError(errMsg, errNum);
                retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
                goto end;
            }

            samplesCnvrt = ((ito::uint16*)localBufPtr - localBufCnvrtPtr) / numChs;
            samplesCnvrt = samplesRead + samplesCnvrt <= samples ? samplesCnvrt : samples - samplesRead;

#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
#pragma omp for schedule(guided)
#endif
            for (int ns = 0; ns < samplesCnvrt; ns++)
            {
                for (int nc = 0; nc < numChs; nc++)
                {
                    // scale to voltage according to selected measurement range
                    doPtr[nc][dStartIdx + samplesRead + ns] = localBufCnvrtPtr[ns * numChs + nc] > 32767 ? (localBufCnvrtPtr[ns * numChs + nc] - 65536) * iscales[nc] : localBufCnvrtPtr[ns * numChs + nc] * iscales[nc];
                    // apply calibration and scale to physical values
                    doPtr[nc][dStartIdx + samplesRead + ns] += voffsets[nc];
                    doPtr[nc][dStartIdx + samplesRead + ns] *= vscales[nc];
                }
            }

            values2Read -= *dataBufSize;
            samplesRead += samplesCnvrt;
            localBufCnvrtPtr += samplesCnvrt * numChs;
        }
        totRep++;
        if (values2Read)
            Sleep(m_rdSleep);
    }
    m_lastIdxRead += samplesRead;

end:
    if (localBuf)
        free(localBuf);
    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
Spider8::Spider8() : AddInDataIO(), 
    m_isgrabbing(false),
    m_pSer(NULL),
    m_aInIsAcquired(false),
    m_dInIsAcquired(false),
    m_dOutIsAcquired(false),
    m_pSpider(NULL),
    m_baud(57600)
{
    ito::Param paramVal("device", ito::ParamBase::String | ito::ParamBase::In | ito::ParamBase::Readonly, "Dev1", tr("Name of the target device").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    // General Parameters
    paramVal = ito::Param("name", ito::ParamBase::String | ito::ParamBase::Readonly, "HBMSpider8", NULL);    
    m_params.insert(paramVal.getName(), paramVal);
//    paramVal = ito::Param("channel", ito::ParamBase::String | ito::ParamBase::Readonly, "", NULL);    
//    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("channelList", ito::ParamBase::String | ito::ParamBase::Readonly, "returns a list with the available channels, modes and measurement ranges", NULL);    
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("actChannelList", ito::ParamBase::String | ito::ParamBase::Readonly, "returns a list of the currently activated channels", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("samplingRate", ito::ParamBase::Double | ito::ParamBase::In, 1.0, 9600.0, 1200.0, tr("sampling rate for measurement").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger", ito::ParamBase::String | ito::ParamBase::In, "", tr("trigger for starting measurement with trigger condition ( acquire(3) ) [Channel,Mode,Level], Mode - 0: above level, 1: below level, positive edge, negative edge, leve: -32769 ... 32767").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bufferManagement", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("0: clear data on second read out, 1: clear data on first read out").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numSamples", ito::ParamBase::Int | ito::ParamBase::In, 1, 2000000000, 1200, tr("number of samples for one measurment cycle").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("preTrgSamples", ito::ParamBase::Int | ito::ParamBase::In, 1, 500, 1, tr("number of samples recorded before trigger is active").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("numCycles", ito::ParamBase::Int | ito::ParamBase::In, 0, 2000000000, 1, tr("number of measurement cycles, 0: infinite").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("reset", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("reset error state").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("statusStr", ito::ParamBase::String | ito::ParamBase::Readonly, "", tr("read status").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("status", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 6, 0, tr("read status").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    double *offsets = (double*)calloc(8, sizeof(double));
    paramVal = ito::Param("offset", ito::ParamBase::DoubleArray, 8, offsets, tr("array with offsets for each channel, default is 0.0; offset first").toLatin1().data());
    paramVal.setAutosave(1);
    m_params.insert(paramVal.getName(), paramVal);
    double *scales = (double*)calloc(8, sizeof(double));
    for (int nc = 0; nc < 8; nc++) scales[nc] = 1.0;
    paramVal = ito::Param("scale", ito::ParamBase::DoubleArray, 8, scales, tr("array with scales for each channel, default is 1.0; offset first").toLatin1().data());
    paramVal.setAutosave(1);
    m_params.insert(paramVal.getName(), paramVal);

    // Channel Parameters
    paramVal = ito::Param("aiChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for analog input channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("diChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for digital input channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("doChParams", ito::ParamBase::String | ito::ParamBase::In, "", tr("Set Parameterset for digital output channel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
Spider8::~Spider8()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (reinterpret_cast<ito::AddInBase *>((*paramsMand)[0].getVal<void *>())->getBasePlugin()->getType() & (ito::typeDataIO | ito::typeRawIO))
    {
        m_pSer = (ito::AddInDataIO *)(*paramsMand)[0].getVal<void *>();
        // communication default is 9600 baud, 8 data bits, 1 stop bit, even parity
        // maximum listed speed is 76800 baud. After switch on 9600 baus is always possible.
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, 9600)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, 8)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 2.0)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, 1)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, 0)), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, "\r\n")), NULL);
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("debugIgnoreEmpty", ito::ParamBase::Int, 1)), NULL);
        // set to single character reading, buffer transfer seems not working
        // TODO: check if sendDelay is necessary
        retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sendDelay", ito::ParamBase::Int, 0)), NULL);
        // Note: according to hbm documentation the first command after power-on or reset must be EST?

        QSharedPointer<char> readBuf(new char[255]);
        QSharedPointer<int> readBufLen(new int);
        *readBufLen = 255;
        retValue += m_pSer->setVal("EST?", 4, NULL);
        Sleep(20);
        retValue += m_pSer->getVal(readBuf, readBufLen, NULL);
        // we got a parity error ... so change parity
        QString outStr;
        int repCnt = 0;
        if (atoi(readBuf.data()) == 10001)
        {
            retValue += m_pSer->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Double, 0.0)), NULL);
            retValue += m_pSer->setVal("EST?", 4, NULL);
            // somewhat data does not come in one read sometimes ...
            do
            {
                Sleep(20);
                retValue += m_pSer->getVal(readBuf, readBufLen, NULL);
                outStr.append(readBuf.data());
                repCnt++;
            } while (outStr[outStr.length() - 1] != '\n' && repCnt < 10);
        }
        if (atoi(outStr.toLatin1().data()) != 10000 && atoi(outStr.toLatin1().data()) != 0)
        {
            QString errMsg;
            Spider8Funcs::hbmGetErrStr(atoi(readBuf.data()), errMsg);
            retValue += ito::RetVal(ito::retError, 0, errMsg.toLatin1().data());
        }
        else
        {
            m_pSpider = new Spider8Funcs(m_pSer);
            m_baud = (*paramsOpt)[0].getVal<int>();
            // now speed up communication
            retValue += m_pSpider->hbmSetBaud(m_baud);
        }
        // set all channels to inactive
        if (!retValue.containsWarningOrError())
        {
            retValue += m_pSpider->hbmStopAcq(QStringList());
        }
        // fill channel list
        if (!retValue.containsWarningOrError())
        {
            retValue += m_pSpider->hbmListChannels(m_channels);
            // fill m_param channel paramters
            QSharedPointer<ito::Param> p(new ito::Param("aiChParams", ito::ParamBase::String));
            getParam(p, NULL);
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 1, tr("Doesn't fit to interface DataIO!").toLatin1().data());
    }

//    emit parametersChanged(m_params);    

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::close(ItomSharedSemaphore *waitCond)
{
    //ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_pSpider)
    {
        delete m_pSpider;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    { //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }
    if (!retValue.containsError())
    {
        // General Parameters
        if (key == "name")
        {
            *val = it.value();
        }
        else if (key == "id")
        {
            QVector<QString> tmpDevStr;
            QVector<int> tmpDevNum;
            retValue += m_pSpider->hbmListDevices(tmpDevNum, &tmpDevStr);
        }
        else if (key == "samplingRate")
        {
            double tmpRate;
            retValue += m_pSpider->hbmGetSamplingRate(tmpRate);
            it->setVal<double>(tmpRate);
            *val = it.value();
        }
        else if (key == "bufferManagement")
        {
            bool tmp;
            retValue += m_pSpider->hbmGetMemMngmnt(tmp);
            it->setVal<char>(tmp);
            *val = it.value();
        }
        else if (key == "channelList")
        {
            QString chList("");
            QMapIterator<int, Spider8Channel> iter(m_channels);
            while (iter.hasNext())
            {
                iter.next();
                chList.append(QString("%1:\n").arg(iter.key()));
                QList<int> keys = iter.value().m_allowedModRanges.keys();
                for (int nm = 0; nm < keys.size(); nm++)
                {
                    chList.append(QString("\t%1: ").arg(keys[nm] - 350));
                    QVector<int> ranges = iter.value().m_allowedModRanges[keys[nm]];
                    for (int nr = 0; nr < ranges.size(); nr++)
                    {
                        chList.append(QString("%1 ").arg(ranges[nr] - 700));
                    }
                    chList.append("\n");
                }
                chList.append("\n");
            }
            it->setVal<char *>(chList.toLatin1().data());
            *val = it.value();
        }
        else if (key == "status")
        {
            int tmpStatus;
            m_pSpider->hbmGetStatus(tmpStatus);
            it->setVal<int>(tmpStatus);
            *val = it.value();
        }
        else if (key == "statusStr")
        {
            int tmpStatus;
            m_pSpider->hbmGetStatus(tmpStatus);
            switch (tmpStatus)
            {
                case 0:
                    it->setVal<char *>(tr("no data available").toLatin1().data());
                break;
                case 1:
                    it->setVal<char *>(tr("waiting").toLatin1().data());
                break;
                case 2:
                    it->setVal<char *>(tr("pre-trigger, filling buffer").toLatin1().data());
                break;
                case 3:
                    it->setVal<char *>(tr("pre-trigger, buffer full, waiting").toLatin1().data());
                break;
                case 4:
                    it->setVal<char *>(tr("post-trigger, filling buffer").toLatin1().data());
                break;
                case 5:
                    it->setVal<char *>(tr("acquistion terminated - ERROR").toLatin1().data());
                break;
                case 6:
                    it->setVal<char *>(tr("acquistion terminated - ok").toLatin1().data());
                break;
            }
            *val = it.value();
        }
/*
        else if (key == "channel")
        {
            // Hier weitermachen
            QString ch = m_channels.getAllChannelAsString().join(",");
            retValue += m_params["channel"].setVal<char*>(ch.toLatin1().data(), ch.size());
            *val = it.value();
        }
*/
        // Channel Paramteters
        else if (key.left(10) == "aiChParams")
        {
            int chaNum = -1;
            if (key.length() > 10)
                chaNum = atoi(key.right(key.length() - 10).toLatin1().data());
            if (m_channels.contains(chaNum))
            {
                QStringList params;
                int chmode = m_channels[chaNum].m_mode;
                int chrange = m_channels[chaNum].m_range;
                int chmeasmode = m_channels[chaNum].m_measMode;
                params.append(QString::number(chaNum));
                params.append(QString::number(chmode));
                params.append(QString::number(chrange));
                params.append(QString::number(chmeasmode));
                QString param = params.join(",");
                retValue += m_params["aiChParams"].setVal<char*>(param.toLatin1().data(), param.size());
                *val = it.value();
            }
            else
            {
                QStringList allParams;
                QString allParam;
                QMapIterator<int, Spider8Channel> iter(m_channels);
                while (iter.hasNext())
                {
                    iter.next();
                    QStringList params;
                    int chmode = iter.value().m_mode;
                    int chrange = iter.value().m_range;
                    int chmeasmode = iter.value().m_measMode;
                    params.append(QString::number(iter.key()));
                    params.append(QString::number(chmode));
                    params.append(QString::number(chrange));
                    params.append(QString::number(chmeasmode));
                    QString param = params.join(",");
                    allParams.append(param);
                }
                allParam = allParams.join(";");
                retValue += m_params["aiChParams"].setVal<char*>(allParam.toLatin1().data(), allParam.size());
                *val = it.value();
            }
        }
        else if (key == "diChParams")
        {
//            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoInput).join(";");
//            retValue += m_params["diChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        else if (key == "doChParams")
        {
//            QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoOutput).join(";");
//            retValue += m_params["doChParams"].setVal<char*>(para.toLatin1().data(), para.size());
            *val = it.value();
        }
        else if (key == "reset")
        {
            QString err;
            int errNum;
            retValue += m_pSpider->hbmGetError(err, errNum);
            it.value().setVal<char*>(err.toLatin1().data());
            *val = it.value();
        }
        else if (key == "actChannelList")
        {
            QStringList chList;
            QVector<int> *actChs = NULL;
            retValue += m_pSpider->getActiveChs(actChs);
            if (!retValue.containsError() && actChs != NULL)
            {
                for (int nc = 0; nc < actChs->size(); nc++)
                    chList << QString::number(actChs->at(nc));
            }
            it->setVal<char *>(chList.join(" ").toLatin1().data());
            *val = it.value();
        }
/*
        // Task Paramters
        else if (key == "taskStatus")
        {
            QStringList res;
            foreach(niTask* t, m_taskMap)
            {            
                QStringList ch;
                ch.append(t->getName());
                if (t->isInitialized())
                {
                    // Hier muss noch der status ausgelesen werden
                    // ist der Task running oder was auch immer
                    if (t->isDone())
                    {
                        ch.append("0");
                    }
                    else
                    {
                        ch.append("1");
                    }
                }
                else
                {
                    ch.append("-1");
                }
                res.append(ch.join(","));
            }
            QString resS = res.join(";");
            retValue += m_params["taskStatus"].setVal<char*>(resS.toLatin1().data(), resS.size());
            *val = it.value();
        }
        else if (key.right(10) == "TaskParams")
        {
            QStringList tl;                
            QString ch = key.left(2);
            if (m_taskMap.contains(ch))
            {
                if (m_taskMap.value(ch)->isInitialized())
                {
                    tl.append(QString::number(m_taskMap.value(ch)->getRateHz()));
                    tl.append(QString::number(m_taskMap.value(ch)->getSamplesToRW()));
                    tl.append(QString::number(m_taskMap.value(ch)->getMode()));
                    ch = tl.join(",");
                }
                else
                {
                    ch = "-1";
                }
            }
            retValue += m_params[key].setVal<char*>(ch.toLatin1().data(), ch.size());
            *val = it.value();
        }
*/
        else
        {
            *val = it.value();
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
ito::RetVal Spider8::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    ////parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

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
        if (key == "samplingRate")
        {
            double tmpRate = val->getVal<double>();
            retValue += m_pSpider->hbmSetSamplingRate(tmpRate);
            it->setVal<double>(tmpRate);
            *val = it.value();
        }
        else if (key == "bufferManagement")
        {
            bool tmp = val->getVal<int>();
            retValue += m_pSpider->hbmSetMemMngmnt(tmp);
            it->setVal<char>(tmp);
            *val = it.value();
        }
        else if (key == "trigger")
        {
            // (channel,mode,level)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (in.size() < 3)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("not enough arguments for setting trigger (channel/mode/level)").toLatin1().data());
                goto end;
            }
            int chnum = atoi(in[0].toLatin1().data());
            int mode = atoi(in[1].toLatin1().data());
            int level = atoi(in[2].toLatin1().data());
            if (chnum < 0 || chnum > 7)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("trigger channel not supported (0 .. 7)").toLatin1().data());
                goto end;
            }
            if (mode < 0 || mode > 3)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("mode not supported (0 .. 3)").toLatin1().data());
                goto end;
            }
            if (level < -32769 || level > 32767)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("level out of range (-32769 .. 32767)").toLatin1().data());
                goto end;
            }

            retValue += m_pSpider->hbmSetTrigger(chnum, mode, level);

            QString chParams = in.join(",");
            retValue += m_params["trigger"].setVal<char*>(chParams.toLatin1().data(), chParams.size());
            *val = it.value();
        }
        else if (key == "aiChParams")
        { 
            // (channel,inConfig,inRange)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (in.length() < 2)
                retValue += ito::RetVal(ito::retError, 0, tr("Not enough parameters passed").toLatin1().data());
            int chnum = atoi(in[0].toLatin1().data());
            if (m_channels.contains(chnum))
            {
                if (in.length() == 2 && atoi(in[1].toLatin1().data()) == -1)
                {
                    // disable channel
                    retValue += m_pSpider->hbmActivateCh(m_channels, chnum, 6200);
                    // and remove from active list
                    retValue += m_pSpider->delActiveChannel(chnum);
                }
                else if (in.length() >= 3)
                {
                    Spider8Channel *chan = &m_channels[chnum];
                    int chmode = atoi(in[1].toLatin1().data());
                    int chrange = atoi(in[2].toLatin1().data());
                    bool check = false;
                    retValue += m_pSpider->hbmCheckChModeRange(*chan, chmode, chrange, check);
                    if (!check || retValue.containsWarningOrError())
                    {
                        retValue += ito::RetVal(ito::retError, 0, tr("invalid mode or range for this channel, call listChannels for possible combinations").toLatin1().data());
                        goto end;
                    }
                    retValue += m_pSpider->hbmSetChConfig(*chan, chnum, chmode, chrange);
                    if (retValue.containsWarningOrError())
                    {
                        goto end;
                    }

                    QString chParams = in.join(",");
                    retValue += m_params["aiChParams"].setVal<char*>(chParams.toLatin1().data(), chParams.size());
                    *val = it.value();
                    retValue += it->copyValueFrom(&(*val));

                    int chactmode = 6201;
                    if (in.size() >= 4)
                        chactmode = atoi(in[3].toLatin1().data());
                    chan->m_measMode = chactmode;
                    chan->m_scale = chan->mapSFactors[chrange + 700];
                    retValue += m_pSpider->hbmActivateCh(m_channels, chnum, chactmode);
                    if (retValue.containsWarningOrError())
                    {
                        goto end;
                    }
                    retValue += m_pSpider->addActiveChannel(chnum);
                }
                else
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("insufficent parameters or command wrong formated").toLatin1().data());
                }
            }
        }
/*
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Your device is not supporting this port or the task is not initialized").toLatin1().data());
        }
        else if (key == "doChParams")
        { // (dev-channel,inConfig,minInLim,maxInLim)
            QStringList in = QString(val->getVal<char*>()).split(",");
            if (m_channels.contains(in[0]) && in.size() == 1 && m_taskMap.value("do")->isInitialized()) 
            {
                niDigitalOutputChannel *dou = NULL;
                if (m_channels.value(in[0]) == NULL)
                { // channel does not exist yet
                    dou = new niDigitalOutputChannel();
                }
                else
                { // channel already exists
                    dou = dynamic_cast<niDigitalOutputChannel*>(m_channels.value(in[0]));
                }
                dou->setDevID(in[0].split('/')[0]);
                dou->setChID(in[0].split('/')[1]);
                // Channel is finished and added to the corresponding input task
                dou->applyParameters(m_taskMap.value("do"));
                // increase the corresponding counter
                m_channels.insert(in[0], dou);
                QString para = m_channels.getAllChParameters(niBaseChannel::chTypeDigital, niBaseChannel::chIoOutput).join(";");
                retValue += m_params["doChParams"].setVal<char*>(para.toLatin1().data(), para.size());
                *val = it.value();
                retValue += it->copyValueFrom( &(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Your device is not supporting this port or the task is not initialized").toLatin1().data());
            }
        }
*/
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom( &(*val) );
        }
    }

end:
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
ito::RetVal Spider8::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += m_pSpider->hbmActivateCh(m_channels, -1, 6201);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::stopDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue(ito::retOk);

    // reset acquired flag - especially in case of continuouse measurements
    m_aInIsAcquired = false;
    m_dInIsAcquired = false;
    m_dOutIsAcquired = false;

    retValue += m_pSpider->hbmStopAcq(QStringList());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    // The trigger is used here in another meaning. It s a bitmask and defines
    // which tasks are started! (all decimal)
    //  1 = Analog Input
    //  2 = Digital Input
    //  4 = Counter Input
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval(ito::retOk);
    QString err;
    int ierr;

    retval += m_pSpider->hbmStartMeas(trigger + 6100, m_params["numSamples"].getVal<int>(), m_params["numCycles"].getVal<int>(), m_params["preTrgSamples"].getVal<int>());
    retval += m_pSpider->hbmGetError(err, ierr);

    if (ierr != 0)
    {
        retval += ito::RetVal(ito::retError, 0, err.toLatin1().data());
    }
    m_aInIsAcquired = true;

/*
    if (trigger & 1)
    {
        retval += m_taskMap.value("ai")->run();
        m_aInIsAcquired = true;
    }
    if (trigger & 2)
    {
        retval += m_taskMap.value("di")->run();
        m_dInIsAcquired = true;
    }
    if (trigger & 4)
    {
        retval += m_taskMap.value("ci")->run();
        m_cInIsAcquired = true;
    }
*/
    if (retval.containsWarning())
    {
        retval += ito::RetVal::format(ito::retWarning, 0, tr("Warning occured while starting read task. \n Code: %i").toLatin1().data(), retval.errorCode());
    }
    else if (retval.containsError())
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("Error occured while starting read task. \n Code: %i").toLatin1().data(), retval.errorCode());
    }
    else
    {
        m_isgrabbing = true;
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();  
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::readAnalog(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;
    bool copyExternal = (externalDataObject != NULL);

    int numChs = 0;
    int samplesRead = 0;
    int samplesTot = 0;
    int status = 0;
    QVector<int> *actChs = NULL;
    ito::uint16 *data = NULL;

//    retValue += m_pSpider->hbmGetNumActChs(numChs);

    m_pSpider->getActiveChs(actChs);
    numChs = actChs->size();
//    if (numChs != actChs->size())
//        retValue += ito::RetVal(ito::retError, 0, tr("configuration error, number of active channels in device and plugin are different, reset device").toLatin1().data());

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get values without triggering exposure").toLatin1().data());
    }
    else if (!retValue.containsWarningOrError())
    {
        samplesTot = m_params["numSamples"].getVal<int>();
        //step 1: create m_data (if not yet available)
        if (externalDataObject)
        {
            retValue += checkData(externalDataObject, numChs, samplesTot); //update external object
        }
        else
        {
            //not always necessary
            retValue += checkData(NULL, numChs, samplesTot); //update external object or m_data
        }

        double *iscales = (double*)calloc(numChs, sizeof(double));
        double *vscales = (double*)calloc(numChs, sizeof(double));
        double *voffsets = (double*)calloc(numChs, sizeof(double));
        double *tscales = (double*)m_params["scale"].getVal<void*>();
        double *toffsets = (double*)m_params["offset"].getVal<void*>();
        QMapIterator<int, Spider8Channel> iter(m_channels);
        for (int nc = 0; nc < numChs; nc++)
        {
            iscales[nc] = m_channels[actChs->at(nc)].m_scale;
            vscales[nc] = tscales[actChs->at(nc)];
            voffsets[nc] = toffsets[actChs->at(nc)];
        }

        retValue += m_pSpider->hbmGetNumSamples(status, samplesRead);
        while (samplesRead < samplesTot && samplesTot < 3000)
        {
            Sleep(m_pSpider->getRdSleep());
            retValue += m_pSpider->hbmGetNumSamples(status, samplesRead);
        }

        if (status == 0 || retValue.containsError())
        {
            retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get values without triggering exposure").toLatin1().data());
        }
        if (samplesRead * numChs < 3000 && m_pSpider->getLastIdx() == 0)
        {
            retValue += m_pSpider->hbmReadNScale(numChs, samplesRead, iscales, voffsets, vscales, dataObj);
            //m_pSpider->resetLastIdx();
        }
        else
        {
            int lastIdx = m_pSpider->getLastIdx();
            int incr = 0;
            for (int nr = m_pSpider->getLastIdx(); nr < samplesRead; nr += incr)
            {
                incr = m_pSpider->getLastIdx() + 3000 / numChs > samplesRead ?
                    samplesRead - m_pSpider->getLastIdx() :
                    3000 / numChs;
                retValue += m_pSpider->hbmReadNScale(numChs, incr, iscales, voffsets, vscales, dataObj);
            }
        }
        if (samplesRead == samplesTot)
            m_pSpider->resetLastIdx();

        free(iscales);
        free(vscales);
        free(voffsets);
/*
        QSharedPointer<char> data(new char[numChs * samples * 2 + 4]);
        QSharedPointer<int> dataSize(new int);
        *dataSize = numChs * samples * 2 + 4;
//        retValue += m_pSpider->hbmReadData(data, dataSize, samples);
        if (!retValue.containsWarningOrError())
        {
            double *scales = (double*)calloc(numChs, sizeof(double));
            for (int nc = 0; nc < numChs; nc++)
            {
                scales[nc] = m_channels[nc].m_scale;
            }

            retValue += m_pSpider->scaleValues((ito::uint16*)&data.data()[2], dataObj, samples, numChs, scales);
            free(scales);
        }
*/
        ito::int32 retSize = -1;
        double vScale = 1;

        // TODO: Hier muss die Skala an den hoechsten Range angepasst werden und alle Reihen im Datenobject entsprechend ihres Ranges durchmultipliziert werden
        //m_channels m_taskMap.value("ai")->getChList()
        //m_channels.getAllChannelOfType(niBaseChannel::chTypeAnalog)[0]->
        m_data.setAxisScale(2, vScale);
        m_data.setAxisUnit(2, "volt");
        m_data.setAxisDescription(2, "voltage");


        if (copyExternal)
        {
            retValue += m_data.deepCopyPartial(*externalDataObject);
        }

        if (status != 4)
            m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::readDigital(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;
    bool copyExternal = (externalDataObject != NULL);

//    int ports = m_taskMap.value("di")->getChCount();
//    int samples = m_taskMap.value("di")->getSamplesToRW();
    int ports = 0;
    int samples = 0;

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get data without triggering recording").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject)
        {
            retValue += checkData(NULL, ports * 8, samples); //update external object or m_data
            retValue += checkData(externalDataObject, ports, samples); //update external object
        }
        else
        {
            //not always necessary
            retValue += checkData(NULL, ports, samples); //update external object or m_data
        }

        int size = ports * samples;
        ito::int32 retSize = -1;

        //int err = DAQmxReadDigitalU8(*m_taskMap.value("di")->getTaskHandle(), samples, DAQmx_Val_WaitInfinitely, DAQmx_Val_GroupByChannel, (ito::uint8*)m_data.rowPtr(0,0), size, &retSize, NULL); 
        m_data.setAxisDescription(0, "status");
        m_data.setAxisDescription(1, "lines");

        if (copyExternal)
        {
            retValue += m_data.deepCopyPartial(*externalDataObject);
        }

        // if not continuous measurement reset grabbing flag
        if (m_params["numCycles"].getVal<int>() != 0 && m_params["numSamples"].getVal<int>() != 1)
            m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::writeDigital(const int channel, ito::DataObject *externalDataObject)
{
    ito::RetVal retval(ito::retOk);

    if (externalDataObject != 0)
    {
        int dtype = externalDataObject->getType();
        if (dtype != ito::tInt16 && dtype != ito::tUInt16)
            return ito::RetVal(ito::retError, 0, tr("Invalid data type for do. Only int and uint are supported").toLatin1().data());
        if (channel % 10 != 8)
            return ito::RetVal(ito::retError, 0, tr("only channel 8, 18, ... can be used as digital output").toLatin1().data());
        char outbuf[15];
        sprintf(outbuf, "DPI%d,%d", channel, externalDataObject->at(0));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as reference.
/*!
    This method returns a reference to the recently acquired image. Therefore this camera size must fit to the data structure of the 
    DataObject.
    
    This method returns a reference to the internal dataObject m_data of the camera where the currently acquired image data is copied to (either
    in the acquire method or in retrieve data). Please remember, that the reference may directly change if a new image is acquired.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the camera.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    
    \sa retrieveImage, copyVal
*/
ito::RetVal Spider8::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int error = -1;
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    
    if (m_aInIsAcquired)
    {
        retValue += readAnalog();
        if (m_params["numCycles"].getVal<int>() != 0 && m_params["numSamples"].getVal<int>() != 1)
            m_aInIsAcquired = false;
        // Die folgende zeile stoppt den task um ihn erneut starten zu können. Rsourcen bleiben erhalten. Vielleicht in extra funktion auslagern
        // error = DAQmxTaskControl(m_taskMap.value("ai")->getTaskHandle(),DAQmx_Val_Task_Reserve);
//        retValue += m_taskMap.value("ai")->stop();
    }
    else if (m_dInIsAcquired)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("The digital read mode is not supported yet, because an external clock source is neccesary").toLatin1().data());
        // retValue += readDigital();
        //m_dInIsAcquired = false;
    }

    if (!retValue.containsError())
    {
        if (dObj)
        {
            (*dObj) = m_data; //copy reference to externally given object
        }
    }

//    retValue += m_taskMap.value("ai")->resetTaskHandle();

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as a deep copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject. 
    
    The given dataObject must either have an empty size (then it is resized to the size and type of the camera image) or its size or adjusted region of
    interest must exactly fit to the size of the camera. Then, the acquired image is copied inside of the given region of interest (copy into a subpart of
    an image stack is possible then)

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    
    \sa retrieveImage, getVal
*/
ito::RetVal Spider8::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    
    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    
    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        if (m_aInIsAcquired)
        {
            retValue += readAnalog();
            m_aInIsAcquired = false;
        }
        else if (m_dInIsAcquired)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("The digital read mode is not supported yet, because an external clock source is neccesary").toLatin1().data());
            //retValue += readDigital();
            m_dInIsAcquired = false;
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
ito::RetVal Spider8::setVal(const char *data, const int length, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    const ito::DataObject *dObj = reinterpret_cast<const ito::DataObject*>(data);

    int error = 0;
 /*
    if (dObj->getDims() != 2 || dObj->getSize(0) != m_taskMap.value("ao")->getChCount() || dObj->getSize(1) != m_taskMap.value("ao")->getSamplesToRW())
    {
        retValue += ito::RetVal::format(ito::retWarning, 0, tr("Error occured: given Dataobject has wrong dimensions.").toLatin1().data(), 0);
    }

    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        if (m_dOutIsAcquired)
        {
            retValue += writeDigital();
            m_taskMap.value("do")->applyParameters();
            error = DAQmxStartTask(*m_taskMap.value("do")->getTaskHandle());
            m_dOutIsAcquired = false;
        }
    }
*/
    if (error > 0)
    {
        retValue += ito::RetVal::format(ito::retWarning, 0, tr("Warning occured while starting read task. \n Code: %i").toLatin1().data(), error);
    }
    else if (error < 0)
    {
        retValue += ito::RetVal::format(ito::retError, 0, tr("Error occured while starting read task. \n Code: %i").toLatin1().data(), error);
    }
    else
    {
        m_isgrabbing = true;
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void Spider8::dockWidgetVisibilityChanged(bool visible)
{
    //if (getDockWidget())
    //{
    //    QWidget *widget = getDockWidget()->widget();
    //    if (visible)
    //    {
    //        connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));

    //        emit parametersChanged(m_params);
    //    }
    //    else
    //    {
    //        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
    //    }
    //}
}

//----------------------------------------------------------------------------------------------------------------------------------
//! method called to show the configuration dialog
/*!
    This method is called from the main thread from itom and should show the configuration dialog of the plugin.
    If the instance of the configuration dialog has been created, its slot 'parametersChanged' is connected to the signal 'parametersChanged'
    of the plugin. By invoking the slot sendParameterRequest of the plugin, the plugin's signal parametersChanged is immediately emitted with
    m_params as argument. Therefore the configuration dialog obtains the current set of parameters and can be adjusted to its values.
    
    The configuration dialog should emit reject() or accept() depending if the user wanted to close the dialog using the ok or cancel button.
    If ok has been clicked (accept()), this method calls applyParameters of the configuration dialog in order to force the dialog to send
    all changed parameters to the plugin. If the user clicks an apply button, the configuration dialog itsself must call applyParameters.
    
    If the configuration dialog is inherited from AbstractAddInConfigDialog, use the api-function apiShowConfigurationDialog that does all
    the things mentioned in this description.
    
    Remember that you need to implement hasConfDialog in your plugin and return 1 in order to signalize itom that the plugin
    has a configuration dialog.
    
    \sa hasConfDialog
*/
const ito::RetVal Spider8::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogSpider8(this, (void*)this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal Spider8::checkData(ito::DataObject *externalDataObject, int channels, int samples)
{
    if (externalDataObject == NULL)
    {
        //check internal object m_data
        if (m_data.getDims() != 2 || m_data.getSize(0) != (unsigned int)channels || m_data.getSize(1) != (unsigned int)samples || m_data.getType() != ito::tFloat64)
        {
            m_data = ito::DataObject(channels, samples, ito::tFloat64);
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0 || externalDataObject->getSize(0) != (unsigned int)channels || externalDataObject->getSize(1) != (unsigned int)samples || externalDataObject->getType() != ito::tFloat64)
        {
            *externalDataObject = ito::DataObject(channels, samples, ito::tFloat64);
        }
        else if (externalDataObject->calcNumMats () > 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toLatin1().data());            
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)channels || externalDataObject->getSize(dims - 1) != (unsigned int)samples || externalDataObject->getType() != ito::tFloat64)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------