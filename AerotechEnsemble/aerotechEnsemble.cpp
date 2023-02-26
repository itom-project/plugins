/* ********************************************************************
    Plugin "AerotechEnsemble" for itom software
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

#include "AerotechEnsemble.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include <math.h>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmutex.h>
#include <qwaitcondition.h>

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsembleInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(AerotechEnsemble)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsembleInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(AerotechEnsemble)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechEnsembleInterface::AerotechEnsembleInterface(QObject *parent)
{
    m_type = ito::typeActuator;
    setObjectName("AerotechEnsemble");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char* docstring = \
"This plugin allows communicating with controllers of type Ensemble (4.xx Version) of company Aerotech. \n\
\n\
If no parameters are given, the plugin connects to all available axes of the controller. Else you can provide \
a list of axis numbers (0..9) that should be connected. The first axis of this list then gets the axis ID 0, the \
second the axis ID 1 and so on. \n\
For running this plugin you need an installed Ensemble driver and a connected device. \n\
\n\
This plugin comes with version 4.06 of the Ensemble driver. You can change them by newer libraries (Version 4.XX). The manual of Ensemble \
allows redistributing the Ensemble libraries without having the end-user install the Ensemble software. For further information about \
license information of Aerotech see their documentation. \n\
\n\
For loading the Ensemble library you need the Visual C++ 2008 SP1 Redistributable Package provided by Microsoft (see Ensemble Programming Help).";
*/
    m_description = QObject::tr("Plugin for the Ensemble-controller of Aerotech");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"This plugin allows communicating with controllers of type Ensemble (4.xx Version) of company Aerotech. \n\
\n\
If no parameters are given, the plugin connects to all available axes of the controller. Else you can provide \
a list of axis numbers (0..9) that should be connected. The first axis of this list then gets the axis ID 0, the \
second the axis ID 1 and so on. \n\
For running this plugin you need an installed Ensemble driver and a connected device. \n\
\n\
This plugin comes with version 4.06 of the Ensemble driver. You can change them by newer libraries (Version 4.XX). The manual of Ensemble \
allows redistributing the Ensemble libraries without having the end-user install the Ensemble software. For further information about \
license information of Aerotech see their documentation. \n\
\n\
For loading the Ensemble library you need the Visual C++ 2008 SP1 Redistributable Package provided by Microsoft (see Ensemble Programming Help).");

    m_author = "A. Bielke, M. Gronle, ITO, University Stuttgart, Juergen Ortmann, Ortmann Digitaltechnik";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LGPL, The Aerotech Ensemble library belongs to Aerotech under their specific license.");
    m_aboutThis = QObject::tr(GITVERSION);     
    
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    ito::Param param = ito::Param("axes", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("list of axes IDs that are enabled (0..9). The first ID then obtains index 0, the second ID index 1... [default: empty list, all available axes are connected]").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechEnsembleInterface::~AerotechEnsembleInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal AerotechEnsemble::showConfDialog(void)
{
    RetVal retValue(retOk);

    dialogAerotechEnsemble *confDialog = new dialogAerotechEnsemble(qobject_cast<ito::AddInActuator*>(this), m_axisNames);    // Create dialog
    confDialog->setVals(&m_params);    // Set up dialog parameters
    if (confDialog->exec())    // Is dialog is endet with exec and not with cancel
    {
        confDialog->getVals(&m_params);    // get parameters from dialog
    }    
    delete confDialog;    // destray dialog

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechEnsemble::AerotechEnsemble() :
    AddInActuator(),
    m_pAerotechEnsembleWid(NULL),
    m_pHandle(NULL),
    m_pHandles(NULL)
{
    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");    // To enable the programm to transmit parameters via signals - slot connections
    qRegisterMetaType<QVector<bool> >("QVector<bool>");
    qRegisterMetaType<QVector<double> >("QVector<double>");

    //ito::tParam;    // Set up the parameter list
    m_params.insert("name", Param("name", ParamBase::String | ParamBase::In | ParamBase::Readonly, "AerotechEnsemble", NULL));

    m_params.insert("controller", Param("controller", ParamBase::String | ParamBase::In | ParamBase::Readonly, "", "name of the connected controller"));
    m_params.insert("communication", Param("communication", ParamBase::String | ParamBase::In | ParamBase::Readonly, "", "type of the communication (USB, Ethernet)"));
    m_params.insert("libraryVersion", Param("libraryVersion", ParamBase::String | ParamBase::In | ParamBase::Readonly, "", "Version of the Ensemble C library"));

    m_params.insert("async", Param("async", ParamBase::Int, 0, 1, 0, tr("asynchronous move (1), synchronous (0) [default]").toLatin1().data()));
    m_async = m_params["async"].getVal<int>();

    m_params.insert("numAxis", Param("numAxis", ParamBase::Int | ParamBase::In | ParamBase::Readonly, 0, 10, 0, "number of connected axes"));

    double axisSpeeds[] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; //mm/s
    Param param = Param("speed", ParamBase::DoubleArray, NULL, tr("speed of every axis").toLatin1().data());
    param.setVal<double*>(axisSpeeds, 10);
    m_params.insert("speed", param);

    m_currentPos.fill(0.0, 10);
    m_currentStatus.fill(0, 10);
    m_targetPos.fill(0.0, 10);

    // memset(m_pos, 0, 10 * sizeof(double));

    // // This is for the docking widged
    // //now create dock widget for this plugin
    if (hasGuiSupport())
    {
        m_pAerotechEnsembleWid = new DockWidgetAerotechEnsemble(m_params, getID(), this);    // Create a new non-modal dialog
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_pAerotechEnsembleWid);    // Give the widget a name ..)
        connect(m_pAerotechEnsembleWid, SIGNAL(MotorTriggerStatusRequest(bool, bool)), this, SLOT(RequestStatusAndPosition(bool, bool)));
        connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), m_pAerotechEnsembleWid, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        connect(this, SIGNAL(dockWidgetAerotechEnsembleInit(QMap<QString, ito::Param>, QStringList)), m_pAerotechEnsembleWid, SLOT(init(QMap<QString, ito::Param>, QStringList)));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechEnsemble::~AerotechEnsemble()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::checkError(bool ensembleReturnValue)
{
    if (ensembleReturnValue == true)
    {
        return ito::retOk;
    }
    else
    {
        char errorString[1024];
        if (EnsembleGetLastErrorString(errorString, 1024))
        {
            return ito::RetVal::format(ito::retError, 0, tr("Ensemble error %i: %s").toLatin1().data(), EnsembleGetLastError(), errorString);
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Unknown ensemble error since the error message was too long").toLatin1().data());
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::getAxisMask(const int *axes, const int numAxes, AXISMASK &mask)
{
    ito::RetVal retValue;
    mask = AXISMASK_None;
    int nums = numAxes;
    for (int i = 0; i < nums; ++i)
    {
        switch(axes[i])
        {
        case 0:
            mask = mask | AXISMASK_0;
            break;
        case 1:
            mask = mask | AXISMASK_1;
            break;
        case 2:
            mask = mask | AXISMASK_2;
            break;
        case 3:
            mask = mask | AXISMASK_3;
            break;
        case 4:
            mask = mask | AXISMASK_4;
            break;
        case 5:
            mask = mask | AXISMASK_5;
            break;
        case 6:
            mask = mask | AXISMASK_6;
            break;
        case 7:
            mask = mask | AXISMASK_7;
            break;
        case 8:
            mask = mask | AXISMASK_8;
            break;
        case 9:
            mask = mask | AXISMASK_9;
            break;
        default:
            retValue += ito::RetVal::format(ito::retError, 0, tr("The axis number %i is not supported. Allowed range [0, 9]").toLatin1().data(), axes[i]);
            break;
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::getAxisMask2(const QVector<int> &axesIndices, AXISMASK &mask)
{
    ito::RetVal retValue;
    mask = AXISMASK_None;

    foreach(const int &index, axesIndices)
    {
        if (index < 0 || index >= m_enabledAxes.size())
        {
            retValue += ito::RetVal::format(ito::retError, 0, tr("axis index %i is out of boundary [0, %i]").toLatin1().data(), index, m_enabledAxes.size()-1);
        }
        else
        {
            switch(m_enabledAxes[index])
            {
            case 0:
                mask = mask | AXISMASK_0;
                break;
            case 1:
                mask = mask | AXISMASK_1;
                break;
            case 2:
                mask = mask | AXISMASK_2;
                break;
            case 3:
                mask = mask | AXISMASK_3;
                break;
            case 4:
                mask = mask | AXISMASK_4;
                break;
            case 5:
                mask = mask | AXISMASK_5;
                break;
            case 6:
                mask = mask | AXISMASK_6;
                break;
            case 7:
                mask = mask | AXISMASK_7;
                break;
            case 8:
                mask = mask | AXISMASK_8;
                break;
            case 9:
                mask = mask | AXISMASK_9;
                break;
            default:
                retValue += ito::RetVal::format(ito::retError, 0, tr("The axis number %i is not supported. Allowed range [0, 9]").toLatin1().data(), m_enabledAxes[index]);
                break;
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    RetVal retValue(retOk);
    char *temp = NULL;

    DWORD handleCount = 0;

    //enable all axes that are contained in the first optional parameter
    int *axes = paramsOpt->at(0).getVal<int*>();
    int axesLength = paramsOpt->at(0).getLen();
    QVector<int> axesIDs;

    // Clear the world
//    retValue += checkError(EnsembleMotionFaultAck(m_pHandle, (AXISMASK_0 | AXISMASK_1 | AXISMASK_2 | AXISMASK_3 | AXISMASK_4 | AXISMASK_5 | AXISMASK_6 | AXISMASK_7 | AXISMASK_8 | AXISMASK_9)));

    for (int i = 0; i < axesLength; ++i)
    {
        axesIDs.append(axes[i]);
        m_allAxesVector << i;
    }

    AXISMASK axisMask;

    retValue += getAxisMask(axes, axesLength, axisMask);

    if (!retValue.containsError())
    {
        retValue += checkError(EnsembleConnect(&m_pHandles, &handleCount));

        if (handleCount > 1)
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Please make sure that only one controller is configured and connected").toLatin1().data());
        }
    }
    
    if (!retValue.containsError())
    {
        m_pHandle = m_pHandles[0];

        //controller name
        QByteArray name(256, '\0');
        if (EnsembleInformationGetName(m_pHandle, name.size(), name.data()))
        {
            m_params["controller"].setVal<char*>(name.data());
            m_identifier = name;
        }

        //communication type
        COMMUNICATIONTYPE communicationType;
        if (EnsembleInformationGetCommunicationType(m_pHandle, &communicationType))
        {
            name = (communicationType == COMMUNICATIONTYPE_Ethernet)? "Ethernet" : "USB";
            m_params["communication"].setVal<char*>(name.data());
        }
        
        //library version
        Version version;
        if (EnsembleInformationGetLibraryVersion(&version))
        {
            name = QString("%1.%2.%3 (%4)").arg(version.major).arg(version.minor, 2, 10, QLatin1Char('0')).arg(version.patch, 3, 10, QLatin1Char('0')).arg(version.build).toLatin1(); //TODO 4.01.006
            m_params["libraryVersion"].setVal<char*>(name.data());
        }

        if (!retValue.containsError())
        {
            retValue += checkError(EnsembleMotionWaitMode(m_pHandle, WAITTYPE_NoWait));
        }

        AXISMASK availableMask;
        retValue += checkError(EnsembleInformationGetAxisMask(m_pHandle, &availableMask));
        
        if (!retValue.containsError())
        {
            if (axesLength <= 0) //no specific axes given, all available are taken
            {
                axesLength = 0;
                axisMask = availableMask;
                if (axisMask & AXISMASK_0)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_0);
                }
                if (axisMask & AXISMASK_1)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_1);
                }
                if (axisMask & AXISMASK_2)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_2);
                }
                if (axisMask & AXISMASK_3)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_3);
                }
                if (axisMask & AXISMASK_4)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_4);
                }
                if (axisMask & AXISMASK_5)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_5);
                }
                if (axisMask & AXISMASK_6)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_6);
                }
                if (axisMask & AXISMASK_7)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_7);
                }
                if (axisMask & AXISMASK_8)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_8);
                }
                if (axisMask & AXISMASK_9)
                {
                    axesLength++;
                    axesIDs.append(AXISINDEX_9);
                }
            }

            for (int i = 0; i < axesLength; ++i)
            {
                m_allAxesVector << i;
            }

            QByteArray name(256, '\0');
            foreach(const int &i, axesIDs)
            {
                EnsembleParameterGetValueString(m_pHandle, PARAMETERID_AxisName, i, name.size(), name.data());
                m_axisNames.append(name);
            }

            if ((availableMask | axisMask) != availableMask)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("Not all desired axes are connected to the controller (desired: %i, available: %i)").toLatin1().data(), axisMask, availableMask);
            }

            if (!retValue.containsError())
            {
                ito::RetVal retValueTemp = checkError(EnsembleMotionEnable(m_pHandle, axisMask));

                if (retValueTemp.containsError())
                {
                    retValue += checkError(EnsembleMotionFaultAck(m_pHandle, axisMask));

                    if (!retValue.containsError())
                    {
                        retValue += checkError(EnsembleMotionEnable(m_pHandle, axisMask));
                    }
                }
                else
                {
                    retValue += retValueTemp;
                }
            }

            
        }
    }

    if (!retValue.containsError())
    {
        m_currentPos.fill(0.0, axesLength);
        m_currentStatus.fill(ito::actuatorAvailable | ito::actuatorAtTarget, axesLength);
        m_targetPos.fill(0.0, axesLength);

        QVector<int> _axes(axesLength);

        //remember enabled axes
        for (int i = 0; i < axesLength; ++i)
        {
            m_enabledAxes.append(axesIDs[i]);
            _axes[i] = i;
        }

        ito::RetVal retValTemp =  doUpdatePosAndState(_axes);
        if (retValTemp.containsError())
        {
            retValue += checkError(EnsembleMotionFaultAck(m_pHandle, axisMask));
            retValue +=  doUpdatePosAndState(_axes);
        }
        else
        {
            retValue += retValTemp;
        }

        m_targetPos = m_currentPos;

        sendStatusUpdate();

        m_params["speed"].setVal<double*>(m_params["speed"].getVal<double*>(), axesLength); //shorten speed array to real number of connected axes
        m_params["numAxis"].setVal<int>(axesLength);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    if (!retValue.containsError())
    {    
        emit parametersChanged(m_params);
    }

    setInitialized(true); //init method has been finished (independent on retval)
    emit dockWidgetAerotechEnsembleInit(m_params, m_axisNames);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_pHandles != NULL)
    {
        retValue += checkError(EnsembleDisconnect(m_pHandles));
        m_enabledAxes.clear();
        m_pHandles = NULL;
        m_pHandle = NULL;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();        
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        *val = apiGetParam(*it, hasIndex, index, retValue);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
        if (key == "async")
        {
            retValue += it->copyValueFrom(&(*val));
            m_async = val->getVal<int>();
        }
        else if (key == "speed")
        {
            int arrayLength = it->getLen(); //current length of speed array must not be changed
            if (hasIndex)
            {
                if (index < 0 || index >= arrayLength)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("given index is out of boundary").toLatin1().data());
                }
                else if (val->getType() != ito::ParamBase::Double)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("given value must be a double if an index is given.").toLatin1().data());
                }
                else
                {
                    it->getVal<double*>()[index] = val->getVal<double>();
                }
            }
            else
            {
                if (val->getType() != ito::ParamBase::DoubleArray)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("given value must be a double array").toLatin1().data());
                }
                else if (val->getLen() != arrayLength)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("length of given double array must correspond to number of axes").toLatin1().data());
                }
                else
                {
                    retValue += it->copyValueFrom(&(*val));
                }
            }
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
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
ito::RetVal AerotechEnsemble::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else if (m_pHandle == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech Ensemble Handle is NULL").toLatin1().data());
    }
    else
    {
        AXISMASK mask;
        retValue += getAxisMask2(axis, mask);

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();
            
            //starts a small worker thread with a timer that regularily calls doAliveTimer to trigger the alive thread such that itom do not run into
            //a timeout if the homing needs lots of time
            QThread *awakeThread = new QThread(this);
            QTimer* timer = new QTimer(NULL); // _not_ this!
            timer->setInterval(500);
            timer->moveToThread(awakeThread);
            // Use a direct connection to make sure that doIt() is called from m_thread.
            connect(timer, SIGNAL(timeout()), SLOT(doAliveTimer()), Qt::DirectConnection);
            // Make sure the timer gets started from m_thread.
            QObject::connect(awakeThread, SIGNAL(started()), timer, SLOT(start()));
            awakeThread->start();
    
            if (axis.size() > 0)
            {
                retValue += checkError(EnsembleMotionHome(m_pHandle, mask));
            }

            awakeThread->quit();
            awakeThread->wait();
            timer->deleteLater();
            delete awakeThread;
            awakeThread = NULL;

            sendTargetUpdate();
            
            replaceStatus(axis, ito::actuatorMoving, ito::actuatorAtTarget); 
            doUpdatePosAndState(axis);

            sendStatusUpdate();

            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void AerotechEnsemble::doAliveTimer()
{
    setAlive();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toLatin1().data());
    }
    else
    {
        foreach(const int axisId, axis)
        {
            if (m_enabledAxes.contains(axisId) == false)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("axis %i is not enabled").toLatin1().data(), axisId);
            }
        }

        AXISMASK axisMask;
        retValue += getAxisMask2(axis, axisMask);

        if (!retValue.containsError())
        {
            double *axisSpeeds = m_params["speed"].getVal<double*>();
            double *positions = new double[axis.size()];
            memset(positions, 0, axis.size() * sizeof(double));

            retValue += checkError(EnsembleMotionMoveAbs(m_pHandle, axisMask, positions, axisSpeeds));
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
ito::RetVal AerotechEnsemble::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    QVector<int> axes;
    axes.reserve(m_enabledAxes.size());

    for (int i = 0; i < m_enabledAxes.size(); ++i)
    {
        axes.append(i);
    }

    retValue += doUpdatePosAndState(axes);

    *status = m_currentStatus;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if ((axis >= m_enabledAxes.size()) || (axis >= 10) || axis < 0)
    {
        retValue += ito::RetVal(ito::retError, 1, tr("axis index is out of bound").toLatin1().data());
    }
    else
    {
        retValue += doUpdatePosAndState(QVector<int>(1, axis));
        (*pos) = m_currentPos[axis];
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::getPos(QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += doUpdatePosAndState(axis);

    for (int naxis = 0; naxis < axis.size(); naxis++)
    {
        if ((axis[naxis] >= m_enabledAxes.size()) || (axis[naxis] >= 10) || axis[naxis] < 0)
        {
            retValue += ito::RetVal(ito::retError, 1, tr("at least one axis index is out of bound").toLatin1().data());
        }
        else
        {
            (*pos)[naxis] = m_currentPos[axis[naxis]];
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
ito::RetVal AerotechEnsemble::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::setPosAbs(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else if (m_pHandle == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech Ensemble Handle is NULL").toLatin1().data());
    }
    else
    {
        AXISMASK mask;
        retValue += getAxisMask2(axis, mask);

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            double posArray[10];
            double speedArray[10];
            double *paramSpeed = m_params["speed"].getVal<double*>(); //mm/s

            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                posArray[naxis] = pos[naxis];
                m_targetPos[axis[naxis]] = pos[naxis];
                speedArray[naxis] = paramSpeed[axis[naxis]];
            }
                    
            if (axis.size() > 0) 
            {
                retValue += checkError(EnsembleMotionMoveAbs(m_pHandle, mask, posArray, speedArray));
            }

            sendTargetUpdate();

            if (m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            ito::RetVal temp = waitForDone(-1, axis); //drops into timeout
            if (temp.containsError() && temp.errorCode() != 9999) //anything else besides timeout
            {
                retValue += temp;
            }
            
            doUpdatePosAndState(axis);

            sendStatusUpdate();

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsemble::setPosRel(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toLatin1().data());
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else if (m_pHandle == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech Ensemble Handle is NULL").toLatin1().data());
    }
    else
    {
        AXISMASK mask;
        retValue += getAxisMask2(axis, mask);

        if (retValue.containsError())
        {
            if (waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
        else
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            sendStatusUpdate();

            double posArray[10];
            double speedArray[10];
            double *paramSpeed = m_params["speed"].getVal<double*>(); //mm/s

            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                posArray[naxis] = pos[naxis];
                m_targetPos[axis[naxis]] = m_currentPos[axis[naxis]] + pos[naxis];
                speedArray[naxis] = paramSpeed[axis[naxis]];        
            }
                    
            if (axis.size() > 0) 
            {
                retValue += checkError(EnsembleMotionMoveInc(m_pHandle, mask, posArray, speedArray));
            }

            sendTargetUpdate();

            if (m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }

            ito::RetVal temp = waitForDone(-1, axis); //drops into timeout
            if (temp.containsError() && temp.errorCode() != 9999) //anything else besides timeout
            {
                retValue += temp;
            }
            
            doUpdatePosAndState(axis);

            sendStatusUpdate();

            if (!m_async && waitCond)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
            }
        }
    }

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal AerotechEnsemble::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    QVector<int> _axis = axis;
    if (_axis.size() == 0) //all axis
    {
        for (int i = 0; i < m_enabledAxes.size(); i++)
        {
            _axis.append(i);
        }
    }
    
    QElapsedTimer timer;
    timer.start();
    QMutex waitMutex;
    QWaitCondition waitCondition;
    long delay = 100; //[ms]

    while (!done && !timeout)
    {
        if (!done && isInterrupted())
        {
            if (m_pHandle != NULL) 
            {
                AXISMASK AxisMask = AXISMASK_All;
                retVal += checkError(EnsembleMotionAbort(m_pHandle, AxisMask));
            }

            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toLatin1().data());
            done = true;
            return retVal;
        }

        //QCoreApplication::processEvents();

        //short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, delay);
        waitMutex.unlock();
        setAlive();

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS)
            {
                timeout = true;
            }
        }

        bool bMove = false;
        retVal += doUpdatePosAndState(_axis);

        foreach(const int axis, _axis)
        {
            if (m_currentStatus[axis] & ito::actuatorMoving)
            {
                bMove = TRUE;
            }
        }
        sendStatusUpdate();
 
        if (bMove == FALSE)
        {
            done = true;
        }
    }

    if (timeout)
    {
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout); 
        retVal += ito::RetVal(ito::retError, 9999, tr("timeout occurred").toLatin1().data());
    }

    //100 ms damit die Achsen sich einpegeln koennen
    waitMutex.lock();
    waitCondition.wait(&waitMutex, 100);
    waitMutex.unlock();
    setAlive();

    doUpdatePosAndState(_axis);
    sendStatusUpdate();

    return retVal;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal AerotechEnsemble::doUpdatePosAndState(const QVector<int> &axes)
{
    ito::RetVal retval;
    if (m_pHandle != NULL) 
    {
        STATUSITEM pItems[3];
        double pDouble[3];
        bool bRet = TRUE;
        bool bMove = FALSE;

        foreach(const int &axis, axes) 
        {
            AXISINDEX axisIndex = (AXISINDEX)(AXISINDEX_0 + m_enabledAxes[axis]);

            pItems[0] = STATUSITEM_ProgramPositionFeedback;
            pItems[1] = STATUSITEM_AxisStatus;
            pItems[2] = STATUSITEM_AxisFault;

            if (EnsembleStatusGetItems(m_pHandle, axisIndex, 3, pItems, pDouble)) 
            {
                m_currentPos[axis] = pDouble[0];
                DWORD State = (DWORD)(pDouble[1]);

                if (State & AXISSTATUS_Enabled)
                {
                    m_currentStatus[axis] |= ito::actuatorEnabled;
                }
                else
                {
                    m_currentStatus[axis] ^= ito::actuatorEnabled;
                }

                if (State & AXISSTATUS_InPosition) 
                {
                    setStatus(m_currentStatus[axis], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                }
                else if (State & AXISSTATUS_MoveActive) 
                {
                    setStatus(m_currentStatus[axis], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                }

                if (State & AXISSTATUS_CwEndOfTravelLimitInput) 
                {
                    setStatus(m_currentStatus[axis], ito::actuatorRightEndSwitch, ito::actMovingMask | ito::actStatusMask);
                }

                if (State & AXISSTATUS_CcwEndOfTravelLimitInput) 
                {
                    setStatus(m_currentStatus[axis], ito::actuatorLeftEndSwitch, ito::actMovingMask | ito::actStatusMask);
                }

                DWORD AxisFault = (DWORD)(pDouble[2]);

                if (AxisFault != 0)
                {
                    retval += axisFaultToRetVal(AxisFault, axis);

                    if (retval.containsError()) //any error in moving, positioning...
                    {
                        setStatus(m_currentStatus[axis], ito::actuatorInterrupted, ito::actSwitchesMask | ito::actStatusMask);
                    }
                }
            }
            else
            {
                retval += checkError(false); //there was an error, parse it into retval.
            }
        }
    }

    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal AerotechEnsemble::axisFaultToRetVal(int axisFault, int axisID)
{
    ito::RetVal retval;
    switch (axisFault)
    {
    case 0:
        break;
    case AXISFAULT_PositionErrorFault:
        retval += ito::RetVal(ito::retError, 0, tr("The absolute value of the difference between the position command and the position feedback exceeded the threshold specified by the PositionErrorThreshold parameter.").toLatin1().data());
        break;
    case AXISFAULT_OverCurrentFault:
        retval += ito::RetVal(ito::retError, 0, tr("The average motor current exceeded the threshold specified by the AverageCurrentThreshold and AverageCurrentTime parameters.").toLatin1().data());
        break;
    case AXISFAULT_CwEndOfTravelLimitFault:
        retval += ito::RetVal(ito::retError, 0, tr("The axis encountered the clockwise (positive) end-of-travel limit switch.").toLatin1().data());
        break;
    case AXISFAULT_CcwEndOfTravelLimitFault:
        retval += ito::RetVal(ito::retError, 0, tr("The axis encountered the counter-clockwise (negative) end-of-travel limit switch.").toLatin1().data());
        break;
    case AXISFAULT_CwSoftwareLimitFault:
        retval += ito::RetVal(ito::retError, 0, tr("The axis was commanded to move beyond the position specified by the SoftwareLimitHigh parameter.").toLatin1().data());
        break;
    case AXISFAULT_CcwSoftwareLimitFault:
        retval += ito::RetVal(ito::retError, 0, tr("The axis was commanded to move beyond the position specified by the SoftwareLimitLow parameter.").toLatin1().data());
        break;
    case AXISFAULT_AmplifierFault:
        retval += ito::RetVal(ito::retError, 0, tr("The amplifier for this axis exceeded its maximum current rating or experienced an internal error.").toLatin1().data());
        break;
    case AXISFAULT_PositionFeedbackFault:
        retval += ito::RetVal(ito::retError, 0, tr("The drive detected a problem with the feedback device specified by the PositionFeedbackType and PositionFeedbackChannel parameters.").toLatin1().data());
        break;
    case AXISFAULT_VelocityFeedbackFault:
        retval += ito::RetVal(ito::retError, 0, tr("The drive detected a problem with the feedback device specified by the VelocityFeedbackType and VelocityFeedbackChannel parameters.").toLatin1().data());
        break;
    case AXISFAULT_HallSensorFault:
        retval += ito::RetVal(ito::retError, 0, tr("The drive detected an invalid state (all high or all low) for the Hall-effect sensor inputs on this axis.").toLatin1().data());
        break;
    case AXISFAULT_MaxVelocityCommandFault:
        retval += ito::RetVal(ito::retError, 0, tr("The commanded velocity is more than the velocity command threshold. Before the axis is homed, this threshold is specified by the VelocityCommandThresholdBeforeHome parameter. After the axis is homed, this threshold is specified by the VelocityCommandThreshold parameter.").toLatin1().data());
        break;
    case AXISFAULT_EmergencyStopFault:
        retval += ito::RetVal(ito::retError, 0, tr("The emergency stop sense input, specified by the ESTOPFaultInput parameter, was triggered.").toLatin1().data());
        break;
    case AXISFAULT_VelocityErrorFault:
        retval += ito::RetVal(ito::retError, 0, tr("The absolute value of the difference between the velocity command and the velocity feedback exceeded the threshold specified by the VelocityErrorThreshold parameter.").toLatin1().data());
        break;
    case AXISFAULT_ExternalFault:
        retval += ito::RetVal(ito::retError, 0, tr("The external fault input, specified by the ExternalFaultAnalogInput or ExternalFaultDigitalInput parameters, was triggered.").toLatin1().data());
        break;
    case AXISFAULT_MotorTemperatureFault:
        retval += ito::RetVal(ito::retError, 0, tr("The motor thermistor input was triggered, which indicates that the motor exceeded its maximum recommended operating temperature.").toLatin1().data());
        break;
    case AXISFAULT_AmplifierTemperatureFault:
        retval += ito::RetVal(ito::retError, 0, tr("The amplifier exceeded its maximum recommended operating temperature.").toLatin1().data());
        break;
    case AXISFAULT_EncoderFault:
        retval += ito::RetVal(ito::retError, 0, tr("The encoder fault input on the motor feedback connector was triggered.").toLatin1().data());
        break;
    case AXISFAULT_CommunicationLostFault:
        retval += ito::RetVal(ito::retError, 0, tr("One or more of the drives on the network lost communications with the controller.").toLatin1().data());
        break;
    case AXISFAULT_FeedbackScalingFault:
        retval += ito::RetVal(ito::retError, 0, tr("The difference between the position feedback and the scaled (adjusted by GainKv) velocity feedback exceeds the threshold specified by the PositionErrorThreshold parameter.").toLatin1().data());
        break;
    case AXISFAULT_MarkerSearchFault:
        retval += ito::RetVal(ito::retError, 0, tr("The distance that the axis moved while searching for the marker exceeded the threshold specified by the MarkerSearchThreshold parameter in axis %1 (%2).").arg(axisID).arg(m_axisNames[axisID]).toLatin1().data());
        break;
/*    case AXISFAULT_VoltageClampFault:
        retval += ito::RetVal(ito::retError, 0, tr("The commanded voltage output exceeded the low or high PiezoVoltageClamp parameter.").toLatin1().data());
        break;*/
    default:
        retval += ito::RetVal::RetVal(ito::retError, 0, tr("Unknown axisFault value: %1 in axis %2 (%3)").arg(axisFault).arg(axisID).arg(m_axisNames[axisID]).toLatin1().data());
    }
    return retval;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal AerotechEnsemble::requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos)
{
    ito::RetVal retValue(ito::retOk);
    retValue += doUpdatePosAndState(m_allAxesVector);

    if (sendCurrentPos)
    {
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

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void AerotechEnsemble::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            QObject::connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), widget, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            QObject::connect(this, SIGNAL(targetChanged(QVector<double>)), widget, SLOT(targetChanged(QVector<double>)));
            requestStatusAndPosition(true, true);
        }
        else
        {
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), widget, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), widget, SLOT(targetChanged(QVector<double>)));
        }
    }
}