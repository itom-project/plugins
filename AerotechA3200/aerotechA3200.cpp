/* ********************************************************************
    Plugin "AerotechA3200" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#include "AerotechA3200.h"

#include "pluginVersion.h"
#include "gitVersion.h"
#include <string.h>
#include <math.h>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmutex.h>
#include <qwaitcondition.h>

#include "errno.h"

#include "dialogAerotechA3200.h"    //! This is the configuration dialog
#include "dockWidgetAerotechA3200.h"    //! This is the control dialog

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200Interface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(AerotechA3200)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200Interface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(AerotechA3200)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechA3200Interface::AerotechA3200Interface(QObject *parent)
{
    m_type = ito::typeActuator;
    setObjectName("AerotechA3200");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char* docstring = \
"This plugin allows communicating with controllers of type A3200 of company Aerotech. \n\
For details please check C:\\A3200\\MANUAL.";*/

    m_description = QObject::tr("Plugin for the A3200-controller of Aerotech");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"This plugin allows communicating with controllers of type A3200 of company Aerotech. \n\
For details please check C:\\A3200\\MANUAL.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;


    ito::Param param = ito::Param("axes", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("list of axes IDs that are enabled (0..2). The first ID then obtains index 0, the second ID index 1... [default: empty list, all available axes are connected]").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechA3200Interface::~AerotechA3200Interface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal AerotechA3200::showConfDialog(void)
{
    RetVal retValue(retOk);

    dialogAerotechA3200 *confDialog = new dialogAerotechA3200(qobject_cast<ito::AddInActuator*>(this), m_axisNames);    // Create dialog
    confDialog->setVals(&m_params);    // Set up dialog parameters
    if (confDialog->exec())    // Is dialog is endet with exec and not with cancel
    {
        confDialog->getVals(&m_params);    // get parameters from dialog
    }
    delete confDialog;    // destray dialog

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechA3200::AerotechA3200() :
    AddInActuator(),
    m_pAerotechA3200Wid(NULL),
    hAerCtrl(NULL)
{
    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");    // To enable the program to transmit parameters via signals - slot connections
    qRegisterMetaType<QVector<bool> >("QVector<bool>");
    qRegisterMetaType<QVector<double> >("QVector<double>");
    //ito::tParam;    // Set up the parameter list
    m_params.insert("name", Param("name", ParamBase::String | ParamBase::In | ParamBase::Readonly, "AerotechA3200", NULL));

    m_params.insert("async", Param("async", ParamBase::Int, 0, 1, 0, tr("asynchronous move (1), synchronous (0) [default]").toLatin1().data()));
    m_async = m_params["async"].getVal<int>();
    m_params.insert("numAxis", Param("numAxis", ParamBase::Int | ParamBase::In | ParamBase::Readonly, 0, 3, 3, "number of connected axes"));

    m_params.insert("scaleFactor", Param("scaleFactor", ParamBase::Double | ParamBase::Readonly, 0.0, 1000000.0, 0.0, tr("scale factor of connected controller, counts per metric unit").toLatin1().data()));

    m_params.insert("clearoffset", Param("clearoffset", ParamBase::Int, 0, 0, 0, tr("reset the offsets to zero, back to absolute coordinate").toLatin1().data()));
    m_params.insert("finished", Param("finished", ParamBase::Int | ParamBase::Readonly, 0, 1, 0, tr("check if the motion of every axis is finished").toLatin1().data()));
    m_params.insert("stop", Param("stop", ParamBase::Int, 0, 0, 0, tr("executes an immediate stop").toLatin1().data()));
    m_params.insert("acknowledge", Param("acknowledge", ParamBase::Int, 0, 0, 0, tr("acknowledge the errors of connected controller").toLatin1().data()));

    m_params.insert("xenabled", Param("xenabled", ParamBase::Int, 0, 1, 0, tr("check if x-axis is enabled").toLatin1().data()));
    m_params.insert("yenabled", Param("yenabled", ParamBase::Int, 0, 1, 0, tr("check if y-axis is enabled").toLatin1().data()));
    m_params.insert("zenabled", Param("zenabled", ParamBase::Int, 0, 1, 0, tr("check if z-axis is enabled").toLatin1().data()));


    double axisSpeeds[] = {20.0, 20.0, 20.0}; //mm/s, naemlich "rate" in altem Skript mcpp
    Param param = Param("speed", ParamBase::DoubleArray, NULL, tr("speed of every axis").toLatin1().data());
    param.setVal<double*>(axisSpeeds, 3);
    m_params.insert("speed", param);

    m_currentPos.fill(0.0, 3);
    m_currentStatus.fill(0, 3);
    m_targetPos.fill(0.0, 3);



//#?#drunter zu modifizieren######

    // // This is for the docking widget
    // //now create dock widget for this plugin
    m_pAerotechA3200Wid = new DockWidgetAerotechA3200(m_params, getID(), this);    // Create a new non-modal dialog
//    m_pAerotechEnsembleWid = new DockWidgetAerotechEnsemble(this);    // Create a new non-modal dialog

    //Marc: connect(this, SIGNAL(statusUpdated(QVector<bool>, QVector<bool>, QVector<double>, QVector<double>, QVector<bool>)), USBMotion3XIIIWid, SLOT(statusUpdated(QVector<bool>, QVector<bool>, QVector<double>, QVector<double>, QVector<bool>)));
    //Marc: connect(this, SIGNAL(targetsChanged(QVector<bool>, QVector<double>)), USBMotion3XIIIWid, SLOT(targetsChanged(QVector<bool>, QVector<double>)));

//    connect(m_pAerotechEnsembleWid, SIGNAL(setAbsTargetDegree(double, double, double)), this, SLOT(setAbsTargetDegree(double, double, double)));
//    connect(m_pAerotechEnsembleWid, SIGNAL(setRelTargetDegree(unsigned int, double)), this, SLOT(setRelTargetDegree(unsigned int, double)));

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_pAerotechA3200Wid);    // Give the widget a name ..)

//    connect(m_pAerotechEnsembleWid, SIGNAL(MoveRelative(const int,const double ,ItomSharedSemaphore*)), this, SLOT(setPosRel(const int,const double, ItomSharedSemaphore*)));
//    connect(m_pAerotechEnsembleWid, SIGNAL(MoveAbsolute(QVector<int>, QVector<double>, ItomSharedSemaphore*)), this, SLOT(setPosAbs(QVector<int>, QVector<double>, ItomSharedSemaphore*)));
    connect(m_pAerotechA3200Wid, SIGNAL(MotorTriggerStatusRequest(bool,bool)), this, SLOT(RequestStatusAndPosition(bool, bool)));
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), m_pAerotechA3200Wid, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(this, SIGNAL(dockWidgetAerotechA3200Init(QMap<QString, ito::Param>, QStringList)), m_pAerotechA3200Wid, SLOT(init(QMap<QString, ito::Param>, QStringList)));
    // till here
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechA3200::~AerotechA3200()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::checkError(int a3200ReturnValue)
{
    if (a3200ReturnValue == AERSVR_NO_FIRECARD)
    {
        a3200ReturnValue = AERERR_NOERR;   // no firewire card in PC
    }
    if (a3200ReturnValue == AERSVR_SELF_ID_NONE)
    {
        a3200ReturnValue = AERERR_NOERR;   // no drives connected to firewire card
    }
    if (a3200ReturnValue == AERSVR_LINK_LAYER_NONE)
    {
        a3200ReturnValue = AERERR_NOERR;   // all drives we are connected to are powered down
    }
    if (a3200ReturnValue)
    {
        if (AerErrGetMessage(a3200ReturnValue, szMsg, MAX_TEXT_LEN, FALSE)!= NULL)
        {
            return ito::RetVal::format(ito::retError, 0, tr("A3200 error: %s").toLatin1().data(), szMsg);
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Unknown A3200 error since the error message was too long").toLatin1().data());
        }
    }
    else
    {
        return ito::retOk;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::getAxisMask(const int *axes, const int numAxes, AXISMASK &mask)
{
    ito::RetVal retValue;
    mask = NULL;
    int nums = numAxes;
    for (int i = 0; i < nums; ++i)
    {
        switch(axes[i])
        {
        case 0:
            mask = mask | AXISMASK_2;
            break;
        case 1:
            mask = mask | AXISMASK_3;
            break;
        case 2:
            mask = mask | AXISMASK_1;
            break;
        default:
            retValue += ito::RetVal::format(ito::retError, 0, tr("The axis number %i is not supported. Allowed range [0, 3]").toLatin1().data(), axes[i]);
            break;
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::getAxisMask2(const QVector<int> &axesIndices, AXISMASK &mask)
{
    ito::RetVal retValue;
    mask = NULL;

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
                mask = mask | AXISMASK_2;
                break;
            case 1:
                mask = mask | AXISMASK_3;
                break;
            case 2:
                mask = mask | AXISMASK_1;
                break;
            default:
                retValue += ito::RetVal::format(ito::retError, 0, tr("The axis number %i is not supported. Allowed range [0, 3]").toLatin1().data(), m_enabledAxes[index]);
                break;
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    RetVal retValue(retOk);
    char *temp = NULL;

    //enable all axes that are contained in the first optional parameter
    int *axes = paramsOpt->at(0).getVal<int*>();
    int axesLength = paramsOpt->at(0).getLen();
    QVector<int> axesIDs;

    /*
                           AXISMASK   axis(init bei itom)      AXISINDEX
                Z             1             2                      1
                X             2             0                      2
                Y             3             1                      3
    */
    for (int i = 0; i < axesLength; ++i)
    {
        axesIDs.append(axes[i]);
        m_allAxesVector << i;
    }

    m_currentStatus.fill(0, axesLength);


    AXISMASK axisMask;

    retValue += getAxisMask(axes, axesLength, axisMask);

    if (!retValue.containsError())
    {
        retValue += checkError(AerSysInitialize(NULL, NULL, 1, &hAerCtrl, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL));
    }
    if (!retValue.containsError())
    {
        AXISMASK availableMask;

        //It is naturally better to call a function regarding the A3200, but three 'available' axes are sufficient.
        availableMask= AXISMASK_1 | AXISMASK_2 | AXISMASK_3;

        if (!retValue.containsError())
        {
            if (axesLength <= 0) //no specific axes given, all available are taken
            {
                axesLength = 0;
                axisMask = availableMask;
                if (axisMask & AXISMASK_2)
                {
                    axesLength++;
                    axesIDs.append(0);//x-achse
                }
                if (axisMask & AXISMASK_3)
                {
                    axesLength++;
                    axesIDs.append(1);//y-achse
                }
                if (axisMask & AXISMASK_1)
                {
                    axesLength++;
                    axesIDs.append(2);//z-achse
                }
            }

            for (int i = 0; i < axesLength; ++i)
            {
                m_allAxesVector << i;
            }

            QByteArray name(256, '\0');
            foreach(const int &i, axesIDs)
            {
                if (i== 0)
                {
                    m_axisNames.append("X");
                }
                if (i== 1)
                {
                    m_axisNames.append("Y");
                }
                if (i== 2)
                {
                    m_axisNames.append("Z");
                }
            }

            if ((availableMask | axisMask) != availableMask)
            {
                retValue += ito::RetVal::format(ito::retError, 0, tr("Not all desired axes are connected to the controller (desired: %i, available: %i)").toLatin1().data(), axisMask, availableMask);
            }

            if (!retValue.containsError())
            {
                ito::RetVal retValueTemp = checkError(AerMoveMEnable(hAerCtrl, axisMask));

                if (retValueTemp.containsError())
                {
                    retValue += checkError(AerSysFaultAck(hAerCtrl, axisMask, TASKMASK_ALL, 0));

                    if (!retValue.containsError())
                    {
                        retValue += checkError(AerMoveMEnable(hAerCtrl, axisMask));
                        for(int i = 0; i < axesLength; ++i)
                        {
                            m_currentStatus[i] |= ito::actuatorEnabled;
                        }
                    }
                }
                else
                {
                    for(int i = 0; i < axesLength;++i)
                    {
                        m_currentStatus[i] |= ito::actuatorEnabled;
                    }
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

        m_offset.append(0.0);
        m_offset.append(0.0);
        m_offset.append(0.0);

        //remember enabled axes
        for (int i = 0; i < axesLength; ++i)
        {
            m_enabledAxes.append(axesIDs[i]);
            _axes[i] = i;
        }

        double factor;
        ito::RetVal retTemp =  checkError(AerParmGetValue(hAerCtrl, AER_PARMTYPE_AXIS, AXISINDEX_1, AXISPARM_CntsPerMetricUnit, 0, &factor));
        if (retTemp.containsError())
        {
            retValue += checkError(AerSysFaultAck(hAerCtrl, axisMask, TASKMASK_ALL, 0));
            retValue +=  checkError(AerParmGetValue(hAerCtrl, AER_PARMTYPE_AXIS, AXISINDEX_1, AXISPARM_CntsPerMetricUnit, 0, &factor));
        }
        else
        {
            retValue += retTemp;
        }
        m_params["scaleFactor"].setVal<double>(factor);

        ito::RetVal retValTemp =  doUpdatePosAndState(_axes);
        if (retValTemp.containsError())
        {
            retValue += checkError(AerSysFaultAck(hAerCtrl, axisMask, TASKMASK_ALL, 0));
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

    for (int i = 0; i < axesLength; ++i)
    {
        switch(axesIDs[i])
        {
        case 0:
            m_params ["xenabled"].setVal<int>(1);
            break;
        case 1:
            m_params ["yenabled"].setVal<int>(1);
            break;
        case 2:
            m_params ["zenabled"].setVal<int>(1);
            break;
        }
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
    emit dockWidgetAerotechA3200Init(m_params, m_axisNames);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (hAerCtrl)
    {
        retValue += checkError(AerSysStop(hAerCtrl));
        m_enabledAxes.clear();
        hAerCtrl = NULL;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

        if (key == "finished")
        {
            //returns 0 if inpos bit is set
            DWORD d = 0;
            d = AerMoveMWaitDone(hAerCtrl, (AXISMASK_1 | AXISMASK_2 | AXISMASK_3), AERMOVEWAIT_CHECK_ONCE , 1);
            if (d == 0)
            {
                it->setVal<double>(1.0);
            }
            else
            {
                it->setVal<double>(0.0);
            }
        }
        else if (key == "stop")
        {
            retValue += checkError(AerMoveAbort(hAerCtrl, AXISINDEX_1));
            retValue += checkError(AerMoveAbort(hAerCtrl, AXISINDEX_2));
            retValue += checkError(AerMoveAbort(hAerCtrl, AXISINDEX_3));
        }
        else if (key == "acknowledge")
        {
            retValue += checkError(AerSysFaultAck(hAerCtrl, AXISMASK_ALL, TASKMASK_ALL, 0));
        }
        else if (key == "clearoffset")
        {
            m_offset[0] = 0.0;
            m_offset[1] = 0.0;
            m_offset[2] = 0.0;
        }

        if (!retValue.containsError())
        {
            *val = apiGetParam(*it, hasIndex, index, retValue);
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
ito::RetVal AerotechA3200::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    int ziel;

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
        else if (key == "xenabled")
        {
            ziel = val->getVal<int>();
            enabledisable(0, ziel);
            m_params["xenabled"].setVal<int>(ziel);
        }
        else if (key == "yenabled")
        {
            ziel = val->getVal<int>();
            enabledisable(1, ziel);
            m_params["yenabled"].setVal<int>(ziel);
        }
        else if (key == "zenabled")
        {
            ziel = val->getVal<int>();
            enabledisable(2, ziel);
            m_params["zenabled"].setVal<int>(ziel);
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
ito::RetVal AerotechA3200::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
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
    else if (hAerCtrl == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech 3200 Handle is NULL").toLatin1().data());
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

            //starts a small worker thread with a timer that regularly calls doAliveTimer to trigger the alive thread such that itom do not run into
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
                retValue += checkError(AerMoveMHome(hAerCtrl, mask));
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
void AerotechA3200::doAliveTimer()
{
    setAlive();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    return setOrigin(QVector<int>(1, axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
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

        if (!retValue.containsError())
        {
            retValue += doUpdatePosAndState(axis);
            foreach(const int axisId, axis)
            {
                m_offset[axisId] = m_offset[axisId] - m_currentPos[axisId];
                m_currentPos[axisId] = 0.0;
                m_targetPos[axisId] = 0.0;
            }
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
ito::RetVal AerotechA3200::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
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
ito::RetVal AerotechA3200::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if ((axis >= m_enabledAxes.size()) || (axis >= 3) || axis < 0)
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
ito::RetVal AerotechA3200::getPos(QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += doUpdatePosAndState(axis);

    for (int naxis = 0; naxis < axis.size(); naxis++)
    {
        if ((axis[naxis] >= m_enabledAxes.size()) || (axis[naxis] >= 3) || axis[naxis] < 0)
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
ito::RetVal AerotechA3200::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosAbs(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::setPosAbs(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
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
    else if (hAerCtrl == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech 3200 Handle is NULL").toLatin1().data());
    }
    else
    {
        AXISMASK mask;

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
            double factor = m_params["scaleFactor"].getVal<double>();

            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                posArray[naxis] = pos[naxis];
                m_targetPos[axis[naxis]] = pos[naxis];
                speedArray[naxis] = paramSpeed[axis[naxis]];
            }

            if (axis.size() > 0)
            {
                // ---AerMoveMAbsolute() nutzt PLONG(machine steps)----
                LONG _posArray[1];
                DWORD _speedArray[1];
                for(int i = 0; i < axis.size(); ++i)
                {
                    _posArray[0]=(LONG) ((posArray[i] - m_offset[axis[i]])  * factor);
                    _speedArray[0]=(DWORD) (speedArray[i] * factor);
                    //test = m_enabledAxes[axis[i]];
                    //retValue += getAxisMask2(axis[i], mask);
                    switch(m_enabledAxes[axis[i]])
                    {
                        case 0:
                            mask = AXISMASK_2;
                            break;
                        case 1:
                            mask = AXISMASK_3;
                            break;
                        case 2:
                            mask = AXISMASK_1;
                            break;
                    }
                    retValue += checkError(AerMoveMAbsolute(hAerCtrl, mask, _posArray, _speedArray));
                }
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
ito::RetVal AerotechA3200::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    return setPosRel(QVector<int>(1, axis), QVector<double>(1, pos), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::setPosRel(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
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
    else if (hAerCtrl == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech 3200 Handle is NULL").toLatin1().data());
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
            double factor = m_params["scaleFactor"].getVal<double>();

            for (int naxis = 0; naxis < axis.size(); naxis++)
            {
                posArray[naxis] = pos[naxis];
                m_targetPos[axis[naxis]] = m_currentPos[axis[naxis]] + pos[naxis];
                speedArray[naxis] = paramSpeed[axis[naxis]];
            }

            if (axis.size() > 0)
            {
                LONG _posArray[1];
                DWORD _speedArray[1];
                for(int i = 0; i < axis.size(); ++i)
                {
                    _posArray[0]=(LONG) (posArray[i] * factor);
                    _speedArray[0]=(DWORD) (speedArray[i] * factor);
                    switch(m_enabledAxes[axis[i]])
                    {
                        case 0:
                            mask = AXISMASK_2;
                            break;
                        case 1:
                            mask = AXISMASK_3;
                            break;
                        case 2:
                            mask = AXISMASK_1;
                            break;
                    }
                    retValue += checkError(AerMoveMIncremental(hAerCtrl, mask, _posArray, _speedArray));
                }
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
ito::RetVal AerotechA3200::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
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

    QTime timer;
    timer.start();
    QMutex waitMutex;
    QWaitCondition waitCondition;
    long delay = 200; //[ms]

    while (!done && !timeout)
    {
        if (!done && isInterrupted())
        {
            if (hAerCtrl != NULL)
            {
                AXISMASK AxisMask = AXISMASK_1 | AXISMASK_2 | AXISMASK_3;
                retVal += checkError(AerMoveMAbort(hAerCtrl, AxisMask));
            }

            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
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
            if (timer.elapsed() > timeoutMS)
            {
                timeout = true;
            }
        }

        bool bMove = false;
        retVal += doUpdatePosAndState(_axis);

        for (int i = 0; i < _axis.size(); i++)
        {
            if (m_currentStatus[_axis[i]] & ito::actuatorMoving)
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
ito::RetVal AerotechA3200::doUpdatePosAndState(const QVector<int> &axes)
{
    ito::RetVal retval;
    if (hAerCtrl != NULL)
    {
        //bool bRet = TRUE;
        //bool bMove = FALSE;
        double pDouble[3];//0: pos, 1: status, 2: fault
        //AXISMASK mAxis;
        double factor;

        factor = m_params["scaleFactor"].getVal<double>();//counts per metric units
        foreach(const int &axis, axes)
        {
            AXISINDEX axisIndex = (AXISINDEX)(AXISINDEX_1 + m_enabledAxes[axis]);
            /*
                             Maske         axis(init)      INDEX
                Z             1             2                1
                X             2             0                2
                Y             3             1                3
            */
            switch(axisIndex)
            {
            case 0:
                //mAxis = AXISMASK_2;
                axisIndex = AXISINDEX_2;
                break;
            case 1:
                //mAxis = AXISMASK_3;
                axisIndex = AXISINDEX_3;
                break;
            case 2:
                //mAxis = AXISMASK_1;
                axisIndex = AXISINDEX_1;
                break;
            }
            //retval += checkError(AerStatusGetAxisInfoPosition(hAerCtrl, mAxis, (DWORD)1, & pDouble[0], NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL));
            retval += checkError(AerParmGetValue(hAerCtrl, AER_PARMTYPE_AXIS, axisIndex, AXISPARM_PositionCnts, 0, & pDouble[0]));
            retval += checkError(AerParmGetValue(hAerCtrl, AER_PARMTYPE_AXIS, axisIndex, AXISPARM_AxisStatus, 0, & pDouble[1]));
            retval += checkError(AerParmGetValue(hAerCtrl, AER_PARMTYPE_AXIS, axisIndex, AXISPARM_Fault, 0, & pDouble[2]));

            if (!retval.containsError())
            {
                m_currentPos[axis] = m_offset[axis] + pDouble[0]/factor;
                DWORD State = (DWORD)(pDouble[1]);

                if (State & MAXS_STATUS_MOVEDONE)
                {
                    setStatus(m_currentStatus[axis], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                }
                else if (State & MAXS_STATUS_DRIVECONTROLLED)
                {
                    setStatus(m_currentStatus[axis], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                }
                if (State & MAXS_STATUS_CLAMPED)
                {
                    setStatus(m_currentStatus[axis], ito::actuatorEndSwitch, ito::actMovingMask | ito::actStatusMask);
                }
                if (State & MAXS_STATUS_INTERRUPT)
                {
                    setStatus(m_currentStatus[axis], ito::actuatorInterrupted, ito::actSwitchesMask | ito::actStatusMask);
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
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::axisFaultToRetVal(int axisFault, int axisID)//
{
    ito::RetVal retval;
    switch (axisFault)
    {
    case 0:
        break;
    case 0x00000001:
        retval += ito::RetVal(ito::retError, 0, tr("Position error").toLatin1().data());
        break;
    case 0x00000002:
        retval += ito::RetVal(ito::retError, 0, tr("Over current").toLatin1().data());
        break;
    case 0x00000004:
        retval += ito::RetVal(ito::retError, 0, tr("CW EOT limit").toLatin1().data());
        break;
    case 0x00000008:
        retval += ito::RetVal(ito::retError, 0, tr("CCW EOT limit").toLatin1().data());
        break;
    case 0x00000010:
        retval += ito::RetVal(ito::retError, 0, tr("CW soft limit").toLatin1().data());
        break;
    case 0x00000020:
        retval += ito::RetVal(ito::retError, 0, tr("CCW soft limit").toLatin1().data());
        break;
    case 0x00000040:
        retval += ito::RetVal(ito::retError, 0, tr("Amplifier fault").toLatin1().data());
        break;
    case 0x00000080:
        retval += ito::RetVal(ito::retError, 0, tr("Position fbk").toLatin1().data());
        break;
    case 0x00000100:
        retval += ito::RetVal(ito::retError, 0, tr("Velocity fbk").toLatin1().data());
        break;
    case 0x00000200:
        retval += ito::RetVal(ito::retError, 0, tr("Hall fault").toLatin1().data());
        break;
    case 0x00000400:
        retval += ito::RetVal(ito::retError, 0, tr("Max velocity cmd").toLatin1().data());
        break;
    case 0x00000800:
        retval += ito::RetVal(ito::retError, 0, tr("ESTOP fault").toLatin1().data());
        break;
    case 0x00001000:
        retval += ito::RetVal(ito::retError, 0, tr("Velocity error").toLatin1().data());
        break;
    case 0x00002000:
        retval += ito::RetVal(ito::retError, 0, tr("Task fault").toLatin1().data());
        break;
    case 0x00004000:
        retval += ito::RetVal(ito::retError, 0, tr("Probe fault").toLatin1().data());
        break;
    case 0x00008000:
        retval += ito::RetVal(ito::retError, 0, tr("Auxiliary fault").toLatin1().data());
        break;
    case 0x00010000:
        retval += ito::RetVal(ito::retError, 0, tr("Safe zone fault").toLatin1().data());
        break;
    case 0x00020000:
        retval += ito::RetVal(ito::retError, 0, tr("Motor temp").toLatin1().data());
        break;
    case 0x00040000:
        retval += ito::RetVal(ito::retError, 0, tr("Amplifier temp").toLatin1().data());
        break;
    case 0x00080000:
        retval += ito::RetVal(ito::retError, 0, tr("Ext encoder fault").toLatin1().data());
        break;
    case 0x00100000:
        retval += ito::RetVal(ito::retError, 0, tr("Comm lost fault").toLatin1().data());
        break;
    default:
        retval += ito::RetVal::RetVal(ito::retError, 0, tr("Unknown axisFault value: %1 in axis %2 (%3)").arg(axisFault).arg(axisID).arg(m_axisNames[axisID]).toLatin1().data());
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::RequestStatusAndPosition(bool sendActPosition, bool sendTargetPos)
{
    ito::RetVal retValue(ito::retOk);
    retValue += doUpdatePosAndState(m_allAxesVector);

    if (sendActPosition)
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
void AerotechA3200::dockWidgetVisibilityChanged(bool visible)
{
    if (m_pAerotechA3200Wid)
    {
        if (visible)
        {
            QObject::connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            QObject::connect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
            RequestStatusAndPosition(true, true);
        }
        else
        {
            QObject::disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), getDockWidget()->widget(), SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            QObject::disconnect(this, SIGNAL(targetChanged(QVector<double>)), getDockWidget()->widget(), SLOT(targetChanged(QVector<double>)));
        }
    }
    /*if (USBMotion3XIIIWid)
    {
        if (visible)
        {
            connect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), USBMotion3XIIIWid, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            connect(this, SIGNAL(targetChanged(QVector<double>)), USBMotion3XIIIWid, SLOT(targetChanged(QVector<double>)));
        }
        else
        {
            disconnect(this, SIGNAL(actuatorStatusChanged(QVector<int>, QVector<double>)), USBMotion3XIIIWid, SLOT(actuatorStatusChanged(QVector<int>, QVector<double>)));
            disconnect(this, SIGNAL(targetChanged(QVector<double>)), USBMotion3XIIIWid, SLOT(targetChanged(QVector<double>)));
        }
    }*/
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechA3200::enabledisable(int axis, int ziel)
{
    ito::RetVal retValue;

    if (hAerCtrl == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech 3200 Handle is NULL").toLatin1().data());
    }
    else
    {
        AXISMASK axisMask;
        retValue += getAxisMask(&axis, 1, axisMask);

        if (!retValue.containsError())
        {
            if (ziel)
            {
                ito::RetVal retValueTemp = checkError(AerMoveMEnable(hAerCtrl, axisMask));

                if (retValueTemp.containsError())
                {
                    retValue += checkError(AerSysFaultAck(hAerCtrl, axisMask, TASKMASK_ALL, 0));

                    if (!retValue.containsError())
                    {
                        retValue += checkError(AerMoveMEnable(hAerCtrl, axisMask));

                    }
                }
                else
                {
                    retValue += retValueTemp;
                }
                for(int i = 0; i < m_enabledAxes.size(); ++i)
                {
                    if (m_enabledAxes[i] == axis)
                    {
                        m_currentStatus[i] |= ito::actuatorEnabled;
                        break;
                    }
                }
            }
            else
            {
                ito::RetVal retValueTemp = checkError(AerMoveMDisable(hAerCtrl, axisMask));

                if (retValueTemp.containsError())
                {
                    retValue += checkError(AerSysFaultAck(hAerCtrl, axisMask, TASKMASK_ALL, 0));

                    if (!retValue.containsError())
                    {
                        retValue += checkError(AerMoveMDisable(hAerCtrl, axisMask));
                    }
                }
                else
                {
                    retValue += retValueTemp;
                }
                for(int i = 0; i < m_enabledAxes.size(); ++i)
                {
                    if (m_enabledAxes[i] == axis)
                    {
                        m_currentStatus[i] |= ito::actuatorEnabled;
                        break;
                    }
                }
            }
        }
    }

    return retValue;
}
