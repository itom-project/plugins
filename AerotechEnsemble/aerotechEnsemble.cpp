#include "AerotechEnsemble.h"
#include "pluginVersion.h"
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
    ito::RetVal retValue(ito::retOk);

    AerotechEnsemble* newInst = new AerotechEnsemble();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AerotechEnsembleInterface::closeThisInst(ito::AddInBase **addInInst)
{
    RetVal retValue(retOk);

    if (*addInInst)
    {
        delete ((AerotechEnsemble *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechEnsembleInterface::AerotechEnsembleInterface(QObject *parent)
{
    m_type = ito::typeActuator;
    setObjectName("AerotechEnsemble");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char* docstring = \
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

    m_description = QObject::tr("Plugin for the Ensemble-controller of Aerotech");
    m_detaildescription = QObject::tr(docstring);
    m_author = "A. Bielke, M. Gronle, ITO, University Stuttgart, Jürgen Ortmann, Ortmann Digitaltechnik";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LGPL, The Aerotech Ensemble library belongs to Aerotech under their specific license.");
    m_aboutThis = QObject::tr("N.A.");     
    
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    ito::Param param = ito::Param("axes", ito::ParamBase::IntArray | ito::ParamBase::In, NULL, tr("list of axes IDs that are enabled (0..9). The first ID then obtains index 0, the second ID index 1... [default: empty list, all available axes are connected]").toAscii().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
AerotechEnsembleInterface::~AerotechEnsembleInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(AerotechEnsembleInterface, AerotechEnsembleInterface)


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal AerotechEnsemble::showConfDialog(void)
{

    RetVal retValue(retOk);

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

    m_params.insert("async", Param("async", ParamBase::Int, 0, 1, 0, tr("asynchronous move (1), synchronous (0) [default]").toAscii().data()));
    m_async = m_params["async"].getVal<int>();

    m_params.insert("numAxis", Param("numAxis", ParamBase::Int | ParamBase::In | ParamBase::Readonly, 0, 10, 0, "number of connected axes"));

    double axisSpeeds[] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; //mm/s
    Param param = Param("speed", ParamBase::DoubleArray, NULL, tr("speed of every axis").toAscii().data());
    param.setVal<double*>(axisSpeeds, 10);
    m_params.insert("speed", param);

    m_currentPos.fill(0.0, 10);
    m_currentStatus.fill(0, 10);
    m_targetPos.fill(0.0, 10);

    // memset(m_pos, 0, 10 * sizeof(double));

    // // This is for the docking widged
    // //now create dock widget for this plugin
    m_pAerotechEnsembleWid = new DockWidgetAerotechEnsemble(m_params, getID(), this);    // Create a new non-modal dialog
//    m_pAerotechEnsembleWid = new DockWidgetAerotechEnsemble(this);    // Create a new non-modal dialog

    //Marc: connect(this, SIGNAL(statusUpdated(QVector<bool>, QVector<bool>, QVector<double>, QVector<double>, QVector<bool>)), USBMotion3XIIIWid, SLOT(statusUpdated(QVector<bool>, QVector<bool>, QVector<double>, QVector<double>, QVector<bool>)));
    //Marc: connect(this, SIGNAL(targetsChanged(QVector<bool>, QVector<double>)), USBMotion3XIIIWid, SLOT(targetsChanged(QVector<bool>, QVector<double>)));
   
//    connect(m_pAerotechEnsembleWid, SIGNAL(setAbsTargetDegree(double, double, double)), this, SLOT(setAbsTargetDegree(double, double, double)));
//    connect(m_pAerotechEnsembleWid, SIGNAL(setRelTargetDegree(unsigned int, double)), this, SLOT(setRelTargetDegree(unsigned int, double)));

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, m_pAerotechEnsembleWid);    // Give the widget a name ..)
   
    connect(m_pAerotechEnsembleWid, SIGNAL(MoveRelative(const int,const double ,ItomSharedSemaphore*)), this, SLOT(setPosRel(const int,const double, ItomSharedSemaphore*)));
    connect(m_pAerotechEnsembleWid, SIGNAL(MoveAbsolute(QVector<int>, QVector<double>, ItomSharedSemaphore*)), this, SLOT(setPosAbs(QVector<int>, QVector<double>, ItomSharedSemaphore*)));
    connect(m_pAerotechEnsembleWid, SIGNAL(MotorTriggerStatusRequest(bool,bool)), this, SLOT(RequestStatusAndPosition(bool, bool)));
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), m_pAerotechEnsembleWid, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(this, SIGNAL(dockWidgetAerotechEnsembleInit(QMap<QString, ito::Param>, QStringList)), m_pAerotechEnsembleWid, SLOT(init(QMap<QString, ito::Param>, QStringList)));
    // till here
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
            return ito::RetVal::format(ito::retError, 0, "Ensemble error %i: %s", EnsembleGetLastError(), errorString);
        }
        else
        {
            return ito::RetVal(ito::retError, 0, "Unknown ensemble error since the error message was too long");
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
            retValue += ito::RetVal::format(ito::retError, 0, "The axis number %i is not supported. Allowed range [0, 9]", axes[i]);
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
            retValue += ito::RetVal::format(ito::retError, 0, "axis index %i is out of boundary [0, %i]", index, m_enabledAxes.size()-1);
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
                retValue += ito::RetVal::format(ito::retError, 0, "The axis number %i is not supported. Allowed range [0, 9]", m_enabledAxes[index]);
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
            retValue += ito::RetVal(ito::retError, 0, "Please make sure that only one controller is configured and connected");
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
            name = QString("%1.%2.%3 (%4)").arg(version.major).arg(version.minor, 2, 10, QLatin1Char('0')).arg(version.patch, 3, 10, QLatin1Char('0')).arg(version.build).toAscii(); //TODO 4.01.006
            m_params["libraryVersion"].setVal<char*>(name.data());
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
                retValue += ito::RetVal::format(ito::retError, 0, "Not all desired axes are connected to the controller (desired: %i, available: %i)", axisMask, availableMask);
            }

            if (!retValue.containsError())
            {
                retValue += checkError(EnsembleMotionEnable(m_pHandle, axisMask));
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

        retValue += doUpdatePosAndState(_axes);
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
        retValue += ito::RetVal(ito::retError, 0, tr("any axis is moving. Parameters cannot be set").toAscii().data());
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
                    retValue += ito::RetVal(ito::retError, 0, "given index is out of boundary");
                }
                else if (val->getType() != ito::ParamBase::Double)
                {
                    retValue += ito::RetVal(ito::retError, 0, "given value must be a double if an index is given.");
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
                    retValue += ito::RetVal(ito::retError, 0, "given value must be a double array");
                }
                else if (val->getLen() != arrayLength)
                {
                    retValue += ito::RetVal(ito::retError, 0, "length of given double array must correspond to number of axes");
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
    ito::RetVal retValue;

    retValue += ito::RetVal(ito::retError, 0, tr("not supported yet").toAscii().data());

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        
    }

    return retValue;
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
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toAscii().data());
    }
    else
    {
        foreach(const int axisId, axis)
        {
            if (m_enabledAxes.contains(axisId) == false)
            {
                retValue += ito::RetVal::format(ito::retError, 0, "axis %i is not enabled", axisId);
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
        retValue += ito::RetVal(ito::retError, 1, tr("axis index is out of bound").toAscii().data());
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
            retValue += ito::RetVal(ito::retError, 1, tr("at least one axis index is out of bound").toAscii().data());
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
        retValue += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toAscii().data());
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else if (m_pHandle == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, "Aerotech Ensemble Handle is NULL");
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
        retValue += ito::RetVal(ito::retError, 0, tr("Any motor axis is moving. The motor is locked.").toAscii().data());
        if (waitCond)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
        }
    }
    else if (m_pHandle == NULL)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Aerotech Ensemble Handle is NULL").toAscii().data());
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
    
    QTime timer;
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
            retVal += ito::RetVal(ito::retError, 0, tr("interrupt occurred").toAscii().data());
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
        doUpdatePosAndState(_axis);

        for (int i = 0; i < _axis.size(); i++) 
        {
            if (m_currentStatus[i] & ito::actuatorMoving)
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
        retVal += ito::RetVal(ito::retError, 9999, tr("timeout occurred").toAscii().data());
    }

    //100 ms damit die Achsen sich einpegeln können
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
ito::RetVal AerotechEnsemble::RequestStatusAndPosition(bool sendActPosition, bool sendTargetPos)
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
void AerotechEnsemble::dockWidgetVisibilityChanged(bool visible)
{
    if (m_pAerotechEnsembleWid)
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