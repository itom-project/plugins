/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
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

#ifndef niDAQmx_H
#define niDAQmx_H

//#include "opencv/cv.h"
#include "common/addInInterface.h"
#include "DataObject/dataobj.h"

#include <qsharedpointer.h>
#include <qmap.h>

#include "dialogNI-DAQmx.h"

// This is required, since NIDAQmx.h (in the NI library) for linux and apple systems defines
// int64 as long long int. This is incompatible with open_cv, which defines it as int64_t

#if defined(__linux__) || defined(__APPLE__)
    #ifndef _NI_int64_DEFINED_
    #define _NI_int64_DEFINED_
        typedef int64_t      int64;
    #endif
#endif

#include "NIDAQmx.h" // This is the NI C Library .h file, which is distinquised from this file by lacking a hyphen between "NI" and "DAQ"
#include "NI-PeripheralClasses.h" // Classes that encapsulate general stuff like channels and tasks

#include "NI-DAQmxError.h"


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    NiDAQmxInterface
  *
  *\brief    Interface-Class for NiDAQmx-Class
  *
  *    \sa    AddInDataIO, NiDAQmx
  *
  */
class NiDAQmxInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        NiDAQmxInterface();
        ~NiDAQmxInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    NiDAQmx

  */
class NiDAQmx : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~NiDAQmx();
        NiDAQmx();


    public:
        friend class NiDAQmxInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

        enum TaskSubTypes
        {
            Analog = 0x01,
            Digital = 0x02,
            Counter = 0x04,
            Input = 0x1000,
            Output = 0x2000
        };

        enum TaskType
        {
            AnalogInput = Analog | Input,
            AnalogOutput = Analog | Output,
            DigitalInput = Digital | Input,
            DigitalOutput = Digital | Output
        };

        enum NiTaskMode
        {
            NiTaskModeFinite = 0,
            NiTaskModeContinuous = 1
        };

        static QVector<void*> ActiveInstances;
        static QMutex ActiveInstancesAccess;

    private:
        static int InstanceCounter; /*!< used to give an auto-incremented taskName if no one is given as initialization parameter */

        ito::RetVal checkInternalData();
        ito::RetVal checkExternalDataToView(ito::DataObject *externalData);
        ito::RetVal scanForAvailableDevicesAndSupportedChannels();
        ito::RetVal stopTask();
        ito::RetVal deleteTask(); /*!< stops the task (if not yet done) and deletes it (if it has been created) */
        ito::RetVal createTask(); /*!< this method only creates a new task (without adding channels) and configures it with respect to the current set of parameters in m_params */
        ito::RetVal createChannelsForTask();
        ito::RetVal configTask();
        ito::RetVal startTask();
        ito::RetVal configLogging(bool emitParametersChanged = true);

        bool m_isgrabbing; /*!< Check if acquire was executed */
        bool m_nSampleEventRegistered;

        /*!< if true, a reference trigger is enabled (only for finite input tasks) and the finite task
        behaves like a continuous task (the stop is automatically executed by the reference trigger)*/
        bool m_refTriggerEnabled;

        bool m_taskStarted;
        int m_deviceStartedCounter; /*!< counts how often the device is started, every call to startDevice will increments this, stopDevice will decrement it. The task is really stopped if it drops to zero again. */
        ito::tDataType m_digitalChannelDataType; /*!< only relevant for digital ports, defines the necessary data type to hold the maximum number of lines that are connected for every port. (<= 8 lines : uint8, <= 16 : uint16, else int32) */
        ito::RetVal m_retrieveRetVal;
        ito::DataObject m_data; //source of data
        ito::DataObject m_dataView; //in finite task, this is a shallow copy of m_data, for continuous tasks this is always a ROI (as shallow copy of m_data) containing the really acquired values.

        TaskType m_taskType; /*!< type of this NI-DAQmx instance */
        QStringList m_availableDevices; /*!< list of all detected devices */
        QStringList m_supportedChannels; /*!< list of all detectable and supported channels (with respect to the TaskType of this instance */
        bool m_sampClkTimingConfigured;

        TaskHandle m_taskHandle;
        QList<NiBaseChannel*> m_channels;
        NiTaskMode m_taskMode;

        // Read-functions
        ito::RetVal readAnalog(int32 &readNumSamples); /*!< Wait for acquired data */
        ito::RetVal readDigital(int32 &readNumSamples); /*!< Wait for acquired data */
        ito::RetVal readCounter(int32 &readNumSamples); /*!< Wait for acquired data */
        ito::RetVal retrieveData();

        // Write-functions
        ito::RetVal writeAnalog(const ito::DataObject *dataObj);
        ito::RetVal writeDigital(const ito::DataObject *dataObj);
        ito::RetVal writeCounter(const ito::DataObject *dataObj);

    public slots:
        //!< Get ADC-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set ADC-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //!< Start the ADC to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //!< Stop the ADC to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //!< Softwaretrigger for the ADC
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //!< stop a continuous acquisition
        ito::RetVal stop(ItomSharedSemaphore *waitCond = NULL);
        //!< Wait for acquired picture, copy the Values to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        //!<
        ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond = NULL);
        //!<
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);

        void taskStopped(TaskHandle taskHandle);
};

#endif // niDAQmx_H
