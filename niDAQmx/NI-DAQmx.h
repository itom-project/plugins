/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
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

#include "NIDAQmx.h" // This is the NI C Library .h file, which is distinquised from this file by lacking a hypen between "NI" and "DAQ"
//#include "NIDAQmx-LibHeader.h"  // include NI-DAQmx Library functions       
#include "NI-PeripheralClasses.h" // Classes that encapsulate general stuff like channels and tasks

#include "NI-DAQmxError.h"
#include <QRegularExpression>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    niDAQmxInterface 
  *
  *\brief    Interface-Class for niDAQmx-Class
  *
  *    \sa    AddInDataIO, niDAQmx
  *
  */
class niDAQmxInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        niDAQmxInterface();
        ~niDAQmxInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    niDAQmx

  */
class niDAQmx : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~niDAQmx();
        niDAQmx();
        
        
    public:
        friend class niDAQmxInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        ito::RetVal checkData(ito::DataObject *externalDataObject, int channels, int samples);

        bool m_isgrabbing; /*!< Check if acquire was executed */

        // These three bools are set true by the acquire method to indicate what kind of data is
        // received in the getVal method. The getVal method also resets the three bools to false

        bool m_aInIsAcquired = false;
        bool m_dInIsAcquired = false;
        bool m_cInIsAcquired = false;

        bool m_aOutIsAcquired = false;
        bool m_dOutIsAcquired = false;
        bool m_cOutIsAcquired = false;
       
        QMap<QString, niTask*> m_taskMap;
        niChannelList m_channels;
        ito::DataObject m_data;
		QString m_configForTesting;
        
        // Read-functions
        ito::RetVal readAnalog(); /*!< Wait for acquired data */
        ito::RetVal readDigital(); /*!< Wait for acquired data */
        ito::RetVal readCounter(); /*!< Wait for acquired data */
        ito::RetVal retrieveData(int *, int *); //marc: new
        ito::RetVal manageTasks(); //dan: new
        ito::RetVal resetTask(QString); //dan: new

        // Write-functions
        ito::RetVal writeAnalog(const ito::DataObject *externalDataObject = NULL);
        ito::RetVal writeDigital(ito::DataObject *externalDataObject = NULL);
        ito::RetVal writeCounter(ito::DataObject *externalDataObject = NULL);
 
        // Exec definition and helper functions
        ito::RetVal help(QString *);
        
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
        //!< Wait for acquired picture, copy the Values to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        //!< 
        ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond = NULL);
        //!< 
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        
        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // niDAQmx_H
