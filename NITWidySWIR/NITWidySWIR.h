/* ********************************************************************
Plugin "NITWidySWIR" for itom software
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

#ifndef NITWIDYSWIR_H
#define NITWIDYSWIR_H

/**** Standard Include ****/
//#include <iostream>
//#include <sstream>

#include "common/addInGrabber.h"
//#include <qsharedpointer.h>
#include "dialogNITWidySWIR.h"
#include <qmutex.h>

namespace NITLibrary{
	class NITException;//forward declaration
	class NITCatalog;//forward declaration
	class NITManager;//forward declaration
	class NITDevice;//forward declaration
	//class NITToolbox;
}

class NITWidySWIR;

#include "NITObserver.h"
#include "NITFilter.h"

using namespace ito;

//----------------------------------------------------------------------------------------------------------------------------------
class NITWidySWIRInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        NITWidySWIRInterface();
        ~NITWidySWIRInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//-----------------------------------------------------------------------------------------------------------------------------------
class NITWidySWIRObserver : public NITLibrary::NITObserver
{
public:
	// Construct
	NITWidySWIRObserver(NITWidySWIR *camera) :
		m_pCamera(camera)
	{
	}

	// Destruct
    ~NITWidySWIRObserver()
    {
    }

protected:
	// Implementation
	void onStart(){ /*Initialisation, reset, ...*/ }

	void onNewFrame(const NITLibrary::NITFrame& frame, const NITLibrary::NITFrameInfo& info);
	ito::RetVal checkFrame(ito::DataObject *externalDataObject);
	void onStop(){/* Display capture result, statistics, ... */ }

private:
	NITWidySWIR *m_pCamera;

public slots:



};

//-----------------------------------------------------------------------------------------------------------------------------------
class NITWidySWIRFilter : public NITLibrary::NITFilter
{
public:
	// Construct
	NITWidySWIRFilter(unsigned int fifoFrameSize = 10) : NITLibrary::NITFilter(fifoFrameSize)
	{
	}

	// Destruct
	~NITWidySWIRFilter()
	{
	}

protected:
	// Implementation
	void onNewFrame(const NITLibrary::NITFrame& frame, const NITLibrary::NITFrameInfo& info)
	{
	}
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    NITWidySWIR

  */
class NITWidySWIR : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~NITWidySWIR();
        //! Constructor
        NITWidySWIR();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */

    public:
        friend class NITWidySWIRInterface;
		friend class NITWidySWIRObserver;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

		enum GrabStatus { GrabIdle, GrabAcquisitionRunning, GrabFrameReceived };


    private:
		QMutex m_mutex; //this mutex makes the access to m_grabbingStatus thread-safe
		GrabStatus m_grabbingStatus;
		NITLibrary::NITManager& m_camManager;
		NITLibrary::NITDevice* m_camDevice;
		NITWidySWIRObserver *m_frameObserver;
		int m_deviceNumber;
		int m_deviceCount = 0;
		static int m_initNum;
		static QList<int> m_initList;
        QString m_nucFilePath;
		cv::Mat m_NUCDark; //already converted to float64
		cv::Mat m_GainMap; //already converted to float64
		int m_minBPP;
		int m_maxBPP;

		bool m_isgrabbing;
		bool m_instanceInitialized;
        bool m_pixelCorrection;

    public slots:
        //!< Get Camera-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set Camera-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //!< Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //!< Stop the camera to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //!< Softwaretrigger for the camera
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //!< Wait for acquired picture, copy the picture to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

		ito::RetVal loadNUCFile(const QString &filePath, double integrationTime);

        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};



#endif // NITWIDYSWIR_H
