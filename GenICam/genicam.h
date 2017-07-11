/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut f�r Technische Optik (ITO),
    Universit�t Stuttgart, Germany

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

#ifndef GENICAM_H
#define GENICAM_H

#include "common/addInGrabber.h"

#include <qsharedpointer.h>
#include <qtimer.h>

#include "GenTL_v1_5.h"
#include "deviceContainer.h"


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    GenICamClass 
  */


class GenICamClass : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~GenICamClass();
        //! Constructor
        GenICamClass();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */

        ito::RetVal searchGenTLProducer(const QString &producer, const QString &vendor, const QString &model);
        ito::RetVal checkGCError(const GenTL::GC_ERROR &error) const;

    public:
        friend class GenICamInterface;
		friend class GenTLDevice; //to access parameterChangedTimerFired

        //const ito::RetVal showConfDialog(void);    //! Open the config nonmodal dialog to set camera parameters 
        int hasConfDialog(void) { return 0; }; //!< indicates that this plugin has got a configuration dialog

    private:
        QSharedPointer<GenTLSystem> m_system;
        QSharedPointer<GenTLInterface> m_interface;
        QSharedPointer<GenTLDevice> m_device;
        QSharedPointer<GenTLDataStream> m_stream;

		ito::RetVal m_acquisitionRetVal;
		bool m_newImageAvailable;
		bool m_hasTriggerSource;

		struct AcquisitionCache
		{
			enum Mode { Continuous, SingleFrame, MultiFrame, Other };
			Mode mode;
			bool triggerMode;
			QByteArray triggerSource;

		};
		AcquisitionCache m_acquisitionCache;

		void cacheAcquisitionParameters();

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


    private slots:
		void parameterChangedTimerFired();
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    GenICamInterface 
  *
  *\brief    Interface-Class for GenICamClass-Class
  *
  *    \sa    AddInDataIO, GenICamClass
  */
class GenICamInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5,0,0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        GenICamInterface();
        ~GenICamInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // GENICAM_H