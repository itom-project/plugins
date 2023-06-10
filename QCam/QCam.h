/* ********************************************************************
    Plugin "QCam" for itom software
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

#ifndef QCAM_H
#define QCAM_H

#include "common/addInGrabber.h"
#include "dialogQCam.h"

#include "QCamApi.h"

#include <qsharedpointer.h>

#define NUMBERBUFFERS 1

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    QCamInterface
  *
  *\brief    Interface-Class for QCam-class
  *
  */
class QCamInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        QCamInterface();
        ~QCamInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    QCam
  *\brief    class to use firewire-Cameras with the QCam driver from QImaging
  *
  *
  *    \sa    AddInDataIO
  *    \author
  *
  */
class QCam : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        QCam();
        ~QCam();

    public:
        friend class QCamInterface;
        const ito::RetVal showConfDialog(void);    /*!< Open the config nonmodal dialog to set camera parameters */

        void frameCallback(unsigned long userData, QCam_Err errcode, unsigned long flags);

        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);    /*! <Wait for acquired picture */
//        ito::RetVal checkData(void);    /*!< Check if objekt has to be reallocated */

        ito::RetVal errorCheck(QCam_Err errcode);
        ito::RetVal supportedFormats(bool &mono, bool &colorFilter, bool &colorBayer);

        ito::RetVal requeueFrame();

    private:

        ito::RetVal rangeCheck(const unsigned long &min, const unsigned long &max, const unsigned long &value, const QByteArray &name);

        QCam_Handle m_camHandle;
        QCam_Settings m_camSettings;

        QCam_Frame m_frames[NUMBERBUFFERS];
        ito::RetVal m_frameCallbackRetVal;
        int m_frameCallbackFrameIdx;
        bool m_waitingForAcquire;

        static int instanceCounter;

    public slots:

        //! returns parameter of m_params with key name.
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        //! sets parameter of m_params with key name.
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        //! Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //! Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //! Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //! Stop the camera to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //! Softwaretrigger for the camera
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //! Calls retrieveData(NULL), than copy the picture to dObj of right type and size
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        //! Deep copy the camera buffer to dObj. Object must be of right size and type. If liveData is running, a second deep-copy is performed to copy data to the grabber
        ito::RetVal copyVal(void *dObj, ItomSharedSemaphore *waitCond);

        //! Retrieve new offset and new gain and give them to the camera dll
        void updateParameters(QMap<QString, ito::ParamBase> params);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};


void QCAMAPI qCamFrameCallback(void * userPtr, unsigned long userData, QCam_Err errcode, unsigned long flags);

//----------------------------------------------------------------------------------------------------------------------------------

#endif // QCAM_H
