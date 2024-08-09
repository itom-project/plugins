/* ********************************************************************
    Plugin "PCOCamera" for itom software
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

#ifndef PCOCAMERA_H
#define PCOCAMERA_H

#include "common/addInGrabber.h"

#include "dialogPCOCamera.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

#include "sc2_defs.h"
#include "PCO_err.h"
#define PCO_ERRT_H_CREATE_OBJECT
#include "sc2_SDKStructures.h"
#include "SC2_CamExport.h"



#define PCO_NUMBER_BUFFERS 2

//----------------------------------------------------------------------------------------------------------------------------------
class PCOCameraInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this PCOCameraInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        PCOCameraInterface();                    /*!< Constructor */
        ~PCOCameraInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of PCOCamera and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of PCOCamera, given by *addInInst */

};

//----------------------------------------------------------------------------------------------------------------------------------
class PCOCamera : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~PCOCamera();
        PCOCamera();

//        ito::RetVal checkData();
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

        void dockWidgetVisibilityChanged(bool visible);

    public:
        friend class PCOCameraInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        ito::RetVal stopCamera();
        ito::RetVal startCamera();
        ito::RetVal sychronizeParameters();

        struct PCOBuffer
        {
            short bufNr;
            WORD* bufData;
            bool bufQueued;
            bool bufError;
            HANDLE bufEvent;
        };

        PCOBuffer m_buffers[PCO_NUMBER_BUFFERS];

        bool m_isgrabbing;
        HANDLE m_hCamera;

        HANDLE m_hEvent;
        //WORD m_wActSeg;

        //WORD *m_wBuf;
        //short m_curBuf;
        PCO_Description m_caminfo;

        ito::RetVal setExposure(double exposure);

        ito::RetVal checkError(DWORD error);

    signals:

    public slots:
        /*ito::RetVal getParam(const char *name, QSharedPointer<char> val, QSharedPointer<int> len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getParam(const char *name, QSharedPointer<double> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const char *val, const int len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const double val, ItomSharedSemaphore *waitCond = NULL);*/

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);


};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // PCOCamera_H
