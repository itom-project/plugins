/* ********************************************************************
    Plugin "PCOSensicam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut für Technische Optik (ITO),
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

#ifndef PCOSENSICAM_H
#define PCOSENSICAM_H

#include "common/addInGrabber.h"

#include "dialogPCOSensicam.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

#include "SencamExport.h"
#include "SC_SDKStructures.h"

#define PCO_SC_CREATE_NAME_OBJECT
#include "sencam_def.h"
#undef PCO_SC_CREATE_NAME_OBJECT



#define PCO_NUMBER_BUFFERS 1

//----------------------------------------------------------------------------------------------------------------------------------
class PCOSensicamInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this PCOSensicamInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        PCOSensicamInterface();                    /*!< Constructor */
        ~PCOSensicamInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of PCOSensicam and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of PCOSensicam, given by *addInInst */

};

//----------------------------------------------------------------------------------------------------------------------------------
class PCOSensicam : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~PCOSensicam();
        PCOSensicam();

//        ito::RetVal checkData();
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

        void dockWidgetVisibilityChanged(bool visible);

    public:
        friend class PCOSensicamInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        struct COCValues
        {
            int mode;
            int trig;
            int roix1;
            int roix2;
            int roiy1;
            int roiy2;
            int hbin;
            int vbin;
            QByteArray table;
        };

        struct PCOBuffer
        {
            int bufNr;
            void* bufData;
            bool bufQueued;
            bool bufError;
            HANDLE bufEvent;
        };

        ito::RetVal stopCamera();
        ito::RetVal startCamera();
        ito::RetVal synchronizeParameters();
        int test_coc2(COCValues &cocValues);

        PCOBuffer m_buffers[PCO_NUMBER_BUFFERS];

        bool m_isgrabbing;
        bool m_isstarted;
        HANDLE m_hCamera;
        COCValues m_cocValues;

        SC_Camera_Description m_caminfo;
        ito::RetVal m_acquisitionRetVal;

        ito::RetVal setExposure(double exposure);

        ito::RetVal checkError(int error);

    signals:

    public slots:
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

#endif // PCOSENSICAM_H
