/* ********************************************************************
    Plugin "IDSuEye" for itom software
    URL: http://www.bitbucket.org/itom/plugins
    Copyright (C) 2014, Pulsar Photonics GmbH, Aachen
    Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

#ifndef IDSUEYE_H
#define IDSUEYE_H

#include "common/addInGrabber.h"

#if linux
    #include "ueye.h"
#else
    #include "IDS/uEye.h"
#endif

#include <qsharedpointer.h>
#include <QTimerEvent>

//----------------------------------------------------------------------------------------------------------------------------------
class IDSuEye : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~IDSuEye();
        IDSuEye();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);
        void dockWidgetVisibilityChanged(bool visible);

        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class IDSInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        enum SyncParams { 
            sPixelClock = 1, 
            sExposure = 2, 
            sBinning = 4,
            sRoi = 8,
            sGain = 16,
            sOffset = 32,
            sTriggerMode = 64,
            sBppAndColorMode = 128,
            sAll = sPixelClock | sExposure | sBinning | sRoi | sGain | sOffset | sTriggerMode | sBppAndColorMode };

        struct MemoryStruct {
            int width;
            int height;
            int bitspixel;
            char *ppcImgMem;
            INT pid;
            bool imageAvailable;
        };

        ito::RetVal checkError(const int &code);

        ito::RetVal synchronizeCameraSettings(int what = sAll);
        ito::RetVal loadSensorInfo();
        ito::RetVal setMinimumFrameRate();

        IS_POINT_2D m_monochromeBitDepthRange;

        HIDS m_camera;
        IS_RANGE_S32 m_blacklevelRange; //range for the offset
        SENSORINFO m_sensorInfo;
        int m_bitspixel; //bits per pixel that needs to be allocated
        MemoryStruct *m_pMemory;
        bool m_colouredOutput;
        ito::RetVal m_acquisitionRetVal;

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

#endif // IDSUEYE_H
