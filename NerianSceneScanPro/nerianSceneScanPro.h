/* ********************************************************************
Plugin "NerianSceneScanPro" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut für Technische Optik (ITO),
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

#ifndef NERIANSCENESCANPRO_H
#define NERIANSCENESCANPRO_H

#include "common/addInGrabber.h"
#if CV_MAJOR_VERSION >= 4
#include "opencv2/opencv.hpp"
#else
#include <opencv/cv.h>
#endif
#include <qsharedpointer.h>
#include "dialogNerianSceneScanPro.h"

namespace visiontransfer
{
    class DeviceParameters;
    class ImageTransfer;
    class ImagePair;
}
//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    NerianSceneScanProInterface
  *
  *\brief    Interface-Class for NerianSceneScanPro-Class
  *
  *    \sa    AddInDataIO, NerianSceneScanPro
  *
  */
class NerianSceneScanProInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        NerianSceneScanProInterface();
        ~NerianSceneScanProInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    NerianSceneScanPro

  */
class NerianSceneScanPro : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~NerianSceneScanPro();
        //! Constructor
        NerianSceneScanPro();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */

    public:
        friend class NerianSceneScanProInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

        //char* bufferPtr; //this can be a pointer holding the image array from the camera. This buffer is then copied to the dataObject m_data (defined in AddInGrabber)

    private:
        enum SyncParams: unsigned long{
            sOperationMode = 0x00000001,
            sDisparityOffset = 0x000000000002,
            sStereoMatching = 0x000000000004,
            sMaskBorderPixels = 0x000000000008,
            sConsistencyCheck = 0x00000000010,
            sUniquenessCheck = 0x000000000020,
            sTextureFilter = 0x000000000040,
            sGapInterpolation = 0x000000000080,
            sNoiseReduction = 0x000000000100,
            sSpeckleFilterIterations = 0x000000000200,
            sAutoMode = 0x000000000400,
            sAutoTargetIntensity = 0x000000000800,
            sAutoIntensityDelta = 0x000000001000,
            sAutoTargetFrame = 0x000000002000,
            sAutoSkippedFrames = 0x000000004000,
            sAutoMaxExposureTime = 0x000000008000,
            sAutoMaxGain = 0x000000010000,
            sManualExposureTime = 0x000000020000,
            sManualGain = 0x000000040000,
            sAutoROI = 0x000000080000,
            sMaxFrameTimeDifference = 0x000000100000,
            sTrigger = 0x000000200000,
            sAutoRecalibration = 0x000000400000,
            sImageFormat = 0x000000800000,
            sAll= sOperationMode| sDisparityOffset| sStereoMatching| sMaskBorderPixels| sConsistencyCheck| sUniquenessCheck|
            sTextureFilter| sGapInterpolation| sNoiseReduction| sSpeckleFilterIterations| sAutoMode| sAutoTargetIntensity|
            sAutoIntensityDelta| sAutoTargetFrame| sAutoSkippedFrames| sAutoMaxExposureTime| sAutoMaxGain| sManualExposureTime| sManualGain|
            sAutoROI| sMaxFrameTimeDifference| sTrigger| sAutoRecalibration | sImageFormat
        };
        bool m_isgrabbing; /*!< Check if acquire was executed */
        ito::RetVal syncParams(SyncParams what = sAll);
        template<typename _Tp>ito::RetVal getParamInfo(_Tp &min, _Tp &max, _Tp &inc,_Tp &value, const char* name);
        visiontransfer::DeviceParameters *m_pParamsObj;
        visiontransfer::ImageTransfer* m_pImageTransferObj;
        visiontransfer::ImagePair* m_pImagePair;

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

        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // NERIANSCENESCANPRO_H
