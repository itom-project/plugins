/* ********************************************************************
    Plugin "MSMediaFoundation" for itom software
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

#ifndef MSMEDIAFOUNDATION_H
#define MSMEDIAFOUNDATION_H

#include "common/addInGrabber.h"

#if CV_MAJOR_VERSION >= 4
#include "opencv2/opencv.hpp"
#else
#include <opencv/cv.h>
#endif

#include <qsharedpointer.h>
#include <qmutex.h>
#include "videoInput.h"
#include "dialogMSMediaFoundation.h"

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MSMediaFoundation

  */
class MSMediaFoundation : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~MSMediaFoundation();
        //! Constructor
        MSMediaFoundation();
 //       ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);    /*!< Check if objekt has to be reallocated */
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */


    public:
        friend class MSMediaFoundationInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

        static QSharedPointer<VideoInput> VideoInputInstance;

    private:

        static QMutex VideoInputCreateMutex;
        static int VideoInputRefCount; //!< reference counter about currently existing instances. Used to create a singleton of VideoInput.

        enum InitState {initNotTested, initSuccessfull, initNotSuccessfull};

        InitState m_initState;

        QSharedPointer<VideoInput> m_videoInput;    /*!< Handle to the VideoInput-Class Eqaul to VideoInputInstance */
        CamParameters m_camParams;
        QHash<QString, Parameter*> m_camParamsHash;

        int m_deviceID; /*!< Camera ID */
        bool m_isgrabbing; /*!< Check if acquire was called */
        bool m_timeout;

        int m_imgChannels; /*!< number of channels of the camera image due to current parameterization */
        int m_imgCols; /*!< cols of the camera image due to current parameterization */
        int m_imgRows; /*!< rows of the camera image due to current parameterization */
        int m_imgBpp; /*!< number of element size of the camera image due to current parameterization */
        bool m_camStatusChecked;
        bool m_flipImage;

        int m_colorMode;

        cv::Mat m_pDataMatBuffer;    /*!< OpenCV DataFile to retrieve datas, this image is already filled after acquire command */

        cv::Mat m_alphaChannel; /* simple uint8, 1-channel image with 255 values filled in case of colorMode. This is the alpha plane */

        ito::RetVal checkCameraAbilities(); /*!< Funktion to check and set aviable data types */

        enum tColorMode
        {
            modeAuto,
            modeColor,
            modeRed,
            modeGreen,
            modeBlue,
            modeGray
        };

        ito::RetVal synchronizeParam(const Parameter &parameter, ito::Param &paramDbl, ito::Param &paramAutoInt);
        ito::RetVal updateCamParam(Parameter &parameter, const ito::ParamBase &paramDbl, const ito::ParamBase &paramAutoInt);
        ito::RetVal synchronizeCameraParametersToParams(bool deleteIfNotAvailable = false);
        ito::RetVal checkInitState();

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

        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MSMediaFoundationInterface
  *
  *\brief    Interface-Class for MSMediaFoundation-Class
  *
  *    \sa    AddInDataIO, MSMediaFoundation
  *
  */
class MSMediaFoundationInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        MSMediaFoundationInterface();
        ~MSMediaFoundationInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // MSMEDIAFOUNDATION_H
