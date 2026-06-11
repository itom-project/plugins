/* ********************************************************************
    Plugin "V4L2" for itom software
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

#ifndef V4L2_H
#define V4L2_H

#include "common/addInGrabber.h"
#include "opencv2/opencv.hpp"
#include <QDir>
#include <qsharedpointer.h>
#include <stdlib.h>
#include <linux/videodev2.h>
#include "v4l2_itom_api.h"

#include "dialogV4L2.h"


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    V4L2

  */

class V4L2 : public ito::AddInGrabber //, public V4L2Interface
{
    Q_OBJECT

    protected:
        //! Destructor
        ~V4L2();
        //! Constructor
        V4L2();
 //       ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);    /*!< Check if object has to be reallocated */
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */


    public:
        friend class V4L2Interface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        DeviceList m_pDL;

        int m_deviceID; /*!< Camera ID */
        bool m_isgrabbing; /*!< Check if acquire was called */
        bool m_timeout;
        bool m_deviceStarted;

        int m_imgChannels; /*!< number of channels of the camera image due to current parameterization */
        int m_imgCols; /*!< cols of the camera image due to current parameterization */
        int m_imgRows; /*!< rows of the camera image due to current parameterization */
        int m_imgBpp; /*!< number of element size of the camera image due to current parameterization */
        bool m_camStatusChecked;

        int m_colorMode;

        cv::Mat m_pDataMatBuffer;    /*!< OpenCV DataFile to retrieve data, this image is already filled after acquire command */

        cv::Mat m_alphaChannel; /* simple uint8, 1-channel image with 255 values filled in case of colorMode. This is the alpha plane */

        unsigned char *m_frame;

        ito::RetVal checkCameraAbilities(); /*!< Function to check and set available data types */

        enum tColorMode
        {
            modeAuto,
            modeColor,
            modeRed,
            modeGreen,
            modeBlue,
            modeGray
        };

        ito::RetVal fill_m_params(); /*!< Function to fill m_param with the camera controls*/
        ito::RetVal updateCamParam(QString &name, const ito::ParamBase &paramInt);
        ito::RetVal synchronizeCameraParametersToParams(bool deleteIfNotAvailable = false);

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
        void dockWidgetValueChanged(int type, double value);
};



//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    V4L2Interface
  *
  *\brief    Interface-Class for V4L2-Class
  *
  *    \sa    AddInDataIO, V4L2
  *
  */
class V4L2Interface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        V4L2Interface();
        ~V4L2Interface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // V4L2_H
