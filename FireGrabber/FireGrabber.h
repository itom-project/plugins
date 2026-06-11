/* ********************************************************************
    Plugin "FireGrabber" for itom software
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

#ifndef FIREGRABBER_H
#define FIREGRABBER_H

#include "common/addInGrabber.h"
#include "dialogFireGrabber.h"

#ifdef WIN32
    #include <FGCamera.h>
#else
    #include <FireGrab/dc1394/dc1394.h>
    #include <FireGrab/dc1394/offsets.h>
    #include <FireGrab/dc1394/vendor/avt.h>
#endif

#include "common/helperCommon.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

#define BUFFERNUMBER 1 //Maximal Number of Buffers: 32

//----------------------------------------------------------------------------------------------------------------------------------

class FireGrabberInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this FileGrabberInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        FireGrabberInterface();                    /*!< Constructor */
        ~FireGrabberInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of FireGrabber and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of FireGrabber, given by *addInInst */

};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    FireGrabber
  *\brief    class to use a the standard grabber from Allied Fire Grab Packet as an itom-Addin.
  *
  *
  *    \sa    AddInDataIO, FireGrabber
  *    \date    Jun.2012
  *    \author    Alexander Bielke (WindowsVersion) , Goran Baer (Linux Version)
  * \warning    NA
  *
  */

class FireGrabber : public ito::AddInGrabber //, public FireGrabberInterface
{
    Q_OBJECT

    protected:
        //! Destructor
        ~FireGrabber();
        //! Constructor
        FireGrabber();
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */


    public:
        friend class FireGrabberInterface;

        const ito::RetVal showConfDialog(void);    //! Open the config nonmodal dialog to set camera parameters
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        #ifdef WIN32
            CFGCamera  Camera;
        #else
            dc1394camera_t *camera;
            int i;
            dc1394featureset_t features;
            dc1394framerates_t framerates;
            dc1394video_modes_t video_modes;
            dc1394framerate_t framerate;
            dc1394video_mode_t video_mode;
            dc1394color_coding_t coding;
            dc1394_t * d;
            dc1394camera_list_t * list;
            dc1394error_t Result;
            dc1394video_frame_t *frame;
        #endif

        struct ExposureParameters
        {
            ExposureParameters() : AVTCam(false), timebaseMs(0.0), offsetMs(0.0) {};
            bool AVTCam;
            double timebaseMs;
            double offsetMs;
        };

        ExposureParameters m_exposureParams;
#ifndef WIN32
        unsigned int  m_xSize, m_ySize;
        typedef dc1394error_t ResultType;
#else
        unsigned long  m_xSize, m_ySize;
        typedef UINT32 ResultType;
#endif

        bool m_isgrabbing; /*!< Check if camera is started */
        bool m_acquireReady; /*!< Check if frame was acquired */
        //bool saturation_on; /*!< Check if saturation is controlled manually */

        ito::RetVal AlliedChkError(int errornumber); /*!< Map Allied-Error-Number to ITOM-Errortype and Message */

        ito::RetVal adjustROI(int x0, int x1, int y0, int y1);

        static int m_numberOfInstances;

#ifdef WIN32
        QMap<QString, FGPINFO> m_camProperties;
#endif
        // cast shutter time to internal whatever
        double shutterToExposureSec(int shutter);
        int exposureSecToShutter(double exposure);

        ito::RetVal initAVTCameras(const char *vendorName, const char *modelName, int desiredTimebase);

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

        void GainOffsetPropertiesChanged(double gain, double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};


//----------------------------------------------------------------------------------------------------------------------------------

#endif // FireGrabber_H
