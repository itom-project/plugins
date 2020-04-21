/* ********************************************************************
Plugin "PmdPico" for itom software
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

#ifndef PMDPICO_H
#define PMDPICO_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>
#include "pmdPico.h"
#include "royale.hpp"

class PmdPico;
//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabberInterface 
  *
  *\brief    Interface-Class for MyGrabber-Class
  *
  *    \sa    AddInDataIO, MyGrabber
  *
  */
class PmdPicoInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        PmdPicoInterface();
        ~PmdPicoInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};
//----------------------------------------------------------------------------------------------------------------------------------
class DataListener : public royale::IDepthDataListener
{
public:
    DataListener(PmdPico* inst);
    void onNewData(const royale::DepthData *data);
    ito::RetVal captureSingleImage(bool updateParams);
    const royale::DepthData *m_data;
    void setLockMutex(QMutex* mutex);
    ito::RetVal freeRunCapture();
private:
    bool m_takeNext; //indicates if the next picture should be stored
    bool m_singleShot; //indicates if the camera should be stopped after receiving the next data
    bool m_updateParams;
    PmdPico* m_host;
    QMutex* m_mutex;

};

//----------------------------------------------------------------------------------------------------------------------------------
class ExposureListener : public QObject , public royale::IExposureListener2 
{
Q_OBJECT
public:
    ExposureListener();
    void onNewExposure(const uint32_t exposureTime, const royale::StreamId streamId);

    ito::int32 getIntegrationTime() const;
signals:
    void integrationTimeChanged(ito::uint32 value);

private:
    ito::uint32 integrationTime;
};
//----------------------------------------------------------------------------------------------------------------------------------

 /**
  *\class    MyGrabber

  */
class PmdPico : public ito::AddInGrabber 
{
    Q_OBJECT

    protected:
        //! Destructor
        ~PmdPico();
        //! Constructor
        PmdPico();
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);
        
    public:

        friend class PmdPicoInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 0; }; //!< indicates that this plugin has got a configuration dialog
        static ito::RetVal getErrStr(const royale::CameraStatus& status);
        ito::RetVal setCapturingState(const bool capture) const;
        ito::RetVal getCapturingState(bool &state) const;
        ito::RetVal updateParamsFromImage();
        ito::RetVal copyDataToBuffer();
        char* bufferPtr;//this can be a pointer holding the image array from the camera. This buffer is then copied to the dataObject m_data (defined in AddInGrabber)


    private:
        enum SyncParams {
            sExposure = 0x0001,
            sUseCase = 0x0002,
            sRoi = 0x0004,
            sAutoExposure = 0x0008,
            sTriggerMode = 0x0010,
            sFrameRate = 0x0020,
            sAll = sExposure | sUseCase | sRoi | sTriggerMode | sFrameRate | sAutoExposure
        };
        enum CapturedPictures
        {
            cXCoordinate = 0x0001,
            cYCoordinate = 0x0002,
            cZValue = 0x0004,
            cGrayImage = 0x0008,
            cConfidenceMap = 0x0010,
            cAll = cXCoordinate | cYCoordinate | cZValue | cGrayImage |cConfidenceMap
        };

        ito::RetVal synchronizeCameraSettings(const int &paramMask = sAll);
        ito::RetVal checkForCoordinatesObj(ito::DataObject* externalObj);
        ito::RetVal switchDataObj();

        inline double musecToSec(double musec) { return (double)musec * 1.0e-6; }
        inline uint32_t secToMusec(double sec) { return (uint32_t)(sec * 1.0e6); }

        bool m_isgrabbing; /*!< Check if acquire was executed */
        royale::ICameraDevice* m_cameraDevice;
        
        ExposureListener m_exposureListener;
        
        DataListener m_dataListener;

        ito::DataObject m_dataGray; //buffer for gray image
        ito::DataObject m_dataConfidence;//buffer for confidence map
        ito::DataObject m_dataYCoordinate;
        ito::DataObject m_dataXCoordinate;
        ito::DataObject m_dataZValue;
        bool m_camInit;
        int m_currentBuffer;/*!< bitmask indicating the captured data*/
        int m_deliverState; /*!< bitmask indicating the captured but not delivered pictures*/

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
        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> >paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, 
            QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond);
        

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};
//----------------------------------------------------------------------------------------------------------------------------------

#endif // MYGRABBER_H
