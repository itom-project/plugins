#ifndef GOPRO_H
#define GOPRO_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>
#include <qnetworkaccessmanager.h>
#include "dialogGoPro.h"

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    GoProInterface 
  *
  *\brief    Interface-Class for GoPro-Class
  *
  *    \sa    AddInDataIO, GoPro
  *
  */
class GoProInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        GoProInterface();
        ~GoProInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    GoPro

  */
class GoPro : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~GoPro();
        //! Constructor
        GoPro();
        
        ito::RetVal retrieveData(
            ito::DataObject* externalDataObject = nullptr); /*!< Wait for acquired picture */
        
    public:
        friend class GoProInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog
        
    private:
        bool m_isGrabbing; /*!< Check if acquire was executed */
        bool m_isStreaming;
        int m_imgChannels; /*!< number of channels of the camera image due to current parameterization */
        int m_colorMode;
        int m_imgCols;
        int m_imgRows;
        int m_imgBpp;

        enum tColorMode
        {
            modeAuto,
            modeColor,
            modeRed,
            modeGreen,
            modeBlue,
            modeGray
        };

        VideoCapture m_VideoCapture;
        Mat m_pDataMatBuffer;
        Mat m_alphaChannel; /* simple uint8, 1-channel image with 255 values filled in case of colorMode. This is the alpha plane */

        QNetworkAccessManager m_NetworkManager;

    public slots:
        //!< Get Camera-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set Camera-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(
            QVector<ito::ParamBase>* paramsMand,
            QVector<ito::ParamBase>* paramsOpt,
            ItomSharedSemaphore* waitCond = nullptr);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //!< Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //!< Stop the camera to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //!< Softwaretrigger for the camera
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore* waitCond = nullptr);
        //!< Wait for acquired picture, copy the picture to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        void get(QString location);
        void post(QString location, QByteArray data);
        
        //checkData usually need not to be overwritten (see comments in source code)
        ito::RetVal checkData(ito::DataObject *externalDataObject = nullptr);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);

        void readyRead();
        void replyFinished(QNetworkReply& reply);
        void slotReadyRead();
        void slotError();
        void slotSslErrors(QNetworkReply& reply, const QList<QSslError> &error);

        void videoCaptureTimerCallBack();
};

#endif // GoPro_H
