#ifndef OPENCVGRABBER_H
#define OPENCVGRABBER_H

#include "common/addInGrabber.h"
#include "dialogOpenCVGrabber.h"

#include "opencv2/opencv.hpp"

#include <qsharedpointer.h>

#define BUFFERNUMBER 1 //Maximal Number of Buffers: 32

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class	OpenCVGrabber 
  *\brief	class to use a the standard grabber from OpenCV as an ITOM-Addin. Child of AddIn - Library (DLL) - Interface
  *
  *
  *	\sa	AddInDataIO, DummyGrabber
  *	\date	Oct.2011
  *	\author	Wolfram Lyda
  * \warning	NA
  *
  */

#define	FCC(ch4) ((((DWORD)(ch4) & 0xFF) << 24) | (((DWORD)(ch4) & 0xFF00) << 8) | (((DWORD)(ch4) & 0xFF0000) >> 8) |(((DWORD)(ch4) & 0xFF000000) >> 24))

class OpenCVGrabber : public ito::AddInGrabber //, public OpenCVGrabberInterface
{
    Q_OBJECT

    protected:
		//! Destructor
        ~OpenCVGrabber();
		//! Constructor
        OpenCVGrabber();
 //       ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);	/*!< Check if objekt has to be reallocated */
		ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */


    public:
        friend class OpenCVGrabberInterface;
        const ito::RetVal showConfDialog(void);	//! Open the config nonmodal dialog to set camera parameters 

    private:

        cv::VideoCapture *m_pCam;	/*!< Handle to the openCV-Cam-Class */

		int m_CCD_ID; /*!< Camera ID */
        bool m_isgrabbing; /*!< Check if acquire was called */

        size_t m_imgChannels; /*!< number of channels of the camera image due to current parameterization */
        int m_imgCols; /*!< cols of the camera image due to current parameterization */
        int m_imgRows; /*!< rows of the camera image due to current parameterization */
        size_t m_imgBpp; /*!< number of element size of the camera image due to current parameterization */
        bool m_camStatusChecked;

        cv::Mat m_pDataMatBuffer;	/*!< OpenCV DataFile to retrieve datas, this image is already filled after acquire command */

		ito::RetVal checkCameraAbilities(); /*!< Funktion to check and set aviable data types */

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

		void updateParameters(QMap<QString, ito::ParamBase> params);

    private slots:

};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class	OpenCVGrabberInterface 
  *
  *\brief	Interface-Class for OpenCVGrabber-Class
  *
  *	\sa	AddInDataIO, OpenCVGrabber
  *	\date	11.10.2010
  *	\author	Wolfram Lyda
  * \warning	NA
  *
  */
class OpenCVGrabberInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)

    protected:

    public:
        OpenCVGrabberInterface();
        ~OpenCVGrabberInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // OpenCVGrabber_H
