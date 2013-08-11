#include "OpenCVGrabber.h"
#include "pluginVersion.h"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

//#include "common/helperCommon.h"

/**
* \file openCVGrabber.cpp
* \brief 
*
* \detail Parameter
*   CV_CAP_PROP_POS_MSEC		Current position of the video file in milliseconds or video capture timestamp.
*   CV_CAP_PROP_POS_FRAMES		0-based index of the frame to be decoded/captured next.
*   CV_CAP_PROP_POS_AVI_RATIO	Relative position of the video file: 0 - start of the film, 1 - end of the film.
*   CV_CAP_PROP_FRAME_WIDTH		Width of the frames in the video stream.
*   CV_CAP_PROP_FRAME_HEIGHT	Height of the frames in the video stream.
*   CV_CAP_PROP_FPS				Frame rate.
*   CV_CAP_PROP_FOURCC			4-character code of codec.
*   CV_CAP_PROP_FRAME_COUNT		Number of frames in the video file.
*   CV_CAP_PROP_FORMAT			Format of the Mat objects returned by retrieve() .
*   CV_CAP_PROP_MODE			Backend-specific value indicating the current capture mode.
*   CV_CAP_PROP_BRIGHTNESS		Brightness of the data (only for cameras).
*   CV_CAP_PROP_CONTRAST		Contrast of the data (only for cameras).
*   CV_CAP_PROP_SATURATION		Saturation of the data (only for cameras).
*   CV_CAP_PROP_HUE				Hue of the data (only for cameras).
*   CV_CAP_PROP_GAIN			Gain of the data (only for cameras).
*   CV_CAP_PROP_EXPOSURE		Exposure (only for cameras).
*   CV_CAP_PROP_CONVERT_RGB		Boolean flags indicating whether datas should be converted to RGB.
*   CV_CAP_PROP_WHITE_BALANCE	Currently not supported
*   CV_CAP_PROP_RECTIFICATION	Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
*   
*   
*   3IV0 	MPEG4-based codec 3ivx
*   3IV1 	MPEG4-based codec 3ivx
*   3IV2 	MPEG4-based codec 3ivx
*   3IVD 	FFmpeg DivX ;-) (MS MPEG-4 v3)
*   3IVX 	MPEG4-based codec 3ivx
*   AAS4 	Autodesk Animator codec (RLE)
*   AASC 	Autodesk Animator codec (RLE)
*   ABYR 	Kensington codec
*   ADV1 	Loronix WaveCodec (used in various CCTV products)
*   ADVJ 	Avid M-JPEG Avid Technology (also known as AVRn)
*   AEMI 	Array VideoONE MPEG1-I Capture
*   AFLC 	Autodesk Animator FLC (256 color)
*   AFLI 	Autodesk Animator FLI (256 color)
*   AMPG 	Array VideoONE MPEG
*   ANIM 	Intel - RDX
*   AP41 	AngelPotion Definitive (hack MS MP43)
*   ASV1 	Asus Video V1
*   ASV2 	Asus Video V2
*   ASVX 	Asus Video 2.0
*   AUR2 	AuraVision - Aura 2 Codec - YUV 422
*   AURA 	AuraVision - Aura 1 Codec - YUV 411
*   AVDJ 	Avid Motion JPEG
*   AVI1 	MainConcept Motion JPEG Codec
*   AVI2 	MainConcept Motion JPEG Codec
*   AVRN 	Avid Motion JPEG (also known as ADVJ)
*   AZPR 	Quicktime Apple Video
*   BGR 	Uncompressed BGR32 8:8:8:8
*   BGR(15) 	Uncompressed BGR15 5:5:5
*   BGR(16) 	Uncompressed BGR16 5:6:5
*   BGR(24) 	Uncompressed BGR24 8:8:8
*   BINK 	Bink Video (RAD Game Tools) (256 color)
*   BITM 	Microsoft H.261
*   BLZ0 	FFmpeg MPEG-4
*   BT20 	Conexant (ex Brooktree) - MediaStream codec
*   BTCV 	Conexant (ex Brooktree) - Composite Video codec
*   BTVC 	Conexant (ex Brooktree) - Composite Video codec
*   BW10 	Data Translation Broadway MPEG Capture/Compression
*   CC12 	Intel - YUV12 codec
*   CDVC 	Canopus - DV codec
*   CFCC 	Conkrete DPS Perception Motion JPEG
*   CGDI 	Camcorder Video (MS Office 97)
*   CHAM 	Winnov, Inc. - MM_WINNOV_CAVIARA_CHAMPAGNE
*   CJPG 	Creative Video Blaster Webcam Go JPEG
*   CLJR 	Cirrus Logic YUV 4:1:1
*   CLPL 	Format similar to YV12 but including a level of indirection.
*   CMYK 	Common Data Format in Printing
*   COL0 	FFmpeg DivX ;-) (MS MPEG-4 v3)
*   COL1 	FFmpeg DivX ;-) (MS MPEG-4 v3)
*   CPLA 	Weitek - 4:2:0 YUV Planar
*   CRAM 	Microsoft Video 1
*   CVID 	Supermac - Cinepak
*   CWLT 	reserved
*   CYUV 	Creative Labs YUV 4:2:2
*   CYUY 	ATI Technologies YUV
*   DUCK 	Duck Corp. - TrueMotion 1.0
*   DVE2 	InSoft - DVE-2 Videoconferencing codec
*   DXT1 	reserved
*   DXT2 	reserved
*   DXT3 	reserved
*   DXT4 	reserved
*   DXT5 	reserved
*   DXTC 	DirectX Texture Compression
*   FLJP 	D-Vision - Field Encoded Motion JPEG With LSI Bitstream Format
*   GWLT 	reserved
*   H260 	Intel - Conferencing codec
*   H261 	Intel - Conferencing codec
*   H262 	Intel - Conferencing codec
*   H263 	Intel - Conferencing codec
*   H264 	Intel - Conferencing codec
*   H265 	Intel - Conferencing codec
*   H266 	Intel - Conferencing codec
*   H267 	Intel - Conferencing codec
*   H268 	Intel - Conferencing codec
*   H269 	Intel - Conferencing codec
*   I263 	Intel - I263
*   I420 	Intel - Indeo 4 codec
*   IAN 	Intel - RDX
*   ICLB 	InSoft - CellB Videoconferencing codec
*   ILVC 	Intel - Layered Video
*   ILVR 	ITU-T - H.263+ compression standard
*   IRAW 	Intel - YUV uncompressed
*   IV30 	Intel - Indeo Video 4 codec
*   IV31 	Intel - Indeo Video 4 codec
*   IV32 	Intel - Indeo Video 4 codec
*   IV33 	Intel - Indeo Video 4 codec
*   IV34 	Intel - Indeo Video 4 codec
*   IV35 	Intel - Indeo Video 4 codec
*   IV36 	Intel - Indeo Video 4 codec
*   IV37 	Intel - Indeo Video 4 codec
*   IV38 	Intel - Indeo Video 4 codec
*   IV39 	Intel - Indeo Video 4 codec
*   IV40 	Intel - Indeo Video 4 codec
*   IV41 	Intel - Indeo Video 4 codec
*   IV42 	Intel - Indeo Video 4 codec
*   IV43 	Intel - Indeo Video 4 codec
*   IV44 	Intel - Indeo Video 4 codec
*   IV45 	Intel - Indeo Video 4 codec
*   IV46 	Intel - Indeo Video 4 codec
*   IV47 	Intel - Indeo Video 4 codec
*   IV48 	Intel - Indeo Video 4 codec
*   IV49 	Intel - Indeo Video 4 codec
*   IV50 	Intel - Indeo 5.0
*   MP42 	Microsoft - MPEG-4 Video Codec V2
*   MPEG 	Chromatic - MPEG 1 Video I Frame
*   MRCA 	FAST Multimedia - Mrcodec
*   MRLE 	Microsoft - Run Length Encoding
*   MSVC 	Microsoft - Video 1
*   NTN1 	Nogatech - Video Compression 1
*   qpeq 	Q-Team - QPEG 1.1 Format video codec
*   RGBT 	Computer Concepts - 32 bit support
*   RT21 	Intel - Indeo 2.1 codec
*   RVX 	Intel - RDX
*   SDCC 	Sun Communications - Digital Camera Codec
*   SFMC 	Crystal Net - SFM Codec
*   SMSC 	Radius - proprietary
*   SMSD 	Radius - proprietary
*   SPLC 	Splash Studios - ACM audio codec
*   SQZ2 	Microsoft - VXtreme Video Codec V2
*   SV10 	Sorenson - Video R1
*   TLMS 	TeraLogic - Motion Intraframe Codec
*   TLST 	TeraLogic - Motion Intraframe Codec
*   TM20 	Duck Corp. - TrueMotion 2.0
*   TMIC 	TeraLogic - Motion Intraframe Codec
*   TMOT 	Horizons Technology - TrueMotion Video Compression Algorithm
*   TR20 	Duck Corp. - TrueMotion RT 2.0
*   V422 	Vitec Multimedia - 24 bit YUV 4:2:2 format (CCIR 601). For this format, 2 consecutive pixels are represented by a 32 bit (4 byte) Y1UY2V color value.
*   V655 	Vitec Multimedia - 16 bit YUV 4:2:2 format.
*   VCR1 	ATI - VCR 1.0
*   VIVO 	Vivo - H.263 Video Codec
*   VIXL 	Miro Computer Products AG - for use with the Miro line of capture cards.
*   VLV1 	Videologic - VLCAP.DRV
*   WBVC 	Winbond Electronics - W9960
*   XLV0 	NetXL, Inc. - XL Video Decoder
*   YC12 	Intel - YUV12 codec
*   YUV8 	Winnov, Inc. - MM_WINNOV_CAVIAR_YUV8
*   YUV9 	Intel - YUV9
*   YUYV 	Canopus - YUYV compressor
*   ZPEG 	Metheus - Video Zipper

* 
*/

#if linux
  typedef uint32_t* LPDWORD;
  typedef uint32_t DWORD;
#endif

#define	FCC(ch4) ((((DWORD)(ch4) & 0xFF) << 24) | (((DWORD)(ch4) & 0xFF00) << 8) | (((DWORD)(ch4) & 0xFF0000) >> 8) |(((DWORD)(ch4) & 0xFF000000) >> 24))


//TODO: '' is reserved for single characters and must not be used to initialize a char array fix this!
enum codecDbl {
    YUY2 = FCC('YUY2'),	// 	Input/Output, YUY2 16-bits packed.
    UYVY = FCC('UYVY'),	// 	Input/Output, UYVY 16-bits packed.
    YV12 = FCC('YV12'),	// 	Input/Output, YV12 12-bits planar.
    I420 = FCC('I420'),  //	Input/Output, I420 12-bits planar.
	RGB32 =	DWORD(0),	// 	Input/Output, RGB 32-bits packed.
	RGB24 =	DWORD(0),	// 	Input/Output, RGB 24-bits packed.
	RGB8 =	DWORD(0),	// Input/Output, RGB 8-bits planar (Grayscale).
	RGB16 =	DWORD(3),	//Output only, RGB 16-bits packed (R:5-bits, G:6-bits, B:5-bits).
    RGB15 =	DWORD(0)	//Output only, RGB 16-bits packed (R:5-bits, G:5-bits, B:5-bits).
};

Q_DECLARE_METATYPE(ito::DataObject)

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabberInterface::getAddInInst(ito::AddInBase **addInInst)
{
    OpenCVGrabber* newInst = new OpenCVGrabber();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabberInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
      delete ((OpenCVGrabber *)*addInInst);
      int idx = m_InstList.indexOf(*addInInst);
      m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVGrabberInterface::OpenCVGrabberInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("OpenCVGrabber");

    m_description = QObject::tr("OpenCV Video Capture (USB-Cams, Firewire CMU1384 (if compiled in OpenCV)...)");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char* docstring = \
"This plugin wraps the video capture framework of OpenCV. Therefore it requires further libraries of OpenCV (core, highgui, improc and partially tbb). \n\
\n\
Usually all ordinary USB cameras are supported. If you compiled OpenCV with the CMU1384 flag, these firewire cameras are supported as well. Currently, a queuing \
problem in the Windows version for USB cameras exists. Therefore the plugin requests multiple images per frame in order to finally get the newest one. \
Therefore this implementation is not the fastest connection to any USB cameras.";
	m_detaildescription = QObject::tr(docstring);

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");     
    
    m_callInitInNewThread = false; //camera must be opened in main-thread

    ito::Param paramVal = ito::Param("cameraNumber", ito::ParamBase::Int, 0, 16, 0, tr("consecutive number of the connected camera (starting with 0, default)").toAscii().data());
    m_initParamsOpt.append(paramVal);
    //paramVal = ito::Param("Init-Dialog", ito::ParamBase::Int, 0, 1, 0, tr("If true, a camera selection dialog is opened during startup").toAscii().data());
    //m_initParamsOpt.append(paramVal);

   return;
}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVGrabberInterface::~OpenCVGrabberInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(OpenCVGrabberinterface, OpenCVGrabberInterface)

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal OpenCVGrabber::showConfDialog(void)
{
	ito::RetVal retValue(ito::retOk);
	int colorSelect_old = 8;
	int color_old = 0;
	int colorSelect_new = 8;
	int color_new = 0;
	double offset_new = 0.0;

    dialogOpenCVGrabber *confDialog = new dialogOpenCVGrabber();

	connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
	connect(confDialog, SIGNAL(changeParameters(QMap<QString, ito::ParamBase>)), this , SLOT(updateParameters(QMap<QString, ito::ParamBase>)));

	colorSelect_old = m_params["channel"].getVal<int>();
	color_old = m_params["color"].getVal<int>();

	confDialog->setVals(&m_params);
    if (confDialog->exec())
    {
        confDialog->getVals(&m_params);
		colorSelect_new = m_params["channel"].getVal<int>();
		color_new = m_params["color"].getVal<int>();
		offset_new = m_params["offset"].getVal<int>();
		if (color_new != color_old)
		{
            retValue += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("color", ito::ParamBase::Int, color_new)), NULL);
		}
		else if (colorSelect_new != colorSelect_old)
		{
            retValue += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("channel", ito::ParamBase::Int, colorSelect_new)), NULL);
		} 
		else		
		{
            retValue += setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Int, offset_new)), NULL);
		}
    }
    delete confDialog;

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
OpenCVGrabber::OpenCVGrabber() : AddInGrabber(), m_isgrabbing(false), m_pCam(NULL), m_CCD_ID(0), m_camStatusChecked(false)
{
   ito::Param paramVal("name", ito::ParamBase::String, "OpenCVGrabber", NULL);
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 2048, 0, tr("x-start for software ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 2048, 0, tr("y-start for software ROI").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizex", ito::ParamBase::Int, 1, 2048, 2048, tr("ROI-Size in x").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("sizey", ito::ParamBase::Int, 1, 2048, 2048, tr("ROI-Size in y").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 24, 8, tr("bpp").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.000010, 10.0, 0.01, tr("Integrationtime of CCD [s]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("brightness", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("brightness [0..1]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("contrast", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("contrast [0..1]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("saturation", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("saturation [0..1]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("hue", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("hue [0..1]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Gain [0..1]").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);
   
   paramVal = ito::Param("channel", ito::ParamBase::Int, 0, 3, 0, tr("selected color channel (all available (0, default), R (1), G (2), B (3)").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

   paramVal = ito::Param("colorConversion", ito::ParamBase::Int, 0, 1, 1, tr("no conversion (0), RGB->Grayscale (1, default). If the camera image only has one channel or channel>0, this parameter is ignored").toAscii().data());
   m_params.insert(paramVal.getName(), paramVal);

}

//----------------------------------------------------------------------------------------------------------------------------------
OpenCVGrabber::~OpenCVGrabber()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// Funktion to set and update data imformations
ito::RetVal OpenCVGrabber::checkCameraAbilities()
{
	bool camRetVal = false;
    m_camStatusChecked = false;
	ito::RetVal retValue;

    //acquire test image in order to get knownledge about camera's abilities
	//camRetVal = m_pCam->grab();
	//camRetVal = m_pCam->retrieve(m_pDataMatBuffer);
    camRetVal = m_pCam->read(m_pDataMatBuffer);
    if(camRetVal)
    {
        m_imgChannels = m_pDataMatBuffer.channels();
        m_imgCols = m_pDataMatBuffer.cols;
        m_imgRows = m_pDataMatBuffer.rows;
        m_imgBpp = m_pDataMatBuffer.elemSize1() * 8;

        static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax( m_imgCols );
        static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax( m_imgRows );
        m_params["sizex"].setVal<int>(m_imgCols);
        m_params["sizey"].setVal<int>(m_imgRows);

        static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax( m_imgCols-1 );
        static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax( m_imgRows-1 );
        m_params["x0"].setVal<int>(0);
        m_params["y0"].setVal<int>(0);


        //m_params["bpp"].setMin(8);
        //m_params["bpp"].setMax(elemSize1*8);
        m_params["bpp"].setMeta( new ito::IntMeta(8, m_imgBpp), true);
        m_params["bpp"].setVal<int>(m_imgBpp);

	    if(m_imgBpp < 8 || m_imgBpp > 32) 
	    {
		    retValue += ito::RetVal(ito::retError, 0, tr("unknown bpp").toAscii().data());
	    }

        m_camStatusChecked = true;
    }
    else
    {
        retValue += ito::RetVal(ito::retError,0,"could not check format of camera image");
        
    }

	return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal OpenCVGrabber::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QMap<QString,ito::Param>::iterator it;
    ito::RetVal retValue = apiGetParamFromMapByKey(m_params, val->getName(), it, false);

    if(!retValue.containsError())
    {
        *val = it.value();
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

   return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This method copies the value of val to to the m_params-parameter and sets the corresponding camera parameters.

    \param [in] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal OpenCVGrabber::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QMap<QString,ito::Param>::iterator it;
    ito::RetVal retValue = apiGetParamFromMapByKey(m_params, val->getName(), it, true);

    if(!retValue.containsError())
    {
        //here the parameter val is checked if it can be casted to *it and if the
        //possible meta information requirements of *it are met.
        retValue += apiValidateParam(*it, *val, false, true);
	}

	if (!retValue.containsError())
	{
        //here you can add specific sub-checks for every keyword and finally put the value into (*it).
        retValue += it->copyValueFrom( &(*val) );
    }

    if (!retValue.containsError())
	{
        retValue += checkCameraAbilities();
		emit parametersChanged(m_params);
	}

	if (waitCond) 
    {
        waitCond->returnValue = retValue;
		waitCond->release();
    }

   return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    //exceptionally, this init dialog is executed in the main thread of itom (m_callInitInNewThread == false in OpenCVGrabberInterface)
	ItomSharedSemaphoreLocker locker(waitCond);
	ito::RetVal retValue(ito::retOk);
    bool ret;

    m_CCD_ID = (*paramsOpt)[0].getVal<int>();

    /*if((*paramsOpt)[1].getVal<int>() == 1)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Dialog for manually selecting any camera not yet implemented.").toAscii().data());
    }
    else
    {*/
        m_pCam = new cv::VideoCapture();
        ret = m_pCam->open(m_CCD_ID);

        if(!m_pCam->isOpened())
        {
            retValue += ito::RetVal::format(ito::retError,0,tr("Camera (%i) could not be opened").toAscii().data(), m_CCD_ID);
        }
    //}

	if(!retValue.containsError())
	{
		//camRetVal = m_pCam->get(CV_CAP_PROP_FOURCC);		//4-character code of codec.
		//
		//tempformat = static_cast<unsigned long>(camRetVal);

		//format[0] = tempformat & 0XFF; 
		//format[1] = (tempformat & 0XFF00) >> 8;
		//format[2] =	(tempformat & 0XFF0000) >> 16;
		//format[3] = (tempformat & 0XFF000000) >> 24;

        m_params["sizex"].setVal<int>( m_pCam->get(CV_CAP_PROP_FRAME_WIDTH) );
        m_params["sizey"].setVal<int>( m_pCam->get(CV_CAP_PROP_FRAME_HEIGHT) );

		if(m_pCam->get(CV_CAP_PROP_BRIGHTNESS) == 0.0)	//Brightness of the data (only for cameras).
        {
            m_params.remove("brightness");
        }
        if(m_pCam->get(CV_CAP_PROP_CONTRAST) == 0.0) //Contrast of the data (only for cameras).
        {
            m_params.remove("contrast");
        } 
        if(m_pCam->get(CV_CAP_PROP_HUE) == 0.0) //Hue of the data (only for cameras).
        {
            m_params.remove("hue");
        }
        if(m_pCam->get(CV_CAP_PROP_SATURATION) == 0.0) //Saturation of the data (only for cameras).
        {
            m_params.remove("saturation");
        }
        if(m_pCam->get(CV_CAP_PROP_GAIN) == 0.0) //Gain of the data (only for cameras).
        {
            m_params.remove("gain");
        }
        if(m_pCam->get(CV_CAP_PROP_EXPOSURE) == 0.0) //Exposure of the data (only for cameras).
        {
            m_params.remove("integration_time");
        }

        double test = m_pCam->get(CV_CAP_PROP_CONVERT_RGB);
        test = m_pCam->get(CV_CAP_PROP_FOURCC); // 4-character code of codec.
        test = m_pCam->get(CV_CAP_PROP_FRAME_COUNT); // Number of frames in the video file.
        test = m_pCam->get(CV_CAP_PROP_FORMAT); //
        test = 0.0;

	}

	if(!retValue.containsError())
	{
		retValue += checkCameraAbilities();
		
        retValue += checkData();
	}
	
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
        
    setInitialized(true); //init method has been finished (independent on retval)
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::close(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	
	if (m_timerID > 0)
	{ 
		killTimer(m_timerID);
		m_timerID=0;
	}

	retValue += stopDevice(NULL);

    if(m_pCam && m_pCam->isOpened())
    {
        m_pCam->release();
    }

    if(m_pCam)
    {
        delete m_pCam;
        m_pCam = NULL;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
	
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::startDevice(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	
	incGrabberStarted();
	
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::stopDevice(ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

	decGrabberStarted();

	if(grabberStartedCount() < 0)
	{
		retValue += ito::RetVal(ito::retWarning, 0, tr("the grabber already had zero users.").toAscii().data());
        setGrabberStarted(0);
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
	return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	bool RetCode = false;

	if (grabberStartedCount() <= 0)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toAscii().data());
	}
    else if(m_camStatusChecked == false)
    {
        retValue += ito::RetVal(ito::retError,0,tr("Cannot acquire image since camera status is unverified").toAscii().data());
    }
	else
	{
		m_isgrabbing = true;
        cv::Mat temp;

        ////workaround, get old images in order to clean buffer queue, which leads to delivery of old images
        m_pCam->retrieve(temp);
        m_pCam->retrieve(temp);
        m_pCam->retrieve(temp);
        m_pCam->retrieve(temp);
        m_pCam->retrieve(temp);

        RetCode = m_pCam->grab();
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

	bool RetCode = false;
    cv::Mat *internalMat = NULL;

	int curxsize = m_params["sizex"].getVal<int>();
	int curysize = m_params["sizey"].getVal<int>();
	int x0 = m_params["x0"].getVal<int>();
	int y0 = m_params["y0"].getVal<int>();
    bool resizeRequired = (x0 > 0 || y0 > 0);

    ito::DataObject *dataObj = &m_data;
    if(externalDataObject)
    {
        dataObj = externalDataObject;
    }

    bool hasListeners = false;
    if(m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }

	if (m_isgrabbing == false)
	{
		retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toAscii().data());
	}
	else
	{
        ////the following lines are commented, since m_pDataMatBuffer already is filled in acquire (command read instead of grab and retrieve, which leads to buffering delays)
        RetCode = m_pCam->retrieve(m_pDataMatBuffer); //get image from cam, m_pDataMatBuffer is reference to internal memory
        //RetCode = m_pCam->read(m_pDataMatBuffer);
        
        if(!RetCode)
        {
            retValue+=ito::RetVal(ito::retError, 0, tr("Error: no data grabbed").toAscii().data());
        }
        if(m_pDataMatBuffer.cols == 0 || m_pDataMatBuffer.rows == 0)
        {
            retValue+=ito::RetVal(ito::retError,0,tr("Error: grabbed image is empty").toAscii().data());
        }

        if(!retValue.containsError())
        {
            int desiredChannel = m_params["channel"].getVal<int>();
            if(desiredChannel > 0 && m_imgChannels == 1)
            {
                desiredChannel = 0; //no r,g,b channel in camera image available (grayscale camera)
            }

            int colorConversion = m_params["colorConversion"].getVal<int>();
            if(colorConversion == 1 /*rgb2gray*/ && (m_imgChannels == 1 || desiredChannel > 0)) 
            {
                colorConversion = 0; //grayscale camera image or selected channel -> no conversion necessary
            }

            int desiredBpp = m_params["bpp"].getVal<int>();
            cv::Mat tempImage;

            if(m_imgCols != curxsize || m_imgRows != curysize)
            {
                resizeRequired = true;
            }

            if(m_imgBpp != 8 && m_imgBpp != 16)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: bpp other than 8 or 16 not allowed.").toAscii().data());
            }
            else if(m_imgChannels != 1 && m_imgChannels != 3)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: channels sizes other than 1 or 3 not allowed.").toAscii().data());
            }
            else if((desiredBpp != 8 && desiredBpp != 16))
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: desired bpp must be 8 or 16 bit.").toAscii().data());
            }
            else
            {   
                //step 1. check ROI
                if(resizeRequired == false)
                {
                    tempImage = m_pDataMatBuffer;
                }
                else
                {
                    cv::Range ranges[] = { cv::Range(y0,y0+curysize), cv::Range(x0,x0+curxsize) };
                    tempImage = cv::Mat(m_pDataMatBuffer, ranges);
                }

                //step 2. check whether 3 channel color should be transformed to 1 channel grayscale
                if(m_imgChannels == 3 && colorConversion > 0)
                {
                    int conversionCode = CV_BGR2GRAY; //camera provides BGR images in OpenCV
                    switch(colorConversion)
                    {
                    case 1:
                        conversionCode = CV_BGR2GRAY;
                        break;
                    default:
                        retValue += ito::RetVal(ito::retError, 0, tr("unknown conversion code.").toAscii().data());
                        break;
                    }

                    cv::cvtColor(tempImage, tempImage, conversionCode, 0);
                }

                //step 3: create m_data (if not yet available)
                retValue += checkData(externalDataObject);

                if(!retValue.containsError())
                {

                    //step 4: check whether tempImage must be converted to other type
                    if(desiredBpp != m_imgBpp)
                    {
                        if(desiredBpp == 8)
                        {
                            tempImage.convertTo( tempImage, CV_8U );
                        }
                        else if(desiredBpp == 16)
                        {
                            tempImage.convertTo( tempImage, CV_16U );
                        }
                        else
                        {
                            retValue += ito::RetVal(ito::retError, 0, tr("Error while converting data format. Unsupported format.").toAscii().data());
                        }
                    }
                
                    
                }
                     
                if(!retValue.containsError())
                {
                    if(tempImage.channels() == 1)
                    {
                        internalMat = (cv::Mat*)(dataObj->get_mdata()[0]);
                        tempImage.copyTo( *(internalMat) );

                        if(externalDataObject && hasListeners)
                        {
                            internalMat = (cv::Mat*)(m_data.get_mdata()[0]);
                            tempImage.copyTo( *(internalMat) );
                        }
                    }
                    else if(tempImage.channels() == 3 && desiredChannel == 0)
                    {
                        cv::Mat out[] = { *(cv::Mat*)(dataObj->get_mdata()[0]) , *(cv::Mat*)(dataObj->get_mdata()[1]) , *(cv::Mat*)(dataObj->get_mdata()[2]) };
                        int fromTo[] = {0,2,1,1,2,0}; //implicit BGR (camera) -> RGB (dataObject style) conversion
                        cv::mixChannels( &tempImage, 1, out, 3, fromTo, 3 );

                        if(externalDataObject && hasListeners)
                        {
                            cv::Mat out[] = { *(cv::Mat*)(m_data.get_mdata()[0]) , *(cv::Mat*)(m_data.get_mdata()[1]) , *(cv::Mat*)(m_data.get_mdata()[2]) };
                            cv::mixChannels( &tempImage, 1, out, 3, fromTo, 3 );
                        }
                    }
                    else if(tempImage.channels() == 3 && desiredChannel > 0)
                    {
                        cv::Mat out[] = { *(cv::Mat*)(dataObj->get_mdata()[0]) };
                        int fromTo[] = {0,0};
                        switch(desiredChannel)
                        {
                        case 1: fromTo[0] = 2; break; //red
                        case 2: fromTo[0] = 1; break; //green
                        default /*3*/: fromTo[0] = 0; break; //blue
                        }
                        cv::mixChannels( &tempImage, 1, out, 1, fromTo, 1 );

                        if(externalDataObject && hasListeners)
                        {
                        cv::Mat out[] = { *(cv::Mat*)(m_data.get_mdata()[0]) };
                        cv::mixChannels( &tempImage, 1, out, 1, fromTo, 1 );
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal(ito::retError,0,tr("unknown color, conversion... combination in retrieveImage").toAscii().data());
                    }
                }
            }
            
        }

		m_isgrabbing = false;
	}

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::checkData(ito::DataObject *externalDataObject)
{
    

    if(!m_camStatusChecked)
    {
        return ito::RetVal(ito::retError,0,tr("current camera status is undefined").toAscii().data());
    }

    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    
    int futureType;
    int bpp = m_params["bpp"].getVal<int>();
    if(bpp <= 8)
    {
        futureType = ito::tUInt8;
    }
    else if(bpp <= 16)
    {
        futureType = ito::tUInt16;
    }
    else if(bpp <= 32)
    {
        futureType = ito::tInt32;
    }
    else 
    {
        futureType = ito::tFloat64;
    }

    int futurePlanes = 1;
    if( m_imgChannels == 3 && m_params["channel"].getVal<int>() == 0 /*all*/ && m_params["colorConversion"].getVal<int>() == 0 /*no conversion*/)
    {
        futurePlanes = 3;
    }

    if(externalDataObject == NULL)
    {
        if(futurePlanes == 1)
        {
            if(m_data.getDims() != 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
            {
                m_data = ito::DataObject(futureHeight,futureWidth,futureType);
            }
        }
        else
        {
            if(m_data.getDims() != 3 || m_data.getSize(0) != 3 || m_data.getSize(1) != (unsigned int)futureHeight || m_data.getSize(2) != (unsigned int)futureWidth || m_data.getType() != futureType)
            {
                m_data = ito::DataObject(3,futureHeight,futureWidth,futureType);
            }
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if(externalDataObject->getDims() == 0) //empty external dataObject
        {
            if(futurePlanes == 1)
            {
                *externalDataObject = ito::DataObject(futureHeight,futureWidth,futureType);
            }
            else
            {
                *externalDataObject = ito::DataObject(futurePlanes,futureHeight,futureWidth,futureType);
            }
        }
        else if(externalDataObject->calcNumMats () > 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toAscii().data());            
        }
        else if(externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toAscii().data());
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal OpenCVGrabber::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

	retValue += retrieveData();

	if(!retValue.containsError())
	{
		sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

		if(dObj)
		{
			(*dObj) = m_data;
		}
	}

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
		waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as a deep copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the 
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal OpenCVGrabber::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
	{
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toAscii().data());
    }

    if(!retValue.containsError())
	{
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
    }

    if(!retValue.containsError())
	{
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
		waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void OpenCVGrabber::updateParameters(QMap<QString, ito::ParamBase> params)
{
	int colorSelect_old = 0;
	int color_old = 0;
	int colorSelect_new = 0;
	int color_new = 0;
	double offset_new = 0.0;
	double value = 0.0;

	char name[40]={0};
	
	colorSelect_old = m_params["colorSelect"].getVal<int>();
	color_old = m_params["color"].getVal<int>();

	foreach(const ito::ParamBase &param1, params)
	{	
		memset(name,0,sizeof(name));
		sprintf(name,"%s", param1.getName());
		if(!strlen(name))
			continue;
		QMap<QString, ito::Param>::iterator paramIt = m_params.find(name);
		if (paramIt != m_params.end())
		{
			value = param1.getVal<double>();
			paramIt.value().setVal<double>(value);
		}
	}

	colorSelect_new = m_params["colorSelect"].getVal<int>();
	color_new = m_params["color"].getVal<int>();
	offset_new = m_params["offset"].getVal<double>();
	
	if (color_new != color_old)
	{
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("color", ito::ParamBase::Int, color_new)), NULL);
	}
	else if (colorSelect_new != colorSelect_old)
	{
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("colorSelect", ito::ParamBase::Int, colorSelect_new)), NULL);
	} 
	else		
	{
        setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Int, offset_new)), NULL);
	}
}
