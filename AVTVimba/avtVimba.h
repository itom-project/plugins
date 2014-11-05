/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef AVTVIMBA_H
#define AVTVIMBA_H

#include "common/addInGrabber.h"
#include "opencv/cv.h"
#include <qsharedpointer.h>
#include "dialogAvtVimba.h"
#include <VimbaCPP/Include/VimbaCPP.h>

using namespace AVT::VmbAPI;

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabberInterface 
  *
  *\brief    Interface-Class for MyGrabber-Class
  *
  *    \sa    AddInDataIO, MyGrabber
  *
  */
class AvtVimbaInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        AvtVimbaInterface();
        ~AvtVimbaInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabber

  */
class AvtVimba : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~AvtVimba();
        //! Constructor
        AvtVimba();
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        
    public:
        friend class AvtVimbaInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
		ito::RetVal checkError(VmbErrorType errCode, const char* prefix = NULL);

		ito::RetVal getIntFeatureByName(const char *name, VmbInt64_t &value);
        ito::RetVal getIntFeatureByName(const char *name, VmbInt64_t &value, VmbInt64_t &max, VmbInt64_t &min, VmbInt64_t &inc);
		ito::RetVal getEnumFeatureByName(const char *name, std::string &value, VmbInt64_t &idx);
		ito::RetVal getDblFeatureByName(const char *name, double &value);
        ito::RetVal getDblFeatureByName(const char *name, double &value, double &min, double &max);
		ito::RetVal setDblFeature(const char *name, const double &value);
		ito::RetVal setIntFeature(const char *name, const int &value);
		ito::RetVal setEnumFeature(const char *name, const char *eValue);
        ito::RetVal setEnumFeature(const char *name, VmbInt64_t value);
		ito::RetVal getRange(const char *name, double &max, double &min);
		ito::RetVal getRange(const char *name, VmbInt64_t &max, VmbInt64_t &min, VmbInt64_t &inc);

        inline double musToS(const double &mus) { return mus * 1.0e-6; }
        inline double sToMus(const double &s) { return s * 1.0e6; }
        inline double msToS(const double &mus) { return mus * 1.0e-3; }
        inline double sToMs(const double &s) { return s * 1.0e3; }

        enum Feature
        {
            fBpp = 0x001,
            fSize = 0x002,
            fBinning = 0x004,
            fExposure = 0x008,
            fGigETransport = 0x010,
            fTrigger = 0x020,
            fGain = 0x040,
            fOffset = 0x080,
            fAll = fBpp | fSize | fBinning | fExposure | fGigETransport | fTrigger | fGain | fOffset
        };

        ito::RetVal synchronizeParameters(int features);


        bool m_isgrabbing; /*!< Check if acquire was executed */
		ito::RetVal m_acquisitionStatus;
		CameraPtr m_camera;
		FramePtr m_frame;
        double gain_range[2];

        enum TransportType
        {
            tGigE,
            tFirewire
        };

        struct BppEnum
        {
            BppEnum() : bppMono8(-1), bppMono10(-1), bppMono12(-1), bppMono14(-1) {}
            VmbInt64_t bppMono8;
            VmbInt64_t bppMono10;
            VmbInt64_t bppMono12;
            VmbInt64_t bppMono14;
        };

        struct TriggerSourceEnum
        {
            TriggerSourceEnum() : triggerFreerun(-1), triggerLine1(-1), triggerLine2(-1), triggerLine3(-1), triggerLine4(-1), triggerFixedRate(-1), triggerSoftware(-1), triggerInputLines(-1) {}
            VmbInt64_t triggerFreerun;
            VmbInt64_t triggerLine1;
            VmbInt64_t triggerLine2;
            VmbInt64_t triggerLine3;
            VmbInt64_t triggerLine4;
            VmbInt64_t triggerFixedRate;
            VmbInt64_t triggerSoftware;
            VmbInt64_t triggerInputLines;
        };

        struct TriggerActivationEnum
        {
            TriggerActivationEnum() : taRisingEdge(-1), taFallingEdge(-1), taAnyEdge(-1), taLevelHigh(-1), taLevelLow(-1) {}
            VmbInt64_t taRisingEdge;
            VmbInt64_t taFallingEdge;
            VmbInt64_t taAnyEdge;
            VmbInt64_t taLevelHigh;
            VmbInt64_t taLevelLow;
        };

        VmbInterfaceType m_interfaceType;
        TriggerSourceEnum m_triggerSourceEnum;
        TriggerActivationEnum m_triggerActivationEnum;
        BppEnum m_bppEnum;
        int timeoutMS;
        
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

#endif // AVTVIMBA_H
