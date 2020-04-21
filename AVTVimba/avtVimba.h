/* ********************************************************************
Plugin "Roughness" for itom software
URL : http ://www.uni-stuttgart.de/ito
Copyright(C) 2016, Institut fuer Technische Optik (ITO), 
                   Universitaet Stuttgart, Germany; 
                   IPROM, TU Braunschweig, Germany

This file is part of a plugin for the measurement software itom.

This itom - plugin is free software; you can redistribute it and / or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or(at
your option) any later version.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef AVTVIMBA_H
#define AVTVIMBA_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>
#include "dialogAvtVimba.h"
#include <VimbaCPP/Include/VimbaCPP.h>
#include "avtEnums.h"
#include <qtimer.h>

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
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
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
            fGamma = 0x100,
            fAll = fBpp | fSize | fBinning | fExposure | fGigETransport | fTrigger | fGain | fOffset | fGamma
        };

        ito::RetVal synchronizeParameters(int features);


        bool m_isgrabbing; /*!< Check if acquire was executed */
        ito::RetVal m_acquisitionStatus;
        CameraPtr m_camera;
        FramePtr m_frame;
        QTimer *m_aliveTimer;
        QThread *m_aliveTimerThread;

        VmbInterfaceType m_interfaceType;
        TriggerSourceEnum m_triggerSourceEnum;
        TriggerActivationEnum m_triggerActivationEnum;
        BppEnum m_bppEnum;
        int timeoutMS;
        const char* nameExposureTime;
        
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

        void aliveTimer_fire();
};

#endif // AVTVIMBA_H
