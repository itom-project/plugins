/* ********************************************************************
Plugin "Tucsen" for itom software
URL : http ://www.uni-stuttgart.de/ito
Copyright(C) 2025, Institut fuer Technische Optik (ITO),
                   Universitaet Stuttgart, Germany;
                   Universidade Federal de Alagoas (UFAL), Brazil

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

#ifndef Tucsen_H
#define Tucsen_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>
#include "dialogTucsen.h"
#include <qtimer.h>

#include <TUDefine.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    TucsenInterface
  *
  *\brief    Interface-Class for Tucsen-Class
  *
  *    \sa    AddInDataIO, MyGrabber
  *
  */
class TucsenInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        TucsenInterface();
        ~TucsenInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    Tucsen

  */
class Tucsen : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~Tucsen();
        //! Constructor
        Tucsen();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */

    public:
        friend class TucsenInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        inline double musToS(const double &mus) { return mus * 1.0e-6; }
        inline double sToMus(const double &s) { return s * 1.0e6; }
        inline double msToS(const double &mus) { return mus * 1.0e-3; }
        inline double sToMs(const double &s) { return s * 1.0e3; }
        ito::RetVal getParamListRaw();

        //enum Feature
        //{
        //    fBpp = 0x001,
        //    fSize = 0x002,
        //    fBinning = 0x004,
        //    fExposure = 0x008,
        //    fGigETransport = 0x010,
        //    fTrigger = 0x020,
        //    fGain = 0x040,
        //    fOffset = 0x080,
        //    fGamma = 0x100,
        //    fAll = fBpp | fSize | fBinning | fExposure | fGigETransport | fTrigger | fGain | fOffset | fGamma
        //};

	    TUCAM_FRAME m_frame; // The frame object
        TUCAM_INIT m_itApi; // SDK API initialized object
        TUCAM_OPEN m_opCam; // Open camera object
        TUCAM_TRIGGER_ATTR m_trg;
        TUCAM_PROP_ATTR m_props[(int)TUIDP_ENDPROPERTY - (int) TUIDP_GLOBALGAIN];
        bool m_hasProp[(int)TUIDP_ENDPROPERTY - (int)TUIDP_GLOBALGAIN];

        bool m_isgrabbing; /*!< Check if acquire was executed */
        ito::RetVal m_acquisitionStatus;
        QTimer *m_aliveTimer;
        QThread *m_aliveTimerThread;

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

#endif // Tucsen_H
