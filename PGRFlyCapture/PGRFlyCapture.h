/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
	Copyright (C) 2013, Institut für Technische Optik, Universität Stuttgart

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

#ifndef PGRFLYCAPTURE_H
#define PGRFLYCAPTURE_H

#include "common/addInGrabber.h"
#include "dialogPGRFlyCapture.h"

#include "FlyCapture2.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

#define MAXPGR 5

//----------------------------------------------------------------------------------------------------------------------------------
class PGRFlyCaptureInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this PGRFlyCaptureInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        PGRFlyCaptureInterface();                    /*!< Constructor */
        ~PGRFlyCaptureInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of PGRFlyCapture and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of PGRFlyCapture, given by *addInInst */

};

//----------------------------------------------------------------------------------------------------------------------------------
class PGRFlyCapture : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~PGRFlyCapture();
        PGRFlyCapture();

//        ito::RetVal checkData();
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class PGRFlyCaptureInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_isgrabbing;
        FlyCapture2::Camera m_myCam;
        FlyCapture2::PGRGuid m_myGUID;
        int m_camIdx;
        int m_colorCam;
        bool m_RunSync;
        bool m_RunSoftwareSync;
        double m_gainMax;
        double m_gainMin;
        double m_offsetMax;
        double m_offsetMin;

        double m_acquireTime;    /*!< Timestamp for acquire in seconds relative to cpu ticks */
		double m_last_acquireTime;
       
        bool m_isInFormat7;

        unsigned int GetBppFromPixelFormat( FlyCapture2::PixelFormat pixelFormat );
        bool GetPixelFormatFromVideoMode( FlyCapture2::VideoMode mode, bool stippled, FlyCapture2::PixelFormat* pixFormat);
        bool GetResolutionFromVideoMode( FlyCapture2::VideoMode mode, int &sizeX, int &sizeY);
        double GetFrameTimeFromFrameRate( FlyCapture2::FrameRate frameRate );
        FlyCapture2::FrameRate GetSuitAbleFrameRateFromFrameTime( double frameTime );

    signals:

    public slots:
        /*ito::RetVal getParam(const char *name, QSharedPointer<char> val, QSharedPointer<int> len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getParam(const char *name, QSharedPointer<double> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const char *val, const int len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const double val, ItomSharedSemaphore *waitCond = NULL);*/

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        //ito::RetVal setVal(const void *dObj, const int length, ItomSharedSemaphore *waitCond);

        //void dataParametersChanged(int sizex, int sizey, int bpp);
        void GainPropertiesChanged(double gain);
        void OffsetPropertiesChanged(double offset);
        void IntegrationPropertiesChanged(double integrationtime);

};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // PGRFlyCapture_H
