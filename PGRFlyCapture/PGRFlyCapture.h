/* ********************************************************************
    Plugin "PGRFlyCapture" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2017, twip optical solutions GmbH
    Copyright (C) 2017, Institut für Technische Optik, Universität Stuttgart

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

#define MAXPGR 10




//----------------------------------------------------------------------------------------------------------------------------------
class PGRFlyCaptureInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
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

        void dockWidgetVisibilityChanged(bool visible);

        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class PGRFlyCaptureInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:

        enum ExtendedShutterType
        {
            UNINITIALIZED,
            NO_EXTENDED_SHUTTER,
            DRAGONFLY_EXTENDED_SHUTTER,
            GENERAL_EXTENDED_SHUTTER
        };

        ito::RetVal flyCapSetAndGetParameter(const QString &name, unsigned int &value, FlyCapture2::PropertyType type, bool autoManualMode = false, bool onOff = true);
        ito::RetVal flyCapSetAndGetParameter(const QString &name, float &value, FlyCapture2::PropertyType type, bool autoManualMode = false, bool onOff = true);

        ito::RetVal flyCapGetParameter(const QString &name, unsigned int &value, FlyCapture2::PropertyType type);
        ito::RetVal flyCapGetParameter(const QString &name, float &value, FlyCapture2::PropertyType type);

        ito::RetVal flyCapChangeFormat7_(bool changeBpp, bool changeROI, int bpp = -1, int x0 = -1, int y0 = -1, int width = -1, int height = -1);
        ito::RetVal flyCapSetExtendedShutter(bool enabled);
        ito::RetVal flyCapSynchronizeFrameRateShutter();

        bool m_isgrabbing;
        FlyCapture2::Camera m_myCam;
        FlyCapture2::PGRGuid m_myGUID;
        int m_camIdx;
        int m_colorCam;
        bool m_colouredOutput;
        bool m_RunSoftwareSync;
        double m_gainMax;
        double m_gainMin;
        double m_offsetMax;
        double m_offsetMin;

        bool m_pendingIdleGrabs;

        ExtendedShutterType m_extendedShutter;
        FlyCapture2::EmbeddedImageInfo m_embeddedInfo;
        bool m_hasFrameInfo;

        FlyCapture2::Format7ImageSettings m_currentFormat7Settings;
        FlyCapture2::Format7PacketInfo m_currentPacketInfo;
        FlyCapture2::Format7Info m_format7Info;
        FlyCapture2::InterfaceType m_interfaceType;
        bool m_hasFormat7;

        ito::RetVal m_acquisitionStatus;
        FlyCapture2::Image m_imageBuffer;
        double m_firstTimestamp;

        unsigned int GetBppFromPixelFormat( FlyCapture2::PixelFormat pixelFormat );
        bool GetPixelFormatFromVideoMode( FlyCapture2::VideoMode mode, bool stippled, FlyCapture2::PixelFormat* pixFormat);
        bool GetResolutionFromVideoMode( FlyCapture2::VideoMode mode, int &sizeX, int &sizeY);
        double GetFrameTimeFromFrameRate( FlyCapture2::FrameRate frameRate );
        FlyCapture2::FrameRate GetSuitAbleFrameRateFromFrameTime( double frameTime );
        double timeStampToDouble(const FlyCapture2::TimeStamp &timestamp);

        ito::RetVal checkError(const FlyCapture2::Error &error);

    signals:

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        //! overwrite this function if you registered exec funcs. Once the exec function is called, this method is executed.
        virtual ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);

};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // PGRFlyCapture_H
