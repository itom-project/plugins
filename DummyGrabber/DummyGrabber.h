/* ********************************************************************
    Plugin "DummyGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#ifndef DUMMYGRABBER_H
#define DUMMYGRABBER_H

#include "common/addInGrabber.h"
#include "dialogDummyGrabber.h"

#include <qsharedpointer.h>
#include <qtimer.h>

//----------------------------------------------------------------------------------------------------------------------------------
class DummyGrabberInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this DummyGrabberInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */

    public:
        DummyGrabberInterface();                    /*!< Constructor */
        ~DummyGrabberInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of DummyGrabber and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of DummyGrabber, given by *addInInst */

};

class SimulatedCam
{
    protected:
        ~SimulatedCam();
        SimulatedCam();

public:
        friend class DummyGrabber;
        
        enum errorValues
        {
            okay = 0,
            triedInitTwice = -1,
            initFailedBuffer = -2,
            wrongBPP = -3,
            missingInit = -4,
            missingAcquire = -10,
            missingBuffer = -11,
            missingStart = -12,
            binningFailed = -21,
            stillRunning = -22,
        };

        int initCamera(int maxXSize, int maxYSize, int maxBitDepth);
        int waitForImage(int timeoutMS);
        char* getImageBuffer(void);
        int acquireImage(void);
        int prepareCamera(void);
        int stopCamera(void);
        int setFrameTime(int newframeTimeMS){m_frameTimeMS = newframeTimeMS; return okay;}
        int getFrameTime(void){return m_frameTimeMS;}
        int setOffsetGain(int /*gain*/, int /*offset*/){return okay;}
        int setBinning(int binX, int binY);
        int getBinning(int &binX, int &binY);
        int getSize(int &sizeX, int &sizeY);

    private:
        int m_frameTimeMS;
        int m_binX;
        int m_binY;
        int m_xsize_max; 
        int m_ysize_max;
        int m_xsize; 
        int m_ysize;
        int m_bpp;
        bool m_initDone;
        bool m_started;
        bool m_grabbed;
        void * m_myInternalBuffer;
};

//----------------------------------------------------------------------------------------------------------------------------------
class DummyGrabber : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~DummyGrabber();
        DummyGrabber();

//        ito::RetVal checkData();
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class DummyGrabberInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_isgrabbing;
        SimulatedCam myCam;

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
        void GainOffsetPropertiesChanged(double gain, double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    private slots:
        ito::RetVal updateCamParams(void);

        void dockWidgetVisibilityChanged(bool visible);


};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // DUMMYGRABBER_H
