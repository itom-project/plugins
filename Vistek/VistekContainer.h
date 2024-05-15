/* ********************************************************************
    Plugin "Vistek" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef VISTEKCONTAINER_H
#define VISTEKCONTAINER_H

#include "common/addInGrabber.h"
#include "dialogVistek.h"
#include "SVGigE.h"

#include <qsharedpointer.h>
#include <QTimerEvent>
#include <qmutex.h>

struct VistekCam
{
    public:
        QString camModel;
        QString camSerialNo;
        QString camVersion;
        QString camIP;
        QString camManufacturer;
        int sensorPixelsX;
        int sensorPixelsY;
        bool started;
};

class VistekContainer : QObject
{

    Q_OBJECT

    public:
        static VistekContainer* getInstance();
        ito::RetVal initCameraContainer();

        VistekCam getCamInfo(int CamNo);
        const int getNextFreeCam();
        const int getCameraBySN(const QString &camSerialNo);
        void freeCameraStatus(const int camnumber);
        CameraContainerClient_handle getCameraContainerHandle();

    protected:
        CameraContainerClient_handle m_camClient;
        QMutex m_mutex;
        bool m_initialized;
        QVector<VistekCam> m_cameras;


    private:
        VistekContainer(void);
        VistekContainer(VistekContainer  &/*copyConstr*/) {}
        ~VistekContainer(void);

        ito::RetVal checkError(const char *prependStr, SVGigE_RETURN returnCode);

        static VistekContainer *m_pVistekContainer;

        //!< singleton nach: http://www.oop-trainer.de/Themen/Singleton.html
        class VistekContainerSingleton
        {
            public:
                ~VistekContainerSingleton()
                {
                    #pragma omp critical
                    {
                        if( VistekContainer::m_pVistekContainer != NULL)
                        {
                            delete VistekContainer::m_pVistekContainer;
                            VistekContainer::m_pVistekContainer = NULL;
                        }
                    }
                }
        };
        friend class VistekContainerSingleton;
};

#endif // VISTEKCONTAINER_H
