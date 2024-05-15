/* ********************************************************************
    Plugin "cmu1394" for itom software
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

/*! \file cmu1394.h
   \brief   main header file for a generic firewire support based on the CMU-driver
   \detailed This is the main header file for the generic firewire support based on the free CMU-driver.
   This driver can be downloaded from http://www.cs.cmu.edu/~iwan/1394/. Current version is 6.4.6.

   \author ITO
   \date 02.2012
*/
#ifndef CMU1394_H
#define CMU1394_H

#include "common/addInGrabber.h"
#include "dialogcmu1394.h"

#include <qsharedpointer.h>

#ifdef WIN32
    #include <windows.h>
#endif

#ifndef Q_MOC_RUN
#define MY1394CAMERA_EXPORTS 1
#include "./cmu1394/include/1394Camera.h"
#endif



#ifdef USEOPENMP
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    CMU1394
  *\brief    class to use firewire-Cameras with the generic CMU-Driver
  *
  *         This class can be used to get firewire cameras running with the generic CMU firewire DLL.
  *            The generic camera driver must be installed.
  *
  *    \sa    AddInDataIO, Dummy1394
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class CMU1394 : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~CMU1394();
        //! Constructor
        CMU1394();

    public:
        friend class CMU1394Interface;
        const ito::RetVal showConfDialog(void);    /*!< Open the config nonmodal dialog to set camera parameters */

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);    /*! <Wait for acquired picture */
//        ito::RetVal checkData(void);    /*!< Check if object has to be reallocated */

    private:
        ito::RetVal copyObjBytesSwapped(ito::DataObject *extDObj, uchar *inpBuffer, int sizeX, int sizeY);
        ito::RetVal copyObjBytesSwapped(ito::DataObject *extDObj, uchar *inpBuffer, int sizeX, int sizeY, int maxSizeX, int x0, int y0);

        BOOL m_saveParamsOnClose; /*!< Check if the parameters should be saved on close */
        BOOL m_isgrabbing; /*!< Check if acquire was called */

        int m_iCamNumber;
        int m_iFireWire_VideoMode;
        int m_iFireWire_VideoRate;
        int m_iFireWire_VideoFormat;
        int m_swapBO;

        C1394Camera *m_ptheCamera;
        C1394CameraControl *m_pC1394gain, *m_pC1394offset, *m_pC1394autoexp;
        C1394CameraControlTrigger *m_pC1394trigger;

    signals:
//        void parametersChanged(QMap<QString, ito::Param> params);    /*! Signal send changed or all parameters to listeners */

    public slots:

        //! returns parameter of m_params with key name.
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        //! sets parameter of m_params with key name.
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        //! Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //! Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //! Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //! Stop the camera to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //! Softwaretrigger for the camera
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //! Calls retrieveData(NULL), than copy the picture to dObj of right type and size
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        //! Deep copy the camera buffer to dObj. Object must be of right size and type. If liveData is running, a second deep-copy is performed to copy data to the grabber
        ito::RetVal copyVal(void *dObj, ItomSharedSemaphore *waitCond);

        //! Retrieve new offset and new gain and give them to the camera dll
        void updateParameters(QMap<QString, ito::ParamBase> params);

        //! Slot to synchronize this plugin with dockingwidget
        void GainPropertiesChanged(double gain);

        //! Slot to synchronize this plugin with dockingwidget
        void OffsetPropertiesChanged(double offset);

    private slots:

};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    CMU1394Interface
  *
  *\brief    Interface-Class for CMU1394-Class
  *
  *    \sa    AddInDataIO, CMU1394
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class CMU1394Interface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        CMU1394Interface();
        ~CMU1394Interface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
        //! auto-increment, static instance counter for all dummy-1394 instances


    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // CMU1394_H
