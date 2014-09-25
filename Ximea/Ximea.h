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

#ifndef XIMEA_H
#define XIMEA_H

#include "common/addInGrabber.h"
#include "dialogXimea.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    Ximea
  *\brief    class to use a Ximea camera as an ITOM-Addin. Child of AddIn - Library (DLL) - Interface
  *
  *         This class can be used to work with a Ximea USB3 camera. It grabbes datas with 8 or ?? Bit.
  *            The "m3api.dll" has to be in a subfolder .\Ximea in the plugin directory
  *
  *    \sa    AddInDataIO, DummyGrabber
  *    \date    04.07.2012
  *    \author  CK, Ly
  * \warning    NA
  *
  */
class Ximea : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~Ximea();
        //! Constructor
        Ximea();

    public:
        friend class XimeaInterface;
        const ito::RetVal showConfDialog(void);    /*!< Open the config nonmodal dialog to set camera parameters */
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);    /*! <Wait for acquired picture */
//        ito::RetVal checkData(void);    /*!< Check if objekt has to be reallocated */

        ito::RetVal setXimeaParam(const char *paramName, int newValue);

    private:

        enum grabState
        {
            grabberStopped = 0x00,
            grabberRunning = 0x01,
            grabberGrabbed = 0x02,
            grabberGrabError = 0x04
        };

        ito::RetVal LoadLib();
        ito::RetVal getErrStr(const int error);
        int m_numDevices;
        int m_device;
        int m_saveParamsOnClose;
#if linux
        void *m_handle;
#else
        HANDLE m_handle;
#endif

        void* m_pvShadingSettings;

        int m_isgrabbing;
        ito::RetVal m_acqRetVal;
    signals:
        //void parametersChanged(QMap<QString, ito::Param> params);    /*! Signal send changed or all parameters to listeners */

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
        //! Calls retrieveData(), than shallow copy the picture to dObj of right type and size
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        //! Calls retrieveData(vpdObj), than deep copy the picture to dObj of right type and size
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        void updateParameters(QMap<QString, ito::Param> params);

        //! Slot to synchronize this plugin with dockingwidget
        void GainPropertiesChanged(double gain);

        //! Slot to synchronize this plugin with dockingwidget
        void OffsetPropertiesChanged(double offset);

        //! Slot to synchronize this plugin with dockingwidget
        void IntegrationPropertiesChanged(double integrationtime);

        //! Slot to run special function
        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond);

    private slots:

};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    XimeaInterface
  *
  *\brief    Interface-Class for Ximea-Class
  *
  *    \sa    AddInDataIO, Ximea
  *    \date    04.07.2012
  *    \author  CK, Ly
  * \warning    NA
  *
  */
class XimeaInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        XimeaInterface();
        ~XimeaInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);


    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // Ximea_H
