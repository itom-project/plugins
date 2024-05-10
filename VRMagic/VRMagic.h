/* ********************************************************************
    Plugin "VRMagic" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2016, Institut für Technische Optik, Universität Stuttgart

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

#ifndef VRMAGIC_H
#define VRMAGIC_H

#include "common/addInGrabber.h"
#include "dialogVRMagic.h"
#include <qsharedpointer.h>

#include "vrmusbcam2.h"
#include "vrmusbcam2props.h"

//----------------------------------------------------------------------------------------------------------------------------------
class VRMagic : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~VRMagic();
        //! Constructor
        VRMagic();
        void dockWidgetVisibilityChanged(bool visible);

    public:
        friend class VRMagicInterface;
        const ito::RetVal showConfDialog(void);    /*!< Open the config nonmodal dialog to set camera parameters */
        int hasConfDialog(void) { return 0; }; //!< indicates that this plugin has got a configuration dialog

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);    /*! <Wait for acquired picture */

        ito::RetVal setVRMagicParam(const char *paramName, int newValue);

    private:
        enum SyncParams {
            sSignalSource = 0x0001,
            sContrast = 0x0002,
            sSaturation = 0x0004,
            sHue = 0x0008,
            sBrightness = 0x0010,
            sAll = sSignalSource | sContrast | sSaturation | sHue | sBrightness
        };


        ito::RetVal synchronizeCameraSettings(int what = sAll);
        ito::RetVal readCameraIntParam(const char *ximeaParamName, const QString &paramName, bool mandatory = false);
        ito::RetVal readCameraFloatParam(const char *ximeaParamName, const QString &paramName, bool mandatory = false);

        ito::RetVal checkError(const VRmRetVal &error, const char* command = NULL);

        ito::RetVal m_acqRetVal;
        VRmUsbCamDevice m_handle;
        VRmDWORD m_port;
        VRmDeviceKey* m_device_key;
        bool m_imagesAreInterlaced;

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

    private slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
class VRMagicInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)

    PLUGIN_ITOM_API

    protected:

    public:
        VRMagicInterface();
        ~VRMagicInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // VRMAGIC_H
