/* ********************************************************************
Plugin "DslrRemote2" for itom software
URL: http://lccv.ufal.br/
Copyright (C) 2017, Universidade Federal de Alagoas (UFAL), Brazil

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

#ifndef DSLRREMOTE_H
#define DSLRREMOTE_H

#include "common/addInGrabber.h"
#include "common/typeDefs.h"
#include "ptpCam.h"
//#include "dialogDslrRemote.h"

#include <qsharedpointer.h>

// forward declaration, interal class
class PtpCam;

//----------------------------------------------------------------------------------------------------------------------------------
class DslrRemoteInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this DummyGrabberInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        DslrRemoteInterface();                    /*!< Constructor */
        ~DslrRemoteInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of DummyGrabber and returns this instance */

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of DummyGrabber, given by *addInInst */

};

//-------------------------------------------------------------------------------------------------------------------------------
class DslrRemote : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~DslrRemote();
        DslrRemote();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class DslrRemoteInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_isgrabbing;
        int64 m_startOfLastAcquisition;
        int m_waittime;
        PtpCam *m_ptp_cam;
        PTP_USB *m_ptp_usb;
        PTPParams *m_ptp_params;
        struct libusb_device *m_ptp_dev;
        int m_ptp_portnum;
        int m_lastImgNum;

        ito::RetVal getFileFromCam(ito::DataObject *data, const uint32_t fhandle, const uint32_t ftype);

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

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > /*paramsOut*/, ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);

};

//-------------------------------------------------------------------------------------------------------------------------------
#endif // DSLRREMOTE_H
