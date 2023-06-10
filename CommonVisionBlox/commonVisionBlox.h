/* ********************************************************************
    Plugin "CommonVisionBlox" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut fuer Technische Optik, Universitaet Stuttgart

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

#ifndef CVB_H
#define CVB_H

#include "common/addInGrabber.h"

#include <iCVCImg.h>
#include <iCVCDriver.h>
#include <iCVCUtilities.h>
#include <iCVGenApi.h>
#include <CVCError.h>

#include <qsharedpointer.h>
#include <QTimerEvent>

//----------------------------------------------------------------------------------------------------------------------------------
class CommonVisionBlox : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~CommonVisionBlox();
        CommonVisionBlox();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);
        void dockWidgetVisibilityChanged(bool visible);

        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class CVBInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 0; }; //!< indicates that this plugin has got a configuration dialog

    private:
        enum sections { integration_time = 0x0001,
                    roi                  = 0x0002,
                    gain                = 0x0032,
                    offset                = 0x0064,
                    all                    = integration_time | roi | gain | offset};

        ito::RetVal checkError(const cvbres_t &code, const char *prefix = NULL) const;

        IMG m_hCamera;
        NODEMAP m_hNodeMap;
        ito::RetVal m_acquisitionRetVal;
        bool m_isGrabbing;

        ito::RetVal synchronize(const sections &what = all);

        ito::RetVal getParamInt(const char *name, cvbint64_t &value);
        ito::RetVal getParamFloat(const char *name, double &value);
        ito::RetVal getParamBool(const char *name, bool &value);
        ito::RetVal getParamString(const char *name, QByteArray &value);

        ito::RetVal getParamFloatInfo(const char *name, ito::DoubleMeta &meta, const double &scale = 1.0);
        ito::RetVal getParamIntInfo(const char *name, ito::IntMeta &meta);

        ito::RetVal setParamInt(const char *name, const cvbint64_t &value);
        ito::RetVal setParamFloat(const char *name, const double &value);
        ito::RetVal setParamBool(const char *name, const bool &value);
        ito::RetVal setParamString(const char *name, const char* value);

        ito::RetVal getParamEnumerationInfo(const char *name, ito::StringMeta &meta);

        bool nodeExists(const char *name) const;
        void checkStatus();

        ito::RetVal scan_for_cameras();

        QMap<QByteArray, QByteArray> m_nameConverter;

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

};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // CVB_H
