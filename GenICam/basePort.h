/* ********************************************************************
    Plugin "GenICam" for itom software
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

#ifndef BASEPORT_H
#define BASEPORT_H

#define NOMINMAX

#include <qlibrary.h>
#include <qstring.h>
#include <qpointer.h>
#include <qsharedpointer.h>
#include <qqueue.h>
#include <qset.h>
#include <qvector.h>
#include <qmap.h>
#include <qtimer.h>
#include <qhash.h>

#include "common/param.h"
#include "common/retVal.h"
#include "datatypes.h"


#include "GenApi/GenApi.h"

#include "GenTL_v1_5.h"
#include "base/GCString.h"
#define PFNC_INCLUDE_HELPERS
#include "PFNC.h"

using namespace GENAPI_NAMESPACE;
using namespace GENICAM_NAMESPACE;

////////////////////////////////////////////////////////////////////////////////////////////
/*
*/
class BasePort : public IPort
{
public:
    enum PortType { TypeCamera, TypeFramegrabber };

    BasePort(QSharedPointer<QLibrary> lib, PortType deviceType, int verbose, ito::RetVal &retval);
    ~BasePort();

    virtual EAccessMode GetAccessMode() const; //overloaded from IPort: if the driver is open, return RW (= read/write), otherwise NA (= not available)
    virtual void Read(void *pBuffer, int64_t Address, int64_t Length); //overloded from IPort
    virtual void Write(const void *pBuffer, int64_t Address, int64_t Length); //overloded from IPort

    ito::RetVal connectToGenApi(ito::uint32 portIndex);

    bool isDeviceParam(const ParamMapIterator &it) const; //returns true if p is managed by device and therefore mapped to INode, else false
    ito::RetVal setDeviceParam(QSharedPointer<ito::ParamBase> newVal, ParamMapIterator it);

    ito::RetVal createParamsFromDevice(QMap<QString, ito::Param> &params, int visibilityLevel = GenApi::Guru);

    QList<gcstring> getCommandNames() const;

    inline CNodeMapRef device() const { return m_device; }

    void setParamsLocked(bool locked);

    virtual void resyncAllParameters() = 0;

    ito::RetVal invokeCommandNode(const gcstring &name, ito::tRetValue errorLevel = ito::retError);

    //call this to update the m_params["sizex"], ["sizey"] and ["bpp"]
    ito::RetVal syncImageParameters(QMap<QString, ito::Param> &params, QSharedPointer<BasePort> fallbackDevice = nullptr); 

    QVector<PfncFormat> supportedImageFormats(QVector<int> *bitdepths = NULL, QStringList *formatNames = NULL, QVector<int> *colortypes = NULL);

    void setCallbackParameterChangedReceiver(QObject* receiver);
    virtual void callbackParameterChanged_(INode *pNode) = 0; //this is the member, called from the static version callbackParameterChanged (this is necessary if more than one GenICam device is connected to the computer)

protected:
    ito::RetVal printPortInfo(ito::uint32 index) const;
    QByteArray getPortInfoString(GenTL::PORT_INFO_CMD_LIST cmd, ito::RetVal &retval) const;

    ito::RetVal createIntParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
    ito::RetVal createFloatParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
    ito::RetVal createStringParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
    ito::RetVal createBoolParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
    ito::RetVal createEnumParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);

    void intMetaFromInteger(const CIntegerPtr &iPtr, ito::IntMeta *intMeta) const;

    static void callbackParameterChanged(INode *pNode);
    QObject *m_pCallbackParameterChangedReceiver;

    QSharedPointer<QLibrary> m_lib;

    GenTL::PDevClose DevClose;
    GenTL::PDevGetNumDataStreams DevGetNumDataStreams;
    GenTL::PDevGetDataStreamID DevGetDataStreamID;
    GenTL::PDevOpenDataStream DevOpenDataStream;
    GenTL::PDevGetPort DevGetPort;
    GenTL::PGCReadPort GCReadPort;
    GenTL::PGCWritePort GCWritePort;
    GenTL::PGCGetNumPortURLs GCGetNumPortURLs;
    GenTL::PGCGetPortURLInfo GCGetPortURLInfo;
    GenTL::PGCGetPortInfo GCGetPortInfo;

    GenTL::PORT_HANDLE m_portHandle; //for camera, this is the port obtained in the constructor from the deviceHandle; for framegrabbers this is the framegrabber itself

    bool m_genApiConnected;
    CNodeMapRef m_device;
    int m_verbose;
    QByteArray m_deviceName;
    QByteArray m_paramPrefix; //for camera: empty string, for framegrabber: "Framegrabber."
    PortType m_portType;

    QMap<gcstring, CCommandPtr> m_commandNodes;

    QHash<QString, GCType*> m_paramMapping; //the first parameter is the iterator to the corresponding ito::Param*, GCType* is owned by this map and has to be deleted at destroy-time
    QHash<INode*, GCType*> m_paramMapping2; //the first parameter is the INode* pointer (for speed reasons there are two look-up tables), the second is the GCType*. It must not be destroyed since this is done via m_paramMapping.

    QVector<PfncFormat> m_supportedFormats;
    QStringList m_supportedFormatsNames;
    QVector<int> m_supportedFormatsBpp; //bitdepths that correspond to m_supportedFormats
    QVector<int> m_supportedFormatsColor; //0 or 1 if dataObject should be a monochrome object (uint8, uint16...) or color (1, rgba32)

    static QHash<INode*, BasePort*> nodeDeviceHash;
};

#endif // BASEPORT_H