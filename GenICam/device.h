/* ********************************************************************
    Plugin "GenICam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2016, Institut für Technische Optik (ITO),
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

#ifndef DEVICE_H
#define DEVICE_H

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
#include "dataStream.h"
#include "datatypes.h"

#include "GenApi/GenApi.h"

#include "GenTL_v1_5.h"

#define PFNC_INCLUDE_HELPERS
#include "PFNC.h"

using namespace GENAPI_NAMESPACE;

////////////////////////////////////////////////////////////////////////////////////////////
/*
*/
class GenTLDevice : public IPort
{
public:
	GenTLDevice(QSharedPointer<QLibrary> lib, GenTL::DEV_HANDLE devHandle, QByteArray deviceID, const QByteArray &identifier, ito::RetVal &retval);
    ~GenTLDevice();

	virtual EAccessMode GetAccessMode() const; //overloaded from IPort: if the driver is open, return RW (= read/write), otherwise NA (= not available)
	virtual void Read(void *pBuffer, int64_t Address, int64_t Length); //overloded from IPort
	virtual void Write(const void *pBuffer, int64_t Address, int64_t Length); //overloded from IPort

    QByteArray getDeviceID() const { return m_deviceID; }
	ito::RetVal connectToGenApi(ito::uint32 portIndex);
	QSharedPointer<GenTLDataStream> getDataStream(ito::int32 streamIndex, bool printInfoAboutAllStreams, ito::RetVal &retval);

	ito::RetVal syncImageParameters(QMap<QString, ito::Param> &params); //call this to update the m_params["sizex"], ["sizey"] and ["bpp"]
	int getPayloadSize() const;

	QVector<PfncFormat> supportedImageFormats(QVector<int> *bitdepths = NULL, QStringList *formatNames = NULL);

	ito::RetVal createParamsFromDevice(QMap<QString, ito::Param> &params, int visibilityLevel = GenApi::Guru);

	bool isDeviceParam(const ParamMapIterator &it) const; //returns true if p is managed by device and therefore mapped to INode, else false
	ito::RetVal setDeviceParam(QSharedPointer<ito::ParamBase> newVal, ParamMapIterator it);
	ito::RetVal invokeCommandNode(const gcstring &name, ito::tRetValue errorLevel = ito::retError);
	bool autoUpdateDependentNodes(); //check all nodes for a change in their access mode and return true if any node including the corresponding ito::Param has been updated, else false
	QByteArray deviceName() const { return m_deviceName; }

	void setCallbackParameterChangedReceiver(QObject* receiver);
	void callbackParameterChanged_(INode *pNode); //this is the member, called from the static version callbackParameterChanged (this is necessary if more than one GenICam device is connected to the computer)

protected:
	ito::RetVal printPortInfo(ito::uint32 index) const;

	ito::RetVal createIntParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
	ito::RetVal createFloatParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
	ito::RetVal createStringParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
	ito::RetVal createBoolParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
	ito::RetVal createEnumParamFromDevice(GenApi::INode *node, QMap<QString, ito::Param> &params, ito::ByteArray &category);
	int flagsFromAccessMode(const GenApi::EAccessMode &accessMode) const;

	int64_t getIntParam(const gcstring &name, bool *valid = NULL);
	void intMetaFromInteger(const CIntegerPtr &iPtr, ito::IntMeta *intMeta) const;

	static void callbackParameterChanged(INode *pNode);
	QSharedPointer<QTimer> m_callbackParameterChangedTimer;
	QObject *m_pCallbackParameterChangedReceiver;
	static QHash<INode*, GenTLDevice*> nodeDeviceHash;

    GenTL::DEV_HANDLE m_handle;
    GenTL::PORT_HANDLE m_portHandle;

    GenTL::PDevClose DevClose;
    GenTL::PDevGetNumDataStreams DevGetNumDataStreams;
    GenTL::PDevGetDataStreamID DevGetDataStreamID;
    GenTL::PDevOpenDataStream DevOpenDataStream;
    GenTL::PDevGetPort DevGetPort;
    GenTL::PGCReadPort GCReadPort;
	GenTL::PGCWritePort GCWritePort;
    GenTL::PGCGetNumPortURLs GCGetNumPortURLs;
    GenTL::PGCGetPortURLInfo GCGetPortURLInfo;

    QHash<QString, GCType*> m_paramMapping; //the first parameter is the iterator to the corresponding ito::Param*, GCType* is owned by this map and has to be deleted at destroy-time
	QHash<INode*, GCType*> m_paramMapping2; //the first parameter is the INode* pointer (for speed reasons there are two look-up tables), the second is the GCType*. It must not be destroyed since this is done via m_paramMapping.
	QMap<gcstring, CCommandPtr> m_commandNodes;

	QSharedPointer<QLibrary> m_lib;
    QByteArray m_deviceID;
	bool m_genApiConnected;

	QByteArray m_deviceName;
	CNodeMapRef m_camera;
	QVector<PfncFormat> m_supportedFormats;
	QStringList m_supportedFormatsNames;
	QVector<int> m_supportedFormatsBpp; //bitdepths that correspond to m_supportedFormats
};

#endif // DEVICE_H