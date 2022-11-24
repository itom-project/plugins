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

#include "framegrabber.h"

#include "gccommon.h"

#include <qfileinfo.h>
#include <qdebug.h>
#include <qset.h>
#include <qurl.h>
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include <iostream>

#include <Base/GCBase.h>
#include <GenApi/GenApi.h>


using namespace GENAPI_NAMESPACE;
using namespace GENICAM_NAMESPACE;

//----------------------------------------------------------------------------------------------------------------------------------
GenTLFramegrabber::GenTLFramegrabber(QSharedPointer<QLibrary> lib, GenTL::DEV_HANDLE framegrabberHandle, int verbose, ito::RetVal &retval) :
    BasePort(lib, BasePort::TypeFramegrabber, verbose, retval),
    m_framegrabberHandle(framegrabberHandle)
{
    m_portHandle = framegrabberHandle;

    if (!retval.containsError())
    {
        ito::uint32 numURLS;
        retval += checkGCError(GCGetNumPortURLs(m_framegrabberHandle, &numURLS), "Framegrabber: GCGetNumPortURLs");

        if (m_verbose >= VERBOSE_INFO)
        {
            std::cout << "Available locations for XML parameter description in framegrabber\n----------------------------------------\n";

            for (ito::uint32 i = 0; i < numURLS; ++i)
            {
                retval += printPortInfo(i);
            }

            std::cout << "\n" << std::endl;
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
GenTLFramegrabber::~GenTLFramegrabber()
{
}

//--------------------------------------------------------------------------------------------------------
/*virtual*/ void GenTLFramegrabber::callbackParameterChanged_(INode *pNode)
{
    CValuePtr ptrValue = pNode;

    if (m_verbose >= VERBOSE_DEBUG)
    {
        try
        {
            std::cout << "Framegrabber: The node '" << pNode->GetName() << "' changed (" << pNode->GetAccessMode() << ")\n";
            qDebug() << "Framegrabber: The node '" << pNode->GetName() << "' changed (" << pNode->GetAccessMode() << ")";

            if (ptrValue.IsValid())
            {
                if (pNode->GetAccessMode() & (RO | RW))
                {
                    std::cout << "New value = " << ptrValue->ToString() << ", access: " << ptrValue->GetAccessMode() << "\n" << std::endl;
                }
                else
                {
                    std::cout << "New value not readable. No read access. Access: " << ptrValue->GetAccessMode() << "\n" << std::endl;
                }
            }
        }
        catch (GenericException ex)
        {
            qDebug() << ex.GetDescription() << ex.what();
        }
    }

    if (m_callbackParameterChangedTimer.isNull())
    {
        m_callbackParameterChangedTimer = QSharedPointer<QTimer>(new QTimer());
        m_callbackParameterChangedTimer->setSingleShot(true);
        m_callbackParameterChangedTimer->setInterval(50);
        m_callbackParameterChangedTimer->stop();
        QObject::connect(m_callbackParameterChangedTimer.data(), SIGNAL(timeout()), m_pCallbackParameterChangedReceiver, SLOT(parameterChangedTimerFiredFramegrabber()));
    }

    if (pNode && m_paramMapping2.contains(pNode))
    {
        m_paramMapping2[pNode]->update(false);
        m_callbackParameterChangedTimer->start();
    }
}

//--------------------------------------------------------------------------------------------------------
void GenTLFramegrabber::resyncAllParameters()
{
    QHash<INode*, GCType*>::const_iterator i = m_paramMapping2.constBegin();
    while (i != m_paramMapping2.constEnd()) 
    {
        try
        {
            i.value()->update(false);
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Error updating parameter '" << i.key()->GetName() << "': " << ex.GetDescription() << "\n" << std::endl;
        }
        ++i;
    }

    if (m_pCallbackParameterChangedReceiver)
    {
        QMetaObject::invokeMethod(m_pCallbackParameterChangedReceiver, "parameterChangedTimerFiredFramegrabber");
    }
}


//----------------------------------------------------------------------------------
ito::RetVal GenTLFramegrabber::special(int num)
{
    if (num == 1)
    {
        try
        {
            CIntegerPtr CIncomingWidthHost = m_device._GetNode("IncomingWidth");
            *CIncomingWidthHost = 5120;
            std::cout << "Special 1: IncomingWidth: " << *CIncomingWidthHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }
        try
        {
            CIntegerPtr CIncomingHeightHost = m_device._GetNode("IncomingHeight");
            *CIncomingHeightHost       = 5120;
            std::cout << "Special 1: IncomingHeight: " << *CIncomingHeightHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }
        try
        {
            CIntegerPtr CWidthHost = m_device._GetNode("Width");
            *CWidthHost = 5120;
            std::cout << "Special 1: Width: " << *CWidthHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }
        try
        {
            CIntegerPtr CHeightHost = m_device._GetNode("Height");
            *CHeightHost = 5120;
            std::cout << "Special 1: Height: " << *CHeightHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }       
    }
    else if (num == 2)
    {
        try
        {
            unsigned int value = 5120;
            Write(&value, 0x0000600 + 0x0014, 4);
            CIntegerPtr CIncomingWidthHost = m_device._GetNode("IncomingWidth");
            std::cout << "Special 2: IncomingWidth: " << *CIncomingWidthHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }
        try
        {
            unsigned int value = 5120;
            Write(&value, 0x0000600 + 0x0018, 4);
            CIntegerPtr CIncomingHeightHost = m_device._GetNode("IncomingHeight");
            std::cout << "Special 2: IncomingHeight: " << *CIncomingHeightHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }
        try
        {
            unsigned int value = 5120;
            Write(&value, 0x0000600 + 0x0004, 4);
            CIntegerPtr CWidthHost = m_device._GetNode("Width");
            std::cout << "Special 2: WidthHost: " << *CWidthHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }
        try
        {
            unsigned int value = 5120;
            Write(&value, 0x0000600 + 0x000c, 4);
            CIntegerPtr CHeightHost = m_device._GetNode("Height");
            std::cout << "Special 2: HeightHost: " << *CHeightHost() << "\n" << std::endl;
        }
        catch (GenericException& ex)
        {
            std::cout << "Framegrabber: Special 1 error " << ex.GetDescription() << "\n" << std::endl;
        }       
    }

    return ito::retOk;
}
