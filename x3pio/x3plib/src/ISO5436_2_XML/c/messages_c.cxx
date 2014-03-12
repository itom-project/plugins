/***************************************************************************
 *   Copyright by Johannes Herwig (NanoFocus AG) 2007                      *
 *   Copyright by Georg Wiora (NanoFocus AG) 2007                          *
 *                                                                         *
 *   This file is part of the openGPS (R)[TM] software library.            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License (LGPL)    *
 *   as published by the Free Software Foundation; either version 3 of     *
 *   the License, or (at your option) any later version.                   *
 *   for detail see the files "licence_LGPL-3.0.txt" and                   *
 *   "licence_GPL-3.0.txt".                                                *
 *                                                                         *
 *   openGPS is distributed in the hope that it will be useful,            *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *                                                                         *
 *   The name "openGPS" and the logo are registered as                     *
 *   European trade mark No. 006178354 for                                 *
 *   Physikalisch Technische Bundesanstalt (PTB)                           *
 *   http://www.ptb.de/                                                    *
 *                                                                         *
 *   More information about openGPS can be found at                        *
 *   http://www.opengps.eu/                                                *
 ***************************************************************************/

#include <opengps/messages.h>
#include "messages_c.hxx"
#include "../cxx/stdafx.hxx"

#ifdef _DEBUG
#  include <iostream>
#endif

const OGPS_Character* ogps_GetErrorMessage()
{
   return OpenGPS::ExceptionHistory::GetLastErrorMessage();
}

const OGPS_Character* ogps_GetErrorDescription()
{
   return OpenGPS::ExceptionHistory::GetLastErrorDescription();
}

OGPS_ExceptionId ogps_GetErrorId()
{
   return OpenGPS::ExceptionHistory::GetLastErrorId();
}

OGPS_Boolean ogps_HasError()
{
   return OpenGPS::ExceptionHistory::GetLastErrorId() != OGPS_ExNone;
}

void OpenGPS::ExceptionHistory::SetLastException(const OpenGPS::Exception& ex)
{
   OpenGPS::String msg;
   msg.FromChar(ex.what());

   m_LastErrorMessage = msg;
   m_LastErrorDescription = ex.details();
   m_LastErrorId = ex.id();
   m_LastErrorSource = ex.method();

   DumpIt();
}

void OpenGPS::ExceptionHistory::SetLastException(const std::exception& ex)
{
   OpenGPS::String msg;
   msg.FromChar(ex.what());

   m_LastErrorMessage = msg;
   m_LastErrorDescription.clear();
   m_LastErrorId = OGPS_ExGeneral;
   m_LastErrorSource.clear();

   DumpIt();
}

void OpenGPS::ExceptionHistory::SetLastException()
{
   /* An exception occured, but no further details are known. */
   m_LastErrorMessage.clear();
   m_LastErrorDescription.clear();
   m_LastErrorSource.clear();

   /* At least there must have been an exception... */
   m_LastErrorId = OGPS_ExGeneral;

   DumpIt();
}

void OpenGPS::ExceptionHistory::DumpIt()
{
#ifdef _DEBUG
   OpenGPS::String title(_T("Exception:")), brief(_T("Brief: ")), detail(_T("Details: ")), method(_T("In method: "));
   std::cerr << std::endl << title.ToChar() << std::endl <<
      brief.ToChar() << m_LastErrorMessage.ToChar() << std::endl <<
      detail.ToChar() << m_LastErrorDescription.ToChar() << std::endl <<
      method.ToChar() << m_LastErrorSource.ToChar() << std::endl << std::endl;
#endif
}

void OpenGPS::ExceptionHistory::Reset()
{
   m_LastErrorMessage.clear();
   m_LastErrorDescription.clear();
   m_LastErrorSource.clear();
   m_LastErrorId = OGPS_ExNone;
}

const OGPS_Character* OpenGPS::ExceptionHistory::GetLastErrorMessage()
{
   return m_LastErrorMessage.empty() ? NULL : m_LastErrorMessage.c_str();
}

const OGPS_Character* OpenGPS::ExceptionHistory::GetLastErrorDescription()
{
   return m_LastErrorDescription.empty() ? NULL : m_LastErrorDescription.c_str();
}

OGPS_ExceptionId OpenGPS::ExceptionHistory::GetLastErrorId()
{
   return m_LastErrorId;
}

OpenGPS::ExceptionHistory::ExceptionHistory()
{
}

OpenGPS::ExceptionHistory::~ExceptionHistory()
{
}

OpenGPS::String OpenGPS::ExceptionHistory::m_LastErrorMessage;
OpenGPS::String OpenGPS::ExceptionHistory::m_LastErrorDescription;
OpenGPS::String OpenGPS::ExceptionHistory::m_LastErrorSource;
OGPS_ExceptionId OpenGPS::ExceptionHistory::m_LastErrorId = OGPS_ExNone;
