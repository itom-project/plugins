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

#include "point_vector_parser_builder.hxx"

#include "point_vector_parser.hxx"

#include "stdafx.hxx"

// TODO: use auto_ptr

PointVectorParserBuilder::PointVectorParserBuilder()
{
   m_Parser = NULL;
}

PointVectorParserBuilder::~PointVectorParserBuilder()
{
   _OPENGPS_DELETE(m_Parser);
}

void PointVectorParserBuilder::BuildParser()
{
   _ASSERT(!m_Parser);

   m_Parser = new PointVectorParser();
}

void PointVectorParserBuilder::BuildX(const OGPS_DataPointType dataType)
{
   _ASSERT(m_Parser);

   m_Parser->SetX(m_Parser->CreateDataPointParser(dataType));
}

void PointVectorParserBuilder::BuildY(const OGPS_DataPointType dataType)
{
   _ASSERT(m_Parser);

   m_Parser->SetY(m_Parser->CreateDataPointParser(dataType));
}

void PointVectorParserBuilder::BuildZ(const OGPS_DataPointType dataType)
{
   _ASSERT(m_Parser);

   m_Parser->SetZ(m_Parser->CreateDataPointParser(dataType));
}

PointVectorParser* PointVectorParserBuilder::GetParser()
{
   return m_Parser;
}
