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

#include <opengps/cxx/info.hxx>
#include <opengps/cxx/string.hxx>
#include <iostream>
#include "stdafx.hxx"

Info::Info()
{
}

Info::~Info()
{
}

void Info::GetCopyright(OpenGPS::String* const text)
{
   _ASSERT(text);

   *text =
      _T("Copyright by\n")
      _T("Johannes Herwig (Uni Duisburg-Essen) 2007\n")
      _T("Georg Wiora (NanoFocus AG) 2007\n")
      _T("Joerg Seewig (Uni Hannover) 2007\n")
      _T("Andreas Walther (NanoFocus AG) 2007\n")
      _T("Mark A. Weber (NanoFocus AG) 2007\n")
      _T("\n")
      _T("\n")
      _T("\n")
      _T("ADDITIONAL CONTRIBUTIONS are copyrighted by\n")
      _T("\n")
      _T("Ian Martins (ianxm@jhu.edu)\n")
      _T("Jean-loup Gailly and Mark Adler 1995-2004\n")
      _T("Christophe Devine 2006-2007\n")
      _T("This product includes software developed by\n")
      _T(" The Apache Software Foundation (http://www.apache.org/).\n\n")
      _T(" Portions of this software were originally based on the following:\n")
      _T("  - software copyright (c) 1999, IBM Corporation., http://www.ibm.com.\n");
}

void Info::GetLicense(OpenGPS::String* const text)
{
   _ASSERT(text);

   *text =
      _T("Licence Declaration for the openGPS(R)[TM] software library\n")
      _T("This program is free software; you can redistribute it and/or modify\n")
      _T("it under the terms of the GNU Lesser General Public License (LGPL)\n")
      _T("as published by the Free Software Foundation; either version 3 of\n")
      _T("the License, or (at your option) any later version.\n")
      _T("for detail see the files \"licence_LGPL-3.0.txt\" and\n")
      _T("\"licence_GPL-3.0.txt\".\n")
      _T("\n")
      _T("openGPS is distributed in the hope that it will be useful,\n")
      _T("but WITHOUT ANY WARRANTY; without even the implied warranty of\n")
      _T("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the\n")
      _T("GNU Lesser General Public License for more details.\n")
      _T("\n")
      _T("You should have received a copy of the GNU General Public License\n")
      _T("along with this program.If not, see <http://www.gnu.org/licenses/>.\n")
      _T("\n")
      _T("Further information on the openGPS project can be found at\n")
      _T("\n")
      _T("  http://www.opengps.eu/\n")
      _T("\n")
      _T("TRADEMARKS\n")
      _T("\n")
      _T("The name \"openGPS\" and the logo are registered as European\n")
      _T("trade mark No. 006178354 for\n")
      _T("\n")
      _T("  Physikalisch Technische Bundesanstalt (PTB)\n")
      _T("  Bundesallee 100\n")
      _T("  38116 Braunschweig\n")
      _T("  Germany\n")
      _T("\n")
      _T("  http://www.ptb.de/\n")
      _T("\n")
      _T("\n")
      _T("\n")
      _T("ADDITIONAL CONTRIBUTIONS\n")
      _T("\n")
      _T("The documentation is created using doxygen. To produce propper output\n")
      _T("from MATLAB code the filter \"mtoc\" from Ian Martins (ianxm@jhu.edu) is\n")
      _T("used. mtoc has been put under the GPL by Ian Martins.\n")
      _T("\n")
      _T("The project uses \"ZLIB DATA COMPRESSION LIBRARY\" V1.2.3\n")
      _T("(C) 1995-2004 Jean-loup Gailly and Mark Adler\n")
      _T("\n")
      _T("The project uses the md5 algorithms from the \"XYSSL Library\" V0.8\n")
      _T("Copyright (C) 2006-2007  Christophe Devine\n")
      _T("\n")
      _T("The project uses the xsd-runtime library (C) 2005-2008 CODE SYNTHESIS\n")
      _T("TOOLS CC as well as code generated by the Code Synthesis xsd-tool\n")
      _T("which is both licensed under GPL. Due to the FLOSS exception defined\n")
      _T("by Code Synthesis, openGPS in combination with the xsd-runtime-library\n")
      _T("can be distributed under LGPL and can be used in proprietary projects.\n\n")
      _T("The project uses the Xerces xml-parsing library from the Apache Xerces\n")
      _T("distribution: It includes software developed by\n")
      _T("  The Apache Software Foundation (http://www.apache.org/).\n")
      _T("  Portions of this software were originally based on the following:\n")
      _T("    - software copyright (c) 1999, IBM Corporation., http://www.ibm.com.\n");

      ;
}

void Info::PrintCopyright()
{
   OpenGPS::String text;
   GetCopyright(&text);

   std::cout << text.ToChar() << std::endl;
}

void Info::PrintLicense()
{
   OpenGPS::String text;
   GetLicense(&text);

   std::cout << text.ToChar() << std::endl;
}

void Info::GetName(OpenGPS::String* const text)
{
   _ASSERT(text);

   *text = _OPENGPS_NAME;
}

void Info::GetAbout(OpenGPS::String* const text)
{
   _ASSERT(text);

   *text = _OPENGPS_DESCRIPTION;
}

void Info::GetVersion(OpenGPS::String* const text)
{
   _ASSERT(text);

   *text = _OPENGPS_VERSIONSTRING;
}

void Info::PrintVersion()
{
   OpenGPS::String version(_OPENGPS_ID);

   std::cout << version.ToChar() << std::endl;
}
