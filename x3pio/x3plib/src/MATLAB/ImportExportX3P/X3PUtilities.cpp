/***************************************************************************
 *   Copyright by Georg Wiora (NanoFocus AG) 2008                          *
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

#include "X3PUtilities.h"

#include <strstream>
#include <iomanip>
#include <ios>
#include <opengps/cxx/info.hxx>

#include <time.h>

using namespace std;
using namespace OpenGPS::Schemas::ISO5436_2;

// Helper functions for X3P import export in MATLAB
// Convert wstring to matlab string
mxArray *
ConvertWtoMStr(const std::wstring &inp)
{
  size_t len = inp.length();
  // String dimensions
  mwSize dims[2]={1,len};
  // Create a matlab string with length of input string
  mxArray *dest = mxCreateCharArray(2, dims);
  // Copy characters to string
  wchar_t *dptr = (wchar_t*)mxGetData(dest);
  for (unsigned int i=0; i<len ; i++)
    *(dptr++) = inp[i];

  // Return pointer to mxArray
  return dest;
}

// Convert matlab string to wstring
std::wstring 
ConvertMtoWStr(const mxArray *inp)
{
  // Destination string
  std::wstring dest;
  
  // Get len from
  mwSize len = mxGetNumberOfElements(inp);
  // Copy characters to string
  wchar_t *sptr = (wchar_t*)mxGetData(inp);
  // copy all characters to destination string
  for (int i=0; i<len ; i++)
    dest.push_back(*(sptr++));

  // Return reference to wstring
  return dest;
}


// Get point info structure returned by openX3P and writeX3P
mxArray *
GetPointInfoStructure(OGPS_ISO5436_2Handle handle)
{
  // Result
  mxArray *res=NULL;

  // Get document 
  const ISO5436_2Type* const document = ogps_GetDocument(handle);
  if (ogps_HasError())
  {
    mexErrMsgIdAndTxt("openGPS:X3P:XMLDocument",
            "Could not access XML document in current x3p file!");
    return NULL;
  }
  // Get a reference to record 1
  const OpenGPS::Schemas::ISO5436_2::ISO5436_2Type::Record1_type &r1 = document->Record1();

  // Create point info structure
  // Number of structure elements
  const unsigned int nelem=8;
  // Field names
  const char *fieldnames[nelem] = {"Revision","FeatureType","IsMatrix","IsList",
                                 "isXIncremental","isYIncremental","isZIncremental","DLL_Version"};
  // Create the structure
  res = mxCreateStructMatrix(1, 1, nelem, &(fieldnames[0]));

  // Get file format revision
  mxSetField(res, 0, "Revision",  ConvertWtoMStr(r1.Revision()));

  // Get Feature type "PRF", "SUR", "PCL"
  mxSetField(res, 0, "FeatureType",  ConvertWtoMStr(r1.FeatureType()));

  // Check for matrix organisation
  mxSetField(res, 0, "IsMatrix",  mxCreateLogicalScalar(ogps_IsMatrix(handle)));
  // Check for list organisation
  mxSetField(res, 0, "IsList",  mxCreateLogicalScalar(ogps_IsMatrix(handle) ? false:true));
  // Check for axis types
  mxSetField(res, 0, "isXIncremental",  mxCreateLogicalScalar(
          r1.Axes().CX().AxisType() == Record1Type::Axes_type::CX_type::AxisType_type::I 
          ? true : false));
  mxSetField(res, 0, "isYIncremental",  mxCreateLogicalScalar(
          r1.Axes().CY().AxisType() == Record1Type::Axes_type::CY_type::AxisType_type::I 
          ? true : false));
  mxSetField(res, 0, "isZIncremental",  mxCreateLogicalScalar(
          r1.Axes().CZ().AxisType() == Record1Type::Axes_type::CZ_type::AxisType_type::I 
          ? true : false));    

  // Set dll version
  mxSetField(res, 0, "DLL_Version",  ConvertWtoMStr(GetX3P_Dll_ID()));
  // Return pointer to structure
  return res;
}


// Get the name and version from X3P dll
std::wstring 
GetX3P_Dll_ID(void)
{  
  // Get dll name
  OpenGPS::String Name;
  OpenGPS::Info::GetName(&Name);
  // Get Version info
  OpenGPS::String Version;
  OpenGPS::Info::GetVersion(&Version);

  // Create output string
  std::wstring msg(Name);
  msg.append(L" V");
  msg.append(Version);

  // Return result string
  return msg;
}


/*!
  @brief Helper function to return the current time properly formated.
  
  @return A string containing the time stamp from now.

  @note There is only a windows implementation yet. In other cases
  return a dummy. That is enough for testing purposes.
*/
#ifdef _WIN32
OpenGPS::String TimeStamp(void)
{
  time_t ltime;
  struct tm lt;
  // Time zone offset
  long tzoff;
  // Set timezone
  _tzset();
  // Get time zone offset
  _get_timezone(&tzoff);
  // Offset ours and minutes
  int tzoff_h,tzoff_m;
  tzoff_h = -(int)floor(((double)tzoff)/3600.);
  tzoff_m = -(int)floor(((double)tzoff)/60. + 0.5) - tzoff_h*60;

  // Get current time
  time( &ltime );
  // get local time
  localtime_s(&lt,&ltime);
  
  // Correct tz offset by dst
  if (lt.tm_isdst > 0)
    tzoff_h++;

  // Absolute offset for printing
  int tzoff_habs = abs(tzoff_h);
  OGPS_Character tzoffsign = tzoff_h<0 ? _T('-') : _T('+');

  // Create a string of pattern "2007-04-30T13:58:02.6+02:00"
  wostringstream sout;
  sout << std::setfill(_T('0')) << setw(4) << (lt.tm_year+1900) << _T('-') << setw(2) << lt.tm_mon << _T('-') << setw(2) << lt.tm_mday 
      << _T('T') << setw(2) << lt.tm_hour << _T(':') << setw(2) << lt.tm_min << _T(':') << setw(2) << lt.tm_sec << _T(".0")
      << tzoffsign << setw(2) << tzoff_habs << _T(':') << setw(2) << tzoff_m << ends;

  return sout.str();
}
#else
// There is only a windows implementation yet.
//In other cases return a dummy. That is enough for testing purposes.
OGPS_String TimeStamp(void)
{
  return _T("2000-01-01T00:00:00.0+00:00");
}

#endif
