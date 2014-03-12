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

// Mex file to write X3P files to matlab
// 07-Dec-2008

// Preconditions:
//   ISO5436_2_XML.dll and lib must be available
//   xerces-c_2_7_vc80.dll and xerces 3.0 headers from CodeSynthesis must be installed.

// Compile from Matlab with 
// mex -L getenv('OPENGPS_LOCATION') -I [getenv(OPENGPS_LOCATION),'/../include/'] writeX3P.cpp
// Replace OPENGPS_LOCATION with your installation location
// mex -L"S:/openGPS/ISO5436_XML/install/lib/" -L"S:/openGPS/ISO5436_XML/install/bin/" -l"ISO5436_2_XML_S" -I"S:/openGPS/ISO5436_XML/install/include/" -I"C:\Programme\CodeSynthesis XSD 3.0\include"  COMPFLAGS="$COMPFLAGS /Zc:wchar_t" writeX3P.cpp
// For debugging use:
// mex -g -L"S:/openGPS/ISO5436_XML/install/lib/" -L"S:/openGPS/ISO5436_XML/install/bin/" -l"ISO5436_2_XML_S" -I"S:/openGPS/ISO5436_XML/install/include/" -I"C:\Programme\CodeSynthesis XSD 3.0\include"  COMPFLAGS="$COMPFLAGS /Zc:wchar_t" writeX3P.cpp

// We want to link to the iso5436_2.dll
#define SHARED_OPENGPS_LIBRARY
#define _UNICODE
#define UNICODE

#include "X3PUtilities.h"
#include <tchar.h>
#include "mex.h"
#include <strstream>
#include <limits>
#include <opengps/cxx/opengps.hxx>
#include <opengps/iso5436_2.h>
#include <opengps/cxx/iso5436_2.hxx>
#include <opengps/cxx/iso5436_2_handle.hxx>
#include <opengps/cxx/iso5436_2_xsd.hxx>
#include <opengps/cxx/point_iterator.hxx>
#include <opengps/cxx/point_vector.hxx>
#include <opengps/cxx/data_point.hxx>
#include <opengps/cxx/string.hxx>
#include <opengps/cxx/info.hxx>

using namespace std;
using namespace OpenGPS::Schemas::ISO5436_2;

// Check for incremental data in axis vector for arbitrary numeric type
// Array must have at least two dimensions
// Returns 0 for non incremental axis
template <class T> double
AxisIncrement(const mxArray *x)
{
  double curr,increment=0;
  // Element pointer
  const T *ptr = (const T*)mxGetPr(x);
  // Check for two dimensions
  mxAssert(mxGetNumberOfElements(x)>1,
          "AxisIncrement Assertion: input array must have at least 2 dimensions!");
  // Number of elements in first two dimensions
  const mwSize *dims = mxGetDimensions(x);
  unsigned long numel = dims[0]*dims[1];
  // number of elements must be > 1
  mxAssert(numel>1, "AxisIncrement Assertion: Number of elements in first two dimensions must be >= 2");

  // estimate increment from first element
  increment = (*(ptr+1)) - (*ptr) ;
  while(numel-- > 0)
  {
    // Get current element
    curr= (*(ptr+1)) - (*ptr);
    // compare current increment to first one
    if (fabs(increment - curr) > std::numeric_limits<T>::epsilon())
      // This difference is too big, this is not an incremental axis
      return 0;
  }
  
  // This is an incremental axis. Return the increment
  return increment;
}

// Check for incremental x or y axis from x or y data vector
// Returns the increment or 0 for non incremental axis
bool
AxisIsIncremental(const mxArray *x, double &increment)
{
  mwSize ndims = mxGetNumberOfDimensions(x);
  const mwSize *dims = mxGetDimensions(x);

  // Preset increment
  increment=0;
  
  // Flag for coord vector
  bool isVector = false;
  
  // Check number of dimensions if only 1 or [1,v] or [u,1] it is a vector
  if ( ndims > 2)
    // This is not incremental
    return false;
  // Check for [u,1,w] or [1,v,w]
  if (ndims == 2)
  {
    if (((dims[0] == 1) && (dims[1] > 1))
      ||((dims[1] == 1) && (dims[0] > 1)))
      // This is a 1d coordinate vector and can be checked to be incremental
      isVector=true;
    else      return false;
  }
  else if (ndims==1)
    // 1d vector
    isVector = true;
  else
    // Scalar or more dimensions
    return false;
  
  // If we found a 1d vector we can check it for constant step size between neighbouring elements:
  mwSize numel = mxGetNumberOfElements(x);
  
  // Get the increment
  // Do it for each type
  switch (mxGetClassID(x))
  {
    case mxDOUBLE_CLASS:
      increment = AxisIncrement<OGPS_Double>(x);
      break;
    case mxSINGLE_CLASS:
      increment = AxisIncrement<OGPS_Float>(x);
      break;
    case mxINT16_CLASS:
      increment = AxisIncrement<OGPS_Int16>(x);
      break;
    case mxINT32_CLASS:
      increment = AxisIncrement<OGPS_Int32>(x);
      break;
    default:
    {
      // Print full description
      ostrstream msg;
      // TODO: Error messages are not correctly printed yet!
      msg << "Error: Input arrays must be Double, Single, Int16 or Int32 type!" 
          << endl << ends;
      mexErrMsgIdAndTxt("openGPS:writeX3P:AxisType",msg.str());
      return false;
    }  
  }
  
  // Return the increment
  return true;
}


// Check meta structure for completeness
bool IsMetaComplete(const mxArray *meta)
{
  // Number of structure elements
  const unsigned int nelem=8;
  // Field names
  const char *fieldnames[nelem] = {"Date",/* Optional: "Creator", */ "Instrument_Manufacturer",
      "Instrument_Model","Instrument_Serial","Instrument_Version",
      "CalibrationDate","ProbingSystem_Type","ProbingSystem_Identification"
      /*, Optional: "Comment" */};
      
  // check all fields for existence
  for (unsigned int i=0; i<nelem; i++)
  {
    if ( NULL == mxGetField(meta, 0, fieldnames[i]))
    {
      // Print full description
      ostrstream msg;
      msg << "Error: Meta data record is incomplete, field \"" 
          << string(fieldnames[i]) << "\" is missing!" << endl << ends;
      mexErrMsgIdAndTxt("openGPS:writeX3P:MetaData",msg.str());
      return false;
    }
  }
  // All elements exist
  return true;
}

// Get type of probing system from string
Record2Type::ProbingSystem_type::Type_type
GetProbingSystemTypeEnum(const mxArray *inp)
{
  // Input array must be a string
  // with one value of "Contacting", "NonContacting", "Software"
  wstring winp(ConvertMtoWStr(mxGetField(inp, 0,"ProbingSystem_Type")));
  
  if (winp == _T("Contacting"))
    return Record2Type::ProbingSystem_type::Type_type::Contacting;
  else if (winp == _T("NonContacting"))
    return Record2Type::ProbingSystem_type::Type_type::NonContacting;
  else if (winp == _T("Software"))
    return Record2Type::ProbingSystem_type::Type_type::Software;

  // Illegal value
  ostrstream msg;
  msg << "Error: Meta data field \"ProbingSystem_type\" has an illegal value\"" 
      << winp << "\". Possible values are one of \"Contacting\", \"NonContacting\" or \"Software\"." << endl << ends;
  mexErrMsgIdAndTxt("openGPS:writeX3P:ProbingSystem_type",msg.str());
  // This statement is never executed!
  return Record2Type::ProbingSystem_type::Type_type::Software;
}


/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  bool bHasStruct=false;
  
  wstring SyntaxHelp(  
     _T("Call Syntax:\n")
     _T("  pinfo = writeX3P(FileName,FeatureType,x,y,z,meta[,...])\n")
     _T("    FileName - name of file to write\n")
     _T("    FeatureType - one of 'PRF' for a line profile, 'SUR' for surface or 'PCL' for a point cloud.\n")
     _T("    x     - 1,2 or 3d array of x coordinates in meter\n")
     _T("    y     - 1,2 or 3d array of y coordinates in meter\n")
     _T("    z     - 1,2 or 3d array of z coordinates in meter\n")
     _T("    meta  - Meta data structure of the file with the following elements:\n")
     _T("     .Date    - Data set creation date of the form '2007-04-30T13:58:02.6+02:00'.\n")
     _T("                Set to empty string to use current time.\n")
     _T("     .Creator - optional name of the creator or empty array.\n")
     _T("     .Instrument_Manufacturer - String with name of the manufacturer\n")
     _T("     .Instrument_Model - String with instrument model or software name\n")
     _T("     .Instrument_Serial - String with serial number of instrument or 'not available'.\n")
     _T("     .Instrument_Version - Hardware and software version string of Instrument\n")
     _T("     .CalibrationDate - Date of last calibration of the form '2007-04-30T13:58:02.6+02:00'\n")
     _T("     .ProbingSystem_Type - one of  'Software','Contacting' or 'NonContacting'\n")
     _T("     .ProbingSystem_Identification - String identifying lens, probe tip, etc.\n")
     _T("     .Comment    - 'A user comment specific to this dataset'\n")          
     _T("    pinfo - Info about data organisation\n")
     _T("  Additional key-value-pairs can be specified after the last argument:\n")
     _T("    'rotation' - a 3x3 real matrix R containing a rotation matrix that is applied\n")
     _T("                 to each 3D-data point P on readback. R1 = R*P+T\n")
     _T("    'translation'- a 3 element vector T containing a translation vector for the data\n")
     _T("                 that is applied to each 3D-data point P on readback: R1 = R*P+T\n")
     _T("    'VendorSpecific' - a path of a file to be packet into x3p file as a vendor\n")
     _T("                 specific extension. The filename portion of the file is used as\n")
     _T("                 the URI for this extension is '") OGPS_VEXT_URI _T("'\n")
     _T("<a href=\"http://www.opengps.eu/\"www.opengps.eu</a>\n\n")
     OGPS_LICENSETEXT );
  SyntaxHelp.append(GetX3P_Dll_ID());

  /* check for proper number of arguments */
  if ((nrhs<6) || (nrhs>12))
  {
    ostrstream msg;
    msg << "writeX3P was called with " << nrhs << " input arguments!" << endl
        << SyntaxHelp << ends;
    mexErrMsgIdAndTxt("openGPS:writeX3P:nrhs",msg.str());
  }
  // Check number of output arguments
  if(nlhs>1)
  {
    ostrstream msg;
    msg << "writeX3P was called with " << nlhs << " output arguments!" << endl
        << SyntaxHelp << ends;
    mexErrMsgIdAndTxt("openGPS:writeX3P:nlhs",msg.str());
  }
  // Flag for structure output argument available
  bool hasPointInfo=false;
  if (nlhs>=1)
    hasPointInfo=true;

  // Filename
  const mxArray *inFileName=prhs[0];
  // feature type string
  const mxArray *inFeatureType=prhs[1];
  // Get pointers to input data
  const mxArray *inMatrixX=prhs[2];           /*input matrix with x-Values*/
  const mxArray *inMatrixY=prhs[3];           /*input matrix with y-Values*/
  const mxArray *inMatrixZ=prhs[4];           /*input matrix with z-Values*/
  const mxArray *meta=prhs[5];                /* Meta data structure */
  const mxArray **inKeywords = NULL;         /* Keyword-value pairs */
  const mxArray *inRotation=NULL;             /* Rotation matrix */
  const mxArray *inTranslation=NULL;          /* Translation vector */
  const mxArray *inVendorSpecific=NULL;       /* Argument string for vendor specific extension */

  /* vendor specific extension filename  */
  std::wstring VendorFileNameL(_T(""));
  
  // Flags for keyword existence
  bool bHasRotation=false;
  bool bHasTranslation=false;
  bool bHasExtension=false;

  const unsigned int cKeywordOffset=6;        //Position of first keyword argument
  const unsigned int inNKeywords = nrhs-cKeywordOffset;    /* number of keyword arguments */

  // Parse keyword arguments
  if (inNKeywords > 0)
  {
    // Get pointer to start of keyword value pairs
    inKeywords = &prhs[cKeywordOffset];
    // Parse keys
    for (unsigned int i=0; i<inNKeywords ; i++)
    {
      // Convert string to C
      wstring key(ConvertMtoWStr(inKeywords[i]));
      // convert keyword name to lower case
      transform(key.begin(), key.end(), key.begin(), tolower);

      // check rotation matrix
      if (key == _T("rotation"))
      {
        // check for available argument
        if (inNKeywords <= i+1)
        {
          // Throw error message
          mexErrMsgIdAndTxt("openGPS:writeX3P:missingArgument","'rotation' keyword not followd by an argument");
        }
        // Fetch next argument and increment index
        inRotation = inKeywords[++i];
        // check for argument type
        if( !mxIsNumeric(inRotation) || !mxIsDouble(inRotation) || mxGetNumberOfElements(inRotation)!=9 ) 
        {
          mexErrMsgIdAndTxt("openGPS:writeX3P:notMatrix","rotation matrix must have 9 double precision elements");
        }
        // Everything is fine
        bHasRotation = true;
      }
      // check rotation matrix
      else if (key == _T("translation"))
      {
        // check for available argument
        if (inNKeywords <= i+1)
        {
          // Throw error message
          mexErrMsgIdAndTxt("openGPS:writeX3P:missingArgument","'translation' keyword not followd by an argument");
        }
        // Fetch next argument and increment index
        inTranslation = inKeywords[++i];
        // check for argument type
        if( !mxIsNumeric(inTranslation) || !mxIsDouble(inTranslation) || mxGetNumberOfElements(inTranslation)!=3 ) 
        {
          mexErrMsgIdAndTxt("openGPS:writeX3P:notVector","translation vector must have 3 double precision elements");
        }
        // Everything is fine
        bHasTranslation = true;
      }
      else if (key == _T("vendorspecific"))
      {
        // check for available argument
        if (inNKeywords <= i+1)
        {
          // Throw error message
          mexErrMsgIdAndTxt("openGPS:writeX3P:missingArgument","'VendorSpecific' keyword not followd by an argument");
        }
        // Fetch next argument and increment index
        inVendorSpecific = inKeywords[++i];
        // Check argument type
        if( !mxIsChar(inVendorSpecific) || 
             mxGetNumberOfElements(inVendorSpecific)<1 ) {
             mexErrMsgIdAndTxt("openGPS:writeX3P:notString","'VendorSpecific' argument must be a string");
        }
        /* get the filename  */
        VendorFileNameL = std::wstring(ConvertMtoWStr(inVendorSpecific));
        bHasExtension = true;        
      }
      else
      {
        // Unkwon argument
        ostrstream msg;
        msg << "Warning: unknown keyword argument on position " << cKeywordOffset+i << "!" << endl << ends;
        mexWarnMsgIdAndTxt("openGPS:writeX3P:IllegalArgument",msg.str());
      }
    }
    
  }
  
  /* make sure the first input argument is string */
  if( !mxIsChar(inFileName) || 
       mxGetNumberOfElements(inFileName)<1 ) {
       mexErrMsgIdAndTxt("openGPS:writeX3P:notString","FileName argument must be a string");
  }
  /* get the filename  */
  std::wstring FileNameL(ConvertMtoWStr(inFileName));

  if( !mxIsChar(inFeatureType) || 
       mxGetNumberOfElements(inFeatureType)<1 ) {
       mexErrMsgIdAndTxt("openGPS:writeX3P:notString","FeatureType argument must be a string");
  }
  
  /* make sure the coordinate arrays are numeric */
  if( !mxIsNumeric(inMatrixX) || 
       mxGetNumberOfElements(inMatrixX)<1 ) {
       mexErrMsgIdAndTxt("openGPS:writeX3P:notNumeric","X input argument must be numeric");
  }
  if( !mxIsNumeric(inMatrixY) || 
       mxGetNumberOfElements(inMatrixY)<1 ) {
       mexErrMsgIdAndTxt("openGPS:writeX3P:notNumeric","Y input argument must be numeric");
  }
  if( !mxIsNumeric(inMatrixZ) || 
       mxGetNumberOfElements(inMatrixZ)<1 ) {
       mexErrMsgIdAndTxt("openGPS:writeX3P:notNumeric","Z input argument must be numeric");
  }
  
  /* make sure the meta data argument is a structure */
  if( !mxIsStruct(meta) || 
       mxGetNumberOfElements(meta)<1 ) {
       mexErrMsgIdAndTxt("openGPS:writeX3P:notNumeric","Meta data input argument must be a meta data structure");
  }

  // Get featuretype
  wstring ftype(ConvertMtoWStr(inFeatureType));
  // Define enum for featuretype
  typedef enum {FT_undefined, FT_profile, FT_surface, FT_pointcloud} FeatureType_T;
  FeatureType_T ft = FT_undefined;
  
  // Check featuretype argument
  if (ftype==OGPS_FEATURE_TYPE_PROFILE_NAME)
    ft = FT_profile;
  else if (ftype==OGPS_FEATURE_TYPE_SURFACE_NAME)
    ft = FT_surface;
  else if (ftype==OGPS_FEATURE_TYPE_POINTCLOUD_NAME)
    ft = FT_pointcloud;
  else
  {
    // Illegal value
    ostrstream msg;
    msg << "Error: FeatureType argument is \"" << ftype 
        << "\" must be one of 'SUR', 'PRF', or 'PCL'!" 
        << endl << ends;
    mexErrMsgIdAndTxt("openGPS:writeX3P:FeatureType",msg.str());
    return;    
  }

  // Get number of data dimensions
  mwSize ndimx = mxGetNumberOfDimensions(inMatrixX);
  mwSize ndimy = mxGetNumberOfDimensions(inMatrixY);
  mwSize ndimz = mxGetNumberOfDimensions(inMatrixZ);
  
  // Has to be one to 3 dimensions
  if ((ndimx < 1) || (ndimx>3) || (ndimy < 1) || (ndimy>3) || (ndimz < 1) || (ndimz>3))
  {
    // Print full description
    ostrstream msg;
    msg << "Error: Input arrays must have 1 to 3 dimensions but have [ndimx,ndimy,ndimz]=[" 
        << ndimx << ", " << ndimy << ", " << ndimz << "]!" << endl << ends;
    mexErrMsgIdAndTxt("openGPS:writeX3P:InputDimensions",msg.str());
    return;
  }  
  
  // Check meta data for completeness
  if (false == IsMetaComplete(meta))
    return;
  
  // Get dimensions of z-data matrix
  // Other dimensions are a surface
  const mwSize *dim = mxGetDimensions(inMatrixZ);

  // x or y are incremental if dimension of x or y are [n,1] or [1,n]
  // and contained values are close to beeing incremental
  double xIncrement=0;
  double yIncrement=0;
  // Flags for incremental axis
  bool xIncremental=false;
  bool yIncremental=false;
  // Switch for featuretype, check data dimensions and incremental axis
  switch (ft)
  {
    //********************** Profile Feature type
    case FT_profile:
      // A profile must have dimensions N,1,M or 1,N,M
      // Both cases are handled the same way
      if ((dim[0] != 1) && (dim[1] != 1))
      {
        // Print full description
        ostrstream msg;
        msg << "Error: For a line profile the input array Z must have dimensions [u,1,w] but has [" 
            << dim[0] << ", " << dim[1] << ", w]!" << endl << ends;
        mexErrMsgIdAndTxt("openGPS:writeX3P:NotProfileData",msg.str());
        return;
      }

      // A profile must have more than one point n>1 and m>1
      if ((dim[0] * dim[1])<2)
      {
        // Print full description
        ostrstream msg;
        msg << "Error: A profile must have at least two points. One of dimensions u and v must be > 1. [u,v,w] is [" 
            << dim[0] << ", " << dim[1] << ", w]!" << endl << ends;
        mexErrMsgIdAndTxt("openGPS:writeX3P:NotProfileData",msg.str());
        return;
      }

      // Check the used axis for incremental type. The other axis is allways incremental.
      if (1==dim[0])
        xIncremental = true;
      else if (AxisIsIncremental(inMatrixX, xIncrement))
        xIncremental = true;
      if (1==dim[1])
        yIncremental = true;
      else if (AxisIsIncremental(inMatrixY, yIncrement))
        yIncremental = true;
      break;
    //********************** Surface Feature type
    case FT_surface:
      // A surface must have n>1 and m>1
      if ((dim[0]<2) || (dim[1]<2))
      {
        // Print full description
        ostrstream msg;
        msg << "Error: For a surface the input array Z must have dimensions [u,v,w] with u>=2 and v>=2 but has [" 
            << dim[0] << ", " << dim[1] << ", w]!" << endl << ends;
        mexErrMsgIdAndTxt("openGPS:writeX3P:NotSurfaceData",msg.str());
        return;
      }
      
      // Check for incremental axis
      if (AxisIsIncremental(inMatrixX, xIncrement))
        xIncremental = true;
      if (AxisIsIncremental(inMatrixY, yIncrement))
        yIncremental = true;
      break;
    //********************** Point cloud Feature type
    case FT_pointcloud:
      // Point cloud allows any dimension but no incremental axis
      xIncremental = false;
      yIncremental = false;
      break;
  }
  
  
  // Create records for document
  Record1Type::Revision_type revision(OGPS_ISO5436_2000_REVISION_NAME);
  
  // Set feature type
  Record1Type::FeatureType_type featureType(ftype);
  
  // Axis type
  Record1Type::Axes_type::CX_type::AxisType_type *xaxisType=NULL;

  if (xIncremental)
    // Make incremental axis
    xaxisType = new Record1Type::Axes_type::CX_type::AxisType_type(Record1Type::Axes_type::CX_type::AxisType_type::I); 
  else
    // Absolute axis
    xaxisType = new Record1Type::Axes_type::CX_type::AxisType_type(Record1Type::Axes_type::CX_type::AxisType_type::A); 

  // TODO: Data type is allways double
  Record1Type::Axes_type::CX_type::DataType_type xdataType(Record1Type::Axes_type::CX_type::DataType_type::D); /* double */  
  // Create axis of specified type
  Record1Type::Axes_type::CX_type xaxis(*xaxisType);
  xaxis.DataType(xdataType);
  /* Set increment and offset. */
  if (xIncrement==0)
    xaxis.Increment(1.0); // Default value
  else
    xaxis.Increment(xIncrement);
  // Todo: Offset should be set to -average of data min/max
  if (bHasTranslation)
    // Set translation to value specified in keyword argument
    xaxis.Offset(((double*)mxGetPr(inTranslation))[0]);
  else
    xaxis.Offset(0.0);
  // Delete the axis type 
  delete xaxisType;


  // Y Axis type
  Record1Type::Axes_type::CY_type::AxisType_type *yaxisType=NULL;

  if (yIncremental)
    // Make incremental axis
    yaxisType = new Record1Type::Axes_type::CY_type::AxisType_type(Record1Type::Axes_type::CY_type::AxisType_type::I);
  else
    yaxisType = new Record1Type::Axes_type::CY_type::AxisType_type(Record1Type::Axes_type::CY_type::AxisType_type::A);
  
  // ToDo: Data type is allways double
  Record1Type::Axes_type::CY_type::DataType_type ydataType(Record1Type::Axes_type::CY_type::DataType_type::D); /* double */  
  // Create axis of specified type
  Record1Type::Axes_type::CY_type yaxis(*yaxisType);
  yaxis.DataType(ydataType);
  /* Set increment and offset. */
  if (yIncrement==0)
    yaxis.Increment(1.0); // Default
  else
    yaxis.Increment(yIncrement);

  // Todo: Offset should be set to -average of data min/max
  if (bHasTranslation)
    // Set translation to value specified in keyword argument
    yaxis.Offset(((double*)mxGetPr(inTranslation))[1]);
  else
    yaxis.Offset(0.0);
  // Delete the axis type 
  delete yaxisType;

  // Setup z-axis, allways double absolute
  Record1Type::Axes_type::CZ_type::AxisType_type zaxisType(Record1Type::Axes_type::CZ_type::AxisType_type::A); /* absolute */
  Record1Type::Axes_type::CZ_type::DataType_type zdataType(Record1Type::Axes_type::CZ_type::DataType_type::D); /* double */
  Record1Type::Axes_type::CZ_type zaxis(zaxisType);
  zaxis.DataType(zdataType);
  zaxis.Increment(1.0);
  // Todo: Offset should be set to average of data min/max
  if (bHasTranslation)
    // Set translation to value specified in keyword argument
    zaxis.Offset(((double*)mxGetPr(inTranslation))[2]);
  else
    zaxis.Offset(0.0);

  // Create the final axis definition for all three axis
  Record1Type::Axes_type axis(xaxis, yaxis, zaxis);
  
  // Create Rotation matrix
  AxesType::Rotation_type *rot=NULL;
  
  if (bHasRotation)
  {
    // Get pointer to rotation matrix
    double *rotMat = mxGetPr(inRotation);
    // Create Rotation matrix
    rot = new AxesType::Rotation_type(
           rotMat[0],rotMat[3],rotMat[6],
           rotMat[1],rotMat[4],rotMat[7],
           rotMat[2],rotMat[5],rotMat[8]);
    // add rotation matrix to axes type element
    axis.Rotation(*rot);
  }
  
  // Create Record1
  Record1Type record1(revision, featureType, axis);
   
  /* Create RECORD2 with information from meta data structure */
  // Data set creation date
  Record2Type::Date_type *date=NULL;
  // Check for a date with correct length given in meta data field
  // TODO: We should at least do some format cecking
  mwSize dlen = mxGetNumberOfElements(mxGetField(meta, 0,"Date"));
  if (dlen != 27)
  {
    // If length is less than 2 we assume it has been left blank intentionally
    if (dlen > 1)
    {
      // Issue a warning
      mexWarnMsgIdAndTxt("openGPS:writeX3P:CreationDate",
              "The creation date must have a length of 27 characters and a format\n"
              "similar to '2007-04-30T13:58:02.6+02:00'. Using the current date as\n"
              "data set creation date\n");
    }
    // Use current date as creation date
    date = new Record2Type::Date_type(TimeStamp());
  }
  else
    // Use the given date
    date = new Record2Type::Date_type(ConvertMtoWStr(mxGetField(meta, 0,"Date")));
    
  
  // Instrument manufacturer
  Record2Type::Instrument_type::Manufacturer_type 
          manufacturer(ConvertMtoWStr(mxGetField(meta, 0,"Instrument_Manufacturer")));
  
  // Instrument model
  Record2Type::Instrument_type::Model_type 
          model(ConvertMtoWStr(mxGetField(meta, 0,"Instrument_Model")));

  // Instrument serial number set to default if not specified
  Record2Type::Instrument_type::Serial_type *serial=NULL;

  if (mxGetNumberOfElements(mxGetField(meta, 0,"Instrument_Serial")) == 0)
  {
    serial = new Record2Type::Instrument_type::Serial_type(_T("not available"));
  }
  else
  {
    serial = new Record2Type::Instrument_type::Serial_type(
            ConvertMtoWStr(mxGetField(meta, 0,"Instrument_Serial")));
  }
  // Instrument hard and software version
  Record2Type::Instrument_type::Version_type 
          version(ConvertMtoWStr(mxGetField(meta, 0,"Instrument_Version")));
  // Assemble instrument information
  Record2Type::Instrument_type instrument(manufacturer, model, *serial, version);
  // Delete serial element
  delete serial;
  serial=NULL;
  
  // Calibration date
  Record2Type::CalibrationDate_type 
          calibrationDate(ConvertMtoWStr(mxGetField(meta, 0,"CalibrationDate")));

  // Type of probing system: Contacting, non Contacting, Software 
  Record2Type::ProbingSystem_type::Type_type type(GetProbingSystemTypeEnum(meta));

  // Type of probing system
  Record2Type::ProbingSystem_type::Identification_type id(ConvertMtoWStr(mxGetField(meta, 0,"ProbingSystem_Identification")));
  // Create the probing system type
  Record2Type::ProbingSystem_type probingSystem(type, id);

  // Create Record2 from collected data
  Record2Type record2(*date, instrument, calibrationDate, probingSystem);
  
  // Delete date record
  if (date)
    delete date;

  // Look out for comment field
  mxArray *cfield = mxGetField(meta, 0,"Comment");
  if (cfield != NULL)
  {
    // Check for elements in comment string
    if (mxGetNumberOfElements(cfield)>0)
    {
      // Add comment to record 2
      record2.Comment(Record2Type::Comment_type(ConvertMtoWStr(cfield)));
    }
    else
      cfield=NULL;
  }
  
  if (cfield==NULL)
  {
    // Issu warning
    mexWarnMsgIdAndTxt("openGPS:writeX3P:Comment",
            "Data should contain a meaningful comment.");
  }

  // Check for creator field
  mxArray *crfield = mxGetField(meta, 0,"Creator");
  if (crfield != NULL)
  {
    // Check for elements in comment string
    if (mxGetNumberOfElements(crfield)>0)
    {
      // Add comment to record 2
      record2.Creator(Record2Type::Creator_type(ConvertMtoWStr(crfield)));
    }
  }

  /* Create MATRIX */
  unsigned long mdims[3];
  mdims[0] = dim[0];
  if (ndimz > 1)
    mdims[1] = dim[1];
  else
    mdims[1] = 1;
  if (ndimz > 2)
    mdims[2] = dim[2];
  else
    mdims[2] = 1;

  bool useBinary=false;
  // For more than 5000 Elements use binary encoding
  if (mxGetNumberOfElements(inMatrixZ)>=5000)
    useBinary=true;

  // Define a handle for the container
  OGPS_ISO5436_2Handle handle = NULL;

  /* Create ISO5436_2 container, */
  if (ft == FT_pointcloud)
  {
    // This is a list data type
    handle =
      ogps_CreateListISO5436_2(FileNameL.c_str(), NULL, record1, &record2, mdims[0]*mdims[1]*mdims[2], useBinary);
  }
  else
  {
    // all other are matrix data types
    // Define dimensions
    MatrixDimensionType dimMatrix(mdims[0], mdims[1], mdims[2]);
    // Create the container
    handle = 
      ogps_CreateMatrixISO5436_2(FileNameL.c_str(), NULL, record1, &record2, dimMatrix, useBinary);
  }

  /* Add data points */
  /* 1. Create point vector buffer */
  OGPS_PointVectorPtr vector = ogps_CreatePointVector();

  /* Matrix indices. */
  OGPS_ULong u = 0;
  OGPS_ULong v = 0;
  OGPS_ULong w = 0;

  // Data pointer to z,y,x
  OGPS_Double *pdblZ=NULL,*pdblY=NULL,*pdblX=NULL;
  OGPS_Float *pfltZ=NULL,*pfltY=NULL,*pfltX=NULL;
  OGPS_Int16 *pshrtZ=NULL,*pshrtY=NULL,*pshrtX=NULL;
  OGPS_Int32 *plngZ=NULL,*plngY=NULL,*plngX=NULL;
  // Data type
  unsigned int dtype=0;
  if (mxIsDouble(inMatrixZ))
    dtype=1;
  else if (mxIsSingle(inMatrixZ))
    dtype=2;
  else if (mxIsUint16(inMatrixZ))
    dtype=3;
  else if (mxIsUint32(inMatrixZ))
    dtype=4;  
  
  // Get data pointer
  switch (dtype)
  {
    case 1:
      // Get pointer to z data
      pdblZ = (OGPS_Double*)mxGetPr(inMatrixZ);
      // Get pointer to xy data if not incremental
      if (!xIncremental)
        pdblX = (OGPS_Double*)mxGetPr(inMatrixX);
      if (!yIncremental)
        pdblY = (OGPS_Double*)mxGetPr(inMatrixY);
      break;
    case 2:
      // Get pointer to z data
      pfltZ = (OGPS_Float*)mxGetPr(inMatrixZ);
      // Get pointer to xy data if not incremental
      if (!xIncremental)
        pfltX = (OGPS_Float*)mxGetPr(inMatrixX);
      if (!yIncremental)
        pfltY = (OGPS_Float*)mxGetPr(inMatrixY);
      break;
    case 3:
      // Get pointer to z data
      pshrtZ = (OGPS_Int16*)mxGetPr(inMatrixZ);
      // Get pointer to xy data if not incremental
      if (!xIncremental)
        pshrtX = (OGPS_Int16*)mxGetPr(inMatrixX);
      if (!yIncremental)
        pshrtY = (OGPS_Int16*)mxGetPr(inMatrixY);
      break;
    case 4:
      // Get pointer to z data
      plngZ = (OGPS_Int32*)mxGetPr(inMatrixZ);
      // Get pointer to xy data if not incremental
      if (!xIncremental)
        plngX = (OGPS_Int32*)mxGetPr(inMatrixX);
      if (!yIncremental)
        plngY = (OGPS_Int32*)mxGetPr(inMatrixY);
      break;
  }
  
  // point index for lists
  OGPS_ULong index=0;
  // Loop over array
  for (w=0; w<mdims[2] ; ++w)
  {
    for (v=0; v<mdims[1]; ++v)
    {
      for (u=0; u<mdims[0]; ++u)
      {
        bool isValid=true;
        // Set z-coordinate
        switch (dtype)
        {
          case 1:
            // Check missing data
            isValid = !mxIsNaN(*pdblZ) 
                    && (xIncremental ? true : !mxIsNaN(*pdblX))
                    && (yIncremental ? true : !mxIsNaN(*pdblY));
            if (isValid)
            {
              // Set z-value
              ogps_SetDoubleZ(vector, *(pdblZ++));
              // Set x value if not incremental
              if (!xIncremental)
                ogps_SetDoubleX(vector, *(pdblX++));
              // Set y value if not incremental
              if (!yIncremental)
                ogps_SetDoubleY(vector, *(pdblY++));
            }
            else
            {
              // Increment pointers
              ++pdblZ;
              if (!xIncremental)
                ++pdblX;
              if (!yIncremental)
                ++pdblY;
            }
            break;
          case 2:
            // Check missing data
            isValid = !mxIsNaN(*pfltZ) 
                    && (xIncremental ? true : !mxIsNaN(*pfltX))
                    && (yIncremental ? true : !mxIsNaN(*pfltY));
            if (isValid)
            {
              // Set z-value
              ogps_SetFloatZ(vector, *(pfltZ++));
              // Set x value if not incremental
              if (!xIncremental)
                ogps_SetFloatX(vector, *(pfltX++));
              // Set y value if not incremental
              if (!yIncremental)
                ogps_SetFloatY(vector, *(pfltY++));
            }
            else
            {
              // Increment pointers
              ++pfltZ;
              if (!xIncremental)
                ++pfltX;
              if (!yIncremental)
                ++pfltY;
            }
            break;
          case 3:
            // BUG: This is not fully implemented yet!
            // Set z-value
            ogps_SetInt16Z(vector, *(pshrtZ++));
            break;
          case 4:
            // BUG: This is not fully implemented yet!
            // Set z-value
            ogps_SetInt32Z(vector, *(plngZ++));
            break;
        }
          
        /* 2. if the z axis is of absolute type and the
        * other two are incremental, we simply set up just z
        * values and leave x an y values untouched (missing). */

        if (isValid)
        {
          /* 3. Write into document */
          // Check for feature type: PCL have list, other have matrix
          if (ft == FT_pointcloud)
            // Unsorted point list
            ogps_SetListPoint(handle, index++, vector);
          else
            // Matrix organized point list
            ogps_SetMatrixPoint(handle, u , v, w, vector);
        }
        else
        {
          // PCL does not have invalid points
          if (ft != FT_pointcloud)
            // Set data point to invalid
            ogps_SetMatrixPoint(handle, u , v, w , NULL);
        }
      }
    }
  }
  /* Free buffer */
  ogps_FreePointVector(&vector);

  /* append vendorspecific data */
  if (bHasExtension)
  {
    // add file to zip archive
    ogps_AppendVendorSpecific(handle, OGPS_VEXT_URI, VendorFileNameL.c_str());
  }

  // Create the point info structure
  // Get document 
  const ISO5436_2Type* const document = ogps_GetDocument(handle);
  if (ogps_HasError())
  {
    mexErrMsgIdAndTxt("openGPS:openX3P:XMLDocument","Could not access XML document!");
  }
  // Get a reference to record 1
  const OpenGPS::Schemas::ISO5436_2::ISO5436_2Type::Record1_type &r1 = document->Record1();

  // get point info 
  if (hasPointInfo)
     plhs[0] = GetPointInfoStructure(handle);
  
  /* Finally: write container to disk. */
  ogps_WriteISO5436_2(handle);
  ogps_CloseISO5436_2(&handle);

  ///***********************************************************************
  // Check for error
  if (ogps_HasError())
  {
    // Print full description
    ostrstream msg;
    msg << "Error writing X3P-file name \"" << FileNameL << "\"!" << endl
        << ogps_GetErrorMessage() << endl
        << ogps_GetErrorDescription() << endl
        << ends;
    mexErrMsgIdAndTxt("openGPS:writeX3P:FileOpen",msg.str());
    return;
  }
}
