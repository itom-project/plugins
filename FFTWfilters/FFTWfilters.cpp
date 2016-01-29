/* ************************************************************************
    This file is part of fftw-plugin for ito's itom measurement software

    The fftw-plugin for itom is a wrapper for the FFTW package.
    The FFTW package was developed at MIT by Matteo Frigo and Steven G. 
    Johnson. It was published unter GNU General Public License and 
    can be downloaded unter http://www.fftw.org/.

    The fftw-plugin is a free software: you can redistribute it and/or 
    modify it under the terms of the GNU General Public License 
    as published by the Free Software Foundation, either version 3 of 
    the License, or (at your option) any later version.

    the fftw-plugin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar. If not, see <http://www.gnu.org/licenses/>.
************************************************************************ */

/*! \file FFTWfilters.cpp
   \brief   This file contains the itomflters class and interface definitions.
   
   \author ITO 
   \date 12.2011
*/

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "FFTWfilters.h"

#include "DataObject/dataObjectFuncs.h"
#include "fftw3.h"
#include "qnumeric.h"
#include "qvariant.h"

#include "pluginVersion.h"

#include <QtCore/QtPlugin>
#include <math.h>

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFiltersInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(FFTWFilters)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFiltersInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(FFTWFilters)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FFTWFiltersInterface::FFTWFiltersInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("FFTW-Filter");

/*    char docstring[] = \
"This plugin provides several wrapper for several fftw-functions for itom::dataObject. These are for instance: \n\
- linewise FFT\n\
- linewise inverse FFT \n\
- 2D-fft \n\
- inverse2d-FFT \n\
- gaussian-filtering according to DIN EN ISO 16610-21 \n\
\n\
The FFTW package was developed at MIT by Matteo Frigo and Steven G. Johnson.\
It was published unter GNU General Public License and can be downloaded unter http://www.fftw.org/ .\n\
\n\
To build this plugin you will need the libs from the fftw.";
*/
    m_description = QObject::tr("Wrapper for the FFTW");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"This plugin provides several wrapper for several fftw-functions for dataObject. These are for instance: \n\
- linewise FFT\n\
- linewise inverse FFT \n\
- 2D-fft \n\
- inverse2d-FFT \n\
\n\
The FFTW package was developed at MIT by Matteo Frigo and Steven G. Johnson.\
It was published unter GNU General Public License and can be downloaded unter http://www.fftw.org/ .\n\
\n\
To build this plugin you will need the libs from the fftw.");

    m_author = "W. Lyda, twip optical solutions GmbH, T. Boettcher, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("GPL (uses FFTW licensed under GPL, too)");
    m_aboutThis = QObject::tr("Algorithms using FFTW");       

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
FFTWFiltersInterface::~FFTWFiltersInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(FFTWFiltersInterface, FFTWFiltersInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
FFTWFilters::FFTWFilters() : AddInAlgo()
{   
}

//----------------------------------------------------------------------------------------------------------------------------------
FFTWFilters::~FFTWFilters() 
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Parameters for fftw filter
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO, Boettcher
   \date
*/
ito::RetVal FFTWFilters::ParamsFFTW(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input Object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output Object").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("plan_flag", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("Planner flag, 0: estimate (default), 1: measure").toLatin1().data());
        paramsOpt->append(param);
    
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs forward dft by means of fftw row by row (# rows >=1) complex to complex
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in, 
                                                        [1]complex Data object out 
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)     
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::fftw1dDOC = QObject::tr("Perfom an unscaled 1D-fft for the given object. \n\
\n\
This filter uses the fft function provided by the fftw-library to perfom a row-wise fft on the input object. \n\
\n\
If the object is a single column object the fft is calculated along the y axis. In other cases the fft is calculated along x axis.\n\
\n\
The filter works in-place and out-of-place. If the input object is not complex128, a temporary object of this type will be created.\n\
If the filter is used with input- as output-object and the type is not complex128, the input-object will be overwritten with the output-type.\n\
\n\
If the object has a ROI a ROI-less object correponding to the ROI is created. \n\
If the filter is used with ROI-object as input- / output-object, the input-object will be overwritten with the ROI-less object.\n");

ito::RetVal FFTWFilters::fftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFTW(paramsMand, paramsOpt, true, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs backward dft by means of fftw row by row (# rows >=1) complex to complex
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in, 
                                                        [1]complex Data object out 
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)     
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::ifftw1dDOC = QObject::tr("Perfom an unscaled inverse 1D-fft for the given object. \n\
\n\
This filter uses the fft function provided by the fftw-library to perfom a inverse row-wise fft on the input object. \n\
\n\
If the object is a single column object the fft is calculated along the y axis. In other cases the fft is calculated along x axis.\n\
\n\
The filter works in-place and out-of-place. If the input object is not complex128, a temporary object of this type will be created.\n\
If the filter is used with input- as output-object and the type is not complex128, the input-object will be overwritten with the output-type.\n\
\n\
If the object has a ROI a ROI-less object correponding to the ROI is created. \n\
If the filter is used with ROI-object as input- / output-object, the input-object will be overwritten with the ROI-less object.\n");

ito::RetVal FFTWFilters::ifftw1d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFTW(paramsMand, paramsOpt, false, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs forward dft of 2D objects complex to complex by means of fftw 
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in, 
                                                        [1]complex Data object out 
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)     
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::fftw2dDOC = QObject::tr("Perfom an unscaled 2D-fft for the given object. \n\
\n\
This filter uses the fft function provided by the fftw-library to perfom a fft on the input object.\n\
\n\
The filter works in-place and out-of-place. If the input object is not complex128, a temporary object of this type will be created.\n\
If the filter is used with input- as output-object and the type is not complex128, the input-object will be overwritten with the output-type.\n\
\n\
If the object has a ROI a ROI-less object correponding to the ROI is created. \n\
If the filter is used with ROI-object as input- / output-object, the input-object will be overwritten with the ROI-less object.\n");

ito::RetVal FFTWFilters::fftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFTW(paramsMand, paramsOpt, true, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs backward dft of 2D objects complex to complex by means of fftw 
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in, 
                                                        [1]complex Data object out 
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)     
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
const QString FFTWFilters::ifftw2dDOC = QObject::tr("Perfom an unscaled inverse 2D-fft for the given object. \n\
\n\
This filter uses the fft function provided by the fftw-library to perfom a inverse fft on the input object.\n\
\n\
The filter works in-place and out-of-place. If the input object is not complex128, a temporary object of this type will be created.\n\
If the filter is used with input- as output-object and the type is not complex128, the input-object will be overwritten with the output-type.\n\
\n\
If the object has a ROI a ROI-less object correponding to the ROI is created. \n\
If the filter is used with ROI-object as input- / output-object, the input-object will be overwritten with the ROI-less object.\n");

ito::RetVal FFTWFilters::ifftw2d(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    return doFFTW(paramsMand, paramsOpt, false, false);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs 1D and 2D ffts forward and backwards according to bool flags by means of fftw 
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in, 
                                                        [1]complex Data object out 
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)     
   \param[in]       forward     toggle fft and ifft
   \param[in]       lineWise    toggle fft-1D and fft-2D
   \author Lyda, Boettcher
   \date 2012.03.06
*/
ito::RetVal FFTWFilters::doFFTW(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, const bool forward, const bool lineWise)
{
    
    ito::RetVal retval = ito::retOk;
    
    const ito::DataObject *dObjIn = (*paramsMand)[0].getVal<ito::DataObject*>();    //Input object
    ito::DataObject *dObjOut = (*paramsMand)[1].getVal<ito::DataObject*>();    //Output object
    int plan_select = (*paramsOpt)[0].getVal<int>();                                            //plan selection string for fftw
    long int dimensions=0;
    unsigned int plan_sel = 0;
    unsigned int planForwardBackWard = forward ? FFTW_FORWARD : FFTW_BACKWARD;
    QString msg;

    if (!dObjIn)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjOut)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }
    
    retval += ito::dObjHelper::verifyDataObjectType(dObjIn, "sourceObject", 9, ito::tInt8, ito::tUInt8, 
                                                                         ito::tInt16, ito::tUInt16, 
                                                                         ito::tInt32, 
                                                                         ito::tFloat32, ito::tFloat64, 
                                                                         ito::tComplex64, ito::tComplex128);

    if (retval.containsError())
    {
        return retval;
    }

    bool neededNewOutput = false;
    bool doItInplace = false;
    bool neededNewInput = true;

    bool inputHasROI = false;
    bool outputHasROI = false;

    dimensions = (*dObjIn).getDims();
    if (dimensions < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image not initilized").toLatin1().data());
    }
    
    ito::DataObject inputObject;
    ito::DataObject outputObject;

    ito::int32 xSize = static_cast<ito::int32>(dObjIn->getSize(dimensions - 1));
    ito::int32 ySize = static_cast<ito::int32>(dObjIn->getSize(dimensions - 2));

    if (!lineWise && (xSize == 1 || ySize == 1))
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image dimensions must not be 1xN or Nx1 for fft2D.").toLatin1().data());
    }

    int *sizes = new int[dimensions];
    int *offsets = new int[dimensions];

    dObjIn->locateROI(sizes, offsets);
    if (sizes[dimensions - 2] != ySize || sizes[dimensions - 1] != xSize)
    {
        inputHasROI = true;
    }

    delete[] sizes;
    delete[] offsets;
    
    if (dObjIn->getType() == ito::tComplex128 && !inputHasROI)
    {
        neededNewInput = false;
        inputObject = *dObjIn;
    }
    else
    {
        neededNewInput = true;
        inputObject = ito::DataObject(dimensions, dObjIn->getSize(), ito::tComplex128);

        const cv::Mat* scrMat;
        ito::complex128* cRowPtr;

        switch(dObjIn->getType())
        {
            case ito::tInt8:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::int8>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tUInt8:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::uint8>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tInt16:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::int16>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tUInt16:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::uint16>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tInt32:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::int32>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tFloat32:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::float32>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tFloat64:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::float64>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tComplex64:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::complex64>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            case ito::tComplex128:
                for (int z = 0; z < inputObject.calcNumMats(); z++)
                {
                    scrMat = (cv::Mat*)(dObjIn->get_mdata()[dObjIn->seekMat(z)]);
                    cRowPtr = (ito::complex128*)((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(z)])->ptr<ito::complex128>();
                    for (int y = 0; y < ySize; y++)
                    {
                        ito::dObjHelper::GetHLineC<ito::complex128>(scrMat, 0, y, xSize, cRowPtr);
                        cRowPtr = &(cRowPtr[xSize]);
                    }
                }
                break;
            default:
                return ito::RetVal(ito::retError, 0, tr("Type not supported ").toLatin1().data());
        }
    }
    
    if (dObjIn == dObjOut)
    {
        outputObject = inputObject;

        if (neededNewInput)
        {
            dObjIn->copyAxisTagsTo(outputObject);
            dObjIn->copyTagMapTo(outputObject);
            
            doItInplace = true;
            neededNewOutput = true;
        }
        else
        {
            //doItInplace = true;
            neededNewOutput = false;
            doItInplace = false;
        }
    }
    else
    {
        int *sizes = new int[dObjOut->getDims()];
        int *offsets = new int[dObjOut->getDims()];

        int xSizeDest = dObjOut->getSize(dObjOut->getDims() - 1);
        int ySizeDest = dObjOut->getSize(dObjOut->getDims() - 2);

        dObjOut->locateROI(sizes, offsets);
        if (sizes[dObjOut->getDims() - 2] != ySizeDest || sizes[dObjOut->getDims() - 1] != xSizeDest)
        {
            outputHasROI = true;
        }

        delete[] sizes;
        delete[] offsets;

        if (dObjIn->calcNumMats() != dObjOut->calcNumMats() ||
           dObjOut->getType() != ito::tComplex128 ||
           xSize !=  xSizeDest ||
           ySize !=  ySizeDest ||
           outputHasROI)
        {
            neededNewOutput = true;
            doItInplace = false;
            if (neededNewInput) outputObject = inputObject;
            else outputObject = ito::DataObject(dimensions, dObjIn->getSize(), ito::tComplex128);
        }
        else
        {
            neededNewOutput = false;
            doItInplace = true;
            outputObject = *dObjOut;
        }
        dObjIn->copyAxisTagsTo(outputObject);
        dObjIn->copyTagMapTo(outputObject);
    }

    if (plan_select == 1)            
    {
        plan_sel = FFTW_MEASURE;                            // from fftw.h: #define FFTW_MEASURE (0U), #define FFTW_ESTIMATE (1U << 6)
    }
    else 
    {
        plan_sel = FFTW_ESTIMATE;                        // estimate as standard, quicker planning, probably slower fft-computing
    }

    int numPlanes = inputObject.getNumPlanes();

    if (lineWise)
    {
        msg = forward ? tr("Applied 1D-fft via FFTW (unscaled!)") : tr("Applied inverse 1D-fft via FFTW (unscaled!)");
        fftw_complex *in  = (fftw_complex*)(((cv::Mat*)inputObject.get_mdata()[inputObject.seekMat(0)])->ptr<ito::complex128>());
        fftw_complex *out = (fftw_complex*)(((cv::Mat*)outputObject.get_mdata()[outputObject.seekMat(0)])->ptr<ito::complex128>());

        //qDebug() << fftw_alignment_of((double*)in) << fftw_alignment_of((double*)out);
        

        // 1D complex to complex
        if (ySize == 1)                    
        {
            fftw_plan plan = fftw_plan_dft_1d(xSize, in, out, planForwardBackWard, plan_sel);
            fftw_execute(plan);

            for (int z = 1; z < numPlanes; z++)
            {
                fftw_complex *in = (fftw_complex*)((inputObject.get_mdata()[inputObject.seekMat(z, numPlanes)])->ptr<ito::complex128>());
                fftw_complex *out = (fftw_complex*)((outputObject.get_mdata()[outputObject.seekMat(z, numPlanes)])->ptr<ito::complex128>());
                fftw_execute_dft(plan, in, out);
            }        // end of 1D

            fftw_destroy_plan(plan);
        }
        else // 2D row by row complex to complex                            
        {
            // calculate plan before initialising _in_! some keywords destroy _in_ while planning
            int _xSize[]={xSize};
            fftw_plan plan = fftw_plan_many_dft(1, _xSize, ySize, in, NULL, 1, xSize, out, NULL, 1, xSize, planForwardBackWard, plan_sel);            
            fftw_execute(plan);

            for (int z = 1; z < numPlanes; z++)
            {
                fftw_complex *in = (fftw_complex*)((inputObject.get_mdata()[inputObject.seekMat(z, numPlanes)])->ptr<ito::complex128>());
                fftw_complex *out = (fftw_complex*)((outputObject.get_mdata()[outputObject.seekMat(z, numPlanes)])->ptr<ito::complex128>());
                fftw_execute_dft(plan, in, out);
            }    

            fftw_destroy_plan(plan);    
        }
    }
    else
    {
        msg = forward ? tr("Applied 2D-fft via FFTW (unscaled!)") : tr("Applied inverse 2D-fft via FFTW (unscaled!)");
        fftw_complex *in = (fftw_complex*)((inputObject.get_mdata()[inputObject.seekMat(0, numPlanes)])->ptr<ito::complex128>());
        fftw_complex *out = (fftw_complex*)((outputObject.get_mdata()[outputObject.seekMat(0, numPlanes)])->ptr<ito::complex128>());
        fftw_plan plan = fftw_plan_dft_2d(ySize, xSize, in, out, planForwardBackWard, plan_sel);
        fftw_execute(plan);

        for (int z = 1; z < numPlanes; z++)
        {
            fftw_complex *in = (fftw_complex*)((inputObject.get_mdata()[inputObject.seekMat(z, numPlanes)])->ptr<ito::complex128>());
            fftw_complex *out = (fftw_complex*)((outputObject.get_mdata()[outputObject.seekMat(z, numPlanes)])->ptr<ito::complex128>());
            fftw_execute_dft(plan, in, out);
        }    
        fftw_destroy_plan(plan);
    }
    
    if (neededNewOutput && !retval.containsError())
    {
        *dObjOut = outputObject;
    }

    if (!retval.containsError())
    {
        if (dObjIn != dObjOut) dObjIn->copyTagMapTo(*dObjOut);
        dObjOut->addToProtocol(std::string(msg.toLatin1().data()));

        int curDim = dObjOut->getDims()-1;
        std::string axisUnit;
        bool test;
        
        ito::float64 newScale = dObjOut->getAxisScale(curDim);
        if (ito::dObjHelper::isFinite<ito::float64>(newScale) && ito::dObjHelper::isNotZero<ito::float64>(newScale))
        {
            newScale = 1/newScale / dObjOut->getSize(curDim);
            axisUnit = ito::dObjHelper::invertUnit(dObjOut->getAxisUnit(curDim, test));
            dObjOut->setAxisUnit(curDim, axisUnit);
        }
        else
        {
            newScale = 1.0;
            dObjOut->setAxisUnit(curDim, "");
        }
        dObjOut->setAxisScale(curDim, newScale);
        dObjOut->setAxisOffset(curDim, 0.0);

        if (!lineWise)
        { 
            curDim --;

            newScale = dObjOut->getAxisScale(curDim);
            
            if (ito::dObjHelper::isFinite<ito::float64>(newScale) && ito::dObjHelper::isNotZero<ito::float64>(newScale))
            {
                newScale = 1/newScale / dObjOut->getSize(curDim);
                axisUnit = ito::dObjHelper::invertUnit(dObjOut->getAxisUnit(curDim, test));
                dObjOut->setAxisUnit(curDim, axisUnit);
            }
            else
            {
                newScale = 1.0;
                dObjOut->setAxisUnit(curDim, "");
            }
            dObjOut->setAxisScale(curDim, newScale);
            dObjOut->setAxisOffset(curDim, 0.0);
        }
    }    

    return retval;
}

#if 0   // these filteres are not included in the public release
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs forward dft by means of fftw row by row (# rows >=1) real to complex or complex to real
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in, 
                                                        [1]complex Data object out 
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)     
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
ito::RetVal FFTWFilters::realFFTW(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    
    ito::DataObject *dObj_in = reinterpret_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());    //Input object
    ito::DataObject *dObj_out = reinterpret_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());    //Output object
    char *plan_select_char = (*paramsOpt)[0].getVal<char*>();                                            //plan selection string for fftw
    std::string plan_select = static_cast<std::string>(plan_select_char);
    long int dimensions=0;
    unsigned int plan_sel =0;
    enum Tp_sel {real_type, complex_type};
    Tp_sel Tp_select_in;
    Tp_sel Tp_select_out;

    if (!dObj_in)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObj_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }
    
    int size;

    if (ito::dObjHelper::isCplxType(dObj_in->getType(), &size))
    {
        Tp_select_in = complex_type;
    }
    else
    {
        Tp_select_in = real_type;
    }

    if (ito::dObjHelper::isCplxType(dObj_out->getType(), &size))
    {
        Tp_select_out = complex_type;
    }
    else
    {
        Tp_select_out = real_type;
    }

    if (Tp_select_in == Tp_select_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: one object must be real, the other complex").toLatin1().data());
    }

    if ((plan_select == "measure") || (plan_select == "Measure"))
    {
        plan_sel = (0U);                            // from fftw.h: #define FFTW_MEASURE (0U), #define FFTW_ESTIMATE (1U << 6)
    }
    else 
    {
        plan_sel = (1U << 6);                        // estimate as standard, quicker planning, probably slower fft-computing
    }

    free(plan_select_char);
    plan_select_char = NULL;
    

    dimensions = (*dObj_in).getDims();
    ito::int32 n0= static_cast<ito::int32>(dObj_in->getSize(0));
    ito::int32 n1= static_cast<ito::int32>(dObj_in->getSize(1));


    //real to complex
    //########################
    if (Tp_select_in == real_type)
    {
        if ((int)dObj_out->getSize(1) < (floor(n1/2.)+1) || ((ito::int32)dObj_out->getSize(0) != n0))
        {
            return ito::RetVal(ito::retError, 0, tr("Error: size of output object for r2c does not fit").toLatin1().data());
        }

        if (dimensions == 2)                    // cvMat always have at least 2 dimensions
        {
            ito::float64 *in= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * n1);

            // 1D real to complex
            //#########################
            if (n0 == 1)
            {
                fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * (floor(n1/2.)+1));
                    
                // calculate plan before initialising in! some keywords destroy in while planning
                fftw_plan plan = fftw_plan_dft_r2c_1d(n1, in, out, plan_sel);
                        
                switch (dObj_in->getType())        //cast all real types to float64 for use of libfftw3-3.dll
                {
                    case ito::tInt8:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::int8>(0,i));
                        }
                       break;
                    case ito::tUInt8:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::uint8>(0,i));
                        }
                       break;
                    case ito::tInt16:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::int16>(0,i));
                        }
                       break;
                    case ito::tUInt16:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::uint16>(0,i));
                        }
                       break;
                    case ito::tInt32:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::int32>(0,i));
                        }
                       break;
                    case ito::tFloat32:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    static_cast<ito::float64>((*dObj_in).at<ito::float32>(0,i));
                        }
                       break;
                    case ito::tFloat64:
                        for (int i=0; i<n1; i++)
                        {
                            in[i]=    (*dObj_in).at<ito::float64>(0,i);
                        }
                       break;
                }
                    
                fftw_execute(plan);
                    
                switch (dObj_out->getType())
                {
                case ito::tComplex64:
                    for (int i=0; i<(floor(n1/2.)+1); i++)
                    {
                        ito::complex64 c = std::complex<ito::float32>(out[i][0], out[i][1]);
                        ((*dObj_out).at<ito::complex64>(0,i)) = c;
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<(floor(n1/2.)+1); i++)
                    {
                        ito::complex128 c = std::complex<ito::float64>(out[i][0], out[i][1]);
                        ((*dObj_out).at<ito::complex128>(0,i)) = c;
                    }
                break;
                }


                fftw_destroy_plan(plan);
                fftw_free(out);
            }        //end of 1D 


            // 2D real to complex row by row
            //##############################
            else
            {
                fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);
                // calculate plan before initialising _in_! some keywords destroy _in_ while planning
                int _n1[]={n1};
                fftw_plan plan = fftw_plan_many_dft_r2c(1, _n1, n0, in, NULL, 1, n1, out, NULL, 1, n1, plan_sel);
                        
                switch (dObj_in->getType())        //cast all real types to float64 for use of libfftw3-3.dll
                {
                    case ito::tInt8:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int8>(j,i));
                            }
                        }
                       break;
                    case ito::tUInt8:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint8>(j,i));
                            }
                        }
                       break;
                    case ito::tInt16:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int16>(j,i));
                            }
                        }
                       break;
                    case ito::tUInt16:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint16>(j,i));
                            }
                        }
                       break;
                    case ito::tInt32:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int32>(j,i));
                            }
                        }
                       break;
                    case ito::tFloat32:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::float32>(j,i));
                            }
                        }
                       break;
                    case ito::tFloat64:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                in[i*n1+j] = (*dObj_in).at<ito::float64>(j,i);
                            }
                        }
                       break;
                }
                

                fftw_execute(plan);
                
                switch (dObj_out->getType())
                {
                    case ito::tComplex64:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                ito::complex64 c = std::complex<ito::float32>(static_cast<ito::float32>(out[i*n1+j][0]), static_cast<ito::float32>(out[i*n1+j][1]));
                                ((*dObj_out).at<ito::complex64>(i,j)) = c;
                            }
                        }
                    break;
                    case ito::tComplex128:
                        for (int i=0; i<n0; i++)
                        {
                            for (int j=0; j<n1; j++)
                            {
                                ito::complex128 c = std::complex<ito::float64>(out[i*n1+j][0], out[i*n1+j][1]);
                                ((*dObj_out).at<ito::complex128>(i,j)) = c;
                            }
                        }
                    break;
                }
                
                fftw_destroy_plan(plan);
                fftw_free(out);
            }        // end of 2D row by row
        }            // end of all 1D and 2D

        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: more than 2 dimensions are not supported").toLatin1().data());
        }
    
        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter forward real to complex (unscaled!)");
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter forward real to complex (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }    
    }                // end of real to complex


    //complex to real
    //#################
    else            // Tp_select != real_type
    {
        if (dObj_out->getType() != ito::tFloat64)
        {
            return ito::RetVal(ito::retError, 0, tr("Error: need float64 output for complex input and c2r mode").toLatin1().data());
        }
        
        fftw_complex *in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);

        if (dimensions == 2)
        {
            // 1D complex to real
            //######################
            if (n0 == 1)
            {
                ito::float64 *out= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * ((n1-1)*2));
                    
                // calculate plan before initialising in! some keywords destroy in while planning
                fftw_plan plan = fftw_plan_dft_c2r_1d(n1, in, out, plan_sel);
                        
                switch (dObj_in->getType())    
                {
                case ito::tComplex64:
                    for (int i=0; i<n1; i++)
                    {
                        ito::complex64 x = (*dObj_in).at<ito::complex64>(0,i);
                        in[i][0] = static_cast<ito::float64>(real(x));
                        in[i][1] = static_cast<ito::float64>(imag(x));
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<n1; i++)
                    {
                        ito::complex128 x = (*dObj_in).at<ito::complex128>(0,i);
                        in[i][0] = real(x);
                        in[i][1] = imag(x);
                    }
                break;
                }

                fftw_execute(plan);
                fftw_destroy_plan(plan);
                for (int i=0; i<((n1-1)*2); i++)
                    {
                        ((*dObj_out).at<ito::float64>(0,i)) = out[i];
                    }

                fftw_free(out);
            }        // end of 1D

            // 2D complex to real row by row
            //######################################
            else
            {
                ito::float64 *out= (ito::float64*)fftw_malloc(sizeof(ito::float64) *n0 * ((n1-1)*2));
                // calculate plan before initialising _in_! some keywords destroy _in_ while planning
                int _n1[]={n1};
                fftw_plan plan = fftw_plan_many_dft_c2r(1, _n1, n0, in, NULL, 1, n1, out, NULL, 1, n1, plan_sel);
                        
                switch (dObj_in->getType())    
                {
                case ito::tComplex64:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex64 x = (*dObj_in).at<ito::complex64>(i,j);
                            in[i*n1+j][0] = real(x);
                            in[i*n1+j][1] = imag(x);
                        }
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex128 x = (*dObj_in).at<ito::complex128>(i,j);
                            in[i*n1+j][0] = real(x);
                            in[i*n1+j][1] = imag(x);
                        }
                    }
                break;
                }
                
                fftw_execute(plan);
                
                for (int i=0; i<n0; i++)
                {
                    for (int j=0; j<n1; j++)
                    {
                        ((*dObj_out).at<ito::float64>(i,j)) = out[i*n1+j];
                    }
                }
                
                fftw_destroy_plan(plan);
                fftw_free(out);
            }  // end of 2D row by row
                    
            fftw_free(in);
        }                            // end of all 1D and 2D
            
        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: more than 2 dimensions are not supported").toLatin1().data());
        }
    
        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter forward complex to real (unscaled!)");  
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter forward complex to real (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }
    }
    
    dObj_in->copyTagMapTo(*dObj_out);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail    Performs 2D dft real to complex or complex to real by means of fftw 
   \param[in|out]   paramsMand  Mandatory parameters:   [0]complex Data object in, 
                                                        [1]complex Data object out 
   \param[in|out]   paramsOpt   Optional parameters:    [0]Plannerstring (estimate or other, cf. fftw-doc.)
   \param[out]      outVals   Outputvalues:                None
   \author ITO, Boettcher
   \date 2012.03.06
*/
ito::RetVal FFTWFilters::realFFTW2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    
    ito::DataObject *dObj_in = reinterpret_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());    //Input object
    ito::DataObject *dObj_out = reinterpret_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());    //Output object
    char *plan_select_char = (*paramsOpt)[0].getVal<char*>();                                            //plan selection string for fftw
    std::string plan_select = static_cast<std::string>(plan_select_char);
    long int dimensions=0;
    unsigned int plan_sel =0;
    enum Tp_sel {real_type, complex_type};
    Tp_sel Tp_select_in;
    Tp_sel Tp_select_out;

    if (!dObj_in)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObj_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    int size;
    if (ito::dObjHelper::isCplxType(dObj_in->getType(), &size))
    {
        Tp_select_in = complex_type;
    }
    else
    {
        Tp_select_in = real_type;
    }

    if (ito::dObjHelper::isCplxType(dObj_out->getType(), &size))
    {
        Tp_select_out = complex_type;
    }
    else
    {
        Tp_select_out = real_type;
    }

    if (Tp_select_in == Tp_select_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: one object must be real, the other complex").toLatin1().data());
    }

    if ((plan_select == "measure") || (plan_select == "Measure"))
    {
        plan_sel = (0U);                            // from fftw.h: #define FFTW_MEASURE (0U), #define FFTW_ESTIMATE (1U << 6)
    }
    else 
    {
        plan_sel = (1U << 6);                        // estimate as standard, quicker planning, probably slower fft-computing
    }

    free(plan_select_char);
    plan_select_char = NULL;
    
    dimensions = (*dObj_in).getDims();
    ito::int32 n0= static_cast<ito::int32>(dObj_in->getSize(0));
    ito::int32 n1= static_cast<ito::int32>(dObj_in->getSize(1));

    //real to complex
    //########################
    if (Tp_select_in == real_type)
    {
        if (dObj_out->getSize(1) < (floor(n1/2.)+1) || ((ito::int32)dObj_out->getSize(0) != n0))
        {
            return ito::RetVal(ito::retError, 0, tr("Error: size of output object for r2c does not fit").toLatin1().data());
        }

        if ((dimensions == 2) && (n0!=1))
        {
            ito::float64 *in= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * n1);
            fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);
                    
            // calculate plan before initialising _in_! some keywords destroy _in_ while planning
            fftw_plan plan = fftw_plan_dft_r2c_2d(n0, n1, in, out, plan_sel);
                        
            switch (dObj_in->getType())        //cast all real types to float64 for use of libfftw3-3.dll
            {
                case ito::tInt8:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int8>(j,i));
                        }
                    }
                   break;
                case ito::tUInt8:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint8>(j,i));
                        }
                    }
                   break;
                case ito::tInt16:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int16>(j,i));
                        }
                    }
                   break;
                case ito::tUInt16:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::uint16>(j,i));
                        }
                    }
                   break;
                case ito::tInt32:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::int32>(j,i));
                        }
                    }
                   break;
                case ito::tFloat32:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = static_cast<ito::float64>((*dObj_in).at<ito::float32>(j,i));
                        }
                    }
                   break;
                case ito::tFloat64:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            in[i*n1+j] = (*dObj_in).at<ito::float64>(j,i);
                        }
                    }
                   break;
            }

            fftw_execute(plan);
                
            switch (dObj_out->getType())
            {
                case ito::tComplex64:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex64 c = std::complex<ito::float32>(static_cast<ito::float32>(out[i*n1+j][0]), static_cast<ito::float32>(out[i*n1+j][1]));
                            ((*dObj_out).at<ito::complex64>(i,j)) = c;
                        }
                    }
                break;
                case ito::tComplex128:
                    for (int i=0; i<n0; i++)
                    {
                        for (int j=0; j<n1; j++)
                        {
                            ito::complex128 c = std::complex<ito::float64>(out[i*n1+j][0], out[i*n1+j][1]);
                            ((*dObj_out).at<ito::complex128>(i,j)) = c;
                        }
                    }
                break;
            }
                
            fftw_destroy_plan(plan);
            fftw_free(out);
            fftw_free(in);
        }        //end of 2D complete
        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: this filter is designed for 2D data. As the name says.....").toLatin1().data());
        }
    
        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter 2D real to complex (unscaled!)");
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter 2D real to complex (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }    
    }            //end of real to complex
    
    //complex to real
    //#################
    else            // Tp_select != real_type
    {
        if (dObj_out->getType() != ito::tFloat64)
        {
            return ito::RetVal(ito::retError, 0, tr("Error: need float64 output for complex input and c2r mode").toLatin1().data());
        }
        
        fftw_complex *in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * n0 * n1);    

        if ((dimensions == 2) && (n0!=1))
        {
            ito::float64 *out= (ito::float64*)fftw_malloc(sizeof(ito::float64) * n0 * n1);
                    
            // calculate plan before initialising in! some keywords destroy in while planning
            fftw_plan plan = fftw_plan_dft_c2r_2d(n0, n1, in, out, plan_sel);
                        
            switch (dObj_in->getType())
            {
            case ito::tComplex64:
                for (int i=0; i<n0; i++)
                {
                    for (int j=0; j<n1; j++)
                    {
                        ito::complex64 x = (*dObj_in).at<ito::complex64>(i,j);
                        in[i*n1+j][0] = real(x);
                        in[i*n1+j][1] = imag(x);
                    }
                }
            break;
            case ito::tComplex128:
                for (int i=0; i<n0; i++)
                {
                    for (int j=0; j<n1; j++)
                    {
                        ito::complex128 x = (*dObj_in).at<ito::complex128>(i,j);
                        in[i*n1+j][0] = real(x);
                        in[i*n1+j][1] = imag(x);
                    }
                }
            break;
            }

            fftw_execute(plan);
            fftw_destroy_plan(plan);
            for (int i=0; i<n0; i++)
            {
                for (int j=0; j<n1; j++)
                {
                    ((*dObj_out).at<ito::float64>(i,j)) = out[i*n1+j];
                }
            }
            fftw_free(out);
            fftw_free(in);
        }                    // end of 2D standard

        else
        {
            retval = ito::RetVal(ito::retError, 0, tr("Error: this filter is designed for 2D data. As the name says.....").toLatin1().data());
        }
    
        if (!retval.containsError())
        {
            // Add Protokoll
//            char prot[81] = {0};
//            _snprintf(prot, 80, "FFTW filter 2D complex to real (unscaled!)");  
//            dObj_out->addToProtocol(std::string(prot));
            QString msg = tr("FFTW filter 2D complex to real (unscaled!)");
            dObj_out->addToProtocol(std::string(msg.toLatin1().data()));
        }
    }
    
    dObj_in->copyTagMapTo(*dObj_out);    

    return retval;
}

//##############################################################################################################################
//NEW IMPLEMENTATION -- NOT TESTED COMPLETELY
//last modified: 2012/02/20 boettcher
//Known bugs:
//    - flanks to flat? --> use more steps?
//##############################################################################################################################

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail

   \param[in|out]   paramsMand  Mandatory parameters:   Data object in (1 row of height data),
                                                        Data object out (1 row of roughness data)
   \param[in|out]   paramsOpt   Optional parameters:    R_z im mm(lookup table will do the job;
                                                            if 0, at least 1 pair of lambdas must be defined or getR_z flag == 1)
                                                        n: steps for FFT (default 0 ==> #pixels in data object),
                                                        lambda_s,
                                                        lambda_c,
                                                        lambda_f,
                                                        Data object for waviness (default O, if !=0, roughness AND waviness are calculated)

   \param[out]      outVals   Outputvalues

   \author ITO, Boettcher
   \sa
   \date 02.2012
*/
ito::RetVal FFTWFilters::calcGaussianFilterRough1D (QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObj_in = reinterpret_cast<ito::DataObject*>((*paramsMand)[0].getVal<void*>());  //Input object

    if (dObj_in == NULL)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source object not defined").toLatin1().data());
    }
    
    if (dObj_in->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("DataObject is empty").toLatin1().data());
    }

    ito::int32 sizeY = dObj_in->getSize(0);
    ito::int32 sizeX = dObj_in->getSize(1);

    if ((dObj_in->getDims() > 2) || (sizeY > 1))    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Input DataObject must be 1xN").toLatin1().data());
    }

    ito::DataObject *dObj_out = reinterpret_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());  //Output object
    if (dObj_out == NULL)    // same here for output object
    {
        return ito::RetVal(ito::retError, 0, tr("Output object not defined").toLatin1().data());
    }

    bool useDstObj = true;
    bool useWaveObj = true;
    bool calcWaveObj = false;

    ito::DataObject *dObj_wavOut = reinterpret_cast<ito::DataObject*>((*paramsOpt)[4].getVal<void*>());  //Output object
    if (dObj_wavOut == NULL)    // same here for output object
    {
        calcWaveObj = false;
    }
    else if (dObj_wavOut == dObj_out)
    {
        return ito::RetVal(ito::retError, 0, tr("Waviness-result must not be equal to roughness output").toLatin1().data());
    }
    else if (dObj_wavOut == dObj_in)
    {
        return ito::RetVal(ito::retError, 0, tr("Waviness-result must not be equal to surface input").toLatin1().data());
    }
    else
    {
        calcWaveObj = true;
    }

    // Check if input and output type are allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObj_in, "dObjIn", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
    {
        return retval;
    }
    ito::DataObject rough_out;
    ito::DataObject wavin_out;

    if (dObj_out == dObj_in)
    {
        if (dObj_out->getType() == ito::tFloat64)
        {
            // inplace is possilbe
            rough_out = *dObj_out;
        }
        else
        {
            // input must be changed to ito::float64 during filtering
            useDstObj = false;
            rough_out = ito::DataObject(sizeY, sizeX, ito::tFloat64);            
        }
    }
    else
    {
        ito::RetVal temp = ito::dObjHelper::verify2DDataObject(dObj_out, "dObjectOut", sizeY, sizeY, sizeX, sizeX, 1, ito::tFloat64);
        if (!temp.containsWarningOrError())
        {
            rough_out = *dObj_out;
        }
        else
        {
            // output must be changed to ito::float64 or right size
            useDstObj = false;
            rough_out = ito::DataObject(sizeY, sizeX, ito::tFloat64);        
        }
    }

    if (calcWaveObj)
    {
        ito::RetVal temp = ito::dObjHelper::verify2DDataObject(dObj_wavOut, "dObjectWave", sizeY, sizeY, sizeX, sizeX, 1, ito::tFloat64);
        if (!temp.containsWarningOrError())
        {
            wavin_out = *dObj_wavOut;
        }
        else
        {
            // output must be changed to ito::float64 or right size
            useWaveObj = false;
            wavin_out = ito::DataObject(sizeY, sizeX, ito::tFloat64);        
        }
    }

    // Check if filterborders or R_z are set
    ito::float64 R_z = static_cast<ito::float64>((*paramsOpt)[0].getVal<ito::float64>());
    ito::float64 lambda_s = static_cast<ito::float64>((*paramsOpt)[1].getVal<ito::float64>());
    ito::float64 lambda_c = static_cast<ito::float64>((*paramsOpt)[2].getVal<ito::float64>());
    ito::float64 lambda_f = static_cast<ito::float64>((*paramsOpt)[3].getVal<ito::float64>());

    if (!ito::dObjHelper::isNotZero(R_z) && !ito::dObjHelper::isNotZero(lambda_c))
    {
        return ito::RetVal(ito::retError, 0, tr("Define R_z or 1 pair of lambdas").toLatin1().data());
    }
    else if (ito::dObjHelper::isNotZero(R_z)) // search lookuptable for adequate lambdas (in mm!!)
    {
        if (R_z <= 0.1e-3)
        {
                lambda_c = 0.08;
                lambda_s = 2.5e-3;
        }
        else if (R_z <= 0.5e-3)
        {
                lambda_c = 0.25;
                lambda_s = 2.5e-3;
        }
        else if (R_z <= 10.e-3)
        {
                lambda_c = 0.8;
                lambda_s = 2.5e-3;
        }
        else if  (R_z <= 50.e-3)
        {
                lambda_c = 2.5;
                lambda_s = 8e-3;
        }
        else if (R_z <= 200.e-3)
        {
                lambda_c = 8.0;
                lambda_s = 25e-3;
        }
        else if (R_z >= 200.e-3)
        {
            return ito::RetVal(ito::retError, 0, tr("R_z over 200 \u00B5m").toLatin1().data());  // mu m
        }
    }
    
    cv::Mat *scrMat = ((cv::Mat *)dObj_in->get_mdata()[dObj_in->seekMat(0)]);
    ito::float64 *dstPtr = ((cv::Mat *)rough_out.get_mdata()[rough_out.seekMat(0)])->ptr<ito::float64>();
    ito::float64 *wavPtr = NULL;
    
    if (calcWaveObj)
    {
        wavPtr = ((cv::Mat *)wavin_out.get_mdata()[wavin_out.seekMat(0)])->ptr<ito::float64>();
    }

    // Allocade buffer for input and output
    ito::int32 sizeXFD = sizeX / 2 + 1;

    ito::float64 *srcInput   = (ito::float64*) fftw_malloc(sizeof(ito::float64) * sizeX);
    fftw_complex *fourDomain = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sizeXFD);
    fftw_complex *filtRough  = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sizeXFD);
    fftw_complex *filtWavi   = NULL;
    
    fftw_plan plan2FD       = fftw_plan_dft_r2c_1d(sizeX, srcInput, fourDomain, FFTW_ESTIMATE);
    fftw_plan planBackRough = fftw_plan_dft_c2r_1d(sizeX, filtRough, srcInput, FFTW_ESTIMATE);
    fftw_plan planBackWavi  = NULL;
    
    if (calcWaveObj)
    {
        filtWavi     = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * sizeXFD);
        planBackWavi = fftw_plan_dft_c2r_1d(sizeX, filtWavi,  srcInput, FFTW_ESTIMATE);
    }

    ito::float64 dx = dObj_in->getAxisScale(1);

    switch(dObj_in->getType())
    {
        case ito::tUInt8:
            ito::dObjHelper::GetHLineD<ito::uint8>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tInt8:
            ito::dObjHelper::GetHLineD<ito::int8>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tInt16:
            ito::dObjHelper::GetHLineD<ito::int16>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tUInt16:
            ito::dObjHelper::GetHLineD<ito::uint16>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tInt32:
            ito::dObjHelper::GetHLineD<ito::int32>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tFloat32:
            ito::dObjHelper::GetHLineD<ito::float32>(scrMat, 0, 0, sizeX, srcInput);
            break;
        case ito::tFloat64:
            ito::dObjHelper::GetHLineD<ito::float64>(scrMat, 0, 0, sizeX, srcInput);
            break;
    }

    fftw_execute(plan2FD);

    //Roughness Filtering
    //filtRough[0][0] = fourDomain[0][0];    //real fourier component (amplitude)

    //const ito::float64 cAlpha =  0.8493218002880191;// 1 / sqrt(2 * ln(2));
    //const ito::float64 cAlpha =  1.2011224087864498;// 1 / sqrt(2 * ln(sqrt(2)));
    const ito::float64 cAlpha =  1.48809737131601248;// 1 / sqrt(2 * ln(sqrt(pi/2)));

    ito::float64 sigmaFS = cAlpha / lambda_s;
    ito::float64 sigmaFC = cAlpha / lambda_c;
    ito::float64 sigmaFF = cAlpha / lambda_f;
    ito::float64 freqInc = 1 / (dx * ((ito::float64)sizeXFD - 1.0));

    ito::DataObject filterFunc = ito::DataObject(2, sizeXFD, ito::tFloat64);
    ito::float64* filterFuncRoughPtr = (ito::float64*)filterFunc.rowPtr(0,0);
    ito::float64* filterFuncWavePtr = (ito::float64*)filterFunc.rowPtr(0,1);

    filterFuncRoughPtr[0] = 0.0;
    filterFuncWavePtr[0] = 0.0;

    ito::DataObject *filterOut = reinterpret_cast<ito::DataObject*>((*paramsOpt)[5].getVal<void*>());  //Output object
    if (filterOut != NULL && filterOut != dObj_wavOut && filterOut != dObj_out && filterOut != dObj_in)    // same here for output object
    {
        *filterOut = filterFunc;
        filterFunc.setAxisScale(1, freqInc);
    }

    if (ito::dObjHelper::isNotZero(lambda_s))
    {
        ito::float64 cuExp1 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFS, 2));
        ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
        for (ito::int32 i = 0; i < sizeXFD; i++)
        {
            ito::float64 sqrI = pow((ito::float64)i, 2);
            filterFuncRoughPtr[i] = exp(cuExp1 * sqrI) * (1-exp(cuExp2 * sqrI));
        }
    }
    else
    {
        ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
        for (ito::int32 i = 0; i < sizeXFD; i++)
        {
            filterFuncRoughPtr[i] = 1-exp(cuExp2 * pow((ito::float64)i, 2));
        }    
    }

    for (ito::int32 i = 0; i < sizeXFD; i++)
    {
        filtRough[i][0] = fourDomain[i][0] * filterFuncRoughPtr[i];
        filtRough[i][1] = fourDomain[i][1] * filterFuncRoughPtr[i];
    }    

    //Waviness Filterung
    if (calcWaveObj)
    {
        //filtWavi[0][0] = fourDomain[0][0];    //real fourier component (amplitude)

        if (ito::dObjHelper::isNotZero(lambda_f))
        {
            ito::float64 cuExp1 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
            ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFF, 2));
            for (ito::int32 i = 0; i < sizeXFD; i++)
            {
                ito::float64 sqrI = pow((ito::float64)i, 2);
                filterFuncWavePtr[i] = exp(cuExp1 * sqrI) * (1-exp(cuExp2 * sqrI));
            }
        }
        else
        {
            ito::float64 cuExp2 = -1.0 * pow(freqInc, 2) / (2*pow(sigmaFC, 2));
            for (ito::int32 i = 0; i < sizeXFD; i++)
            {
                filterFuncWavePtr[i] = exp(cuExp2 * pow((ito::float64)i, 2));
            }    
        }

        for (ito::int32 i = 0; i < sizeXFD; i++)
        {
            filtWavi[i][0] = fourDomain[i][0] * filterFuncWavePtr[i];
            filtWavi[i][1] = fourDomain[i][1] * filterFuncWavePtr[i];
        } 

        fftw_execute(planBackWavi);  //destroys filtered?!

        for (ito::int32 i=0; i< sizeX ;i++)
        {
            wavPtr[i] = srcInput[i] / sizeX;
        }
    }
    fftw_execute(planBackRough);  //destroys filtered?!

    for (ito::int32 i=0; i< sizeX ;i++)
    {
        dstPtr[i] = srcInput[i] / sizeX;
    }

    ito::DataObject *fourierOut = reinterpret_cast<ito::DataObject*>((*paramsOpt)[6].getVal<void*>());  //Output object
    if (filterOut != NULL && filterOut != dObj_wavOut && filterOut != dObj_out && filterOut != dObj_in)    // same here for output object
    {
        *fourierOut = ito::DataObject(1, sizeXFD, ito::tComplex128);
        ito::complex128* cRowPtr = (ito::complex128*)fourierOut->rowPtr(0,0);
        
        memcpy(cRowPtr, fourDomain, sizeof(fourDomain)* sizeXFD);
    }

    //destroy fft stuff
    fftw_destroy_plan(plan2FD);
    fftw_destroy_plan(planBackRough);

    if (planBackWavi)
    {
        fftw_destroy_plan(planBackWavi);
    }

    if (srcInput)
    {
        fftw_free(srcInput);
    }

    if (filtRough)
    {
        fftw_free(filtRough);
    }

    if (filtWavi)
    {
        fftw_free(filtWavi);
    }

    if (fourDomain)
    {
        fftw_free(fourDomain);
    }

    if (!retval.containsError())
    {
        dObj_in->copyTagMapTo(rough_out);
        ito::dObjHelper::dObjCopyLastNAxisTags(*dObj_in, rough_out, 2, true, true);

        QString msg = tr("Roughness after gaussian filter with lambdaS %1 mm and lambdaC %2").arg(QString::number(lambda_s), QString::number(lambda_c));
        rough_out.addToProtocol(std::string(msg.toLatin1().data()));
        if (useDstObj == false)
        {
            *dObj_out = rough_out;
        }
    }

    if (!retval.containsError() && calcWaveObj)
    {
        dObj_in->copyTagMapTo(wavin_out);
        ito::dObjHelper::dObjCopyLastNAxisTags(*dObj_in, wavin_out, 2, true, true);

        QString msg = tr("Waviness after gaussian filter with lambdaC %1 mm and lambdaF %2").arg(QString::number(lambda_c), QString::number(lambda_f));
        wavin_out.addToProtocol(std::string(msg.toLatin1().data()));
        if (useWaveObj == false)
        {
            *dObj_wavOut = wavin_out;
        }
    }
    
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail Get the mandatory parameters "DataObject",
   \param[out]   paramsMand  Mandatory parameters for the filter function
   \param[out]   paramsOpt   Optinal parameters for the filter function :
   \author ITO, Boettcher
   \sa  mcppfilters::calcGaussianFilterRough1D
   \date
*/
ito::RetVal FFTWFilters::getGaussianRough1DParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::Param param;

    if (!paramsMand)
    {
        retval = ito::RetVal(ito::retError, 0, tr("uninitialized vector for mandatory parameters!").toLatin1().data());
        goto end;
    }
    if (!paramsOpt)
    {
        retval = ito::RetVal(ito::retError, 0, tr("uninitialized vector for optional parameters!").toLatin1().data());
        goto end;
    }

    param = ito::Param("DataObject_in", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("DataObject_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsMand->append(param);

    param = ito::Param("R_z", ito::ParamBase::Double, 0., 0.2, 0., "R_z");
    paramsOpt->append(param);
    param = ito::Param("lambda_s", ito::ParamBase::Double, 0.0, std::numeric_limits<double>::max(), 0.0, tr("Short wavelength to filter").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("lambda_c", ito::ParamBase::Double, 0.0, std::numeric_limits<double>::max(), 0.0, tr("Wavelength to seperate between roughness and waviness").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("lambda_f", ito::ParamBase::Double, 0.0, std::numeric_limits<double>::max(), 0.0, tr("Wavelength to seperate between waviness and form").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("DataObject_waviness_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("DataObject_filterFunc_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("DataObject_fourier_out", ito::ParamBase::DObjPtr, NULL, tr("see Algorithm-Doc").toLatin1().data());
    paramsOpt->append(param);
end:
    return retval;
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFilters::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    if (sizeof(fftw_complex) == sizeof(ito::complex128))
    {
        filter = new FilterDef(FFTWFilters::fftw1d, FFTWFilters::ParamsFFTW, fftw1dDOC);
        m_filterList.insert("fftw", filter);
        filter = new FilterDef(FFTWFilters::ifftw1d, FFTWFilters::ParamsFFTW, ifftw1dDOC);
        m_filterList.insert("ifftw", filter);
        filter = new FilterDef(FFTWFilters::fftw2d, FFTWFilters::ParamsFFTW, fftw2dDOC);
        m_filterList.insert("fftw2D", filter);
        filter = new FilterDef(FFTWFilters::ifftw2d, FFTWFilters::ParamsFFTW, ifftw2dDOC);
        m_filterList.insert("ifftw2D", filter);
    }
    else
    {
        retval += ito::RetVal(ito::retWarning, 0, tr("Warning: compatibility error between fftw_complex and ito::complex128").toLatin1().data());
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FFTWFilters::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}



