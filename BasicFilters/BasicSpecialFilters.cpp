/* ********************************************************************
    Plugin "BasicFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

/*! \file itomspecialfilter.cpp
   \brief   This file contains special filter.

   \author ITO
   \date 01.2012
*/

#include <math.h>

#include "BasicFilters.h"

#include "DataObject/dataObjectFuncs.h"
#include "common/sharedStructuresPrimitives.h"

//#include "common/helperCommon.h"

extern int NTHREADS;

template<typename _Tp> inline _Tp swapByte(const _Tp val)
{
    return 0;
}

template<> inline ito::uint16 swapByte<ito::uint16>(const ito::uint16 val)
{
    return ((val & 0xFF00) >> 8) | ((val & 0x00FF) << 8);
}

template<> inline ito::int16 swapByte<ito::int16>(const ito::int16 val)
{
    return ((val & 0xFF00) >> 8) | ((val & 0x00FF) << 8);
}

template<> inline ito::uint32 swapByte<ito::uint32>(const ito::uint32 val)
{
    return ((val & 0xFF000000) >> 24) | ((val & 0x00FF0000) >> 8) | ((val & 0x0000FF00) << 8) | ((val & 0x000000FF) << 24);
}

template<> inline ito::int32 swapByte<ito::int32>(const ito::int32 val)
{
    return ((val & 0xFF000000) >> 24) | ((val & 0x00FF0000) >> 8) | ((val & 0x0000FF00) << 8) | ((val & 0x000000FF) << 24);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail copy the rows from different z-planes of a multi-plane single row object to a single plane 2D-Object
   \param[in]   dObjSrc     DataObject with shape Z x 1 x X
   \param[out]  dObjDst     DataObject with shape (Z=Y) x X
   \author ITO
   \date
*/
template<typename _Tp> ito::RetVal copyRows(ito::DataObject* dObjSrc, ito::DataObject* dObjDst)
{
    _Tp* rowPtrIn;
    _Tp* rowPtrOut;
    
//    unsigned long x, y;

    
    cv::Mat *matOut =  ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(0)]);

    unsigned long zsize = dObjSrc->getSize(0);
//    unsigned long ysize = dObjDst->getSize(0);
    unsigned long xsize = dObjDst->getSize(1);

    for(unsigned long z = 0; z < zsize; z++)
    {
        cv::Mat *matIn =  ((cv::Mat *)dObjSrc->get_mdata()[dObjSrc->seekMat(z)]);
        rowPtrIn = (_Tp*)matIn->ptr(0);
        rowPtrOut = (_Tp*)matOut->ptr(z);
        memcpy(rowPtrOut, rowPtrIn, xsize * sizeof(_Tp));
    }

    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail copy the columns from different z-planes of a multi-plane single column object to a single plane 2D-Object
   \param[in]   dObjSrc     DataObject with shape Z x Y x 1 
   \param[out]  dObjDst     DataObject with shape Y x (Z=X) 
   \author ITO
   \date 
*/
template<typename _Tp> ito::RetVal copyCols(ito::DataObject* dObjSrc, ito::DataObject* dObjDst)
{
   
//    unsigned long x, y;

    
    cv::Mat *matOut =  ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(0)]);

    unsigned long zsize = dObjSrc->getSize(0);
    unsigned long ysize = dObjDst->getSize(0);
//    unsigned long xsize = dObjDst->getSize(1);

    for(unsigned long z = 0; z < zsize; z++)
    {
        cv::Mat *matIn =  ((cv::Mat *)dObjSrc->get_mdata()[dObjSrc->seekMat(z)]);
        
        for(unsigned long y = 0; y < ysize; y++)
        {
            matOut->at<_Tp>(y,z) = matIn->at<_Tp>(y,0);
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in]   
   \param[out]   
   \author ITO
   \date 
*/
template<typename _Tp> ito::RetVal swapBits(ito::DataObject* dObjSrc, ito::DataObject* dObjDst)
{

    cv::Mat_<_Tp> *matOut =  NULL;
    cv::Mat_<_Tp> *matIn  =  NULL;

    _Tp* rowPtrIn;
    _Tp* rowPtrOut;

    //unsigned long zsize = dObjSrc->getSize(0);
    unsigned long ysize = dObjSrc->getSize(0);
    unsigned long xsize = dObjSrc->getSize(1);

    for(unsigned long z = 0; z < 1; z++)
    {
        matIn =  ((cv::Mat_<_Tp> *)dObjSrc->get_mdata()[dObjSrc->seekMat(z)]);
        matOut =  ((cv::Mat_<_Tp> *)dObjDst->get_mdata()[dObjSrc->seekMat(z)]);
        
        for(unsigned long y = 0; y < ysize; y++)
        {
            rowPtrIn = (_Tp*)matIn->ptr(y);
            rowPtrOut = (_Tp*)matOut->ptr(y);

            for(unsigned long x = 0; x < xsize; x++)
            {
                rowPtrOut[x] = swapByte(rowPtrIn[x]);
            }
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date 02.2012
*/
ito::RetVal BasicFilters::flaten3Dto2D(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    bool needNewObj = false;
    bool keepX = true;
    int xsize = 1;
    int ysize = 1;

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjSrc->getDims()  != 3 )
    { 
        return ito::RetVal(ito::retError, 0, tr("Error: Input image must be 3D").toLatin1().data());
    }

    if (dObjSrc->getSize(1) != 1 && dObjSrc->getSize(2) != 1) 
    { 
        return ito::RetVal(ito::retError, 0, tr("Error: one dimension of input image must be equal to 1").toLatin1().data());
    }
    else if(dObjSrc->getSize(1) != 1)
    {
        keepX = false;
    }

    if (dObjSrc->getDims()  != 2 )
    {
        needNewObj = true;
        if(keepX)
        {
            xsize = dObjSrc->getSize(2);
            ysize = dObjSrc->getSize(0);
        }
        else
        {
            xsize = dObjSrc->getSize(0);
            ysize = dObjSrc->getSize(1);
        }
    }
    else
    {
        if(keepX)
        {
            if(dObjSrc->getSize(0) != dObjDst->getSize(0) || dObjSrc->getSize(2) != dObjDst->getSize(1))
            {
                needNewObj = true;
                xsize = dObjSrc->getSize(2);
                ysize = dObjSrc->getSize(0);
            }
        }
        else
        {
            if(dObjSrc->getSize(0) != dObjDst->getSize(1) || dObjSrc->getSize(1) != dObjDst->getSize(0))
            {
                needNewObj = true;
                xsize = dObjSrc->getSize(0);
                ysize = dObjSrc->getSize(1);
            }        
        }
    }

    ito::DataObject tempObj;

    if(needNewObj)
    {
        tempObj = ito::DataObject(ysize, xsize, dObjSrc->getType());
    }
    else
    {
        tempObj = *dObjDst;
    }

    // von hier an muss ich noch arbeiten

    switch(dObjSrc->getType())
    {
        case ito::tInt8:
            if(keepX) copyRows<ito::int8>(dObjSrc, &tempObj);
            else copyCols<ito::int8>(dObjSrc, &tempObj);
        break;
        case ito::tUInt8:
            if(keepX) copyRows<ito::uint8>(dObjSrc, &tempObj);
            else copyCols<ito::uint8>(dObjSrc, &tempObj);
        break;
        case ito::tInt16:
            if(keepX) copyRows<ito::int16>(dObjSrc, &tempObj);
            else copyCols<ito::int16>(dObjSrc, &tempObj);
        break;
        case ito::tUInt16:
            if(keepX) copyRows<ito::uint16>(dObjSrc, &tempObj);
            else copyCols<ito::uint16>(dObjSrc, &tempObj);
        break;
        case ito::tInt32:
            if(keepX) copyRows<ito::int32>(dObjSrc, &tempObj);
            else copyCols<ito::int32>(dObjSrc, &tempObj);
        break;
        case ito::tFloat32:
            if(keepX) copyRows<ito::float32>(dObjSrc, &tempObj);
            else copyCols<ito::float32>(dObjSrc, &tempObj);
        break;
        case ito::tFloat64:
            if(keepX) copyRows<ito::float32>(dObjSrc, &tempObj);
            else copyCols<ito::float32>(dObjSrc, &tempObj);
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toLatin1().data());
    }

    *dObjDst = tempObj;

    if(!retval.containsError())
    {
        // Add scale and offset for all z + (y and x)
//        char prot[81] = {0};
//        _snprintf(prot, 80, "Flattened object from 3d to 2d");
//        dObjDst->addToProtocol(std::string(prot));
        QString msg = tr("Flattened object from 3d to 2d");
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }



    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date 02.2012
*/
ito::RetVal BasicFilters::swapByteOrder(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

//    bool needNewObj = false;
//    bool keepX = true;
//    int xsize = 1;
//    int ysize = 1;

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjSrc->getDims()  != 2 )
    { 
        return ito::RetVal(ito::retError, 0, tr("Error: Input image must be 2D").toLatin1().data());
    }

    if((retval += ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 3, ito::tInt32, ito::tUInt16, ito::tInt16)).containsError())
        return retval;

    if(dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if(ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        //(*dObjDst).setT(dObjSrc->isT());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    // von hier an muss ich noch arbeiten

    switch(dObjSrc->getType())
    {
        case ito::tInt32:
            swapBits<ito::int32>(dObjSrc, dObjDst);
        break;
        case ito::tInt16:
            swapBits<ito::int16>(dObjSrc, dObjDst);
        break;
        case ito::tUInt16:
            swapBits<ito::uint16>(dObjSrc, dObjDst);
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toLatin1().data());
    }

    if(!retval.containsError())
    {
        // Add scale and offset for all z + (y and x)
//        char prot[81] = {0};
//        _snprintf(prot, 80, "Flattened object from 3d to 2d");
//        dObjDst->addToProtocol(std::string(prot));
        QString msg = tr("Swap byte order");
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::replaceInfAndNaN(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjReplace = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[2].getVal<void*>();

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjReplace)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: replace image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if(!ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjReplace))
    {
        return ito::RetVal(ito::retError, 0, tr("source and replace image must have the same type and size").toLatin1().data());
    }

    if(dObjSrc->getType() != ito::tFloat32 && dObjSrc->getType() != ito::tFloat64)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: this filter is only usable for float or double matrices").toLatin1().data());
    }

    if(dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if(ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        //(*dObjDst).setT(dObjSrc->isT());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    int numMats = dObjSrc->calcNumMats();
    int srcIdx, replaceIdx, destIdx;
    cv::Mat *srcMat = NULL;
    cv::Mat *replaceMat = NULL;
    cv::Mat *destMat = NULL;

    ito::float32 *floatLinePtr1, *floatLinePtr2, *floatLinePtr3;
    ito::float64 *doubleLinePtr1, *doubleLinePtr2, *doubleLinePtr3;
    int nrOfReplacements = 0;

    for(int z = 0; z<numMats;z++)
    {
        srcIdx = dObjSrc->seekMat(z);
        replaceIdx = dObjReplace->seekMat(z);
        destIdx = dObjDst->seekMat(z);

        srcMat = ((cv::Mat*)((dObjSrc->get_mdata())[srcIdx]));
        replaceMat = ((cv::Mat*)((dObjReplace->get_mdata())[replaceIdx]));
        destMat = ((cv::Mat*)((dObjDst->get_mdata())[destIdx]));

        if(dObjSrc->getType() == ito::tFloat32)
        {
            for(int row = 0 ; row < srcMat->rows ; row++)
            {
                floatLinePtr1 = srcMat->ptr<ito::float32>(row);
                floatLinePtr2 = replaceMat->ptr<ito::float32>(row);
                floatLinePtr3 = destMat->ptr<ito::float32>(row);

                if(floatLinePtr3 != floatLinePtr1) //are equal if src == dest
                {
                    memcpy(floatLinePtr3, floatLinePtr1, sizeof(ito::float32) * srcMat->cols);
                }

                for(int col = 0 ; col < srcMat->cols ; col++)
                {
                    if(!ito::dObjHelper::isFinite(floatLinePtr1[col]))
                    {
                        floatLinePtr3[col] = floatLinePtr2[col];
                        nrOfReplacements++;
                    }
                }
            }
        }
        else //double
        {
            for(int row = 0 ; row < srcMat->rows ; row++)
            {
                doubleLinePtr1 = srcMat->ptr<ito::float64>(row);
                doubleLinePtr2 = replaceMat->ptr<ito::float64>(row);
                doubleLinePtr3 = destMat->ptr<ito::float64>(row);

                if(doubleLinePtr3 != doubleLinePtr1) //are equal if src == dest
                {
                    memcpy(doubleLinePtr3, doubleLinePtr1, sizeof(ito::float64) * srcMat->cols);
                }

                for(int col = 0 ; col < srcMat->cols ; col++)
                {
                    if(!ito::dObjHelper::isFinite(doubleLinePtr1[col]))
                    {
                        doubleLinePtr3[col] = doubleLinePtr2[col];
                        nrOfReplacements++;
                    }
                }
            }
        }
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if(!retval.containsError())
    {
        // Add Protokoll

//        char prot[81] = {0};
//        _snprintf(prot, 80, "replace NaN and infinity values");
//        dObjDst->addToProtocol(std::string(prot));
        QString msg = tr("replace NaN and infinity values");
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    /*QVariant nrOfRepl(nrOfReplacements);
    outVals->append(nrOfRepl);*/

    (*paramsOut)[0].setVal<int>(nrOfReplacements);

    return retval;

}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::replaceInfAndNaNParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("srcImg", ito::ParamBase::DObjPtr, NULL, tr("Input image").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("replaceImg", ito::ParamBase::DObjPtr, NULL, tr("Image with values which will be used for replacement").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImg", ito::ParamBase::DObjPtr, NULL, tr("Output image").toLatin1().data());
        paramsMand->append(param);

        paramsOut->append( ito::Param("nrOfReplacements", ito::ParamBase::Int | ito::ParamBase::Out, 0, NULL, tr("number of replacments").toLatin1().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::mergeColorPlanesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("srcImg", ito::ParamBase::DObjPtr, NULL, tr("Input image with 3 or 4 uint8 planes").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImg", ito::ParamBase::DObjPtr, NULL, tr("Output image with uint32 planes").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("toggleByteOrder", ito::ParamBase::Int, 0, 3, 0, tr("Switch between RGBA = 0, BGRA = 1, ARGB = 2, ABGR = 3").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::mergeColorPlane(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();
    
    ito::DataObject tempDest;

    int toggleByteOrder = (int)(*paramsOpt)[0].getVal<int>();

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: replace image ptr empty").toLatin1().data());
    }

    int numMats = dObjSrc->calcNumMats();

    if(numMats == 3 && toggleByteOrder > 1)
    {
        toggleByteOrder -= 2;
    }

//    if(dObjSrc->getType() != ito::tUInt8 && dObjSrc->getDims() != 3 && (numMats != 3 || numMats != 4))
	// TODO: malformed if statement changed || to && is this what was intended?
    if(dObjSrc->getType() != ito::tUInt8 && dObjSrc->getDims() != 3 && (numMats != 3 && numMats != 4))
    {
        return ito::RetVal(ito::retError, 0, tr("Error: The primary object must be of type tUInt8").toLatin1().data());
    }

    int planeSize[2] = {dObjSrc->getSize(1), dObjSrc->getSize(2)};
    bool check;

    if((dObjSrc == dObjDst) || 
        (dObjDst->getType() != ito::tUInt32 && dObjDst->getType() != ito::tRGBA32)|| 
        dObjDst->getDims() != 2 || 
        planeSize[0] != dObjDst->getSize(0) || 
        planeSize[1] != dObjDst->getSize(1) ) // Check if dimensions of new object are okay
    {
        tempDest = ito::DataObject(2, planeSize, ito::tRGBA32);
    }
    else
    {
        tempDest = (*dObjDst);
    }
    
    //tempDest.setT(dObjSrc->isT());
    dObjSrc->copyTagMapTo(tempDest);
    tempDest.setAxisDescription(0, dObjSrc->getAxisDescription(1, check));
    tempDest.setAxisDescription(1, dObjSrc->getAxisDescription(2, check));
    tempDest.setAxisUnit(0, dObjSrc->getAxisUnit(1, check));
    tempDest.setAxisUnit(1, dObjSrc->getAxisUnit(2, check));
    tempDest.setAxisScale(0, dObjSrc->getAxisScale(1));
    tempDest.setAxisScale(1, dObjSrc->getAxisScale(2));
    tempDest.setAxisOffset(0, dObjSrc->getAxisOffset(1));
    tempDest.setAxisOffset(1, dObjSrc->getAxisOffset(2));
 

    cv::Mat_<ito::uint8>* matR = NULL;
    cv::Mat_<ito::uint8>* matG = NULL;
    cv::Mat_<ito::uint8>* matB = NULL;
    cv::Mat_<ito::uint8>* matA = NULL;
    cv::Mat_<ito::int32>* matRes = (cv::Mat_<ito::int32>*)(tempDest.get_mdata()[0]);;

    switch(toggleByteOrder)
    {
        case 0:
            matR = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[0]);
            matG = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[1]);
            matB = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[2]);
            if(numMats == 4) matA = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[3]);
        break;
        case 1:
            matB = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[0]);
            matG = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[1]);
            matR = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[2]);
            if(numMats == 4) matA = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[3]);
        break;
        case 2:
            matA = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[0]);
            matR = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[1]);
            matG = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[2]);
            matB = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[3]);  
        break;
        case 3:
            matA = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[0]);
            matB = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[1]);
            matG = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[2]);
            matR = (cv::Mat_<ito::uint8>*)(dObjSrc->get_mdata()[3]);         
        break;
    }

    bool convertToInt32 = dObjDst->getType() != ito::tRGBA32;

    if(numMats == 4 && convertToInt32)
    {
        ito::uint8* rowPtrR;
        ito::uint8* rowPtrG;
        ito::uint8* rowPtrB;
        ito::uint8* rowPtrA;
        ito::int32* rowPtrDst;

        for( int y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrA = matA->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::int32>(y);
            for( int x = 0; x < planeSize[1]; x++)
            {
                rowPtrDst[x] =  (ito::int32)rowPtrR[x];
                rowPtrDst[x] += ((ito::int32)rowPtrG[x]) << 8;
                rowPtrDst[x] += ((ito::int32)rowPtrB[x]) << 16;
                rowPtrDst[x] += ((ito::int32)rowPtrA[x]) << 24;
            }
        }
    }
    else if(convertToInt32)
    {
        ito::uint8* rowPtrR;
        ito::uint8* rowPtrG;
        ito::uint8* rowPtrB;
        ito::int32* rowPtrDst;

        for( int y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::int32>(y);

            for( int x = 0; x < planeSize[1]; x++)
            {
                rowPtrDst[x] =  (ito::int32)rowPtrR[x];
                rowPtrDst[x] += ((ito::int32)rowPtrG[x]) << 8;
                rowPtrDst[x] += ((ito::int32)rowPtrB[x]) << 16;
            }
        }
    }
    else if(numMats == 4)
    {
        ito::uint8* rowPtrR;
        ito::uint8* rowPtrG;
        ito::uint8* rowPtrB;
        ito::uint8* rowPtrA;
        ito::RgbaBase32* rowPtrDst;

        for( int y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrA = matA->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::Rgba32>(y);

            for( int x = 0; x < planeSize[1]; x++)
            {
                rowPtrDst[x].a =  (ito::int32)rowPtrA[x];
                rowPtrDst[x].b =  (ito::int32)rowPtrB[x];
                rowPtrDst[x].g =  (ito::int32)rowPtrG[x];
                rowPtrDst[x].r =  (ito::int32)rowPtrR[x];
            }
        }
    }
    else
    {
        ito::uint8* rowPtrR;
        ito::uint8* rowPtrG;
        ito::uint8* rowPtrB;
        ito::RgbaBase32* rowPtrDst;

        for( int y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::Rgba32>(y);

            for( int x = 0; x < planeSize[1]; x++)
            {
                rowPtrDst[x].b =  (ito::int32)rowPtrB[x];
                rowPtrDst[x].g =  (ito::int32)rowPtrG[x];
                rowPtrDst[x].r =  (ito::int32)rowPtrR[x];
            }
        }
    }

    if(dObjSrc == dObjDst) (*dObjDst) = tempDest;

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if(!retval.containsError())
    {
        // Add Protokoll

//        char prot[81] = {0};
//        _snprintf(prot, 80, "replace NaN and infinity values");
//        dObjDst->addToProtocol(std::string(prot));
        QString msg = tr("Merged from multiplane color object");
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::calcMeanOverZParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("srcImg", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input object with multiple z-planes").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImg", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image with 1 or 2 float64 planes").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("ignoreInf", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Ignore invalid-Values for floating point").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("calcStd", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("If true, calculate standard deviation").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Type, typename _TypeDst> void calcMeanOverZHelp(_Type ***srcPtr, cv::Mat *dstMat, const int *sizes, const bool toggleInf)
{
    _TypeDst *linePtr = 0;
    ito::float64 value = 0.0;
    ito::float64 cnts = 0;
    if(std::numeric_limits<_Type>::is_exact)
    {
        for(int y = 0; y < sizes[1]; y++)
        {
            linePtr = dstMat->ptr<_TypeDst>(y);
            for(int x = 0; x < sizes[2]; x++)
            {
                value = 0.0;
                cnts = 0.0;
                for(int z = 0; z < sizes[0]; z ++)
                {
                    value += (ito::float64)(srcPtr[z][y][x]);
                    cnts++;
                }
                linePtr[x] = cv::saturate_cast<_TypeDst>(value / cnts);
            }
        }
    }
    else
    {
        if(toggleInf)
        {
            for(int y = 0; y < sizes[1]; y++)
            {
                linePtr = dstMat->ptr<_TypeDst>(y);
                for(int x = 0; x < sizes[2]; x++)
                {
                    value = 0.0;
                    cnts = 0.0;

                    for(int z = 0; z < sizes[0]; z ++)
                    {
                        if(ito::dObjHelper::isFinite<_Type>(srcPtr[z][y][x]))
                        {
                            value += (ito::float64)(srcPtr[z][y][x]);
                            cnts++;                    
                        }
                    }
                    if(cnts > 0.0)
                    {
                        linePtr[x] = cv::saturate_cast<_TypeDst>(value / cnts);
                    }
                    else
                    {
                        linePtr[x] = std::numeric_limits<_TypeDst>::quiet_NaN();
                    }
                
                }
            } 
        }
        else
        {
            for(int y = 0; y < sizes[1]; y++)
            {
                linePtr = dstMat->ptr<_TypeDst>(y);
                for(int x = 0; x < sizes[2]; x++)
                {
                    value = 0.0;
                    cnts = 0.0;

                    for(int z = 0; z < sizes[0]; z ++)
                    {
                        value += (ito::float64)(srcPtr[z][y][x]);
                        cnts++;                    
                    }
                    if(cnts > 0.0)
                    {
                        linePtr[x] = cv::saturate_cast<_TypeDst>(value / cnts);
                    }
                    else
                    {
                        linePtr[x] = std::numeric_limits<_TypeDst>::quiet_NaN();
                    }
                
                }
            }
        }    
    }
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Type, typename _TypeDst> void calcDeviationOverZHelp(_Type ***srcPtr, cv::Mat *meanMat, cv::Mat *dstMat, const int *sizes, const bool toggleInf)
{
    _TypeDst *linePtr = 0;
    _TypeDst *meanlinePtr = 0;
    ito::float64 value = 0.0;
    ito::float64 cnts = 0;
    if(std::numeric_limits<_Type>::is_exact)
    {
        for(int y = 0; y < sizes[1]; y++)
        {
            linePtr = dstMat->ptr<_TypeDst>(y);
            meanlinePtr = meanMat->ptr<_TypeDst>(y);
            for(int x = 0; x < sizes[2]; x++)
            {
                value = 0.0;
                cnts = 0.0;
                for(int z = 0; z < sizes[0]; z ++)
                {
                    value += pow((ito::float64)(srcPtr[z][y][x]) - (ito::float64)(meanlinePtr[x]), 2);
                    cnts++;
                }
                linePtr[x] = cv::saturate_cast<_TypeDst>(sqrt(value / cnts));
            }
        }
    }
    else
    {
        if(toggleInf)
        {
            for(int y = 0; y < sizes[1]; y++)
            {
                linePtr = dstMat->ptr<_TypeDst>(y);
                meanlinePtr = meanMat->ptr<_TypeDst>(y);
                for(int x = 0; x < sizes[2]; x++)
                {
                    value = 0.0;
                    cnts = 0.0;

                    for(int z = 0; z < sizes[0]; z ++)
                    {
                        if(ito::dObjHelper::isFinite<_Type>(srcPtr[z][y][x]))
                        {
                            value += pow((ito::float64)(srcPtr[z][y][x]) - (ito::float64)(meanlinePtr[x]), 2);
                            cnts++;                    
                        }
                    }
                    if(cnts > 0.0)
                    {
                        linePtr[x] = cv::saturate_cast<_TypeDst>(sqrt(value / cnts));
                    }
                    else
                    {
                        linePtr[x] = std::numeric_limits<_TypeDst>::quiet_NaN();
                    }
                
                }
            } 
        }
        else
        {
            for(int y = 0; y < sizes[1]; y++)
            {
                linePtr = dstMat->ptr<_TypeDst>(y);
                meanlinePtr = meanMat->ptr<_TypeDst>(y);
                for(int x = 0; x < sizes[2]; x++)
                {
                    value = 0.0;
                    cnts = 0.0;

                    for(int z = 0; z < sizes[0]; z ++)
                    {
                        value += pow((ito::float64)(srcPtr[z][y][x]) - (ito::float64)(meanlinePtr[x]), 2);
                        cnts++;                    
                    }
                    if(cnts > 0.0)
                    {
                        linePtr[x] = cv::saturate_cast<_TypeDst>(sqrt(value / cnts));
                    }
                    else
                    {
                        linePtr[x] = std::numeric_limits<_TypeDst>::quiet_NaN();
                    }
                
                }
            }
        }    
    }
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date 02.2012
*/
ito::RetVal BasicFilters::calcMeanOverZ(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    bool overWrite = true;
    int xsize = 1;
    int ysize = 1;

    ito::DataObject destPlane;

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImageStack is Null-Pointer").toLatin1().data());
    }
    
    int nessPlanes = (*paramsOpt)[1].getVal<int>() > 0 ? 2 : 1;
    bool toggleInf = (*paramsOpt)[0].getVal<int>() > 0;

    xsize = dObjSrc->getSize(dObjSrc->getDims() - 1);
    ysize = dObjSrc->getSize(dObjSrc->getDims() - 2);

    retval += ito::dObjHelper::verify3DDataObject(dObjSrc, "sourceImageStack", 2, 256,  ysize, ysize, xsize, xsize, 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if(retval.containsError())
    {
        return retval;
    }

    if(dObjDst == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("destinationPlane is a uninitialized dataObject!").toLatin1().data());
    }
    else if(!retval.containsError())
    {
        if(dObjDst != dObjSrc)
        {
            ito::RetVal tRetval = ito::dObjHelper::verify3DDataObject(dObjDst, "destinationPlane", nessPlanes, nessPlanes, ysize, ysize, xsize, xsize,  1, dObjSrc->getType());
            if(tRetval.containsError())
            {
                destPlane = ito::DataObject(nessPlanes, ysize, xsize, dObjSrc->getType());
            }
            else
            {
                destPlane = *dObjDst;
                overWrite = false;
            } 
        }
        else
        {
            destPlane = ito::DataObject( ysize, xsize, dObjSrc->getType());
        }
    }

    if(retval.containsError())
    {
        return retval;
    }

    int sizes[3] = {dObjSrc->getSize(0), dObjSrc->getSize(1), dObjSrc->getSize(2)};
    switch(dObjSrc->getType())
    {
        case ito::tInt8:
        {
            ito::int8 ***srcPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::int8>(dObjSrc, srcPtr);
            calcMeanOverZHelp<ito::int8, ito::int8>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toggleInf);
            if(nessPlanes > 1) calcDeviationOverZHelp<ito::int8, ito::int8>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), (cv::Mat*)(destPlane.get_mdata()[1]), sizes, toggleInf);
            ito::dObjHelper::freeRowPointer<ito::int8>(srcPtr);
        }
        break;
        case ito::tUInt8:
        {
            ito::uint8 ***srcPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::uint8>(dObjSrc, srcPtr);
            calcMeanOverZHelp<ito::uint8, ito::uint8>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toggleInf);
            if(nessPlanes > 1) calcDeviationOverZHelp<ito::uint8, ito::uint8>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), (cv::Mat*)(destPlane.get_mdata()[1]), sizes, toggleInf);
            ito::dObjHelper::freeRowPointer<ito::uint8>(srcPtr);
        }
        break;
        case ito::tInt16:
        {
            ito::int16 ***srcPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::int16>(dObjSrc, srcPtr);
            calcMeanOverZHelp<ito::int16, ito::int16>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toggleInf);
            if(nessPlanes > 1) calcDeviationOverZHelp<ito::int16, ito::int16>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), (cv::Mat*)(destPlane.get_mdata()[1]), sizes, toggleInf);
            ito::dObjHelper::freeRowPointer<ito::int16>(srcPtr);
        }
        break;
        case ito::tUInt16:
        {
            ito::uint16 ***srcPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::uint16>(dObjSrc, srcPtr);
            calcMeanOverZHelp<ito::uint16, ito::uint16>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toggleInf);
            if(nessPlanes > 1) calcDeviationOverZHelp<ito::uint16, ito::uint16>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), (cv::Mat*)(destPlane.get_mdata()[1]), sizes, toggleInf);
            ito::dObjHelper::freeRowPointer<ito::uint16>(srcPtr);
        }
        break;
        case ito::tInt32:
        {
            ito::int32 ***srcPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::int32>(dObjSrc, srcPtr);
            calcMeanOverZHelp<ito::int32, ito::int32>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toggleInf);
            if(nessPlanes > 1) calcDeviationOverZHelp<ito::int32, ito::int32>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), (cv::Mat*)(destPlane.get_mdata()[1]), sizes, toggleInf);
            ito::dObjHelper::freeRowPointer<ito::int32>(srcPtr);
        }
        break;
        case ito::tFloat32:
        {
            ito::float32 ***srcPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::float32>(dObjSrc, srcPtr);
            calcMeanOverZHelp<ito::float32, ito::float32>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toggleInf);
            if(nessPlanes > 1) calcDeviationOverZHelp<ito::float32, ito::float32>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), (cv::Mat*)(destPlane.get_mdata()[1]), sizes, toggleInf);
            ito::dObjHelper::freeRowPointer<ito::float32>(srcPtr);
        }
        break;
        case ito::tFloat64:
        {
            ito::float64 ***srcPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::float64>(dObjSrc, srcPtr);
            calcMeanOverZHelp<ito::float64, ito::float64>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toggleInf);
            if(nessPlanes > 1) calcDeviationOverZHelp<ito::float64, ito::float64>(srcPtr, (cv::Mat*)(destPlane.get_mdata()[0]), (cv::Mat*)(destPlane.get_mdata()[1]), sizes, toggleInf);
            ito::dObjHelper::freeRowPointer<ito::float64>(srcPtr);
        }
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toLatin1().data());
    }

    if(!retval.containsError())
    {
        dObjSrc->copyTagMapTo(destPlane);
        ito::dObjHelper::dObjCopyLastNAxisTags(*dObjSrc, destPlane, 2, true, true);
        QString msg = tr("Calculated mean value in z-Direction from 3D-Object");
        destPlane.addToProtocol(std::string(msg.toLatin1().data()));
    }

    if(overWrite)
    {
        *dObjDst = destPlane;
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail Filter parameters for calcObjSlice
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date 02.2012
*/
ito::RetVal BasicFilters::calcObjSliceParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("srcImg", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D image or single plane n-D object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("dstSlice", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Slice with output data").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("x0", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 0.0, tr("x0-coordinate for slice").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("y0", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 1.0, tr("y0-coordinate for slice").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("x1", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 0.0, tr("x1-coordinate for slice").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("y1", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 1.0, tr("y1-coordinate for slice").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("interpolation mode", ito::ParamBase::Int, 0, 0, 0, tr("0: Bresenham or Nearest, 1: weighted").toLatin1().data());
        paramsOpt->append(param);
        //param = ito::Param("numer of pixel", ito::ParamBase::Int, 0, 65366, 0, tr("Number of pixels for output image").toLatin1().data());
        //paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date 02.2012
*/
ito::RetVal BasicFilters::calcObjSlice(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut */)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is Null-Pointer").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is Null-Pointer").toLatin1().data());
    }
   
    ito::int32 dims = dObjSrc->getDims();

    if(dims < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage must have at least 2 dimensions").toLatin1().data());
    }

    if(dObjSrc->calcNumMats() > 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage must not have more than 1 plane").toLatin1().data());
    }
  

    ito::float64 physX0 = (*paramsMand)[2].getVal<double>();
    ito::float64 physY0 = (*paramsMand)[3].getVal<double>();
    ito::float64 physX1 = (*paramsMand)[4].getVal<double>();
    ito::float64 physY1 = (*paramsMand)[5].getVal<double>();

    ito::int32 sliceXSize = 0;
    //ito::int32 sliceXSize = (*paramsOpt)[1].getVal<int>();
/*
    if( sliceXSize < 1 )
    {
        
    }
*/

    ito::int32 xsize = dObjSrc->getSize(dims - 1);
    ito::int32 ysize = dObjSrc->getSize(dims - 2);

    retval += ito::dObjHelper::verifyDataObjectType(dObjSrc, "sourceImageStack", 10, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64,  ito::tComplex64, ito::tComplex128, ito::tRGBA32);
    if(retval.containsError())
    {
        return retval;
    }

    bool _unused;

    ito::int32 pxX0 = qRound(dObjSrc->getPhysToPix(dims-1, physX0, _unused));
    ito::int32 pxY0 = qRound(dObjSrc->getPhysToPix(dims-2, physY0, _unused));
    ito::int32 pxX1 = qRound(dObjSrc->getPhysToPix(dims-1, physX1, _unused));
    ito::int32 pxY1 = qRound(dObjSrc->getPhysToPix(dims-2, physY1, _unused));

    pxX0 = pxX0 < 0 ? 0 : pxX0 > (xsize - 1) ? xsize - 1 : pxX0;
    pxX1 = pxX1 < 0 ? 0 : pxX1 > (xsize - 1) ? xsize - 1 : pxX1;

    pxY0 = pxY0 < 0 ? 0 : pxY0 > (ysize - 1) ? ysize - 1 : pxY0;
    pxY1 = pxY1 < 0 ? 0 : pxY1 > (ysize - 1) ? ysize - 1 : pxY1;

    cv::Mat* srcMat = (cv::Mat*)dObjSrc->get_mdata()[ dObjSrc->seekMat(0) ]; //first plane in ROI

    ito::int32   sampleDir = 0;
    ito::float64 startPhys = 0.0;
    ito::float64 startPx = 0.0;
    ito::float64 right = 1.0;
    ito::float64 stepSizePhys = 1.0;

    std::string axisDescription = "";
    std::string axisUnit = "";

    std::string valueDescription = "";
    std::string valueUnit = "";


    ito::int32 matOffset = 0;
    QVector<ito::int32> matStepSize;
    matStepSize.clear();

    if( pxX0 == pxX1 ) //pure line in y-direction
    {
        sampleDir = 1;

        if(pxY1 >= pxY0)
        {
            sliceXSize   = 1 + pxY1 - pxY0;
            startPhys    = dObjSrc->getPixToPhys(dims-2, pxY0, _unused);
            startPx      = pxY0;
            right        = dObjSrc->getPixToPhys(dims-2, pxY1, _unused);
            stepSizePhys = sliceXSize > 1 ? (right - startPhys) / (ito::float64)(sliceXSize - 1) : 0.0;

            matStepSize.resize(sliceXSize);

            matOffset    = (int)srcMat->step[0] * pxY0 + (int)srcMat->step[1] * pxX0;
            matStepSize.fill((int)srcMat->step[0], sliceXSize);
        }
        else
        {
            sliceXSize   = 1 + pxY0 - pxY1;
            startPhys    = dObjSrc->getPixToPhys(dims-2, pxY1, _unused); 
            startPx      = pxY1;
            right        = dObjSrc->getPixToPhys(dims-2, pxY0, _unused);
            stepSizePhys = sliceXSize > 1 ? (right - startPhys) / (ito::float64)(sliceXSize - 1) : 0.0;

            matStepSize.resize(sliceXSize);

            matOffset    = (int)srcMat->step[0] * pxY1 + (int)srcMat->step[1] * pxX0;
            matStepSize.fill((int)srcMat->step[0], sliceXSize);
        }

        axisDescription = dObjSrc->getAxisDescription(dims - 2, _unused);
        axisUnit = dObjSrc->getAxisUnit(dims - 2, _unused);

        if(axisDescription == "") axisDescription = "y-axis";
               
        valueDescription = dObjSrc->getAxisDescription(dims - 2, _unused);
        valueUnit = dObjSrc->getAxisUnit(dims - 2, _unused);
    }
    else if( pxY0 == pxY1 ) //pure line in x-direction
    {
        sampleDir = 0;

        if(pxX1 >= pxX0)
        {
            sliceXSize   = 1 + pxX1 - pxX0;
            startPhys    = dObjSrc->getPixToPhys(dims-1, pxX0, _unused); 
            startPx      = pxX0;
            right        = dObjSrc->getPixToPhys(dims-1, pxX1, _unused);
            stepSizePhys = sliceXSize > 1 ? (right - startPhys) / (ito::float64)(sliceXSize - 1) : 0.0;
                
            matStepSize.resize(sliceXSize);

            matOffset = (int)srcMat->step[0] * pxY0 + (int)srcMat->step[1] * pxX0;
            matStepSize.fill((int)srcMat->step[1], sliceXSize);
        }
        else
        {
            sliceXSize   = 1 + pxX0 - pxX1;
            startPhys    = dObjSrc->getPixToPhys(dims-1, pxX1, _unused);
            startPx      = pxX1;
            right        = dObjSrc->getPixToPhys(dims-1, pxX0, _unused); 
            stepSizePhys = sliceXSize > 1 ? (right - startPhys) / (ito::float64)(sliceXSize - 1) : 0.0;

            matStepSize.resize(sliceXSize);

            matOffset = (int)srcMat->step[0] * pxY0 + (int)srcMat->step[1] * pxX1; 
            matStepSize.fill((int)srcMat->step[1], sliceXSize);
        }

        axisDescription = dObjSrc->getAxisDescription(dims-1,_unused);
        axisUnit = dObjSrc->getAxisUnit(dims-1,_unused);
        if(axisDescription == "") axisDescription = "x-axis";
  
        valueDescription = dObjSrc->getValueDescription();
        valueUnit = dObjSrc->getValueUnit();

    }
    else
    {
        sampleDir = 2;
                
        if (true)
        {
            // simple line points calculation using Bresenham
            // http://de.wikipedia.org/wiki/Bresenham-Algorithmus#C-Implementierung
            
            int dx = abs( pxX1 - pxX0 );
            int incx = pxX0 <= pxX1 ? 1 : -1;
            int dy = abs( pxY1 - pxY0 );
            int incy = pxY0 <= pxY1 ? 1 : -1;

            sliceXSize = 1 + std::max(dx,dy);

            startPhys= 0.0;  //there is no physical starting point for diagonal lines.

            if(sliceXSize > 0)
            {
                double dxPhys = dObjSrc->getPixToPhys(dims-1, pxX1, _unused) - dObjSrc->getPixToPhys(dims-1, pxX0, _unused);
                double dyPhys = dObjSrc->getPixToPhys(dims-2, pxY1, _unused) - dObjSrc->getPixToPhys(dims-2, pxY0, _unused);
                stepSizePhys = sqrt((dxPhys * dxPhys) + (dyPhys * dyPhys)) / (sliceXSize - 1);
            }
            else
            {
                stepSizePhys = 0.0;
            }

            matOffset = (int)srcMat->step[0] * pxY0 + (int)srcMat->step[1] * pxX0; 

            int pdx, pdy, ddx, ddy, es, el;
            if(dx>dy)
            {
                pdx = incx;
                pdy = 0;
                ddx = incx;
                ddy = incy;
                es = dy;
                el = dx;
            }
            else
            {
                pdx = 0;
                pdy = incy;
                ddx = incx;
                ddy = incy;
                es = dx;
                el = dy;
            }

            int err = el / 2; //0; /* error value e_xy */
            int x = 0; //pxX0;
            int y = 0; //pxY0;

            matStepSize.resize(sliceXSize);

            for(unsigned int n = 0; n < (unsigned int)sliceXSize; n++)
            {  /* loop */
                //setPixel(x,y)
                matStepSize[n] = (int)srcMat->step[0] * y + (int)srcMat->step[1] * x;

                err -= es;
                if(err < 0)
                {
                    err += el;
                    x += ddx;
                    y += ddy;
                }
                else
                {
                    x += pdx;
                    y += pdy;
                }
            }

            for(unsigned int n = sliceXSize - 1; n > 0; n--)
            {
                matStepSize[n] -= matStepSize[n-1];
            }

        }
        else
        {

        }

        axisDescription = "x/y-axis";
        
        if(dObjSrc->getAxisUnit(dims-1,_unused) == dObjSrc->getAxisUnit(dims-2,_unused)) axisUnit = dObjSrc->getAxisUnit(dims-1,_unused);
        else axisUnit = "";

        valueDescription = dObjSrc->getValueDescription();
        valueUnit = dObjSrc->getValueUnit();

    }

    if(sliceXSize < 1 && !retval.containsError())
    {
        retval += ito::RetVal(ito::retError, 0, tr("slice has not defined size").toLatin1().data());
    }

    bool needNewObj = false;

    if(!retval.containsError())
    {
        if (dObjDst->getDims()  != 2 )
        {
            needNewObj = true;
        }
        else
        {
            if((dObjDst->getSize(0) != 1) || (dObjDst->getSize(1) != sliceXSize) || (dObjDst->getType() != dObjSrc->getType()))
            {
                needNewObj = true;
            }
        }
    }
    ito::DataObject tempObj;

    if(!retval.containsError())
    {
        if(needNewObj)
        {
            tempObj = ito::DataObject(1, sliceXSize, dObjSrc->getType());
        }
        else
        {
            tempObj = *dObjDst;
        }
    }

    if(!matStepSize.isEmpty() && !retval.containsError())
    {
        switch(tempObj.getType())
        {
            case ito::tInt8:
            {
                ito::int8 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::int8>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::int8*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tInt16:
            {
                ito::int16 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::int16>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::int16*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tInt32:
            {
                ito::int32 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::int32>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::int32*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tUInt8:
            {
                ito::uint8 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::uint8>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::uint8*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tUInt16:
            {
                ito::uint16 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::uint16>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::uint16*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tFloat32:
            {
                ito::float32 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::float32>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::float32*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tFloat64:
            {
                ito::float64 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::float64>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::float64*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tComplex64:
            {
                ito::complex64 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::complex64>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::complex64*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tComplex128:
            {
                ito::complex128 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::complex128>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::complex128*>(srcPtr));
                    srcPtr += matStepSize[x];
                }
            }
            break;
            case ito::tRGBA32:
            {
                ito::RgbaBase32 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::Rgba32>(0);
                uchar *srcPtr = srcMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x].rgba = (reinterpret_cast<ito::RgbaBase32*>(srcPtr))->rgba;
                    srcPtr += matStepSize[x];
                }            
            }
            break;
            default:
                retval += ito::RetVal(ito::retError, 0, tr("datatype not supported").toLatin1().data());
                break;
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("matrix step vector for matrix is empty").toLatin1().data());
    }

    if(!retval.containsError())
    {
        dObjSrc->copyTagMapTo(tempObj);
        if(needNewObj)
        {
            *dObjDst = tempObj;
        }
        QString msg = tr("Cut 1D slice out of 2D-data from [ %2, %3] to [ %4, %5]").arg(physX0).arg(physY0).arg(physX1).arg(physY1);
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
        dObjDst->setAxisDescription(1, axisDescription);
        dObjDst->setAxisUnit(1, axisUnit);

        if(ito::dObjHelper::isNotZero(stepSizePhys)) dObjDst->setAxisScale(1, stepSizePhys);
        else dObjDst->setAxisScale(1, stepSizePhys);

        dObjDst->setAxisOffset(1, startPx);
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\brief This function sets values below minVal and above maxVal to newValue
   \detail
   \param[in|out]   planeInOut   Image data
   \param[in]   minVal   Lowest Value in new Image
   \param[in]   maxVal   Highest Value in new Image
   \param[in]   newValue New value for pixel, which are out of range
   \param[in]   outside  If true, values outside if [min max] are clipped else inside;
   \author ITO
   \sa  
   \date
*/
template<typename _Tp> ito::RetVal clipValuesHelper(cv::Mat *planeInOut, _Tp minVal, _Tp maxVal, _Tp newValue, bool outside)
{

    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif  

    _Tp* rowPtr = NULL;
    
    int x, y;

    if(outside)
    {
        if(std::numeric_limits<_Tp>::is_exact)
        {
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif 
            for(y = 0; y < planeInOut->rows; y++)
            {
                rowPtr = (_Tp*)planeInOut->ptr(y);
                for(x = 0; x < planeInOut->cols; x++)
                {
                    if ( (rowPtr[x] < minVal) )
                    {
                        rowPtr[x] = newValue;
                    }
                    else if (rowPtr[x] > maxVal)
                    {
                        rowPtr[x] = newValue;
                    }
                }
            }
        }
        else
        {
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif 
            for(y = 0; y < planeInOut->rows; y++)
            {
                rowPtr = (_Tp*)planeInOut->ptr(y);
                for(x = 0; x < planeInOut->cols; x++)
                {
                    if ( !ito::dObjHelper::isFinite<_Tp>(rowPtr[x]))
                    {
                        rowPtr[x] = newValue;
                    }
                    else if ( (rowPtr[x] < minVal) )
                    {
                        rowPtr[x] = newValue;
                    }
                    else if (rowPtr[x] > maxVal)
                    {
                        rowPtr[x] = newValue;
                    }
                }
            }
        }
    }
    else
    {
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif 
        for(y = 0; y < planeInOut->rows; y++)
        {
            rowPtr = (_Tp*)planeInOut->ptr(y);
            for(x = 0; x < planeInOut->cols; x++)
            {
                /*if ( !ito::dObjHelper::isFinite<_Tp>(rowPtr[x]))
                {
                    rowPtr[x] = newValue;
                }
                else */if (rowPtr[x] > minVal && rowPtr[x] < maxVal)
                {
                    rowPtr[x] = newValue;
                }
            }
        }  

    }
    #if (USEOMP)
    }
    #endif
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::clipValueFilterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("input image [real typed data object]").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("destination image (inplace possible)").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("minValue", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("lowest value in range").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("maxValue", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("highest value in range").toLatin1().data());
    paramsMand->append(param);

    param = ito::Param("newValue", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("value set to clipped values (default: 0.0)").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("insideFlag", ito::ParamBase::Int, 0, 1, 0, tr("0: clip values outside of given range (default), 1: clip inside").toLatin1().data());
    paramsOpt->append(param);

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail  
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \sa  mcppfilters::clipValueFilter
   \date
*/
ito::RetVal BasicFilters::clipValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image empty").toLatin1().data());
    }

    if (dObjImages->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
    }

    if( (*paramsMand)[2].getVal<double>() > (*paramsMand)[3].getVal<double>())
    {
        return ito::RetVal(ito::retError, 0, tr("Error: minValue must be smaller than maxValue").toLatin1().data());
    }
    bool outsideFlag = (*paramsOpt)[1].getVal<int>() == 0 ? true : false;

    double minValProt = 0;
    double maxValProt = 0;
    double valProt = 0;

    if(dObjImages != dObjDst)
    {
        dObjImages->copyTo(*dObjDst);
    }
    
    int z_length = dObjImages->calcNumMats();
    cv::Mat *cvMatOut;

    switch(dObjImages->getType())
    {
        case ito::tInt8:
        {
            ito::int8 minVal = cv::saturate_cast<ito::int8>((*paramsMand)[2].getVal<double>());
            ito::int8 maxVal = cv::saturate_cast<ito::int8>((*paramsMand)[3].getVal<double>());
            ito::int8 newValue = cv::saturate_cast<ito::int8>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipValuesHelper<ito::int8>(cvMatOut, minVal, maxVal, newValue, outsideFlag);
            }
            minValProt = minVal;
            maxValProt = maxVal;
            valProt = newValue;
        }
        break;
        case ito::tUInt8:
        {
            ito::uint8 minVal = cv::saturate_cast<ito::uint8>((*paramsMand)[2].getVal<double>());
            ito::uint8 maxVal = cv::saturate_cast<ito::uint8>((*paramsMand)[3].getVal<double>());
            ito::uint8 newValue = cv::saturate_cast<ito::uint8>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipValuesHelper<ito::uint8>(cvMatOut, minVal, maxVal, newValue, outsideFlag);
            }

            minValProt = minVal;
            maxValProt = maxVal;
            valProt = newValue;
        }
        break;
        case ito::tUInt16:
        {
            ito::uint16 minVal = cv::saturate_cast<ito::uint16>((*paramsMand)[2].getVal<double>());
            ito::uint16 maxVal = cv::saturate_cast<ito::uint16>((*paramsMand)[3].getVal<double>());
            ito::uint16 newValue = cv::saturate_cast<ito::uint16>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipValuesHelper<ito::uint16>(cvMatOut, minVal, maxVal, newValue, outsideFlag);
            }

            minValProt = minVal;
            maxValProt = maxVal;
            valProt = newValue;
        }
        break;
        case ito::tInt16:
        {
            ito::int16 minVal = cv::saturate_cast<ito::int16>((*paramsMand)[2].getVal<double>());
            ito::int16 maxVal = cv::saturate_cast<ito::int16>((*paramsMand)[3].getVal<double>());
            ito::int16 newValue = cv::saturate_cast<ito::int16>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipValuesHelper<ito::int16>(cvMatOut, minVal, maxVal, newValue, outsideFlag);
            }

            minValProt = minVal;
            maxValProt = maxVal;
            valProt = newValue;
        }
        break;
        case ito::tInt32:
        {
            ito::int32 minVal = cv::saturate_cast<ito::int32>((*paramsMand)[2].getVal<double>());
            ito::int32 maxVal = cv::saturate_cast<ito::int32>((*paramsMand)[3].getVal<double>());
            ito::int32 newValue = cv::saturate_cast<ito::int32>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipValuesHelper<ito::int32>(cvMatOut, minVal, maxVal, newValue, outsideFlag);
            }

            minValProt = minVal;
            maxValProt = maxVal;
            valProt = newValue;
        }
        break;
        case ito::tFloat32:
        {
            ito::float32 minVal = cv::saturate_cast<ito::float32>((*paramsMand)[2].getVal<double>());
            ito::float32 maxVal = cv::saturate_cast<ito::float32>((*paramsMand)[3].getVal<double>());
            ito::float32 newValue;
            valProt = (*paramsOpt)[0].getVal<double>();
            if(ito::dObjHelper::isNaN<double>(valProt))
            {
                newValue = std::numeric_limits<ito::float32>::quiet_NaN();
            }
            else if(ito::dObjHelper::isInf<double>(valProt))
            {
                newValue = std::numeric_limits<ito::float32>::infinity();
            }
            else
            {
                newValue = cv::saturate_cast<ito::float32>(valProt);
                valProt = newValue;
            }

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipValuesHelper<ito::float32>(cvMatOut, minVal, maxVal, newValue, outsideFlag);
            }

            minValProt = minVal;
            maxValProt = maxVal;
        }
        break;
        case ito::tFloat64:
        {
            ito::float64 minVal = (*paramsMand)[2].getVal<double>();
            ito::float64 maxVal = (*paramsMand)[3].getVal<double>();
            ito::float64 newValue = (*paramsOpt)[0].getVal<double>();

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipValuesHelper<ito::float64>(cvMatOut, minVal, maxVal, newValue, outsideFlag);
            }

            minValProt = minVal;
            maxValProt = maxVal;
            valProt = newValue;
        }
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("unknown type or type not implemented (e.g. complex)").toLatin1().data());
    }


    if(!retval.containsError())
    {
        QString msg;
        if(outsideFlag)
        {
            msg = tr("Clipped values outside %1 : %2 to %3").arg(minValProt).arg(maxValProt).arg(valProt);
        }
        else
        {
            msg = tr("Clipped values inside %1 : %2 to %3").arg(minValProt).arg(maxValProt).arg(valProt);
        }
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\brief This function sets values below minVal and above maxVal to newValue
   \detail
   \param[in|out]   planeInOut   Image data
   \param[in]   minVal   Lowest Value in new Image
   \param[in]   maxVal   Highest Value in new Image
   \param[in]   newValue New value for pixel, which are out of range
   \param[in]   outside  If true, values outside if [min max] are clipped else inside;
   \author ITO
   \sa  
   \date
*/
template<typename _Tp> ito::RetVal clipAbyBFirstHelper(const cv::Mat *planeComp, cv::Mat &planeBinary, _Tp minVal, _Tp maxVal, bool outside)
{
    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif  

    const _Tp* rowPtrComp = NULL;
    ito::uint8* rowPtrInOut = NULL;
    
    int x, y;

    if(outside)
    {
        if(std::numeric_limits<_Tp>::is_exact)
        {
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif 
            for(y = 0; y < planeBinary.rows; y++)
            {
                rowPtrInOut = (ito::uint8*)planeBinary.ptr(y);
                rowPtrComp = (_Tp*)planeComp->ptr(y);
                for(x = 0; x < planeBinary.cols; x++)
                {
                    if ( (rowPtrComp[x] < minVal) )
                    {
                        rowPtrInOut[x] = 0;
                    }
                    else if (rowPtrComp[x] > maxVal)
                    {
                        rowPtrInOut[x] = 0;
                    }
                    else
                    {
                        rowPtrInOut[x] = 1;
                    }
                }
            }
        }
        else
        {
            #if (USEOMP)
            #pragma omp for schedule(guided)
            #endif 
            for(y = 0; y < planeBinary.rows; y++)
            {
                rowPtrInOut = (ito::uint8*)planeBinary.ptr(y);
                rowPtrComp = (_Tp*)planeComp->ptr(y);
                for(x = 0; x < planeBinary.cols; x++)
                {
                    if ( !ito::dObjHelper::isFinite<_Tp>(rowPtrComp[x]))
                    {
                        rowPtrInOut[x] = 0;
                    }
                    else if (rowPtrComp[x] < minVal)
                    {
                        rowPtrInOut[x] = 0;
                    }
                    else if (rowPtrComp[x] > maxVal)
                    {
                        rowPtrInOut[x] = 0;
                    }
                    else
                    {
                        rowPtrInOut[x] = 1;
                    }
                }
            }
        }
    }
    else
    {
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif 
        for(y = 0; y < planeBinary.rows; y++)
        {
            rowPtrInOut = (ito::uint8*)planeBinary.ptr(y);
            rowPtrComp = (_Tp*)planeComp->ptr(y);
            for(x = 0; x < planeBinary.cols; x++)
            {
                /* if ( !ito::dObjHelper::isFinite<_Tp>(rowPtrComp[x]))
                {
                    rowPtr[x] = newValue;
                }
                else */ if (rowPtrComp[x] > minVal && rowPtrComp[x] < maxVal)
                {
                    rowPtrInOut[x] = 0;
                }
                else
                {
                    rowPtrInOut[x] = 1;
                }
            }
        }  

    }
    #if (USEOMP)
    }
    #endif
    return ito::retOk;
}
template<typename _Tp> ito::RetVal clipAbyBSecondHelper(const cv::Mat &planeBinary, cv::Mat *planeInOut, _Tp newValue)
{
    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif  

    const ito::uint8* rowPtrComp = NULL;
    _Tp* rowPtrInOut = NULL;
    
    int x, y;

    #if (USEOMP)
    #pragma omp for schedule(guided)
    #endif 
    for(y = 0; y < planeBinary.rows; y++)
    {
        rowPtrInOut = (_Tp*)planeInOut->ptr(y);
        rowPtrComp = (const ito::uint8*)planeBinary.ptr(y);
        for(x = 0; x < planeBinary.cols; x++)
        {
            if (rowPtrComp[x] == 0)
            {
                rowPtrInOut[x] = newValue;
            }
        }
    }

    #if (USEOMP)
    }
    #endif
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::clipAbyBFilterParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    param = ito::Param("sourceImage", ito::ParamBase::DObjPtr, NULL, tr("input image [real typed data object]").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("comparisonImage", ito::ParamBase::DObjPtr, NULL, tr("input image [real typed data object] for comparision").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("destinationImage", ito::ParamBase::DObjPtr, NULL, tr("destination image (inplace possible)").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("minValue", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("lowest value in range").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("maxValue", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("highest value in range").toLatin1().data());
    paramsMand->append(param);

    param = ito::Param("newValue", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("value set to clipped values (default: 0.0)").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("insideFlag", ito::ParamBase::Int, 0, 1, 0, tr("0: clip values outside of given range (default), 1: clip inside").toLatin1().data());
    paramsOpt->append(param);

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail  
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \sa  mcppfilters::clipValueFilter
   \date
*/
ito::RetVal BasicFilters::clipAbyBFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (*paramsMand)[0].getVal<ito::DataObject*>();
    ito::DataObject *dObjComperator = (*paramsMand)[1].getVal<ito::DataObject*>();
    ito::DataObject *dObjDst = (*paramsMand)[2].getVal<ito::DataObject*>();

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image empty").toLatin1().data());
    }

    if (!dObjComperator)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: comparison image empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image empty").toLatin1().data());
    }

    if (dObjImages->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
    }

    retval += ito::dObjHelper::verify2DDataObject(
                                dObjComperator, 
                                "comparison image", 
                                dObjImages->getSize(dObjImages->getDims() - 2), 
                                dObjImages->getSize(dObjImages->getDims() - 2), 
                                dObjImages->getSize(dObjImages->getDims() - 1), 
                                dObjImages->getSize(dObjImages->getDims() - 1), 
                                7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64); 

    if( (*paramsMand)[3].getVal<double>() > (*paramsMand)[4].getVal<double>())
    {
        return ito::RetVal(ito::retError, 0, tr("Error: minValue must be smaller than maxValue").toLatin1().data());
    }
    bool outsideFlag = (*paramsOpt)[1].getVal<int>() == 0 ? true : false;

    double minValProt = 0;
    double maxValProt = 0;
    double valProt = 0;

    if(dObjImages != dObjDst)
    {
        dObjImages->copyTo(*dObjDst);
    }
    
    int z_length = dObjImages->calcNumMats();
    cv::Mat *cvMatOut;
    cv::Mat *cvMatComp = ((cv::Mat *)dObjComperator->get_mdata()[dObjComperator->seekMat(0)]);

    cv::Mat combBin(cvMatComp->rows, cvMatComp->cols, CV_8U);
    switch(dObjComperator->getType())
    {
        case ito::tInt8:
        {
            ito::int8 minVal = cv::saturate_cast<ito::int8>((*paramsMand)[3].getVal<double>());
            ito::int8 maxVal = cv::saturate_cast<ito::int8>((*paramsMand)[4].getVal<double>()); 

            clipAbyBFirstHelper(cvMatComp, combBin, minVal, maxVal, outsideFlag);
            minValProt = (double)minVal;
            maxValProt = (double)maxVal;
        }
        break;
        case ito::tUInt8:
        {
            ito::uint8 minVal = cv::saturate_cast<ito::uint8>((*paramsMand)[3].getVal<double>());
            ito::uint8 maxVal = cv::saturate_cast<ito::uint8>((*paramsMand)[4].getVal<double>()); 

            clipAbyBFirstHelper(cvMatComp, combBin, minVal, maxVal, outsideFlag);
            minValProt = (double)minVal;
            maxValProt = (double)maxVal;
        }
        break;
        case ito::tInt16:
        {
            ito::int16 minVal = cv::saturate_cast<ito::int16>((*paramsMand)[3].getVal<double>());
            ito::int16 maxVal = cv::saturate_cast<ito::int16>((*paramsMand)[4].getVal<double>()); 

            clipAbyBFirstHelper(cvMatComp, combBin, minVal, maxVal, outsideFlag);
            minValProt = (double)minVal;
            maxValProt = (double)maxVal;
        }
        break;
        case ito::tUInt16:
        {
            ito::uint16 minVal = cv::saturate_cast<ito::uint16>((*paramsMand)[3].getVal<double>());
            ito::uint16 maxVal = cv::saturate_cast<ito::uint16>((*paramsMand)[4].getVal<double>()); 

            clipAbyBFirstHelper(cvMatComp, combBin, minVal, maxVal, outsideFlag);
            minValProt = (double)minVal;
            maxValProt = (double)maxVal;
        }
        break;
        case ito::tInt32:
        {
            ito::int32 minVal = cv::saturate_cast<ito::int32>((*paramsMand)[3].getVal<double>());
            ito::int32 maxVal = cv::saturate_cast<ito::int32>((*paramsMand)[4].getVal<double>()); 

            clipAbyBFirstHelper(cvMatComp, combBin, minVal, maxVal, outsideFlag);
            minValProt = (double)minVal;
            maxValProt = (double)maxVal;
        }
        break;
        case ito::tFloat32:
        {
            ito::float32 minVal = cv::saturate_cast<ito::float32>((*paramsMand)[3].getVal<double>());
            ito::float32 maxVal = cv::saturate_cast<ito::float32>((*paramsMand)[4].getVal<double>()); 

            clipAbyBFirstHelper(cvMatComp, combBin, minVal, maxVal, outsideFlag);
            minValProt = (double)minVal;
            maxValProt = (double)maxVal;
        }
        break;
        case ito::tFloat64:
        {
            ito::float64 minVal = cv::saturate_cast<ito::float64>((*paramsMand)[3].getVal<double>());
            ito::float64 maxVal = cv::saturate_cast<ito::float64>((*paramsMand)[4].getVal<double>()); 

            clipAbyBFirstHelper(cvMatComp, combBin, minVal, maxVal, outsideFlag);
            minValProt = (double)minVal;
            maxValProt = (double)maxVal;
        }
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("unknown type or type not implemented (e.g. complex)").toLatin1().data());
    }

    switch(dObjImages->getType())
    {
        case ito::tInt8:
        {
            ito::int8 newValue = cv::saturate_cast<ito::int8>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipAbyBSecondHelper(combBin, cvMatOut, newValue);
            }
            valProt = (double)newValue;
        }
        break;
        case ito::tUInt8:
        {
            ito::uint8 newValue = cv::saturate_cast<ito::uint8>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipAbyBSecondHelper(combBin, cvMatOut, newValue);
            }
            valProt = (double)newValue;
        }
        break;
        case ito::tInt16:
        {
            ito::int16 newValue = cv::saturate_cast<ito::int16>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipAbyBSecondHelper(combBin, cvMatOut, newValue);
            }
            valProt = (double)newValue;
        }
        break;
        case ito::tUInt16:
        {
            ito::uint16 newValue = cv::saturate_cast<ito::uint16>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipAbyBSecondHelper(combBin, cvMatOut, newValue);
            }
            valProt = (double)newValue;
        }
        break;
        case ito::tInt32:
        {
            ito::int32 newValue = cv::saturate_cast<ito::int32>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipAbyBSecondHelper(combBin, cvMatOut, newValue);
            }
            valProt = (double)newValue;
        }
        break;
        case ito::tFloat32:
        {
            ito::float32 newValue;
            double valProt = (*paramsOpt)[0].getVal<double>();
            if(ito::dObjHelper::isNaN<double>(valProt))
            {
                newValue = std::numeric_limits<ito::float32>::quiet_NaN();
            }
            else if(ito::dObjHelper::isInf<double>(valProt))
            {
                newValue = std::numeric_limits<ito::float32>::infinity();
            }
            else
            {
                newValue = cv::saturate_cast<ito::float32>(valProt);
                valProt = newValue;
            }

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipAbyBSecondHelper(combBin, cvMatOut, newValue);
            }
            valProt = newValue;
        }
        break;
        case ito::tFloat64:
        {
            ito::float64 newValue = cv::saturate_cast<ito::float64>((*paramsOpt)[0].getVal<double>());

            for(int z = 0; z < z_length; z++)
            {
                cvMatOut = ((cv::Mat *)dObjDst->get_mdata()[dObjDst->seekMat(z)]);
                clipAbyBSecondHelper(combBin, cvMatOut, newValue);
            }
            valProt = newValue;
        }
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("unknown type or type not implemented (e.g. complex)").toLatin1().data());
    }

    if(!retval.containsError())
    {
        QString msg;
        if(outsideFlag)
        {
            msg = tr("Clipped values outside %1 : %2 to %3").arg(minValProt).arg(maxValProt).arg(valProt);
        }
        else
        {
            msg = tr("Clipped values inside %1 : %2 to %3").arg(minValProt).arg(maxValProt).arg(valProt);
        }
        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail This function calculates a the histogram for a plane
   \param[in]   planeIn  Inputplane
   \param[in|out]   histOut   Preallocated histogramm buffer
   \param[in]   min   Lowest value in this histogramm
   \param[in]   max   Highest value in this histogramm
   \author ITO
   \sa  mcppfilters::calcHistParams, mcppfilters::calcHistFilter
   \date
*/
template<typename _Tp> ito::RetVal HistogrammBlock(cv::Mat *planeIn, cv::Mat *histOut, double min, double max)
{
    ito::int32 * rowPtrOut;
    rowPtrOut = (ito::int32*)histOut->ptr(0);
    memset(rowPtrOut, 0, histOut->cols * sizeof(ito::int32));
    ito::int32 x, y;
    ito::float64 indexfaktor = (histOut->cols - 1) / (max - min);

    for (y = 0; y < planeIn->rows; y++)
    {
        #if (USEOMP)
        #pragma omp parallel num_threads(NTHREADS)
        {
        #endif
        ito::int32 index;
        const _Tp* rowPtrIn;

        rowPtrIn = (_Tp*)planeIn->ptr(y);
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (x = 0; x < planeIn->cols; x++)
        {
            index = static_cast<ito::int32>((rowPtrIn[x] - min) * indexfaktor);
            if(index >= 0 && index < histOut->cols)
            {
                #if (USEOMP)
                #pragma omp atomic
                #endif
                rowPtrOut[index]++;
            }
        }
        #if (USEOMP)
        }
        #endif
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::calcHistParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Image of type Integer or float").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("(Empty) dataObject-hanlde. Will be source type later").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("Steps", ito::ParamBase::Int, 0, 2048, 0, tr("Number of steps").toLatin1().data());
    paramsOpt->append(param);

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \sa  mcppfilters::calcHistFilter, HistogrammBlock
   \date
*/
ito::RetVal BasicFilters::calcHistFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjImages = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    if (!dObjImages)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjImages->getDims() < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a matrix or image stack").toLatin1().data());
    }

    int hbins = (*paramsOpt)[0].getVal<int>();
    double maxVal = 0;
    double minVal = 0;
    bool recalcMinMax = false;

    switch(dObjImages->getType())
    {
        case ito::tInt8:
            hbins = 256;
            minVal = -128;
            maxVal = 127;
            break;
        case ito::tUInt8:
            hbins = 256;
            minVal = 0;
            maxVal = 255;
            break;
        case ito::tUInt16:
        case ito::tInt16:
        case ito::tUInt32:
        case ito::tInt32:
        case ito::tFloat32:
        case ito::tFloat64:
            if( hbins < 1 )
            {
                hbins = 1024;
            }
            recalcMinMax = true;
            break;
        default:
            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toLatin1().data());
    }

    ito::uint32 minLoc[3] = {0, 0, 0};
    ito::uint32 maxLoc[3] = {0, 0, 0};

    ito::dObjHelper::minMaxValue(dObjImages, minVal, minLoc, maxVal, maxLoc);

    int z_length = dObjImages->calcNumMats();

    int * sizesVector;
    sizesVector = (int*)calloc(dObjImages->getDims(), sizeof(int));

    for(int i = 0; i < dObjImages->getDims()-2; i++)
    {
        sizesVector[i] = dObjImages->getSize(i);
    }

    sizesVector[dObjImages->getDims()-2] = 1;
    sizesVector[dObjImages->getDims()-1] = hbins;

    ito::DataObject dObjDestination(dObjImages->getDims(), sizesVector, ito::tInt32);

    free(sizesVector);

    cv::Mat *cvMatIn;
    cv::Mat *cvMatOut;
    
    if(fabs(maxVal - minVal) < std::numeric_limits<double>::epsilon() * hbins)
    {
        if(maxVal < minVal)
        {
            minVal += (std::numeric_limits<double>::epsilon() * hbins - fabs(maxVal - minVal)) / 2.0;
            maxVal -= (std::numeric_limits<double>::epsilon() * hbins - fabs(maxVal - minVal)) / 2.0;
        }
        else
        {
            minVal -= (std::numeric_limits<double>::epsilon() * hbins - fabs(maxVal - minVal)) / 2.0;
            maxVal += (std::numeric_limits<double>::epsilon() * hbins - fabs(maxVal - minVal)) / 2.0;
        }
    }

    for(int z = 0; z < z_length; z++)
    {
        cvMatIn = ((cv::Mat *)dObjImages->get_mdata()[dObjImages->seekMat(z)]);
        cvMatOut = ((cv::Mat *)dObjDestination.get_mdata()[dObjDestination.seekMat(z)]);

        switch(dObjImages->getType())
        {
            case ito::tUInt8:
                HistogrammBlock<ito::uint8>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
            case ito::tInt8:
                HistogrammBlock<ito::int8>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
            case ito::tUInt16:
                HistogrammBlock<ito::uint16>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
            case ito::tInt16:
                HistogrammBlock<ito::int16>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
            case ito::tUInt32:
                HistogrammBlock<ito::uint32>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
            case ito::tInt32:
                HistogrammBlock<ito::int32>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
            case ito::tFloat32:
                HistogrammBlock<ito::float32>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
            case ito::tFloat64:
                HistogrammBlock<ito::float64>(cvMatIn, cvMatOut, minVal, maxVal);
            break;
        }
    }

    // Add scale and offset


    if(!retval.containsError())
    {

        // Add scale and offset for all z + (y and x)
        for(int planes = 0; planes < dObjImages->getDims()-2; planes++)
        {
            dObjDestination.setAxisOffset(planes, dObjImages->getAxisOffset(planes));
            dObjDestination.setAxisScale(planes, dObjImages->getAxisScale(planes));

            bool isValid = false;
            std::string tempDes = dObjImages->getAxisDescription(planes, isValid);
            if(isValid) dObjDestination.setAxisDescription(planes, tempDes);
            isValid = false;
            std::string tempUnit = dObjImages->getAxisUnit(planes, isValid);
            if(isValid) dObjDestination.setAxisUnit(planes, tempUnit);
        }

        dObjDestination.setAxisOffset(dObjDestination.getDims()-1, -minVal/(maxVal-minVal) * hbins);
        dObjDestination.setAxisScale(dObjDestination.getDims()-1, (maxVal-minVal) / (hbins-1));
        dObjDestination.setAxisDescription(dObjDestination.getDims()-1,std::string((*dObjImages).getValueDescription()));
        dObjDestination.setAxisUnit(dObjDestination.getDims()-1,std::string((*dObjImages).getValueUnit()));;

        // Add Protokoll
        dObjImages->copyTagMapTo(dObjDestination);
        QString msg = tr("Calculated Histogramm between %1 : %2").arg(minVal).arg(maxVal);
        dObjDestination.addToProtocol(std::string(msg.toLatin1().data()));

        *dObjDst = dObjDestination;
    }


    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::fillGeometricParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(retval.containsError()) return retval;

    param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Preallocated dataObject to be filled").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("geometricElement", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Geometric primitiv according to definition").toLatin1().data());
    paramsMand->append(param);
    param = ito::Param("insideFlag", ito::ParamBase::Int, 1, 3, 1, tr("Switch between fill inside, outside or both").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("edgeFlag", ito::ParamBase::Int, 0, 0, 0, tr("Edge-Flag, currently not used").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("newValueInside", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 1.0, tr("New value for pixels inside the geometric element").toLatin1().data());
    paramsOpt->append(param);
    param = ito::Param("newValueOutside", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("New value for pixels outside the geometric element").toLatin1().data());
    paramsOpt->append(param);
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void BasicFilters::fillGeoCircle(cv::Mat *dst, const ito::float64 x0, const ito::float64 y0, const ito::float64 radius, const bool inside, const bool outside, const _Tp insideVal, const _Tp outsideVal)
{
    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif

    _Tp* rowPtr;
    ito::int32 y;
    ito::int32 x;

    ito::float64 evalY;
    ito::float64 eval;
    ito::float64 radQ2 = radius * radius;

    #if (USEOMP)
    #pragma omp for schedule(guided)
    #endif
    for (y = 0; y < dst->rows; y++)
    {
        
        rowPtr = (_Tp*)dst->ptr(y);

        evalY = pow((ito::float64)y - y0, 2);

        for (x = 0; x < dst->cols; x++)
        {
            eval = (pow((ito::float64)x - x0, 2) + evalY);
            if(eval < radQ2)
            {
                if(inside)
                {
                    rowPtr[x] = insideVal;
                }
            }
            else
            {
                if(outside)
                {
                    rowPtr[x] = outsideVal;
                }            
            }

        }

    }
    #if (USEOMP)
    }
    #endif
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void BasicFilters::fillGeoEllipse(cv::Mat *dst, const ito::float64 x0, const ito::float64 y0, const ito::float64 radiusX, const ito::float64 radiusY, const bool inside, const bool outside, const _Tp insideVal, const _Tp outsideVal)
{
    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif

    _Tp* rowPtr;
    ito::int32 y;
    ito::int32 x;

    ito::float64 evalY;
    ito::float64 eval;

    #if (USEOMP)
    #pragma omp for schedule(guided)
    #endif
    for (y = 0; y < dst->rows; y++)
    {
        
        rowPtr = (_Tp*)dst->ptr(y);

        evalY = pow(((ito::float64)y - y0) / radiusY, 2);

        for (x = 0; x < dst->cols; x++)
        {
            eval = (pow(((ito::float64)x - x0) / radiusX, 2) + evalY);
            if(eval < 1.0)
            {
                if(inside)
                {
                    rowPtr[x] = insideVal;
                }
            }
            else
            {
                if(outside)
                {
                    rowPtr[x] = outsideVal;
                }            
            }

        }

    }
    #if (USEOMP)
    }
    #endif
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void BasicFilters::fillGeoRectangle(cv::Mat *dst, const ito::float64 x0, const ito::float64 y0, const ito::float64 x1, const ito::float64 y1, const bool inside, const bool outside, const _Tp insideVal, const _Tp outsideVal)
{
    #if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #endif

    _Tp* rowPtr;
    ito::int32 y;
    ito::int32 x;

    bool evalY;

    #if (USEOMP)
    #pragma omp for schedule(guided)
    #endif
    for (y = 0; y < dst->rows; y++)
    {
        
        rowPtr = (_Tp*)dst->ptr(y);

        evalY = (y > y0) && (y < y1);

        for (x = 0; x < dst->cols; x++)
        {
            if((x > x0) && (x < x1) && evalY)
            {
                if(inside)
                {
                    rowPtr[x] = insideVal;
                }
            }
            else
            {
                if(outside)
                {
                    rowPtr[x] = outsideVal;
                }            
            }

        }

    }
    #if (USEOMP)
    }
    #endif
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail
   \param[in|out]   paramsMand  Mandatory parameters for the filter function
   \param[in|out]   paramsOpt   Optinal parameters for the filter function
   \param[out]   outVals   Outputvalues, not implemented for this function
   \author ITO
   \date
*/
ito::RetVal BasicFilters::fillGeometricPrimitiv(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *dObjPrimitiv = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();

    if (!dObjPrimitiv)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: geometricElement ptr empty").toLatin1().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toLatin1().data());
    }

    if (dObjDst->getDims() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source is not a 2d matrix").toLatin1().data());
    }

    if (dObjPrimitiv->getDims() != 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: geometricElement is not a 2d matrix").toLatin1().data());
    }

    if (dObjPrimitiv->getType() != ito::tFloat32 && dObjPrimitiv->getType() != ito::tFloat64)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: geometricElement must be either float32 or float64").toLatin1().data());
    }

    //bool check;
    ito::int32 xDim = 1;
    ito::int32 yDim = 0;
    ito::int32 type = 0; 

    // Type 1 Structure will be dataObject([1, 11], 'float32') with [idx, type, x, y, 0, 0, 0, 0]
    // Type 2 Structure will be dataObject([8, 1], 'float32') with [[idx], [type], [x], [y], [0], [0], [0],[0]]

    ito::float64 x0;
    ito::float64 x1;
    ito::float64 y0;
    ito::float64 y1;
    ito::float64 rA;
    ito::float64 rB;

    ito::float64 insideVal = (*paramsOpt)[2].getVal<double>();
    ito::float64 outsideVal = (*paramsOpt)[3].getVal<double>();

    ito::float64 xScale = dObjDst->getAxisScale(xDim);
    ito::float64 yScale = dObjDst->getAxisScale(yDim);
    ito::float64 xOffset = dObjDst->getAxisOffset(xDim);
    ito::float64 yOffset = dObjDst->getAxisOffset(yDim);

    bool inFlag = (*paramsOpt)[0].getVal<int>() & 0x01;
    bool outFlag = (*paramsOpt)[0].getVal<int>() & 0x02;

    if(dObjPrimitiv->getSize(0) == 1 && dObjPrimitiv->getSize(1) > 8)
    {
        if(dObjPrimitiv->getType() == ito::tFloat32) type = cv::saturate_cast<int>(dObjPrimitiv->at<ito::float32>(0, 1));
        else type = cv::saturate_cast<int>(dObjPrimitiv->at<ito::float64>(0, 1));


        switch(type & 0x000000FF)
        {
            default:
                return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            case ito::PrimitiveContainer::tCircle:
                if(dObjPrimitiv->getType() == ito::tFloat32)
                {
                    x0 = dObjPrimitiv->at<ito::float32>(0, 2);
                    y0 = dObjPrimitiv->at<ito::float32>(0, 3);
                    rA = dObjPrimitiv->at<ito::float32>(0, 5);
                    rB = dObjPrimitiv->at<ito::float32>(0, 5);
                }
                else
                {
                    x0 = dObjPrimitiv->at<ito::float64>(0, 2);
                    y0 = dObjPrimitiv->at<ito::float64>(0, 3);
                    rA = dObjPrimitiv->at<ito::float64>(0, 5);
                    rB = dObjPrimitiv->at<ito::float64>(0, 5);
                }
                break;
            case ito::PrimitiveContainer::tEllipse:
                if(dObjPrimitiv->getType() == ito::tFloat32)
                {
                    x0 = dObjPrimitiv->at<ito::float32>(0, 2);
                    y0 = dObjPrimitiv->at<ito::float32>(0, 3);
                    rA = dObjPrimitiv->at<ito::float32>(0, 5);
                    rB = dObjPrimitiv->at<ito::float32>(0, 6);
                }
                else
                {
                    x0 = dObjPrimitiv->at<ito::float64>(0, 2);
                    y0 = dObjPrimitiv->at<ito::float64>(0, 3);
                    rA = dObjPrimitiv->at<ito::float64>(0, 5);
                    rB = dObjPrimitiv->at<ito::float64>(0, 6);
                }
                break;
            case ito::PrimitiveContainer::tRectangle:
                if(dObjPrimitiv->getType() == ito::tFloat32)
                {
                    x0 = dObjPrimitiv->at<ito::float32>(0, 2);
                    y0 = dObjPrimitiv->at<ito::float32>(0, 3);
                    x1 = dObjPrimitiv->at<ito::float32>(0, 5);
                    y1 = dObjPrimitiv->at<ito::float32>(0, 6);
                }
                else
                {
                    x0 = dObjPrimitiv->at<ito::float64>(0, 2);
                    y0 = dObjPrimitiv->at<ito::float64>(0, 3);
                    x1 = dObjPrimitiv->at<ito::float64>(0, 5);
                    y1 = dObjPrimitiv->at<ito::float64>(0, 6);
                }
                break;
            case ito::PrimitiveContainer::tSquare:
                if(dObjPrimitiv->getType() == ito::tFloat32)
                {
                    x0 = dObjPrimitiv->at<ito::float32>(0, 2) - dObjPrimitiv->at<ito::float32>(0, 5) / 2.0;
                    y0 = dObjPrimitiv->at<ito::float32>(0, 3) - dObjPrimitiv->at<ito::float32>(0, 5) / 2.0;
                    x1 = dObjPrimitiv->at<ito::float32>(0, 2) + dObjPrimitiv->at<ito::float32>(0, 5) / 2.0;
                    y1 = dObjPrimitiv->at<ito::float32>(0, 3) + dObjPrimitiv->at<ito::float32>(0, 5) / 2.0;
                }
                else
                {
                    x0 = dObjPrimitiv->at<ito::float64>(0, 2) - dObjPrimitiv->at<ito::float64>(0, 5) / 2.0;
                    y0 = dObjPrimitiv->at<ito::float64>(0, 3) - dObjPrimitiv->at<ito::float64>(0, 5) / 2.0;
                    x1 = dObjPrimitiv->at<ito::float64>(0, 2) + dObjPrimitiv->at<ito::float64>(0, 5) / 2.0;
                    y1 = dObjPrimitiv->at<ito::float64>(0, 3) + dObjPrimitiv->at<ito::float64>(0, 5) / 2.0;
                }
                break;
        }

    }
    else if(dObjPrimitiv->getSize(0) > 6 && dObjPrimitiv->getSize(1) == 1)
    {
        if(dObjPrimitiv->getType() == ito::tFloat32) type = cv::saturate_cast<int>(dObjPrimitiv->at<ito::float32>(1, 0));
        else type = cv::saturate_cast<int>(dObjPrimitiv->at<ito::float64>(1, 0));

        switch(type & 0x000000FF)
        {
            default:
                return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            case ito::PrimitiveContainer::tCircle:
            case ito::PrimitiveContainer::tEllipse:
                if(dObjPrimitiv->getType() == ito::tFloat32)
                {
                    x0 = (dObjPrimitiv->at<ito::float32>(2, 0) + dObjPrimitiv->at<ito::float32>(4, 0)) / 2.0;
                    y0 = (dObjPrimitiv->at<ito::float32>(3, 0) + dObjPrimitiv->at<ito::float32>(5, 0)) / 2.0;
                    rA = fabs(dObjPrimitiv->at<ito::float32>(2, 0) - dObjPrimitiv->at<ito::float32>(4, 0)) / 2.0;
                    rB = fabs(dObjPrimitiv->at<ito::float32>(3, 0) - dObjPrimitiv->at<ito::float32>(5, 0)) / 2.0;
                }
                else
                {
                    x0 = (dObjPrimitiv->at<ito::float64>(2, 0) + dObjPrimitiv->at<ito::float64>(4, 0)) / 2.0;
                    y0 = (dObjPrimitiv->at<ito::float64>(3, 0) + dObjPrimitiv->at<ito::float64>(5, 0)) / 2.0;
                    rA = fabs(dObjPrimitiv->at<ito::float64>(2, 0) - dObjPrimitiv->at<ito::float64>(4, 0)) / 2.0;
                    rB = fabs(dObjPrimitiv->at<ito::float64>(3, 0) - dObjPrimitiv->at<ito::float64>(5, 0)) / 2.0;
                }
                break;
            case ito::PrimitiveContainer::tRectangle:
            case ito::PrimitiveContainer::tSquare:
                if(dObjPrimitiv->getType() == ito::tFloat32)
                {
                    x0 = dObjPrimitiv->at<ito::float32>(2, 0);
                    y0 = dObjPrimitiv->at<ito::float32>(3, 0);
                    x1 = dObjPrimitiv->at<ito::float32>(4, 0);
                    y1 = dObjPrimitiv->at<ito::float32>(5, 0);
                }
                else
                {
                    x0 = dObjPrimitiv->at<ito::float64>(2, 0);
                    y0 = dObjPrimitiv->at<ito::float64>(3, 0);
                    x1 = dObjPrimitiv->at<ito::float64>(4, 0);
                    y1 = dObjPrimitiv->at<ito::float64>(5, 0);
                }
                break;
        }
    }
    else 
    {
        return ito::RetVal(ito::retError, 0, tr("Error: geometricElement must be either marker-style (8x1) or primitiv-style (1x11)").toLatin1().data());
    }


    switch(type & 0x000000FF)
    {
        case ito::PrimitiveContainer::tCircle:
        case ito::PrimitiveContainer::tEllipse:
            x0 = x0 / xScale + xOffset;
            y0 = y0 / yScale + yOffset;
            rA = rA / xScale;
            rB = rB / yScale;
            y1 = 0.0;
            x1 = 0.0;
            if(fabs(rA-rB) < dObjDst->getAxisScale(xDim) && fabs(rA-rB) < dObjDst->getAxisScale(yDim)) type = ito::PrimitiveContainer::tCircle;
            else type = ito::PrimitiveContainer::tEllipse;

            if(!ito::dObjHelper::isNotZero(rA) || !ito::dObjHelper::isNotZero(rB))  
            {
                return ito::RetVal(ito::retError, 0, tr("Error: radii of geometricElement must not be zero").toLatin1().data());
            }
            break;
        case ito::PrimitiveContainer::tRectangle:
        case ito::PrimitiveContainer::tSquare:
            x0 = x0 / xScale + xOffset;
            y0 = y0 / yScale + yOffset;
            x1 = x1 / xScale + xOffset;
            y1 = y1 / yScale + yOffset;
            rA = 0.0;
            rB = 0.0;
            break;
    }

    cv::Mat* myMat = (cv::Mat*)(dObjDst->get_mdata()[dObjDst->seekMat(0)]);

    if(!ito::dObjHelper::isFinite(rA) || !ito::dObjHelper::isFinite(rB) || !ito::dObjHelper::isFinite(x0) || !ito::dObjHelper::isFinite(y1) || !ito::dObjHelper::isFinite(x1) || !ito::dObjHelper::isFinite(y0))  
    {
        return ito::RetVal(ito::retError, 0, tr("Error: coordinates of geometricElement must be finite").toLatin1().data());
    }

    switch(dObjDst->getType())
    {
        case ito::tUInt8:
        {
            ito::uint8 inVal = cv::saturate_cast<ito::uint8>(insideVal);
            ito::uint8 outVal = cv::saturate_cast<ito::uint8>(outsideVal);

            switch(type & 0x000000FF)
            {
                case ito::PrimitiveContainer::tCircle:
                    fillGeoCircle(myMat, x0, y0, rA, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tEllipse:
                    fillGeoEllipse(myMat, x0, y0, rA, rB, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tRectangle:
                case ito::PrimitiveContainer::tSquare:
                    fillGeoRectangle(myMat, x0, y0, x1, y1, inFlag, outFlag, inVal, outVal);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            }
            break;
        }
        case ito::tInt8:
        {
            ito::int8 inVal = cv::saturate_cast<ito::int8>(insideVal);
            ito::int8 outVal = cv::saturate_cast<ito::int8>(outsideVal);

            switch(type & 0x000000FF)
            {
                case ito::PrimitiveContainer::tCircle:
                    fillGeoCircle(myMat, x0, y0, rA, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tEllipse:
                    fillGeoEllipse(myMat, x0, y0, rA, rB, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tRectangle:
                case ito::PrimitiveContainer::tSquare:
                    fillGeoRectangle(myMat, x0, y0, x1, y1, inFlag, outFlag, inVal, outVal);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            }
            break;
        }
        case ito::tUInt16:
        {
            ito::uint16 inVal = cv::saturate_cast<ito::uint16>(insideVal);
            ito::uint16 outVal = cv::saturate_cast<ito::uint16>(outsideVal);

            switch(type & 0x000000FF)
            {
                case ito::PrimitiveContainer::tCircle:
                    fillGeoCircle(myMat, x0, y0, rA, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tEllipse:
                    fillGeoEllipse(myMat, x0, y0, rA, rB, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tRectangle:
                case ito::PrimitiveContainer::tSquare:
                    fillGeoRectangle(myMat, x0, y0, x1, y1, inFlag, outFlag, inVal, outVal);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            }
            break;
        }
        case ito::tInt16:
        {
            ito::int16 inVal = cv::saturate_cast<ito::int16>(insideVal);
            ito::int16 outVal = cv::saturate_cast<ito::int16>(outsideVal);

            switch(type & 0x000000FF)
            {
                case ito::PrimitiveContainer::tCircle:
                    fillGeoCircle(myMat, x0, y0, rA, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tEllipse:
                    fillGeoEllipse(myMat, x0, y0, rA, rB, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tRectangle:
                case ito::PrimitiveContainer::tSquare:
                    fillGeoRectangle(myMat, x0, y0, x1, y1, inFlag, outFlag, inVal, outVal);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            }
            break;
        }
        case ito::tInt32:
        {
            ito::int32 inVal = cv::saturate_cast<ito::int32>(insideVal);
            ito::int32 outVal = cv::saturate_cast<ito::int32>(outsideVal);

            switch(type & 0x000000FF)
            {
                case ito::PrimitiveContainer::tCircle:
                    fillGeoCircle(myMat, x0, y0, rA, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tEllipse:
                    fillGeoEllipse(myMat, x0, y0, rA, rB, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tRectangle:
                case ito::PrimitiveContainer::tSquare:
                    fillGeoRectangle(myMat, x0, y0, x1, y1, inFlag, outFlag, inVal, outVal);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            }
            break;
        }
        case ito::tFloat32:
        {
            ito::float32 inVal = cv::saturate_cast<ito::float32>(insideVal);
            ito::float32 outVal = cv::saturate_cast<ito::float32>(outsideVal);

            switch(type & 0x000000FF)
            {
                case ito::PrimitiveContainer::tCircle:
                    fillGeoCircle(myMat, x0, y0, rA, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tEllipse:
                    fillGeoEllipse(myMat, x0, y0, rA, rB, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tRectangle:
                case ito::PrimitiveContainer::tSquare:
                    fillGeoRectangle(myMat, x0, y0, x1, y1, inFlag, outFlag, inVal, outVal);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            }
            break;
        }
        case ito::tFloat64:
        {
            ito::float64 inVal = cv::saturate_cast<ito::float64>(insideVal);
            ito::float64 outVal = cv::saturate_cast<ito::float64>(outsideVal);

            switch(type & 0x000000FF)
            {
                case ito::PrimitiveContainer::tCircle:
                    fillGeoCircle(myMat, x0, y0, rA, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tEllipse:
                    fillGeoEllipse(myMat, x0, y0, rA, rB, inFlag, outFlag, inVal, outVal);
                    break;
                case ito::PrimitiveContainer::tRectangle:
                case ito::PrimitiveContainer::tSquare:
                    fillGeoRectangle(myMat, x0, y0, x1, y1, inFlag, outFlag, inVal, outVal);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("Error: geometric primitiv not supported for filling").toLatin1().data());
            }
            break;
        }
        default:
            return ito::RetVal(ito::retError, 0, tr("Error: destination object type is not suppirted for filling").toLatin1().data());
    }

    return retval;
}
