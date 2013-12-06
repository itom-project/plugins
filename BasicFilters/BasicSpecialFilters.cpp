/*! \file itomspecialfilter.cpp
   \brief   This file contains special filter.

   \author ITO
   \date 01.2012
*/

#include <math.h>

#include "BasicFilters.h"

#include "DataObject/dataObjectFuncs.h"


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
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toAscii().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toAscii().data());
    }

    if (dObjSrc->getDims()  != 3 )
    { 
        return ito::RetVal(ito::retError, 0, tr("Error: Input image must be 3D").toAscii().data());
    }

    if (dObjSrc->getSize(1) != 1 && dObjSrc->getSize(2) != 1) 
    { 
        return ito::RetVal(ito::retError, 0, tr("Error: one dimension of input image must be equal to 1").toAscii().data());
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
            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toAscii().data());
    }

    *dObjDst = tempObj;

    if(!retval.containsError())
    {
        // Add scale and offset for all z + (y and x)
//        char prot[81] = {0};
//        _snprintf(prot, 80, "Flattened object from 3d to 2d");
//        dObjDst->addToProtocol(std::string(prot));
        QString msg = tr("Flattened object from 3d to 2d");
        dObjDst->addToProtocol(std::string(msg.toAscii().data()));
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
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toAscii().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toAscii().data());
    }

    if (dObjSrc->getDims()  != 2 )
    { 
        return ito::RetVal(ito::retError, 0, tr("Error: Input image must be 2D").toAscii().data());
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
            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toAscii().data());
    }

    if(!retval.containsError())
    {
        // Add scale and offset for all z + (y and x)
//        char prot[81] = {0};
//        _snprintf(prot, 80, "Flattened object from 3d to 2d");
//        dObjDst->addToProtocol(std::string(prot));
        QString msg = tr("Swap byte order");
        dObjDst->addToProtocol(std::string(msg.toAscii().data()));
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
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toAscii().data());
    }

    if (!dObjReplace)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: replace image ptr empty").toAscii().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: dest image ptr empty").toAscii().data());
    }

    if(!ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjReplace))
    {
        return ito::RetVal(ito::retError, 0, tr("source and replace image must have the same type and size").toAscii().data());
    }

    if(dObjSrc->getType() != ito::tFloat32 && dObjSrc->getType() != ito::tFloat64)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: this filter is only usable for float or double matrices").toAscii().data());
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
        dObjDst->addToProtocol(std::string(msg.toAscii().data()));
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
        ito::Param param = ito::Param("scrImg", ito::ParamBase::DObjPtr, NULL, tr("Input image").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("replaceImg", ito::ParamBase::DObjPtr, NULL, tr("Image with values which will be used for replacement").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("destImg", ito::ParamBase::DObjPtr, NULL, tr("Output image").toAscii().data());
        paramsMand->append(param);

        paramsOut->append( ito::Param("nrOfReplacements", ito::ParamBase::Int | ito::ParamBase::Out, 0, NULL, tr("number of replacments").toAscii().data()));
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::mergeColorPlanesParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("scrImg", ito::ParamBase::DObjPtr, NULL, tr("Input image with 3 or 4 uint8 planes").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("destImg", ito::ParamBase::DObjPtr, NULL, tr("Output image with uint32 planes").toAscii().data());
        paramsMand->append(param);

        param = ito::Param("toogleByteOrder", ito::ParamBase::Int, 0, 3, 0, tr("Switch between RGBA = 0, BGRA = 1, ARGB = 2, ABGR = 3").toAscii().data());
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

    int toogleByteOrder = (int)(*paramsOpt)[0].getVal<int>();

    if (!dObjSrc)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: source image ptr empty").toAscii().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: replace image ptr empty").toAscii().data());
    }

    int numMats = dObjSrc->calcNumMats();

    if(numMats == 3 && toogleByteOrder > 1)
    {
        toogleByteOrder -= 2;
    }

    if(dObjSrc->getType() != ito::tUInt8 && dObjSrc->getDims() != 3 && (numMats != 3 || numMats != 4))
    {
        return ito::RetVal(ito::retError, 0, tr("Error: The primary object must be of type tUInt8").toAscii().data());
    }

    size_t planeSize[2] = {dObjSrc->getSize(1), dObjSrc->getSize(2)};
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

    switch(toogleByteOrder)
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

        for( size_t y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrA = matA->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::int32>(y);
            for( size_t x = 0; x < planeSize[1]; x++)
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

        for( size_t y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::int32>(y);

            for( size_t x = 0; x < planeSize[1]; x++)
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

        for( size_t y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrA = matA->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::Rgba32>(y);

            for( size_t x = 0; x < planeSize[1]; x++)
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

        for( size_t y = 0; y < planeSize[0]; y++)
        {
            rowPtrR = matR->ptr<ito::uint8>(y);
            rowPtrG = matG->ptr<ito::uint8>(y);
            rowPtrB = matB->ptr<ito::uint8>(y);
            rowPtrDst = matRes->ptr<ito::Rgba32>(y);

            for( size_t x = 0; x < planeSize[1]; x++)
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
        dObjDst->addToProtocol(std::string(msg.toAscii().data()));
    }

    return retval;

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::calcMeanOverZParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("scrImg", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Input image with 3 or 4 uint8 planes").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("destImg", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Output image with uint32 planes").toAscii().data());
        paramsMand->append(param);

        param = ito::Param("ignoreInf", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Ignore invalid-Values for floating point").toAscii().data());
        paramsOpt->append(param);
    }

    return retval;
}
template<typename _Type, typename _TypeDst> void calcMeanOverZHelp(_Type ***scrPtr, cv::Mat *dstMat, const int *sizes, const bool toogleInf)
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
                    value += (ito::float64)(scrPtr[z][y][x]);
                    cnts++;
                }
                linePtr[x] = cv::saturate_cast<_TypeDst>(value / cnts);
            }
        }
    }
    else
    {
        if(toogleInf)
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
                        if(ito::dObjHelper::isFinite<_Type>(scrPtr[z][y][x]))
                        {
                            value += (ito::float64)(scrPtr[z][y][x]);
                            cnts++;                    
                        }
                    }
                    if(cnts > 0.0)
                    {
                        linePtr[x] = cv::saturate_cast<_TypeDst>(value / cnts);
                    }
                    else
                    {
                        linePtr[x] = std::numeric_limits<_TypeDst>::signaling_NaN();
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
                        value += (ito::float64)(scrPtr[z][y][x]);
                        cnts++;                    
                    }
                    if(cnts > 0.0)
                    {
                        linePtr[x] = cv::saturate_cast<_TypeDst>(value / cnts);
                    }
                    else
                    {
                        linePtr[x] = std::numeric_limits<_TypeDst>::signaling_NaN();
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
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImageStack is Null-Pointer").toAscii().data());
    }
    
    bool toogleInf = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;

    xsize = dObjSrc->getSize(dObjSrc->getDims() - 1);
    ysize = dObjSrc->getSize(dObjSrc->getDims() - 2);

    retval += ito::dObjHelper::verify3DDataObject(dObjSrc, "sourceImageStack", 2, 256,  ysize, ysize, xsize, xsize, 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if(retval.containsError())
    {
        return retval;
    }

    if(dObjDst == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, tr("destinationPlane is a uninitialized dataObject!").toAscii().data());
    }
    else if(!retval.containsError())
    {
        if(dObjDst != dObjSrc)
        {
            ito::RetVal tRetval = ito::dObjHelper::verify2DDataObject(dObjDst, "destinationPlane", ysize, ysize, xsize, xsize,  1, dObjSrc->getType());
            if(tRetval.containsError())
            {
                destPlane = ito::DataObject(ysize, xsize, dObjSrc->getType());
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
            ito::int8 ***scrPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::int8>(dObjSrc, scrPtr);
            calcMeanOverZHelp<ito::int8, ito::int8>(scrPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toogleInf);
            ito::dObjHelper::freeRowPointer<ito::int8>(scrPtr);
        }
        break;
        case ito::tUInt8:
        {
            ito::uint8 ***scrPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::uint8>(dObjSrc, scrPtr);
            calcMeanOverZHelp<ito::uint8, ito::uint8>(scrPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toogleInf);
            ito::dObjHelper::freeRowPointer<ito::uint8>(scrPtr);
        }
        break;
        case ito::tInt16:
        {
            ito::int16 ***scrPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::int16>(dObjSrc, scrPtr);
            calcMeanOverZHelp<ito::int16, ito::int16>(scrPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toogleInf);
            ito::dObjHelper::freeRowPointer<ito::int16>(scrPtr);
        }
        break;
        case ito::tUInt16:
        {
            ito::uint16 ***scrPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::uint16>(dObjSrc, scrPtr);
            calcMeanOverZHelp<ito::uint16, ito::uint16>(scrPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toogleInf);
            ito::dObjHelper::freeRowPointer<ito::uint16>(scrPtr);
        }
        break;
        case ito::tInt32:
        {
            ito::int32 ***scrPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::int32>(dObjSrc, scrPtr);
            calcMeanOverZHelp<ito::int32, ito::int32>(scrPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toogleInf);
            ito::dObjHelper::freeRowPointer<ito::int32>(scrPtr);
        }
        break;
        case ito::tFloat32:
        {
            ito::float32 ***scrPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::float32>(dObjSrc, scrPtr);
            calcMeanOverZHelp<ito::float32, ito::float32>(scrPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toogleInf);
            ito::dObjHelper::freeRowPointer<ito::float32>(scrPtr);
        }
        break;
        case ito::tFloat64:
        {
            ito::float64 ***scrPtr = NULL;
            ito::dObjHelper::getRowPointer<ito::float64>(dObjSrc, scrPtr);
            calcMeanOverZHelp<ito::float64, ito::float64>(scrPtr, (cv::Mat*)(destPlane.get_mdata()[0]), sizes, toogleInf);
            ito::dObjHelper::freeRowPointer<ito::float64>(scrPtr);
        }
        break;
        default:
            return ito::RetVal(ito::retError, 0, tr("Unknown type or type not implemented").toAscii().data());
    }

    if(!retval.containsError())
    {
        dObjSrc->copyTagMapTo(destPlane);
        ito::dObjHelper::dObjCopyLastNAxisTags(*dObjSrc, destPlane, 2, true, true);
        QString msg = tr("Calculated mean value in z-Direction from 3D-Object");
        destPlane.addToProtocol(std::string(msg.toAscii().data()));
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
        ito::Param param = ito::Param("scrImg", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D image or single plane n-D object").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("dstSlice", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Slice with output data").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("x0", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 0.0, tr("x0-coordinate for slice").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("y0", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 1.0, tr("y0-coordinate for slice").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("x1", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 0.0, tr("x1-coordinate for slice").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("y1", ito::ParamBase::Double, -std::numeric_limits<ito::float32>::max(), std::numeric_limits<ito::float32>::max(), 1.0, tr("y1-coordinate for slice").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("interpolation mode", ito::ParamBase::Int, 0, 0, 0, tr("0: Bresenham or Nearest, 1: weighted").toAscii().data());
        paramsOpt->append(param);
        //param = ito::Param("numer of pixel", ito::ParamBase::Int, 0, 65366, 0, tr("Number of pixels for output image").toAscii().data());
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
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is Null-Pointer").toAscii().data());
    }

    if (!dObjDst)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage is Null-Pointer").toAscii().data());
    }
   
    ito::int32 dims = dObjSrc->getDims();

    if(dims < 2)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage must have at least 2 dimensions").toAscii().data());
    }

    if(dObjSrc->calcNumMats() > 1)
    {
        return ito::RetVal(ito::retError, 0, tr("Error: sourceImage must not have more than 1 plane").toAscii().data());
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

    cv::Mat* scrMat = (cv::Mat*)dObjSrc->get_mdata()[ dObjSrc->seekMat(0) ]; //first plane in ROI

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

            matOffset    = scrMat->step[0] * pxY0 + scrMat->step[1] * pxX0;
            matStepSize.fill(scrMat->step[0], sliceXSize);
        }
        else
        {
            sliceXSize   = 1 + pxY0 - pxY1;
            startPhys    = dObjSrc->getPixToPhys(dims-2, pxY1, _unused); 
            startPx      = pxY1;
            right        = dObjSrc->getPixToPhys(dims-2, pxY0, _unused);
            stepSizePhys = sliceXSize > 1 ? (right - startPhys) / (ito::float64)(sliceXSize - 1) : 0.0;

            matStepSize.resize(sliceXSize);

            matOffset    = scrMat->step[0] * pxY1 + scrMat->step[1] * pxX0;
            matStepSize.fill(scrMat->step[0], sliceXSize);
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

            matOffset = scrMat->step[0] * pxY0 + scrMat->step[1] * pxX0;
            matStepSize.fill(scrMat->step[1], sliceXSize);
        }
        else
        {
            sliceXSize   = 1 + pxX0 - pxX1;
            startPhys    = dObjSrc->getPixToPhys(dims-1, pxX1, _unused);
            startPx      = pxX1;
            right        = dObjSrc->getPixToPhys(dims-1, pxX0, _unused); 
            stepSizePhys = sliceXSize > 1 ? (right - startPhys) / (ito::float64)(sliceXSize - 1) : 0.0;

            matStepSize.resize(sliceXSize);

            matOffset = scrMat->step[0] * pxY0 + scrMat->step[1] * pxX1; 
            matStepSize.fill(scrMat->step[1], sliceXSize);
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

            matOffset = scrMat->step[0] * pxY0 + scrMat->step[1] * pxX0; 

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

            for(unsigned int n = 0; n < sliceXSize; n++)
            {  /* loop */
                //setPixel(x,y)
                matStepSize[n] = scrMat->step[0] * y + scrMat->step[1] * x;

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
        retval += ito::RetVal(ito::retError, 0, tr("slice has not defined size").toAscii().data());
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
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::int8*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tInt16:
            {
                ito::int16 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::int16>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::int16*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tInt32:
            {
                ito::int32 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::int32>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::int32*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tUInt8:
            {
                ito::uint8 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::uint8>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::uint8*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tUInt16:
            {
                ito::uint16 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::uint16>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::uint16*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tFloat32:
            {
                ito::float32 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::float32>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::float32*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tFloat64:
            {
                ito::float64 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::float64>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::float64*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tComplex64:
            {
                ito::complex64 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::complex64>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::complex64*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tComplex128:
            {
                ito::complex128 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::complex128>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x] = *(reinterpret_cast<ito::complex128*>(scrPtr));
                    scrPtr += matStepSize[x];
                }
            }
            break;
            case ito::tRGBA32:
            {
                ito::RgbaBase32 *dstPtr = ((cv::Mat*)(tempObj.get_mdata()[0]))->ptr<ito::Rgba32>(0);
                uchar *scrPtr = scrMat->data + matOffset;

                for(int x = 0; x < sliceXSize; x++)
                {
                    dstPtr[x].rgba = (reinterpret_cast<ito::RgbaBase32*>(scrPtr))->rgba;
                    scrPtr += matStepSize[x];
                }            
            }
            break;
            default:
                retval += ito::RetVal(ito::retError, 0, tr("datatype not supported").toAscii().data());
                break;
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, tr("matrix step vector for matrix is empty").toAscii().data());
    }

    if(!retval.containsError())
    {
        dObjSrc->copyTagMapTo(tempObj);
        if(needNewObj)
        {
            *dObjDst = tempObj;
        }
        QString msg = tr("Cut 1D slice out of 2D-data from [ %2, %3] to [ %4, %5]").arg(physX0).arg(physY0).arg(physX1).arg(physY1);
        dObjDst->addToProtocol(std::string(msg.toAscii().data()));
        dObjDst->setAxisDescription(1, axisDescription);
        dObjDst->setAxisUnit(1, axisUnit);

        if(ito::dObjHelper::isNotZero(stepSizePhys)) dObjDst->setAxisScale(1, stepSizePhys);
        else dObjDst->setAxisScale(1, stepSizePhys);

        dObjDst->setAxisOffset(1, startPx);
    }

    return retval;
}
