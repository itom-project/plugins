/*! \file BasicGenericFilters.cpp
\brief   This file contains the generic filter engine.

\author twip optical solutions
\date 12.2013
*/

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant

#include "DataObject/dataObjectFuncs.h"
#include "BasicFilters.h"
#define useomp 1

//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
/*! \fn Get
\brief   This function copies a part of the cv:Mat plane to buf according to filterparameters
\detail  This function copies data from an integer type cv::Mat to the int32-buffer buf. The number of elements to copie depends line size and the kernelsize.
After the copy procedure, the data is checked for invalids and if invalid are found, they are exchanged by the last valid value.
\param[in]   plane     Inputdataplane
\param[in]   x0  Startvalue of copyprocedure
\param[in]   y0  Current line to copy
\param[in]   dx  Number of elements to copy
\param[in|out]   buf     Buffer of type current type _Tp     (pointer)
\param[in|out]   inv     Map of invalid pixel from type char (pointer)
\param[in]   kern        Size of the kernel in x
\param[in]   invalid     value of invalid pixels
\author  ITO
\sa  GenericFilterEngine::runFilter
\date    12.2011
*/


template<typename _Tp> void Get(cv::Mat *plane, const ito::int32 x0, const ito::int32 y0, ito::int32 dx, _Tp *buf, ito::int8 *inv, const ito::int32 kern, const ito::float64 /*invalid*/ )
{
    ito::int32 a = 0, b = 0, i;
    ito::int32 lastval;
    ito::int32 x = x0; 
    ito::int32 y = y0;

    //bool check = ito::dObjHelper::isFinite(invalid) && (invalid < std::numeric_limits<ito::int32>::max()) && (invalid > std::numeric_limits<ito::int32>::min());
    //ito::int32 invalidInt = cv::saturate_cast<ito::int32>(invalid);
    
    // Check if y colidates with image boarder
    if (y < 0)
    {
        y = 0;
    }

    // Check if y colidates with image boarder
    if (y >= plane->rows)//o->sizes[1])
    {
        y = plane->rows - 1;
    }

    memset(inv, 1, dx);
    //memset(inv, 0, dx);
    // Check if x or dx colidate with image boarder
    
    if (x < 0)
    {
        a = -x0;
        x = 0;
        dx -= a;
    }

    // Check if x or dx colidate with image boarder
    if (x + dx > plane->cols)
    {
        b = x + dx - plane->cols;
        dx = plane->cols - x;
    }
    // Copy a line with data from plane to buffer
    memcpy((void*)&buf[a], (void*)&(plane->ptr<_Tp>(y)[x]), dx * sizeof(_Tp));
    //memcpy((void*)&buf[a], (_Tp*)&((plane->ptr(y))[x]), dx * sizeof(_Tp));

    lastval =- kern;

/*
    if(check)
    {
        for (i = 0; i < dx; i++)
        {
            if (buf[i + a] == invalidInt)
            {
                max = kern;
                v = 0;
                if (i + a - lastval < max)
                {
                    max = i + a - lastval;
                    v = buf[lastval];
                }
                for (k = 1; k < max; k++)
                {
                    if (k + i >= dx)
                        break;
                    if (buf[i + a + k] != invalidInt)
                    {
                        max = k;
                        v = buf[i + a + k];
                        break;
                    }
                }
                for (k = 1; k < max; k++)
                {
                    if ((y + k >= 0) && (y + k < plane->rows))
                    {
                        w = plane->at<_Tp>(y + k, i + x);
                        if (w != invalidInt)
                        {
                            v = w;
                            break;
                        }
                    }
                    if ((y - k >= 0) && (y - k < plane->rows))
                    {
                        w = plane->at<_Tp>(y - k, i + x);
                        if(w != invalidInt)
                        {
                            v = w;
                            break;
                        }
                    }
                }
                buf[i + a] = v;
            }
            else
            {
                inv[i + a] = 1;
                lastval = i + a;
            }
        }
    }
*/
 
    for (i = 0; i < a; i++)
    {
        buf[i] = buf[a];
    }

    //memcpy((void*)buf[i], (void*)buf[a], a * sizeof(_Tp));


    for (i = 0; i < b; i++)
    {
        buf[a + dx + i] = buf[a + dx - 1];
    }

    //memcpy((void*)buf[a + dx + i], (void*)buf[a + dx - 1], n * sizeof(_Tp));
}
//----------------------------------------------------------------------------------------------------------------------------------
template<> void Get<ito::float32>(cv::Mat *plane, const ito::int32 x0, const ito::int32 y0, ito::int32 dx, ito::float32 *buf, ito::int8 *inv, const ito::int32 kern, const ito::float64 /*invalid*/ )
{
    ito::int32 a = 0, b = 0, i, k;
    ito::float32 v, w;
    ito::int32 lastval, max;
    ito::int32 x = x0; 
    ito::int32 y = y0;

    //bool check = ito::dObjHelper::isFinite(invalid) && (invalid < std::numeric_limits<ito::int32>::max()) && (invalid > std::numeric_limits<ito::int32>::min());    
    // Check if y colidates with image boarder
    if (y < 0)
    {
        y = 0;
    }

    // Check if y colidates with image boarder
    if (y >= plane->rows)//o->sizes[1])
    {
        y = plane->rows - 1;
    }

    //prefill invalid map with all invalids
    memset(inv, 0, dx);

    // Check if x or dx colidate with image boarder
    if (x < 0)
    {
        a = -x0;
        x = 0;
        dx -= a;
    }

    // Check if x or dx colidate with image boarder
    if (x + dx > plane->cols)
    {
        b = x + dx - plane->cols;
        dx = plane->cols - x;
    }
    // Copy a line with data from plane to buffer
    //memcpy((void*)&buf[a], (_Tp*)&((plane->ptr(y))[x]), dx * sizeof(_Tp));
    memcpy((void*)&buf[a], (void*)&(plane->ptr<ito::float32>(y)[x]), dx * sizeof(ito::float32));

    lastval =- kern;

    // Do the invalid check for each pixel
 
    for (i = 0; i < dx; i++)
    {
        if (ito::dObjHelper::isFinite<ito::float32>(buf[i + a]) /*|| buf[i + a] == invalidInt*/)
        {
            inv[i + a] = 1;
            lastval = i + a;
        }
        else
        {
            max = kern;
            v = 0;
            if (i + a - lastval < max)
            {
                max = i + a - lastval;
                v = buf[lastval];
            }
            for (k = 1; k < max; k++)
            {
                if (k + i >= dx)
                    break;
                if (ito::dObjHelper::isFinite<ito::float32>(buf[i + a + k]))
                {
                    max = k;
                    v = buf[i + a + k];
                    break;
                }
            }
            for (k = 1; k < max; k++)
            {
                if ((y + k >= 0) && (y + k < plane->rows))
                {
                    w = plane->at<ito::float32>(y + k, i + x);
                    if (ito::dObjHelper::isFinite<ito::float32>(w))
                    {
                        v = w;
                        break;
                    }
                }
                if ((y - k >= 0) && (y - k < plane->rows))
                {
                    w = plane->at<ito::float32>(y - k, i + x);
                    if (ito::dObjHelper::isFinite<ito::float32>(w))
                    {
                        v = w;
                        break;
                    }
                }
            }
            buf[i + a] = v;
        }
    }    

    for (i = 0; i < a; i++)
    {
        buf[i] = buf[a];
    }

    //memcpy((void*)buf[i], (void*)buf[a], a * sizeof(_Tp));

    for (i = 0; i < b; i++)
    {
        buf[a + dx + i] = buf[a + dx - 1];
    }
    //memcpy((void*)buf[a + dx + i], (void*)buf[a + dx - 1], n * sizeof(_Tp));
}
//----------------------------------------------------------------------------------------------------------------------------------
template<> void Get<ito::float64>(cv::Mat *plane, const ito::int32 x0, const ito::int32 y0, ito::int32 dx, ito::float64 *buf, ito::int8 *inv, const ito::int32 kern, const ito::float64 /*invalid*/ )
{
    ito::int32 a = 0, b = 0, i, k;
    ito::float64 v, w;
    ito::int32 lastval, max;
    ito::int32 x = x0; 
    ito::int32 y = y0;

    //bool check = ito::dObjHelper::isFinite(invalid) && (invalid < std::numeric_limits<ito::int32>::max()) && (invalid > std::numeric_limits<ito::int32>::min());    
    // Check if y colidates with image boarder
    if (y < 0)
    {
        y = 0;
    }

    // Check if y colidates with image boarder
    if (y >= plane->rows)//o->sizes[1])
    {
        y = plane->rows - 1;
    }

    //prefill invalid map with all invalids
    memset(inv, 0, dx);

    // Check if x or dx colidate with image boarder
    if (x < 0)
    {
        a = -x0;
        x = 0;
        dx -= a;
    }

    // Check if x or dx colidate with image boarder
    if (x + dx > plane->cols)
    {
        b = x + dx - plane->cols;
        dx = plane->cols - x;
    }
    // Copy a line with data from plane to buffer
    //memcpy((void*)&buf[a], (_Tp*)&((plane->ptr(y))[x]), dx * sizeof(_Tp));
    memcpy((void*)&buf[a], (void*)&(plane->ptr<ito::float64>(y)[x]), dx * sizeof(ito::float64));

    lastval =- kern;

    // Do the invalid check for each pixel
 
    for (i = 0; i < dx; i++)
    {
        if (ito::dObjHelper::isFinite<ito::float64>(buf[i + a]) /*|| buf[i + a] == invalidInt*/)
        {
            inv[i + a] = 1;
            lastval = i + a;
        }
        else
        {
            max = kern;
            v = 0;
            if (i + a - lastval < max)
            {
                max = i + a - lastval;
                v = buf[lastval];
            }
            for (k = 1; k < max; k++)
            {
                if (k + i >= dx)
                    break;
                if (ito::dObjHelper::isFinite<ito::float64>(buf[i + a + k]))
                {
                    max = k;
                    v = buf[i + a + k];
                    break;
                }
            }
            for (k = 1; k < max; k++)
            {
                if ((y + k >= 0) && (y + k < plane->rows))
                {
                    w = plane->at<ito::float64>(y + k, i + x);
                    if (ito::dObjHelper::isFinite<ito::float64>(w))
                    {
                        v = w;
                        break;
                    }
                }
                if ((y - k >= 0) && (y - k < plane->rows))
                {
                    w = plane->at<ito::float64>(y - k, i + x);
                    if (ito::dObjHelper::isFinite<ito::float64>(w))
                    {
                        v = w;
                        break;
                    }
                }
            }
            buf[i + a] = v;
        }
    }    

    for (i = 0; i < a; i++)
    {
        buf[i] = buf[a];
    }

    //memcpy((void*)buf[i], (void*)buf[a], a * sizeof(_Tp));

    for (i = 0; i < b; i++)
    {
        buf[a + dx + i] = buf[a + dx - 1];
    }
    //memcpy((void*)buf[a + dx + i], (void*)buf[a + dx - 1], n * sizeof(_Tp));
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail This function gives the standard parameters for most of the genericfilter blocks to the addin-interface.
\param[out]   paramsMand  Mandatory parameters for the filter function
\param[out]   paramsOpt   Optinal parameters for the filter function
\author ITO
\sa  BasicFilters::genericLowPassFilter, 
\date
*/
ito::RetVal BasicFilters::genericStdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("n-dim DataObject").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("n-dim DataObject of type sourceImage").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("kernelx", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Odd kernelsize in x").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("kernely", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Odd kernelsize in y").toAscii().data());
        paramsMand->append(param);
        param = ito::Param("replaceNaN", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 0 NaN values in input image will be copied to output image (default)").toAscii().data());
        paramsOpt->append(param);
    }

    return retval;
}

//-----------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal GenericFilterEngine<_Tp>::runFilter(bool replaceNaN)
{
    ito::RetVal err = ito::retOk;

    ito::float64 invalid = std::numeric_limits<ito::float64>::signaling_NaN();

    if(std::numeric_limits<_Tp>::is_exact) replaceNaN = true;

    if(!m_initilized)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("Tried to run generic filter engine without correct initilization of all buffers").toAscii().data());
    }

    if ((m_kernelSizeX == 0) || (m_kernelSizeY == 0) || (m_kernelSizeX >  m_dx) || (m_kernelSizeY >  m_dy))
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("One kernel dimension is zero or bigger than the image size").toAscii().data());
    }

    m_pInLines = (_Tp **)calloc(m_kernelSizeY, sizeof(_Tp*));

    if(m_pInLines == NULL)
    {
        err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate linebuffer").toAscii().data());
        qDebug() << "Not enough memory to allocate linebuffer";
        return err;
    }

    m_pInvalidMap = (ito::int8 **)calloc(m_kernelSizeY, sizeof(ito::int8 *));
    if(m_pInvalidMap == NULL)
    {
        err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate invalidbuffer").toAscii().data());
        qDebug() << "Not enough memory to allocate invalidbuffer";
        free(m_pInLines);
        return err;
    }

    for (ito::int16 kernRow = 0; kernRow < m_kernelSizeY; kernRow++)
    {
        m_pInLines[kernRow] = (_Tp*)calloc(m_dx + m_kernelSizeX - 1, sizeof(_Tp));

        if (m_pInLines[kernRow] == NULL)
        {
            err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate kernel linebuffer").toAscii().data());
        }

        m_pInvalidMap[kernRow] = (ito::int8 *)calloc(m_dx + m_kernelSizeX - 1, sizeof(ito::int8) );
        if (m_pInvalidMap[kernRow] == NULL)
        {
            err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate invalid linebuffer").toAscii().data());
        }
    }

    m_pOutLine = (_Tp*)calloc(m_dx, sizeof(_Tp));

    if (m_pOutLine == NULL)
    {
        err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate output line buffer").toAscii().data());
        qDebug() << "memerr\n";
    }

    ito::int32 kern = m_kernelSizeX < m_kernelSizeY ? m_kernelSizeY : m_kernelSizeX;

    _Tp* pTempLine = NULL;
    ito::int8* pTempInvLine = NULL;
    for(ito::uint32 zPlaneCnt = 0; zPlaneCnt < m_pInpObj->calcNumMats(); zPlaneCnt++)
    {
        cv::Mat* planeIn = (cv::Mat*)(m_pInpObj->get_mdata()[m_pInpObj->seekMat(zPlaneCnt)]);
        cv::Mat* planeOut = (cv::Mat*)(m_pOutObj->get_mdata()[m_pOutObj->seekMat(zPlaneCnt)]);

        if (!err.containsWarningOrError())
        {
            for (ito::int16 kernRow = 0; kernRow <  m_kernelSizeY - 1; kernRow++)
            {                   
                    Get<_Tp>(planeIn, m_x0 - m_AnchorX, m_y0 + kernRow - m_AnchorY, m_dx + m_kernelSizeX - 1, m_pInLines[kernRow + 1], m_pInvalidMap[kernRow + 1], kern, invalid);
            }

            for (ito::int32 y = 0; y < m_dy; y++)
            {
                pTempLine = m_pInLines[0];
                pTempInvLine =  m_pInvalidMap[0];

                ito::int32 curLastRow = m_kernelSizeY - 1;

                for (ito::int16 kernRow = 0; kernRow <  curLastRow; kernRow++)
                {
                     m_pInLines[kernRow] = m_pInLines[kernRow + 1];
                     m_pInvalidMap[kernRow] = m_pInvalidMap[kernRow + 1];
                }
                m_pInLines[curLastRow] = pTempLine;
                m_pInvalidMap[curLastRow] = pTempInvLine;           

                Get<_Tp>(planeIn, m_x0 - m_AnchorX, m_y0 + y + curLastRow - m_AnchorY, m_dx + m_kernelSizeX - 1, m_pInLines[curLastRow], m_pInvalidMap[curLastRow], kern, invalid);

                err += filterFunc();

                // Okay Invalid correction
                if(replaceNaN == false)
                {
                    for (ito::int32 x = 0; x < m_dx; x++)
                    {
                        if (!m_pInvalidMap[m_AnchorY][m_AnchorX + x])
                            m_pOutLine[x] = invalid;
                    }
                }

                memcpy(&(planeOut->ptr<_Tp>(m_y0+y)[m_x0]), m_pOutLine, m_dx *sizeof(_Tp));
            }
        }
    }

    if(m_pOutLine)
    {
        free(m_pOutLine);
    }

    if (m_pInLines)
    {
        for (ito::int16 kernRow = 0; kernRow < m_kernelSizeY; kernRow++)
        {
            free(m_pInLines[kernRow]);
        }
        free(m_pInLines);
    }

    if(m_pInvalidMap)
    {
        for (ito::int16 kernRow = 0; kernRow < m_kernelSizeY; kernRow++)
        {
            free(m_pInvalidMap[kernRow]);
        }
        free(m_pInvalidMap);
    }

    return err;
}



template<typename _Tp> ito::RetVal LowValueFilter<_Tp>::filterFunc()
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax

    //ito::int32 *buf=(ito::int32 *)f->buffer;
    #if (useomp)
    #pragma omp parallel num_threads(NTHREADS)
    {
    ito::int32 x, x1, y1, l;
    _Tp a, b;    
    #pragma omp for schedule(guided)
    #endif     
    for (x = 0; x < this->m_dx; x++)
    {
        for (x1 = 0; x1 < this->m_kernelSizeX; x1++)
        {
            for (y1 = 0; y1 < this->m_kernelSizeX; y1++)
            {
                kbuf[x1 + this->m_kernelSizeX * y1] = this->m_pInLines[y1][x + x1];

                //std::cout << " " << y1 << "  " << buf[x1+gf->m_kernelSizeX*y1] << "\n" << std::endl;
            }
        }
        b = kbuf[0];
        for (l = 0; l < this->m_bufsize; l++)
        {
            a = kbuf[l];
            if (a < b)
                b = a;
        }
        this->m_pOutLine[x] = b;
        //std::cout << x << "  " << b << "\n" << std::endl;
    }
    #if (useomp)
    }
    #endif

    return ito::retOk;
}

template<typename _Tp> ito::RetVal HighValueFilter<_Tp>::filterFunc()
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
    //    ito::int32 x, x1, y1;    
//    k = this->m_bufsize / 2;

    //ito::int32 *buf=(ito::int32 *)f->buffer;
    #if (useomp)
    #pragma omp parallel num_threads(NTHREADS)
    {
    ito::int32 x, x1, y1, l;
    _Tp a, b;    
    #pragma omp for schedule(guided)
    #endif    
    for (x = 0; x < this->m_dx; x++)
    {
        for (x1 = 0; x1 < this->m_kernelSizeX; x1++)
        {
            for (y1 = 0; y1 < this->m_kernelSizeX; y1++)
            {
                kbuf[x1 + this->m_kernelSizeX * y1] = this->m_pInLines[y1][x + x1];

                //std::cout << " " << y1 << "  " << buf[x1+gf->m_kernelSizeX*y1] << "\n" << std::endl;
            }
        }
        b = kbuf[0];
        for (l = 0; l < this->m_bufsize; l++)
        {
            a = kbuf[l];
            if (a > b)
                b = a;
        }
        this->m_pOutLine[x] = b;
        //std::cout << x << "  " << b << "\n" << std::endl;
    }
    #if (useomp)
    }
    #endif        

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
\detail This function use to generic filter engine to set values to the lowest or the highest pixelvalue in the kernel
\param[in|out]   paramsMand  Mandatory parameters for the filter function
\param[in|out]   paramsOpt   Optinal parameters for the filter function
\param[out]   outVals   Outputvalues, not implemented for this function
\param[in]   lowHigh  Flag which toggles low or high filter
\author ITO
\sa  BasicFilters::genericStdParams
\date
*/
ito::RetVal BasicFilters::genericLowHighValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/, bool lowHigh)
{

    ito::RetVal retval = ito::retOk;

    ito::DataObject *dObjScr = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();  //Input object
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();  //Filtered output object

    if(!dObjScr)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source object not defined").toAscii().data());
    }
    else if(dObjScr->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("Ito data object is empty").toAscii().data());
    }
    if(!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination object not defined").toAscii().data());
    }

    if(dObjScr == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if(ito::dObjHelper::dObjareEqualShort(dObjScr, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjScr->copyAxisTagsTo(*dObjDst);
        dObjScr->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjScr->getDims(), dObjScr->getSize(), dObjScr->getType(), dObjScr->getContinuous());
        dObjScr->copyAxisTagsTo(*dObjDst);
        dObjScr->copyTagMapTo(*dObjDst);
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjScr, "dObjScr", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if(retval.containsError())
        return retval;

    // get the kernelsize
    ito::int32 kernelsizex = (*paramsMand)[2].getVal<int>();
    ito::int32 kernelsizey = (*paramsMand)[3].getVal<int>();

    bool replaceNaN = (*paramsOpt)[0].getVal<int>() != 0 ? true : false; //false (default): NaN values in input image will become NaN in output, else: output will be interpolated (somehow)

    if(kernelsizex % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in x must be odd").toAscii().data());
    }

    if(kernelsizey % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in y must be odd").toAscii().data());
    }

    ito::int32 z_length = dObjScr->calcNumMats();  // get the number of Mats (planes) in the input object

    if(lowHigh)
    {
        switch(dObjScr->getType())
        {
            case ito::tInt8:
            {
                HighValueFilter<ito::int8> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt8:
            {
                HighValueFilter<ito::uint8> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt16:
            {
                HighValueFilter<ito::int16> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt16:
            {
                HighValueFilter<ito::uint16> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt32:
            {
                HighValueFilter<ito::int32> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat32:
            {
                HighValueFilter<ito::float32> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat64:
            {
                HighValueFilter<ito::float64> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
        }
    }
    else
    {
        switch(dObjScr->getType())
        {
            case ito::tInt8:
            {
                LowValueFilter<ito::int8> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt8:
            {
                LowValueFilter<ito::uint8> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt16:
            {
                LowValueFilter<ito::int16> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt16:
            {
                LowValueFilter<ito::uint16> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt32:
            {
                LowValueFilter<ito::int32> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat32:
            {
                LowValueFilter<ito::float32> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat64:
            {
                LowValueFilter<ito::float64> filterEngine(dObjScr, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjScr->getSize(dObjScr->getDims()-1), 
                                                          dObjScr->getSize(dObjScr->getDims()-2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex  /2, 
                                                          kernelsizey/2);
                filterEngine.runFilter(replaceNaN);
            }
            break;
        }
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if(!retval.containsError())
    {
        // Add Protokoll

        //        char prot[81] = {0};
        QString msg;
        if(lowHigh)
        {
            //            _snprintf(prot, 80, "high value filter with kernel %i : %i", kernelsizex, kernelsizey);
            msg = tr("high value filter with kernel %1 x %2").arg(kernelsizex).arg(kernelsizey);
        }
        else
        {
            //            _snprintf(prot, 80, "Low value filter with kernel %i : %i", kernelsizex, kernelsizey);
            msg = tr("Low value filter with kernel %1 x %2").arg(kernelsizex).arg(kernelsizey);
        }
        //        dObjDst -> addToProtocol(std::string(prot));

        if(replaceNaN)
        {
            msg.append( tr(" and removed NaN-values"));
        }

        dObjDst -> addToProtocol(std::string(msg.toAscii().data()));
    }

//    if(f.buffer)      // delete the f.buffer defined in medianFilter-struct
//    {
//        free(f.buffer);
//    }

    //int64 testend = cv::getTickCount() - teststart;
    //ito::float64 duration = (ito::float64)testend / cv::getTickFrequency();
    //std::cout << "Time: " << duration << "ms\n";

    return retval;

}
//-----------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::genericLowValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return genericLowHighValueFilter(paramsMand, paramsOpt, paramsOut, false);
}
//-----------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::genericHighValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return genericLowHighValueFilter(paramsMand, paramsOpt, paramsOut, true);
}



//----------------------------------------------------------------------------------------------------------------------------------
