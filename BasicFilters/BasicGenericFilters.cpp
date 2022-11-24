/* ********************************************************************
    Plugin "BasicFilters" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

/*! \file BasicGenericFilters.cpp
\brief   This file contains the generic filter engine.

\author twip optical solutions
\date 12.2013
*/

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant


#include "BasicFilters.h"

#include "common/numeric.h"
#include "common/helperCommon.h"
#include "DataObject/dataObjectFuncs.h"

#if (USEOMP)
    #include <omp.h>
#endif

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

    // Check if y collides with image boarder
    if (y < 0)
    {
        y = 0;
    }

    // Check if y collides with image boarder
    if (y >= plane->rows)//o->sizes[1])
    {
        y = plane->rows - 1;
    }
    
    // Check if x or dx collide with image boarder
    if (x < 0)
    {
        a = -x0;
        x = 0;
        dx -= a;
    }

    // Check if x or dx collide with image boarder
    if (x + dx > plane->cols)
    {
        b = x + dx - plane->cols;
        dx = plane->cols - x;
    }
    // Copy a line with data from plane to buffer
    memcpy((void*)&buf[a], (void*)&(plane->ptr<_Tp>(y)[x]), dx * sizeof(_Tp));

    lastval = -kern;
 
    for (i = 0; i < a; ++i)
    {
        buf[i] = buf[a];
    }

    for (i = 0; i < b; ++i)
    {
        buf[a + dx + i] = buf[a + dx - 1];
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void Get<ito::float32>(cv::Mat *plane, const ito::int32 x0, const ito::int32 y0, ito::int32 dx, ito::float32 *buf, ito::int8 *inv, const ito::int32 kern, const ito::float64 /*invalid*/ )
{
    ito::int32 a = 0, b = 0, i, k;
    ito::float32 v, w, vTemp = 0;
    ito::int32 lastval, max;
    ito::int32 x = x0; 
    ito::int32 y = y0;
    ito::int32 validCnt = 1;
   
    // Check if y collides with image boarder
    if (y < 0)
    {
        y = 0;
    }

    // Check if y collides with image boarder
    if (y >= plane->rows)//o->sizes[1])
    {
        y = plane->rows - 1;
    }

    //prefill invalid map with all invalids
    memset(inv, 0, dx);

    // Check if x or dx collide with image boarder
    if (x < 0)
    {
        a = -x0;
        x = 0;
        dx -= a;
    }

    // Check if x or dx collide with image boarder
    if (x + dx > plane->cols)
    {
        b = x + dx - plane->cols;
        dx = plane->cols - x;
    }
    // Copy a line with data from plane to buffer
    memcpy((void*)&buf[a], (void*)&(plane->ptr<ito::float32>(y)[x]), dx * sizeof(ito::float32));

    lastval = -kern;

    // Do the invalid check for each pixel
 
    for(i = 0; i < dx; i++)
    {
        if (ito::isFinite<ito::float32>(buf[i + a]) /*|| buf[i + a] == invalidInt*/)
        {
            vTemp = buf[i + a];
            break;
        }
    }

    for (i = 0; i < dx; ++i)
    {
        if (ito::isFinite<ito::float32>(buf[i + a]) /*|| buf[i + a] == invalidInt*/)
        {
            inv[i + a] = 1;
            lastval = i + a;
            vTemp = buf[i + a];
        }
        else
        {
            max = kern;
            v = vTemp;
            validCnt = 1;
            if (i + a - lastval < max)
            {
                max = i + a - lastval;
                v = buf[lastval];
            }
            for (k = 1; k < max; ++k)
            {
                if (k + i >= dx)
                    break;
                if (ito::isFinite<ito::float32>(buf[i + a + k]))
                {
                    max = k;
                    v += buf[i + a + k];
                    validCnt++;
                    break;
                }
            }
            for (k = 1; k < max; ++k)
            {
                if ((y + k >= 0) && (y + k < plane->rows))
                {
                    w = plane->at<ito::float32>(y + k, i + x);
                    if (ito::isFinite<ito::float32>(w))
                    {
                        v += w;
                        validCnt ++;
                        break;
                    }
                }
            }
            for (k = 1; k < max; ++k)
            {
                if ((y - k >= 0) && (y - k < plane->rows))
                {
                    w = plane->at<ito::float32>(y - k, i + x);
                    if (ito::isFinite<ito::float32>(w))
                    {
                        v += w;
                        validCnt++;
                        break;
                    }
                }
            }
            buf[i + a] = v/validCnt;
        }
    }    

    for (i = 0; i < a; ++i)
    {
        buf[i] = buf[a];
    }

    for (i = 0; i < b; ++i)
    {
        buf[a + dx + i] = buf[a + dx - 1];
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<> void Get<ito::float64>(cv::Mat *plane, const ito::int32 x0, const ito::int32 y0, ito::int32 dx, ito::float64 *buf, ito::int8 *inv, const ito::int32 kern, const ito::float64 /*invalid*/ )
{
    ito::int32 a = 0, b = 0, i, k;
    ito::float64 v, w, vTemp = 0;
    ito::int32 lastval, max;
    ito::int32 x = x0; 
    ito::int32 y = y0;
    ito::int32 validCnt = 1;

    // Check if y collides with image boarder
    if (y < 0)
    {
        y = 0;
    }

    // Check if y collides with image boarder
    if (y >= plane->rows)//o->sizes[1])
    {
        y = plane->rows - 1;
    }

    //prefill invalid map with all invalids
    memset(inv, 0, dx);

    // Check if x or dx collide with image boarder
    if (x < 0)
    {
        a = -x0;
        x = 0;
        dx -= a;
    }

    // Check if x or dx collide with image boarder
    if (x + dx > plane->cols)
    {
        b = x + dx - plane->cols;
        dx = plane->cols - x;
    }
    // Copy a line with data from plane to buffer
    memcpy((void*)&buf[a], (void*)&(plane->ptr<ito::float64>(y)[x]), dx * sizeof(ito::float64));

    lastval = -kern;

    // Do the invalid check for each pixel
    // First previll the default kernel value for this line
    for(i = 0; i < dx; i++)
    {
        if (ito::isFinite<ito::float64>(buf[i + a]) /*|| buf[i + a] == invalidInt*/)
        {
            vTemp = buf[i + a];
            break;
        }
    }

    for (i = 0; i < dx; ++i)
    {
        if (ito::isFinite<ito::float64>(buf[i + a]) /*|| buf[i + a] == invalidInt*/)
        {
            inv[i + a] = 1;
            lastval = i + a;
            vTemp = buf[i + a];
        }
        else
        {
            max = kern;
            v = vTemp;
            validCnt = 1;
            if (i + a - lastval < max)
            {
                max = i + a - lastval;
                v = buf[lastval];
                
            }
            for (k = 1; k < max; ++k /*k++*/)
            {
                if (k + i >= dx)
                    break;
                if (ito::isFinite<ito::float64>(buf[i + a + k]))
                {
                    max = k;
                    v += buf[i + a + k];
                    validCnt++;
                    break;
                }
            }
            for (k = 1; k < max; ++k /*k++*/)
            {
                if ((y + k >= 0) && (y + k < plane->rows))
                {
                    w = plane->at<ito::float64>(y + k, i + x);
                    if (ito::isFinite<ito::float64>(w))
                    {
                        v += w;
                        validCnt ++;
                        break;
                    }
                }
            }
            for (k = 1; k < max; ++k /*k++*/)
            {
                if ((y - k >= 0) && (y - k < plane->rows))
                {
                    w = plane->at<ito::float64>(y - k, i + x);
                    if (ito::isFinite<ito::float64>(w))
                    {
                        v += w;
                        validCnt ++;
                        break;
                    }
                }
            }
            buf[i + a] = v / validCnt;
        }
    }    

    for (i = 0; i < a; ++i /*k++*/)
    {
        buf[i] = buf[a];
    }

    for (i = 0; i < b; ++i /*k++*/)
    {
        buf[a + dx + i] = buf[a + dx - 1];
    }
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
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("N-dimensional dataObject of any real type. This object is processed plane-by-plane.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Resulting dataObject that has the same size and type than the sourceImage.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("kernelx", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Size of kernel in x-direction (odd values only)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("kernely", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Size of kernel in y-direction (odd values only)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("replaceNaN", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 1, NaN values of sourceImage will be replaced by interpolated filter values in destImage, else destImage also contains NaN values at these positions (0, default).").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//-----------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal GenericFilterEngine<_Tp>::runFilter(bool replaceNaN)
{
    ito::RetVal err = ito::retOk;

    _Tp invalid = 0;
    
    if (std::numeric_limits<_Tp>::is_exact) replaceNaN = true;
    else invalid = std::numeric_limits<_Tp>::quiet_NaN();

    if (!m_initialized)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("Tried to run generic filter engine without correct initilization of all buffers").toLatin1().data());
    }

    if ((m_kernelSizeX == 0) || (m_kernelSizeY == 0) || (m_kernelSizeX >  m_dx) || (m_kernelSizeY >  m_dy))
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("One kernel dimension is zero or bigger than the image size").toLatin1().data());
    }

    m_pInLines = (_Tp **)calloc(m_kernelSizeY, sizeof(_Tp*));

    if (m_pInLines == NULL)
    {
        err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate linebuffer").toLatin1().data());
        qDebug() << "Not enough memory to allocate linebuffer";
        return err;
    }

    m_pInvalidMap = (ito::int8 **)calloc(m_kernelSizeY, sizeof(ito::int8 *));
    if (m_pInvalidMap == NULL)
    {
        err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate invalidbuffer").toLatin1().data());
        qDebug() << "Not enough memory to allocate invalidbuffer";
        free(m_pInLines);
        return err;
    }

    for (ito::int16 kernRow = 0; kernRow < m_kernelSizeY; ++kernRow /*kernRow++*/)
    {
        m_pInLines[kernRow] = (_Tp*)calloc(m_dx + m_kernelSizeX - 1, sizeof(_Tp));

        if (m_pInLines[kernRow] == NULL)
        {
            err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate kernel linebuffer").toLatin1().data());
        }

        m_pInvalidMap[kernRow] = (ito::int8 *)calloc(m_dx + m_kernelSizeX - 1, sizeof(ito::int8) );
        if (m_pInvalidMap[kernRow] == NULL)
        {
            err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate invalid linebuffer").toLatin1().data());
        }
    }

    m_pOutLine = (_Tp*)calloc(m_dx, sizeof(_Tp));

    if (m_pOutLine == NULL)
    {
        err += ito::RetVal(ito::retError, 0, QObject::tr("Not enough memory to allocate output line buffer").toLatin1().data());
        qDebug() << "memerr\n";
    }

    ito::int32 kern = m_kernelSizeX < m_kernelSizeY ? m_kernelSizeY : m_kernelSizeX;

    _Tp* pTempLine = NULL;
    ito::int8* pTempInvLine = NULL;
    for(ito::uint32 zPlaneCnt = 0; zPlaneCnt < (ito::uint32)m_pInpObj->calcNumMats(); ++zPlaneCnt /*zPlaneCnt++*/)
    {
        cv::Mat* planeIn = (cv::Mat*)(m_pInpObj->get_mdata()[m_pInpObj->seekMat(zPlaneCnt)]);
        cv::Mat* planeOut = (cv::Mat*)(m_pOutObj->get_mdata()[m_pOutObj->seekMat(zPlaneCnt)]);

        clearFunc();

        if (!err.containsWarningOrError())
        {
            for (ito::int16 kernRow = 0; kernRow <  m_kernelSizeY - 1; ++kernRow /*kernRow++*/)
            {                   
                    Get<_Tp>(planeIn, m_x0 - m_AnchorX, m_y0 + kernRow - m_AnchorY, m_dx + m_kernelSizeX - 1, m_pInLines[kernRow + 1], m_pInvalidMap[kernRow + 1], kern, invalid);
            }

            for (ito::int32 y = 0; y < m_dy; ++y)
            {
                pTempLine = m_pInLines[0];
                pTempInvLine =  m_pInvalidMap[0];

                ito::int32 curLastRow = m_kernelSizeY - 1;

                for (ito::int16 kernRow = 0; kernRow <  curLastRow; ++kernRow /*kernRow++*/)
                {
                     m_pInLines[kernRow] = m_pInLines[kernRow + 1];
                     m_pInvalidMap[kernRow] = m_pInvalidMap[kernRow + 1];
                }
                m_pInLines[curLastRow] = pTempLine;
                m_pInvalidMap[curLastRow] = pTempInvLine;           

                Get<_Tp>(planeIn, m_x0 - m_AnchorX, m_y0 + y + curLastRow - m_AnchorY, m_dx + m_kernelSizeX - 1, m_pInLines[curLastRow], m_pInvalidMap[curLastRow], kern, invalid);

                //err += filterFunc();
                
                filterFunc();

                // Okay Invalid correction
                if (replaceNaN == false)
                {
                    for (ito::int32 x = 0; x < m_dx; ++x)
                    {
                        if (!m_pInvalidMap[m_AnchorY][m_AnchorX + x])
                            m_pOutLine[x] = invalid;
                    }
                }

                memcpy(&(planeOut->ptr<_Tp>(m_y0+y)[m_x0]), m_pOutLine, m_dx *sizeof(_Tp));
            }
        }
    }

    if (m_pOutLine)
    {
        free(m_pOutLine);
    }

    if (m_pInLines)
    {
        for (ito::int16 kernRow = 0; kernRow < m_kernelSizeY; ++kernRow)
        {
            free(m_pInLines[kernRow]);
        }
        free(m_pInLines);
    }

    if (m_pInvalidMap)
    {
        for (ito::int16 kernRow = 0; kernRow < m_kernelSizeY; ++kernRow)
        {
            free(m_pInvalidMap[kernRow]);
        }
        free(m_pInvalidMap);
    }

    return err;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! LowValueFilter
* \brief   This function calculated the Lowfilter
* \detail  The function calulates the LowFilter-function
*          for the data specified in cv::mat planeIn in the dogenericfilter-function.
*          The actual work is done in the runFilter method.
*     
* \param   GenericFilter   Handle to the filter engine
* \author  ITO
* \sa  GenericFilter::DoGenericFilter, _LowPassFilter
* \date 12.2013
*/
template<typename _Tp> LowValueFilter<_Tp>::LowValueFilter(const ito::DataObject *in, 
    ito::DataObject *out, 
    ito::int32 roiX0, 
    ito::int32 roiY0, 
    ito::int32 roiXSize, 
    ito::int32 roiYSize, 
    ito::int16 kernelSizeX, 
    ito::int16 kernelSizeY,
    ito::int32 anchorPosX,
    ito::int32 anchorPosY
    ) :GenericFilterEngine<_Tp>::GenericFilterEngine()
{ 
    this->m_pInpObj = in;
    this->m_pOutObj = out;

    this->m_x0 = roiX0;
    this->m_y0 = roiY0;

    this->m_dx = roiXSize;
    this->m_dy = roiYSize;

    this->m_kernelSizeX = kernelSizeX;
    this->m_kernelSizeY = kernelSizeY;

    this->m_AnchorX = anchorPosX;
    this->m_AnchorY = anchorPosY;
    
    this->m_bufsize = this->m_kernelSizeX * this->m_kernelSizeY;

#if (USEOMP)
    int numThreads = ito::AddInBase::getMaximumThreadCount();
    kbuf = new _Tp*[numThreads];
    if (kbuf != NULL)
    {
        this->m_initialized = true;
    }
    for(int i = 0; i < numThreads; ++i)
    {
        kbuf[i] = new _Tp[this->m_bufsize];
        if (kbuf[i] == NULL)
        {
            this->m_initialized = false;
        }
    }
#else
    kbuf = new _Tp[this->m_bufsize];
    if (kbuf != NULL)
    {
        this->m_initialized = true;
    }
#endif
}
        
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> LowValueFilter<_Tp>::~LowValueFilter()
{
    if (kbuf != NULL)
    {
#if (USEOMP)
        for (int i = 0; i < ito::AddInBase::getMaximumThreadCount(); ++i)
        {
            delete kbuf[i];
        }
        delete kbuf;
#else
        delete kbuf;
#endif
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void LowValueFilter<_Tp>::clearFunc()
{
    // Do nothing
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> /*ito::RetVal*/ void LowValueFilter<_Tp>::filterFunc()
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax

    //ito::int32 *buf=(ito::int32 *)f->buffer;
    #if (USEOMP)
#pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
    {
    int bufNum = omp_get_thread_num();
//    qDebug() << " " << bufNum << "\n"; // << std::endl;
    _Tp* curKernelBuff = kbuf[bufNum];
#endif  

    ito::int32 x, x1, y1, l;
    _Tp a, b;

    #if (USEOMP)
    #pragma omp for schedule(guided)
    #endif     
    for (x = 0; x < this->m_dx; ++x)
    {
        for (x1 = 0; x1 < this->m_kernelSizeX; ++x1)
        {
            for (y1 = 0; y1 < this->m_kernelSizeY; ++y1)
            {
    #if (USEOMP)
                curKernelBuff[x1 + this->m_kernelSizeX * y1] = this->m_pInLines[y1][x + x1];
    #else
                kbuf[x1 + this->m_kernelSizeX * y1] = this->m_pInLines[y1][x + x1];
    #endif  
            }
        }
    #if (USEOMP)
        b = curKernelBuff[0];
    #else
        b = kbuf[0];
    #endif  
        for (l = 1; l < this->m_bufsize; ++l)
        {
        #if (USEOMP)
            a = curKernelBuff[l];
        #else
            a = kbuf[l];
        #endif  
            
            if (a < b)
                b = a;
        }
        this->m_pOutLine[x] = b;
        //std::cout << x << "  " << b << "\n" << std::endl;
    }
    #if (USEOMP)
    }
    #endif

    //return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! HighValueFilter
 * \brief   This function calculated the Highfilter
 * \detail  The function calulates the HighFilter-function
 *          for the data specified in cv::mat planeIn in the dogenericfilter-function.
 *          The actual work is done in the runFilter method.
 *     
 * \param   GenericFilter   Handle to the filter engine
 * \author  ITO
 * \sa  GenericFilter::DoGenericFilter, _LowPassFilter
 * \date 12.2013
 */
template<typename _Tp> HighValueFilter<_Tp>::HighValueFilter(const ito::DataObject *in, 
                                                           ito::DataObject *out, 
                                                           ito::int32 roiX0, 
                                                           ito::int32 roiY0, 
                                                           ito::int32 roiXSize, 
                                                           ito::int32 roiYSize, 
                                                           ito::int16 kernelSizeX, 
                                                           ito::int16 kernelSizeY,
                                                           ito::int32 anchorPosX,
                                                           ito::int32 anchorPosY
) :GenericFilterEngine<_Tp>::GenericFilterEngine()
{ 
    this->m_pInpObj = in;
    this->m_pOutObj = out;
    
    this->m_x0 = roiX0;
    this->m_y0 = roiY0;
    
    this->m_dx = roiXSize;
    this->m_dy = roiYSize;
    
    this->m_kernelSizeX = kernelSizeX;
    this->m_kernelSizeY = kernelSizeY;
    
    this->m_AnchorX = anchorPosX;
    this->m_AnchorY = anchorPosY;
    
    this->m_bufsize = this->m_kernelSizeX * this->m_kernelSizeY;
    
#if (USEOMP)
    int numThreads = ito::AddInBase::getMaximumThreadCount();
    kbuf = new _Tp*[numThreads];
    if (kbuf != NULL)
    {
        this->m_initialized = true;
    }
    for (int i = 0; i < numThreads; ++i)
    {
        kbuf[i] = new _Tp[this->m_bufsize];
        if (kbuf[i] == NULL)
        {
            this->m_initialized = false;
        }
    }
#else
    kbuf = new _Tp[this->m_bufsize];
    if (kbuf != NULL)
    {
        this->m_initialized = true;
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> HighValueFilter<_Tp>::~HighValueFilter()
{
    if (kbuf != NULL)
    {
#if (USEOMP)
        for (int i = 0; i < ito::AddInBase::getMaximumThreadCount(); ++i)
        {
            delete kbuf[i];
        }
        delete kbuf;
#else
        delete kbuf;
#endif
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void HighValueFilter<_Tp>::clearFunc()
{
    // Do nothing
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> /*ito::RetVal*/ void HighValueFilter<_Tp>::filterFunc()
{
    // in case we want to access the protected members of the templated parent class we have to take special care!
    // the easiest way is using the this-> syntax
#if (USEOMP)
    #pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
    {
    int bufNum = omp_get_thread_num();
    _Tp* curKernelBuff = kbuf[bufNum];
#endif  
    ito::int32 x, x1, y1, l;
    _Tp a, b;

#if (USEOMP)
    #pragma omp for schedule(guided)
#endif    
    for (x = 0; x < this->m_dx; ++x)
    {
        for (x1 = 0; x1 < this->m_kernelSizeX; ++x1)
        {
            for (y1 = 0; y1 < this->m_kernelSizeY; ++y1)
            {
#if (USEOMP)
                curKernelBuff[x1 + this->m_kernelSizeX * y1] = this->m_pInLines[y1][x + x1];
#else
                kbuf[x1 + this->m_kernelSizeX * y1] = this->m_pInLines[y1][x + x1];
#endif
                //std::cout << " " << y1 << "  " << buf[x1+gf->m_kernelSizeX*y1] << "\n" << std::endl;
            }
        }
#if (USEOMP)
        b = curKernelBuff[0];
#else
        b = kbuf[0];
#endif
        for (l = 1; l < this->m_bufsize; ++l)
        {
#if (USEOMP)
            a = curKernelBuff[l];
#else
            a = kbuf[l];
#endif
            if (a > b)
                b = a;
        }
        this->m_pOutLine[x] = b;
        //std::cout << x << "  " << b << "\n" << std::endl;
    }
    #if (USEOMP)
    }
    #endif        

    //return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
/*!
\detail This function use the generic filter engine to set values to the lowest or the highest pixelvalue in the kernel
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

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();  //Input object
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();  //Filtered output object

    if (!dObjSrc)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is invalid").toLatin1().data());
    }
    else if (dObjSrc->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is empty").toLatin1().data());
    }
    if (!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination image is invalid").toLatin1().data());
    }

    if (dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if (ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    // get the kernelsize
    ito::int32 kernelsizex = (*paramsMand)[2].getVal<int>();
    ito::int32 kernelsizey = (*paramsMand)[3].getVal<int>();

    bool replaceNaN = (*paramsOpt)[0].getVal<int>() != 0 ? true : false; //false (default): NaN values in input image will become NaN in output, else: output will be interpolated (somehow)

    if (kernelsizex % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in x must be odd").toLatin1().data());
    }

    if (kernelsizey % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in y must be odd").toLatin1().data());
    }

    //ito::int32 z_length = dObjSrc->calcNumMats();  // get the number of Mats (planes) in the input object

    if (lowHigh)
    {
        switch(dObjSrc->getType())
        {
            case ito::tInt8:
            {
                HighValueFilter<ito::int8> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt8:
            {
                HighValueFilter<ito::uint8> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt16:
            {
                HighValueFilter<ito::int16> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt16:
            {
                HighValueFilter<ito::uint16> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt32:
            {
                HighValueFilter<ito::int32> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat32:
            {
                HighValueFilter<ito::float32> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat64:
            {
                HighValueFilter<ito::float64> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
        }
    }
    else
    {
        switch(dObjSrc->getType())
        {
            case ito::tInt8:
            {
                LowValueFilter<ito::int8> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt8:
            {
                LowValueFilter<ito::uint8> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt16:
            {
                LowValueFilter<ito::int16> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tUInt16:
            {
                LowValueFilter<ito::uint16> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tInt32:
            {
                LowValueFilter<ito::int32> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat32:
            {
                LowValueFilter<ito::float32> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
            case ito::tFloat64:
            {
                LowValueFilter<ito::float64> filterEngine(dObjSrc, 
                                                          dObjDst, 
                                                          0, 
                                                          0, 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                          dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                          kernelsizex, 
                                                          kernelsizey, 
                                                          kernelsizex / 2, 
                                                          kernelsizey / 2);
                retval += filterEngine.runFilter(replaceNaN);
            }
            break;
        }
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add protocol
        QString msg;
        if (lowHigh)
        {
            msg = tr("high value filter with kernel %1 x %2").arg(kernelsizex).arg(kernelsizey);
        }
        else
        {
            msg = tr("Low value filter with kernel %1 x %2").arg(kernelsizex).arg(kernelsizey);
        }

        if (replaceNaN)
        {
            msg.append( tr(" and removed NaN-values"));
        }

        dObjDst -> addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

const QString BasicFilters::genericLowValueFilterDoc = QObject::tr("2D minimum filter that set a pixel to the lowest value within the surrounding kernel. \n\
\n\
The rectangular kernel of size (kernely, kernelx) is centered at each pixel of the source image. The same pixel in the destination image \n\
is then set to the minimum value within the kernel. Invalid values within the kernel are ignored. It is possible to fill these invalid values \n\
by the minimum of the surrounding kernel, too, if the parameter 'replaceNaN' is set to 1. \n\
\n\
This filter can be applied inplace, hence, the source and destination image can be the same. This filter is implemented for all \n\
real data types. 3 or higher dimensional dataObjects are processed plane-by-plane, such that the destination image has always the same size \n\
and type than the input image. \n\
\n\
See also: highValueFilter");
//-----------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::genericLowValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return genericLowHighValueFilter(paramsMand, paramsOpt, paramsOut, false);
}

const QString BasicFilters::genericHighValueFilterDoc = QObject::tr("2D maximum filter that set a pixel to the highest value within the surrounding kernel. \n\
\n\
The rectangular kernel of size (kernely, kernelx) is centered at each pixel of the source image. The same pixel in the destination image \n\
is then set to the maximum value within the kernel. Invalid values within the kernel are ignored. It is possible to fill these invalid values \n\
by the maximum of the surrounding kernel, too, if the parameter 'replaceNaN' is set to 1. \n\
\n\
This filter can be applied inplace, hence, the source and destination image can be the same. This filter is implemented for all \n\
real data types. 3 or higher dimensional dataObjects are processed plane-by-plane, such that the destination image has always the same size \n\
and type than the input image \n\
\n\
See also: lowValueFilter");
//-----------------------------------------------------------------------------------------------
ito::RetVal BasicFilters::genericHighValueFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return genericLowHighValueFilter(paramsMand, paramsOpt, paramsOut, true);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*! \class MedianFilter
 * \brief   This class filters an input-image median
 * \detail  The class calulates the MedianFilter-function
 *          for the data specified in cv::mat planeIn in the dogenericfilter-function.
 *          The actual work is done in the runFilter method.
 *     
 * \param   GenericFilter   Handle to the filter engine
 * \author  ITO
 * \sa  GenericFilter::DoGenericFilter, _LowPassFilter
 * \date 12.2013
 */
template<typename _Tp> MedianFilter<_Tp>::MedianFilter(const ito::DataObject *in, 
                                                           ito::DataObject *out, 
                                                           ito::int32 roiX0, 
                                                           ito::int32 roiY0, 
                                                           ito::int32 roiXSize, 
                                                           ito::int32 roiYSize, 
                                                           ito::int16 kernelSizeX, 
                                                           ito::int16 kernelSizeY,
                                                           ito::int32 anchorPosX,
                                                           ito::int32 anchorPosY
) :GenericFilterEngine<_Tp>::GenericFilterEngine()
{ 
    this->m_pInpObj = in;
    this->m_pOutObj = out;
    
    this->m_x0 = roiX0;
    this->m_y0 = roiY0;
    
    this->m_dx = roiXSize;
    this->m_dy = roiYSize;
    
    this->m_kernelSizeX = kernelSizeX;
    this->m_kernelSizeY = kernelSizeY;
    
    this->m_AnchorX = anchorPosX;
    this->m_AnchorY = anchorPosY;
    
    this->m_bufsize = this->m_kernelSizeX * this->m_kernelSizeY;
    
#if (USEOMP)
    int numThreads = ito::AddInBase::getMaximumThreadCount();
    kbuf = new _Tp*[numThreads];
    kbufPtr = new _Tp**[numThreads];
    for (int i = 0; i < numThreads; ++i)
    {
        kbuf[i] = new _Tp[this->m_bufsize];
        kbufPtr[i] = new _Tp*[this->m_bufsize];
        for (ito::int16 j = 0; j < this->m_bufsize; ++j)
            kbufPtr[i][j] = (_Tp*)&(kbuf[i][j]);
    }
    this->m_initialized = true;
#else
    kbuf = new _Tp[this->m_bufsize];
    kbufPtr = new _Tp*[this->m_bufsize];
    if (kbuf != NULL && kbufPtr != NULL)
    {
        this->m_initialized = true;
    }
    for (ito::int16 i = 0; i < this->m_bufsize; ++i)
        kbufPtr[i] = (_Tp*)&(kbuf[i]);
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void MedianFilter<_Tp>::clearFunc()
{
    // Do nothing
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> MedianFilter<_Tp>::~MedianFilter()
{
#if (USEOMP)
    int numThreads = ito::AddInBase::getMaximumThreadCount();
#endif
    if (kbuf != NULL)
    {
#if (USEOMP)
        for (int i = 0; i < numThreads; ++i)
        {
            delete kbuf[i];
        }
        delete kbuf;
#else
        delete kbuf;
#endif
    }
    if (kbufPtr != NULL)
    {
#if (USEOMP)
        for (int i = 0; i < numThreads; ++i)
        {
            delete kbufPtr[i];
        }
        delete kbufPtr;
#else
        delete kbufPtr;
#endif
    }
}

//-----------------------------------------------------------------------------------------------
/* ! MedianFilter
*   \brief   This function calculats a medianfiltered image from data input
*   \detail  The function calulates the lowpassfilter-function
*            for the data specified in cv::mat planeIn in the dogenericfilter-function.
*
*   \param   GenericFilter   Handle to the filter engine
*   \author  ITO
*   \sa  GenericFilter::DoGenericFilter, _MedFilter
*   \date 12.2013
*/
template<typename _Tp> /*ito::RetVal*/ void MedianFilter<_Tp>::filterFunc()
{
#if (USEOMP)
    #pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
    {
    int bufNum = omp_get_thread_num();
    _Tp **pptr = kbufPtr[bufNum];
    _Tp *dptr = kbuf[bufNum];
#else
    _Tp **pptr = kbufPtr;
    _Tp *dptr = kbuf;
#endif

    ito::uint32 leftElement = 0;
    ito::uint32 rightElement = 0;
    ito::uint32 leftPos = 0;
    ito::uint32 rightPos = 0;
    ito::int32 x = 0, x1 = 0, y1 = 0;
    
    ito::uint32 halfKernSize = this->m_bufsize / 2;
    
    ito::int32 size = 0;
    _Tp a;
    _Tp *tempValuePtr;

    // buf : kernelbuffer
    // buffer: internal buffer
    // bufptr: index buffer to internal buffer

    for (x1 = 0; x1 < (this->m_kernelSizeX - 1); ++x1)
    {
        for (y1 = 0; y1 < this->m_kernelSizeY; ++y1)
        {
            *dptr++ = this->m_pInLines[y1][x1];
        }
    }

#if (USEOMP)
    ito::int32 nextIdleX = 0;
    #pragma omp for schedule(guided)
#endif    
    for (x = 0; x < this->m_dx; ++x)
    {
#if (USEOMP)
        ito::int16 cn =  ((x - nextIdleX) >= this->m_kernelSizeX ? this->m_kernelSizeX - 1 : (x - nextIdleX));

        for (; cn >= 0; cn--)
        {            
#endif
            for (y1 = 0; y1 < this->m_kernelSizeY; ++y1)
            {
    //            *dptr++ = ((ito::float64 **)gf->buf)[y1][x + gf->nkx - 1];
#if (USEOMP)
                *dptr++ = this->m_pInLines[y1][x - cn + this->m_kernelSizeX - 1];
#else
                *dptr++ = this->m_pInLines[y1][x + this->m_kernelSizeX - 1];
#endif
            }
            x1++;
            if (x1 >= this->m_kernelSizeX)
            {
    //            dptr = (ito::float64 *)f->buffer;
#if (USEOMP)
                dptr = kbuf[bufNum];
#else
                dptr = kbuf;
#endif

                x1 = 0;
            }
#if (USEOMP)
        }
        nextIdleX = x + 1;
#endif
        //this algorithms seems to be like the following: http://www.i-programmer.info/babbages-bag/505-quick-median.html?start=1
        leftElement = 0;
        rightElement = this->m_bufsize - 1;
        while(leftElement < rightElement)
        {
            a = *pptr[halfKernSize];
            leftPos = leftElement;
            rightPos = rightElement;
            do
            {
                while(*pptr[leftPos] < a)
                {
                    leftPos++;
                }
                while(*pptr[rightPos] > a)
                {
                    rightPos--;
                }
                if (leftPos <= rightPos)
                {
                    tempValuePtr = pptr[leftPos];
                    pptr[leftPos] = pptr[rightPos];
                    pptr[rightPos] = tempValuePtr;
                    leftPos++;
                    rightPos--;
                }
            } while(leftPos <= rightPos);

            if (rightPos < halfKernSize)
            {
                leftElement = leftPos;
            }
            if (halfKernSize < leftPos)
            {
                rightElement = rightPos;
            }
        }
        this->m_pOutLine[x] = *pptr[halfKernSize];
    }
#if (USEOMP)
    }
#endif
    //return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::genericMedianFilterDoc = QObject::tr("2D median filter that set a pixel to the median value of all valid pixels within the kernel. \n\
\n\
The rectangular kernel of size (kernely, kernelx) is centered at each pixel of the source image. The same pixel in the destination image \n\
is then set to the median value of all valid pixels within the kernel. Invalid values within the kernel are ignored. It is possible to fill these invalid values \n\
by the median value of the surrounding kernel, too, if the parameter 'replaceNaN' is set to 1. \n\
\n\
This filter can be applied inplace, hence, the source and destination image can be the same. This filter is implemented for all \n\
real data types. 3 or higher dimensional dataObjects are processed plane-by-plane, such that the destination image has always the same size \n\
and type than the input image");
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
ito::RetVal BasicFilters::genericMedianFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();  //Input object
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();  //Filtered output object

    if (!dObjSrc)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is invalid").toLatin1().data());
    }
    else if (dObjSrc->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is empty").toLatin1().data());
    }
    if (!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination image is invalid").toLatin1().data());
    }

    if (dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if (ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    // get the kernelsize
    ito::int32 kernelsizex = (*paramsMand)[2].getVal<int>();
    ito::int32 kernelsizey = (*paramsMand)[3].getVal<int>();

    bool replaceNaN = (*paramsOpt)[0].getVal<int>() != 0 ? true : false; //false (default): NaN values in input image will become NaN in output, else: output will be interpolated (somehow)

    if (kernelsizex % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in x must be odd").toLatin1().data());
    }

    if (kernelsizey % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in y must be odd").toLatin1().data());
    }

    //ito::int32 z_length = dObjSrc->calcNumMats();  // get the number of Mats (planes) in the input object

    switch(dObjSrc->getType())
    {
        case ito::tInt8:
        {
            MedianFilter<ito::int8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt8:
        {
            MedianFilter<ito::uint8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt16:
        {
            MedianFilter<ito::int16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt16:
        {
            MedianFilter<ito::uint16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt32:
        {
            MedianFilter<ito::int32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat32:
        {
            MedianFilter<ito::float32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat64:
        {
            MedianFilter<ito::float64> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add Protokoll
        QString msg;
        msg = tr("median filter with kernel %1 x %2").arg(kernelsizex).arg(kernelsizey);
        //        dObjDst -> addToProtocol(std::string(prot));

        if (replaceNaN)
        {
            msg.append( tr(" and removed NaN-values"));
        }

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    //int64 testend = cv::getTickCount() - teststart;
    //ito::float64 duration = (ito::float64)testend / cv::getTickFrequency();
    //std::cout << "Time: " << duration << "ms\n";

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> SobelOptFilter<_Tp>::SobelOptFilter(ito::DataObject *in,
    ito::DataObject *out,
    ito::int32 roiX0,
    ito::int32 roiY0,
    ito::int32 roiXSize,
    ito::int32 roiYSize,
    ito::uint8 gradDir
    ) :GenericFilterEngine<_Tp>::GenericFilterEngine()
{
    this->m_pInpObj = in;
    this->m_pOutObj = out;

    this->m_x0 = roiX0;
    this->m_y0 = roiY0;

    this->m_dx = roiXSize;
    this->m_dy = roiYSize;

    this->m_kernelSizeX = 3;
    this->m_kernelSizeY = 3;

    this->m_AnchorX = 1;
    this->m_AnchorY = 1;

    this->m_initialized = true;

    m_gradDir = gradDir;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> SobelOptFilter<_Tp>::~SobelOptFilter()
{
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void SobelOptFilter<_Tp>::clearFunc()
{
    // Do nothing
    return;
}

//-----------------------------------------------------------------------------------------------
template<typename _Tp> /*ito::RetVal*/ void SobelOptFilter<_Tp>::filterFunc()
{
    #if (USEOMP)
    #pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
    {
    #endif

    ito::float64 gradx, grady;

    if (m_gradDir == 0)
    {
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif  
        for (ito::int32 x = 0; x < this->m_dx; x++)
        {
            this->m_pOutLine[x] = 3.0 / 32.0 * this->m_pInLines[0][x] + 10.0 / 32.0 * this->m_pInLines[1][x] + 3.0 / 32.0 * this->m_pInLines[2][x]
                - 3.0 / 32.0 * this->m_pInLines[0][x + 2] - 10.0 / 32.0 * this->m_pInLines[1][x + 2] - 3.0 / 32.0 * this->m_pInLines[2][x + 2];
        }
    }
    else if (m_gradDir == 1)
    {
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif  
        for (ito::int32 x = 0; x < this->m_dx; x++)
        {
            this->m_pOutLine[x] = 3.0 / 32.0 * this->m_pInLines[0][x] + 10.0 / 32.0 * this->m_pInLines[0][x + 1] + 3.0 / 32.0 * this->m_pInLines[0][x + 2]
                - 3.0 / 32.0 * this->m_pInLines[2][x] - 10.0 / 32.0 * this->m_pInLines[2][x + 1] - 3.0 / 32.0 * this->m_pInLines[2][x + 2];
        }
    }
    else
    {
        #if (USEOMP)
        #pragma omp for schedule(guided)
        #endif
        for (ito::int32 x = 0; x < this->m_dx; x++)
        {
            gradx = 3.0 / 32.0 * this->m_pInLines[0][x] + 10.0 / 32.0 * this->m_pInLines[0][x + 1] + 3.0 / 32.0 * this->m_pInLines[0][x + 2]
                - 3.0 / 32.0 * this->m_pInLines[2][x] - 10.0 / 32.0 * this->m_pInLines[2][x + 1] - 3.0 / 32.0 * this->m_pInLines[2][x + 2];
            grady = 3.0 / 32.0 * this->m_pInLines[0][x] + 10.0 / 32.0 * this->m_pInLines[1][x] + 3.0 / 32.0 * this->m_pInLines[2][x]
                - 3.0 / 32.0 * this->m_pInLines[0][x + 2] - 10.0 / 32.0 * this->m_pInLines[1][x + 2] - 3.0 / 32.0 * this->m_pInLines[2][x + 2];
            this->m_pOutLine[x] = sqrt(gradx * gradx + grady * grady);
        }
    }
    #if (USEOMP)
    }
    #endif
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail This function gives the standard parameters for optimized Sobel filter.
\param[out]   paramsMand  Mandatory parameters for the filter function
\param[out]   paramsOpt   Optinal parameters for the filter function
\author ITO
\sa  BasicFilters::genericLowPassFilter,
\date
*/
ito::RetVal BasicFilters::genericSobelOptParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("n-dim DataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("n-dim DataObject of type sourceImage").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("dir", ito::ParamBase::Int | ito::ParamBase::In, 0, 2, 0, tr("derivative in x: 0, y: 1 direction or 2: 2D magnitude sqrt(dx*dx + dy*dy)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("replaceNaN", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("if 0 NaN values in input image will be copied to output image (default)").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
\detail This function uses the generic filter engine to run the optimized Sobel filter (s. Jähne Digitale Bildverarbeitung)
\param[in|out]   paramsMand  Mandatory parameters for the filter function
\param[in|out]   paramsOpt   Optinal parameters for the filter function
\param[out]   outVals   Outputvalues, not implemented for this function
\param[in]   lowHigh  Flag which toggles low or high filter
\author ITO
\sa  BasicFilters::genericStdParams
\date
*/
ito::RetVal BasicFilters::genericSobelOptFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();  //Input object
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();  //Filtered output object
    ito::uint8 inPlace = 0;

    if (!dObjSrc)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source object not defined").toLatin1().data());
    }
    else if (dObjSrc->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("Ito data object is empty").toLatin1().data());
    }
    if (!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination object not defined").toLatin1().data());
    }

    if (dObjDst == dObjSrc)
    {
        dObjDst = new ito::DataObject();
        inPlace = 1;
    }

    switch (dObjSrc->getType())
    {
        case ito::tUInt8:
            (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), ito::tInt8, dObjSrc->getContinuous());
        break;

        case ito::tUInt16:
            (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), ito::tInt16, dObjSrc->getContinuous());
        break;

        case ito::tUInt32:
            (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), ito::tInt32, dObjSrc->getContinuous());
        break;

        default:
            (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        break;
    }
    dObjDst->deleteAllTags();
    dObjSrc->copyAxisTagsTo(*dObjDst);
    dObjSrc->copyTagMapTo(*dObjDst);

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    // get the kernelsize
    ito::uint8 gradDir = (*paramsMand)[2].getVal<int>();

    bool replaceNaN = (*paramsOpt)[0].getVal<int>() != 0 ? true : false; //false (default): NaN values in input image will become NaN in output, else: output will be interpolated (somehow)

    //ito::int32 z_length = dObjSrc->calcNumMats();  // get the number of Mats (planes) in the input object
    switch (dObjSrc->getType())
    {
        case ito::tInt8:
        {
            SobelOptFilter<ito::int8> filterEngine(dObjSrc,
                dObjDst,
                0,
                0,
                dObjSrc->getSize(dObjSrc->getDims() - 1),
                dObjSrc->getSize(dObjSrc->getDims() - 2),
                gradDir);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt8:
        {
            SobelOptFilter<ito::uint8> filterEngine(dObjSrc,
                dObjDst,
                0,
                0,
                dObjSrc->getSize(dObjSrc->getDims() - 1),
                dObjSrc->getSize(dObjSrc->getDims() - 2),
                gradDir);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt16:
        {
            SobelOptFilter<ito::int16> filterEngine(dObjSrc,
                dObjDst,
                0,
                0,
                dObjSrc->getSize(dObjSrc->getDims() - 1),
                dObjSrc->getSize(dObjSrc->getDims() - 2),
                gradDir);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt16:
        {
            SobelOptFilter<ito::uint16> filterEngine(dObjSrc,
                dObjDst,
                0,
                0,
                dObjSrc->getSize(dObjSrc->getDims() - 1),
                dObjSrc->getSize(dObjSrc->getDims() - 2),
                gradDir);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt32:
        {
            SobelOptFilter<ito::int32> filterEngine(dObjSrc,
                dObjDst,
                0,
                0,
                dObjSrc->getSize(dObjSrc->getDims() - 1),
                dObjSrc->getSize(dObjSrc->getDims() - 2),
                gradDir);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat32:
        {
            SobelOptFilter<ito::float32> filterEngine(dObjSrc,
                dObjDst,
                0,
                0,
                dObjSrc->getSize(dObjSrc->getDims() - 1),
                dObjSrc->getSize(dObjSrc->getDims() - 2),
                gradDir);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat64:
        {
            SobelOptFilter<ito::float64> filterEngine(dObjSrc,
                dObjDst,
                0,
                0,
                dObjSrc->getSize(dObjSrc->getDims() - 1),
                dObjSrc->getSize(dObjSrc->getDims() - 2),
                gradDir);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add Protokoll
        QString msg;
        msg = tr("Optimized Sobel filter with kernel 3 x 3");

        if (replaceNaN)
        {
            msg.append(tr(" and removed NaN-values"));
        }

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    if (inPlace)
        *dObjSrc = *dObjDst;
    //int64 testend = cv::getTickCount() - teststart;
    //ito::float64 duration = (ito::float64)testend / cv::getTickFrequency();
    //std::cout << "Time: " << duration << "ms\n";

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> LowPassFilter<_Tp>::LowPassFilter(  const ito::DataObject *in,
                                                           ito::DataObject *out, 
                                                           ito::int32 roiX0, 
                                                           ito::int32 roiY0, 
                                                           ito::int32 roiXSize, 
                                                           ito::int32 roiYSize, 
                                                           ito::int16 kernelSizeX, 
                                                           ito::int16 kernelSizeY,
                                                           ito::int32 anchorPosX,
                                                           ito::int32 anchorPosY
) :GenericFilterEngine<_Tp>::GenericFilterEngine()
{ 
    this->m_pInpObj = in;
    this->m_pOutObj = out;
    
    this->m_x0 = roiX0;
    this->m_y0 = roiY0;
    
    this->m_dx = roiXSize;
    this->m_dy = roiYSize;
    
    this->m_kernelSizeX = kernelSizeX;
    this->m_kernelSizeY = kernelSizeY;
    
    this->m_AnchorX = anchorPosX;
    this->m_AnchorY = anchorPosY;
    
    this->m_bufsize = this->m_kernelSizeX * this->m_kernelSizeY;
    
    this->m_colwiseSumBuffer = new ito::float64[roiXSize + this->m_kernelSizeX - 1];

    this->m_divisor = this->m_bufsize;

    if (m_colwiseSumBuffer != NULL)
    {
        this->m_initialized = true;
    }

    this->m_isFilled = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> LowPassFilter<_Tp>::~LowPassFilter()
{
    if (m_colwiseSumBuffer != NULL)
    {
        delete m_colwiseSumBuffer;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void LowPassFilter<_Tp>::clearFunc()
{
    // Do nothing
    m_isFilled = false;
    return;
}
//-----------------------------------------------------------------------------------------------
template<typename _Tp> /*ito::RetVal*/ void LowPassFilter<_Tp>::filterFunc()
{
    ito::float64 summe = 0;

//    #if (USEOMP)
//    #pragma omp parallel num_threads(NTHREADS)
//    {
//    #endif
    ito::int16 kernelLastY = this->m_kernelSizeY - 1;
    ito::int16 kernelLastX = this->m_kernelSizeX - 1;

    
    if (!this->m_isFilled)
    {
        this->m_isFilled = true;

   //     #if (USEOMP)
   //     #pragma omp for schedule(guided)
   //     #endif    
        for(ito::uint32 x = 0; x < this->m_dx + kernelLastX; x++)
        {
            this->m_colwiseSumBuffer[x] = (ito::float64) this->m_pInLines[0][x];
            for(ito::uint16 y = 1; y < this->m_kernelSizeY; y++)
            {
                //((ito::int32 *)this->csum)[x]+=((ito::int32 **)this->buf)[y][x];

                this->m_colwiseSumBuffer[x] += (ito::float64) this->m_pInLines[y][x];
            }
        }
    }
    else
    { 
 //       #if (USEOMP)
 //       #pragma omp for schedule(guided)
 //       #endif  
        for(ito::uint32 x = 0; x < this->m_dx + kernelLastX; x++)
        {
            //((ito::int32 *)this->csum)[x]+=((ito::int32 **)this->buf)[this->nky-1][x];
            this->m_colwiseSumBuffer[x] += (ito::float64) this->m_pInLines[kernelLastY][x];
        }
    }

//    if (true)
//    {
//        #pragma omp barrier
//    }

 //   #if (USEOMP)
 //   #pragma omp master
 //   {
 //   #endif  
    for(ito::uint16 x = 0; x < kernelLastX; x++)
    {
        //summe+=((ito::int32 *)this->csum)[x];
        summe += this->m_colwiseSumBuffer[x];
    }

    for(ito::uint32 x = 0; x < this->m_dx; x++)
    {
        //summe+=((ito::int32 *)this->csum)[x+this->nkx-1];
        //((ito::int32 *)this->out)[x]=summe/this->divisor;
        //summe-=((ito::int32 *)f->csum)[x];

        summe += this->m_colwiseSumBuffer[x + kernelLastX];
        this->m_pOutLine[x] = summe / m_divisor;
        summe -= this->m_colwiseSumBuffer[x];
    }
 //   #if (USEOMP)
 //   }
 //   #endif

//    if (true)
//    {
//        #pragma omp barrier
//    }

//    #if (USEOMP)
//    #pragma omp for schedule(guided)
//    #endif  
    for(ito::uint32 x = 0; x < this->m_dx + kernelLastX; x++)
    {
        //((ito::int32 *)f->csum)[x]-=((ito::int32 **)this->buf)[0][x];
        this->m_colwiseSumBuffer[x] -= (ito::float64) this->m_pInLines[0][x];
    }
    
//    #if (USEOMP)
//    }
//    #endif
    //return ito::retOk;
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

const QString BasicFilters::genericLowPassFilterDoc = QObject::tr("2D lowpass (blurring) filter that set a pixel to the mean value of all valid pixels within the kernel. \n\
\n\
The rectangular kernel of size (kernely, kernelx) is centered at each pixel of the source image. The same pixel in the destination image \n\
is then set to the mean value of all valid pixels within the kernel. It is possible to fill the invalid values \n\
by the mean value of the surrounding kernel, too, if the parameter 'replaceNaN' is set to 1. If the kernel only contains invalid values, \n\
no replacement is possible and the corresponding pixel in the destination image remains invalid. \n\
\n\
This filter can be applied inplace, hence, the source and destination image can be the same. This filter is implemented for all \n\
real data types. 3 or higher dimensional dataObjects are processed plane-by-plane, such that the destination image has always the same size \n\
and type than the input image");

ito::RetVal BasicFilters::genericLowPassFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();  //Input object
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();  //Filtered output object

    if (!dObjSrc)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is invalid").toLatin1().data());
    }
    else if (dObjSrc->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is empty").toLatin1().data());
    }
    if (!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination image is invalid").toLatin1().data());
    }

    if (dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if (ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    // get the kernelsize
    ito::int32 kernelsizex = (*paramsMand)[2].getVal<int>();
    ito::int32 kernelsizey = (*paramsMand)[3].getVal<int>();

    bool replaceNaN = (*paramsOpt)[0].getVal<int>() != 0 ? true : false; //false (default): NaN values in input image will become NaN in output, else: output will be interpolated (somehow)

    if (kernelsizex % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in x must be odd").toLatin1().data());
    }

    if (kernelsizey % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in y must be odd").toLatin1().data());
    }

    //ito::int32 z_length = dObjSrc->calcNumMats();  // get the number of Mats (planes) in the input object

    switch(dObjSrc->getType())
    {
        case ito::tInt8:
        {
            LowPassFilter<ito::int8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt8:
        {
            LowPassFilter<ito::uint8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt16:
        {
            LowPassFilter<ito::int16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt16:
        {
            LowPassFilter<ito::uint16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt32:
        {
            LowPassFilter<ito::int32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat32:
        {
            LowPassFilter<ito::float32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat64:
        {
            LowPassFilter<ito::float64> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelsizex, 
                                                kernelsizey, 
                                                kernelsizex / 2, 
                                                kernelsizey / 2);
            retval += filterEngine.runFilter(replaceNaN);
        }
        break;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add Protokoll
        QString msg;
        msg = tr("lowpass-filter (mean) with kernel %1 x %2").arg(kernelsizex).arg(kernelsizey);
        //        dObjDst -> addToProtocol(std::string(prot));

        if (replaceNaN)
        {
            msg.append( tr(" and removed NaN-values"));
        }

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    //int64 testend = cv::getTickCount() - teststart;
    //ito::float64 duration = (ito::float64)testend / cv::getTickFrequency();
    //std::cout << "Time: " << duration << "ms\n";

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> GaussianFilter<_Tp>::GaussianFilter( const ito::DataObject *in, 
                                                           ito::DataObject *out, 
                                                           ito::int32 roiX0, 
                                                           ito::int32 roiY0, 
                                                           ito::int32 roiXSize, 
                                                           ito::int32 roiYSize, 
                                                           ito::float64 sigmaSizeX, 
                                                           ito::float64 epsilonSizeX,
                                                           ito::float64 sigmaSizeY, 
                                                           ito::float64 epsilonSizeY
) :GenericFilterEngine<_Tp>::GenericFilterEngine()
{ 
    this->m_pInpObj = in;
    this->m_pOutObj = out;
    
    this->m_x0 = roiX0;
    this->m_y0 = roiY0;
    
    this->m_dx = roiXSize;
    this->m_dy = roiYSize;
    
    ito::float64 a = - 2.0 * sigmaSizeY * sigmaSizeY * log( sigmaSizeY * epsilonSizeY * sqrt(2.0 * M_PI) );
    if (a < 0.5)
    {
        this->m_kernelSizeY = 1;
        
    }
    else
    {
        this->m_kernelSizeY = 2 * cv::saturate_cast<ito::int16>(ceil(sqrt(a))) + 1;
    }

    a = - 2.0 * sigmaSizeX * sigmaSizeX * log( sigmaSizeX * epsilonSizeX * sqrt(2.0 * M_PI) );
    if (a < 0.5)
    {
        this->m_kernelSizeX = 1;
    }
    else
    {
        this->m_kernelSizeX = 2 * cv::saturate_cast<ito::int16>(ceil(sqrt(a))) + 1;
        
    }

    this->m_AnchorX = this->m_kernelSizeX / 2;
    this->m_AnchorY = this->m_kernelSizeY / 2;
    
    this->m_bufsize = this->m_kernelSizeX * this->m_kernelSizeY;

    this->m_pRowKernel = new ito::float64[this->m_kernelSizeX];
    this->m_pColKernel = new ito::float64[this->m_kernelSizeY];

    if (this->m_kernelSizeY == 1)
    {
        m_pColKernel[0] = 1.0;
    }
    else
    {
        ito::float64 sigSQR = sigmaSizeY * sigmaSizeY;
        ito::float64 norm = 1 / sqrt( 2.0 * M_PI * sigSQR);
        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            this->m_pColKernel[y] =  norm * exp( pow( (ito::float64)(y - this->m_AnchorY), 2) * -0.5 / sigSQR ); 
        }
        norm = 0.0;

        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            norm +=  this->m_pColKernel[y]; 
        }

        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            this->m_pColKernel[y] /=  norm; 
        }
    }

    if (this->m_kernelSizeX == 1)
    {
        m_pRowKernel[0] = 1.0;
    }
    else
    {
        ito::float64 sigSQR = sigmaSizeX * sigmaSizeX;
        ito::float64 norm = 1 / sqrt( 2.0 * M_PI * sigSQR);
        for(int x = 0; x < this->m_kernelSizeX; x++)
        {
            this->m_pRowKernel[x] =  norm * exp( pow( (ito::float64)(x - this->m_AnchorX), 2) * -0.5 / sigSQR ); 
        }
        norm = 0.0;

        for(int x = 0; x < this->m_kernelSizeX; x++)
        {
            norm +=  this->m_pRowKernel[x]; 
        }

        for(int x = 0; x < this->m_kernelSizeX; x++)
        {
            this->m_pRowKernel[x] /=  norm; 
        }  
    }

    if (this->m_pColKernel != NULL && this->m_pRowKernel != NULL)
    {
        this->m_initialized = true;
    }

    this->m_pInLinesFiltered = new ito::float64*[this->m_kernelSizeY];
    

    if (this->m_pInLinesFiltered == NULL)
    {
        this->m_initialized = false;
    }
    else
    {
        memset(this->m_pInLinesFiltered, 0, sizeof(ito::float64*));
        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            this->m_pInLinesFiltered[y] = new ito::float64[roiXSize + this->m_kernelSizeX];
            if (this->m_pInLinesFiltered[y] == NULL)
            {
                this->m_initialized = false;
                break;
            }
        }
    }


    this->m_isFilled = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> GaussianFilter<_Tp>::GaussianFilter( const ito::DataObject *in, 
                                                           ito::DataObject *out, 
                                                           ito::int32 roiX0, 
                                                           ito::int32 roiY0, 
                                                           ito::int32 roiXSize, 
                                                           ito::int32 roiYSize,
                                                           ito::int32 kernelSizeX,
                                                           ito::int32 kernelSizeY,
                                                           ito::float64 sigmaSizeX, 
                                                           ito::float64 sigmaSizeY
) :GenericFilterEngine<_Tp>::GenericFilterEngine()
{ 
    this->m_pInpObj = in;
    this->m_pOutObj = out;
    
    this->m_x0 = roiX0;
    this->m_y0 = roiY0;
    
    this->m_dx = roiXSize;
    this->m_dy = roiYSize;
    
    this->m_kernelSizeX = kernelSizeX;
    this->m_kernelSizeY = kernelSizeY;

    this->m_AnchorX = this->m_kernelSizeX / 2;
    this->m_AnchorY = this->m_kernelSizeY / 2;
    
    this->m_bufsize = this->m_kernelSizeX * this->m_kernelSizeY;

    this->m_pRowKernel = new ito::float64[this->m_kernelSizeX];
    this->m_pColKernel = new ito::float64[this->m_kernelSizeY];

    if (this->m_kernelSizeY == 1)
    {
        m_pColKernel[0] = 1.0;
    }
    else
    {
        ito::float64 sigSQR = sigmaSizeY * sigmaSizeY;
        ito::float64 norm = 1 / sqrt( 2.0 * M_PI * sigSQR);
        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            this->m_pColKernel[y] =  norm * exp( pow( (ito::float64)(y - this->m_AnchorY), 2) * -0.5 / sigSQR ); 
        }
        norm = 0.0;

        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            norm +=  this->m_pColKernel[y]; 
        }

        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            this->m_pColKernel[y] /=  norm; 
        }
    }

    if (this->m_kernelSizeX == 1)
    {
        m_pRowKernel[0] = 1.0;
    }
    else
    {
        ito::float64 sigSQR = sigmaSizeX * sigmaSizeX;
        ito::float64 norm = 1.0 / sqrt( 2.0 * M_PI * sigSQR);
        for(int x = 0; x < this->m_kernelSizeX; x++)
        {
            this->m_pRowKernel[x] =  norm * exp( pow( (ito::float64)(x - this->m_AnchorX), 2) * -0.5 / sigSQR ); 
        }
        norm = 0.0;

        for(int x = 0; x < this->m_kernelSizeX; x++)
        {
            norm +=  this->m_pRowKernel[x]; 
        }

        for(int x = 0; x < this->m_kernelSizeX; x++)
        {
            this->m_pRowKernel[x] /=  norm; 
        }  
    }

    if (this->m_pColKernel != NULL && this->m_pRowKernel != NULL)
    {
        this->m_initialized = true;
    }

    this->m_pInLinesFiltered = new ito::float64*[this->m_kernelSizeY];
    

    if (this->m_pInLinesFiltered == NULL)
    {
        this->m_initialized = false;
    }
    else
    {
        memset(this->m_pInLinesFiltered, 0, sizeof(ito::float64*));
        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            this->m_pInLinesFiltered[y] = new ito::float64[roiXSize + this->m_kernelSizeX];
            if (this->m_pInLinesFiltered[y] == NULL)
            {
                this->m_initialized = false;
                break;
            }
        }
    }


    this->m_isFilled = false;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> void GaussianFilter<_Tp>::clearFunc()
{
    // Do nothing
    m_isFilled = false;
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> GaussianFilter<_Tp>::~GaussianFilter()
{
    if (m_pRowKernel != NULL)
    {
        delete m_pRowKernel;
    }
    if (m_pColKernel != NULL)
    {
        delete m_pColKernel;
    }

    if (this->m_pInLinesFiltered != NULL)
    {
        for(int y = 0; y < this->m_kernelSizeY; y++)
        {
            if (this->m_pInLinesFiltered[y] == NULL)
            {
                delete this->m_pInLinesFiltered[y];
            }
        }
        delete this->m_pInLinesFiltered;
    }
}

//-----------------------------------------------------------------------------------------------
template<typename _Tp> /*ito::RetVal*/ void GaussianFilter<_Tp>::filterFunc()
{

    ito::int16 kernelLastY = this->m_kernelSizeY - 1;
    ito::int16 kernelLastX = this->m_kernelSizeX - 1;
   
    if (!this->m_isFilled)
    {
        this->m_isFilled = true;

        //#if (USEOMP)
        //#pragma omp for schedule(guided)
        //#endif  
        for(ito::int16 y = 0; y < this->m_kernelSizeY; y++)
        {

            for(ito::int32 x = 0; x < this->m_dx; x++)
            {
                this->m_pInLinesFiltered[y][x] = 0.0;

                for(ito::int32 xk = 0; xk < this->m_kernelSizeX; xk++)
                {
                    this->m_pInLinesFiltered[y][x] += ((ito::float64) this->m_pInLines[y][x + xk]) * this->m_pRowKernel[xk];
                }
            }
        }
    }
    else
    { 
        
        //if (true)
        //{
        //    #pragma omp barrier
        //}

        //#if (USEOMP)
        //#pragma omp master
        //{
        //#endif  

        ito::float64* temp = this->m_pInLinesFiltered[0];
        for(ito::int16 y = 1; y < this->m_kernelSizeY; y++)
        {
            this->m_pInLinesFiltered[y - 1] = this->m_pInLinesFiltered[y]; 
        }
        this->m_pInLinesFiltered[kernelLastY] = temp;

        //#if (USEOMP)
        //}
        //#endif

        //#if (USEOMP)
        //#pragma omp for schedule(guided)
        //#endif  
        for(ito::int32 x = 0; x < this->m_dx; x++)
        {
            this->m_pInLinesFiltered[kernelLastY][x] = 0.0;

            for(ito::int32 xk = 0; xk < this->m_kernelSizeX; xk++)
            {
                this->m_pInLinesFiltered[kernelLastY][x] += ((ito::float64) this->m_pInLines[kernelLastY][x + xk]) * this->m_pRowKernel[xk];
            }
        }

    }

    //if (true)
    //{
    //    #pragma omp barrier
    //}

#if (USEOMP)
    #pragma omp parallel num_threads(ito::AddInBase::getMaximumThreadCount())
    {
    #pragma omp for schedule(guided)
#endif  
    for(ito::int32 x = 0; x < this->m_dx; x++)
    {
        this->m_pOutLine[x] = 0;

        for(ito::int32 yk = 0; yk < this->m_kernelSizeY; yk++)
        {
            this->m_pOutLine[x] += cv::saturate_cast<_Tp>(this->m_pInLinesFiltered[yk][x] * this->m_pColKernel[yk]);
        }
    }
    
#if (USEOMP)
    }
#endif

    //return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail This function gives the standard parameters for most of the genericfilter blocks to the addin-interface.
\param[out]   paramsMand  Mandatory parameters for the filter function
\param[out]   paramsOpt   Optinal parameters for the filter function
\author ITO
\sa  BasicFilters::genericLowPassFilter, 
\date
*/
ito::RetVal BasicFilters::genericGaussianParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("N-dimensional dataObject of any real type. This object is processed plane-by-plane.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Resulting dataObject that has the same size and type than the sourceImage.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("kernelx", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Size of kernel in x-direction (odd values only)").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("kernely", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Size of kernel in y-direction (odd values only)").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("sigmaX", ito::ParamBase::Double | ito::ParamBase::In, 0.1, 5000.0, 0.84, tr("Standard deviation in x direction").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("sigmaY", ito::ParamBase::Double | ito::ParamBase::In, 0.1, 5000.0, 0.84, tr("Standard deviation in y direction").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("replaceNaN", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 1, NaN values of sourceImage will be replaced by interpolated filter values in destImage, else destImage also contains NaN values at these positions (0, default).").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::genericGaussianFilterDoc = QObject::tr("2D gaussian blur filter that convolves the source image with a Gaussian kernel. \n\
\n\
The convolution kernel is rectangular with a side length of (kernely, kernelx). The gaussian kernel for a pixel (x0,y0) is defined by: \n\
\n\
f(x,y) = A * exp(-[(x-x0)^2/(2*sigma_x^2) + (y-y0)^2/(2*sigma_y^2)]) \n\
\n\
with \n\
\n\
A = 1 / (2 * pi * sigmaX * sigmaY)) \n\
\n\
Invalid pixels within the kernel are excluded from the convolution. It is possible to fill the invalid values \n\
by the Gaussian blurred value of the surrounding kernel, too, if the parameter 'replaceNaN' is set to 1. If the kernel only contains invalid values, \n\
no replacement is possible and the corresponding pixel in the destination image remains invalid. \n\
\n\
This filter can be applied inplace, hence, the source and destination image can be the same. This filter is implemented for all \n\
real data types. 3 or higher dimensional dataObjects are processed plane-by-plane, such that the destination image has always the same size \n\
and type than the input image");
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
ito::RetVal BasicFilters::genericGaussianFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject *dObjSrc = (*paramsMand)[0].getVal<const ito::DataObject*>();  //Input object
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();  //Filtered output object

    if (!dObjSrc)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is invalid").toLatin1().data());
    }
    else if (dObjSrc->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is empty").toLatin1().data());
    }
    if (!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination image is invalid").toLatin1().data());
    }

    if (dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if (ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    // get the kernelsize
    ito::int32 kernelX = (*paramsOpt)[0].getVal<int>();
    ito::int32 kernelY = (*paramsOpt)[1].getVal<int>();

    if (kernelX % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in x must be odd").toLatin1().data());
    }

    if (kernelY % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in y must be odd").toLatin1().data());
    }

    ito::float64 sigmaX = (*paramsOpt)[2].getVal<double>();
    ito::float64 sigmaY = (*paramsOpt)[3].getVal<double>();
    

    bool replaceNaN = (*paramsOpt)[4].getVal<int>() != 0 ? true : false; //false (default): NaN values in input image will become NaN in output, else: output will be interpolated (somehow)

    switch(dObjSrc->getType())
    {
        case ito::tInt8:
        {
            GaussianFilter<ito::int8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelX,
                                                kernelY, 
                                                sigmaX, 
                                                sigmaY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt8:
        {
            GaussianFilter<ito::uint8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelX,
                                                kernelY, 
                                                sigmaX, 
                                                sigmaY);;
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt16:
        {
            GaussianFilter<ito::int16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelX,
                                                kernelY, 
                                                sigmaX, 
                                                sigmaY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt16:
        {
            GaussianFilter<ito::uint16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelX,
                                                kernelY, 
                                                sigmaX, 
                                                sigmaY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt32:
        {
            GaussianFilter<ito::int32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelX,
                                                kernelY, 
                                                sigmaX, 
                                                sigmaY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat32:
        {
            GaussianFilter<ito::float32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelX,
                                                kernelY, 
                                                sigmaX, 
                                                sigmaY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat64:
        {
            GaussianFilter<ito::float64> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                kernelX,
                                                kernelY, 
                                                sigmaX, 
                                                sigmaY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add Protokoll
        QString msg;
        msg = tr("gaussian-filter with sigma %1 x %2").arg(sigmaX).arg(sigmaY);
        //        dObjDst -> addToProtocol(std::string(prot));

        if (replaceNaN)
        {
            msg.append( tr(" and removed NaN-values"));
        }

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    //int64 testend = cv::getTickCount() - teststart;
    //ito::float64 duration = (ito::float64)testend / cv::getTickFrequency();
    //std::cout << "Time: " << duration << "ms\n";

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail This function gives the standard parameters for most of the genericfilter blocks to the addin-interface.
\param[out]   paramsMand  Mandatory parameters for the filter function
\param[out]   paramsOpt   Optinal parameters for the filter function
\author ITO
\sa  BasicFilters::genericLowPassFilter, 
\date
*/
ito::RetVal BasicFilters::genericGaussianEpsilonParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("N-dimensional dataObject of any real type. This object is processed plane-by-plane.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Resulting dataObject that has the same size and type than the sourceImage.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("sigmaX", ito::ParamBase::Double | ito::ParamBase::In, 0.1, 5000.0, 0.84, tr("Standard deviation in x direction").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("epsilonX", ito::ParamBase::Double | ito::ParamBase::In, 0.00001, 1.0, 0.001, tr("Stop condition in x-direction, the kernel in x-direction is cropped for Gaussian values below this limit").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("sigmaY", ito::ParamBase::Double | ito::ParamBase::In, 0.1, 5000.0, 0.84, tr("Standard deviation in y direction").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("epsilonY", ito::ParamBase::Double | ito::ParamBase::In, 0.00001, 1.0, 0.001, tr("Stop condition in y-direction, the kernel in y-direction is cropped for Gaussian values below this limit").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("replaceNaN", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 1, NaN values of sourceImage will be replaced by interpolated filter values in destImage, else destImage also contains NaN values at these positions (0, default).").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::genericGaussianEpsilonFilterDoc = QObject::tr("2D gaussian blur filter that convolves the source image with a Gaussian kernel. \n\
\n\
The convolution kernel is rectangular with a side length that is determined by the epsilon values epsilonX and epsilonY. \n\
The gaussian kernel for a pixel (x0,y0) is defined by: \n\
\n\
f(x,y) = A * exp(-[(x-x0)^2/(2*sigmaX^2) + (y-y0)^2/(2*sigmaY^2)]) \n\
\n\
with \n\
\n\
A = 1 / (2 * pi * sigmaX * sigmaY)) \n\
\n\
The size of the kernel is adjusted such that the limit values in x- and y-direction fullfill the following inequation: \n\
\n\
f(x,y=0) = exp(-[(x-x0)^2/(2*sigmaX^2)]) / sqrt(2 * pi * sigmaX^2) >= epsilonX \n\
f(x=0,y) = exp(-[(y-y0)^2/(2*sigmaY^2)]) / sqrt(2 * pi * sigmaY^2) >= epsilonY \n\
\n\
Invalid pixels within the kernel are excluded from the convolution. It is possible to fill the invalid values \n\
by the Gaussian blurred value of the surrounding kernel, too, if the parameter 'replaceNaN' is set to 1. If the kernel only contains invalid values, \n\
no replacement is possible and the corresponding pixel in the destination image remains invalid. \n\
\n\
This filter can be applied inplace, hence, the source and destination image can be the same. This filter is implemented for all \n\
real data types. 3 or higher dimensional dataObjects are processed plane-by-plane, such that the destination image has always the same size \n\
and type than the input image");
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
ito::RetVal BasicFilters::genericGaussianEpsilonFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal retval = ito::retOk;
    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();  //Input object
    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();  //Filtered output object

    if (!dObjSrc)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is invalid").toLatin1().data());
    }
    else if (dObjSrc->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is empty").toLatin1().data());
    }
    if (!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination image is invalid").toLatin1().data());
    }

    if (dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        // Nothing
    }
    else if (ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;

    // get the kernelsize
    ito::float64 sigmaX = (*paramsMand)[2].getVal<double>();
    ito::float64 epsX = (*paramsMand)[3].getVal<double>();
    ito::float64 sigmaY = (*paramsMand)[4].getVal<double>();
    ito::float64 epsY = (*paramsMand)[5].getVal<double>();

    bool replaceNaN = (*paramsOpt)[0].getVal<int>() != 0 ? true : false; //false (default): NaN values in input image will become NaN in output, else: output will be interpolated (somehow)

    //ito::int32 z_length = dObjSrc->calcNumMats();  // get the number of Mats (planes) in the input object

    switch(dObjSrc->getType())
    {
        case ito::tInt8:
        {
            GaussianFilter<ito::int8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                sigmaX, 
                                                epsX, 
                                                sigmaY, 
                                                epsY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt8:
        {
            GaussianFilter<ito::uint8> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                sigmaX, 
                                                epsX, 
                                                sigmaY, 
                                                epsY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt16:
        {
            GaussianFilter<ito::int16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                sigmaX, 
                                                epsX, 
                                                sigmaY, 
                                                epsY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tUInt16:
        {
            GaussianFilter<ito::uint16> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                sigmaX, 
                                                epsX, 
                                                sigmaY, 
                                                epsY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tInt32:
        {
            GaussianFilter<ito::int32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                sigmaX, 
                                                epsX, 
                                                sigmaY, 
                                                epsY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat32:
        {
            GaussianFilter<ito::float32> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                sigmaX, 
                                                epsX, 
                                                sigmaY, 
                                                epsY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
        case ito::tFloat64:
        {
            GaussianFilter<ito::float64> filterEngine(dObjSrc, 
                                                dObjDst, 
                                                0, 
                                                0, 
                                                dObjSrc->getSize(dObjSrc->getDims() - 1), 
                                                dObjSrc->getSize(dObjSrc->getDims() - 2), 
                                                sigmaX, 
                                                epsX, 
                                                sigmaY, 
                                                epsY);
            filterEngine.runFilter(replaceNaN);
        }
        break;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add Protokoll
        QString msg;
        msg = tr("gaussian-filter with sigma %1 x %2").arg(sigmaX).arg(sigmaY);
        //        dObjDst -> addToProtocol(std::string(prot));

        if (replaceNaN)
        {
            msg.append( tr(" and removed NaN-values"));
        }

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    //int64 testend = cv::getTickCount() - teststart;
    //ito::float64 duration = (ito::float64)testend / cv::getTickFrequency();
    //std::cout << "Time: " << duration << "ms\n";

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!\detail This function gives the parameters for the spike filter to the addin-interface.
\param[out]   paramsMand  Mandatory parameters for the filter function
\param[out]   paramsOpt   Optinal parameters for the filter function
\author ITO
\sa  BasicFilters::genericLowPassFilter, 
\date
*/
ito::RetVal BasicFilters::spikeCompFilterStdParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("source dataObject of any real data type.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("destImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("destination dataObject (can be the same than source)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("kernelx", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Odd kernelsize in x").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("kernely", ito::ParamBase::Int | ito::ParamBase::In, 1, 101, 3, tr("Odd kernelsize in y").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("delta", ito::ParamBase::Double | ito::ParamBase::In, std::numeric_limits<ito::float64>::epsilon() * 10, std::numeric_limits<ito::float64>::max(), 0.05, tr("Delta value for comparison").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("newValue", ito::ParamBase::Double, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("value set to clipped values").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("replaceSpikeByNewValue", ito::ParamBase::Int, 0, 1, 1, tr("if 1 (default), a spike value is replaced by the parameter 'newValue', else it is replaced by the value from the filtered object.").toLatin1().data());
        paramsOpt->append(param);

    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Type, typename _cType> ito::RetVal SpikeCompBlock(const ito::DataObject *scrObj, const ito::DataObject &compObj, ito::DataObject *outObj, const _cType delta, const _Type newValue, const bool noCopyNeeded, const bool replaceByNewValueNotCompObj)
{
    const _Type *compRowPtr = NULL;
    const _Type *scrRowPtr = NULL;
    _Type *dstRowPtr = NULL;

    const cv::Mat *scrMat;
    const cv::Mat *compMat;
    cv::Mat *dstMat;

    ito::int32 z_length = outObj->getNumPlanes();  // get the number of Mats (planes) in the input object

    if (std::numeric_limits<_Type>::is_integer && std::numeric_limits<_cType>::is_integer)
    {
        for(int plane = 0; plane < z_length; ++plane)
        {
            scrMat = scrObj->get_mdata()[scrObj->seekMat(plane, z_length)];
            compMat = compObj.get_mdata()[compObj.seekMat(plane, z_length)];
            dstMat = outObj->get_mdata()[outObj->seekMat(plane, z_length)];
            
            if (noCopyNeeded)
            {
                for(int y = 0; y < compMat->rows; y++)
                {
                    scrRowPtr = scrMat->ptr<_Type>(y);
                    compRowPtr = compMat->ptr<_Type>(y);
                    dstRowPtr = dstMat->ptr<_Type>(y);
                    for(int x = 0; x < compMat->cols; x++)
                    {
                        if (myAbs((_cType)(compRowPtr[x]) - (_cType)(scrRowPtr[x])) > delta)
                        {
                            dstRowPtr[x] = replaceByNewValueNotCompObj ? newValue : compRowPtr[x];
                        }
                    }            
                }
            }
            else
            {
                for(int y = 0; y < compMat->rows; y++)
                {
                    scrRowPtr = scrMat->ptr<_Type>(y);
                    compRowPtr = compMat->ptr<_Type>(y);
                    dstRowPtr = dstMat->ptr<_Type>(y);
                    for(int x = 0; x < compMat->cols; x++)
                    {
                        if (myAbs((_cType)(compRowPtr[x]) - (_cType)(scrRowPtr[x])) > delta)
                        {
                            dstRowPtr[x] = replaceByNewValueNotCompObj ? newValue : compRowPtr[x];
                        }
                        else
                        {
                            dstRowPtr[x] = scrRowPtr[x];
                        }
                    }            
                }
            }
        }
    }
    else
    {
        for(int plane = 0; plane < z_length; plane++)
        {
            scrMat = scrObj->get_mdata()[scrObj->seekMat(plane, z_length)];
            compMat = compObj.get_mdata()[compObj.seekMat(plane, z_length)];
            dstMat = outObj->get_mdata()[outObj->seekMat(plane, z_length)];

            if (noCopyNeeded)
            {
                for(int y = 0; y < compMat->rows; y++)
                {
                    scrRowPtr = scrMat->ptr<_Type>(y);
                    compRowPtr = compMat->ptr<_Type>(y);
                    dstRowPtr = dstMat->ptr<_Type>(y);
                    for(int x = 0; x < compMat->cols; x++)
                    {
                        if (ito::isFinite(scrRowPtr[x]) && ito::isFinite(compRowPtr[x]) && myAbs((_cType)(compRowPtr[x]) - (_cType)(scrRowPtr[x])) > delta)
                        {
                            dstRowPtr[x] = replaceByNewValueNotCompObj ? newValue : compRowPtr[x];
                        }
                    }            
                }
            }
            else
            {
                for(int y = 0; y < compMat->rows; y++)
                {
                    scrRowPtr = scrMat->ptr<_Type>(y);
                    compRowPtr = compMat->ptr<_Type>(y);
                    dstRowPtr = dstMat->ptr<_Type>(y);
                    for(int x = 0; x < compMat->cols; x++)
                    {
                        if (ito::isFinite(scrRowPtr[x]) && ito::isFinite(compRowPtr[x]) && myAbs((_cType)(compRowPtr[x]) - (_cType)(scrRowPtr[x])) > delta)
                        {
                            dstRowPtr[x] = replaceByNewValueNotCompObj ? newValue : compRowPtr[x];
                        }
                        else
                        {
                            dstRowPtr[x] = scrRowPtr[x];
                        }
                    }            
                }
            }
        }    
    }
    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::spikeMedianFilterDoc = QObject::tr("Performs a median filter with kernelsize (kernelx, kernely) and pixelwise comparison of \n\
filtered image and original image to remove spikes according to delta value. \n\
\n\
At first the input image is filtered by a median filter (filter: 'medianFilter') using the given kernel size. The output \n\
image then contains the input image, where every pixel is replaced 'newValue' if the absolute distance between the median \n\
filtered and the input image at the specific pixel is bigger than 'delta'.\n\
\n\
This filter also works inplace (same source and destination).");
/*!
\detail This function use the generic filter engine with the median filter to set values to remove spikes from an image
\param[in|out]   paramsMand  Mandatory parameters for the filter function
\param[in|out]   paramsOpt   Optinal parameters for the filter function
\param[out]   outVals   Outputvalues, not implemented for this function
\param[in]   lowHigh  Flag which toggles low or high filter
\author ITO
\sa  BasicFilters::genericStdParams
\date
*/
ito::RetVal BasicFilters::spikeMedianFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    return spikeGenericFilter(paramsMand, paramsOpt, paramsOut, tGenericMedian);
}
//----------------------------------------------------------------------------------------------------------------------------------
const QString BasicFilters::spikeMeanFilterDoc = QObject::tr("Performs a linear mean filter with kernelsize (kernelx, kernely) and pixelwise comparison of \n\
filtered image and original image to remove spikes according to delta value. \n\
\n\
At first the input image is filtered by a linear mean filter (filter: 'lowPassFilter') using the given kernel size. The output \n\
image then contains the input image, where every pixel is replaced 'newValue' if the absolute distance between the median \n\
filtered and the input image at the specific pixel is bigger than 'delta'. \n\
\n\
This filter also works inplace (same source and destination).");
/*!
\detail This function use the generic filter engine with the median filter to set values to remove spikes from an image
\param[in|out]   paramsMand  Mandatory parameters for the filter function
\param[in|out]   paramsOpt   Optinal parameters for the filter function
\param[out]   outVals   Outputvalues, not implemented for this function
\param[in]   lowHigh  Flag which toggles low or high filter
\author ITO
\sa  BasicFilters::genericStdParams
\date
*/
ito::RetVal BasicFilters::spikeMeanFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    return spikeGenericFilter(paramsMand, paramsOpt, paramsOut, tGenericLowPass);
}

ito::RetVal BasicFilters::spikeGenericFilter(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut, const tFilterType filter )
{
    ito::RetVal retval = ito::retOk;
    const ito::DataObject *dObjSrc = (*paramsMand)[0].getVal<ito::DataObject*>();  //Input object
    ito::DataObject *dObjDst = (*paramsMand)[1].getVal<ito::DataObject*>();  //Filtered output object
    // get the kernelsize
    ito::int32 kernelsizex = (*paramsMand)[2].getVal<int>();
    ito::int32 kernelsizey = (*paramsMand)[3].getVal<int>();

    ito::float64 deltaValue = (*paramsMand)[4].getVal<double>();
    ito::float64 newValue = (*paramsMand)[5].getVal<double>();

    bool replaceSpikeByNewValue = paramsOpt->at(0).getVal<int>() > 0;

    ito::DataObject tempFilterObject;

    bool noCopyNeeded = false;

    if (!dObjSrc)    // Report error if input object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Source image is invalid").toLatin1().data());
    }
    else if (dObjSrc->getDims() < 1) // Report error of input object is empty
    {
        return ito::RetVal(ito::retError, 0, tr("source object must not be empty.").toLatin1().data());
    }
    if (!dObjDst)    // Report error of output object is not defined
    {
        return ito::RetVal(ito::retError, 0, tr("Destination image is invalid").toLatin1().data());
    }

    if (dObjSrc == dObjDst) // If both pointer are equal or the object are equal take it else make a new destObject
    {
        noCopyNeeded = true;
    }
    else if (ito::dObjHelper::dObjareEqualShort(dObjSrc, dObjDst))
    {
        dObjDst->deleteAllTags();
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }
    else
    {
        (*dObjDst) = ito::DataObject(dObjSrc->getDims(), dObjSrc->getSize(), dObjSrc->getType(), dObjSrc->getContinuous());
        dObjSrc->copyAxisTagsTo(*dObjDst);
        dObjSrc->copyTagMapTo(*dObjDst);
    }

    // Check if input type is allowed or not
    retval = ito::dObjHelper::verifyDataObjectType(dObjSrc, "dObjSrc", 7, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tFloat32, ito::tFloat64);
    if (retval.containsError())
        return retval;


    if (kernelsizex % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in x must be odd").toLatin1().data());
    }

    if (kernelsizey % 2 == 0) //even
    {
        return ito::RetVal(ito::retError, 0, tr("Error: kernel in y must be odd").toLatin1().data());
    }

    QVector<ito::Param> paramsMandMedian;
    QVector<ito::Param> paramsOptMedian;
    QVector<ito::Param> paramsOutMedian;

    retval += BasicFilters::genericStdParams(&paramsMandMedian, &paramsOptMedian, &paramsOutMedian);
    if (retval.containsError())
    {
        retval.appendRetMessage(tr(" while running spike removal by median filter").toLatin1().data());
        return retval;
    }

    ito::Param* param = NULL;
    param = ito::getParamByName(&paramsMandMedian, "sourceImage", &retval);
    if (param == NULL || retval.containsError())
    {
        retval.appendRetMessage(tr(" while running spike removal by median filter").toLatin1().data());
        return retval;        
    }
    else
    {
        param->setVal<const ito::DataObject*>(dObjSrc);
    }

    param = ito::getParamByName(&paramsMandMedian, "destImage", &retval);
    if (param == NULL || retval.containsError())
    {
        retval.appendRetMessage(tr(" while running spike removal by median filter").toLatin1().data());
        return retval;        
    }
    else
    {
        param->setVal<ito::DataObject*>(&tempFilterObject);
    }

    param = ito::getParamByName(&paramsMandMedian, "kernelx", &retval);
    if (param == NULL || retval.containsError())
    {
        retval.appendRetMessage(tr(" while running spike removal by median filter").toLatin1().data());
        return retval;        
    }
    else
    {
        param->setVal<int>(kernelsizex);
    }

    param = ito::getParamByName(&paramsMandMedian, "kernely", &retval);
    if (param == NULL || retval.containsError())
    {
        retval.appendRetMessage(tr(" while running spike removal by median filter").toLatin1().data());
        return retval;        
    }
    else
    {
        param->setVal<int>(kernelsizey);
    }

    QVector<ito::ParamBase> paramsMandMedianBase;
    QVector<ito::ParamBase> paramsOptMedianBase;
    QVector<ito::ParamBase> paramsOutMedianBase;

    for(int i = 0; i < paramsMandMedian.size(); i++)
    {
        paramsMandMedianBase.append((ito::ParamBase)paramsMandMedian[i]);
    }
    for(int i = 0; i < paramsOptMedian.size(); i++)
    {
        paramsOptMedianBase.append((ito::ParamBase)paramsOptMedian[i]);
    }
    for(int i = 0; i < paramsOutMedian.size(); i++)
    {
        paramsOutMedianBase.append((ito::ParamBase)paramsOutMedian[i]);
    }

    switch(filter)
    {
        case tGenericLowPass:  
            retval += genericLowPassFilter(&paramsMandMedianBase, &paramsOptMedianBase, &paramsOutMedianBase);
            if (retval.containsError())
            {
                retval.appendRetMessage(tr(" while running spike removal by mean filter").toLatin1().data());
                return retval;        
            }
        break;
        case tGenericMedian:  
            retval += genericMedianFilter(&paramsMandMedianBase, &paramsOptMedianBase, &paramsOutMedianBase);
            if (retval.containsError())
            {
                retval.appendRetMessage(tr(" while running spike removal by median filter").toLatin1().data());
                return retval;        
            }
        break;
        default:
        {
            return ito::RetVal(ito::retError, 0, tr("Error: filter type not implemented for generic spike filter").toLatin1().data());  
        }
    }

    double newValueCasted = 0.0;
    double deltaCasted = 0.0;

    switch(dObjSrc->getType())
    {
        case ito::tInt8:
        {
            ito::int8 newVal = cv::saturate_cast<ito::int8>(newValue);
            newValueCasted = (ito::float64)(newVal);

            ito::int32 deltaVal = cv::saturate_cast<ito::int32>(deltaValue);
            deltaCasted = (ito::float64)(deltaVal);

            SpikeCompBlock<ito::int8, ito::int32>(dObjSrc, tempFilterObject, dObjDst, deltaVal, newVal, noCopyNeeded, replaceSpikeByNewValue);
        }
        break;
        case ito::tUInt8:
        {
            ito::uint8 newVal = cv::saturate_cast<ito::uint8>(newValue);
            newValueCasted = (ito::float64)(newVal);

            ito::int32 deltaVal = cv::saturate_cast<ito::int32>(deltaValue);
            deltaCasted = (ito::float64)(deltaVal);

            SpikeCompBlock<ito::uint8, ito::int32>(dObjSrc, tempFilterObject, dObjDst, deltaVal, newVal, noCopyNeeded, replaceSpikeByNewValue);
        }
        break;
        case ito::tInt16:
        {
            ito::int16 newVal = cv::saturate_cast<ito::int16>(newValue);
            newValueCasted = (ito::float64)(newVal);

            ito::int32 deltaVal = cv::saturate_cast<ito::int32>(deltaValue);
            deltaCasted = (ito::float64)(deltaVal);

            SpikeCompBlock<ito::int16, ito::int32>(dObjSrc, tempFilterObject, dObjDst, deltaVal, newVal, noCopyNeeded, replaceSpikeByNewValue);
        }
        break;
        case ito::tUInt16:
        {
            ito::uint16 newVal = cv::saturate_cast<ito::uint16>(newValue);
            newValueCasted = (ito::float64)(newVal);

            ito::int32 deltaVal = cv::saturate_cast<ito::int32>(deltaValue);
            deltaCasted = (ito::float64)(deltaVal);
            SpikeCompBlock<ito::uint16, ito::int32>(dObjSrc, tempFilterObject, dObjDst, deltaVal, newVal, noCopyNeeded, replaceSpikeByNewValue);
        }
        break;
        case ito::tInt32:
        {
            ito::int32 newVal = cv::saturate_cast<ito::int32>(newValue);
            newValueCasted = (ito::float64)(newVal);

            ito::int32 deltaVal = cv::saturate_cast<ito::int32>(deltaValue);
            deltaCasted = (ito::int32)(deltaVal);

            SpikeCompBlock<ito::int32, ito::int32>(dObjSrc, tempFilterObject, dObjDst, deltaVal, newVal, noCopyNeeded, replaceSpikeByNewValue);
        }
        break;
        case ito::tFloat32:
        {
            ito::float32 newVal = std::numeric_limits<ito::float32>::quiet_NaN();
            if (ito::isFinite(newValue))
            {
                ito::float32 newVal = cv::saturate_cast<ito::float32>(newValue);
                newValueCasted = (ito::float64)(newVal);
            }
            else
            {
                newValueCasted = std::numeric_limits<ito::float64>::quiet_NaN();
            }

            ito::float32 deltaVal = cv::saturate_cast<ito::float32>(deltaValue);
            deltaCasted = (ito::float32)(deltaVal);
            SpikeCompBlock<ito::float32, ito::float32>(dObjSrc, tempFilterObject, dObjDst, deltaVal, newVal, noCopyNeeded, replaceSpikeByNewValue);
        }
        break;
        case ito::tFloat64:
        {
            newValueCasted = newValue;
            deltaCasted = deltaValue;
            SpikeCompBlock<ito::float64, ito::float64>(dObjSrc, tempFilterObject, dObjDst, deltaValue, newValue, noCopyNeeded, replaceSpikeByNewValue);
        }
        break;
    }

    // if no errors reported -> create new dataobject with values stored in cvMatOut
    if (!retval.containsError())
    {
        // Add protocol
        QString msg = tr("spike removal filter via median filter with kernel %1 x %2 for delta = %3 and newValue %4 ").arg(kernelsizex).arg(kernelsizey).arg(deltaCasted).arg(newValueCasted);

        dObjDst->addToProtocol(std::string(msg.toLatin1().data()));
    }

    //int64 testend = cv::getTickCount() - teststart;
    //ito::float64 duration = (ito::float64)testend / cv::getTickFrequency();
    //std::cout << "Time: " << duration << "ms\n";

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------