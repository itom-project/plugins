/* ********************************************************************
    Plugin "FringeProj" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include <math.h>
#include "FringeProj.h"

#include "DataObject/dataobj.h"
#include "DataObject/dataObjectFuncs.h"

#include <QtCore/QtPlugin>

#include "pluginVersion.h"
#include "gitVersion.h"

//#include "common/helperCommon.h"

#define CUDAPI 3.1415926535897932384626433832795f
#define CUDA2PI 6.2831853071795864769252867665590f
#define CUDAPI2 1.5707963267948966192313216916398f
#define MAXPHASHIFT 16
#define MAXGRAYBITS 12
#define BPSLUTSIZE 1 << MAXGRAYBITS
#define CFPTYPE float
#define INVPHA -10 //must be a fixed-point value smaller than -pi and smaller than 0

#ifdef USEOPENMP
    #define useomp 1
#else
    #define useomp 0
#endif

int NTHREADS = 2;

struct tFloatArray2D {
    long sizes[2];
    float *vals;
};

struct tShortArray2D {
    long sizes[2];
    short *vals;
};

struct tvArray2D {
    long sizes[2];
    void *vals;
};

struct tvArray3D {
    long sizes[3];
    void *vals;
};

//----------------------------------------------------------------------------------------------------------------------------------
/** function to calculate a lookup table for the gray code
*   @param [in]  maxBits        the number of graycode bits for which the lut should be calculated
*   @param [out] BPS2CITable    lookup table to transform gc code words to code index numbers
*   @param [out] bitshift       order of two numbers with the bitshifts for the graycode bits
*
*   This function calculates a lookup table for the transformation of bitplane stack values to code index values.
*   This a necessary step in the gray code calculation as the gray code does not form ordered ascending numbers in
*   a binray number system. To speed up the calculation afterwards a lookup table is calculated here which gives
*   the according ordered number for a gray code bitplane stack number.
*/
int CalcBPS2CILut(const char maxBits, unsigned short *BPS2CITable, unsigned short *bitshift)
{
#if (USEOMP)
    #pragma omp parallel num_threads(NTHREADS)
    {
#endif
    int b = 0, bitmask = 0, i = 0, invert = 0;

#if (USEOMP)
    #pragma omp for schedule(guided)
#endif
    for (int g = 0; g <= (1 << maxBits); g++)
    {
        b = g;
        invert = 0;
        for (i = maxBits; i >= 0; i--)
        {
           bitmask = 1 << i;
           if (invert)
           {
                b = b ^ bitmask;
           }
           if (g & bitmask)
           {
                invert = !invert;
           }
        }
        BPS2CITable[g] = b;
    }
    for (int g = 0; g <= maxBits; g++)
    {
        bitshift[g] = 1 << (maxBits - g - 1);
//        bitshift[g] = 1 << g;
    }
    #if (USEOMP)
    }
    #endif

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** calculate codeindexmap out of a number of graycode images.
*   @param [in]  images     the recorded graycode images
*   @param [in]  contThreas absolute minimum threshold (modulation) for a valid graycode pixel
*   @param [out] CiMap      codeindex map (fringe order map)
*
*   The first two images (entirely black and white) are used to calculate the threshold for each pixel.
*   Using this threshold for each image of the sequence the pixel bit values (0 or 1) are determined.
*   The result of this calculus is a graycode code word which does not directly represent an ordered
*   fringe number. To convert the gc code word to an ordered fringe number the lookup table calculated
*   with the \ref CalcBPS2CILut function is used.
*/
template<typename _Tp> int CalcCIMap(struct tvArray3D **images, const float contThreas, const float brightUpperLimit, const float darkLowerLimit, const float safetyFactor, struct tShortArray2D **CiMap)
{
    int ret = 0;
    int width = (*images)->sizes[2];
    int height = (*images)->sizes[1];
    int pagesize = width * height;
    int numbits = (*images)->sizes[0] - 2;
    unsigned short bps2cilut[BPSLUTSIZE];
    unsigned short bitshift[MAXGRAYBITS];



    CalcBPS2CILut(numbits, bps2cilut, bitshift);

#if (useomp)
    #pragma omp parallel num_threads(NTHREADS)
    {
#endif

    _Tp threas = 0;
    unsigned short bitplanestack = 0;
    float brightPixel = 0.0;
    float darkPixel = 0.0;
    _Tp safetyWidth = 0;
    _Tp brightPixelTp = 0;
    _Tp darkPixelTp = 0;
    _Tp curVal;
    bool err;

#if (useomp)
    #pragma omp for schedule(guided)
#endif
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            brightPixelTp = ((_Tp*)(*images)->vals)[pagesize + y * width + x];
            darkPixelTp = ((_Tp*)(*images)->vals)[y * width + x];
            brightPixel = static_cast<float>(brightPixelTp);
            darkPixel = static_cast<float>(darkPixelTp);

            if (brightPixel - darkPixel > contThreas && brightPixel <= brightUpperLimit && darkPixel >= darkLowerLimit)
            {
                threas = (darkPixelTp + brightPixelTp) / 2.0;
                bitplanestack = 0;

                if (safetyFactor > 0.0)
                {
                    safetyWidth = static_cast<_Tp>((safetyFactor * 0.5) * (brightPixel - darkPixel));
                    err = false;

                    for(int imgNr = 2; imgNr < numbits + 2; imgNr++)
                    {
                        curVal = ((_Tp*)(*images)->vals)[pagesize * imgNr + y * width + x];
                        if (curVal > (threas + safetyWidth))
                        {
                            bitplanestack |= bitshift[imgNr - 2];
                        }
                        else if ( curVal >= (threas - safetyWidth) )
                        {
                            err = true;
                            break;
                        }
                    }

                    if (!err)
                    {
                        ((short*)(*CiMap)->vals)[y * width + x] = static_cast<short>(bps2cilut[bitplanestack]);
                    }
                    else
                    {
                        ((short*)(*CiMap)->vals)[y * width + x] = static_cast<short>(INVPHA);
                    }
                }
                else
                {
                    for(int imgNr = 2; imgNr < numbits + 2; imgNr++)
                    {
                        if (((_Tp*)(*images)->vals)[pagesize * imgNr + y * width + x] > threas)
                        {
                            bitplanestack |= bitshift[imgNr - 2];
                        }
                    }

                    ((short*)(*CiMap)->vals)[y * width + x] = static_cast<short>(bps2cilut[bitplanestack]);
                }
            }
            else
            {
                ((short*)(*CiMap)->vals)[y * width + x] = static_cast<short>(INVPHA);
            }
        }
    }
    #if (useomp)
    }
    #endif

//end:
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
template int CalcCIMap<ito::uint8>(struct tvArray3D **images, const float contThreas, const float brightUpperLimit, const float darkLowerLimit, const float safetyFactor, struct tShortArray2D **CiMap);
template int CalcCIMap<ito::uint16>(struct tvArray3D **images, const float contThreas, const float brightUpperLimit, const float darkLowerLimit, const float safetyFactor, struct tShortArray2D **CiMap);

//----------------------------------------------------------------------------------------------------------------------------------
/** calculate the modulus 2 pi phase map of a sequence of 4 90° shifted phase images
*   @param [in]  images     the recorded fringe images (90° shifted)
*   @param [in]  contThreas absolute minimum threshold (modulation) for a valid phase pixel
*   @param [in]  overExp    gray level over which recorded intensity values are overexposed, i.e. the phase for that pixel invalid
*   @param [out] PhaseMap   the calculated modulo 2 pi phase map
*   @param [out] ModulationMap the calculated modulation map
*
*   For all pixels the modulo 2 pi phase map is calculated using the simple carré algorithm atan2pi((I2 - I4) / (I1 - I3))
*   in addition it is checked if the modulation for this pixel is above the threshold and the maximum recorded intensity
*   below the overexposed value, otherwise the pixel is marked as invalid.
*/
template<typename _Tp> ito::RetVal calcPhaseMap4Tmpl(const ito::DataObject *source, const float contThreas, const _Tp overExp, ito::DataObject &phaseMap, ito::DataObject &modulationMap)
{
    /*source must be 3D and size(0) must be 4. Check this before!*/

    int width = source->getSize(2);
    int height = source->getSize(1);

    if (phaseMap.getDims() == 0)
    {
        phaseMap = ito::DataObject(height, width, ito::tFloat32);
    }

    if (modulationMap.getDims() == 0)
    {
        modulationMap = ito::DataObject(height, width, ito::tFloat32);
    }

#if (useomp)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
        int buf1, buf2, max, max1, max2;
        ito::float32 contrast;
        const _Tp *rowI0, *rowI1, *rowI2, *rowI3;
        ito::float32 *rowPhase, *rowModulation;
        _Tp intens[4];

#if (useomp)
#pragma omp for schedule(guided)
#endif

        for (int y = 0; y < height; y++)
        {
            rowI0 = source->rowPtr<_Tp>(0, y);
            rowI1 = source->rowPtr<_Tp>(1, y);
            rowI2 = source->rowPtr<_Tp>(2, y);
            rowI3 = source->rowPtr<_Tp>(3, y);
            rowPhase = phaseMap.rowPtr<ito::float32>(0, y);
            rowModulation = modulationMap.rowPtr<ito::float32>(0, y);

            for (int x = 0; x < width; x++)
            {
                intens[0] = rowI0[x];
                intens[1] = rowI1[x];
                intens[2] = rowI2[x];
                intens[3] = rowI3[x];

                if (intens[0] > intens[1])
                {
                    max1 = intens[0];
                }
                else
                {
                    max1 = intens[1];
                }
                if (intens[2] > intens[3])
                {
                    max2 = intens[2];
                }
                else
                {
                    max2 = intens[3];
                }
                if (max1 > max2)
                {
                    max = max1;
                }
                else
                {
                    max = max2;
                }

                buf1 = intens[3] - intens[1];
                buf2 = intens[2] - intens[0];
                contrast = sqrt((float)(buf1 * buf1 + buf2 * buf2));

                if ((contrast > contThreas) && (((overExp) && (max < overExp)) || !overExp))
                {
                    rowPhase[x] = atan2((CFPTYPE)buf1, (CFPTYPE)buf2);
                    rowModulation[x] = contrast;
                }
                else
                {
                    rowPhase[x] = INVPHA;
                    rowModulation[x] = 0;
                }
            }
        }
#if (useomp)
    }
#endif

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** calculate the modulus 2 pi phase map of a sequence of equally shifted fringe images
*   @param [in]  images     the recorded fringe images
*   @param [in]  contThreas absolute minimum threshold (modulation) for a valid phase pixel
*   @param [in]  overExp    gray level over which recorded intensity values are overexposed, i.e. the phase for that pixel invalid
*   @param [out] PhaseMap   the calculated modulo 2 pi phase map
*   @param [out] ModulationMap the calculated modulation map
*
*   For all pixels the modulo 2 pi phase map is calculated using the sum of sines and cosines. It is important that the phase
*   shifts are equal for all images. As in the version for four images the contrast threshold and overexposure are
*   checked and out of boundaries pixels are marked invalid.
*/
template<typename _Tp> ito::RetVal calcPhaseMapNTmpl(const ito::DataObject *source, const float contThreas, const _Tp overExp, ito::DataObject &phaseMap, ito::DataObject &modulationMap)
{
    int width = source->getSize(2);
    int height = source->getSize(1);
    int numPhases = source->getSize(0);
    CFPTYPE sines[MAXPHASHIFT], cosines[MAXPHASHIFT];

    for (int nimg = 0; nimg < numPhases; ++nimg)
    {
        sines[nimg] = sin(-CUDA2PI / 4 + nimg * CUDA2PI / (CFPTYPE)(numPhases));
        cosines[nimg] = -1.0 * cos(-CUDA2PI / 4 + nimg * CUDA2PI / (CFPTYPE)(numPhases));
    }

#if (useomp)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
        ito::float32 buf1, buf2;
        ito::float32 contrast;
        ito::float32 *rowPhase, *rowModulation;
        _Tp intens;
        const _Tp* row[MAXPHASHIFT];
        _Tp max = 0;

#if (useomp)
#pragma omp for schedule(guided)
#endif
    for (int y = 0; y < height; y++)
    {
        for (int n = 0; n < numPhases; ++n)
        {
            row[n] = source->rowPtr<const _Tp>(n, y);
        }
        rowPhase = phaseMap.rowPtr<ito::float32>(0, y);
        rowModulation = modulationMap.rowPtr<ito::float32>(0, y);

        for (int x = 0; x < width; x++)
        {
            buf1 = 0;
            buf2 = 0;
            max = 0;

            for (int n = 0; n < numPhases; n++)
            {
                intens = row[n][x];
                if (intens > max)
                {
                    max = intens;
                }

                buf1 += intens * cosines[n];
                buf2 += intens * sines[n];
            }
            contrast = sqrt(buf1 * buf1 + buf2 * buf2);

            if ((contrast > contThreas) && (((overExp) && (max < overExp)) || !overExp))
            {
                rowPhase[x] = atan2(buf1, buf2);
                rowModulation[x] = contrast;
            }
            else
            {
                rowPhase[x] = INVPHA;
                rowModulation[x] = 0;
            }
        }
    }
    #if (useomp)
    }
    #endif

    return ito::retOk;
}

//-----------------------------------------------------------------------------
/** unwrap a modulo 2 pi phase map using a gray code code index map
*   @param [in]  contThreas contrast threshold for a valid pixel
*   @param [in]  maxpha     maximum valid phase value
*   @param [in]  CiMap      code index map out of graycode images
*   @param [in]  RawPhase   modulo 2 pi phase map
*   @param [out] PhaseMap   unwrapped phase map
*
*   The modulo 2 pi phase map is unwrapped using the code index map out of the graycode images. If a pixel has a contrast value
*   below the threshold or a phase value above the maximum phase value it is marked invalid.
*/
int UnwrapPhaseGray(float contThreas, unsigned short maxpha, struct tShortArray2D **CiMap, struct tFloatArray2D **RawPhase, struct tFloatArray2D **ModulationMap, struct tFloatArray2D **PhaseMap)
{
    int ret = 0;
    int width = (*CiMap)->sizes[1];
    int height = (*CiMap)->sizes[0];
    short *ciMap = (short*)(*CiMap)->vals;
    CFPTYPE *modMap = (CFPTYPE*)(*ModulationMap)->vals;
    CFPTYPE *phaseMap = (CFPTYPE*)(*PhaseMap)->vals;

    #if (useomp)
    #pragma omp parallel num_threads(NTHREADS)
    {
    #pragma omp for schedule(guided)
    #endif
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            CFPTYPE rawPhase = ((CFPTYPE*)(*RawPhase)->vals)[y * width + x];

            if (rawPhase < (INVPHA + 0.1) || ciMap[y * width + x] < (INVPHA + 0.1) || (modMap[y * width + x] == 0)) //rawPhase is invalid, set phaseMap to NaN
            {
                phaseMap[y * width + x] = std::numeric_limits<ito::float32>::signaling_NaN();
            }
            else
            {
                //Phase Unwrapping mit Codeindizes
                //-Pi/2                    //pi/2
                if ((rawPhase >= -CUDAPI2) && (rawPhase <= CUDAPI2))
                {
                    if (modMap[y * width + x] > contThreas)
                    {
                        phaseMap[y * width + x] = rawPhase + CUDAPI + (ciMap[y * width + x] / 2) * CUDA2PI;
                    }
                    else
                    {
                         phaseMap[y * width + x] = INVPHA;
                    }
                }
                else if (rawPhase > CUDAPI2)
                {
                    if (modMap[y * width + x] > contThreas)
                    {
                        phaseMap[y * width + x] = rawPhase + CUDAPI + ((ciMap[y * width + x] + 1) / 2 - 1) * CUDA2PI;
                    }
                    else
                    {
                        phaseMap[y * width + x] = INVPHA;
                    }
                }
                else //(rawPhase < CUDAPI2)
                {
                    if (modMap[y * width + x] > contThreas)
                    {
                        phaseMap[y * width + x] = rawPhase + CUDAPI + ((ciMap[y * width + x] + 1) / 2) * CUDA2PI;
                    }
                    else
                    {
                        phaseMap[y * width + x] = INVPHA;
                    }
                }

                //check boundaries -> 0<=phaseMap <=maxpha
                if ( (phaseMap[y * width + x] < 0) || (phaseMap[y * width + x] > maxpha) )
                {
                    phaseMap[y * width + x] = std::numeric_limits<ito::float32>::signaling_NaN();
                }
            }

            //// Added rawPhase == INVPHA check!!!
            //if ((phaseMap[y * width + x] < 0) || (phaseMap[y * width + x] > maxpha) || (rawPhase < (INVPHA + 0.1))) //the last +0.1 is for solving floating point insecurity
            //{
            //    //phaseMap[y * width + x] = INVPHA; // This leads to problems with
            //    phaseMap[y * width + x] = std::numeric_limits<ito::float32>::signaling_NaN();
            //}
        }
    }
    #if (useomp)
    }
    #endif

//end:
    return ret;
}

//-----------------------------------------------------------------------------
/** calculate an unwrapped phase map out of graycode and phase shifted images
*   @param [in]  images     the recorded phase images
*   @param [in]  numBits    number of graycode bits
*   @param [in]  numPhaShift number of phase shifted images
*   @param [in]  contThreas contrast threshold for a pixel to become valid
*   @param [in]  overExp    maximum intensity value
*   @param [out] PhaseMap   the unwrapped phase map
*   @param [out] ModulationMap modulation map
*
*   In principle this is just a convenience function summing up the above functions for calculating a raw phase map
*   out of phase shifted images, calculate a code index map out of gray code images and to unwrap the raw phase
*   using the code index map. Anyway the function should give a speed up compared to a separate call to all the
*   functions above. In case the intermediate results aren't required this function should be used to calculate
*   an unwrapped phase map for fringe projection systems based on graycode and phase shifting.
*/
template<typename _Tp> int CalcPhaseAbs(struct tvArray3D **images, const unsigned char numBits, const unsigned char numPhaShift, const float contThreas, const unsigned short maxpha, const _Tp overExp, struct tFloatArray2D **PhaseMap, struct tFloatArray2D **ModulationMap)
{
    int ret = 0;

end:
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FringeProjInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(FringeProj)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FringeProjInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(FringeProj)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
FringeProjInterface::FringeProjInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("FringeProj");

    m_description = QObject::tr("Algorithms used for fringe projection (phase shifting and Gray code)");
    m_detaildescription = QObject::tr("This DLL contains several reconstruction algorithms for fringe projection.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);


    NTHREADS  = QThread::idealThreadCount();
}

//----------------------------------------------------------------------------------------------------------------------------------
FringeProjInterface::~FringeProjInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
FringeProj::FringeProj() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
FringeProj::~FringeProj()
{
    FilterDef *filter;
    foreach(filter, m_filterList)
    {
        delete filter;
    }
    m_filterList.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize filter functions within this addIn
*    @param [in]    paramsMand    mandatory parameters that have to passed to the addIn on initialization
*    @param [in]    paramsOpt    optional parameters that can be passed to the addIn on initialization
*    @return                    retError in case of an error
*
*    Here are the filter functions defined that are available through this addIn.
*    These are:
*        - calcCiMap            calculate code index map (graycode)
*        - calcPhaseMapN        calculate phase map from n phase images
*        - calcPhaseMap4        calculate phase map from 4 90° shifted phase images
*        - unwrapPhaseGray    unwrap modulo 2 pi phase map using a code index map
*/
ito::RetVal FringeProj::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(
        FringeProj::calcCiMap, FringeProj::calcCiMapParams, tr("Calculate the indexmap for graycode image stack"));
    m_filterList.insert("calcCiMap", filter);

    filter = new FilterDef(
        FringeProj::calcPhaseMapN,
        FringeProj::calcPhaseMapNParams,
        tr("Reconstructs wrapped phase from N phaseshifted images with a shift of 2pi / N. The definition of the phase is equal to calcPhaseMap4."));
    m_filterList.insert("calcPhaseMapN", filter);

    filter = new FilterDef(
        FringeProj::calcPhaseMap4,
        FringeProj::calcPhaseMap4Params,
        tr("Reconstructs wrapped phase from four 90degree phase shifted images. The phase value is determined using the Carré algorithm: atan2(I3-I1,I2-I0)."));
    m_filterList.insert("calcPhaseMap4", filter);

    filter = new FilterDef(
        FringeProj::unwrapPhaseGray,
        FringeProj::unwrapPhaseGrayParams,
        tr("Unwrapped phase by Graycode (CiMap)"));
    m_filterList.insert("unwrapPhaseGray", filter);

    filter = new FilterDef(
        FringeProj::createXYMaps,
        FringeProj::createXYMapsParams,
        tr("Creates the X- and Y-Map for the given disparity map. The values consider the given scaling factor and the disparity-dependent shift due to the tilted illumination."));
    m_filterList.insert("createXYMaps", filter);

    filter = new FilterDef(
        FringeProj::gray2DecLookup,
        FringeProj::gray2DecLookupParams,
        gray2DecLookupDoc);
    m_filterList.insert("gray2DecLookup", filter);

    filter = new FilterDef(
        FringeProj::genGraycodePattern,
        FringeProj::genGraycodePatternParams,
        genGraycodePatternDoc);
    m_filterList.insert("genGraycodePattern", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FringeProj::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for code index map calculation
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    mand. Params:
*        - images, the graycode images
*        - contThreas contrast threshold for a valid pixel
*        - ciMap, the calculated codeindex map
*/
ito::RetVal FringeProj::calcCiMapParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("images", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Continuous 3D-image stack (uint8 or uint16)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("contThres", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 65535.0, 10.0, tr("Threshold for contrast. Only pixels with ((bright-dark) > contThres) will be considered").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("brightUpperLimit", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 65535.0, 65535.0, tr("Pixels with bright image > brightUpperLimit will be set to invalid (-10)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("darkLowerLimit", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 65535.0, 10.0, tr("Pixels with dark image < darkLowerLimit will be set to invalid (-10)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("ciMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("2D-Output object [int16, 2pi-phase-index (>=0) or -10 for invalid]").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("safetyFactor", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("Intensity values that lie in a band around the mean value (bright+dark)/2 will be ignored. The width of the band is given by safetyFactor*(bright-dark)").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** codeindex map calculation
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    function for the calculation of a codeindex map out of graycode images. This function
*    is only a wrapper that either calls the CPU or GPU based calculation. See \ref CalcCIMap
*/
ito::RetVal FringeProj::calcCiMap(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    int ret = 0;

    const ito::DataObject *dObjImages = (*paramsMand)[0].getVal<const ito::DataObject*>();
    if (!dObjImages || !dObjImages->getContinuous() || (dObjImages->getDims() != 3))
    {
        return ito::RetVal(ito::retError, 0, tr("image memory used by calcPhaseMap4 must be continuous!").toLatin1().data());
    }
    double contThreas = (*paramsMand)[1].getVal<double>();
    double brightUpperLimit = (*paramsMand)[2].getVal<double>();
    double darkLowerLimit = (*paramsMand)[3].getVal<double>();
    ito::DataObject *dObjCiMap = (*paramsMand)[4].getVal<ito::DataObject*>();

    double safetyFactor = paramsOpt->at(0).getVal<double>();

    struct tvArray3D *images = new tvArray3D;
    struct tShortArray2D *ciMap = new tShortArray2D;
    images->vals = (void*)(((cv::Mat *)dObjImages->get_mdata()[dObjImages->seekMat(0)])->data);
    images->sizes[2] = dObjImages->getSize(2);
    images->sizes[1] = dObjImages->getSize(1);
    images->sizes[0] = dObjImages->getSize(0);

    if ((dObjCiMap->getDims() < 2) || (dObjCiMap->getSize(1) != dObjImages->getSize(2)) || (dObjCiMap->getSize(0) != dObjImages->getSize(1)))
    {
        (*dObjCiMap) = ito::DataObject(dObjImages->getSize(1), dObjImages->getSize(2), ito::tInt16);
    }

    ciMap->vals = (short*)(((cv::Mat *)dObjCiMap->get_mdata()[dObjCiMap->seekMat(0)])->data);
    ciMap->sizes[1] = dObjCiMap->getSize(1);
    ciMap->sizes[0] = dObjCiMap->getSize(0);

    switch (dObjImages->getType())
    {
        case ito::tUInt8:
            if ((ret = CalcCIMap<ito::uint8>(&images, (float)contThreas, (float)brightUpperLimit, (float)darkLowerLimit, (float)safetyFactor, &ciMap)))
            {
                return ito::RetVal(ito::retError, 0, tr("error calling CalcCIMap").toLatin1().data());
            }
        break;

        case ito::tUInt16:
            if ((ret = CalcCIMap<ito::uint16>(&images, (float)contThreas, (float)brightUpperLimit, (float)darkLowerLimit, (float)safetyFactor, &ciMap)))
            {
                return ito::RetVal(ito::retError, 0, tr("error calling CalcCIMap").toLatin1().data());
            }
        break;

        default:
            return ito::RetVal(ito::retError, 0, tr("continuous image stack must have format uint8 or uint16").toLatin1().data());
            break;
    }

//end:
    if (images)
    {
        delete images;
    }
    if (ciMap)
    {
        delete ciMap;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for modulo 2 pi phase map calculation out of 4 90° images
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    mand. Params:
*        - images, the four 90° phase shifted images
*        - contThreas contrast threshold for a valid pixel
*        - overExp, over exposed threshold for a valid pixel
*        - phaseMap, the calculated modulo 2 pi phase map
*        - modulationMap, the modulation map
*/
ito::RetVal FringeProj::calcPhaseMap4Params(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("images", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("4 x Y x X image stack (uint8 or uint16) with 4 phase shifted images (90° each)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("contThreas", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 65535.0, 10.0, tr("Contrast threshold (val < threas = invalid)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("overExp", ito::ParamBase::Int | ito::ParamBase::In, 0, 65535, 255, tr("Value for over-exposed pixels or 0 if it should not be considered").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("phasePhase", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Wrapped phase result (float32, [-pi..pi] or -10 for invalid)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("modulationMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Map with intensity modulation (float32, [0..max. overExp], invalids are not marked here)").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** phase map calculation
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    function for the calculation of a modulo 2 pi phase map out of 4 90° shifted fringe images.
*    This function is only a wrapper that either calls the CPU or GPU based calculation. See \ref CalcPhaseMap4
*/
ito::RetVal FringeProj::calcPhaseMap4(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval;
    const ito::DataObject *dObjImages = (*paramsMand)[0].getVal<const ito::DataObject*>();
    retval += ito::dObjHelper::verify3DDataObject(dObjImages, "images", 4, 4, 0, std::numeric_limits<int>::max(), 0, std::numeric_limits<int>::max(), 2, ito::tUInt8, ito::tUInt16);

    if (!retval.containsError())
    {
        double contThreas = (*paramsMand)[1].getVal<double>();
        int overExp = (*paramsMand)[2].getVal<int>();
        ito::DataObject *dObjPhaseMap = (*paramsMand)[3].getVal<ito::DataObject*>();
        ito::DataObject *dObjModMap = (*paramsMand)[4].getVal<ito::DataObject*>();

        if ((dObjPhaseMap->getDims() < 2) || (dObjPhaseMap->getSize(1) != dObjImages->getSize(2)) || (dObjPhaseMap->getSize(0) != dObjImages->getSize(1)) || dObjPhaseMap->getType() != ito::tFloat32)
        {
            (*dObjPhaseMap) = ito::DataObject(dObjImages->getSize(1), dObjImages->getSize(2), ito::tFloat32);
        }

        if ((dObjModMap->getDims() < 2) || (dObjModMap->getSize(1) != dObjImages->getSize(2)) || (dObjModMap->getSize(0) != dObjImages->getSize(1)) || dObjModMap->getType() != ito::tFloat32)
        {
            (*dObjModMap) = ito::DataObject(dObjImages->getSize(1), dObjImages->getSize(2), ito::tFloat32);
        }

        switch (dObjImages->getType())
        {
        case ito::tUInt8:
            retval += calcPhaseMap4Tmpl<ito::uint8>(dObjImages, contThreas, cv::saturate_cast<ito::uint8>(overExp), *dObjPhaseMap, *dObjModMap);
            break;

        case ito::tUInt16:
            retval += calcPhaseMap4Tmpl<ito::uint16>(dObjImages, contThreas, cv::saturate_cast<ito::uint16>(overExp), *dObjPhaseMap, *dObjModMap);
            break;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for modulo 2 pi phase map calculation out of n images
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    mand. Params:
*        - images, the n equally shifted fringe images
*        - contThreas contrast threshold for a valid pixel
*        - overExp, over exposed threshold for a valid pixel
*        - phaseMap, the calculated modulo 2 pi phase map
*        - modulationMap, the modulation map
*/
ito::RetVal FringeProj::calcPhaseMapNParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("images", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("N x Y x X continuous image stack").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("contThreas", ito::ParamBase::Double, 0.0, 65535.0, 10.0, tr("Contrast threshold (val < threas = invalid)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("overExp", ito::ParamBase::Int | ito::ParamBase::In, 0, 65535, 255, tr("Value for over-exposed pixels or 0 if it should not be considered").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("phasePhase", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Wrapped phase result").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("modulationMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Map with intensity modulation").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** phase map calculation
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    function for the calculation of a modulo 2 pi phase map out of n equally shifted fringe images.
*    This function is only a wrapper that either calls the CPU or GPU based calculation. See \ref CalcPhaseMapN
*/
ito::RetVal FringeProj::calcPhaseMapN(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval;
    const ito::DataObject *dObjImages = (*paramsMand)[0].getVal<const ito::DataObject*>();
    retval += ito::dObjHelper::verify3DDataObject(dObjImages, "images", 3, MAXPHASHIFT, 0, std::numeric_limits<int>::max(), 0, std::numeric_limits<int>::max(), 2, ito::tUInt8, ito::tUInt16);

    if (!retval.containsError())
    {
        double contThreas = (*paramsMand)[1].getVal<double>();
        int overExp = (*paramsMand)[2].getVal<int>();
        ito::DataObject *dObjPhaseMap = (*paramsMand)[3].getVal<ito::DataObject*>();
        ito::DataObject *dObjModMap = (*paramsMand)[4].getVal<ito::DataObject*>();

        if ((dObjPhaseMap->getDims() < 2) || (dObjPhaseMap->getSize(1) != dObjImages->getSize(2)) || (dObjPhaseMap->getSize(0) != dObjImages->getSize(1)) || dObjPhaseMap->getType() != ito::tFloat32)
        {
            (*dObjPhaseMap) = ito::DataObject(dObjImages->getSize(1), dObjImages->getSize(2), ito::tFloat32);
        }

        if ((dObjModMap->getDims() < 2) || (dObjModMap->getSize(1) != dObjImages->getSize(2)) || (dObjModMap->getSize(0) != dObjImages->getSize(1)) || dObjModMap->getType() != ito::tFloat32)
        {
            (*dObjModMap) = ito::DataObject(dObjImages->getSize(1), dObjImages->getSize(2), ito::tFloat32);
        }

        switch (dObjImages->getType())
        {
        case ito::tUInt8:
            retval += calcPhaseMapNTmpl<ito::uint8>(dObjImages, contThreas, cv::saturate_cast<ito::uint8>(overExp), *dObjPhaseMap, *dObjModMap);
            break;

        case ito::tUInt16:
            retval += calcPhaseMapNTmpl<ito::uint16>(dObjImages, contThreas, cv::saturate_cast<ito::uint16>(overExp), *dObjPhaseMap, *dObjModMap);
            break;
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for unwrapped phase map calculation
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    mand. Params:
*        - contThreas contrast threshold for a valid pixel
*        - maxPha, maximum phase for a valid pixel
*        - ciMap, the code index map - calculated from the graycode images
*        - rawPhase, the modulo 2 pi phase map
*        - modulationMap, the modulation map out of the mod 2pi phase map calculation
*        - phaseMap, the unwrapped phase map
*/
ito::RetVal FringeProj::unwrapPhaseGrayParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("contThreas", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 65535.0, 10.0, tr("Contrast threshold (val < threas = invalid)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("maxPha", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 65535.0, 65535.0, tr("Highest possible unwrapped phase").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("ciMap", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D Inputobject from evaluated Graycode (int16)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("rawPhase", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D Raw (wrapped) phase (float32) (NaN is represented -10, else [-pi,pi])").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("modulationMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("2D Modulation map from phase evaluation (float32)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("phaseMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Absolute height value (result) [float32] range: [0,maxPha] or NaN for invalid phases").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** calculate unwrapped phase map from code index map and raw phase
*    @param [in]    paramsMand    mandatory parameters for cimap calculation
*    @param [in]    paramsOpt    optional parameters for cimap calculation
*
*    function for the calculation of the unwrapped phase map the code index map and the raw phase.
*    This function is only a wrapper that either calls the CPU or GPU based calculation. See \ref UnwrapPhaseGray
*/
ito::RetVal FringeProj::unwrapPhaseGray(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    int ret = 0;

    double contThreas = (*paramsMand)[0].getVal<double>();
    double maxPha = (*paramsMand)[1].getVal<double>();
    const ito::DataObject *dObjCiMap = (*paramsMand)[2].getVal<const ito::DataObject*>();
    const ito::DataObject *dObjRawPhase = (*paramsMand)[3].getVal<const ito::DataObject*>();
    const ito::DataObject *dObjModMap = (*paramsMand)[4].getVal<const ito::DataObject*>();
    ito::DataObject *dObjPhaseMap = (*paramsMand)[5].getVal<ito::DataObject*>();

    if (!dObjCiMap || !dObjCiMap->getContinuous() || (dObjCiMap->getDims() != 2)
        || !dObjRawPhase || !dObjRawPhase->getContinuous() || (dObjRawPhase->getDims() != 2)
        || !dObjModMap || !dObjModMap->getContinuous() || (dObjModMap->getDims() != 2))
    {
        return ito::RetVal(ito::retError, 0, tr("image memory used by unwrapPhaseGray must be continuous and all objects have 2 dimensions only!").toLatin1().data());
    }

    if (dObjCiMap->getType() != ito::tInt16)
    {
        return ito::RetVal(ito::retError, 0, tr("ciMap must have format int16 (short)").toLatin1().data());
    }
    if (dObjRawPhase->getType() != ito::tFloat32)
    {
        return ito::RetVal(ito::retError, 0, tr("rawPhase must have format float32").toLatin1().data());
    }
    if (dObjModMap->getType() != ito::tFloat32)
    {
        return ito::RetVal(ito::retError, 0, tr("modulationMap must have format float32").toLatin1().data());
    }

    struct tShortArray2D *ciMap = new tShortArray2D;
    struct tFloatArray2D *rawPhase = new tFloatArray2D, *modMap = new tFloatArray2D, *phaseMap = new tFloatArray2D;
    ciMap->vals = (short*)(((cv::Mat *)dObjCiMap->get_mdata()[dObjCiMap->seekMat(0)])->data);
    ciMap->sizes[1] = dObjCiMap->getSize(1);
    ciMap->sizes[0] = dObjCiMap->getSize(0);
    rawPhase->vals = (float*)(((cv::Mat *)dObjRawPhase->get_mdata()[dObjRawPhase->seekMat(0)])->data);
    rawPhase->sizes[1] = dObjRawPhase->getSize(1);
    rawPhase->sizes[0] = dObjRawPhase->getSize(0);
    modMap->vals = (float*)(((cv::Mat *)dObjModMap->get_mdata()[dObjModMap->seekMat(0)])->data);
    modMap->sizes[1] = dObjModMap->getSize(1);
    modMap->sizes[0] = dObjModMap->getSize(0);

    if ((ciMap->sizes[0] != rawPhase->sizes[0]) || (ciMap->sizes[1] != rawPhase->sizes[1])
        || (ciMap->sizes[0] != modMap->sizes[0]) || (ciMap->sizes[1] != modMap->sizes[1]))
    {
        delete ciMap;
        delete rawPhase;
        delete modMap;
        delete phaseMap;
        return ito::RetVal(ito::retError, 0, tr("input dataObject differ in size!").toLatin1().data());
    }

    if ((dObjPhaseMap->getDims() < 2) || (dObjPhaseMap->getSize(1) != dObjRawPhase->getSize(2)) || (dObjPhaseMap->getSize(0) != dObjRawPhase->getSize(1)))
    {
        (*dObjPhaseMap) = ito::DataObject(dObjRawPhase->getSize(0), dObjRawPhase->getSize(1), ito::tFloat32);
    }
    phaseMap->vals = (float*)(((cv::Mat *)dObjPhaseMap->get_mdata()[dObjPhaseMap->seekMat(0)])->data);
    phaseMap->sizes[1] = dObjPhaseMap->getSize(1);
    phaseMap->sizes[0] = dObjPhaseMap->getSize(0);

    if ((ret = UnwrapPhaseGray((float)contThreas, (short)maxPha, &ciMap, &rawPhase, &modMap, &phaseMap)))
    {
        delete ciMap;
        delete rawPhase;
        delete modMap;
        delete phaseMap;
        return ito::RetVal(ito::retError, 0, tr("error calling calcPhaseMap4").toLatin1().data());
    }

//end:
    if (ciMap)
    {
        delete ciMap;
    }
    if (rawPhase)
    {
        delete rawPhase;
    }
    if (modMap)
    {
        delete modMap;
    }
    if (phaseMap)
    {
        delete phaseMap;
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FringeProj::createXYMapsParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    ito::Param param;

    if (!retval.containsError())
    {
        param = ito::Param("dispMap", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D disparity map (float32)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("xMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("2D x-map (float32)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("yMap", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("2D y-map (float32)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("scale", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 65535.0, 10.0, tr("Base-Scaling value (mm/px)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("dLateral", ito::ParamBase::Double | ito::ParamBase::In, -65535.0, 65535.0, 0.0, tr("Lateral-shift per disparity value (mm/mm)").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("shiftInXNotY", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("0: lateral shift in y-direction, 1: in x-direction").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FringeProj::createXYMaps(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    const ito::DataObject *dispMap = (const ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    ito::DataObject *xMap = (ito::DataObject*)(*paramsMand)[1].getVal<void*>();
    ito::DataObject *yMap = (ito::DataObject*)(*paramsMand)[2].getVal<void*>();

    ito::float32 scale = cv::saturate_cast<ito::float32>( (*paramsMand)[3].getVal<double>() );
    ito::float32 dLateral = cv::saturate_cast<ito::float32>( (*paramsMand)[4].getVal<double>() );
    int shiftInXNotY = (*paramsMand)[5].getVal<int>();

    if (dispMap == NULL)
    {
        retval += ito::RetVal(ito::retError,0,tr("disparity map must not be NULL").toLatin1().data());
    }
    else
    {
        retval += ito::dObjHelper::verify2DDataObject( dispMap, "disparity Map", 0, 1000000, 0, 1000000, 1, ito::tFloat32 );

        if (!retval.containsError())
        {
            int height = dispMap->getSize(0);
            int width = dispMap->getSize(1);

            *xMap = ito::DataObject(height,width,ito::tFloat32);
            *yMap = ito::DataObject(height,width,ito::tFloat32);

            cv::Mat_<ito::float32> *cvXMap = (cv::Mat_<ito::float32>*)xMap->get_mdata()[0];
            cv::Mat_<ito::float32> *cvYMap = (cv::Mat_<ito::float32>*)yMap->get_mdata()[0];
            const cv::Mat_<ito::float32> *cvDispMap = (const cv::Mat_<ito::float32>*)dispMap->get_mdata()[0];

            ito::float32* cvXMapRow = NULL;
            ito::float32* cvYMapRow = NULL;
            const ito::float32* cvDispMapRow = NULL;
            ito::float32 t;

            if (shiftInXNotY)
            {
                for(int m = 0 ; m < height ; m++)
                {
                    cvXMapRow = (ito::float32*)cvXMap->ptr(m);
                    cvYMapRow = (ito::float32*)cvYMap->ptr(m);
                    cvDispMapRow = (ito::float32*)cvDispMap->ptr(m);
                    t = scale * (ito::float32)m;

                    for(int n = 0 ; n < width ; n++)
                    {
                        cvXMapRow[n] = scale * (ito::float32)n - dLateral * cvDispMapRow[n];
                        cvYMapRow[n] = t;
                    }
                }
            }
            else
            {
                for(int m = 0 ; m < height ; m++)
                {
                    cvXMapRow = (ito::float32*)cvXMap->ptr(m);
                    cvYMapRow = (ito::float32*)cvYMap->ptr(m);
                    cvDispMapRow = (ito::float32*)cvDispMap->ptr(m);
                    t = scale * (ito::float32)m;

                    for(int n = 0 ; n < width ; n++)
                    {
                        cvXMapRow[n] = scale * (ito::float32)n;
                        cvYMapRow[n] = t - dLateral * cvDispMapRow[n];
                    }
                }
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*      From Wikipedia: http://en.wikipedia.org/wiki/Gray_code
        The purpose of this function is to convert an unsigned
        binary number to reflected binary Gray code.
*/
template<typename _Tp> _Tp binaryToGray(_Tp num)
{
        return (num>>1) ^ num;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FringeProj::gray2DecLookupDoc = QObject::tr(
"creates a 1xN lookup table to convert a gray-code value into its corresponding decimal value. \n\
\n\
The gray-code has a maximal width of graycodeBitWidth. Hence, N corresponds 1 << (graycodeBitWidth+1) \n\
To apply this lookup table to a dataObject or numpy array, consider using the numpy method take(lut,array) that returns (lut[array[i]] for i in array).");

/*static*/ ito::RetVal FringeProj::gray2DecLookupParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("graycodeBitWidth", ito::ParamBase::Int | ito::ParamBase::In, 1, 32, 3, "number of bits in the gray code (number of used gray-code sequences)") );
    paramsMand->append( ito::Param("gcToDecLUT", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "lookup table, 1xN, uint8, uint16 or uint32 depending on graycodeBitWidth") );

    paramsOpt->append( ito::Param("offset", ito::ParamBase::Int | ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0, "possible offset that is subtracted from each decimal value in the lookup table. This is necessary if the graycode sequence has been generated using a positive offset") );
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FringeProj::gray2DecLookup(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal;

    int bitWidth = paramsMand->at(0).getVal<int>(); //1..32
    int offset = paramsOpt->at(0).getVal<int>();
    ito::DataObject *dataObject = paramsMand->at(1).getVal<ito::DataObject*>();
    if (!dataObject)
    {
        retVal += ito::RetVal(ito::retError, 0, "gcToDecLUT must not be NULL");
    }
    else
    {
        ito::uint32 maxValue = 1 << (bitWidth+1); //this is the maximum graycode-value - 1 that can occur with bitWidth bits.

        ito::DataObject lut;
        int byteWidth;

        if (bitWidth <= 8)
        {
            if ((maxValue >> 8) > 0)
            {
                retVal += ito::RetVal(ito::retError, 0, "type uint8 for lut is too small for holding all values");
            }

            lut = ito::DataObject(1, maxValue, ito::tUInt8);
            byteWidth = 1;
        }
        else if (bitWidth <= 16)
        {
            if ((maxValue >> 16) > 0)
            {
                retVal += ito::RetVal(ito::retError, 0, "type uint16 for lut is too small for holding all values");
            }

            lut = ito::DataObject(1, maxValue, ito::tUInt16);
            byteWidth = 2;
        }
        else
        {
            lut = ito::DataObject(1, maxValue, ito::tUInt32);
            byteWidth = 4;
        }

        if (!retVal.containsError())
        {
            //here decValue is converted to gray (easier) and the lookup table is inversely filled
            switch (byteWidth)
            {
            case 1:
                {
                    ito::uint8 gcValue;
                    ito::uint8 *rowPtr = lut.rowPtr(0,0);
                    for (ito::uint8 decValue = 0; decValue < maxValue; ++decValue)
                    {
                        gcValue = binaryToGray<ito::uint8>(decValue);
                        rowPtr[gcValue] = decValue - offset;
                    }
                }
                break;
            case 2:
                {
                    ito::uint16 gcValue;
                    ito::uint16 *rowPtr = (ito::uint16*)lut.rowPtr(0,0);
                    for (ito::uint16 decValue = 0; decValue < maxValue; ++decValue)
                    {
                        gcValue = binaryToGray<ito::uint16>(decValue);
                        rowPtr[gcValue] = decValue - offset;
                    }
                }
                break;
            case 4:
                {
                    ito::uint32 gcValue;
                    ito::uint32 *rowPtr = (ito::uint32*)lut.rowPtr(0,0);
                    for (ito::uint32 decValue = 0; decValue < maxValue; ++decValue)
                    {
                        gcValue = binaryToGray<ito::uint32>(decValue);
                        rowPtr[gcValue] = decValue - offset;
                    }
                }
                break;
            }

            *dataObject = lut;
        }
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString FringeProj::genGraycodePatternDoc = QObject::tr("generates the graycode pattern that fills up the given data object of type uint8, uint16 or uint32");

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FringeProj::genGraycodePatternParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::Param param;
    ito::RetVal retval = ito::retOk;
    retval += prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (retval.containsError()) return retval;

    paramsMand->append( ito::Param("dataObj", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "2d data object whose data is filled with the pattern along the given axis (0, 1). Type must be uint8, uint16 or uint32") );
    paramsMand->append( ito::Param("axis", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Out, 0, 1, 0, "axis along the pattern is generated (0: along columns, 1: along rows)") );

    paramsOpt->append( ito::Param("patternSequence", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "MxN data object of type uint8 with the single patterns (black = 0, white = 255). N corresponds to the necessary bit width, M is equal to the number of rows or columns of dataObj depending on axis") );
    paramsOpt->append( ito::Param("offset", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, "it is possible to apply an offset to each row- or column number before calculating the graycode. This can be done in order to 'center' the graycode sequence. If offsetAutoCenter is 1, the offset is set to [(1<<necessaryBitWidth) - (rows or cols)]/2") );
    paramsOpt->append( ito::Param("offsetAutoCenter", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, "see offset") );

    paramsOut->append( ito::Param("usedOffset", ito::ParamBase::Int | ito::ParamBase::Out, std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0, "applied offset to each pixel number, that is the gray-code is determined from [(row or col) + offset]") );
    paramsOut->append( ito::Param("bitWidth", ito::ParamBase::Int | ito::ParamBase::Out, 0, std::numeric_limits<int>::max(), 1, "bit width for the requested pattern") );

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal FringeProj::genGraycodePattern(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal;

    int axis = paramsMand->at(1).getVal<int>(); //0, 1
    ito::DataObject dObj = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "dataObj", ito::Range::all(), ito::Range::all(), retVal, -1, 3, ito::tUInt8, ito::tUInt16, ito::tUInt32);
    ito::DataObject *pattern = paramsOpt->at(0).getVal<ito::DataObject*>();
    bool offsetAutoCenter = paramsOpt->at(2).getVal<int>() > 0;
    int offset = paramsOpt->at(1).getVal<int>();

    if (retVal.containsError() == false)
    {
        int elems = (axis == 0) ? dObj.getSize(0) : dObj.getSize(1);
        int bitWidth = std::ceil(std::log((double)elems) / std::log((double)2));

        if (offsetAutoCenter)
        {
            offset = ((1 << bitWidth) - elems) / 2;
        }

        unsigned int hmask = (axis == 1) ? 1 : 0;
        unsigned int vmask = (axis == 0) ? 0xffffff : 0x000000;

        switch (dObj.getType())
        {
        case ito::tUInt8:
            {
                if ((offset + std::max(dObj.getSize(0), dObj.getSize(1))) > std::numeric_limits<ito::uint8>::max())
                {
                    retVal += ito::RetVal(ito::retError, 0, "the size of the data object including the possible offset exceeds the possible range of its data type uint8");
                }
                else
                {
                    ito::uint8 *rowPtr;
                    ito::uint8 rv;
                    for (int r = 0; r < dObj.getSize(0); ++r)
                    {
                        rowPtr = (ito::uint8*)(dObj.rowPtr(0, r));
                        rv = axis > 0 ? 0 : binaryToGray<ito::uint8>(r + offset);

                        for (int c = 0; c < dObj.getSize(1); ++c)
                        {
                            rowPtr[c] = axis > 0 ? binaryToGray<ito::uint8>(c + offset) + rv : rv;
                        }
                    }
                }
            }
            break;
        case ito::tUInt16:
            {
                if ((offset + std::max(dObj.getSize(0), dObj.getSize(1))) > std::numeric_limits<ito::uint16>::max())
                {
                    retVal += ito::RetVal(ito::retError, 0, "the size of the data object including the possible offset exceeds the possible range of its data type uint16");
                }
                else
                {
                    ito::uint16 *rowPtr;
                    ito::uint16 rv;
                    for (int r = 0; r < dObj.getSize(0); ++r)
                    {
                        rowPtr = (ito::uint16*)(dObj.rowPtr(0, r));
                        rv = axis > 0 ? 0 : binaryToGray<ito::uint16>(r + offset);

                        for (int c = 0; c < dObj.getSize(1); ++c)
                        {
                            rowPtr[c] = axis > 0 ? binaryToGray<ito::uint16>(c + offset) + rv : rv;
                        }
                    }
                }
            }
            break;
        case ito::tUInt32:
            {
                if ((offset + std::max(dObj.getSize(0), dObj.getSize(1))) > std::numeric_limits<ito::uint32>::max())
                {
                    retVal += ito::RetVal(ito::retError, 0, "the size of the data object including the possible offset exceeds the possible range of its data type uint32");
                }
                else
                {
                    ito::uint32 *rowPtr;
                    ito::uint32 rv;
                    for (int r = 0; r < dObj.getSize(0); ++r)
                    {
                        rowPtr = (ito::uint32*)(dObj.rowPtr(0, r));
                        rv = axis > 0 ? 0 : binaryToGray<ito::uint32>(r + offset);

                        for (int c = 0; c < dObj.getSize(1); ++c)
                        {
                            rowPtr[c] = axis > 0 ? binaryToGray<ito::uint32>(c + offset) + rv : rv;
                        }
                    }
                }
            }
            break;
        }

        if (pattern)
        {


            ito::DataObject p(elems, bitWidth, ito::tUInt8);
            ito::uint8 *ptr;

            for (ito::uint32 r = 0; r < (ito::uint32)elems; ++r)
            {
                ito::uint32 gray = binaryToGray<ito::uint32>(r + offset);
                ptr = (ito::uint8*)p.rowPtr(0,r);

                for (int b = 0; b < bitWidth; ++b)
                {
                    ptr[b] = (gray & (1 << b)) ? 255 : 0;
                }
            }

            *pattern = p;
        }

        //return real offset
        (*paramsOut)[0].setVal<int>(offset);

        //return real bitWidth
        (*paramsOut)[1].setVal<int>(bitWidth);
    }

    return retVal;
}
