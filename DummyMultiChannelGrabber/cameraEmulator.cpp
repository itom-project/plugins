/* ********************************************************************
    Plugin "DummyMultiChannelGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut fuer Technische Optik (ITO),
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

#include "cameraEmulator.h"

#include <qelapsedtimer.h>
#include <qthread.h>

//-------------------------------------------------------------------------------------
/** @func   fastrand
 *   @brief  function for pseudo random values
 *
 *   This function delivers the noise for the image.
 */
template <typename _Tp> inline _Tp fastrand(cv::RNG& rng, _Tp maxval, float offset, float gain)
{
    return cv::saturate_cast<_Tp>(offset * maxval + gain * (((ito::uint32)rng.next()) & maxval));
}

//-------------------------------------------------------------------------------------
/** @func   fastrand
 *   @brief  function for pseudo random values
 *
 *   This function delivers the noise for the image.
 */
template <typename _Tp>
inline _Tp fastrand_mean(cv::RNG& rng, _Tp maxval, ito::uint8 numMeans, float offset, float gain)
{
    ito::uint32 val = 0;

    for (ito::uint8 i = 0; i < numMeans; ++i)
    {
        val += ((ito::uint32)rng.next()) & maxval;
    }

    return cv::saturate_cast<_Tp>(offset * maxval + (gain / (float)numMeans) * val);
}

//-------------------------------------------------------------------------------------
/** @func   gaussFunc
 *   @brief  function for 2d Gaussian function
 *
 *   This function delivers a 2d dataObject with a Gaussian function
 */
template <typename _Tp> ito::RetVal gaussFunc(cv::RNG& rng, ito::DataObject &dObj, float amplitude)
{
    int width = dObj.getSize(1);
    int height = dObj.getSize(0);
    _Tp* rowPtr;
    float xval, yval;
    int planeID = dObj.seekMat(0);

    float yRandOffset = rng.uniform(0.f, 20.f);
    float xRandOffset = rng.uniform(0.f, 20.f);
    float aRandOfset = rng.uniform(0.f, 20.f);

    float sigmaX = width * rng.uniform(0.09f, 0.11f);
    float sigmaY = height * rng.uniform(0.09f, 0.11f);

    for (int y = 0; y < height; y++)
    {
        rowPtr = dObj.rowPtr<_Tp>(planeID, y);
        yval = ((y - height / 2 + yRandOffset) * ((float)y - height / 2 + yRandOffset)) /
            (2.0f * sigmaY * sigmaY);

        for (int x = 0; x < width; x++)
        {
            xval = ((x - width / 2 + xRandOffset) * ((float)x - width / 2 + xRandOffset)) /
                (2.0f * sigmaX * sigmaX);
            rowPtr[x] = (float)(amplitude - aRandOfset) * exp(-(xval + yval));
        }
    }

    return ito::retOk;
}



//-------------------------------------------------------------------------------------
CameraEmulator::CameraEmulator() :
    m_imageColorAlpha(false),
    m_imageMonoBpp(8)
{

}

//-------------------------------------------------------------------------------------
void CameraEmulator::configureImageMono(const QRect& roi, int bpp)
{
    int h = roi.height();
    int w = roi.width();

    switch (bpp)
    {
    case 8:
        m_imageMonoBpp = 8;
        m_imageMono = ito::DataObject(h, w, ito::tUInt8);
        break;
    case 10:
        m_imageMonoBpp = 10;
        m_imageMono = ito::DataObject(h, w, ito::tUInt16);
        break;
    case 12:
        m_imageMonoBpp = 12;
        m_imageMono = ito::DataObject(h, w, ito::tUInt16);
        break;
    case 16:
        m_imageMonoBpp = 16;
        m_imageMono = ito::DataObject(h, w, ito::tUInt16);
    default:
        break;
    }
}

//-------------------------------------------------------------------------------------
void CameraEmulator::configureImageTopography(const QRect& roi, bool singlePrecision)
{
    int h = roi.height();
    int w = roi.width();

    if (singlePrecision)
    {
        m_imageTopography = ito::DataObject(h, w, ito::tFloat32);
    }
    else
    {
        m_imageTopography = ito::DataObject(h, w, ito::tFloat64);
    }
}

//-------------------------------------------------------------------------------------
void CameraEmulator::configureImageColor(const QRect& roi, bool alpha)
{
    int h = roi.height();
    int w = roi.width();
    m_imageColorAlpha = alpha;

    m_imageColor = ito::DataObject(h, w, ito::tRGBA32);
}

//-------------------------------------------------------------------------------------
bool CameraEmulator::grabImages(bool imgMono, bool imgFloat, bool imgColor, float exposureTimeMs)
{
    bool result = true;
    QElapsedTimer timer;
    timer.start();

    if (imgMono)
    {
        result &= grabMono();
    }

    if (imgFloat)
    {
        result &= grabTopography();
    }

    if (imgColor)
    {
        result &= grabColor();
    }

    exposureTimeMs -= timer.elapsed();

    if (exposureTimeMs > 0)
    {
        QThread::usleep(1000 * exposureTimeMs);
    }

    return result;
}

//-------------------------------------------------------------------------------------
bool CameraEmulator::grabMono()
{
    const unsigned int low = 0;
    const unsigned int high = std::pow(m_imageMonoBpp, 2) - 1;

    cv::randu(*(m_imageMono.getCvPlaneMat(0)), low, high);

    return true;
}

//-------------------------------------------------------------------------------------
bool CameraEmulator::grabTopography()
{
    cv::RNG& rng = cv::theRNG();

    if (m_imageTopography.getType() == ito::tFloat32)
    {
        gaussFunc<ito::float32>(rng, m_imageTopography, 10.0);
    }
    else
    {
        gaussFunc<ito::float64>(rng, m_imageTopography, 10.0);
    }

    return true;
}

//-------------------------------------------------------------------------------------
bool CameraEmulator::grabColor()
{
    cv::Mat* mat = m_imageColor.getCvPlaneMat(0);
    ito::uint32* rowPtr;

    for (int r = 0; r < mat->rows; ++r)
    {
        rowPtr = mat->ptr<ito::uint32>(r);

        for (int c = 0; c < mat->cols; ++c)
        {
            rowPtr[c] = cv::randu<ito::uint32>();

            if (!m_imageColorAlpha)
            {
                ((ito::Rgba32*)rowPtr)[c].a = 255;
            }
        }
    }

    return true;
}
