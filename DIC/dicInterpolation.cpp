/* ********************************************************************
    Plugin "DIC" for itom software
    URL: http://lccv.ufal.br/
    Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil

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

//#include "dic.h"
#include "dicInterpolation.h"
#include "dicInterpolationMats.h"
#include "dicInterpolationBMatBiCu.h"

#include "opencv2/imgproc/imgproc.hpp"

extern int NTHREADS;

struct MatT AMatBiCu, AMatBiQu, AMatBiHe;

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal doInterpolateLinear(const _Tp* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags)
{
    // Here we have to do the real job
    // we loop over all points, using openmp to speed up a little bit
    int colwise = (flags & 512) > 0;
	int fSize, sSize;

	if (colwise)
	{
		fSize = height;
		sSize = width;
	}
	else
	{
		fSize = width;
		sSize = height;
	}

#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
    float fIdxPos;
    float sIdxPos;
    int sIdx1, sIdx2, fIdx1, fIdx2;
    float DY1, DY2, DX1, DX2;

    //float X1Y1, X2Y1, X1Y2, X2Y2, DX1, DX2, DY1, DY2;
    //int row1, col1, row2, col2;
    //float rowPos, colPos;

#if (USEOMP)
#pragma omp for schedule(guided)
#endif
    for (int npts = 0; npts < numPos; npts++)
    {
        if (colwise)
        {
            fIdxPos = positions[npts * 2 + 1];
            sIdxPos = positions[npts * 2];
        }
        else
        {
            fIdxPos = positions[npts * 2];
            sIdxPos = positions[npts * 2 + 1];
        }

        if ((fIdxPos >= fSize) || (sIdxPos >= sSize) || (fIdxPos < 0) || (sIdxPos < 0)
            || !std::isfinite(fIdxPos) || !std::isfinite(sIdxPos))
        {
            outPtr[npts * stepout] = std::numeric_limits<float>::max();
            outPtr[npts * stepout + 1] = std::numeric_limits<float>::max();
            outPtr[npts * stepout + 2] = std::numeric_limits<float>::max();
            continue;
        }

        if (sIdxPos < (sSize - 1))
        {
            sIdx1 = (int)floor(sIdxPos);
            sIdx2 = sIdx1 + 1;
        }
        else
        {
            sIdx1 = (int)floor(sIdxPos - 1);
            sIdx2 = sIdx1 + 1;
        }

        if (fIdxPos < (fSize - 1))
        {
            fIdx1 = (int)floor(fIdxPos);
            fIdx2 = fIdx1 + 1;
        }
        else
        {
            fIdx1 = (int)floor(fIdxPos - 1);
            fIdx2 = fIdx1 + 1;
        }

        float X1Y1 = inPtr[sIdx1 * stepin + fIdx1];
        float X1Y2 = inPtr[sIdx2 * stepin + fIdx1];
        float X2Y1 = inPtr[sIdx1 * stepin + fIdx2];
        float X2Y2 = inPtr[sIdx2 * stepin + fIdx2];

        if (!std::isfinite(X1Y1) || !std::isfinite(X1Y2) || !std::isfinite(X2Y1) || !std::isfinite(X2Y2))
        {
            //intensVals[ptId] = NPP_MAXABS_32F;
            //outPtr[npts] = std::numeric_limits<_Tp>::max();
            outPtr[npts * stepout] = std::numeric_limits<float>::max();
            outPtr[npts * stepout + 1] = std::numeric_limits<float>::max();
            outPtr[npts * stepout + 2] = std::numeric_limits<float>::max();
            continue;
        }

        if (colwise)
        {
            DY1 = fIdxPos - fIdx1;
            DY2 = fIdx2 - fIdxPos;
            DX1 = sIdxPos - sIdx1;
            DX2 = sIdx2 - sIdxPos;
        }
        else
        {
            DX1 = fIdxPos - fIdx1;
            DX2 = fIdx2 - fIdxPos;
            DY1 = sIdxPos - sIdx1;
            DY2 = sIdx2 - sIdxPos;
        }

        outPtr[npts * stepout] = (X1Y1 * DX2 * DY2 + X2Y1 * DX1 * DY2 + X1Y2 * DX2 * DY1 + X2Y2 * DX1 * DY1);
        outPtr[npts * stepout + 1] = (X2Y1 - X1Y1) + ((X2Y2 + X1Y1) - (X2Y1 + X1Y2)) * DY1;
        outPtr[npts * stepout + 2] = (X1Y2 - X1Y1) + ((X2Y2 + X1Y1) - (X2Y1 + X1Y2)) * DX1;
    }
#if (USEOMP)
    }
#endif

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void clearFilterCache(void)
{
    if (AMatBiCu.data)
    {
        free(AMatBiCu.data);
        AMatBiCu.data = NULL;
        AMatBiCu.sizex = 0;
        AMatBiCu.sizey = 0;
    }
    if (AMatBiQu.data)
    {
        free(AMatBiQu.data);
        AMatBiQu.data = NULL;
        AMatBiQu.sizex = 0;
        AMatBiQu.sizey = 0;
    }
    if (AMatBiHe.data)
    {
        free(AMatBiHe.data);
        AMatBiHe.data = NULL;
        AMatBiHe.sizex = 0;
        AMatBiHe.sizey = 0;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doCalcAMat(const float *img, const int sizex, const int sizey, const int imgStep, const int interpType, const int flags)
{
    ito::float32 kernel[9] = { -3.0 / 32.0, 0.0,  3.0 / 32.0,
        -10.0 / 32.0, 0.0, 10.0 / 32.0,
        -3.0 / 32.0, 0.0,  3.0 / 32.0 };
    cv::Mat kernelDx = cv::Mat(cv::Size(3, 3), CV_32F, kernel);
    cv::Mat kernelDy = kernelDx.t();

    //ito::float32 kernel[3] = { -0.5, 0.0, 0.5 };
    //cv::Mat kernelDx = cv::Mat(cv::Size(3, 1), CV_32F, kernel);
    //cv::Mat kernelDy = cv::Mat(cv::Size(1, 3), CV_32F, kernel);
    cv::Mat matDx, matDy, matDxy;

	int nimgStep = imgStep;
    cv::Mat imgMat;
    // adapt Matlab type columnwise matrix first
    if (flags & 512)
    {
        imgMat = cv::Mat(sizex, sizey, CV_32F, (void*)img);
        imgMat = imgMat.t();
		nimgStep = sizex;
    }
    else
    {
        imgMat = cv::Mat(sizey, sizex, CV_32F, (void*)img);
    }

    cv::filter2D(imgMat, matDx, -1, kernelDx, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
    cv::filter2D(imgMat, matDy, -1, kernelDy, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
    ito::float32 *dxp = (ito::float32*)matDx.data;
    ito::float32 *dyp = (ito::float32*)matDy.data;
	// this is necessary when we do the transposing for use in Matlab
	ito::float32 *imgp = (ito::float32*)imgMat.data;

    // border correction
    for (int y = 0; y < sizey; y++)
    {
        dxp[y * sizex] = dxp[y * sizex + 1];
        dxp[y * sizex + sizex - 1] = dxp[y * sizex + sizex - 2];
        dyp[y * sizex] = dyp[y * sizex + 1];
        dyp[y * sizex + sizex - 1] = dyp[y * sizex + sizex - 2];
    }
    for (int x = 0; x < sizex; x++)
    {
        dxp[x] = dxp[sizex + x];
        dxp[(sizey - 1) * sizex + x] = dxp[(sizey - 2) * sizex + x];
        dyp[x] = dyp[sizex + x];
        dyp[(sizey - 1) * sizex + x] = dyp[(sizey - 2) * sizex + x];
    }

    cv::filter2D(matDx, matDxy, -1, kernelDy, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
    // border correction
    ito::float32 *dxyp = (ito::float32*)matDxy.data;
    for (int x = 0; x < sizex; x++)
    {
        dxyp[x] = dxyp[sizex + x];
        dxyp[(sizey - 1) * sizex + x] = dxyp[(sizey - 2) * sizex + x];
    }
    for (int y = 0; y < sizey; y++)
    {
        dxyp[y * sizex] = dxyp[y * sizex + 1];
        dxyp[y * sizex + sizex - 1] = dxyp[y * sizex + sizex - 2];
    }

    switch (interpType)
    {
        // bicubic
        case 1:
        {
            if (AMatBiCu.data)
            {
                free(AMatBiCu.data);
            }
            AMatBiCu.data = (float*)malloc(16 * sizex * sizey * sizeof(float));
            AMatBiCu.sizex = sizex;
            AMatBiCu.sizey = sizey;
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
        {
#pragma omp for schedule(guided)
#endif
            for (int y = 0; y < sizey - 1; y++)
            {
                for (int x = 0; x < sizex - 1; x++)
                {
                    AMatBiCu.data[16 * (y * sizex + x)] = BMatBiCu0	* imgp[y	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 1] = BMatBiCu20	* dxp[y	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 2] = BMatBiCu32	* imgp[y	* nimgStep + x]
                        + BMatBiCu33	* imgp[y	* nimgStep + x + 1]
                        + BMatBiCu36	* dxp[y	* nimgStep + x]
                        + BMatBiCu37	* dxp[y	* nimgStep + x + 1];
                    AMatBiCu.data[16 * (y * sizex + x) + 3] = BMatBiCu48	* imgp[y	* nimgStep + x]
                        + BMatBiCu49	* imgp[y	* nimgStep + x + 1]
                        + BMatBiCu52	* dxp[y	* nimgStep + x]
                        + BMatBiCu53	* dxp[y	* nimgStep + x + 1];
                    AMatBiCu.data[16 * (y * sizex + x) + 4] = BMatBiCu72	* dyp[y	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 5] = BMatBiCu92	* dxyp[y	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 6] = BMatBiCu104	* dyp[y	* nimgStep + x]
                        + BMatBiCu105	* dyp[y	* nimgStep + x + 1]
                        + BMatBiCu108	* dxyp[y	* nimgStep + x]
                        + BMatBiCu109	* dxyp[y	* nimgStep + x + 1];
                    AMatBiCu.data[16 * (y * sizex + x) + 7] = BMatBiCu120	* dyp[y	* nimgStep + x]
                        + BMatBiCu121	* dyp[y	* nimgStep + x + 1]
                        + BMatBiCu124	* dxyp[y	* nimgStep + x]
                        + BMatBiCu125	* dxyp[y	* nimgStep + x + 1];
                    AMatBiCu.data[16 * (y * sizex + x) + 8] = BMatBiCu128	* imgp[y	* nimgStep + x]
                        + BMatBiCu130	* imgp[(y + 1)	* nimgStep + x]
                        + BMatBiCu136	* dyp[y	* nimgStep + x]
                        + BMatBiCu138	* dyp[(y + 1)	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 9] = BMatBiCu148	* dxp[y	* nimgStep + x]
                        + BMatBiCu150	* dxp[(y + 1)	* nimgStep + x]
                        + BMatBiCu156	* dxyp[y	* nimgStep + x]
                        + BMatBiCu158	* dxyp[(y + 1)	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 10] = BMatBiCu160	* imgp[y	* nimgStep + x]
                        + BMatBiCu161	* imgp[y	* nimgStep + x + 1]
                        + BMatBiCu162	* imgp[(y + 1)	* nimgStep + x]
                        + BMatBiCu163	* imgp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu164	* dxp[y	* nimgStep + x]
                        + BMatBiCu165	* dxp[y	* nimgStep + x + 1]
                        + BMatBiCu166	* dxp[(y + 1)	* nimgStep + x]
                        + BMatBiCu167	* dxp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu168	* dyp[y	* nimgStep + x]
                        + BMatBiCu169	* dyp[y	* nimgStep + x + 1]
                        + BMatBiCu170	* dyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu171	* dyp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu172	* dxyp[y	* nimgStep + x]
                        + BMatBiCu173	* dxyp[y	* nimgStep + x + 1]
                        + BMatBiCu174	* dxyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu175	* dxyp[(y + 1)	* nimgStep + x + 1];
                    AMatBiCu.data[16 * (y * sizex + x) + 11] = BMatBiCu176	* imgp[y	* nimgStep + x]
                        + BMatBiCu177	* imgp[y	* nimgStep + x + 1]
                        + BMatBiCu178	* imgp[(y + 1)	* nimgStep + x]
                        + BMatBiCu179	* imgp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu180	* dxp[y	* nimgStep + x]
                        + BMatBiCu181	* dxp[y	* nimgStep + x + 1]
                        + BMatBiCu182	* dxp[(y + 1)	* nimgStep + x]
                        + BMatBiCu183	* dxp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu184	* dyp[y	* nimgStep + x]
                        + BMatBiCu185	* dyp[y	* nimgStep + x + 1]
                        + BMatBiCu186	* dyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu187	* dyp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu188	* dxyp[y	* nimgStep + x]
                        + BMatBiCu189	* dxyp[y	* nimgStep + x + 1]
                        + BMatBiCu190	* dxyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu191	* dxyp[(y + 1)	* nimgStep + x + 1];
                    AMatBiCu.data[16 * (y * sizex + x) + 12] = BMatBiCu192	* imgp[y	* nimgStep + x]
                        + BMatBiCu194	* imgp[(y + 1)	* nimgStep + x]
                        + BMatBiCu200	* dyp[y	* nimgStep + x]
                        + BMatBiCu202	* dyp[(y + 1)	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 13] = BMatBiCu212	* dxp[y	* nimgStep + x]
                        + BMatBiCu214	* dxp[(y + 1)	* nimgStep + x]
                        + BMatBiCu220	* dxyp[y	* nimgStep + x]
                        + BMatBiCu222	* dxyp[(y + 1)	* nimgStep + x];
                    AMatBiCu.data[16 * (y * sizex + x) + 14] = BMatBiCu224	* imgp[y	* nimgStep + x]
                        + BMatBiCu225	* imgp[y	* nimgStep + x + 1]
                        + BMatBiCu226	* imgp[(y + 1)	* nimgStep + x]
                        + BMatBiCu227	* imgp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu228	* dxp[y	* nimgStep + x]
                        + BMatBiCu229	* dxp[y	* nimgStep + x + 1]
                        + BMatBiCu230	* dxp[(y + 1)	* nimgStep + x]
                        + BMatBiCu231	* dxp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu232	* dyp[y	* nimgStep + x]
                        + BMatBiCu233	* dyp[y	* nimgStep + x + 1]
                        + BMatBiCu234	* dyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu235	* dyp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu236	* dxyp[y	* nimgStep + x]
                        + BMatBiCu237	* dxyp[y	* nimgStep + x + 1]
                        + BMatBiCu238	* dxyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu239	* dxyp[(y + 1)	* nimgStep + x + 1];
                    AMatBiCu.data[16 * (y * sizex + x) + 15] = BMatBiCu240	* imgp[y	* nimgStep + x]
                        + BMatBiCu241	* imgp[y	* nimgStep + x + 1]
                        + BMatBiCu242	* imgp[(y + 1)	* nimgStep + x]
                        + BMatBiCu243	* imgp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu244	* dxp[y	* nimgStep + x]
                        + BMatBiCu245	* dxp[y	* nimgStep + x + 1]
                        + BMatBiCu246	* dxp[(y + 1)	* nimgStep + x]
                        + BMatBiCu247	* dxp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu248	* dyp[y	* nimgStep + x]
                        + BMatBiCu249	* dyp[y	* nimgStep + x + 1]
                        + BMatBiCu250	* dyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu251	* dyp[(y + 1)	* nimgStep + x + 1]
                        + BMatBiCu252	* dxyp[y	* nimgStep + x]
                        + BMatBiCu253	* dxyp[y	* nimgStep + x + 1]
                        + BMatBiCu254	* dxyp[(y + 1)	* nimgStep + x]
                        + BMatBiCu255	* dxyp[(y + 1)	* nimgStep + x + 1];
                }
            }
#if (USEOMP)
        }
#endif
        }
        break;

        // biquintic
        case 2:
        {
            if (AMatBiQu.data)
            {
                free(AMatBiQu.data);
            }
            AMatBiQu.data = (float*)malloc(36 * sizex * sizey * sizeof(float));
            AMatBiQu.sizex = sizex;
            AMatBiQu.sizey = sizey;
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
        {
#pragma omp for schedule(guided)
#endif
            for (int y = 1; y < sizey - 1; y++)
            {
                for (int x = 1; x < sizex - 1; x++)
                {
                    for (int v = 0; v < 36; v++)
                    {
                        AMatBiQu.data[36 * (y * (sizex - 1) + x) + v] = BMatBiQu[v * 36] * imgp[(y - 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 1] * imgp[(y - 1) * nimgStep + x] + BMatBiQu[v * 36 + 2] * imgp[(y - 1) * nimgStep + x + 1]
                            + BMatBiQu[v * 36 + 3] * imgp[y * nimgStep + x - 1] + BMatBiQu[v * 36 + 4] * imgp[y * sizex + x] + BMatBiQu[v * 36 + 5] * imgp[y * sizex + x + 1]
                            + BMatBiQu[v * 36 + 6] * imgp[(y + 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 7] * imgp[(y + 1) * sizex + x] + BMatBiQu[v * 36 + 8] * imgp[(y + 1) * sizex + x + 1]

                            + BMatBiQu[v * 36 + 9] * dxp[(y - 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 10] * dxp[(y - 1) * nimgStep + x] + BMatBiQu[v * 36 + 11] * dxp[(y - 1) * nimgStep + x + 1]
                            + BMatBiQu[v * 36 + 12] * dxp[y * nimgStep + x - 1] + BMatBiQu[v * 36 + 13] * dxp[y * sizex + x] + BMatBiQu[v * 36 + 14] * dxp[y * sizex + x + 1]
                            + BMatBiQu[v * 36 + 15] * dxp[(y + 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 16] * dxp[(y + 1) * sizex + x] + BMatBiQu[v * 36 + 17] * dxp[(y + 1) * sizex + x + 1]

                            + BMatBiQu[v * 36 + 18] * dyp[(y - 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 19] * dyp[(y - 1) * nimgStep + x] + BMatBiQu[v * 36 + 20] * dyp[(y - 1) * nimgStep + x + 1]
                            + BMatBiQu[v * 36 + 21] * dyp[y * nimgStep + x - 1] + BMatBiQu[v * 36 + 22] * dyp[y * sizex + x] + BMatBiQu[v * 36 + 23] * dyp[y * sizex + x + 1]
                            + BMatBiQu[v * 36 + 24] * dyp[(y + 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 25] * dyp[(y + 1) * sizex + x] + BMatBiQu[v * 36 + 26] * dyp[(y + 1) * sizex + x + 1]

                            + BMatBiQu[v * 36 + 27] * dxyp[(y - 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 28] * dxyp[(y - 1) * nimgStep + x] + BMatBiQu[v * 36 + 29] * dxyp[(y - 1) * nimgStep + x + 1]
                            + BMatBiQu[v * 36 + 30] * dxyp[y * nimgStep + x - 1] + BMatBiQu[v * 36 + 31] * dxyp[y * sizex + x] + BMatBiQu[v * 36 + 32] * dxyp[y * sizex + x + 1]
                            + BMatBiQu[v * 36 + 33] * dxyp[(y + 1) * nimgStep + x - 1] + BMatBiQu[v * 36 + 34] * dxyp[(y + 1) * sizex + x] + BMatBiQu[v * 36 + 35] * dxyp[(y + 1) * sizex + x + 1];
                    }
                }
            }
#if (USEOMP)
        }
#endif
        }
        break;

        // biheptic
        case 3:
        {
            if (AMatBiHe.data)
            {
                free(AMatBiHe.data);
            }
            AMatBiHe.data = (float*)malloc(64 * sizex * sizey * sizeof(float));
            AMatBiHe.sizex = sizex;
            AMatBiHe.sizey = sizey;
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
        {
#pragma omp for schedule(guided)
#endif
            for (int y = 1; y < sizey - 2; y++)
            {
                for (int x = 1; x < sizex - 2; x++)
                {
                    for (int v = 0; v < 64; v++)
                    {
                        AMatBiHe.data[64 * (y * (sizex - 2) + x) + v] = BMatBiHe[v * 64] * imgp[(y - 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 1] * imgp[(y - 1) * nimgStep + x] + BMatBiHe[v * 64 + 2] * imgp[(y - 1) * nimgStep + x + 1] + BMatBiHe[v * 64 + 3] * imgp[(y - 1) * nimgStep + x + 2]
                            + BMatBiHe[v * 64 + 4] * imgp[y * nimgStep + x - 1] + BMatBiHe[v * 64 + 5] * imgp[y * sizex + x] + BMatBiHe[v * 64 + 6] * imgp[y * sizex + x + 1] + BMatBiHe[v * 64 + 7] * imgp[y * sizex + x + 2]
                            + BMatBiHe[v * 64 + 8] * imgp[(y + 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 9] * imgp[(y + 1) * sizex + x] + BMatBiHe[v * 64 + 10] * imgp[(y + 1) * sizex + x + 1] + BMatBiHe[v * 64 + 11] * imgp[(y + 1) * sizex + x + 2]
                            + BMatBiHe[v * 64 + 12] * imgp[(y + 2) * nimgStep + x - 1] + BMatBiHe[v * 64 + 13] * imgp[(y + 2) * sizex + x] + BMatBiHe[v * 64 + 14] * imgp[(y + 2) * sizex + x + 1] + BMatBiHe[v * 64 + 15] * imgp[(y + 2) * sizex + x + 2]

                            + BMatBiHe[v * 64 + 16] * dxp[(y - 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 17] * dxp[(y - 1) * nimgStep + x] + BMatBiHe[v * 64 + 18] * dxp[(y - 1) * nimgStep + x + 1] + BMatBiHe[v * 64 + 19] * dxp[(y - 1) * nimgStep + x + 2]
                            + BMatBiHe[v * 64 + 20] * dxp[y * nimgStep + x - 1] + BMatBiHe[v * 64 + 21] * dxp[y * sizex + x] + BMatBiHe[v * 64 + 22] * dxp[y * sizex + x + 1] + BMatBiHe[v * 64 + 23] * dxp[y * sizex + x + 2]
                            + BMatBiHe[v * 64 + 24] * dxp[(y + 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 25] * dxp[(y + 1) * sizex + x] + BMatBiHe[v * 64 + 26] * dxp[(y + 1) * sizex + x + 1] + BMatBiHe[v * 64 + 27] * dxp[(y + 1) * sizex + x + 2]
                            + BMatBiHe[v * 64 + 28] * dxp[(y + 2) * nimgStep + x - 1] + BMatBiHe[v * 64 + 29] * dxp[(y + 2) * sizex + x] + BMatBiHe[v * 64 + 30] * dxp[(y + 2) * sizex + x + 1] + BMatBiHe[v * 64 + 31] * dxp[(y + 2) * sizex + x + 2]

                            + BMatBiHe[v * 64 + 32] * dyp[(y - 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 33] * dyp[(y - 1) * nimgStep + x] + BMatBiHe[v * 64 + 34] * dyp[(y - 1) * nimgStep + x + 1] + BMatBiHe[v * 64 + 35] * dyp[(y - 1) * nimgStep + x + 2]
                            + BMatBiHe[v * 64 + 66] * dyp[y * nimgStep + x - 1] + BMatBiHe[v * 64 + 37] * dyp[y * sizex + x] + BMatBiHe[v * 64 + 38] * dyp[y * sizex + x + 1] + BMatBiHe[v * 64 + 39] * dyp[y * sizex + x + 2]
                            + BMatBiHe[v * 64 + 40] * dyp[(y + 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 41] * dyp[(y + 1) * sizex + x] + BMatBiHe[v * 64 + 42] * dyp[(y + 1) * sizex + x + 1] + BMatBiHe[v * 64 + 43] * dyp[(y + 1) * sizex + x + 2]
                            + BMatBiHe[v * 64 + 44] * dyp[(y + 2) * nimgStep + x - 1] + BMatBiHe[v * 64 + 45] * dyp[(y + 2) * sizex + x] + BMatBiHe[v * 64 + 46] * dyp[(y + 2) * sizex + x + 1] + BMatBiHe[v * 64 + 47] * dyp[(y + 2) * sizex + x + 2]

                            + BMatBiHe[v * 64 + 48] * dxyp[(y - 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 49] * dxyp[(y - 1) * nimgStep + x] + BMatBiHe[v * 64 + 50] * dxyp[(y - 1) * nimgStep + x + 1] + BMatBiHe[v * 64 + 51] * dxyp[(y - 1) * nimgStep + x + 2]
                            + BMatBiHe[v * 64 + 52] * dxyp[y * nimgStep + x - 1] + BMatBiHe[v * 64 + 53] * dxyp[y * sizex + x] + BMatBiHe[v * 64 + 54] * dxyp[y * sizex + x + 1] + BMatBiHe[v * 64 + 55] * dxyp[y * sizex + x + 2]
                            + BMatBiHe[v * 64 + 56] * dxyp[(y + 1) * nimgStep + x - 1] + BMatBiHe[v * 64 + 57] * dxyp[(y + 1) * sizex + x] + BMatBiHe[v * 64 + 58] * dxyp[(y + 1) * sizex + x + 1] + BMatBiHe[v * 64 + 59] * dxyp[(y + 1) * sizex + x + 2]
                            + BMatBiHe[v * 64 + 60] * dxyp[(y + 2) * nimgStep + x - 1] + BMatBiHe[v * 64 + 61] * dxyp[(y + 2) * sizex + x] + BMatBiHe[v * 64 + 62] * dxyp[(y + 2) * sizex + x + 1] + BMatBiHe[v * 64 + 63] * dxyp[(y + 2) * sizex + x + 2];
                    }
                }
            }
#if (USEOMP)
        }
#endif
        }
        break;
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doInterpolateCubicPts(const float *img, const int sizex, const int sizey, const int imgStep, const float *positions, const int numPts, float *res, const int flags = 0)
{
    // Here we have to do the real job
    // we loop over all points, using openmp to speed up a little bit

    ito::float32 kernel[9] = { -3.0 / 32.0, 0.0,  3.0 / 32.0,
        -10.0 / 32.0, 0.0, 10.0 / 32.0,
        -3.0 / 32.0, 0.0,  3.0 / 32.0 };
    cv::Mat kernelDx = cv::Mat(cv::Size(3, 3), CV_32F, kernel);
    cv::Mat kernelDy = kernelDx.t();

    //ito::float32 kernel[3] = { -0.5, 0.0, 0.5 };
    //cv::Mat kernelDx = cv::Mat(cv::Size(3, 1), CV_32F, kernel);
    //cv::Mat kernelDy = cv::Mat(cv::Size(1, 3), CV_32F, kernel);
    cv::Mat matDx, matDy, matDxy;

    cv::Mat imgMat(sizey, sizex, CV_32F, (void*)img);

    cv::filter2D(imgMat, matDx, -1, kernelDx, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
    cv::filter2D(imgMat, matDy, -1, kernelDy, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
    ito::float32 *dxp = (ito::float32*)matDx.data;
    ito::float32 *dyp = (ito::float32*)matDy.data;

    // border correction
    for (int y = 0; y < sizey; y++)
    {
        dxp[y * sizex] = dxp[y * sizex + 1];
        dxp[y * sizex + sizex - 1] = dxp[y * sizex + sizex - 2];
        dyp[y * sizex] = dyp[y * sizex + 1];
        dyp[y * sizex + sizex - 1] = dyp[y * sizex + sizex - 2];
    }
    for (int x = 0; x < sizex; x++)
    {
        dxp[x] = dxp[sizex + x];
        dxp[(sizey - 1) * sizex + x] = dxp[(sizey - 2) * sizex + x];
        dyp[x] = dyp[sizex + x];
        dyp[(sizey - 1) * sizex + x] = dyp[(sizey - 2) * sizex + x];
    }

    cv::filter2D(matDx, matDxy, -1, kernelDy, cv::Point(-1, -1), 0.0, cv::BORDER_ISOLATED);
    // border correction
    ito::float32 *dxyp = (ito::float32*)matDxy.data;
    for (int x = 0; x < sizex; x++)
    {
        dxyp[x] = dxyp[sizex + x];
        dxyp[(sizey - 1) * sizex + x] = dxyp[(sizey - 2) * sizex + x];
    }
    for (int y = 0; y < sizey; y++)
    {
        dxyp[y * sizex] = dxyp[y * sizex + 1];
        dxyp[y * sizex + sizex - 1] = dxyp[y * sizex + sizex - 2];
    }

#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
        int row1, col1;
        float rowPos, colPos;
        float coeffVec[16];
#if (USEOMP)
#pragma omp for schedule(guided)
#endif
        for (int npts = 0; npts < numPts; npts++)
        {
            colPos = positions[npts * 2];
            rowPos = positions[npts * 2 + 1];

            if ((rowPos >= sizey) || (colPos >= sizex) || (rowPos < 1) || (colPos < 1) || !std::isfinite(rowPos) || !std::isfinite(colPos))
                continue;

            if (rowPos < (sizey - 1))
            {
                row1 = (int)floor(rowPos);
            }
            else
            {
                row1 = (int)floor(rowPos - 1);
            }
            if (colPos < (sizex - 1))
            {
                col1 = (int)floor(colPos);
            }
            else
            {
                col1 = (int)floor(colPos - 1);
            }

            float h = rowPos - row1;
            float t = colPos - col1;
            float t2 = t * t, t3 = t * t * t;
            float h2 = h * h, h3 = h * h * h;

            // calculate interpolated intensity by multiplying ht with A
            // ht = [1     t        t^2        t^3 ...
            //       h     h * t    h*t^2      h * t^3 ...
            //       h^2   h^2 * t  h^2 * t^2  h^2 * t^3 ...
            //       h^3   h^3 * t  h^3 * t^2  h^3 * t^3];

            // wikipedia version
            coeffVec[0] = BMatBiCu0	* img[row1	* imgStep + col1];
            coeffVec[1] = BMatBiCu20	* dxp[row1	* imgStep + col1];
            coeffVec[2] = BMatBiCu32	* img[row1	* imgStep + col1]
                + BMatBiCu33	* img[row1	* imgStep + col1 + 1]
                + BMatBiCu36	* dxp[row1	* imgStep + col1]
                + BMatBiCu37	* dxp[row1	* imgStep + col1 + 1];
            coeffVec[3] = BMatBiCu48	* img[row1	* imgStep + col1]
                + BMatBiCu49	* img[row1	* imgStep + col1 + 1]
                + BMatBiCu52	* dxp[row1	* imgStep + col1]
                + BMatBiCu53	* dxp[row1	* imgStep + col1 + 1];
            coeffVec[4] = BMatBiCu72	* dyp[row1	* imgStep + col1];
            coeffVec[5] = BMatBiCu92	* dxyp[row1	* imgStep + col1];
            coeffVec[6] = BMatBiCu104	* dyp[row1	* imgStep + col1]
                + BMatBiCu105	* dyp[row1	* imgStep + col1 + 1]
                + BMatBiCu108	* dxyp[row1	* imgStep + col1]
                + BMatBiCu109	* dxyp[row1	* imgStep + col1 + 1];
            coeffVec[7] = BMatBiCu120	* dyp[row1	* imgStep + col1]
                + BMatBiCu121	* dyp[row1	* imgStep + col1 + 1]
                + BMatBiCu124	* dxyp[row1	* imgStep + col1]
                + BMatBiCu125	* dxyp[row1	* imgStep + col1 + 1];
            coeffVec[8] = BMatBiCu128	* img[row1	* imgStep + col1]
                + BMatBiCu130	* img[(row1 + 1)	* imgStep + col1]
                + BMatBiCu136	* dyp[row1	* imgStep + col1]
                + BMatBiCu138	* dyp[(row1 + 1)	* imgStep + col1];
            coeffVec[9] = BMatBiCu148	* dxp[row1	* imgStep + col1]
                + BMatBiCu150	* dxp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu156	* dxyp[row1	* imgStep + col1]
                + BMatBiCu158	* dxyp[(row1 + 1)	* imgStep + col1];
            coeffVec[10] = BMatBiCu160	* img[row1	* imgStep + col1]
                + BMatBiCu161	* img[row1	* imgStep + col1 + 1]
                + BMatBiCu162	* img[(row1 + 1)	* imgStep + col1]
                + BMatBiCu163	* img[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu164	* dxp[row1	* imgStep + col1]
                + BMatBiCu165	* dxp[row1	* imgStep + col1 + 1]
                + BMatBiCu166	* dxp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu167	* dxp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu168	* dyp[row1	* imgStep + col1]
                + BMatBiCu169	* dyp[row1	* imgStep + col1 + 1]
                + BMatBiCu170	* dyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu171	* dyp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu172	* dxyp[row1	* imgStep + col1]
                + BMatBiCu173	* dxyp[row1	* imgStep + col1 + 1]
                + BMatBiCu174	* dxyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu175	* dxyp[(row1 + 1)	* imgStep + col1 + 1];
            coeffVec[11] = BMatBiCu176	* img[row1	* imgStep + col1]
                + BMatBiCu177	* img[row1	* imgStep + col1 + 1]
                + BMatBiCu178	* img[(row1 + 1)	* imgStep + col1]
                + BMatBiCu179	* img[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu180	* dxp[row1	* imgStep + col1]
                + BMatBiCu181	* dxp[row1	* imgStep + col1 + 1]
                + BMatBiCu182	* dxp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu183	* dxp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu184	* dyp[row1	* imgStep + col1]
                + BMatBiCu185	* dyp[row1	* imgStep + col1 + 1]
                + BMatBiCu186	* dyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu187	* dyp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu188	* dxyp[row1	* imgStep + col1]
                + BMatBiCu189	* dxyp[row1	* imgStep + col1 + 1]
                + BMatBiCu190	* dxyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu191	* dxyp[(row1 + 1)	* imgStep + col1 + 1];
            coeffVec[12] = BMatBiCu192	* img[row1	* imgStep + col1]
                + BMatBiCu194	* img[(row1 + 1)	* imgStep + col1]
                + BMatBiCu200	* dyp[row1	* imgStep + col1]
                + BMatBiCu202	* dyp[(row1 + 1)	* imgStep + col1];
            coeffVec[13] = BMatBiCu212	* dxp[row1	* imgStep + col1]
                + BMatBiCu214	* dxp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu220	* dxyp[row1	* imgStep + col1]
                + BMatBiCu222	* dxyp[(row1 + 1)	* imgStep + col1];
            coeffVec[14] = BMatBiCu224	* img[row1	* imgStep + col1]
                + BMatBiCu225	* img[row1	* imgStep + col1 + 1]
                + BMatBiCu226	* img[(row1 + 1)	* imgStep + col1]
                + BMatBiCu227	* img[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu228	* dxp[row1	* imgStep + col1]
                + BMatBiCu229	* dxp[row1	* imgStep + col1 + 1]
                + BMatBiCu230	* dxp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu231	* dxp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu232	* dyp[row1	* imgStep + col1]
                + BMatBiCu233	* dyp[row1	* imgStep + col1 + 1]
                + BMatBiCu234	* dyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu235	* dyp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu236	* dxyp[row1	* imgStep + col1]
                + BMatBiCu237	* dxyp[row1	* imgStep + col1 + 1]
                + BMatBiCu238	* dxyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu239	* dxyp[(row1 + 1)	* imgStep + col1 + 1];
            coeffVec[15] = BMatBiCu240	* img[row1	* imgStep + col1]
                + BMatBiCu241	* img[row1	* imgStep + col1 + 1]
                + BMatBiCu242	* img[(row1 + 1)	* imgStep + col1]
                + BMatBiCu243	* img[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu244	* dxp[row1	* imgStep + col1]
                + BMatBiCu245	* dxp[row1	* imgStep + col1 + 1]
                + BMatBiCu246	* dxp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu247	* dxp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu248	* dyp[row1	* imgStep + col1]
                + BMatBiCu249	* dyp[row1	* imgStep + col1 + 1]
                + BMatBiCu250	* dyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu251	* dyp[(row1 + 1)	* imgStep + col1 + 1]
                + BMatBiCu252	* dxyp[row1	* imgStep + col1]
                + BMatBiCu253	* dxyp[row1	* imgStep + col1 + 1]
                + BMatBiCu254	* dxyp[(row1 + 1)	* imgStep + col1]
                + BMatBiCu255	* dxyp[(row1 + 1)	* imgStep + col1 + 1];

            res[npts * 3] = coeffVec[0] + coeffVec[1] * t + coeffVec[2] * t2 + coeffVec[3] * t3
                + coeffVec[4] * h + coeffVec[5] * h * t + coeffVec[6] * h * t2 + coeffVec[7] * h * t3
                + coeffVec[8] * h2 + coeffVec[9] * h2 * t + coeffVec[10] * h2 * t2 + coeffVec[11] * h2 * t3
                + coeffVec[12] * h3 + coeffVec[13] * h3 * t + coeffVec[14] * h3 * t2 + coeffVec[15] * h3 * t3;

            if (flags & 1)
            {
                // calculate interpolated derivative(s) dhdx(t)
                // dhdxt = a(4) + a(5) * t + a(6) * t^2 + a(7) * t^3 ...
                //        + 2 * a(8) * h + 2 * a(9) * h * t + 2 * a(10) * h * t^2 + 2 * a(11) * h * t^3 ...
                //        + 3 * a(12) * h^2 + 3 * a(13) * h^2 * t + 3 * a(14) * h^2 * t^2 + 3 * a(15) * h^2 * t^3;
                res[npts * 3 + 1] = coeffVec[1] + coeffVec[2] * 2.0F * t + coeffVec[3] * 3.0F * t2 + coeffVec[5] * h
                    + coeffVec[6] * 2.0F * h * t + coeffVec[7] * 3.0F * h * t2 + coeffVec[9] * h2 + coeffVec[10] * 2.0F * h2 * t
                    + coeffVec[11] * 3.0F * h2 * t2 + coeffVec[13] * h3 + coeffVec[14] * 2.0F * h3 * t + coeffVec[15] * 3.0F * h3 * t2;

                // calculate interpolated derivative(s) dhdy(t)
                // dhdyt = a(1) + 2 * a(2) * t + 3 * a(3) * t^2 + ...
                //    a(5) * h + 2 * a(6) * h * t + 3 * a(7) * h * t^2 + ...
                //    a(9) * h^2 + 2 * a(10) * h^2 * t + 3 * a(11) * h^2 * t^2 + ...
                //    a(13) * h^3 + 2 * a(14)v* h^3 * t + 3 * a(15) * h^3 * t^2;
                res[npts * 3 + 2] = coeffVec[4] + coeffVec[5] * t + coeffVec[6] * t2 + coeffVec[7] * t3
                    + coeffVec[8] * 2.0F * h + coeffVec[9] * 2.0F * h * t + coeffVec[10] * 2.0F * h * t2 + coeffVec[11] * 2.0F * h * t3
                    + coeffVec[12] * 3.0F * h2 + coeffVec[13] * 3.0F * h2 * t + coeffVec[14] * 3.0F * h2 * t2 + coeffVec[15] * 3.0F * h2 * t3;
            }
        }
#if (USEOMP)
    }
#endif

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doInterpolateCubic(const float *positions, const int numPts, float *res, const int flags = 0)
{
    // Here we have to do the real job
    // we loop over all points, using openmp to speed up a little bit

    float *AMat = AMatBiCu.data;
    int sizex = AMatBiCu.sizex;
    int sizey = AMatBiCu.sizey;
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
        int row1, col1;
        float rowPos, colPos;
#if (USEOMP)
#pragma omp for schedule(guided)
#endif
        for (int npts = 0; npts < numPts; npts++)
        {
            colPos = positions[npts * 2];
            rowPos = positions[npts * 2 + 1];
            if ((rowPos >= sizey) || (colPos >= sizex) || (rowPos < 1) || (colPos < 1) || !std::isfinite(rowPos) || !std::isfinite(colPos))
                continue;

            if (rowPos < (sizey - 1))
            {
                row1 = (int)floor(rowPos);
            }
            else
            {
                row1 = (int)floor(rowPos - 1);
            }
            if (colPos < (sizex - 1))
            {
                col1 = (int)floor(colPos);
            }
            else
            {
                col1 = (int)floor(colPos - 1);
            }

            float h = rowPos - row1;
            float t = colPos - col1;
            float t2 = t * t, t3 = t * t * t;
            float h2 = h * h, h3 = h * h * h;

            // calculate interpolated intensity by multiplying ht with A
            // ht = [1     t        t^2        t^3 ...
            //       h     h * t    h*t^2      h * t^3 ...
            //       h^2   h^2 * t  h^2 * t^2  h^2 * t^3 ...
            //       h^3   h^3 * t  h^3 * t^2  h^3 * t^3];

            // wikipedia version
            res[npts * 3] = AMat[16 * (row1 * sizex + col1)] + AMat[16 * (row1 * sizex + col1) + 1] * t
                + AMat[16 * (row1 * sizex + col1) + 2] * t2       + AMat[16 * (row1 * sizex + col1) + 3] * t3
                + AMat[16 * (row1 * sizex + col1) + 4] * h        + AMat[16 * (row1 * sizex + col1) + 5] * h * t
                + AMat[16 * (row1 * sizex + col1) + 6] * h * t2   + AMat[16 * (row1 * sizex + col1) + 7] * h * t3
                + AMat[16 * (row1 * sizex + col1) + 8] * h2       + AMat[16 * (row1 * sizex + col1) + 9] * h2 * t
                + AMat[16 * (row1 * sizex + col1) + 10] * h2 * t2 + AMat[16 * (row1 * sizex + col1) + 11] * h2 * t3
                + AMat[16 * (row1 * sizex + col1) + 12] * h3      + AMat[16 * (row1 * sizex + col1) + 13] * h3 * t
                + AMat[16 * (row1 * sizex + col1) + 14] * h3 * t2 + AMat[16 * (row1 * sizex + col1) + 15] * h3 * t3;

            if (flags & 1)
            {
                // calculate interpolated derivative(s) dhdx(t)
                // dhdxt = a(4) + a(5) * t + a(6) * t^2 + a(7) * t^3 ...
                //        + 2 * a(8) * h + 2 * a(9) * h * t + 2 * a(10) * h * t^2 + 2 * a(11) * h * t^3 ...
                //        + 3 * a(12) * h^2 + 3 * a(13) * h^2 * t + 3 * a(14) * h^2 * t^2 + 3 * a(15) * h^2 * t^3;
                res[npts * 3 + 1] = AMat[16 * (row1 * sizex + col1) + 1] + AMat[16 * (row1 * sizex + col1) + 2] * 2.0F * t
                    + AMat[16 * (row1 * sizex + col1) + 3] * 3.0F * t2       + AMat[16 * (row1 * sizex + col1) + 5] * h
                    + AMat[16 * (row1 * sizex + col1) + 6] * 2.0F * h * t    + AMat[16 * (row1 * sizex + col1) + 7] * 3.0F * h * t2
                    + AMat[16 * (row1 * sizex + col1) + 9] * h2             + AMat[16 * (row1 * sizex + col1) + 10] * 2.0F * h2 * t
                    + AMat[16 * (row1 * sizex + col1) + 11] * 3.0F * h2 * t2 + AMat[16 * (row1 * sizex + col1) + 13] * h3
                    + AMat[16 * (row1 * sizex + col1) + 14] * 2.0F * h3 * t  + AMat[16 * (row1 * sizex + col1) + 15] * 3.0F * h3 * t2;

                // calculate interpolated derivative(s) dhdy(t)
                // dhdyt = a(1) + 2 * a(2) * t + 3 * a(3) * t^2 + ...
                //    a(5) * h + 2 * a(6) * h * t + 3 * a(7) * h * t^2 + ...
                //    a(9) * h^2 + 2 * a(10) * h^2 * t + 3 * a(11) * h^2 * t^2 + ...
                //    a(13) * h^3 + 2 * a(14)v* h^3 * t + 3 * a(15) * h^3 * t^2;
                res[npts * 3 + 2] = AMat[16 * (row1 * sizex + col1) + 4] + AMat[16 * (row1 * sizex + col1) + 5] * t
                    + AMat[16 * (row1 * sizex + col1) + 6] * t2             + AMat[16 * (row1 * sizex + col1) + 7] * t3
                    + AMat[16 * (row1 * sizex + col1) + 8] * 2.0F * h        + AMat[16 * (row1 * sizex + col1) + 9] * 2.0F * h * t
                    + AMat[16 * (row1 * sizex + col1) + 10] * 2.0F * h * t2  + AMat[16 * (row1 * sizex + col1) + 11] * 2.0F * h * t3
                    + AMat[16 * (row1 * sizex + col1) + 12] * 3.0F * h2      + AMat[16 * (row1 * sizex + col1) + 13] * 3.0F * h2 * t
                    + AMat[16 * (row1 * sizex + col1) + 14] * 3.0F * h2 * t2 + AMat[16 * (row1 * sizex + col1) + 15] * 3.0F * h2 * t3;
            }
        }
    #if (USEOMP)
    }
    #endif

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doInterpolateQuintic(const float *positions, const int numPts, float *res, const int flags = 0)
{
    // Here we have to do the real job
    // we loop over all points, using openmp to speed up a little bit

    float *AMat = AMatBiQu.data;
    int sizex = AMatBiQu.sizex;
    int sizey = AMatBiQu.sizey;
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
        int row1, col1;
        float rowPos, colPos;
#if (USEOMP)
#pragma omp for schedule(guided)
#endif
        for (int npts = 0; npts < numPts; npts++)
        {
            colPos = positions[npts * 2];
            rowPos = positions[npts * 2 + 1];
            if ((rowPos >= sizey) || (colPos >= sizex) || (rowPos < 1) || (colPos < 1) || !std::isfinite(rowPos) || !std::isfinite(colPos))
                continue;

            if (rowPos < (sizey - 1))
            {
                row1 = (int)floor(rowPos);
            }
            else
            {
                row1 = (int)floor(rowPos - 1);
            }
            if (colPos < (sizex - 1))
            {
                col1 = (int)floor(colPos);
            }
            else
            {
                col1 = (int)floor(colPos - 1);
            }

            float h = rowPos - row1;
            float t = colPos - col1;
            float t2 = t * t, t3 = t * t * t, t4 = t * t * t * t, t5 = t * t * t * t * t;
            float h2 = h * h, h3 = h * h * h, h4 = h * h * h * h, h5 = h * h * h * h * h;

            // calculate interpolated intensity by multiplying ht with A
            // ht = [1     t        t^2        t^3          t^4         t^5 ...
            //       h     h * t    h*t^2      h * t^3      h * t^4     h * t^5 ...
            //       h^2   h^2 * t  h^2 * t^2  h^2 * t^3    h^2 * t^4   h^2 * t^5 ...
            //       h^3   h^3 * t  h^3 * t^2  h^3 * t^3    h^3 * t^4   h^3 * t^5 ...
            //       h^4   h^4 * t  h^4 * t^2  h^4 * t^3    h^4 * t^4   h^4 * t^5 ...
            //       h^5   h^5 * t  h^5 * t^2  h^5 * t^3    h^5 * t^4   h^5 * t^5];

            res[npts * 3] = AMat[16 * (row1 * sizex + col1)]     + AMat[16 * (row1 * sizex + col1) + 1] * t
                + AMat[16 * (row1 * sizex + col1) + 2] * t2         + AMat[16 * (row1 * sizex + col1) + 3] * t3
                + AMat[16 * (row1 * sizex + col1) + 4] * t4         + AMat[16 * (row1 * sizex + col1) + 5] * t5
                + AMat[16 * (row1 * sizex + col1) + 6] * h          + AMat[16 * (row1 * sizex + col1) + 7] * h * t
                + AMat[16 * (row1 * sizex + col1) + 8] * h * t2     + AMat[16 * (row1 * sizex + col1) + 9] * h * t3
                + AMat[16 * (row1 * sizex + col1) + 10] * h * t4    + AMat[16 * (row1 * sizex + col1) + 11] * h * t5
                + AMat[16 * (row1 * sizex + col1) + 12] * h2        + AMat[16 * (row1 * sizex + col1) + 13] * h2 * t
                + AMat[16 * (row1 * sizex + col1) + 14] * h2 * t2   + AMat[16 * (row1 * sizex + col1) + 15] * h2 * t3
                + AMat[16 * (row1 * sizex + col1) + 16] * h2 * t4   + AMat[16 * (row1 * sizex + col1) + 17] * h2 * t5
                + AMat[16 * (row1 * sizex + col1) + 18] * h3        + AMat[16 * (row1 * sizex + col1) + 19] * h3 * t
                + AMat[16 * (row1 * sizex + col1) + 20] * h3 * t2   + AMat[16 * (row1 * sizex + col1) + 21] * h3 * t3
                + AMat[16 * (row1 * sizex + col1) + 22] * h3 * t4   + AMat[16 * (row1 * sizex + col1) + 23] * h3 * t5
                + AMat[16 * (row1 * sizex + col1) + 24] * h4        + AMat[16 * (row1 * sizex + col1) + 25] * h4 * t
                + AMat[16 * (row1 * sizex + col1) + 26] * h4 * t2   + AMat[16 * (row1 * sizex + col1) + 27] * h4 * t3
                + AMat[16 * (row1 * sizex + col1) + 28] * h4 * t4   + AMat[16 * (row1 * sizex + col1) + 29] * h4 * t5
                + AMat[16 * (row1 * sizex + col1) + 30] * h5        + AMat[16 * (row1 * sizex + col1) + 31] * h5 * t
                + AMat[16 * (row1 * sizex + col1) + 32] * h5 * t2   + AMat[16 * (row1 * sizex + col1) + 33] * h5 * t3
                + AMat[16 * (row1 * sizex + col1) + 34] * h5 * t4   + AMat[16 * (row1 * sizex + col1) + 35] * h5 * t5;

            if (flags & 1)
            {
                // calculate interpolated derivative(s) dhdx(t)
                res[npts * 3 + 1] = AMat[16 * (row1 * sizex + col1) + 1] + 2.0F * AMat[16 * (row1 * sizex + col1) + 2] * h
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 3] * h2 + 4.0F * AMat[16 * (row1 * sizex + col1) + 4] * h3
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 5] * h4 + AMat[16 * (row1 * sizex + col1) + 7] * t
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 8] * h * t + 3.0F * AMat[16 * (row1 * sizex + col1) + 9] * h2 * t
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 10] * h3 * t + 5.0F * AMat[16 * (row1 * sizex + col1) + 11] * h4 * t
                    + AMat[16 * (row1 * sizex + col1) + 13] * t2 + 2.0F * AMat[16 * (row1 * sizex + col1) + 14] * h * t2
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 15] * h2 * t2 + 4.0F * AMat[16 * (row1 * sizex + col1) + 16] * h3 * t2
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 17] * h4 * t2 + AMat[16 * (row1 * sizex + col1) + 19] * t3
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 20] * h * t3 + 3.0F * AMat[16 * (row1 * sizex + col1) + 21] * h2 * t3
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 22] * h3 * t3 + 5.0F * AMat[16 * (row1 * sizex + col1) + 23] * h4 * t3
                    + AMat[16 * (row1 * sizex + col1) + 25] * t4 + 2.0F * AMat[16 * (row1 * sizex + col1) + 26] * h * t4
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 27] * h2 * t4 + 4.0F * AMat[16 * (row1 * sizex + col1) + 28] * h3 * t4
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 29] * h4 * t4 + AMat[16 * (row1 * sizex + col1) + 31] * t5
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 32] * h * t5 + 3.0F * AMat[16 * (row1 * sizex + col1) + 33] * h2 * t5
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 34] * h3 * t5  + 5.0F * AMat[16 * (row1 * sizex + col1) + 35] * h4 * t5;

                // calculate interpolated derivative(s) dhdy(t)
                res[npts * 3 + 2] = AMat[16 * (row1 * sizex + col1) + 6] + AMat[16 * (row1 * sizex + col1) + 7] * h
                    + AMat[16 * (row1 * sizex + col1) + 8] * h2 + AMat[16 * (row1 * sizex + col1) + 9] * h3
                    + AMat[16 * (row1 * sizex + col1) + 10] * h4 + AMat[16 * (row1 * sizex + col1) + 11] * h5
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 12] * t + 2.0F * AMat[16 * (row1 * sizex + col1) + 13] * h * t
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 14] * h2 * t + 2.0F * AMat[16 * (row1 * sizex + col1) + 15] * h3 * t
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 16] * h4 * t + 2.0F * AMat[16 * (row1 * sizex + col1) + 17] * h5 * t
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 18] * t2 + 3.0F * AMat[16 * (row1 * sizex + col1) + 19] * h * t2
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 20] * h2 * t2 + 3.0F * AMat[16 * (row1 * sizex + col1) + 21] * h3 * t2
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 22] * h4 * t2 + 3.0F * AMat[16 * (row1 * sizex + col1) + 23] * h5 * t2
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 24] * t3 + 4.0F * AMat[16 * (row1 * sizex + col1) + 25] * h * t3
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 26] * h2 * t3 + 4.0F * AMat[16 * (row1 * sizex + col1) + 27] * h3 * t3
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 28] * h4 * t3 + 4.0F * AMat[16 * (row1 * sizex + col1) + 29] * h5 * t3
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 30] * t4 + 5.0F * AMat[16 * (row1 * sizex + col1) + 31] * h * t4
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 32] * h2 * t4 + 5.0F * AMat[16 * (row1 * sizex + col1) + 33] * h3 * t4
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 34] * h4 * t4 + 5.0F * AMat[16 * (row1 * sizex + col1) + 35] * h5 * t4;
            }
        }
#if (USEOMP)
    }
#endif

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal doInterpolateHeptic(const float *positions, const int numPos, float *res, const int flags = 0)
{
    // Here we have to do the real job
    // we loop over all points, using openmp to speed up a little bit

    float *AMat = AMatBiHe.data;
    int sizex = AMatBiHe.sizex;
    int sizey = AMatBiHe.sizey;
#if (USEOMP)
#pragma omp parallel num_threads(NTHREADS)
    {
#endif
        int row1, col1;
        float rowPos, colPos;
#if (USEOMP)
#pragma omp for schedule(guided)
#endif
        for (int npts = 0; npts < numPos; npts++)
        {
            colPos = positions[npts * 2];
            rowPos = positions[npts * 2 + 1];
            if ((rowPos >= sizey - 1) || (colPos >= sizex - 1) || (rowPos < 1) || (colPos < 1) || !std::isfinite(rowPos) || !std::isfinite(colPos))
                continue;

            if (rowPos < (sizey - 2))
            {
                row1 = (int)floor(rowPos);
            }
            else
            {
                row1 = (int)floor(rowPos - 2);
            }
            if (colPos < (sizex - 2))
            {
                col1 = (int)floor(colPos);
            }
            else
            {
                col1 = (int)floor(colPos - 2);
            }

            float h = rowPos - row1;
            float t = colPos - col1;
            float t2 = t * t, t3 = t * t * t, t4 = t * t * t * t, t5 = t * t * t * t * t, t6 = t * t * t * t * t * t, t7 = t * t * t * t * t * t * t;
            float h2 = h * h, h3 = h * h * h, h4 = h * h * h * h, h5 = h * h * h * h * h, h6 = h * h * h * h * h * h, h7 = h * h * h * h * h * h * h;

            // calculate interpolated intensity by multiplying ht with A
            // ht = [1     t        t^2        t^3          t^4         t^5         t^6         t^7 ...
            //       h     h * t    h*t^2      h * t^3      h * t^4     h * t^5     h * t^6     h * t^7 ...
            //       h^2   h^2 * t  h^2 * t^2  h^2 * t^3    h^2 * t^4   h^2 * t^5   h^2 * t^6   h^2 * t^7 ...
            //       h^3   h^3 * t  h^3 * t^2  h^3 * t^3    h^3 * t^4   h^3 * t^5   h^3 * t^6   h^3 * t^7 ...
            //       h^4   h^4 * t  h^4 * t^2  h^4 * t^3    h^4 * t^4   h^4 * t^5   h^4 * t^6   h^4 * t^7 ...
            //       h^5   h^5 * t  h^5 * t^2  h^5 * t^3    h^5 * t^4   h^5 * t^5   h^5 * t^6   h^5 * t^7 ...
            //       h^6   h^6 * t  h^6 * t^2  h^6 * t^3    h^6 * t^4   h^6 * t^5   h^6 * t^6   h^6 * t^7 ...
            //       h^7   h^5 * t  h^7 * t^2  h^7 * t^3    h^7 * t^4   h^7 * t^5   h^7 * t^6   h^7 * t^7];

            res[npts * 3] = AMat[16 * (row1 * sizex + col1)]     + AMat[16 * (row1 * sizex + col1) + 1] * t
                + AMat[16 * (row1 * sizex + col1) + 2] * t2         + AMat[16 * (row1 * sizex + col1) + 3] * t3
                + AMat[16 * (row1 * sizex + col1) + 4] * t4         + AMat[16 * (row1 * sizex + col1) + 5] * t5
                + AMat[16 * (row1 * sizex + col1) + 6] * t6         + AMat[16 * (row1 * sizex + col1) + 7] * t7

                + AMat[16 * (row1 * sizex + col1) + 8] * h          + AMat[16 * (row1 * sizex + col1) + 9] * h * t
                + AMat[16 * (row1 * sizex + col1) + 10] * h * t2    + AMat[16 * (row1 * sizex + col1) + 11] * h * t3
                + AMat[16 * (row1 * sizex + col1) + 12] * h * t4    + AMat[16 * (row1 * sizex + col1) + 13] * h * t5
                + AMat[16 * (row1 * sizex + col1) + 14] * h * t6    + AMat[16 * (row1 * sizex + col1) + 15] * h * t7

                + AMat[16 * (row1 * sizex + col1) + 16] * h2        + AMat[16 * (row1 * sizex + col1) + 17] * h2 * t
                + AMat[16 * (row1 * sizex + col1) + 18] * h2 * t2   + AMat[16 * (row1 * sizex + col1) + 19] * h2 * t3
                + AMat[16 * (row1 * sizex + col1) + 20] * h2 * t4   + AMat[16 * (row1 * sizex + col1) + 21] * h2 * t5
                + AMat[16 * (row1 * sizex + col1) + 22] * h2 * t6   + AMat[16 * (row1 * sizex + col1) + 23] * h2 * t7

                + AMat[16 * (row1 * sizex + col1) + 24] * h3        + AMat[16 * (row1 * sizex + col1) + 25] * h3 * t
                + AMat[16 * (row1 * sizex + col1) + 26] * h3 * t2   + AMat[16 * (row1 * sizex + col1) + 27] * h3 * t3
                + AMat[16 * (row1 * sizex + col1) + 28] * h3 * t4   + AMat[16 * (row1 * sizex + col1) + 29] * h3 * t5
                + AMat[16 * (row1 * sizex + col1) + 30] * h3 * t6   + AMat[16 * (row1 * sizex + col1) + 31] * h3 * t7

                + AMat[16 * (row1 * sizex + col1) + 32] * h4        + AMat[16 * (row1 * sizex + col1) + 33] * h4 * t
                + AMat[16 * (row1 * sizex + col1) + 34] * h4 * t2   + AMat[16 * (row1 * sizex + col1) + 35] * h4 * t3
                + AMat[16 * (row1 * sizex + col1) + 36] * h4 * t4   + AMat[16 * (row1 * sizex + col1) + 37] * h4 * t5
                + AMat[16 * (row1 * sizex + col1) + 38] * h4 * t6   + AMat[16 * (row1 * sizex + col1) + 39] * h4 * t7

                + AMat[16 * (row1 * sizex + col1) + 40] * h5        + AMat[16 * (row1 * sizex + col1) + 41] * h5 * t
                + AMat[16 * (row1 * sizex + col1) + 42] * h5 * t2   + AMat[16 * (row1 * sizex + col1) + 43] * h5 * t3
                + AMat[16 * (row1 * sizex + col1) + 44] * h5 * t4   + AMat[16 * (row1 * sizex + col1) + 45] * h5 * t5
                + AMat[16 * (row1 * sizex + col1) + 46] * h5 * t6   + AMat[16 * (row1 * sizex + col1) + 47] * h5 * t7

                + AMat[16 * (row1 * sizex + col1) + 48] * h6        + AMat[16 * (row1 * sizex + col1) + 49] * h6 * t
                + AMat[16 * (row1 * sizex + col1) + 50] * h6 * t2   + AMat[16 * (row1 * sizex + col1) + 51] * h6 * t3
                + AMat[16 * (row1 * sizex + col1) + 52] * h6 * t4   + AMat[16 * (row1 * sizex + col1) + 53] * h6 * t5
                + AMat[16 * (row1 * sizex + col1) + 54] * h6 * t6   + AMat[16 * (row1 * sizex + col1) + 55] * h6 * t7

                + AMat[16 * (row1 * sizex + col1) + 56] * h7        + AMat[16 * (row1 * sizex + col1) + 57] * h7 * t
                + AMat[16 * (row1 * sizex + col1) + 58] * h7 * t2   + AMat[16 * (row1 * sizex + col1) + 59] * h7 * t3
                + AMat[16 * (row1 * sizex + col1) + 60] * h7 * t4   + AMat[16 * (row1 * sizex + col1) + 61] * h7 * t5
                + AMat[16 * (row1 * sizex + col1) + 62] * h7 * t6   + AMat[16 * (row1 * sizex + col1) + 63] * h7 * t7;

            if (flags & 1)
            {
                // calculate interpolated derivative(s) dhdx(t)
                // check last coefficients of derivative, #63 was duplicated, #62 missing
                res[npts * 3 + 1] = AMat[16 * (row1 * sizex + col1) + 1] + 2.0F * AMat[16 * (row1 * sizex + col1) + 2] * h
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 3] * h2 + 4.0F * AMat[16 * (row1 * sizex + col1) + 4] * h3
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 5] * h4 + 6.0F * AMat[16 * (row1 * sizex + col1) + 6] * h5
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 7] * h6 + AMat[16 * (row1 * sizex + col1) + 9] * t
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 10] * h * t + 3.0F * AMat[16 * (row1 * sizex + col1) + 11] * h2 * t
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 12] * h3 * t + 5.0F * AMat[16 * (row1 * sizex + col1) + 13] * h4 * t
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 14] * h5 * t + 7.0F * AMat[16 * (row1 * sizex + col1) + 15] * h6 * t
                    + AMat[16 * (row1 * sizex + col1) + 17] * t2 + 2.0F * AMat[16 * (row1 * sizex + col1) + 18] * h * t2
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 19] * h2 * t2 + 4.0F * AMat[16 * (row1 * sizex + col1) + 20] * h3 * t2
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 21] * h4 * t2 + 6.0F * AMat[16 * (row1 * sizex + col1) + 22] * h5 * t2
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 23] * h6 * t2 + AMat[16 * (row1 * sizex + col1) + 25] * t3
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 26] * h * t3 + 3.0F * AMat[16 * (row1 * sizex + col1) + 27] * h2 * t3
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 28] * h3 * t3 + 5.0F * AMat[16 * (row1 * sizex + col1) + 29] * h4 * t3
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 30] * h5 * t3 + 7.0F * AMat[16 * (row1 * sizex + col1) + 31] * h6 * t3
                    + AMat[16 * (row1 * sizex + col1) + 33] * t4 + 2.0F * AMat[16 * (row1 * sizex + col1) + 34] * h * t4
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 35] * h2 * t4 + 4.0F * AMat[16 * (row1 * sizex + col1) + 36] * h3 * t4
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 37] * h4 * t4 + 6.0F * AMat[16 * (row1 * sizex + col1) + 38] * h5 * t4
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 39] * h6 * t4 + AMat[16 * (row1 * sizex + col1) + 41] * t5
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 42] * h * t5 + 3.0F * AMat[16 * (row1 * sizex + col1) + 43] * h2 * t5
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 44] * h3 * t5 + 5.0F * AMat[16 * (row1 * sizex + col1) + 45] * h4 * t5
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 46] * h5 * t5 + 7.0F * AMat[16 * (row1 * sizex + col1) + 47] * h6 * t5
                    + AMat[16 * (row1 * sizex + col1) + 49] * t6 + 2.0F * AMat[16 * (row1 * sizex + col1) + 50] * h * t6
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 51] * h2 * t6 + 4.0F * AMat[16 * (row1 * sizex + col1) + 52] * h3 * t6
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 53] * h4 * t6 + 6.0F * AMat[16 * (row1 * sizex + col1) + 54] * h5 * t6
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 55] * h6 * t6 + AMat[16 * (row1 * sizex + col1) + 57] * t7
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 58] * h * t7 + 3.0F * AMat[16 * (row1 * sizex + col1) + 59] * h2 * t7
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 60] * h3 * t7 + 5.0F * AMat[16 * (row1 * sizex + col1) + 61] * h4 * t7
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 62] * h5 * t7 + 7.0F * AMat[16 * (row1 * sizex + col1) + 63] * h6 * t7;

                // calculate interpolated derivative(s) dhdy(t)
                res[npts * 3 + 2] = AMat[16 * (row1 * sizex + col1) + 8] + AMat[16 * (row1 * sizex + col1) + 9] * h
                    + AMat[16 * (row1 * sizex + col1) + 10] * h2 + AMat[16 * (row1 * sizex + col1) + 11] * h3
                    + AMat[16 * (row1 * sizex + col1) + 12] * h4 + AMat[16 * (row1 * sizex + col1) + 13] * h5
                    + AMat[16 * (row1 * sizex + col1) + 14] * h6 + AMat[16 * (row1 * sizex + col1) + 15] * h7
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 16] * t + 2.0F * AMat[16 * (row1 * sizex + col1) + 17] * h * t
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 18] * h2 * t + 2.0F * AMat[16 * (row1 * sizex + col1) + 19] * h3 * t
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 20] * h4 * t + 2.0F * AMat[16 * (row1 * sizex + col1) + 21] * h5 * t
                    + 2.0F * AMat[16 * (row1 * sizex + col1) + 22] * h6 * t + 2.0F * AMat[16 * (row1 * sizex + col1) + 23] * h7 * t
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 24] * t2 + 3.0F * AMat[16 * (row1 * sizex + col1) + 25] * h * t2
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 26] * h2 * t2 + 3.0F * AMat[16 * (row1 * sizex + col1) + 27] * h3 * t2
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 28] * h4 * t2 + 3.0F * AMat[16 * (row1 * sizex + col1) + 29] * h5 * t2
                    + 3.0F * AMat[16 * (row1 * sizex + col1) + 30] * h6 * t2 + 3.0F * AMat[16 * (row1 * sizex + col1) + 31] * h7 * t2
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 32] * t3 + 4.0F * AMat[16 * (row1 * sizex + col1) + 33] * h * t3
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 34] * h2 * t3 + 4.0F * AMat[16 * (row1 * sizex + col1) + 35] * h3 * t3
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 36] * h4 * t3 + 4.0F * AMat[16 * (row1 * sizex + col1) + 37] * h5 * t3
                    + 4.0F * AMat[16 * (row1 * sizex + col1) + 38] * h6 * t3 + 4.0F * AMat[16 * (row1 * sizex + col1) + 39] * h7 * t3
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 40] * t4 + 5.0F * AMat[16 * (row1 * sizex + col1) + 41] * h * t4
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 42] * h2 * t4 + 5.0F * AMat[16 * (row1 * sizex + col1) + 43] * h3 * t4
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 44] * h4 * t4 + 5.0F * AMat[16 * (row1 * sizex + col1) + 45] * h5 * t4
                    + 5.0F * AMat[16 * (row1 * sizex + col1) + 46] * h6 * t4 + 5.0F * AMat[16 * (row1 * sizex + col1) + 47] * h7 * t4
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 48] * t5 + 6.0F * AMat[16 * (row1 * sizex + col1) + 49] * h * t5
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 50] * h2 * t5 + 6.0F * AMat[16 * (row1 * sizex + col1) + 51] * h3 * t5
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 52] * h4 * t5 + 6.0F * AMat[16 * (row1 * sizex + col1) + 53] * h5 * t5
                    + 6.0F * AMat[16 * (row1 * sizex + col1) + 54] * h6 * t5 + 6.0F * AMat[16 * (row1 * sizex + col1) + 55] * h7 * t5
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 56] * t6 + 7.0F * AMat[16 * (row1 * sizex + col1) + 57] * h * t6
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 58] * h2 * t6 + 7.0F * AMat[16 * (row1 * sizex + col1) + 59] * h3 * t6
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 60] * h4 * t6 + 7.0F * AMat[16 * (row1 * sizex + col1) + 61] * h5 * t6
                    + 7.0F * AMat[16 * (row1 * sizex + col1) + 62] * h6 * t6 + 7.0F * AMat[16 * (row1 * sizex + col1) + 63] * h7 * t6;
            }
        }
#if (USEOMP)
    }
#endif

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
template ito::RetVal doInterpolateLinear<unsigned char>(const unsigned char* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
template ito::RetVal doInterpolateLinear<signed char>(const signed char* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
template ito::RetVal doInterpolateLinear<unsigned short>(const unsigned short* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
template ito::RetVal doInterpolateLinear<signed short>(const signed short* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
template ito::RetVal doInterpolateLinear<unsigned int>(const unsigned int* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
template ito::RetVal doInterpolateLinear<signed int>(const signed int* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
template ito::RetVal doInterpolateLinear<float>(const float* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
template ito::RetVal doInterpolateLinear<double>(const double* inPtr, const int width, const int height, const int stepin,
    const float* positions, const int numPos, float* outPtr, const int stepout, const int flags);
