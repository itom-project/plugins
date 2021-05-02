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

#include <vector>
#include "dicInterpolation.hu"
//#include "dicInterpolationMats.h"
#include "dicInterpolationBMatBiCu.h"

#include "opencv2/imgproc/imgproc.hpp"

struct dImgMat {
    int sizex;
    int sizey;
    int step;
    void *d_ptr;
    void *h_ptr;
    
    dImgMat() : sizex(0), sizey(0), step(0), d_ptr(NULL), h_ptr(NULL) {}
};

std::vector<struct dImgMat> dImgBuffers;

//extern double BMatBiCu[256];
//extern double BMatBiQu[1296];
//extern double BMatBiQi[4096];
// Matrices are stored in row-major order:
// M(row, col) = *(M.elements + row * M.stride + col)
/*
template<typename _Tp> class Matrix {
    int width;
    int height;
    size_t step;
    char colWise;
    _Tp* elements;

    Matrix() : width(0), height(0), step(0), colWise(0), elements(NULL) {}
};

template<typename _Tp> class Vector {
    int length;
    _Tp *elements;

    Vector() : length(0), elements(NULL) {}
};
*/
//--------------------------------------------------------------------------------------------------
/*
// Get a matrix element
template<typename _Tp> __device__ float d_GetElement(const Matrix<_Tp> &A, int row, int col)
{
    if (A.colWise)
        return A.elements[row * A.stride + col];
    else
        return A.elements[col * A.stride + row];
}

//--------------------------------------------------------------------------------------------------
// Set a matrix element
template<typename _Tp> __device__ void d_SetElement(Matrix<_Tp> A, int row, int col,
                           _Tp value)
{
    if (A.colWise)
        A.elements[row * A.stride + col] = value;
    else
        A.elements[col * A.stride + row] = value;
}

//--------------------------------------------------------------------------------------------------
// Get the BLOCK_SIZExBLOCK_SIZE sub-matrix Asub of A that is
// located col sub-matrices to the right and row sub-matrices down
// from the upper-left corner of A
 template<typename _Tp> __device__ void d_GetSubMatrix(Matrix<_Tp> &A, int row, int col) 
{
    Matrix<_Tp> Asub;
    Asub.width    = BLOCK_SIZE;
    Asub.height   = BLOCK_SIZE;
    Asub.stride   = A.stride;
    if (A.colPri)
        Asub.elements = &A.elements[A.stride * BLOCK_SIZE * row
                                            + BLOCK_SIZE * col];
    else
        Asub.elements = &A.elements[A.stride * BLOCK_SIZE * col
                                            + BLOCK_SIZE * row];
    return;
}
*/
//--------------------------------------------------------------------------------------------------
__global__ void d_interpolBiCu(const float* __restrict__ pts, int numPts,
    const float* __restrict__ imgIn, int width, int height, int step, int colWise, const float* __restrict__ dxp, 
    const float* __restrict__ dyp, const float* __restrict__ dxyp, float* __restrict__ intensVals)
{
    int blockId = blockIdx.x + blockIdx.y * gridDim.x;
    int ptId = blockId * (blockDim.x * blockDim.y) + (threadIdx.y * blockDim.x) + threadIdx.x;

    if (ptId < numPts)
    {
        float fIdxPos;
        float sIdxPos;
        int sIdx1, fIdx1;
        int fSize, sSize;

        if (colWise)
        {
            fIdxPos = pts[ptId * 2 + 1];
            sIdxPos = pts[ptId * 2];
            fSize = height;
            sSize = width;
        }
        else
        {
            fIdxPos = pts[ptId * 2];
            sIdxPos = pts[ptId * 2 + 1];
            fSize = width;
            sSize = height;
        }

        if ((fIdxPos >= fSize) || (sIdxPos >= sSize) || (fIdxPos < 0) || (sIdxPos < 0)
            || !isfinite(fIdxPos) || !isfinite(sIdxPos))
            return;

        if (sIdxPos < (sSize - 1))
        {
            sIdx1 = (int)floor(sIdxPos);
        }
        else
        {
            sIdx1 = (int)floor(sIdxPos - 1);
        }
        if (fIdxPos < (fSize - 1))
        {
            fIdx1 = (int)floor(fIdxPos);
        }
        else
        {
            fIdx1 = (int)floor(fIdxPos - 1);
        }

        float h = sIdxPos - sIdx1;
        float t = fIdxPos - fIdx1;
        float t2 = t * t, t3 = t * t * t;
        float h2 = h * h, h3 = h * h * h;

        // calculate interpolated intensity by multiplying ht with A
        // ht = [1     t        t^2        t^3 ...
        //       h     h * t    h*t^2      h * t^3 ...
        //       h^2   h^2 * t  h^2 * t^2  h^2 * t^3 ...
        //       h^3   h^3 * t  h^3 * t^2  h^3 * t^3];

        // wikipedia version
        float coeffVec[16];
        float I0 = imgIn[sIdx1 * step + fIdx1];
        float I1 = imgIn[sIdx1 * step + fIdx1 + 1];
        float I2 = imgIn[(sIdx1 + 1) * step + fIdx1];
        float I3 = imgIn[(sIdx1 + 1) * step + fIdx1 + 1];
        float DX0 = dxp[sIdx1 * step + fIdx1];
        float DX1 = dxp[sIdx1 * step + fIdx1 + 1];
        float DX2 = dxp[(sIdx1 + 1) * step + fIdx1];
        float DX3 = dxp[(sIdx1 + 1) * step + fIdx1 + 1];
        float DY0 = dyp[sIdx1 * step + fIdx1];
        float DY1 = dyp[sIdx1 * step + fIdx1 + 1];
        float DY2 = dyp[(sIdx1 + 1) * step + fIdx1];
        float DY3 = dyp[(sIdx1 + 1) * step + fIdx1 + 1];
        float DXY0 = dxyp[sIdx1 * step + fIdx1];
        float DXY1 = dxyp[sIdx1 * step + fIdx1 + 1];
        float DXY2 = dxyp[(sIdx1 + 1) * step + fIdx1];
        float DXY3 = dxyp[(sIdx1 + 1) * step + fIdx1 + 1];
        coeffVec[0] = BMatBiCu0	* I0;
        coeffVec[1] = BMatBiCu20	* DX0;
        coeffVec[2] = BMatBiCu32	* I0
            + BMatBiCu33	* I1
            + BMatBiCu36	* DX0
            + BMatBiCu37	* DX1;
        coeffVec[3] = BMatBiCu48	* I0
            + BMatBiCu49	* I1
            + BMatBiCu52	* DX0
            + BMatBiCu53	* DX1;
        coeffVec[4] = BMatBiCu72	* DY0;
        coeffVec[5] = BMatBiCu92	* DXY0;
        coeffVec[6] = BMatBiCu104	* DY0
            + BMatBiCu105	* DY1
            + BMatBiCu108	* DXY0
            + BMatBiCu109	* DXY1;
        coeffVec[7] = BMatBiCu120	* DY0
            + BMatBiCu121	* DY1
            + BMatBiCu124	* DXY0
            + BMatBiCu125	* DXY1;
        coeffVec[8] = BMatBiCu128	* I0
            + BMatBiCu130	* I2
            + BMatBiCu136	* DY0
            + BMatBiCu138	* DY2;
        coeffVec[9] = BMatBiCu148	* DX0
            + BMatBiCu150	* DX2
            + BMatBiCu156	* DXY0
            + BMatBiCu158	* DXY2;
        coeffVec[10] = BMatBiCu160	* I0
            + BMatBiCu161	* I1
            + BMatBiCu162	* I2
            + BMatBiCu163	* I3
            + BMatBiCu164	* DX0
            + BMatBiCu165	* DX1
            + BMatBiCu166	* DX2
            + BMatBiCu167	* DX3
            + BMatBiCu168	* DY0
            + BMatBiCu169	* DY1
            + BMatBiCu170	* DY2
            + BMatBiCu171	* DY3
            + BMatBiCu172	* DXY0
            + BMatBiCu173	* DXY1
            + BMatBiCu174	* DXY2
            + BMatBiCu175	* DXY3;
        coeffVec[11] = BMatBiCu176	* I0
            + BMatBiCu177	* I1
            + BMatBiCu178	* I2
            + BMatBiCu179	* I3
            + BMatBiCu180	* DX0
            + BMatBiCu181	* DX1
            + BMatBiCu182	* DX2
            + BMatBiCu183	* DX3
            + BMatBiCu184	* DY0
            + BMatBiCu185	* DY1
            + BMatBiCu186	* DY2
            + BMatBiCu187	* DY3
            + BMatBiCu188	* DXY0
            + BMatBiCu189	* DXY1
            + BMatBiCu190	* DXY2
            + BMatBiCu191	* DXY3;
        coeffVec[12] = BMatBiCu192	* I0
            + BMatBiCu194	* I2
            + BMatBiCu200	* DY0
            + BMatBiCu202	* DY2;
        coeffVec[13] = BMatBiCu212	* DX0
            + BMatBiCu214	* DX2
            + BMatBiCu220	* DXY0
            + BMatBiCu222	* DXY2;
        coeffVec[14] = BMatBiCu224	* I0
            + BMatBiCu225	* I1
            + BMatBiCu226	* I2
            + BMatBiCu227	* I3
            + BMatBiCu228	* DX0
            + BMatBiCu229	* DX1
            + BMatBiCu230	* DX2
            + BMatBiCu231	* DX3
            + BMatBiCu232	* DY0
            + BMatBiCu233	* DY1
            + BMatBiCu234	* DY2
            + BMatBiCu235	* DY3
            + BMatBiCu236	* DXY0
            + BMatBiCu237	* DXY1
            + BMatBiCu238	* DXY2
            + BMatBiCu239	* DXY3;
        coeffVec[15] = BMatBiCu240	* I0
            + BMatBiCu241	* I1
            + BMatBiCu242	* I2
            + BMatBiCu243	* I3
            + BMatBiCu244	* DX0
            + BMatBiCu245	* DX1
            + BMatBiCu246	* DX2
            + BMatBiCu247	* DX3
            + BMatBiCu248	* DY0
            + BMatBiCu249	* DY1
            + BMatBiCu250	* DY2
            + BMatBiCu251	* DY3
            + BMatBiCu252	* DXY0
            + BMatBiCu253	* DXY1
            + BMatBiCu254	* DXY2
            + BMatBiCu255	* DXY3;

        intensVals[ptId * 3] = coeffVec[0] + coeffVec[1] * t + coeffVec[2] * t2 + coeffVec[3] * t3
            + coeffVec[4] * h + coeffVec[5] * h * t + coeffVec[6] * h * t2 + coeffVec[7] * h * t3
            + coeffVec[8] * h2 + coeffVec[9] * h2 * t + coeffVec[10] * h2 * t2 + coeffVec[11] * h2 * t3
            + coeffVec[12] * h3 + coeffVec[13] * h3 * t + coeffVec[14] * h3 * t2 + coeffVec[15] * h3 * t3;

        //if (flags & 1)
        {
            // calculate interpolated derivative(s) dhdx(t)
            // dhdxt = a(4) + a(5) * t + a(6) * t^2 + a(7) * t^3 ...
            //        + 2 * a(8) * h + 2 * a(9) * h * t + 2 * a(10) * h * t^2 + 2 * a(11) * h * t^3 ...
            //        + 3 * a(12) * h^2 + 3 * a(13) * h^2 * t + 3 * a(14) * h^2 * t^2 + 3 * a(15) * h^2 * t^3;
            intensVals[ptId * 3 + 1] = coeffVec[1] + coeffVec[2] * 2.0 * t + coeffVec[3] * 3.0 * t2 + coeffVec[5] * h
                + coeffVec[6] * 2.0 * h * t + coeffVec[7] * 3.0 * h * t2 + coeffVec[9] * h2 + coeffVec[10] * 2.0 * h2 * t
                + coeffVec[11] * 3.0 * h2 * t2 + coeffVec[13] * h3 + coeffVec[14] * 2.0 * h3 * t + coeffVec[15] * 3.0 * h3 * t2;

            // calculate interpolated derivative(s) dhdy(t)
            // dhdyt = a(1) + 2 * a(2) * t + 3 * a(3) * t^2 + ...
            //    a(5) * h + 2 * a(6) * h * t + 3 * a(7) * h * t^2 + ...
            //    a(9) * h^2 + 2 * a(10) * h^2 * t + 3 * a(11) * h^2 * t^2 + ...
            //    a(13) * h^3 + 2 * a(14)v* h^3 * t + 3 * a(15) * h^3 * t^2;
            intensVals[ptId * 3 + 2] = coeffVec[4] + coeffVec[5] * t + coeffVec[6] * t2 + coeffVec[7] * t3
                + coeffVec[8] * 2.0 * h + coeffVec[9] * 2.0 * h * t + coeffVec[10] * 2.0 * h * t2 + coeffVec[11] * 2.0 * h * t3
                + coeffVec[12] * 3.0 * h2 + coeffVec[13] * 3.0 * h2 * t + coeffVec[14] * 3.0 * h2 * t2 + coeffVec[15] * 3.0 * h2 * t3;
        }
    }
}

//--------------------------------------------------------------------------------------------------
template<typename _Tp> __global__ void d_interpolBiLi(const float* __restrict__ pts, int numPts, 
    const _Tp* __restrict__ imgIn, int width, int height, int step, int colWise, float* __restrict__ intensVals)
{
    int blockId = blockIdx.x + blockIdx.y * gridDim.x;
    int ptId = blockId * (blockDim.x * blockDim.y) + (threadIdx.y * blockDim.x) + threadIdx.x;

    if (ptId < numPts)
    {
        float fIdxPos;
        float sIdxPos;
        int sIdx1, sIdx2, fIdx1, fIdx2;
        int fSize, sSize;
        float DY1, DY2, DX1, DX2;

        if (colWise)
        {
            fIdxPos = pts[ptId * 2 + 1];
            sIdxPos = pts[ptId * 2];
            fSize = height;
            sSize = width;
        }
        else
        {
            fIdxPos = pts[ptId * 2];
            sIdxPos = pts[ptId * 2 + 1];
            fSize = width;
            sSize = height;
        }

        if ((fIdxPos >= fSize) || (sIdxPos >= sSize) || (fIdxPos < 0) || (sIdxPos < 0)
            || !isfinite(fIdxPos) || !isfinite(sIdxPos))
            return;

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

        float X1Y1 = imgIn[sIdx1 * step + fIdx1];
        float X1Y2 = imgIn[sIdx2 * step + fIdx1];
        float X2Y1 = imgIn[sIdx1 * step + fIdx2];
        float X2Y2 = imgIn[sIdx2 * step + fIdx2];

        if (!isfinite(X1Y1) || !isfinite(X1Y2) || !isfinite(X2Y1) || !isfinite(X2Y2))
        {
            //intensVals[ptId] = NPP_MAXABS_32F;
            //outPtr[npts] = std::numeric_limits<_Tp>::max();
            return;
        }

        if (colWise)
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

        intensVals[ptId * 3] = (X1Y1 * DX2 * DY2 + X2Y1 * DX1 * DY2 + X1Y2 * DX2 * DY1 + X2Y2 * DX1 * DY1);
        intensVals[ptId * 3 + 1] = (X2Y1 - X1Y1) + ((X2Y2 + X1Y1) - (X2Y1 + X1Y2)) * DY1;
        intensVals[ptId * 3 + 2] = (X1Y2 - X1Y1) + ((X2Y2 + X1Y1) - (X2Y1 + X1Y2)) * DX1;
    }
}

//--------------------------------------------------------------------------------------------------
template<typename _Tp> ito::RetVal h_interpolBiLi(const _Tp *inPtr, const int sizex, const int sizey, const int stepin, 
    const float *positions, const int numPos, float *outPtr, const int stepOut, const int flag)
{
    ito::RetVal retval;

    cudaError_t cerror;
    dim3 dimGrid;
    dim3 dimBlocks;
    static _Tp *dp_img = NULL;
    static const _Tp *hp_img = NULL;
    static int imgSizex = 0, imgSizey = 0;
    float *dp_pts = NULL, *dp_int = NULL;

    if (numPos <= 256)
    {
        int bs = ceil(sqrt(numPos));
        dimBlocks = dim3(bs, bs);
    }
    else
    {
        dimBlocks = dim3(16, 16);
        int ng1 = ceil(sqrt(numPos / 256.0));
        int ng2 = ceil(numPos / (256.0 * ng1));
        dimGrid = dim3(ng1, ng2);
    }

    // maybe we should refine the combination of checks here. In fact unregistering host memory and right afterwards
    // reregistering it does not work. Anyways this actually should not occur, as, when memory size changed the pointer 
    // address should change. So there should be no need to do that.
    if ((hp_img == NULL || dp_img == NULL || imgSizex != sizex || imgSizey != sizey || flag & 4) && (hp_img != inPtr))
    {
        if (hp_img != NULL)
        {
            cudaHostUnregister((void*)hp_img);
        }
        cudaHostRegister((void*)inPtr, sizex * sizey * sizeof(_Tp), cudaHostRegisterMapped);
        cudaHostGetDevicePointer(&dp_img, (void*)inPtr, 0);
        if (dp_img == NULL)
        {
            cerror = cudaGetLastError();
            return ito::RetVal(ito::retError, 0, cudaGetErrorString(cerror));
        }

        hp_img = inPtr;
        imgSizex = sizex;
        imgSizey = sizey;
    }

    cudaHostRegister((void*)positions, numPos * 2 * sizeof(float), cudaHostRegisterMapped);
    cudaHostGetDevicePointer(&dp_pts, (void*)positions, 0);
    cudaMalloc((void**)&dp_int, 3 * sizeof(float) * numPos);
    
    if (flag & 512)
        d_interpolBiLi<_Tp> << <dimGrid, dimBlocks >> >(dp_pts, numPos, dp_img, sizex, sizey, sizey, 1, dp_int);
    else
        d_interpolBiLi<_Tp> << <dimGrid, dimBlocks >> >(dp_pts, numPos, dp_img, sizex, sizey, sizex, 0, dp_int);
    cudaDeviceSynchronize();

    cudaMemcpy(outPtr, dp_int, 3 * sizeof(float) * numPos, cudaMemcpyDeviceToHost);
    if ((cerror = cudaGetLastError()))
        retval += ito::RetVal(ito::retError, 0, cudaGetErrorString(cerror));

    cudaFree(dp_int);
    cudaHostUnregister((void*)positions);
    if (flag & 2 != 2)
    {
        cudaHostUnregister((void*)hp_img);
        hp_img = NULL;
        dp_img = NULL;
        imgSizex = 0;
        imgSizey = 0;
    }
    if ((cerror = cudaGetLastError()))
        return ito::RetVal(ito::retError, 0, cudaGetErrorString(cerror));
    
    return retval;
}

//--------------------------------------------------------------------------------------------------
ito::RetVal h_interpolAMat(const float *inPtr, const int sizex, const int sizey, const int stepin,
    const float *positions, const int numPos, float *outPtr, const int interpAlgo, const int flag)
{
    ito::RetVal retval;

    cudaError_t cerror;
    dim3 dimGrid;
    dim3 dimBlocks;
    static const float *dp_img = NULL, *dp_dx = NULL, *dp_dy = NULL, *dp_dxy = NULL;
    static const float *hp_img = NULL, *hp_dx = NULL, *hp_dy = NULL, *hp_dxy = NULL;
    static cv::Mat matDx, matDy, matDxy;
    static int imgSizex = 0, imgSizey = 0;
    const float *dp_pts = NULL;
    float *dp_int = NULL;

    if (numPos <= 256)
    {
        int bs = ceil(sqrt(numPos));
        dimBlocks = dim3(bs, bs);
    }
    else
    {
        dimBlocks = dim3(16, 16);
        int ng1 = ceil(sqrt(numPos / 256.0));
        int ng2 = ceil(numPos / (256.0 * ng1));
        dimGrid = dim3(ng1, ng2);
    }

    if (hp_img == NULL || dp_img == NULL || imgSizex != sizex || imgSizey != sizey || (flag & 4) == 4 || (flag & 2 != 2))
    {
        if (hp_img != NULL)
            cudaHostUnregister((void*)hp_img);
        if (hp_dx != NULL)
            cudaHostUnregister((void*)hp_dx);
        if (hp_dy != NULL)
            cudaHostUnregister((void*)hp_dy);
        if (hp_dxy != NULL)
            cudaHostUnregister((void*)hp_dxy);

        if (inPtr == NULL)
            return ito::RetVal(ito::retError, 0, "Error input image pointer is NULL in CUDA interpolate BiCu");
        cudaHostRegister((void*)inPtr, sizex * sizey * sizeof(float), cudaHostRegisterMapped);
        cudaHostGetDevicePointer(&dp_img, (void*)inPtr, 0);

        ito::float32 kernel[9] = { -3.0, 0.0,  3.0,
            -10.0, 0.0, 10.0,
            -3.0, 0.0,  3.0 };
        cv::Mat kernelDx = cv::Mat(cv::Size(3, 3), CV_32F, kernel);
        cv::Mat kernelDy = kernelDx.t();

        //ito::float32 kernel[3] = { -0.5, 0.0, 0.5 };
        //cv::Mat kernelDx = cv::Mat(cv::Size(3, 1), CV_32F, kernel);
        //cv::Mat kernelDy = cv::Mat(cv::Size(1, 3), CV_32F, kernel);

        cv::Mat imgMat(sizey, sizex, CV_32F, (void*)inPtr);

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

        cudaHostRegister((void*)dxp, sizex * sizey * sizeof(float), cudaHostRegisterMapped);
        cudaHostGetDevicePointer(&dp_dx, (void*)dxp, 0);
        cudaHostRegister((void*)dyp, sizex * sizey * sizeof(float), cudaHostRegisterMapped);
        cudaHostGetDevicePointer(&dp_dy, (void*)dyp, 0);
        cudaHostRegister((void*)dxyp, sizex * sizey * sizeof(float), cudaHostRegisterMapped);
        cudaHostGetDevicePointer(&dp_dxy, (void*)dxyp, 0);

        if (dp_img == NULL || dp_dx == NULL || dp_dy == NULL || dp_dxy == NULL)
        {
            cerror = cudaGetLastError();
            return ito::RetVal(ito::retError, 0, cudaGetErrorString(cerror));
        }
        hp_img = inPtr;
        hp_dx = dxp;
        hp_dy = dyp;
        hp_dxy = dxyp;
        imgSizex = sizex;
        imgSizey = sizey;
    }

    cudaHostRegister((void*)positions, numPos * 2 * sizeof(float), cudaHostRegisterMapped);
    cudaHostGetDevicePointer(&dp_pts, (void*)positions, 0);
    cudaMalloc((void**)&dp_int, 3 * sizeof(float) * numPos);

    if (interpAlgo == 1)
    {
        d_interpolBiCu << <dimGrid, dimBlocks >> > (dp_pts, numPos, dp_img, sizex, sizey, sizex, 0, dp_dx, dp_dy, dp_dxy, dp_int);
    }
    else if (interpAlgo == 2)
    {
        //d_interpolBiQi<_Tp> << <dimGrid, dimBlocks >> > (dp_ptsPtr, numPos, dp_imgPtr, sizex, sizey, sizex, 0, dp_intPtr);
    }
    else if (interpAlgo == 3)
    {
        //d_interpolBiHe<_Tp> << <dimGrid, dimBlocks >> > (dp_ptsPtr, numPos, dp_imgPtr, sizex, sizey, sizex, 0, dp_intPtr);
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "unknown interpolation algorithm, no output generated!");
    }
    cudaDeviceSynchronize();

    cudaMemcpy(outPtr, dp_int, 3 * sizeof(float) * numPos, cudaMemcpyDeviceToHost);
    if ((cerror = cudaGetLastError()))
        retval += ito::RetVal(ito::retError, 0, cudaGetErrorString(cerror));

    cudaFree(dp_int);
    cudaHostUnregister((void*)positions);
    if ((flag & 2) != 2)
    {
        cudaHostUnregister((void*)hp_img);
        hp_img = NULL;
        dp_img = NULL;
        cudaHostUnregister((void*)hp_dx);
        hp_dx = NULL;
        dp_dx = NULL;
        cudaHostUnregister((void*)hp_dy);
        hp_dy = NULL;
        dp_dy = NULL;
        cudaHostUnregister((void*)hp_dxy);
        hp_dxy = NULL;
        dp_dxy = NULL;
        imgSizex = 0;
        imgSizey = 0;

        matDx = cv::Mat();
        matDy = cv::Mat();
        matDxy = cv::Mat();
    }
    if ((cerror = cudaGetLastError()))
        return ito::RetVal(ito::retError, 0, cudaGetErrorString(cerror));

    return retval;
}

//--------------------------------------------------------------------------------------------------
ito::RetVal InitCudaDevice(std::vector<int> &devices)
{
    struct cudaDeviceProp prop;
    int numdev;
    cudaThreadExit();
    cudaGetDeviceCount(&numdev);
    if (numdev == 0)
    {
        return -1;
    }

    for (int r = 0; r < numdev; r++)
    {
        cudaGetDeviceProperties(&prop, r);
        if (prop.major >= 1)
        {
            devices.push_back(r);
        }

        if (r == numdev - 1)
        {
            return ito::RetVal(ito::retError, 0, "No CUDA capable device found!\nAborting!\n");
        }
    }

    return ito::retOk;
}

//--------------------------------------------------------------------------------------------------
// template instantiation

template ito::RetVal h_interpolBiLi<unsigned char>(const unsigned char *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);
template ito::RetVal h_interpolBiLi<signed char>(const signed char *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);
template ito::RetVal h_interpolBiLi<unsigned short>(const unsigned short *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);
template ito::RetVal h_interpolBiLi<short>(const short *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);
template ito::RetVal h_interpolBiLi<unsigned long>(const unsigned long *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);
template ito::RetVal h_interpolBiLi<long>(const long *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);
template ito::RetVal h_interpolBiLi<float>(const float *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);
template ito::RetVal h_interpolBiLi<double>(const double *inPtr, const int sizex, const int sizey, const int stepin, const float *positions, const int numPos, float *outPtr, const int stepout, const int flag);

//--------------------------------------------------------------------------------------------------