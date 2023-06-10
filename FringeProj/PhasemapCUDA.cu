/*
 * Copyright 1993-2007 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO USER:
 *
 * This source code is subject to NVIDIA ownership rights under U.S. and
 * international Copyright laws.  Users and possessors of this source code
 * are hereby granted a nonexclusive, royalty-free license to use this code
 * in individual and commercial software.
 *
 * NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE
 * CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR
 * IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
 * IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS,  WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION,  ARISING OUT OF OR IN CONNECTION WITH THE USE
 * OR PERFORMANCE OF THIS SOURCE CODE.
 *
 * U.S. Government End Users.   This source code is a "commercial item" as
 * that term is defined at  48 C.F.R. 2.101 (OCT 1995), consisting  of
 * "commercial computer  software"  and "commercial computer software
 * documentation" as such terms are  used in 48 C.F.R. 12.212 (SEPT 1995)
 * and is provided to the U.S. Government only as a commercial end item.
 * Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through
 * 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the
 * source code with only those rights set forth herein.
 *
 * Any use of this source code in individual and commercial software must
 * include, in the user documentation and internal comments to the code,
 * the above Disclaimer and U.S. Government End Users Notice.
 */

///////////////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#endif

#include "PhasemapCUDADll.h"

#pragma unroll

__device__ __constant__ unsigned short ui_d_Bitshift[MAXGRAYBITS];
__device__ unsigned short bps2cilut[BPSLUTSIZE];

//--------------------------------------------------------------------------------------------------
__global__
void CalcBPS2CILutCUDA(unsigned char maxBits, unsigned short *d_tmpBps2cilut)
{
//	unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
	unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

	int b, bitmask = 0, invert = 0;

	b = y;
	for (int i = maxBits; i >= 0; i-- )
	{
		bitmask = 1<<i;
		if ( invert )
			b = b ^ bitmask;
		if ( y & bitmask )
			invert = !invert;
	}
//	d_tmpBps2cilut[y] = b;
	bps2cilut[y] = b;

	return;
}

extern "C" int CalcBPS2CILut(unsigned char numBits)
{
    dim3 dBlock, dGrid;
    unsigned short numThreads=1<<numBits;
    cudaError_t cerror;
    unsigned short *d_tempBps2cilut=NULL, *h_tempBps2cilut=NULL;

    cudaMalloc((void**)&d_tempBps2cilut, BPSLUTSIZE*sizeof(unsigned short));
    h_tempBps2cilut = (unsigned short*)calloc(BPSLUTSIZE, sizeof(unsigned short));
    CalcDimsVec(numThreads, &dBlock, &dGrid);

    CalcBPS2CILutCUDA<<<dimGrid, dimBlocks>>>(numBits, d_tempBps2cilut);

    cudaThreadSynchronize();
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error invoke kernel calcBPSLut\n" << std::endl;
//        MessageBox(NULL, "Error invoke kernel calcBPSLut", "CDUA", MB_ICONEXCLAMATION|MB_OK);
        return cerror;
    }
//	cudaMemcpy(h_tempBps2cilut, d_tempBps2cilut, BPSLUTSIZE*sizeof(unsigned short), cudaMemcpyDeviceToHost);
//	cudaMemcpyToSymbol(bps2cilut, h_tempBps2cilut, BPSLUTSIZE*sizeof(unsigned short));
    cudaThreadSynchronize();
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error copying BPSLut\n" << std::endl;
//        MessageBox(NULL, "Error copying BPSLut", "CDUA", MB_ICONEXCLAMATION|MB_OK);
        return cerror;
    }

    cudaFree(d_tempBps2cilut);
    free(h_tempBps2cilut);

    return 0;
}

//--------------------------------------------------------------------------------------------------
__global__
void BPS2CIMapGpu(unsigned char maxBits, int pitchCiMap, short *ui_d_CiMap)
{
    unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;
    unsigned int b, g, bitmask = 0;
    unsigned char invert = 0;

    g = b = ((short*)((char*)ui_d_CiMap + y * pitchCiMap))[x];
    for (int i = maxBits; i >= 0; i--)
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
    ((short*)((char*)ui_d_CiMap + y * pitchCiMap))[x] = b;
}


template<typename _Tp> __global__
void calcBPSGpu(float f_contThreas, struct cudaPitchedPtr pp_d_images, struct cudaExtent extent, int pitchBPS, short *ui_d_BPS)
{
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
    char* devPtr = (char*)pp_d_images.ptr;
    int pitch = pp_d_images.pitch;
    int slicePitch = pitch * extent.height;
    float threas;
    unsigned short bitplanestack;

    if ((((_Tp*)(devPtr + slicePitch + y * pitch))[x] - ((_Tp*)(devPtr + y * pitch))[x]) > f_contThreas)
    {
        threas = (((_Tp*)((char *)devPtr + y * pitch))[x] + ((_Tp*)((char *)devPtr + slicePitch + y * pitch))[x]) / 2.0;

        bitplanestack = 0;

        for(int imgNr = 2; imgNr < extent.depth; imgNr++)
        {
            if (((_Tp*)(devPtr + slicePitch * imgNr + y * pitch))[x] > threas)
            {
                bitplanestack |= ui_d_Bitshift[imgNr - 2];
            }
        }

//		((unsigned short*)((char*)ui_d_BPS+y*pitchBPS))[x] = bitplanestack;
        ((short*)((char*)ui_d_BPS + y * pitchBPS))[x] = bps2cilut[bitplanestack];
    }
    else
    {
        ((short*)((char*)ui_d_BPS + y * pitchBPS))[x] = INVPHA;
    }
}

extern "C" template<typename _Tp> int CalcCIMap(struct tvArray3D **images, float contThreas, struct tvArray2D **CiMap)
{
    dim3 dimBlocks, dimGrid;
    int ret = 0;
    cudaError_t cerror;
    struct cudaExtent imageExt;
    struct cudaPitchedPtr pp_d_images;
    struct cudaMemcpy3DParms imgMemcpyParms;
    int pitchCiMap;
    short *ui_d_CiMap = NULL;
    unsigned short bitshift[MAXGRAYBITS];

//    if ((images == NULL) || (CiMap == NULL) || (cudaInit == 0))
//    {
//        return EPARAM;
//    }

//    if ((*images == NULL) || ((*images)->sizes[1] != (*CiMap)->sizes[0]) || ((*images)->sizes[2] != (*CiMap)->sizes[1]))
//    {
//        return EPARAM;
//    }

    CalcDims((*images)->sizes[2], (*images)->sizes[1], &dimBlocks, &dimGrid);

    imageExt.width = (*images)->sizes[2] * sizeof(_Tp);
    imageExt.height = (*images)->sizes[1];
    imageExt.depth = (*images)->sizes[0];

    cudaMalloc3D(&pp_d_images, imageExt);
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error malloc images - CUDA cimap\n" << std::endl;
//        MessageBox(NULL, "Error malloc images", "CDUA - CiMap", MB_ICONEXCLAMATION|MB_OK);
        ret = cerror;
        goto end;
    }

    imgMemcpyParms.srcArray = 0;
    imgMemcpyParms.dstArray = 0;
    imgMemcpyParms.srcPos = make_cudaPos(0, 0, 0);
    imgMemcpyParms.dstPtr = pp_d_images;
    imgMemcpyParms.extent = imageExt;
    imgMemcpyParms.kind = cudaMemcpyHostToDevice;
    imgMemcpyParms.srcPtr = make_cudaPitchedPtr((*images)->vals, (*images)->sizes[2] * sizeof(_Tp), (*images)->sizes[2], (*images)->sizes[1]);
    imgMemcpyParms.dstPos = make_cudaPos(0, 0, 0);
    cudaMemcpy3D(&imgMemcpyParms);

    cudaMallocPitch((void**)&ui_d_CiMap, &pitchCiMap, (*images)->sizes[2] * sizeof(short), (*images)->sizes[1]);

    if ((ret = CalcBPS2CILut(uc_numBits)))
    {
        return ret;
    }

    memset(bitshift, 0, MAXGRAYBITS * sizeof(unsigned short));
    for (int g = 0; g < uc_numBits; g++)
    {
        bitshift[g] = 1 << (uc_numBits - g - 1);
    }

    cudaMemcpyToSymbol(ui_d_Bitshift, bitshift, MAXGRAYBITS * sizeof(unsigned short), 0, cudaMemcpyHostToDevice);
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error copy bitshift vector\n" << std::endl;
//        MessageBox(NULL, "Error copy bitshift vector", "CDUA", MB_ICONEXCLAMATION|MB_OK);
        return cerror;
    }

    calcBPSUCGpu<<<dimGrid, dimBlocks>>>(f_contThreas, pp_d_images, imageExt, pitchCiMap, ui_d_CiMap);
    cudaThreadSynchronize();
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error invoke kernel BPS\n" << std::endl;
//        MessageBox(NULL, "Error invoke kernel BPS", "CDUA", MB_ICONEXCLAMATION|MB_OK);
        return cerror;
    }

    cudaMemcpy2D((*CiMap)->vals, (*images)->sizes[2]*sizeof(short), ui_d_CiMap, pitchCiMap, (*images)->sizes[2]*sizeof(short), (*images)->sizes[1], cudaMemcpyDeviceToHost);

end:
    if (ui_d_CiMap)
    {
        cudaFree(ui_d_CiMap);
    }
    if (pp_d_images.ptr)
    {
        cudaFree(pp_d_images.ptr);
    }

    return ret;
}

//--------------------------------------------------------------------------------------------------
template<typename _Tp> __global__
void calcPhaseMap4Gpu(float f_contThreas, _Tp overExp, struct cudaPitchedPtr pp_d_images, struct cudaExtent extent, int pitchPhaseMap, CFPTYPE *f_d_phaseMap, int pitchModulationMap, CFPTYPE *f_d_modulationMap)
{
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
    char* devPtr = (char*)pp_d_images.ptr;
    int pitch = pp_d_images.pitch;
    int slicePitch = pitch * extent.height;
    _Tp max = 0;
    _Tp Int[4];
    CFPTYPE contrast = 0, buf1 = 0, buf2 = 0;

    for (int n = 0; n < 4; n++)
    {
        Int[n] = ((_Tp*)(devPtr + slicePitch * n + y * pitch))[x];
        if (Int[n] > max)
        {
            max = Int[n];
        }
    }

    buf1 = Int[1] - Int[3];
    buf2 = Int[2] - Int[0];
    contrast = sqrt(buf1 * buf1 + buf2 * buf2);

    ((CFPTYPE*)((char*)f_d_modulationMap + pitchModulationMap * y))[x] = contrast;
    if ((contrast>f_contThreas) && (((overExp) && (max < overExp)) || !overExp))
    {
        ((CFPTYPE*)((char*)f_d_phaseMap + pitchPhaseMap * y))[x] = atan2(buf1, buf2);
    }
    else
    {
        ((CFPTYPE*)((char*)f_d_phaseMap + pitchPhaseMap * y))[x] = INVPHA;
    }
}

extern "C" template<typename _Tp> int CalcPhaseMap4(struct tvArray3D **images, float contThreas, _Tp overExp, struct tFloatArray2D **PhaseMap, struct tFloatArray2D **ModulationMap)
{
    dim3 dimBlocks, dimGrid;
    int ret = 0;
    cudaError_t cerror;
    struct cudaExtent imageExt;
    struct cudaPitchedPtr pp_d_images;
    struct cudaMemcpy3DParms imgMemcpyParms;
    int pitchPhaseMap, pitchModulationMap;
    CFPTYPE *f_d_PhaseMap = NULL, *f_d_ModulationMap = NULL;

//    if ((images == NULL) || (PhaseMap == NULL) || (cudaInit == 0))
//    {
//        return EPARAM;
//    }

//    if ((*images == NULL) || ((*images)->sizes[0] != 4)
//        || ((*images)->sizes[1] != (*PhaseMap)->sizes[0])
//        || ((*images)->sizes[2] != (*PhaseMap)->sizes[1]))
//    {
//        return EPARAM;
//    }

    CalcDims((*images)->sizes[2], (*images)->sizes[1], &dimBlocks, &dimGrid);

    imageExt.width = (*images)->sizes[2] * sizeof(_Tp);
    imageExt.height = (*images)->sizes[1];
    imageExt.depth = 4;

    cudaMalloc3D(&pp_d_images, imageExt);
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error malloc images - CUDA phasemap 4\n" << std::endl;
//        MessageBox(NULL, "Error malloc images", "CDUA - Phamap 4", MB_ICONEXCLAMATION|MB_OK);
        ret = cerror;
        goto end;
    }

    imgMemcpyParms.srcArray = 0;
    imgMemcpyParms.dstArray = 0;
    imgMemcpyParms.srcPos = make_cudaPos(0, 0, 0);
    imgMemcpyParms.dstPtr = pp_d_images;
    imgMemcpyParms.extent = imageExt;
    imgMemcpyParms.kind = cudaMemcpyHostToDevice;
    imgMemcpyParms.srcPtr = make_cudaPitchedPtr((*images)->vals, (*images)->sizes[2] * sizeof(_Tp), (*images)->sizes[2], (*images)->sizes[1]);
    imgMemcpyParms.dstPos = make_cudaPos(0, 0, 0);
    cudaMemcpy3D(&imgMemcpyParms);

    cudaMallocPitch((void**)&f_d_PhaseMap, &pitchPhaseMap, (*images)->sizes[2] * sizeof(CFPTYPE), (*images)->sizes[1]);
    cudaMallocPitch((void**)&f_d_ModulationMap, &pitchModulationMap, (*images)->sizes[2] * sizeof(CFPTYPE), (*images)->sizes[1]);

    calcPhaseMap4Gpu<_Tp><<<dimGrid, dimBlocks>>>(f_contThreas, overExp, pp_d_images, imageExt, pitchPhaseMap, f_d_PhaseMap, pitchModulationMap, f_d_ModulationMap);
    cudaThreadSynchronize();
    if ((cerror = cudaGetLastError()))
    {
        ret = cerror;
        goto end;
    }

    cudaMemcpy2D((*ModulationMap)->vals, (*images)->sizes[2] * sizeof(CFPTYPE), f_d_ModulationMap, pitchModulationMap, (*images)->sizes[2]*sizeof(CFPTYPE), (*images)->sizes[1], cudaMemcpyDeviceToHost);
    cudaMemcpy2D((*PhaseMap)->vals, (*images)->sizes[2] * sizeof(CFPTYPE), f_d_PhaseMap, pitchPhaseMap, (*images)->sizes[2]*sizeof(CFPTYPE), (*images)->sizes[1], cudaMemcpyDeviceToHost);

end:
    if (f_d_PhaseMap)
    {
        cudaFree(f_d_PhaseMap);
    }
    if (f_d_ModulationMap)
    {
        cudaFree(f_d_ModulationMap);
    }
    if (pp_d_images.ptr)
    {
        cudaFree(pp_d_images.ptr);
    }

    return ret;
}

//--------------------------------------------------------------------------------------------------
template<typename _Tp> __global__
void calcPhaseMapNGpu(unsigned char numImages, CFPTYPE *f_d_sines, CFPTYPE *f_d_cosines, float f_contThreas, _Tp overExp, struct cudaPitchedPtr pp_d_images, struct cudaExtent extent, int pitchPhaseMap, CFPTYPE *f_d_phaseMap, int pitchModulationMap, CFPTYPE *f_d_modulationMap)
{
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
    char* devPtr = (char*)pp_d_images.ptr;
    int pitch = pp_d_images.pitch;
    int slicePitch = pitch * extent.height;
    _Tp max = 0;
    _Tp Int[MAXPHASHIFT];
    CFPTYPE contrast = 0, buf1 = 0, buf2 = 0;

    for (int n = 0; n < numImages; n++)
    {
        Int[n] = ((_Tp*)(devPtr + slicePitch * n + y * pitch))[x];
        if (Int[n] > max)
        {
            max = Int[n];
        }
    }

//	((CFPTYPE*)((char*)f_d_modulationMap+pitchModulationMap*y))[x] = x;
    for (int n = 0; n < numImages; n++)
    {
         buf1 += Int[n] * f_d_sines[n];
         buf2 += Int[n] * f_d_cosines[n];
    }

    contrast = sqrt(buf1 * buf1 + buf2 * buf2);

    ((CFPTYPE*)((char*)f_d_modulationMap + pitchModulationMap * y))[x] = contrast;
    if ((contrast > f_contThreas) && (((overExp) && (max < overExp)) || !overExp))
    {
        ((CFPTYPE*)((char*)f_d_phaseMap + pitchPhaseMap * y))[x] = atan2(buf1, buf2);
    }
    else
    {
        ((CFPTYPE*)((char*)f_d_phaseMap + pitchPhaseMap * y))[x] = INVPHA;
    }
}

extern "C" template<typename _Tp> int CalcPhaseMapN(struct tvArray3D **images, float contThreas, _Tp overExp, struct tFloatArray2D **PhaseMap, struct tFloatArray2D **ModulationMap)
{
    dim3 dimBlocks, dimGrid;
    int ret = 0;
    cudaError_t cerror;
    struct cudaExtent imageExt;
    struct cudaPitchedPtr pp_d_images;
    struct cudaMemcpy3DParms imgMemcpyParms;
    int pitchPhaseMap, pitchModulationMap;
    CFPTYPE *f_d_PhaseMap = NULL, *f_d_ModulationMap = NULL;
    CFPTYPE *f_d_sines = NULL, *f_d_cosines = NULL, *f_h_sines = NULL, *f_h_cosines = NULL;

//    if ((images == NULL) || (PhaseMap == NULL) || (cudaInit == 0))
//    {
//        return EPARAM;
//    }

//    if ((*images == NULL)
//        || ((*images)->sizes[1] != (*PhaseMap)->sizes[0]) || ((*images)->sizes[2] != (*PhaseMap)->sizes[1]))
//    {
//        return EPARAM;
//    }

    CalcDims((*images)->sizes[2], (*images)->sizes[1], &dimBlocks, &dimGrid);

    imageExt.width = (*images)->sizes[2] * sizeof(_Tp);
    imageExt.height = (*images)->sizes[1];
    imageExt.depth = (*images)->sizes[0];

    cudaMalloc3D(&pp_d_images, imageExt);
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error malloc images - CUDA phasemap N\n" << std::endl;
//        MessageBox(NULL, "Error malloc images", "CDUA - Phamap N", MB_ICONEXCLAMATION|MB_OK);
        ret = cerror;
        goto end;
    }

    imgMemcpyParms.srcArray = 0;
    imgMemcpyParms.dstArray = 0;
    imgMemcpyParms.srcPos = make_cudaPos(0, 0, 0);
    imgMemcpyParms.dstPtr = pp_d_images;
    imgMemcpyParms.extent = imageExt;
    imgMemcpyParms.kind = cudaMemcpyHostToDevice;
    imgMemcpyParms.srcPtr = make_cudaPitchedPtr((*images)->vals, (*images)->sizes[2] * sizeof(_Tp), (*images)->sizes[2], (*images)->sizes[1]);
    imgMemcpyParms.dstPos = make_cudaPos(0, 0, 0);
    cudaMemcpy3D(&imgMemcpyParms);

    cudaMallocPitch((void**)&f_d_PhaseMap, &pitchPhaseMap, (*images)->sizes[2] * sizeof(CFPTYPE), (*images)->sizes[1]);
    cudaMallocPitch((void**)&f_d_ModulationMap, &pitchModulationMap, (*images)->sizes[2] * sizeof(CFPTYPE), (*images)->sizes[1]);

    f_h_sines = (CFPTYPE*)malloc(numImages * sizeof(CFPTYPE));
    f_h_cosines = (CFPTYPE*)malloc(numImages * sizeof(CFPTYPE));
    cudaMalloc(&f_d_sines, numImages * sizeof(CFPTYPE));
    cudaMalloc(&f_d_cosines, numImages * sizeof(CFPTYPE));

    for (int nimg = 0; nimg < numImages; nimg++)
    {
        f_h_sines[nimg] = sin(nimg * CUDA2PI / (CFPTYPE)(numImages));
        f_h_cosines[nimg] = -1.0 * cos(nimg * CUDA2PI / (CFPTYPE)(numImages));
    }

    cudaMemcpy(f_d_sines, f_h_sines, numImages * sizeof(CFPTYPE), cudaMemcpyHostToDevice);
    cudaMemcpy(f_d_cosines, f_h_cosines, numImages * sizeof(CFPTYPE), cudaMemcpyHostToDevice);
    free(f_h_sines);
    free(f_h_cosines);

    calcPhaseMapNGpu<_Tp><<<dimGrid, dimBlocks>>>(numImages, f_d_sines, f_d_cosines, f_contThreas, overExp, pp_d_images, imageExt, pitchPhaseMap, f_d_PhaseMap, pitchModulationMap, f_d_ModulationMap);
    cudaThreadSynchronize();

    cudaFree(f_d_cosines);
    cudaFree(f_d_sines);

    cudaMemcpy2D((*ModulationMap)->vals, (*images)->sizes[2] * sizeof(CFPTYPE), f_d_ModulationMap, pitchModulationMap, (*images)->sizes[2]*sizeof(CFPTYPE), (*images)->sizes[1], cudaMemcpyDeviceToHost);
    cudaMemcpy2D((*PhaseMap)->vals, (*images)->sizes[2] * sizeof(CFPTYPE), f_d_PhaseMap, pitchPhaseMap, (*images)->sizes[2]*sizeof(CFPTYPE), (*images)->sizes[1], cudaMemcpyDeviceToHost);

    if (f_d_PhaseMap)
    {
        cudaFree(f_d_PhaseMap);
    }
    if (f_d_ModulationMap)
    {
        cudaFree(f_d_ModulationMap);
    }
    if (pp_d_images.ptr)
    {
        cudaFree(pp_d_images.ptr);
    }

    return ret;
}

//--------------------------------------------------------------------------------------------------
__global__
void unwrapPhaseGrayGpu(float contThreas, unsigned short maxpha, int pitchCiMap, short *ui_d_CiMap, int pitchRawPhase, CFPTYPE *f_d_RawPhase, int pitchModulationMap, CFPTYPE *f_d_ModulationMap, int pitchPhaseMap, CFPTYPE *f_d_PhaseMap)
{
    unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    short CiMap = ((short*)((char*)ui_d_CiMap + y * pitchCiMap))[x];
    CFPTYPE rawPhase = ((CFPTYPE*)((char*)f_d_RawPhase + y * pitchRawPhase))[x];

    //Phase Unwrapping mit Codeindizes
    //-Pi/2					//pi/2
    if((rawPhase >= -CUDAPI2) && (rawPhase <= CUDAPI2))
    {
        if (((CFPTYPE*)((char*)f_d_ModulationMap + y * pitchModulationMap))[x] > contThreas)
        {
            ((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] = rawPhase + CUDAPI + (CiMap / 2) * CUDA2PI;
        }
        else
        {
            ((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] = INVPHA;
        }
    }
    else if(rawPhase > CUDAPI2)
    {
        if (((CFPTYPE*)((char*)f_d_ModulationMap + y * pitchModulationMap))[x] > contThreas)
        {
            ((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] = rawPhase + CUDAPI + ((CiMap + 1) / 2 - 1) * CUDA2PI;
        }
        else
        {
            ((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] = INVPHA;
        }
    }
    else //(rawPhase < CUDAPI2)
    {
        if (((CFPTYPE*)((char*)f_d_ModulationMap + y * pitchModulationMap))[x] > contThreas)
        {
            ((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] = rawPhase + CUDAPI + ((CiMap + 1) / 2) * CUDA2PI;
        }
        else
        {
            ((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] = INVPHA;
        }
    }

    if ((((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] < 0) || (((CFPTYPE*)((char*)f_d_PhaseMap + y * pitchPhaseMap))[x] > maxpha))
    {
        ((CFPTYPE*)((char*)f_d_PhaseMap+y*pitchPhaseMap))[x] = INVPHA;
    }
}

extern "C" int UnwrapPhaseGray(float contThreas, unsigned short maxpha, struct tShortArray2D **CiMap, struct tFloatArray2D **RawPhase, struct tFloatArray2D **ModulationMap, struct tFloatArray2D **PhaseMap)
{
    dim3 dimBlocks, dimGrid;
    cudaError_t cerror;
    int ret = 0;
    int pitchRawPhase, pitchPhaseMap, pitchModulationMap, pitchCiMap;
    CFPTYPE *f_d_RawPhase = NULL, *f_d_PhaseMap = NULL, *f_d_ModulationMap = NULL;
    short *ui_d_CiMap = NULL;

//    if ((RawPhase == NULL) || (CiMap == NULL) || (PhaseMap == NULL) || (cudaInit == 0))
//    {
//        return EPARAM;
//    }

//    if (((*CiMap)->sizes[0] != (*PhaseMap)->sizes[0]) || ((*CiMap)->sizes[1] != (*PhaseMap)->sizes[1])
//            || ((*RawPhase)->sizes[0] != (*PhaseMap)->sizes[0])
//            || ((*RawPhase)->sizes[1] != (*PhaseMap)->sizes[1]))
//    {
//        return EPARAM;
//    }

    CalcDims((*CiMap)->sizes[1], (*CiMap)->sizes[0], &dimBlocks, &dimGrid);

    cudaMallocPitch((void**)&f_d_RawPhase, &pitchRawPhase, (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[0]);
    cudaMallocPitch((void**)&ui_d_CiMap, &pitchCiMap, (*CiMap)->sizes[1]*sizeof(short), (*CiMap)->sizes[0]);
    cudaMallocPitch((void**)&f_d_ModulationMap, &pitchModulationMap, (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[0]);
    if ((cerror = cudaGetLastError()))
    {
        std::cerr << "Error malloc images - CUDA unwrap\n" << std::endl;
//        MessageBox(NULL, "Error malloc images", "CDUA - Unwrap", MB_ICONEXCLAMATION|MB_OK);
        ret = cerror;
        goto end;
    }

    cudaMemcpy2D(f_d_RawPhase, pitchRawPhase, (*RawPhase)->vals, (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[0], cudaMemcpyHostToDevice);
    cudaMemcpy2D(ui_d_CiMap, pitchCiMap, (*CiMap)->vals, (*CiMap)->sizes[1]*sizeof(short), (*CiMap)->sizes[1]*sizeof(short), (*CiMap)->sizes[0], cudaMemcpyHostToDevice);
    cudaMemcpy2D(f_d_ModulationMap, pitchModulationMap, (*ModulationMap)->vals, (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[0], cudaMemcpyHostToDevice);

    cudaMallocPitch((void**)&f_d_PhaseMap, &pitchPhaseMap, (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[0]);

    unwrapPhaseGrayGpu<<<dimGrid, dimBlocks>>>(contThreas, maxpha, pitchCiMap, ui_d_CiMap, pitchRawPhase, f_d_RawPhase, pitchModulationMap, f_d_ModulationMap, pitchPhaseMap, f_d_PhaseMap);
    cudaThreadSynchronize();
    if ((cerror = cudaGetLastError()))
    {
        ret = cerror;
        goto end;
    }

    cudaMemcpy2D((*PhaseMap)->vals, (*CiMap)->sizes[1]*sizeof(CFPTYPE), f_d_PhaseMap, pitchPhaseMap, (*CiMap)->sizes[1]*sizeof(CFPTYPE), (*CiMap)->sizes[0], cudaMemcpyDeviceToHost);

end:
    if (f_d_RawPhase)
    {
        cudaFree(f_d_RawPhase);
    }
    if (ui_d_CiMap)
    {
        cudaFree(ui_d_CiMap);
    }
    if (f_d_ModulationMap)
    {
        cudaFree(f_d_ModulationMap);
    }
    if (f_d_PhaseMap)
    {
        cudaFree(f_d_PhaseMap);
    }

    return 0;
}

//--------------------------------------------------------------------------------------------------
//extern "C" int CLoadLUT(unsigned short *ui_h_BPS2CITable)
//{
//    cudaError_t cerror;

///*
//    cudaMemcpyToSymbol(ui_d_BPS2CITable, ui_h_BPS2CITable, (1<<(MAXGRAYBITS+1))*sizeof(unsigned short), 0, cudaMemcpyHostToDevice);
//    if ((cerror = cudaGetLastError()))
//    {
//        return cerror;
//    }
//*/
//    return 0;
//}

//--------------------------------------------------------------------------------------------------
extern "C" int InitCudaDevice(int num)
{
    struct cudaDeviceProp prop;
    int numdev, cudadev;
    cudaError_t cerror;

    cudaThreadExit();
    cerror = cudaGetLastError();
    cerror = cudaGetDeviceCount(&numdev);
    if (numdev == 0)
    {
        return -1;
    }

//	cerror = cudaSetDevice(num);
//	cudadev = num;

    for (int r = 0; r < numdev; r++)
    {
        cerror = cudaGetDeviceProperties(&prop, r);
        if (prop.major >= 1)
        {
            cerror = cudaSetDevice(r);
            cudadev = r;
            break;
        }

        if (r == numdev - 1)
        {
            std::cerr << "No CUDA capable device found\n" << std::endl;
//          cout << "No CUDA capable device found!\nAborting!\n";
            return -1;
        }
    }
//	cudaThreadExit();

    return 0;
}

//--------------------------------------------------------------------------------------------------
extern "C" int CalcDimsVec(long length, dim3 *dimBlock, dim3 *dimGrid)
{
    unsigned short maxbsize = 16;
    unsigned short MaxBlocks = 256;
    int devnum = -1;
    struct cudaDeviceProp prop;
    cudaError_t cerror;

//	cudaGetDevice(&devnum);
//	cerror = cudaGetDeviceProperties(&prop, devnum);
//	MaxBlocks = prop.maxThreadsPerBlock;
//	maxbsize = (double)MaxBlocks;
	(*dimBlock).y = MaxBlocks;
/*
        for (int n = maxbsize; n > 0; n--)
	{
            if ((floor((float)length / (float)n) == ((float)length / (float)n)) && (n))
            {
                (*dimBlock).x = n;
                break;
            }
	}
*/
	(*dimBlock).x = 1;

	(*dimGrid).y = (length + (*dimBlock).y - 1) / (*dimBlock).y;
        //> make dims a multiple of 16 for faster calculation (see CUDA doku)
        (*dimGrid).y = ceil((*dimGrid).y / 16.0) * 16;
	(*dimGrid).x = 1;
/*
	char buf[200];
	sprintf(buf, "Len: %d\nMaxB: %d\nBlocks: %d\nGrid: %d", length, maxbsize, (*dimBlock).x, (*dimGrid).x);
	MessageBox(NULL, buf, "", MB_OK);
*/
	return 0;
}

//--------------------------------------------------------------------------------------------------
extern "C" int CalcDims(long width, long height, dim3 *dimBlock, dim3 *dimGrid)
{
    unsigned short maxbsize = 16;
    unsigned short MaxBlocks = 128;
    int devnum;
    struct cudaDeviceProp prop;
    cudaError_t cerror;

//	cudaGetDevice(&devnum);
//	cerror = cudaGetDeviceProperties(&prop, devnum);
//	MaxBlocks = prop.maxThreadsPerBlock;
//	maxbsize = floor(sqrt((double)MaxBlocks));

    for (int n = maxbsize; n > 0; n--)
    {
        if ((floor((CFPTYPE)width / (CFPTYPE)n) == ((CFPTYPE)width / (CFPTYPE)n)) && (n))
        {
            (*dimBlock).x = n;
            break;
        }
    }
    (*dimBlock).x = ceil((*dimBlock).x / 16.0) * 16;
    maxbsize = floor((CFPTYPE)MaxBlocks / (CFPTYPE)(*dimBlock).x);
    for (int n = maxbsize; n > 0; n--)
    {
        if (floor((CFPTYPE)height / (CFPTYPE)n) == ((CFPTYPE)height / (CFPTYPE)n))
        {
            (*dimBlock).y = n;
            break;
        }
    }
    (*dimBlock).y = ceil((*dimBlock).y / 16.0) * 16;
    (*dimGrid).x = (width + (*dimBlock).x - 1) / (*dimBlock).x;
    (*dimGrid).y = (height + (*dimBlock).y - 1) / (*dimBlock).y;

    return 0;
}

//--------------------------------------------------------------------------------------------------
