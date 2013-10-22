/******************************************************************************/
/* This header file exports all FGCamera functions as standard C functions    */
/* First parameter of each function is a this pointer to FGamera object.      */
/* c. kuehnel, intek, 18.4.2005                                               */
/******************************************************************************/

#ifndef FGWRAP_H
#define FGWRAP_H

#include <FGCamera.h>

/* Include this header to get defines and constants. The CFGCamera class is   */
/* excluded in standard C projects. We also import the following declarations */
/* from there:                                                                */
/* UINT32 FSPEC FGInitModule(FGINIT *pArg);                                   */
/* void   FSPEC FGExitModule();                                               */
/* UINT32 FSPEC FGGetNodeList(FGNODEINFO *pInfo,UINT32 MaxCnt,                */
/*                            UINT32 *pRealCnt);                              */

typedef void* FGHANDLE;

#ifdef FGWEXPORT
 #undef  FSPEC
 #define FSPEC __declspec(dllexport) __stdcall
#endif

#ifdef __cplusplus
extern "C" {
#endif

UINT32   FSPEC FGSetInitialPhySpeed(UINT8 Speed);

UINT32   FSPEC FGGetCameraHandle(FGHANDLE *phCamera,UINT32HL *pGuid,void* IsoContext);
void     FSPEC FGPutCameraHandle(FGHANDLE hCamera);

UINT32   FSPEC FGWriteRegister(FGHANDLE hCamera,UINT32 Address,UINT32 Value);
UINT32   FSPEC FGReadRegister(FGHANDLE hCamera,UINT32 Address,UINT32 *pValue);

UINT32   FSPEC FGWriteBlock(FGHANDLE hCamera,UINT32 Address,UINT8 *pData,UINT32 Length);
UINT32   FSPEC FGReadBlock(FGHANDLE hCamera,UINT32 Address,UINT8 *pData,UINT32 Length);

UINT32   FSPEC FGSetParameter(FGHANDLE hCamera,UINT16 Which,UINT32 Value);
UINT32   FSPEC FGGetParameter(FGHANDLE hCamera,UINT16 Which,UINT32 *pValue);
UINT32   FSPEC FGGetParameterInfo(FGHANDLE hCamera,UINT16 Which,FGPINFO *pInfo);

UINT32   FSPEC FGOpenCapture(FGHANDLE hCamera);
UINT32   FSPEC FGCloseCapture(FGHANDLE hCamera);

UINT32   FSPEC FGStartDevice(FGHANDLE hCamera);
UINT32   FSPEC FGStopDevice(FGHANDLE hCamera);

UINT32   FSPEC FGGetFrame(FGHANDLE hCamera,FGFRAME *pFrame,UINT32 TimeoutInMs);
UINT32   FSPEC FGPutFrame(FGHANDLE hCamera,FGFRAME *pFrame);
UINT32   FSPEC FGDiscardFrames(FGHANDLE hCamera);

UINT32   FSPEC FGGetDeviceName(FGHANDLE hCamera,char *pBuf,UINT32 MaxLength);
FGHANDLE FSPEC FGGetContext(FGHANDLE hCamera);

UINT32   FSPEC FGAssignUserBuffers(FGHANDLE hCamera,UINT32 Cnt,UINT32 Size,
                                   void* *ppMemArray);

#ifdef __cplusplus
}
#endif

#endif
