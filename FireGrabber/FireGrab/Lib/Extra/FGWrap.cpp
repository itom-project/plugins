/********************************************************************************/
/* Module file to port FGamera class functions to standard C.                  */
/* c. kuehnel, intek, 18.4.2005                                                 */
/********************************************************************************/

#include <fgwrap.h>

static UINT8 PhySpeed=PS_AUTO;

UINT32 FSPEC FGSetInitialPhySpeed(UINT8 Speed)
{
  if(Speed>PS_AUTO)
   return FCE_INPARMS;
   
  PhySpeed=Speed;
  
  return FCE_NOERROR;
}

UINT32 FSPEC FGGetCameraHandle(FGHANDLE *phCamera,UINT32HL *pGuid,void* IsoContext)
{
  CFGCamera *pCamera;
  UINT32    Result;

  *phCamera=NULL;

  pCamera=new CFGCamera;
  if(!pCamera)
   return FCE_NOMEM;

  if(PhySpeed!=PS_AUTO)
   pCamera->SetParameter(FGP_PHYSPEED,(UINT32)PhySpeed);

  Result=pCamera->Connect(pGuid,IsoContext);
  if(Result!=FCE_NOERROR)
   delete pCamera;
  else
   *phCamera=(void*)pCamera;

  return Result;
}

void FSPEC FGPutCameraHandle(FGHANDLE hCamera)
{
  ((CFGCamera*)hCamera)->Disconnect();
  delete (CFGCamera*)hCamera;
}

UINT32 FSPEC FGWriteRegister(FGHANDLE hCamera,UINT32 Address,UINT32 Value)
{
  return ((CFGCamera*)hCamera)->WriteRegister(Address,Value);
}

UINT32 FSPEC FGReadRegister(FGHANDLE hCamera,UINT32 Address,UINT32 *pValue)
{
  return ((CFGCamera*)hCamera)->ReadRegister(Address,pValue);
}

UINT32 FSPEC FGWriteBlock(FGHANDLE hCamera,UINT32 Address,UINT8 *pData,UINT32 Length)
{
  return ((CFGCamera*)hCamera)->WriteBlock(Address,pData,Length);
}

UINT32 FSPEC FGReadBlock(FGHANDLE hCamera,UINT32 Address,UINT8 *pData,UINT32 Length)
{
  return ((CFGCamera*)hCamera)->ReadBlock(Address,pData,Length);
}

UINT32 FSPEC FGSetParameter(FGHANDLE hCamera,UINT16 Which,UINT32 Value)
{
  return ((CFGCamera*)hCamera)->SetParameter(Which,Value);
}

UINT32 FSPEC FGGetParameter(FGHANDLE hCamera,UINT16 Which,UINT32 *pValue)
{
  return ((CFGCamera*)hCamera)->GetParameter(Which,pValue);
}

UINT32 FSPEC FGGetParameterInfo(FGHANDLE hCamera,UINT16 Which,FGPINFO *pInfo)
{
  return ((CFGCamera*)hCamera)->GetParameterInfo(Which,pInfo);
}

UINT32 FSPEC FGOpenCapture(FGHANDLE hCamera)
{
  return ((CFGCamera*)hCamera)->OpenCapture();
}

UINT32 FSPEC FGCloseCapture(FGHANDLE hCamera)
{
  return ((CFGCamera*)hCamera)->CloseCapture();
}

UINT32 FSPEC FGStartDevice(FGHANDLE hCamera)
{
  return ((CFGCamera*)hCamera)->StartDevice();
}

UINT32 FSPEC FGStopDevice(FGHANDLE hCamera)
{
  return ((CFGCamera*)hCamera)->StopDevice();
}

UINT32 FSPEC FGGetFrame(FGHANDLE hCamera,FGFRAME *pFrame,UINT32 TimeoutInMs)
{
  return ((CFGCamera*)hCamera)->GetFrame(pFrame,TimeoutInMs);
}

UINT32 FSPEC FGPutFrame(FGHANDLE hCamera,FGFRAME *pFrame)
{
  return ((CFGCamera*)hCamera)->PutFrame(pFrame);
}

UINT32 FSPEC FGDiscardFrames(FGHANDLE hCamera)
{
  return ((CFGCamera*)hCamera)->DiscardFrames();
}

UINT32 FSPEC FGGetDeviceName(FGHANDLE hCamera,char *pBuf,UINT32 MaxLength)
{
  return ((CFGCamera*)hCamera)->GetDeviceName(pBuf,MaxLength);
}

FGHANDLE FSPEC FGGetContext(FGHANDLE hCamera)
{
  return ((CFGCamera*)hCamera)->GetContext();
}

UINT32 FSPEC FGAssignUserBuffers(FGHANDLE hCamera,UINT32 Cnt,UINT32 Size,
                           void* *ppMemArray)
{
  return ((CFGCamera*)hCamera)->AssignUserBuffers(Cnt,Size,ppMemArray);
}

