////////////////////////////////////////////////////////////////////////////////
// Headerfile for IsoDma stuff.
// intek, C. Kuehnel, 29.1.2008
////////////////////////////////////////////////////////////////////////////////

#ifndef ISODMA_H
#define ISODMA_H

#include <fgcamera.h>

class CFGNodeMgr;
class CFifo;
class CStack;

////////////////////////////////////////////////////////////////////////////////
// Class for a DMA.
////////////////////////////////////////////////////////////////////////////////

class CSPEC CIsoDma
{
private:
  CFGNodeMgr           *m_pNodeMgr;             // Our node manager

  CStack               *m_pPhysMemStack;        // Stack really allocated frames
  CStack               *m_pFrameStack;          // Our stock for frames
  CFifo                *m_pPushFifo;            // Fifo for pushed frames
  
  void*                *m_ppUbMem;              // User buffer memory
  UINT32                m_UbCnt;                // # of user buffers
  UINT32                m_UbSize;               // Size of user buffers 

  void*                 m_hIsoDma;              // ISO DMA from driver
  
public:
  FGISODMAPARMS         m_Parms;                // Our parameters

protected:
  virtual UINT32        AllocateFrameBuffers(); 
  virtual void          FreeFrameBuffers();     

public:
                        CIsoDma(CFGNodeMgr *pNodeMgr);
  virtual               ~CIsoDma();
  
  virtual UINT32        OpenCapture(FGISODMAPARMS *pParms);
  virtual UINT32        CloseCapture();

  virtual UINT32        AssignUserBuffers(UINT32 Cnt,UINT32 Size,void* *ppMemArray);
  virtual UINT32        GetFrame(FGFRAME *pFrame,UINT32 TimeoutInMs);
  virtual UINT32        PutFrame(FGFRAME *pFrame);
  virtual UINT32        DiscardFrames();
  virtual UINT32        Resize(UINT32 PktCnt,UINT32 PktSize);
  
  virtual UINT8         IsOpen()                { return m_hIsoDma?true:false; }
  
  virtual void          RemoveNodeMgr();
};

#endif
