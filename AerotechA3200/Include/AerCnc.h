/*+++

   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_CNC_H__
#define __AER_CNC_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Library functions
*/
AERERR_CODE AER_DLLENTRY AerTaskStatusGetName( HAERCTRL hAerCtrl, DWORD dwBit, LPTSTR pszName );
AERERR_CODE AER_DLLENTRY AerTaskModeGetName( HAERCTRL hAerCtrl, DWORD dwBit, LPTSTR pszName );

AERERR_CODE AER_DLLENTRY AerSysProgramAllocate( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PAER_PROG_HEADER pHeader,
                                             DWORD dwType, DWORD dwQueueSize, DWORD dwQueueRetain );
AERERR_CODE AER_DLLENTRY AerSysProgramFree( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle );
AERERR_CODE AER_DLLENTRY AerSysProgramGetHandle( HAERCTRL hAerCtrl, DWORD dwNum, PAER_PROG_HANDLE pHandle );
AERERR_CODE AER_DLLENTRY AerSysProgramGetInfo( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PAER_PROG_INFO pInfo );
AERERR_CODE AER_DLLENTRY AerSysProgramGetHeader( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PAER_PROG_HEADER pHeader );
AERERR_CODE AER_DLLENTRY AerSysProgramGetNumber( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PDWORD pdw960ProgNum );

AERERR_CODE AER_DLLENTRY AerSysProgramLoadLine( HAERCTRL hAerCtrl, DWORD dwProg, PCODE_PACKET pCode );
AERERR_CODE AER_DLLENTRY AerSysProgramLoadLineArray( HAERCTRL hAerCtrl, DWORD dwProg, DWORD dwNLines, PCODE_PACKET pCode );

AERERR_CODE AER_DLLENTRY AerSysProgramLoadLabel( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PAER_PROG_LABEL_INFO pLabelInfo );

// dwOn_Off = AER_BP_ON (true), AER_BP_OFF (false), AER_BP_TOGGLE (-1)
AERERR_CODE AER_DLLENTRY AerSysProgramSetBreakPoint( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle,
                                                  DWORD dwLineUser, DWORD dwOn_Off);
AERERR_CODE AER_DLLENTRY AerSysProgramGetBreakPoint( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle,
                                                  DWORD dwLineUser, PBOOL pbOn );
AERERR_CODE AER_DLLENTRY AerSysProgramSetFlags( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, DWORD dwFlags);
AERERR_CODE AER_DLLENTRY AerSysProgramGetFlags( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PDWORD dwFlags);
AERERR_CODE AER_DLLENTRY AerSysProgramGetHandlePlus( HAERCTRL hAerCtrl, DWORD dwNum,
                                                     PAER_PROG_HANDLE pHandle, PAER_PROG_STATUS pStatus);


//AERERR_CODE AER_DLLENTRY AerTaskGetPhysAxisFromTaskAxis( HAERCTRL hAerCtrl,
//                                                         TASKINDEX iTask,
//                                                         TASKAXISINDEX iTaskAxis,
//                                                         PPHYSAXISINDEX piPhysAxis );
//AERERR_CODE AER_DLLENTRY AerTaskGetTaskAxisFromPhysAxis( HAERCTRL hAerCtrl,
//                                                         TASKINDEX iTask,
//                                                         PHYSAXISINDEX iPhysAxis,
//                                                         PTASKAXISINDEX piTaskAxis );

AERERR_CODE AER_DLLENTRY AerCannedFunctionGetData( HAERCTRL hAerCtrl, DWORD iIndex, PCANNEDFUNCTION_DATA pData);
AERERR_CODE AER_DLLENTRY AerDataEvaluateMonitor( HAERCTRL hAerCtrl, PMONITOR_DATA pMonitor,
                                                 PDWORD pdwCond );

AERERR_CODE AER_DLLENTRY AerSysProgramInitHeader( PAER_PROG_HEADER pHeader );
AERERR_CODE AER_DLLENTRY AerSysProgramSetNumLabels( PAER_PROG_HEADER pHeader, DWORD dwNumLabels );
AERERR_CODE AER_DLLENTRY AerSysProgramSetNumDoubles( PAER_PROG_HEADER pHeader, DWORD dwNumDoubles );
AERERR_CODE AER_DLLENTRY AerSysProgramSetNumDoublesTemp( PAER_PROG_HEADER pHeader, DWORD dwNumDoublesTemp );
AERERR_CODE AER_DLLENTRY AerSysProgramSetNumStrings( PAER_PROG_HEADER pHeader, DWORD dwNumStrings );
AERERR_CODE AER_DLLENTRY AerSysProgramSetFileName( PAER_PROG_HEADER pHeader,
                                                LPCTSTR pszFile, DWORD dwDate,
                                                DWORD dwTime );
AERERR_CODE AER_DLLENTRY AerSysProgramAddLine( PAER_PROG_HEADER pHeader, PCODE_PACKET pCode );

AERERR_CODE AER_DLLENTRY AerSysProgramGetLine( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, DWORD dwLine960, PCODE_PACKET pCode );
AERERR_CODE AER_DLLENTRY AerSysProgramGetLabel( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, DWORD dwIndex, PAER_PROG_LABEL_INFO pLabelInfo );
AERERR_CODE AER_DLLENTRY AerSysProgramGetMeasurements( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, PAER_MEASUREMENT_SUMMARY pMeasurements );
AERERR_CODE AER_DLLENTRY AerSysProgramGetLineArray( HAERCTRL hAerCtrl, PAER_PROG_HANDLE pHandle, DWORD dwLine960Start, DWORD dwLine960End, PCODE_PACKET pCodes );
/*
Code Packet coding functions

   AerCode...   Code packet functions
   AerData...   Data structure functions

   Structures with a 'Set' function must call the 'Set' function first before any 'Add' functions.
   Structures without a 'Set' function must call the 'InitStruct' function first before any 'Add' functions.
*/
#define InitStruct(ps)  MEMSET(ps,0,sizeof(*ps))

BOOL IsValidPtrCSParmType(DWORD dwType);
BOOL IsValidPtrAxisPointType(DWORD dwType);
BOOL IsValidPtrStringType(DWORD dwType);
BOOL IsValidPtrNumericType(DWORD dwType);
BOOL IsValidPtrDblType(DWORD vstoragetype);
DWORD IsValidPtrParmeter(DWORD dwType);
BOOL IsValidPtrOffsetType(DWORD dwType);
BOOL IsValidPtrNoParmOffsetType(DWORD dwType);
BOOL IsValidPtrParmOffsetType(DWORD vstoragetype);
BOOL IsValidPtrParmOffsetTypeAxis(DWORD vstoragetype);
BOOL IsValidPtrParmOffsetTypeTask(DWORD vstoragetype);
BOOL IsValidPtrCallStackParm(DWORD vstoragetype);
BOOL IsValidPtrAptVar(DWORD vstoragetype);
BOOL IsValidPtrProg(DWORD vstoragetype);
BOOL IsValidPtrNumericVarType(DWORD vstoragetype);
BOOL IsValidPtrAssignable(DWORD vstoragetype);
BOOL IsValidPtrIndexVar(DWORD dwType);

PSZ AER_DLLENTRY szAerDecodeTaskAxisData(AXISINDEX i, CHAR *pszText);

/* CODE_PACKET */
AERERR_CODE AER_DLLENTRY AerCodeSetType(PCODE_PACKET pCode, DWORD dwType);
AERERR_CODE AER_DLLENTRY AerCodeAddLineUser(PCODE_PACKET pCode, DWORD dwLineUser);
AERERR_CODE AER_DLLENTRY AerCodeAddAttribute(PCODE_PACKET pCode, DWORD dwAttr);
PSZ AER_DLLENTRY szAerDecodeLine(PCODE_PACKET pCode, CHAR *pszLine);

/* PTR_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetPtr(PPTR_DATA pPtr, DWORD dwType, DWORD dwNum);
AERERR_CODE AER_DLLENTRY AerDataAddPtrTask(PPTR_DATA pPtr, TASKINDEX iTask);
AERERR_CODE AER_DLLENTRY AerDataAddPtrOffset(PPTR_DATA pPtr, PPTR_DATA pPtrOffset);
AERERR_CODE AER_DLLENTRY AerDataAddPtrPhysAxisIndex(PPTR_DATA pPtr, AXISINDEX iAxis);
AERERR_CODE AER_DLLENTRY AerDataAddPtrTaskAxisIndex(PPTR_DATA pPtr, AXISINDEX iAxis);
AERERR_CODE AER_DLLENTRY AerDataAddPtrStrVarIndex(PPTR_DATA pPtr, DWORD dwVarType, DWORD dwVarIndx);
AERERR_CODE AER_DLLENTRY AerDataAddPtrCSParmIndex(PPTR_DATA pPtr, CSPARMINDEX iCSParm);
PSZ AER_DLLENTRY szAerDecodePtr(PPTR_DATA pPtr, CHAR *pszText);

/* DWORD_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetDWORDLit(PDWORD_DATA pData, DWORD dwValue);
AERERR_CODE AER_DLLENTRY AerDataSetDWORDPtr(PDWORD_DATA pData, PPTR_DATA pPtr);
PSZ AER_DLLENTRY szAerDecodeDWORDData(PDWORD_DATA pData, CHAR *pszText);

/* DOUBLE_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetDoubleLit(PDOUBLE_DATA pData, double fdValue);
AERERR_CODE AER_DLLENTRY AerDataSetDoubleMask(PDOUBLE_DATA pData, MASK_DATA64 mMask);
AERERR_CODE AER_DLLENTRY AerDataSetDoublePtr(PDOUBLE_DATA pData, PPTR_DATA pPtr);
PSZ AER_DLLENTRY szAerDecodeDoubleData(PDOUBLE_DATA pData, CHAR *pszText);

/* AERSTRING32_DATA */
/*
AERERR_CODE AER_DLLENTRY AerDataSetString32Lit(PSTRING32_DATA pData, CHAR *pszValue);
AERERR_CODE AER_DLLENTRY AerDataSetString32Ptr(PSTRING32_DATA pData, PPTR_DATA pPtr);
PSZ AER_DLLENTRY szAerDecodeString32Data(PSTRING32_DATA pData, CHAR *pszText);
*/

/* AERSTRING128_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetString128Lit(PSTRING128_DATA pData, CHAR *pszValue);
AERERR_CODE AER_DLLENTRY AerDataSetString128Ptr(PSTRING128_DATA pData, PPTR_DATA pPtr);
PSZ AER_DLLENTRY szAerDecodeString128Data(PSTRING128_DATA pData, CHAR *pszText);

/* COND_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetCond(PCOND_DATA pCond, PDOUBLE_DATA pData1, PDOUBLE_DATA pData2, DWORD dwType);
PSZ AER_DLLENTRY szAerDecodeCondData(PCOND_DATA pCond, CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerDataEvaluateCondData( HAERCTRL hAerCtrl,
                                                  PCOND_DATA pCond,
                                                  PDWORD pdwCond,
                                                  PDOUBLE pdValue1,
                                                  PDOUBLE pdValue2 );

/* DBL2STR_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetDbl2Str(PDBL2STR_DATA pDblStr, PDOUBLE_DATA pData, DWORD dwCount);
PSZ AER_DLLENTRY szAerDecodeDbl2StrData(PDBL2STR_DATA pDblStr, CHAR *pszText);

/* MATHSTR_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetMathStr(PMATHSTR_DATA pMathStr, DWORD dwType, PDBL2STR_DATA pDblStr, PSTRING128_DATA pString128);
PSZ AER_DLLENTRY szAerDecodeMathStr(PMATHSTR_DATA pMathStr, CHAR *pszText);

/* MATHDBL_DATA */
typedef struct tagTEXT
{
   CHAR  sz[MAX_TEXT_LEN];
} TEXT;
typedef TEXT *PTEXT;
AERERR_CODE AER_DLLENTRY AerDataSetMathDbl(PMATHDBL_DATA pMathDbl, DWORD dwType, PDOUBLE_DATA pDouble);
PSZ AER_DLLENTRY szAerDecodeMathDbl(PMATHDBL_DATA pMathDbl, PTEXT pText, PDWORD pdwIndex);

/* MASK_DATA */
/* InitStruct required */
AERERR_CODE AER_DLLENTRY AerDataAddMask(PMASK_DATA pMask, DWORD dwIndex);
PSZ AER_DLLENTRY szAerDecodeMask(PMASK_DATA pMask, CHAR *pszText);

/* MASK_DATA64 */
/* InitStruct required */
AERERR_CODE AER_DLLENTRY AerDataAddMask64(PMASK_DATA64 pMask, DWORD dwIndex);
AERERR_CODE AER_DLLENTRY AerDataAddMask64Var(PMASK_DATA64 pMask, DWORD dwType, DWORD dwNum, DWORD dwArrayType, DWORD dwArrayNum);
PSZ AER_DLLENTRY szAerDecodeMask64(PMASK_DATA64 pMask, CHAR *pszText);

/* MATHSTR_LIST */
/* InitStruct required */
AERERR_CODE AER_DLLENTRY AerDataAddMathStr(PMATHSTR_LIST pList, PMATHSTR_DATA pMathStr);
PSZ AER_DLLENTRY szAerDecodeMathStrList(PMATHSTR_LIST pList, CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerCodeAddMathStrList(PCODE_PACKET pCode, PMATHSTR_LIST pList);
//AERERR_CODE AER_DLLENTRY AerCodeAddMathStrList2(CODE_ASSIGN_STR* pAssignStr, PMATHSTR_LIST pList);

/* MATHDBL_LIST */
/* InitStruct required */
AERERR_CODE AER_DLLENTRY AerDataAddMathDbl(PMATHDBL_LIST pList, PMATHDBL_DATA pMathDbl);
AERERR_CODE AER_DLLENTRY AerDataAddMathDblD(PMATHDBL_LIST pList, PMATHDBL_DATA pMathDbl);
AERERR_CODE AER_DLLENTRY AerDataAddMathDblS(PMATHDBL_LIST pList, PMATHSTR_DATA pMathStr);
PSZ AER_DLLENTRY szAerDecodeMathDblList(PMATHDBL_LIST pList, CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerCodeAddMathDblList(PCODE_PACKET pCode, PMATHDBL_LIST pList);
AERERR_CODE AER_DLLENTRY AerCodeAddMathDblList2(CODE_ASSIGN_DBL* pAssignDbl, PMATHDBL_LIST pList);

/* AXISDOUBLE_LIST */
/* InitStruct required */
AERERR_CODE AER_DLLENTRY AerDataAddAxisDouble(PAXISDOUBLE_LIST pList, DWORD iIndex, PDOUBLE_DATA const pDouble);
AERERR_CODE AER_DLLENTRY AerDataRemoveAxisDouble(PAXISDOUBLE_LIST pList, DWORD iIndex);
PSZ AER_DLLENTRY szAerDecodeAxisDoubleList(PAXISDOUBLE_LIST pList, CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerCodeAddAxisDoubleList(PCODE_PACKET pCode, PAXISDOUBLE_LIST const pList);
AERERR_CODE AER_DLLENTRY AerDataAddAxisDouble2(PAXISDOUBLE_LIST pDoubleList1, PAXISDOUBLE_LIST const pDoubleList2);

/* AXISPOINT_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetAxisPointLit(PAXISPOINT_DATA pData, PAXISDOUBLE_LIST pList);
AERERR_CODE AER_DLLENTRY AerDataSetAxisPointPtr(PAXISPOINT_DATA pData, PPTR_DATA pPtr);
PSZ AER_DLLENTRY szAerDecodeAxisPointData(PAXISPOINT_DATA pData,CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerCodeAddAxisPointData(PCODE_PACKET pCode, PAXISPOINT_DATA pData);
AERERR_CODE AER_DLLENTRY AerDataSetAxisPointStr(PAXISPOINT_DATA pData, PSTRING128_DATA pStr);

/* CSPARM_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetCSParmLit(PCSPARM_DATA pData, PAXISDOUBLE_LIST pList);
AERERR_CODE AER_DLLENTRY AerDataSetCSParmPtr(PCSPARM_DATA pData, PPTR_DATA pPtr);
PSZ AER_DLLENTRY szAerDecodeCSParmData(PCSPARM_DATA pData,CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerCodeAddCSParmData(PCODE_PACKET pCode, PCSPARM_DATA pData);

/* LINEAR_DATA */
/* InitStruct required */
AERERR_CODE AER_DLLENTRY AerDataAddLinearType(PLINEAR_DATA pData, DWORD dwType);
AERERR_CODE AER_DLLENTRY AerDataAddLinearMode(PLINEAR_DATA pData, DWORD dwMode);
AERERR_CODE AER_DLLENTRY AerDataAddLinearTarget(PLINEAR_DATA pData, PAXISPOINT_DATA pTarget);
PSZ AER_DLLENTRY szAerDecodeLinearData(PLINEAR_DATA pData, CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerCodeAddLinearData(PCODE_PACKET pCode, PLINEAR_DATA pData);

/* CIRCULAR_DATA */
AERERR_CODE AER_DLLENTRY AerDataSetCircularType(PCIRCULAR_DATA pData, BYTE bCCW, BYTE bEllipse, BOOL bInitStruct);
AERERR_CODE AER_DLLENTRY AerDataAddCircularTarget(PCIRCULAR_DATA pData, AXISINDEX iAxis, PDOUBLE_DATA pTarget);
AERERR_CODE AER_DLLENTRY AerDataAddCircularOffset(PCIRCULAR_DATA pData, DWORD iOffset, PDOUBLE_DATA pOffset);
PSZ AER_DLLENTRY szAerDecodeCircularData(PCIRCULAR_DATA pData, DWORD iCoordSystem, CHAR *pszText);
PSZ AER_DLLENTRY szAerDecodeEllipticalData(PCIRCULAR_DATA pData, DWORD iCoordSystem, CHAR *pszText);
AERERR_CODE AER_DLLENTRY AerCodeAddCircularData(PCODE_PACKET pCode, DWORD iCoordSystem, PCIRCULAR_DATA pData);

/* CODE_SPINDLE */
AERERR_CODE AER_DLLENTRY AerCodeAddSpindleData(PCODE_PACKET pCode, DWORD dwSpindleType, SPINDLEINDEX iSpindle);

AERERR_CODE AerSysProgramAddLineAssignSimple(HAERCTRL hAerCtrl, TASKINDEX iTask, DWORD dwAssignType, DOUBLE dValue, PAER_PROG_HEADER pheader, PCODE_PACKET paCNCLine, DWORD dwLineUser);
AERERR_CODE AerSysProgramAddLineG1(HAERCTRL hAerCtrl, TASKINDEX iTask, AXISMASK mAxis, PAIDOUBLE pdDist, BOOL bIntAccel, BOOL bIntDecel, PAER_PROG_HEADER pheader, PCODE_PACKET paCNCLine, DWORD dwLineUser);
AERERR_CODE AerSysProgramAddLineAssignSimpleIO(HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwAssignType, DOUBLE dValue, WORD wwIONum, PCODE_PACKET paCNCLine, DWORD dwLineUser);

AERERR_CODE AerSysProgramAddLineG2G3(HAERCTRL hAerCtrl, TASKINDEX iTask, AXISMASK mAxis, BOOL bG2, PAIDOUBLE pdTarget, PAIDOUBLE pdOffsetIJ, double dRadius, PAER_PROG_HEADER pHeader, PCODE_PACKET paCNCLine, DWORD dwProgNum, DWORD dwLineUser);

#ifdef __cplusplus
}
#endif

#endif
// __AER_CNC_H__
