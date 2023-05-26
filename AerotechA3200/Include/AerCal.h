/*+++
   Copyright (c) Aerotech Inc., 1996 - 1999

   This header file is used with the Aerotech System libraries.
+++*/

#ifndef __AER_AXISCAL_H__
#define __AER_AXISCAL_H__

#define MAX_CALIBRATION_FILES_PER_AXIS 8

typedef struct tagAER_AXISCAL_ALLOC_PACKET
{
   DWORD  dwTable;
   DWORD  dwSize;
} AER_AXISCAL_ALLOC_PACKET;
typedef AER_AXISCAL_ALLOC_PACKET *PAER_AXISCAL_ALLOC_PACKET;

typedef struct tagAER_AXISCAL2D_ALLOC_PACKET
{
   DWORD    dwTable;
   DWORD    dwNumRows;
   DWORD    dwNumColumns;
}AER_AXISCAL2D_ALLOC_PACKET;
typedef AER_AXISCAL2D_ALLOC_PACKET *PAER_AXISCAL2D_ALLOC_PACKET;

typedef struct tagAER_GAINCAL_ALLOC_PACKET
{
   DWORD  dwTable;
   DWORD  dwSize;
} AER_GAINCAL_ALLOC_PACKET;
typedef AER_GAINCAL_ALLOC_PACKET *PAER_GAINCAL_ALLOC_PACKET;

#ifdef __cplusplus
extern "C" {
#endif

AERERR_CODE AER_DLLENTRY AerAxisCalRemoveAll( HAERCTRL hAerCtrl, AXISINDEX iCorrectedAxis);
AERERR_CODE AER_DLLENTRY AerAxisCalFileDownload( HAERCTRL hAerCtrl, LPCTSTR pszFileName, PDWORD pdwTable);
AERERR_CODE AER_DLLENTRY AerAxisCalAllocateTable( HAERCTRL hAerCtrl, DWORD dwTable,
                                                  DWORD dwSize );
AERERR_CODE AER_DLLENTRY AerAxisCalGetStatusPacket (HAERCTRL hAerCtrl, DWORD dwTable,
                                                    PAER_AXISCAL_STATUS_PACKET pStatus);
AERERR_CODE AER_DLLENTRY AerAxisCalGetStatus (HAERCTRL hAerCtrl, DWORD dwTable, PDWORD pdwSize,
                                              PDWORD pdwStatus,  PAXISINDEX piMasterAxis,
                                              PAXISINDEX piCorrectedAxis );

AERERR_CODE AER_DLLENTRY AerAxisCalFreeTable( HAERCTRL hAerCtrl, DWORD dwTable );

AERERR_CODE AER_DLLENTRY AerAxisCalSetPoint (HAERCTRL hAerCtrl, DWORD dwTable, DWORD wPoint,
                                             DOUBLE dPosition, DOUBLE dCorrection);
AERERR_CODE AER_DLLENTRY AerAxisCalGetPoint (HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwPoint,
                                             PDOUBLE pdPosition, PDOUBLE pdCorrection, PDWORD pdwStatus );

AERERR_CODE AER_DLLENTRY AerAxisCalReset( HAERCTRL hAerCtrl, AXISINDEX iAxis );

AERERR_CODE AER_DLLENTRY AerAxisCalSetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwMode );
AERERR_CODE AER_DLLENTRY AerAxisCalGetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis, PDWORD pdwMode );

AERERR_CODE AER_DLLENTRY AerAxisCalSetAxisCalParameter( HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwParmNumber,
                                                        DOUBLE dParameter );
AERERR_CODE AER_DLLENTRY AerAxisCalGetAxisCalParameter( HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwParmNumber,
                                                        PDOUBLE pdParameter );

AERERR_CODE AER_DLLENTRY AerAxisCalGetUnCalPosition( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                                     PDOUBLE pdPos );

AERERR_CODE AER_DLLENTRY AerAxisCalSet( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwTable,
                                        AXISINDEX iMasterAxis );
AERERR_CODE AER_DLLENTRY AerAxisCalGetItems( HAERCTRL hAerCtrl, AXISINDEX iAxis, PDWORD pdwItems );
AERERR_CODE AER_DLLENTRY AerAxisCalGetPacket( HAERCTRL hAerCtrl, AXISINDEX iAxis,
                                              DWORD dwItem,  PAER_AXISCAL_PACKET pEMap );
AERERR_CODE AER_DLLENTRY AerAxisCalGet( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwItem, PDWORD pdwTable,
                                        PAXISINDEX piMasterAxis );

AERERR_CODE AER_DLLENTRY AerAxisCal2DAllocateTable( HAERCTRL hAerCtrl, DWORD dwTable,
                                                    DWORD dwNumRows, DWORD dwNumColumns );
AERERR_CODE AER_DLLENTRY AerAxisCal2DFreeTable( HAERCTRL hAerCtrl, DWORD dwTable );

AERERR_CODE AER_DLLENTRY AerAxisCal2DGetStatus( HAERCTRL hAerCtrl, DWORD dwTable,
                                                PDWORD pdwAllocated, PDWORD pdwMode );

AERERR_CODE AER_DLLENTRY AerAxisCal2DSetCorrectPosFlag( HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwCorrectPos );
AERERR_CODE AER_DLLENTRY AerAxisCal2DGetCorrectPosFlag( HAERCTRL hAerCtrl, DWORD dwTable, PDWORD pdwCorrectPos );

AERERR_CODE AER_DLLENTRY AerAxisCal2DSetMode( HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwMode );
AERERR_CODE AER_DLLENTRY AerAxisCal2DGetMode( HAERCTRL hAerCtrl, DWORD dwTable, PDWORD pdwMode );

AERERR_CODE AER_DLLENTRY AerAxisCal2DSetPoint( HAERCTRL hAerCtrl, DWORD dwTable,
                                               DWORD dwRow, DWORD dwColumn,
                                               DOUBLE dCorrectionAxis1,
                                               DOUBLE dCorrectionAxis2,
                                               DOUBLE dCorrectionAxis3 );
AERERR_CODE AER_DLLENTRY AerAxisCal2DSetPointMulti( HAERCTRL hAerCtrl,
                                                    DWORD dwTable,
                                                    DWORD dwNumPoints,
                                                    PDWORD pdwRow,
                                                    PDWORD pdwColumn,
                                                    PDOUBLE pdCorrectionAxis1,
                                                    PDOUBLE pdCorrectionAxis2,
                                                    PDOUBLE pdCorrectionAxis3 );

AERERR_CODE AER_DLLENTRY AerAxisCal2DGetPoint( HAERCTRL hAerCtrl, DWORD dwTable,
                                               DWORD dwRow, DWORD dwColumn,
                                               PDOUBLE pdCorrectionAxis1,
                                               PDOUBLE pdCorrectionAxis2,
                                               PDOUBLE pdCorrectionAxis3 );

AERERR_CODE AER_DLLENTRY AerAxisCal2DSetData( HAERCTRL hAerCtrl, DWORD dwTable,
                                              DOUBLE dRowSampDist, DOUBLE dColSampDist,
                                              DOUBLE dRowOffset, DOUBLE dColOffset,
                                              AXISINDEX iRowAxis, AXISINDEX iColumnAxis,
                                              AXISINDEX iOutputAxis1, AXISINDEX iOutputAxis2,
                                              AXISINDEX iOutputAxis3, DWORD dwCorrectAxis1Enable,
                                              DWORD dwCorrectAxis2Enable, DWORD dwCorrectAxis3Enable );
AERERR_CODE AER_DLLENTRY AerAxisCal2DGetData( HAERCTRL hAerCtrl, DWORD dwTable,
                                              PDOUBLE pdRowSampDist, PDOUBLE pdColumnSampDist,
                                              PDOUBLE pdRowOffset, PDOUBLE pdColumnOffset,
                                              PDWORD pdwNumRows, PDWORD pdwNumColumns,
                                              PAXISINDEX piRowAxis, PAXISINDEX piColumnAxis,
                                              PAXISINDEX piOutputAxis1, PAXISINDEX piOutputAxis2,
                                              PAXISINDEX piOutputAxis3, PDWORD pdwCorrectAxis1Enable,
                                              PDWORD pdwCorrectAxis2Enable, PDWORD pdwCorrectAxis3Enable );

AERERR_CODE AER_DLLENTRY AerAxisCal2DFileDownload( HAERCTRL hAerCtrl, LPCTSTR pszFileName, PDWORD pdwTable);

AERERR_CODE AER_DLLENTRY AerGainCalRemoveAll( HAERCTRL hAerCtrl, AXISINDEX iCorrectedAxis);
AERERR_CODE AER_DLLENTRY AerGainCalFileDownload( HAERCTRL hAerCtrl, LPCTSTR pszFileName,
                                                 PDWORD pdwTable);
AERERR_CODE AER_DLLENTRY AerGainCalAllocateTable( HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwSize );
AERERR_CODE AER_DLLENTRY AerGainCalGetStatusPacket( HAERCTRL hAerCtrl, DWORD dwTable, PAER_GAINCAL_STATUS_PACKET pStatus );
AERERR_CODE AER_DLLENTRY AerGainCalGetStatus (HAERCTRL hAerCtrl, DWORD dwTable, PDWORD pdwSize,
                                              PDWORD pdwStatus,  PAXISINDEX piMasterAxis,
                                              PAXISINDEX piCorrectedAxis);

AERERR_CODE AER_DLLENTRY AerGainCalReset( HAERCTRL hAerCtrl, AXISINDEX iAxis );

AERERR_CODE AER_DLLENTRY AerGainCalSetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwMode );
AERERR_CODE AER_DLLENTRY AerGainCalGetMode( HAERCTRL hAerCtrl, AXISINDEX iAxis, PDWORD pdwMode );

AERERR_CODE AER_DLLENTRY AerGainCalSet (HAERCTRL hAerCtrl, AXISINDEX iAxis, DWORD dwTable,
                                        AXISINDEX iMasterAxis);

AERERR_CODE AER_DLLENTRY AerGainCalFreeTable( HAERCTRL hAerCtrl, DWORD dwTable );

AERERR_CODE AER_DLLENTRY AerGainCalSetPoint( HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwPoint,
                                             DOUBLE dPosition, DOUBLE dKpos, DOUBLE dKi,
                                             DOUBLE dKp, DOUBLE dAff );

AERERR_CODE AER_DLLENTRY AerGainCalGetPoint( HAERCTRL hAerCtrl, DWORD dwTable, DWORD dwPoint,
                                             PDOUBLE pdPosition, PDOUBLE pdKpos, PDOUBLE pdKi,
                                             PDOUBLE pdKp, PDOUBLE pdAff, PDWORD pdwStatus );

#ifdef __cplusplus
}
#endif
//
// utilities
#define MAX_WORD_LENGTH 30
#define MAX_FILE_SIZE    0x1312D00
#define MAX_LINE_SIZE    10000   // maimum size of a chunk read from a cal file or a cam file

#define NUM_AXISCAL_UNITS 4
#define NUM_GAINCAL_UNITS 4
#define NUM_AXISCAL_KEYWORDS 16

//{":START", "MASTER", "POSUNIT",
// "CORUNIT", "SAMPLEDIST", "OFFSET",
// "EXPANDCOEFF", "MATERIALTEMP", "NEGPOS",
// "NEGCOR", "HOMEDIRECTION", "HOMEOFFSET",
// "FULLTRAVEL", "NEGCPU", ":END", ":GAIN"};

#define AXISCAL_KEYWORD_INVALID       -1
#define AXISCAL_KEYWORD_START          0
#define AXISCAL_KEYWORD_MASTER         1
#define AXISCAL_KEYWORD_POSUNIT        2
#define AXISCAL_KEYWORD_CORUNIT        3
#define AXISCAL_KEYWORD_SAMPLEDIST     4
#define AXISCAL_KEYWORD_OFFSET         5
#define AXISCAL_KEYWORD_EXPANDCOEFF    6
#define AXISCAL_KEYWORD_MATERIALTEMP   7
#define AXISCAL_KEYWORD_NEGPOS         8
#define AXISCAL_KEYWORD_NEGCOR         9
#define AXISCAL_KEYWORD_HOMEDIRECTION  10
#define AXISCAL_KEYWORD_HOMEOFFSET     11
#define AXISCAL_KEYWORD_FULLTRAVEL     12
#define AXISCAL_KEYWORD_NEGCPU         13
#define AXISCAL_KEYWORD_END            14
#define AXISCAL_KEYWORD_GAIN           15

#define NUM_2D_AXISCAL_KEYWORDS 11

// {":START", "OUTAXIS3", "POSUNIT",
//  "CORUNIT", "OFFSETROW", "OFFSETCOL",
//  "NEGCOR", "INTABLE", "NOCPUSIGN",
//  ":END", ":START2D"};

#define AXISCAL_2D_KEYWORD_START          0
#define AXISCAL_2D_KEYWORD_OUTAXIS3       1
#define AXISCAL_2D_KEYWORD_POSUNIT        2
#define AXISCAL_2D_KEYWORD_CORUNIT        3
#define AXISCAL_2D_KEYWORD_OFFSETROW      4
#define AXISCAL_2D_KEYWORD_OFFSETCOL      5
#define AXISCAL_2D_KEYWORD_NEGCOR         6
#define AXISCAL_2D_KEYWORD_INTABLE        7
#define AXISCAL_2D_KEYWORD_NOCPUSIGN      8
#define AXISCAL_2D_KEYWORD_END            9
#define AXISCAL_2D_KEYWORD_START2D       10

#define NUM_GAINCAL_KEYWORDS 7

// {":GAIN", "MASTER", "POSUNIT",
//  "SAMPLEDIST", "OFFSET", ":END"};

#define GAINCAL_KEYWORD_GAIN              0
#define GAINCAL_KEYWORD_MASTER            1
#define GAINCAL_KEYWORD_POSUNIT           2
#define GAINCAL_KEYWORD_SAMPLEDIST        3
#define GAINCAL_KEYWORD_OFFSET            4
#define GAINCAL_KEYWORD_NEGPOS			  5
#define GAINCAL_KEYWORD_END               6

BOOL aerIsNumberPart(TCHAR cc);
AERERR_CODE aerParmGetCountConversionFactor(HAERCTRL hAerCtrl, DWORD dwCorrectedAxis, DWORD dwUnitType, double* pfdCntsPerInch);
AERERR_CODE aerGetLineByte( HANDLE hFile, LPTSTR szLine );
AERERR_CODE aerGetLineChunk( HANDLE hFile, LPTSTR szLine, LPTSTR pszFileBuffer, PDWORD pdwBufferSize,
                             PDWORD pdwLocation, PDWORD pdwStatus );
AERERR_CODE aerGetLine( HANDLE hFile, LPTSTR szLine, ULONG ulszLineLen, LPTSTR pszFileBuffer, PDWORD pdwBufferSize,
                        PDWORD pdwLocation );

AERERR_CODE aerReadFileWhole( HANDLE hFile, DWORD dwFileSize, LPTSTR pszFileBuffer );
AERERR_CODE aerGetFileSize(LPCTSTR pszFileName, PDWORD pdwSize);

AERERR_CODE aerParse_START_Line (LPTSTR szLine, PAXISINDEX piCorrectedAxis, PAXISINDEX piMasterAxis,
                                 PDWORD pdwPositionUnitType, PDOUBLE pdPositionUnitDivisor,
                                 PDWORD pdwCorrectionUnitType, PDOUBLE pdCorrectionUnitDivisor,
                                 PBOOL pbTableFormat2, PDOUBLE pdSampleDist, PDOUBLE pdOffset,
                                 PDOUBLE pdExpandCoeff, PDOUBLE pdReferenceTemp, PBOOL pbChangePosSign,
                                 PBOOL pbChangeCorSign, PDWORD pdwHomeDirection, PDOUBLE pdHomeOffset,
                                 PDOUBLE pdStageFullTravel, PBOOL pbNegativeCntsPerUnit);

AERERR_CODE aerParse_2D_START_Line( LPTSTR szLine, PDOUBLE pdRowAxis, PDOUBLE pdColumnAxis,
                                    PDOUBLE pdOutputAxis1, PDOUBLE pdOutputAxis2, PDOUBLE pdOutputAxis3,
                                    PDOUBLE pdRowSampleDistance, PDOUBLE pdColumnSampleDistance,
                                    PDWORD pdwNumberOfColumns, PDOUBLE pdOffsetRow, PDOUBLE pdOffsetCol,
                                    PDWORD pdwPositionUnitType, PDOUBLE pdPositionUnitDivisor,
                                    PDWORD pdwCorrectionUnitType, PDOUBLE pdCorrectionUnitDivisor,
                                    PBOOL pbChangeCorSign, PBOOL pbCorrectInTableOnly );

#endif
// __AER_AXISCAL_H__
