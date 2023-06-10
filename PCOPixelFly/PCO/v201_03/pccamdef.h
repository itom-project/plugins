/*function defines from pccamio
 */
#ifndef PCCAMDEF_H
#define PCCAMDEF_H


#ifndef PCO_TYPES
#define PCO_TYPES
typedef unsigned char  byte;     /* 8-bit  */
typedef unsigned short word;     /* 16-bit */
typedef unsigned long  dword;    /* 32-bit */
#endif

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif


//defines for READVERSION
#define MAX_VERSION_LENGTH 40
#define MAX_VERSION_ITEMS 6
#define PLUTO 1
#define CIRCE 2
#define ORION 3
#define HW    4
#define HEAD  5
#define CPLD  6

//define for READEEPROM
#define HEAD_EEPROM 0
#define PROZ_EEPROM 1


//defines for SET_MODE(...)

//defines for mode parameter
#define HW_TRIGGER 0
#define SW_TRIGGER 1

#define MODE_MASK      0xF0

#define ASYNC_SHUTTER  0x10
#define DOUBLE_SHUTTER 0x20
#define VIDEO_MODE     0x30
#define AUTO_EXPOSURE  0x40


//Wide Pixel
#define WIDEPIXEL  8


//@ver1.007
//defines for vbin setting
#define VBIN_1X 0
#define VBIN_2X 1
#define VBIN_4X 2
#define VBIN_420LINES 8         //board must support this feature
                                //not supported in mode DOUBLE_SHUTTER
//@ver1.008
#define VBIN_9X 9               //board must support this feature
                                //not supported in mode DOUBLE_SHUTTER
#define VBIN_9X_LINES 114

//defines for hbin setting
#define HBIN_1X 0
#define HBIN_2X 1

//defines for mode SETORIONINT and GETORIONINT
#define ORION_END_DATA_OUT  1
#define ORION_END_DATA_IN   2
#define ORION_END_COMMAND   3
#define ORION_PRE_DATA_OUT  4
#define ORION_PRE_DATA_IN   5
#define ORION_PRE_COMMAND   6


//@ver1.008 Standby function modes
#define FAN     0x01
#define PELTIER 0x02
#define HEADELE 0x04

//defines for SETDRIVEREVENT mode
#define PCC_HEAD_EVENT    0x0000
#define PCC_EVENT_ENABLE  (0x0000<<16)
#define PCC_EVENT_DISABLE (0x8000<<16)
#define PCC_HEAD_EVENT_ENABLE   PCC_HEAD_EVENT+PCC_EVENT_ENABLE
#define PCC_HEAD_EVENT_DISABLE  PCC_HEAD_EVENT+PCC_EVENT_DISABLE

//defines for pgio
#define PROG_PLUTO_SIZE 4096
#define PROG_CIRCE_SIZE 4096
#define PROG_CIRCE_SIZE_L 8182
#define PROG_ORION_SIZE 8182

#define READ_EEPROM_SIZE  256
#define WRITE_EEPROM_SIZE 128


//defined CCDTYPES
#define ICX074AL     0x00
#define ICX074AK     0x01

#define ICX085AL     0x10
#define ICX085AK     0x11

#define ICX205AL     0x20
#define ICX205AK     0x21

#define ICX249AL     0x30

#define ICX414AL_20  0x08
#define ICX414AK_20  0x09
#define ICX414AL_16  0x0A
#define ICX414AK_16  0x0B

#define ICX205AL_EC  0x40
#define ICX205AK_EC  0x41
#define ICX205AL_E   0x42
#define ICX205AK_E   0x43

#define ICX285AL_C   0x48
#define ICX285AK_C   0x49
#define ICX285AL     0x4A
#define ICX285AK     0x4B

#define ICX274AL     0x50
#define ICX274AK     0x51

#define ICX285AL_E   0x60
#define ICX285AK_E   0x61

#define ICX285AL_F32 0x80
#define ICX285AK_F32 0x81

#define ICX285AL_F40 0x90
#define ICX285AK_F40 0x91

//defines to extract board information with GETBOARDVAL function
#define PCC_VAL_BOARD_INFO    0x00
#define PCC_VAL_BOARD_STATUS  0x01
#define PCC_VAL_CCDXSIZE      0x02
#define PCC_VAL_CCDYSIZE      0x03
#define PCC_VAL_MODE          0x04
#define PCC_VAL_EXPTIME       0x05
#define PCC_VAL_EXPLEVEL      0x06
#define PCC_VAL_BINNING       0x07
#define PCC_VAL_AGAIN         0x08
#define PCC_VAL_BITPIX        0x09
#define PCC_VAL_SHIFT         0x0A
#define PCC_VAL_OFFSET        0x0B
#define PCC_VAL_LASTEXP       0x0C
#define PCC_VAL_EXTMODE       0x0D
#define PCC_VAL_CCDTYPE       0x0E
#define PCC_VAL_LINETIME      0x0F

#define PCC_VAL_TIMEOUT_PROC   0x20
#define PCC_VAL_TIMEOUT_DMA    0x21
#define PCC_VAL_TIMEOUT_HEAD   0x22

#define PCC_VAL_DMACOUNT         0x30
#define PCC_VAL_ERRORCOUNT       0x31
#define PCC_VAL_FIFOCOUNT        0x32
#define PCC_VAL_DMATIMEOUTCOUNT  0x33

#define PCC_VAL_DMABUFFER_ACTUAL 0x34
#define PCC_VAL_DMABUFFER_NEXT   0x35

#define PCC_VAL_FRAMETIME        0x40
#define PCC_VAL_READOUTTIME      0x41
#define PCC_VAL_VBIN             0x42
#define PCC_VAL_HBIN             0x43
#define PCC_VAL_WIDE             0x44


//defines to extract from PCC_VAL_BOARD_INFO
#define PCC_INFO_TYP(info)     (info&0x0FF0)
#define PCC_INFO_NR(info)      (info&0x0F)
#define PCC_INFO_INIT(info)    (info&0x10000000) ? 1 : 0)

//check if board supports these options
#define PCC_INFO_OPT_420L(info)   ((info&0x00001000) ? 1 : 0)
#define PCC_INFO_OPT_SVGA(info)   ((info&0x00002000) ? 1 : 0)
#define PCC_INFO_OPT_HVGA(info)   ((info&0x00004000) ? 1 : 0)
#define PCC_INFO_OPT_IR(info)     ((info&0x00008000) ? 1 : 0)
#define PCC_INFO_OPT_DOUBLE(info) ((info&0x00010000) ? 1 : 0)
#define PCC_INFO_OPT_EXP(info)    ((info&0x00020000) ? 1 : 0)
#define PCC_INFO_OPT_VGA2(info)   ((info&0x00040000) ? 1 : 0)
#define PCC_INFO_OPT_QE(info)     ((info&0x00080000) ? 1 : 0)
#define PCC_INFO_OPT_SMALL(info)  ((info&0x00100000) ? 1 : 0)
#define PCC_INFO_OPT_5US(info)    ((info&0x00200000) ? 1 : 0)
#define PCC_INFO_OPT_F32(info)    ((info&0x00400000) ? 1 : 0)


//defines to extract from PCC_VAL_BOARD_STATUS
#define PCC_STATUS_CAM_RUN(status)  (status&0x01)
#define PCC_STATUS_NO_HEAD(status)  ((status>>27)&0x01)

//defines to extract from PCC_VAL_BINNING
#define PCC_BINNING_HBIN(binning)    ((binning>>7)&0x01)
#define PCC_BINNING_VBIN(binning)    (binning&0x0F)
#define PCC_BINNING_REGW(binning)    ((binning>>4)&0x01)

#define PCC_EXTMODE_DOUBLE(extmode)       ((extmode)&0x01)
#define PCC_EXTMODE_PRISMA(extmode)       ((extmode>>8)&0x0FF)
#define PCC_EXTMODE_LOGLUT(extmode)       ((extmode>>16)&0x01)
#define PCC_EXTMODE_LUTPOSSIBLE(extmode)  ((extmode>>16)&0x02)
#define PCC_EXTMODE_LUTREQUIRED(extmode)  ((extmode>>16)&0x04)

#define PCC_CCDTYPE_COLOR(ccdtype)   ((ccdtype)&0x01)


//defines to extract buffer information with GETBUFFERVAL function
#define PCC_BUFVAL_STATUS     0x00
#define PCC_BUFVAL_EXPTIME    0x01
#define PCC_BUFVAL_OFFSET     0x02
#define PCC_BUFVAL_SIZE_SET   0x03
#define PCC_BUFVAL_SIZE_DONE  0x04
#define PCC_BUFVAL_SIZE_TOTAL 0x05

//defines to extract from BUFVAL_STATUS
//buffer DMA is running or setup
#define PCC_BUF_STATUS_WRITE(status)        (status&0x01)
//buffer DMA is done
#define PCC_BUF_STATUS_WRITE_DONE(status)   ((status>>1)&0x01)
//buffer is in list
#define PCC_BUF_STATUS_QUEUED(status)       ((status>>2)&0x01)
//buffer was cancelled, during DMA
#define PCC_BUF_STATUS_CANCELLED(status)    ((status>>3)&0x01)

//buffer has event enabled
#define PCC_BUF_STATUS_SELECT(status)       ((status>>4)&0x01)
//buffer event is done
#define PCC_BUF_STATUS_SELECT_DONE(status)  ((status>>5)&0x01)
//buffer mapped
#define PCC_BUF_STATUS_MAPPED(status)       ((status>>6)&0x01)
//first transfer done
#define PCC_BUF_STATUS_I4BYTES(status)      ((status>>7)&0x01)

//buffer removed from list with sw-comand
#define PCC_BUF_STATUS_REMOVED(status)      ((status>>8)&0x01)
//buffer conversion in driver
#define PCC_BUF_STATUS_TRANS_BYTE(status)   ((status>>9)&0x01)
#define PCC_BUF_STATUS_OPENCOUNT(status)    ((status>>16)&0xFF)


//buffer errorflags
#define PCC_BUF_STATUS_ERROR(status)         ((status>>12)&0x0F)
#define PCC_BUF_STATUS_BURST_ERROR(status)   ((status>>12)&0x01)
#define PCC_BUF_STATUS_SIZE_ERROR(status)    ((status>>13)&0x01)
#define PCC_BUF_STATUS_PCI_ERROR(status)     ((status>>14)&0x01)
#define PCC_BUF_STATUS_TIMEOUT_ERROR(status) ((status>>15)&0x01)

/*
//these definitions are obsolete and should not be used in new projects
//defines for struct BOARDVAL
//ptr is the pointer to the buffer, which keeps the structure BOARDVAL
#define PCC_BOARDTYP(ptr) (*((dword *)ptr+0)&0x0FF0)
#define PCC_BOARDNR(ptr)  (*((dword *)ptr+0)&0x0F)

//@ver1.007
//check if board is initialized
#define PCC_BOARDINIT(ptr)    ((*((dword *)ptr+0)&0x10000000) ? 1 : 0)

//check if board supports these options
#define PCC_BOARD_420L(ptr)   ((*((dword *)ptr+0)&0x00001000) ? 1 : 0)
#define PCC_BOARD_SVGA(ptr)   ((*((dword *)ptr+0)&0x00002000) ? 1 : 0)
#define PCC_BOARD_HVGA(ptr)   ((*((dword *)ptr+0)&0x00004000) ? 1 : 0)
#define PCC_BOARD_IR(ptr)     ((*((dword *)ptr+0)&0x00008000) ? 1 : 0)
#define PCC_BOARD_DOUBLE(ptr) ((*((dword *)ptr+0)&0x00010000) ? 1 : 0)
#define PCC_BOARD_EXP(ptr)    ((*((dword *)ptr+0)&0x00020000) ? 1 : 0)
//@ver1.008
#define PCC_BOARD_VGA2(ptr)   ((*((dword *)ptr+0)&0x00040000) ? 1 : 0)
#define PCC_BOARD_QE(ptr)     ((*((dword *)ptr+0)&0x00080000) ? 1 : 0)
#define PCC_BOARD_SMALL(ptr)  ((*((dword *)ptr+0)&0x00100000) ? 1 : 0)
#define PCC_BOARD_5US(ptr)    ((*((dword *)ptr+0)&0x00200000) ? 1 : 0)
//@ver1.013
#define PCC_BOARD_F32(ptr)    ((*((dword *)ptr+0)&0x00400000) ? 1 : 0)

//camera status
#define PCC_CAMSTAT(ptr)  (*((dword *)ptr+1))
#define PCC_CAM_RUN(ptr)  (*((dword *)ptr+1)&0x01)
#define PCC_NO_HEAD(ptr) ((*((dword *)ptr+1)>>27)&0x01)

//actual ccdsize
#define PCC_CCDXSIZE(ptr)  *((dword *)ptr+2)
#define PCC_CCDYSIZE(ptr)  *((dword *)ptr+3)

//actual settings
#define PCC_MODE(ptr)      *((dword *)ptr+4)
#define PCC_EXPTIME(ptr)   *((dword *)ptr+5)
#define PCC_EXPLEVEL(ptr)  *((dword *)ptr+6)
#define PCC_HBIN(ptr)    ((*((dword *)ptr+7)>>7)&0x01)
#define PCC_VBIN(ptr)     (*((dword *)ptr+7)&0x0F) //@ver1.012 back to 0x0f
#define PCC_REGW(ptr)    ((*((dword *)ptr+7)>>4)&0x01)
#define PCC_GAIN(ptr)      *((dword *)ptr+8)
#define PCC_BITPIX(ptr)  ((*((dword *)ptr+9)==0) ? 12 : 8)
#define PCC_SHIFT(ptr)   ((*((dword *)ptr+9)==0) ? 0  : (*((dword *)ptr+9)-1))
#define PCC_OFFSET(ptr)    *((dword *)ptr+10)

//exposure time of last image
#define PCC_LASTEXP(ptr)   *((dword *)ptr+11)

//@ver1.007
#define PCC_CCDTYPE(ptr)   *((dword *)ptr+17)
#define PCC_LINETIME(ptr)   *((dword *)ptr+18)


//check if sytem (board + head) supports these options
#define PCC_DOUBLE(ptr)   (*((dword *)ptr+12)&0x01)
#define PCC_PRISMA(ptr)  ((*((dword *)ptr+12)>>8)&0x0FF)
#define PCC_COLOR(ptr)    (*((dword *)ptr+17)&0x01)  //@ver1.008

//@ver1.007
//check for installed CCD
#define PCC_CCDVGA(ptr)  (((*((dword *)ptr+17)&~0x0F)==0x00) ? 1 : 0)
#define PCC_CCDSVGA(ptr) (((*((dword *)ptr+17)&~0x0F)==0x10) ? 1 : 0)
#define PCC_CCDHVGA(ptr) (((*((dword *)ptr+17)&~0x0F)==0x20) ? 1 : 0)
#define PCC_CCDIR(ptr)   (((*((dword *)ptr+17)&~0x0F)==0x30) ? 1 : 0)

//handle opencount
#define PCC_DEVOPENCOUNT(ptr) *((dword *)ptr+20)

//check if buffer is in DMA or waiting for start of DMA
#define PCC_ACTBUFFERIN(ptr)  ((*((dword *)ptr+13)==0) ? FALSE : TRUE)

//check if other buffers are in list
#define PCC_NEXTBUFFERIN(ptr) ((*((dword *)ptr+14)==*((dword *)ptr+15)) ? FALSE : TRUE)


//defines for struct DEVBUF
//ptr is the pointer to the buffer, which keeps the structure DEVBUF
//buffer DMA is running or setup
#define PCC_BUF_STAT_WRITE(ptr)        (*((dword *)ptr+0)&0x01)
//buffer DMA is done
#define PCC_BUF_STAT_WRITE_DONE(ptr)  ((*((dword *)ptr+0)>>1)&0x01)
//buffer is in list
#define PCC_BUF_STAT_QUEUED(ptr)      ((*((dword *)ptr+0)>>2)&0x01)
//buffer was cancelled, during DMA
#define PCC_BUF_STAT_CANCELLED(ptr)   ((*((dword *)ptr+0)>>3)&0x01)

//buffer has event enabled
#define PCC_BUF_STAT_SELECT(ptr)      ((*((dword *)ptr+0)>>4)&0x01)
//buffer event is done
#define PCC_BUF_STAT_SELECT_DONE(ptr) ((*((dword *)ptr+0)>>5)&0x01)
//buffer mapped
#define PCC_BUF_STAT_MAPPED(ptr)      ((*((dword *)ptr+0)>>6)&0x01)
//first transfer done
#define PCC_BUF_STAT_I4BYTES(ptr)     ((*((dword *)ptr+0)>>7)&0x01)

//buffer removed from list with sw-comand
#define PCC_BUF_STAT_REMOVED(ptr)     ((*((dword *)ptr+0)>>8)&0x01)
//buffer conversion in driver
#define PCC_BUF_STAT_TRANS_BYTE(ptr)  ((*((dword *)ptr+0)>>9)&0x01)


//buffer errorflags
#define PCC_BUF_STAT_ERROR(ptr)       ((*((dword *)ptr+0)>>12)&0x0F)
#define PCC_BUF_STAT_BURST_ERROR(ptr) ((*((dword *)ptr+0)>>12)&0x01)
#define PCC_BUF_STAT_SIZE_ERROR(ptr)  ((*((dword *)ptr+0)>>13)&0x01)
#define PCC_BUF_STAT_PCI_ERROR(ptr)   ((*((dword *)ptr+0)>>14)&0x01)
#define PCC_BUF_STAT_TIMEOUT_ERROR(ptr) ((*((dword *)ptr+0)>>15)&0x01)

//buffer errorflags
#define PCC_BUF_OPENCOUNT(ptr) ((*((dword *)ptr+0)>>16)&0xFF)
#define PCC_BUF_MAPCOUNT(ptr)   (*((dword *)ptr+14)>>16)

//exposure time of last done image
#define PCC_BUF_EXPTIME(ptr)      *((dword *)ptr+1)

#define PCC_BUF_ACT_OFFSET(ptr)   *((dword *)ptr+17)
#define PCC_BUF_ACT_SIZE(ptr)     *((dword *)ptr+18)
#define PCC_BUF_ACT_TRANSFER(ptr) *((dword *)ptr+19)

#define PCC_BUF_TOTAL_SIZE(ptr)    (*((dword *)ptr+21) * *((dword *)ptr+22))
*/

//loglevels for interface dll
#define ERROR_M     0x0001
#define INIT_M      0x0002
#define BUFFER_M    0x0004
#define PROCESS_M   0x0008

#define COC_M       0x0010
#define INFO_M      0x0020
#define COMMAND_M   0x0040

#define PCI_M       0x0020

#define TIME_M      0x1000
#define TIME_MD     0x2000

#endif /* PCCAMDEF_H */
