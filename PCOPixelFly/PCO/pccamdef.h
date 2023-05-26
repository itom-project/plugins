/*function defines from pccamio
 */
#ifndef PCCAMDEF_H
#define PCCAMDEF_H

#ifndef PCCAM_TYPES
#define PCCAM_TYPES
typedef unsigned char  byte;     /* 8-bit  */
typedef unsigned short word;     /* 16-bit */
typedef unsigned int   dword;    /* 32-bit */
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


//@ver1.008 Standby function modes
#define FAN     0x01
#define PELTIER 0x02
#define HEADELE 0x04


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
#define PCC_VBIN(ptr)     (*((dword *)ptr+7)&0x0F) //@ver1.007
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

//buffer removed from list with sw-comand
#define PCC_BUF_STAT_REMOVED(ptr)     ((*((dword *)ptr+0)>>8)&0x01)

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

#endif /* PCCAMDEF_H */
