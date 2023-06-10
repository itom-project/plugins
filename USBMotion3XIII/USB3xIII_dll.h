// Copyright (c) 2010 by Coptonix GmbH. All rights reserved.
//---------------------------------------------------------------------------
#ifndef USB3xIII_DLL_H
#define USB3xIII_DLL_H
//---------------------------------------------------------------------------
//------ Common Constants - NULL ------
//#ifdef    NULL
//#  undef NULL
//#endif
//#ifndef __cplusplus
//#  define NULL ((void *)0)
//#else
//#  define NULL 0
//#endif

//------ Constants ------
#define RET_OK          0x00000000

//------ DLL Functions------



#if defined _WIN64
    //in 64bit, char* returns a 2byte character (respectively).
    //therefore the methods are forwarded to wchar_t. In 64bit it is necessary to set /Zc:wchar_t- compiler flag
    /*Original message (10.04.2013):
        Sehr geehrter Herr Gronle,

        die 64Bit-DLL wurde mit Delphi XE2/64Bit geschrieben.
        Daher hat der Typ Char 2Bytes.

        Mit freundlichen Gruessen,
        Yasar Channaa
    */
    typedef unsigned int (__stdcall* LPINITUSBMC)(wchar_t*);
    typedef wchar_t* (__stdcall* LPGETPRODUCTVERSION)(int);
    typedef wchar_t* (__stdcall* LPGETVENDORNAME)(int);
    typedef wchar_t* (__stdcall* LPGETPRODUCTNAME)(int);
    typedef wchar_t* (__stdcall* LPGETSERIALNUMBER)(int);
    typedef wchar_t* (__stdcall* LPGETI2CSTATUSSTRING)(unsigned short);
    typedef wchar_t* (__stdcall* LPGETERRORSTRING)(unsigned int);
    typedef unsigned int (__stdcall* LPOPENDEVICEBYSERIAL)(wchar_t*);

    #define RETSTRING(fctcall) QString::fromWCharArray((const wchar_t*)fctcall)

#else
    typedef unsigned int (__stdcall* LPINITUSBMC)(char*);
    typedef char* (__stdcall* LPGETPRODUCTVERSION)(int);
    typedef char* (__stdcall* LPGETVENDORNAME)(int);
    typedef char* (__stdcall* LPGETPRODUCTNAME)(int);
    typedef char* (__stdcall* LPGETSERIALNUMBER)(int);
    typedef char* (__stdcall* LPGETI2CSTATUSSTRING)(unsigned short);
    typedef char* (__stdcall* LPGETERRORSTRING)(unsigned int);
    typedef unsigned int (__stdcall* LPOPENDEVICEBYSERIAL)(char*);

    #define RETSTRING(fctcall) QString(fctcall)
#endif

typedef unsigned int (__stdcall* LPENUMDEVICES)(void);
typedef unsigned int (__stdcall* LPOPENDEVICEBYINDEX)(int);
typedef unsigned int (__stdcall* LPCLOSEDEVICE)(void);
typedef int (__stdcall* LPCURRENTDEVICEINDEX)(void);
typedef unsigned int (__stdcall* LPWRITEI2C)(unsigned char, unsigned char*, unsigned short,
                                             unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPREADI2C)(unsigned char, unsigned char*, unsigned short&,
                                            unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPSCANI2C)(unsigned char*, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETSCLHILO)(unsigned short, unsigned short, unsigned int);
typedef unsigned int (__stdcall* LPGETSCLHILO)(unsigned short&, unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPSETSCL)(unsigned int, unsigned int);
typedef unsigned int (__stdcall* LPGETSCL)(unsigned int&, unsigned int);
typedef unsigned int (__stdcall* LPCHKSLVADDR)(unsigned char, unsigned char&, unsigned int);
typedef short (__stdcall* LPVMAXCALC)(short, unsigned int, unsigned char);
typedef short (__stdcall* LPRHZCALC)(short, unsigned int, unsigned char);
typedef unsigned short (__stdcall* LPAMAXCALC)(unsigned short, unsigned int, unsigned char, unsigned char);
typedef short (__stdcall* LPDRHZCALC)(short, unsigned int, unsigned char, unsigned char);
typedef short (__stdcall* LPRHZFULLSTEP)(short, unsigned char);
typedef unsigned short (__stdcall* LPDRHZFULLSTEP)(unsigned short, unsigned char);
typedef void (__stdcall* LPCALCPMULPDIV)(unsigned short, unsigned short, unsigned short,
                                         float, unsigned short&, unsigned short&);
typedef void (__stdcall* LPSETGLOBALPARAM)(unsigned int, unsigned char, unsigned char, float);
typedef unsigned int (__stdcall* LPMAKES24BIT)(unsigned char, unsigned char, unsigned char);
typedef short (__stdcall* LPMAKES12BIT)(unsigned char, unsigned char);

typedef unsigned int (__stdcall* LPSENDTMC428)(unsigned char, unsigned char, unsigned char,
                                               unsigned char, unsigned char, unsigned char&,
                                               unsigned int);
typedef unsigned int (__stdcall* LPREADTMC428)(unsigned char, unsigned char, unsigned char&,
                                               unsigned char&, unsigned char&, unsigned char&,
                                               unsigned int);
typedef unsigned int (__stdcall* LPSETXYZTARGET)(unsigned char, int, int, int,
                                                 unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETXTARGET)(unsigned char, int, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETXACTUAL)(unsigned char, int, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETVMIN)(unsigned char, unsigned short, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETVMAX)(unsigned char, unsigned short, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETVTARGET)(unsigned char, short, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETVACTUAL)(unsigned char, short, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETPMULPDIV)(unsigned char, unsigned char, unsigned char,
                                                unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETAMAX)(unsigned char, unsigned short, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETCOILCURRENT)(unsigned char, unsigned char, unsigned char, unsigned char,
                                                   unsigned short, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETCONFRM)(unsigned char, unsigned char, unsigned char,
                                             unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETMASKFLAG)(unsigned char, unsigned char, unsigned char,
                                                unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETPRDIVUSRS)(unsigned char, unsigned char, unsigned char,
                                                 unsigned char, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETREFTOLERANCE)(unsigned char, unsigned short, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETXLATCHED)(unsigned char, int, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETSMGP)(unsigned char, unsigned char, unsigned char,
                                            unsigned char, unsigned char, unsigned char,
                                            unsigned char, unsigned char, unsigned char,
                                            unsigned char, unsigned char,
                                            unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETDGLOWWORD)(unsigned int, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETDGHIGHWORD)(unsigned int, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETCOVERPOSLEN)(unsigned char, unsigned char, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETCOVERDATAGRAMM)(unsigned int, unsigned char&, unsigned int);


typedef unsigned int (__stdcall* LPGETXVACTUAL)(int&, int&, int&, short&, short&, short&,
                                                unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETXTARGET)(unsigned char, unsigned char&, int&, unsigned int);
typedef unsigned int (__stdcall* LPGETXACTUAL)(unsigned char, unsigned char&, int&, unsigned int);
typedef unsigned int (__stdcall* LPGETVMIN)(unsigned char, unsigned char&, unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPGETVMAX)(unsigned char, unsigned char&, unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPGETVTARGET)(unsigned char, unsigned char&, short&, unsigned int);
typedef unsigned int (__stdcall* LPGETVACTUAL)(unsigned char, unsigned char&, short&, unsigned int);
typedef unsigned int (__stdcall* LPGETAMAX)(unsigned char, unsigned char&, unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPGETAACTUAL)(unsigned char, unsigned char&, short&, unsigned int);
typedef unsigned int (__stdcall* LPGETCOILCURRENT)(unsigned char, unsigned char&, unsigned char&,
                                                   unsigned char&, unsigned char&,
                                                   unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPGETPMULPDIV)(unsigned char, unsigned char&, unsigned char&,
                                                unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETCONFRM)(unsigned char, unsigned char&, unsigned char&,
                                              unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETMASKFLAG)(unsigned char, unsigned char&, unsigned char&,
                                                unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETPRDIVUSRS)(unsigned char, unsigned char&, unsigned char&,
                                                 unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETREFTOLERANCE)(unsigned char, unsigned char&,
                                                    unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPGETXLATCHED)(unsigned char, unsigned char&, int&, unsigned int);
typedef unsigned int (__stdcall* LPGETSMGP)(unsigned char&, unsigned char&, unsigned char&,
                                            unsigned char&, unsigned char&, unsigned char&,
                                            unsigned char&, unsigned char&, unsigned char&,
                                            unsigned char&, unsigned char&, unsigned char&,
                                            unsigned int);
typedef unsigned int (__stdcall* LPGETDGLOWWORD)(unsigned char&, unsigned int&, unsigned int);
typedef unsigned int (__stdcall* LPGETDGHIGHWORD)(unsigned char&, unsigned int&, unsigned int);
typedef unsigned int (__stdcall* LPGETCOVERPOSLEN)(unsigned char&, unsigned char&, unsigned char&,
                                                   unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETCOVERDATAGRAMM)(unsigned char&, unsigned int&, unsigned int);
typedef unsigned int (__stdcall* LPGETPOWERDOWN)(unsigned char&, unsigned int&, unsigned int);
typedef unsigned int (__stdcall* LPGETSWITCH)(unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPINITPOSITION)(unsigned char, int, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSAVEPOSITION)(unsigned char, unsigned int);
typedef unsigned int (__stdcall* LPRESTOREPOSITION)(unsigned char, unsigned int);
typedef unsigned int (__stdcall* LPSETMODE)(unsigned char, unsigned char, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETMICROSTEPS)(unsigned char, unsigned char, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETSWITCHSETTINGS)(unsigned char, unsigned char, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSETLASTMOTOR)(unsigned char, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETMODE)(unsigned char, unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETSWITCHSETTINGS)(unsigned char, unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETLPBIT)(unsigned char, unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETMICROSTEPS)(unsigned char, unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETLASTMOTOR)(unsigned char&, unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGOHOME)(unsigned char, unsigned char, unsigned short,
                                           unsigned short, int, unsigned int);
typedef unsigned int (__stdcall* LPABORTHOMING)(unsigned char, unsigned int);
typedef unsigned int (__stdcall* LPGETHOMINGSTATE)(unsigned char&, unsigned char&,
                                                   unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPSAVEDRIVERTABLETOEEP)(unsigned char, unsigned int);
typedef unsigned int (__stdcall* LPSAVEPARAMTOEEP)(unsigned char, unsigned int);
typedef unsigned int (__stdcall* LPINITPARAMFROMEEP)(unsigned int);
typedef unsigned int (__stdcall* LPINITPARAMTODEFAULT)(unsigned int);
typedef unsigned int (__stdcall* LPENABLEDRIVERCHAIN)(unsigned char, unsigned char, unsigned char,
                                                      unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPENABLEJOYSTICK)(unsigned char&, unsigned int);
typedef unsigned int (__stdcall* LPGETADCVALUES)(unsigned short&, unsigned short&,
                                                 unsigned short&, unsigned int);
typedef unsigned int (__stdcall* LPSETJOYSTICKTOLERANCE)(unsigned short, unsigned short,
                                                         unsigned short, unsigned int);
typedef unsigned int (__stdcall* LPGETJOYSTICKTOLERANCE)(unsigned short&, unsigned short&,
                                                         unsigned short&, unsigned int);

//---------------------------------------------------------------------------------------------
LPINITUSBMC              initusbmc = 0;
LPENUMDEVICES            enumdevices = 0;
LPGETPRODUCTVERSION      getproductversion = 0;
LPGETVENDORNAME          getvendorname = 0;
LPGETPRODUCTNAME         getproductname = 0;
LPGETSERIALNUMBER        getserialnumber = 0;
LPOPENDEVICEBYINDEX      opendevicebyindex = 0;
LPOPENDEVICEBYSERIAL     opendevicebyserial = 0;
LPCLOSEDEVICE            closedevice = 0;
LPCURRENTDEVICEINDEX     currentdeviceindex = 0;
LPGETI2CSTATUSSTRING     geti2cstatusstring = 0;
LPGETERRORSTRING         geterrorstring = 0;
LPWRITEI2C               writei2c = 0;
LPREADI2C                readi2c = 0;
LPSCANI2C                scani2c = 0;
LPSETSCLHILO             setsclhilo = 0;
LPGETSCLHILO             getsclhilo = 0;
LPSETSCL                 setscl = 0;
LPGETSCL                 getscl = 0;
LPCHKSLVADDR             chkslvaddr = 0;
LPVMAXCALC               vmax_calc = 0;
LPRHZCALC                rhz_calc = 0;
LPAMAXCALC               amax_calc = 0;
LPDRHZCALC               drhz_calc = 0;
LPRHZFULLSTEP            rhz_fullstep = 0;
LPDRHZFULLSTEP           drhz_fullstep = 0;
LPCALCPMULPDIV           calcpmulpdiv = 0;
LPSETGLOBALPARAM         setglobalparam = 0;
LPMAKES24BIT             makes24bit = 0;
LPMAKES12BIT             makes12bit = 0;
LPSENDTMC428             sendtmc428 = 0;
LPREADTMC428             readtmc428 = 0;
LPSETXYZTARGET           setxyztarget = 0;
LPSETXTARGET             setxtarget = 0;
LPSETXACTUAL             setxactual = 0;
LPSETVMIN                setvmin = 0;
LPSETVMAX                setvmax = 0;
LPSETVTARGET             setvtarget = 0;
LPSETVACTUAL             setvactual = 0;
LPSETPMULPDIV            setpmulpdiv = 0;
LPSETAMAX                setamax = 0;
LPSETCOILCURRENT         setcoilcurrent = 0;
LPSETCONFRM              setconfrm = 0;
LPSETMASKFLAG            setmaskflag = 0;
LPSETPRDIVUSRS           setprdivusrs = 0;
LPSETREFTOLERANCE        setreftolerance = 0;
LPSETXLATCHED            setxlatched = 0;
LPSETSMGP                setsmgp = 0;
LPSETDGLOWWORD           setdglowword = 0;
LPSETCOVERPOSLEN         setcoverposlen = 0;
LPSETCOVERDATAGRAMM      setcoverdatagramm = 0;
LPGETXVACTUAL            getxvactual = 0;
LPGETXTARGET             getxtarget = 0;
LPGETXACTUAL             getxactual = 0;
LPGETVMIN                getvmin = 0;
LPGETVMAX                getvmax = 0;
LPGETVTARGET             getvtarget = 0;
LPGETVACTUAL             getvactual = 0;
LPGETAMAX                getamax = 0;
LPGETAACTUAL             getaactual = 0;
LPGETCOILCURRENT         getcoilcurrent = 0;
LPGETPMULPDIV            getpmulpdiv = 0;
LPGETCONFRM              getconfrm = 0;
LPGETMASKFLAG            getmaskflag = 0;
LPGETPRDIVUSRS           getprdivusrs = 0;
LPGETREFTOLERANCE        getreftolerance = 0;
LPGETXLATCHED            getxlatched = 0;
LPGETSMGP                getsmgp = 0;
LPGETDGLOWWORD           getdglowword = 0;
LPGETDGHIGHWORD          getdghighword = 0;
LPGETCOVERPOSLEN         getcoverposlen = 0;
LPGETCOVERDATAGRAMM      getcoverdatagramm = 0;
LPGETPOWERDOWN           getpowerdown = 0;
LPGETSWITCH              getswitch = 0;
LPINITPOSITION           initposition = 0;
LPSAVEPOSITION           saveposition = 0;
LPRESTOREPOSITION        restoreposition = 0;
LPSETMODE                setmode = 0;
LPSETMICROSTEPS          setmicrosteps = 0;
LPSETSWITCHSETTINGS      setswitchsettings = 0;
LPSETLASTMOTOR           setlastmotor = 0;
LPGETMODE                getmode = 0;
LPGETSWITCHSETTINGS      getswitchsettings = 0;
LPGETLPBIT               getlpbit = 0;
LPGETMICROSTEPS          getmicrosteps = 0;
LPGETLASTMOTOR           getlastmotor = 0;
LPGOHOME                 gohome = 0;
LPABORTHOMING            aborthoming = 0;
LPGETHOMINGSTATE         gethomingstate = 0;
LPSAVEDRIVERTABLETOEEP   savedrivertabletoeep = 0;
LPSAVEPARAMTOEEP         saveparamtoeep = 0;
LPINITPARAMFROMEEP       initparamfromeep = 0;
LPINITPARAMTODEFAULT     initparamtodefault = 0;
LPENABLEDRIVERCHAIN      enabledriverchain = 0;
LPENABLEJOYSTICK         enablejoystick = 0;
LPGETADCVALUES           getadcvalues = 0;
LPSETJOYSTICKTOLERANCE   setjoysticktolerance = 0;
LPGETJOYSTICKTOLERANCE   getjoysticktolerance = 0;
//
//unsigned int uiLoadLibrary(void);
//void uiFreeLibrary(void);

enum IntFlag
{
    INT_POS_END = 0x01,
    INT_REF_WRONG = 0x02,
    INT_REF_MISS = 0x04,
    INT_STOP = 0x08,
    INT_STOP_LEFT_LOW = 0x10,
    INT_STOP_RIGHT_LOW = 0x20,
    INT_STOP_LEFT_HIGH = 0x40,
    INT_STOP_RIGHT_HIGH = 0x80
};

enum IntMask
{
    MASK_POS_END = 0x01,
    MASK_REF_WRONG = 0x02,
    MASK_REF_MISS = 0x04,
    MASK_STOP = 0x08,
    MASK_STOP_LEFT_LOW= 0x10,
    MASK_STOP_RIGHT_LOW = 0x20,
    MASK_STOP_LEFT_HIGH = 0x40,
    MASK_STOP_RIGHT_HIGH = 0x80
};

enum MCStatusMask
{
    _INT     = 0x80,
    CDGW    = 0x40,
    RS3     = 0x20,
    xEQt3   = 0x10,
    RS2     = 0x08,
    xEQt2   = 0x04,
    RS1     = 0x02,
    xEQt1   = 0x01
};

#define DWTIMEOUT 500

#endif
