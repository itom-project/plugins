/* *\file DummyMotor.cpp
* \brief In this file the functions for the classes of the DummyMotor and its Interface are defined
*
*	The Dummymotor is a virtual device to test positining function and to give developers a template for the implementation of actuators and their GUI 
*	This functions are based on the DummyMotor.cpp which was implemented into the ITO M and ITO M++ measurement programm at ITO, university stuttgart.
*
*\sa DummyMotorInterface, DummyMotor, DummyMotor.h
*\author ITO
*\date	Oct2011
*/

#include "USBMotion3XIII.h"
#include "pluginVersion.h"
#include <math.h>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmutex.h>
#include <qwaitcondition.h>

#ifdef __linux__
    #include <unistd.h>
#else
    #include <windows.h>
#endif


#include "USB3xIII_dll.h"


using namespace ito;


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIIIInterface::getAddInInst(ito::AddInBase **addInInst)
{
    ito::RetVal retValue(ito::retOk);

    if (m_InstList.size() <= 0) //the first instance is instantiated
    {
        retValue += loadDLL();
    }

    if (retValue.containsError()) //error while loading DLL -> no instance can be built
    {
        *addInInst = NULL;
    }
    else
    {
        USBMotion3XIII* newInst = new USBMotion3XIII();
        newInst->setBasePlugin(this);
        *addInInst = qobject_cast<ito::AddInBase*>(newInst);

        m_InstList.append(*addInInst);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIIIInterface::closeThisInst(ito::AddInBase **addInInst)
{
    RetVal retValue(retOk);

    if (*addInInst)
    {
        delete ((USBMotion3XIII *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    if (m_InstList.size() <= 0)
    {
        //retValue += unloadDLL(); //the unloading with a windows machine needs a lot of time, therefore we don't do it here
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
USBMotion3XIIIInterface::USBMotion3XIIIInterface(QObject *parent)
{
    m_type = ito::typeActuator;
    setObjectName("USBMotion3XIII");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char* docstring = \
"This plugin is a wrapper for the driver of the USB Motion 3x III card of COPTONIX GmbH (www.coptonix.com). It is able to address \
up to three different 3-phase stepper motors with up to 64 microstep-accuracy. \n\
\n\
The controller is connected to the computer by a USB connection. The wrapper addresses the methods given by the Windows library \
provided by COPTONIX. For more information about the driver see the corresponding website of the manufacturer (The 64bit DLL was directly \
provided by COPTONIX). \n\
\n\
If you change any parameters like speed or acceleration, set the values and then read the values again, since the currently set values \
might slightly differ from the desired values due to rounding uncertainties.";

	m_description = QObject::tr("A motor driver for the USB Motion 3x III card, COPTONIX GmbH, www.coptonix.com");
    m_detaildescription = QObject::tr(docstring);
	m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");     
    
    m_autoLoadPolicy = ito::autoLoadAlways;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_initParamsMand.append(ito::Param("serialNumber", ito::ParamBase::String, "[unknown]", tr("serial number or string of the device to open").toAscii().data()));
    m_initParamsOpt.append(ito::Param("axisSteps1", ito::ParamBase::Int, 0, new ito::IntMeta(0,100000), tr("number of full steps per turn of axis 1, 0: axis not connected [default]").toAscii().data()));
    m_initParamsOpt.append(ito::Param("axisSteps2", ito::ParamBase::Int, 0, new ito::IntMeta(0,100000), tr("number of full steps per turn of axis 2, 0: axis not connected [default]").toAscii().data()));
    m_initParamsOpt.append(ito::Param("axisSteps3", ito::ParamBase::Int, 0, new ito::IntMeta(0,100000), tr("number of full steps per turn of axis 3, 0: axis not connected [default]").toAscii().data()));
	//ito::tParam paramVal = ito::tParam("Number of Axis", ito::ParamBase::Int, 0, 10, 6, "Number of axis for this Motor");
    //m_initParamsOpt.append(paramVal);

}

//----------------------------------------------------------------------------------------------------------------------------------
USBMotion3XIIIInterface::~USBMotion3XIIIInterface()
{
    //unloadDLL();

    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

ito::RetVal USBMotion3XIIIInterface::loadDLL()
{
    ito::RetVal retValue(ito::retOk);
    if (mLib.isLoaded() )
    {
        return ito::retOk;
    }

#if defined Q_OS_WIN64
    //mLib.setFileName(":/library/USB3xIII"); //if you want to load the library from the resource.qrc file
    mLib.setFileName("plugins/USBMotion3XIII/USB3xIII64");
#elif defined Q_OS_WIN32
    mLib.setFileName("plugins/USBMotion3XIII/USB3xIII");
#endif

    if (!mLib.load())
    {
        retValue += ito::RetVal(ito::retError, 1, tr("%1").arg(mLib.errorString()).toAscii().data());
    }
    else
    {
        geterrorstring = (LPGETERRORSTRING) mLib.resolve("geterrorstring");
        initusbmc = (LPINITUSBMC) mLib.resolve("initusbmc");
        enumdevices = (LPENUMDEVICES) mLib.resolve("enumdevices");
        getproductversion  = (LPGETPRODUCTVERSION) mLib.resolve("getproductversion");
        getvendorname = (LPGETVENDORNAME) mLib.resolve("getvendorname");
        getproductname = (LPGETPRODUCTNAME) mLib.resolve("getproductname");
        getserialnumber = (LPGETSERIALNUMBER) mLib.resolve("getserialnumber");
        opendevicebyindex = (LPOPENDEVICEBYINDEX) mLib.resolve("opendevicebyindex");
        opendevicebyserial = (LPOPENDEVICEBYSERIAL) mLib.resolve("opendevicebyserial");
        closedevice = (LPCLOSEDEVICE) mLib.resolve("closedevice");
        currentdeviceindex = (LPCURRENTDEVICEINDEX) mLib.resolve("currentdeviceindex");
        geti2cstatusstring = (LPGETI2CSTATUSSTRING) mLib.resolve("geti2cstatusstring");
        geterrorstring = (LPGETERRORSTRING) mLib.resolve("geterrorstring");
        writei2c = (LPWRITEI2C) mLib.resolve("writei2c");
        readi2c = (LPREADI2C) mLib.resolve("readi2c");
        scani2c = (LPSCANI2C) mLib.resolve("scani2c");
        setsclhilo = (LPSETSCLHILO) mLib.resolve("setsclhilo");
        getsclhilo = (LPGETSCLHILO) mLib.resolve("getsclhilo");
        setscl = (LPSETSCL) mLib.resolve("setsc");
        getscl = (LPGETSCL) mLib.resolve("getsc");
        chkslvaddr = (LPCHKSLVADDR) mLib.resolve("chkslvaddr");
        vmax_calc = (LPVMAXCALC) mLib.resolve("vmax_calc");
        rhz_calc = (LPRHZCALC) mLib.resolve("rhz_calc");
        amax_calc = (LPAMAXCALC) mLib.resolve("amax_calc");
        drhz_calc = (LPDRHZCALC) mLib.resolve("drhz_calc");
        rhz_fullstep = (LPRHZFULLSTEP) mLib.resolve("rhz_fullstep");
        drhz_fullstep = (LPDRHZFULLSTEP) mLib.resolve("drhz_fullstep");
        calcpmulpdiv = (LPCALCPMULPDIV) mLib.resolve("calcpmulpdiv");
        setglobalparam = (LPSETGLOBALPARAM) mLib.resolve("setglobalparam");
        makes24bit = (LPMAKES24BIT) mLib.resolve("makes24bit");
        makes12bit = (LPMAKES12BIT) mLib.resolve("makes12bit");
        sendtmc428 = (LPSENDTMC428) mLib.resolve("sendtmc428");
        readtmc428 = (LPREADTMC428) mLib.resolve("readtmc428");
        setxyztarget = (LPSETXYZTARGET) mLib.resolve("setxyztarget");
        setxtarget = (LPSETXTARGET) mLib.resolve("setxtarget");
        setxactual = (LPSETXACTUAL) mLib.resolve("setxactual");
        setvmin = (LPSETVMIN) mLib.resolve("setvmin");
        setvmax = (LPSETVMAX) mLib.resolve("setvmax");
        setvtarget = (LPSETVTARGET) mLib.resolve("setvtarget");
        setvactual = (LPSETVACTUAL) mLib.resolve("setvactual");
        setpmulpdiv = (LPSETPMULPDIV) mLib.resolve("setpmulpdiv");
        setamax = (LPSETAMAX) mLib.resolve("setamax");
        setcoilcurrent = (LPSETCOILCURRENT) mLib.resolve("setcoilcurrent");
        setconfrm = (LPSETCONFRM) mLib.resolve("setconfrm");
        setmaskflag = (LPSETMASKFLAG) mLib.resolve("setmaskflag");
        setprdivusrs = (LPSETPRDIVUSRS) mLib.resolve("setprdivusrs");
        setreftolerance = (LPSETREFTOLERANCE) mLib.resolve("setreftolerance");
        setxlatched = (LPSETXLATCHED) mLib.resolve("setxlatched");
        setsmgp = (LPSETSMGP) mLib.resolve("setsmgp");
        setdglowword = (LPSETDGLOWWORD) mLib.resolve("setdglowword");
        setcoverposlen = (LPSETCOVERPOSLEN) mLib.resolve("setcoverposlen");
        setcoverdatagramm = (LPSETCOVERDATAGRAMM) mLib.resolve("setcoverdatagramm");
        getxvactual = (LPGETXVACTUAL) mLib.resolve("getxvactual");
        getxtarget = (LPGETXTARGET) mLib.resolve("getxtarget");
        getxactual = (LPGETXACTUAL) mLib.resolve("getxactual");
        getvmin = (LPGETVMIN) mLib.resolve("getvmin");
        getvmax = (LPGETVMAX) mLib.resolve("getvmax");
        getvtarget = (LPGETVTARGET) mLib.resolve("getvtarget");
        getvactual = (LPGETVACTUAL) mLib.resolve("getvactual");
        getamax = (LPGETAMAX) mLib.resolve("getamax");
        getaactual = (LPGETAACTUAL) mLib.resolve("getaactual");
        getcoilcurrent = (LPGETCOILCURRENT) mLib.resolve("getcoilcurrent");
        getpmulpdiv = (LPGETPMULPDIV) mLib.resolve("getpmulpdiv");
        getconfrm = (LPGETCONFRM) mLib.resolve("getconfrm");
        getmaskflag = (LPGETMASKFLAG) mLib.resolve("getmaskflag");
        getprdivusrs = (LPGETPRDIVUSRS) mLib.resolve("getprdivusrs");
        getreftolerance = (LPGETREFTOLERANCE) mLib.resolve("getreftolerance");
        getxlatched = (LPGETXLATCHED) mLib.resolve("getxlatched");
        getsmgp = (LPGETSMGP) mLib.resolve("getsmgp");
        getdglowword = (LPGETDGLOWWORD) mLib.resolve("getdglowword");
        getdghighword = (LPGETDGHIGHWORD) mLib.resolve("getdghighword");
        getcoverposlen = (LPGETCOVERPOSLEN) mLib.resolve("getcoverposlen");
        getcoverdatagramm = (LPGETCOVERDATAGRAMM) mLib.resolve("getcoverdatagramm");
        getpowerdown = (LPGETPOWERDOWN) mLib.resolve("getpowerdown");
        getswitch = (LPGETSWITCH) mLib.resolve("getswitch");
        initposition = (LPINITPOSITION) mLib.resolve("initposition");
        saveposition = (LPSAVEPOSITION) mLib.resolve("saveposition");
        restoreposition = (LPRESTOREPOSITION) mLib.resolve("restoreposition");
        setmode = (LPSETMODE) mLib.resolve("setmode");
        setmicrosteps = (LPSETMICROSTEPS) mLib.resolve("setmicrosteps");
        setswitchsettings = (LPSETSWITCHSETTINGS) mLib.resolve("setswitchsettings");
        setlastmotor = (LPSETLASTMOTOR) mLib.resolve("setlastmotor");
        getmode = (LPGETMODE) mLib.resolve("getmode");
        getswitchsettings = (LPGETSWITCHSETTINGS) mLib.resolve("getswitchsettings");
        getlpbit = (LPGETLPBIT) mLib.resolve("getlpbit");
        getmicrosteps = (LPGETMICROSTEPS) mLib.resolve("getmicrosteps");
        getlastmotor = (LPGETLASTMOTOR) mLib.resolve("getlastmotor");
        gohome = (LPGOHOME) mLib.resolve("gohome");
        aborthoming = (LPABORTHOMING) mLib.resolve("aborthoming");
        gethomingstate = (LPGETHOMINGSTATE) mLib.resolve("gethomingstate");
        savedrivertabletoeep = (LPSAVEDRIVERTABLETOEEP) mLib.resolve("savedrivertabletoeep");
        saveparamtoeep = (LPSAVEPARAMTOEEP) mLib.resolve("saveparamtoeep");
        initparamfromeep = (LPINITPARAMFROMEEP) mLib.resolve("initparamfromeep");
        initparamtodefault = (LPINITPARAMTODEFAULT) mLib.resolve("initparamtodefault");
        enabledriverchain = (LPENABLEDRIVERCHAIN) mLib.resolve("enabledriverchain");
        enablejoystick = (LPENABLEJOYSTICK) mLib.resolve("enablejoystick");
        getadcvalues = (LPGETADCVALUES) mLib.resolve("getadcvalues");
        setjoysticktolerance = (LPSETJOYSTICKTOLERANCE) mLib.resolve("setjoysticktolerance");
        getjoysticktolerance = (LPGETJOYSTICKTOLERANCE) mLib.resolve("getjoysticktolerance");

        //check (some) for success:
        if (!( geterrorstring && initusbmc && initposition && gohome && getjoysticktolerance))
        {
            retValue += ito::RetVal(ito::retError, 1, tr("error while loading DLL 'USB3xIII.dll': %1").arg(mLib.errorString()).toAscii().data());
        }
    }
    return retValue;
}

ito::RetVal USBMotion3XIIIInterface::unloadDLL()
{
    if (mLib.isLoaded())
    {
        if (!mLib.unload())
        {
            return ito::RetVal(ito::retWarning, 2, tr("DLL could not be unloaded").toAscii().data());
        }

        initusbmc = 0; enumdevices = 0; getproductversion = 0; getvendorname = 0;
        getproductname = 0; getserialnumber = 0; opendevicebyindex = 0; opendevicebyserial = 0;
        closedevice = 0; currentdeviceindex = 0; geti2cstatusstring = 0; geterrorstring = 0;
        writei2c = 0; readi2c = 0; scani2c = 0; setsclhilo = 0; getsclhilo = 0; setscl = 0; 
        getscl = 0; chkslvaddr = 0; vmax_calc = 0; rhz_calc = 0; amax_calc = 0; drhz_calc = 0; 
        rhz_fullstep = 0; drhz_fullstep = 0; calcpmulpdiv = 0; setglobalparam = 0; 
        makes24bit = 0; makes12bit = 0; sendtmc428 = 0; readtmc428 = 0; 
        setxyztarget = 0; setxtarget = 0; setxactual = 0; setvmin = 0; setvmax = 0; 
        setvtarget = 0; setvactual = 0; setpmulpdiv = 0; setamax = 0; setcoilcurrent = 0; 
        setconfrm = 0; setmaskflag = 0; setprdivusrs = 0; setreftolerance = 0; setxlatched = 0; 
        setsmgp = 0; setdglowword = 0; setcoverposlen = 0; setcoverdatagramm = 0; 
        getxvactual = 0; getxtarget = 0; getxactual = 0; getvmin = 0; 
        getvmax = 0; getvtarget = 0; getvactual = 0; getamax = 0; getaactual = 0; 
        getcoilcurrent = 0; getpmulpdiv = 0; getconfrm = 0; getmaskflag = 0; getprdivusrs = 0; 
        getreftolerance = 0;  getxlatched = 0; getsmgp = 0; getdglowword = 0; getdghighword = 0; 
        getcoverposlen = 0; getcoverdatagramm = 0; getpowerdown = 0; getswitch = 0; 
        initposition = 0; saveposition = 0; restoreposition = 0; setmode = 0; 
        setmicrosteps = 0; setswitchsettings = 0; setlastmotor = 0; getmode = 0; 
        getswitchsettings = 0; getlpbit = 0; getmicrosteps = 0; getlastmotor = 0; 
        gohome = 0; aborthoming = 0; gethomingstate = 0; savedrivertabletoeep = 0; 
        saveparamtoeep = 0; initparamfromeep = 0; initparamtodefault = 0; enabledriverchain = 0; 
        enablejoystick = 0; getadcvalues = 0; setjoysticktolerance = 0; getjoysticktolerance = 0; 
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(USBMotion3XIIIInterface, USBMotion3XIIIInterface)


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal USBMotion3XIII::showConfDialog(void)
{
    int enabled;
    int microSteps;
    double vMin, vMax, aMax;
    double coilThreshold, coilHigh, coilLow, coilRest;
    RetVal retValue(retOk);

	DialogUSBMotion3XIII *confDialog = new DialogUSBMotion3XIII( getID() );	// Create dialog
    confDialog->setVals(&m_params);	// Set up dialog parameters
    if (confDialog->exec())	// Is dialog is endet with exec and not with cancel
    {
        //setParam("async",(double)confDialog->getRunMode(), NULL);
        m_params["async"].setVal<int>( (int)confDialog->getRunMode() );
        m_async = confDialog->getRunMode();

        for (int i=1;i<=3;i++)
        {
            confDialog->getAxisValues(i,enabled,microSteps,vMin,vMax,aMax,coilThreshold,coilHigh,coilLow,coilRest);
            retValue += setMicroSteps(i,microSteps); //this must be first, since it influences the rest
            retValue += setAcceleration(i,aMax);
            retValue += setCoilCurrents(i,1 | 2 | 4 | 8,coilHigh,coilLow,coilRest,coilThreshold);
            retValue += setEnabled(i,enabled);
            retValue += setSpeed(i,1 | 2, vMin, vMax);
        }

    }	
    delete confDialog;	// destroy dialog
    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
USBMotion3XIII::USBMotion3XIII() : AddInActuator(), m_curDeviceIndex(-1), m_timerId(0), m_timerInterval(0), USBMotion3XIIIWid(NULL)
{
    Q_FLAGS(IntFlag IntFlags);
    Q_FLAGS(MaskFlag MaskFlags);
    Q_FLAGS(MCStatusMask MCStatusMasks);

    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");	// To enable the programm to transmit parameters via signals - slot connections
    qRegisterMetaType<QVector<bool> >("QVector<bool>");
    qRegisterMetaType<QVector<double> >("QVector<double>");

    //ito::tParam ;	// Set up the parameter list
    m_params.insert( "name", Param("name", ParamBase::String | ParamBase::Readonly, "USBMotion3XIII", NULL));
    m_params.insert( "connected", Param("connected", ParamBase::Int | ParamBase::Readonly, 0, 1, 0, tr("indicates whether motor driver is connected (1) or not (0)").toAscii().data())); //read-only
    m_params.insert( "serialNumber", Param("serialNumber", ParamBase::String | ParamBase::Readonly, "", tr("serial number for this motor driver").toAscii().data())); //read-only
    m_params.insert( "productVersion", Param("productVersion", ParamBase::String | ParamBase::Readonly, "", tr("product version for this motor driver").toAscii().data())); //read-only
    m_params.insert( "vendorName", Param("vendorName", ParamBase::String | ParamBase::Readonly, "", tr("vendor name for this motor driver").toAscii().data())); //read-only
    m_params.insert( "productName", Param("productName", ParamBase::String | ParamBase::Readonly, "", tr("product name for this motor driver").toAscii().data())); //read-only
    m_params.insert( "async", Param("async", ParamBase::Int, 0, 1, 0, tr("asynchronous move (1), synchronous (0) [default]").toAscii().data()));
    m_async = m_params["async"].getVal<int>();

    m_params.insert( "axisSteps1", Param("axisSteps1", ParamBase::Int | ParamBase::Readonly, 0, 100000, 200, tr("number of full steps per turn of motor 1, 0: motor not connected").toAscii().data())); //read-only
    m_params.insert( "vMin1", Param("vMin1", ParamBase::Double, 0.0, 10000.0, 90.0, tr("minimal speed in degree per second of motor 1").toAscii().data()));
    m_params.insert( "vMax1", Param("vMax1", ParamBase::Double, 0.0, 10000.0, 90.0, tr("maximal speed in degree per second of motor 1").toAscii().data()));
    m_params.insert( "aMax1", Param("aMax1", ParamBase::Double, 0.0, 10000.0, 5.0, tr("maximal acceleration in degree/s^2 of motor 1").toAscii().data()));
    m_params.insert( "coilCurrentHigh1", Param("coilCurrentHigh1", ParamBase::Double, 12.5, 100.0, 50.0, tr("coil current if acceleration is higher than coilCurrentThreshold1 [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentLow1", Param("coilCurrentLow1", ParamBase::Double, 12.5, 100.0, 25.0, tr("coil current if acceleration is lower than coilCurrentThreshold1 [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentRest1", Param("coilCurrentRest1", ParamBase::Double, 12.5, 100.0, 12.5, tr("coil current if motor 1 is in rest [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentThreshold1", Param("coilCurrentThreshold1", ParamBase::Double, 0.0, 10000.0, 5.0, tr("threshold acceleration for distinction between coilCurrentHigh and coilCurrentLow").toAscii().data()));
    m_params.insert( "axisEnabled1", Param("axisEnabled1", ParamBase::Int, 0, 1, 1, tr("determine if motor 1 is enabled (1) or disabled (0). If disabled, this motor is manually moveable").toAscii().data()));
    m_params.insert( "microSteps1", Param("microSteps1", ParamBase::Int, 1, 64, 1, tr("micro steps for motor 1 [1,2,4,8,16,32,64]").toAscii().data()));

    m_params.insert( "axisSteps2", Param("axisSteps2", ParamBase::Int | ParamBase::Readonly, 0, 100000, 200, tr("number of full steps per turn of motor 2, 0: motor not connected").toAscii().data())); //read-only
    m_params.insert( "vMin2", Param("vMin2", ParamBase::Double, 0.0, 10000.0, 90.0, tr("minimal speed in degree per second of motor 2").toAscii().data()));
    m_params.insert( "vMax2", Param("vMax2", ParamBase::Double, 0.0, 10000.0, 90.0, tr("maximal speed in degree per second of motor 2").toAscii().data()));
    m_params.insert( "aMax2", Param("aMax2", ParamBase::Double, 0.0, 10000.0, 5.0, tr("maximal acceleration in degree/s^2 of motor 2").toAscii().data()));
    m_params.insert( "coilCurrentHigh2", Param("coilCurrentHigh2", ParamBase::Double, 12.5, 100.0, 50.0, tr("coil current if acceleration is higher than coilCurrentThreshold2 [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentLow2", Param("coilCurrentLow2", ParamBase::Double, 12.5, 100.0, 25.0, tr("coil current if acceleration is lower than coilCurrentThreshold2 [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentRest2", Param("coilCurrentRest2", ParamBase::Double, 12.5, 100.0, 12.5, tr("coil current if motor 2 is in rest [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentThreshold2", Param("coilCurrentThreshold2", ParamBase::Double, 0.0, 10000.0, 5.0, tr("threshold acceleration for distinction between coilCurrentHigh and coilCurrentLow").toAscii().data()));
    m_params.insert( "axisEnabled2", Param("axisEnabled2", ParamBase::Int, 0, 1, 1, tr("determine if motor 2 is enabled (1) or disabled (0). If disabled, this motor is manually moveable").toAscii().data()));
    m_params.insert( "microSteps2", Param("microSteps2", ParamBase::Int, 1, 64, 1, tr("micro steps for motor 2 [1,2,4,8,16,32,64]").toAscii().data()));

    m_params.insert( "axisSteps3", Param("axisSteps3", ParamBase::Int | ParamBase::Readonly, 0, 100000, 200, tr("number of full steps per turn of motor 3, 0: motor not connected").toAscii().data())); //read-only
    m_params.insert( "vMin3", Param("vMin3", ParamBase::Double, 0.0, 10000.0, 90.0, tr("minimal speed in degree per second of motor 3").toAscii().data()));
    m_params.insert( "vMax3", Param("vMax3", ParamBase::Double, 0.0, 10000.0, 90.0, tr("maximal speed in degree per second of motor 3").toAscii().data()));
    m_params.insert( "aMax3", Param("aMax3", ParamBase::Double, 0.0, 10000.0, 5.0, tr("maximal acceleration in degree/s^2 of motor 3").toAscii().data()));
    m_params.insert( "coilCurrentHigh3", Param("coilCurrentHigh3", ParamBase::Double, 12.5, 100.0, 50.0, tr("coil current if acceleration is higher than coilCurrentThreshold3 [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentLow3", Param("coilCurrentLow3", ParamBase::Double, 12.5, 100.0, 25.0, tr("coil current if acceleration is lower than coilCurrentThreshold3 [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentRest3", Param("coilCurrentRest3", ParamBase::Double, 12.5, 100.0, 12.5, tr("coil current if motor 3 is in rest [12.5%, 25%, ... 87.5%, 100%]").toAscii().data()));
    m_params.insert( "coilCurrentThreshold3", Param("coilCurrentThreshold3", ParamBase::Double, 0.0, 10000.0, 5.0, tr("threshold acceleration for distinction between coilCurrentHigh and coilCurrentLow").toAscii().data()));
    m_params.insert( "axisEnabled3", Param("axisEnabled3", ParamBase::Int, 0, 1, 1, tr("determine if motor 3 is enabled (1) or disabled (0). If disabled, this motor is manually moveable").toAscii().data()));
    m_params.insert( "microSteps3", Param("microSteps3", ParamBase::Int, 1, 64, 1, tr("micro steps for motor 3 [1,2,4,8,16,32,64]").toAscii().data()));

    m_currentPos.fill(0.0,3);
    m_currentStatus.fill(0,3);
    m_targetPos.fill(0.0,3);

   // memset(m_pos, 0, 10 * sizeof(double));

   // // This is for the docking widged
   // //now create dock widget for this plugin
   USBMotion3XIIIWid = new DockWidgetUSBMotion3XIII(this);	// Create a new non-modal dialog

   //Marc: connect(this, SIGNAL(statusUpdated(QVector<bool>, QVector<bool>, QVector<double>, QVector<double>, QVector<bool>)), USBMotion3XIIIWid, SLOT(statusUpdated(QVector<bool>, QVector<bool>, QVector<double>, QVector<double>, QVector<bool>)));
   //Marc: connect(this, SIGNAL(targetsChanged(QVector<bool>, QVector<double>)), USBMotion3XIIIWid, SLOT(targetsChanged(QVector<bool>, QVector<double>)));
   
   connect(USBMotion3XIIIWid, SIGNAL(setAbsTargetDegree(double, double, double)), this, SLOT(setAbsTargetDegree(double, double, double)));
   connect(USBMotion3XIIIWid, SIGNAL(setRelTargetDegree(unsigned int, double)), this, SLOT(setRelTargetDegree(unsigned int, double)));

   Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
   QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
   createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, USBMotion3XIIIWid);	// Give the widget a name ..)
   
   // till here
    changeStatusTimer(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::checkConnection()
{
    RetVal retValue(retOk);

    if (initusbmc == 0)
    {
        retValue += ito::RetVal(retError, 0, tr("DLL USB3xIII.dll not loaded").toAscii().data());
    }
    else if (m_params["connected"].getVal<int>() == 0)
    {
        retValue += ito::RetVal(retError, 0, tr("motor driver not connected").toAscii().data());
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::errorCheck(unsigned int driverErrorNumber)
{
    if (driverErrorNumber != 0)
    {
		QString errorMsg = RETSTRING( geterrorstring(driverErrorNumber) );
        return ito::RetVal(ito::retError, 1, tr("driver error: %1 [%2]").arg(errorMsg, QString::number(driverErrorNumber, 16) ).toAscii().data());
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::changeStatusTimer(bool anyMotorIsMoving)
{
    int interval = (anyMotorIsMoving == true) ? 200 : 4000; //200 ms if anything is moving, else 4000

    if (interval != m_timerInterval)
    {
        if (m_timerId != 0)
        {
            killTimer(m_timerId);
        }
        m_timerId = startTimer(interval);
    }
    m_timerInterval = interval;
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::updateStatus()
{
    QVector<bool> available = QVector<bool>() << false << false << false;
    QVector<bool> enabled = QVector<bool>() << false << false << false;
    QVector<bool> running = QVector<bool>() << false << false << false;
    QVector<double> actPosDbl = QVector<double>() << 0.0 << 0.0 << 0.0; 
    QVector<double> targetPosDbl = QVector<double>() << 0.0 << 0.0 << 0.0; 

    int targetPos[] = {0,0,0};
    int actualPos[] = {0,0,0};

    unsigned char mcStatus;
    unsigned char dvrChain;
    int stepsPerTurn;

    enabledriverchain(0x00, 0x00, 1 | 2 | 4, dvrChain, DWTIMEOUT);

    for (int i=0;i<3;i++)
    {
        enabled[i] = dvrChain & (1 << i);
        if (m_availableAxis.contains(i))
        {
            m_currentStatus[i] = m_currentStatus[i] | ito::actuatorAvailable; //available[i] = true;
            
            getxtarget(i, mcStatus, targetPos[i], DWTIMEOUT);
            getxactual(i, mcStatus, actualPos[i], DWTIMEOUT);

            stepsPerTurn = getTotalStepsPerTurn(i);
            m_currentPos[i] = /*actPosDbl[i] =*/ static_cast<double>(360.0 * actualPos[i]) / static_cast<double>(stepsPerTurn);
            m_targetPos[i] = /*targetPosDbl[i] =*/ static_cast<double>(360.0 * targetPos[i]) / static_cast<double>(stepsPerTurn);

            if (mcStatus & (1 << (2*i)))
            {
                //target position reached
                replaceStatus(m_currentStatus[i], ito::actuatorMoving, ito::actuatorAtTarget); 
                running[i] = false;
            }
            else
            {
                if( (m_currentStatus[i] & ito::actuatorInterrupted) == 0)
                {
                    setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask); 
                }
                running[i] = true;
            }
        }
        else
        {
            m_currentStatus[i] = m_currentStatus[i] ^ ito::actuatorAvailable; //available[i] = false;
            
        }
    }

    changeStatusTimer(running[0] || running[1] || running[2]);

    sendStatusUpdate();

    //emit statusUpdated(available, enabled, actPosDbl, targetPosDbl, running);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void USBMotion3XIII::timerEvent( QTimerEvent *event )
{
    updateStatus();
}

//----------------------------------------------------------------------------------------------------------------------------------
int USBMotion3XIII::getTotalStepsPerTurn(int axis) //axis = 0,1,2
{
    int fullSteps = m_params["axisSteps" + QString::number(axis + 1)].getVal<int>();
    int microSteps = m_params["microSteps" + QString::number(axis + 1)].getVal<int>();
    return fullSteps * microSteps; //0 if motor is not available
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::loadDriverSettingsToParams()
{
    unsigned char mcStatus;
    unsigned char valueC1, valueC2, valueC3;
    unsigned short valueS1;
    int steps;
    int microSteps;

    RetVal retValue(retOk);
    
    for (unsigned int axis = 0; axis <= 2; axis ++)
    {
        //motorSteps must already been loaded
        steps = m_params["axisSteps" + QString::number(axis+1)].getVal<int>();
        if (steps <= 0) steps = 200; //if steps == 0, then motor is not available, but in order to avoid a crash in the subsequent calculation, set steps to a default value of 200.

        //microSteps
        retValue += errorCheck(getmicrosteps( axis, valueC1, mcStatus, DWTIMEOUT ));
        if (valueC1 <= 6)
        {
            m_params["microSteps" + QString::number(axis+1)].setVal<int>( 1 << valueC1 );
        }
        else
        {
            m_params["microSteps" + QString::number(axis+1)].setVal<int>(64);
        }
        microSteps = m_params["microSteps" + QString::number(axis+1)].getVal<int>();

        //enabled
        retValue += errorCheck(enabledriverchain(0x00 , 0x00, 1 << axis, valueC1, DWTIMEOUT ));
        m_params["axisEnabled" + QString::number(axis+1)].setVal<int>( valueC1 & (1 << axis) );

        //vmin
        retValue += errorCheck(getvmin(axis, mcStatus, valueS1, DWTIMEOUT ));
        m_params["vMin" + QString::number(axis+1)].setVal<double>( 360.0 * (double)valueS1 / ( steps * microSteps ) );

        //vmax
        retValue += errorCheck(getvmax(axis, mcStatus, valueS1, DWTIMEOUT ));
        m_params["vMax" + QString::number(axis+1)].setVal<double>( 360 * valueS1 / ( steps * microSteps ) );

        //amax
        retValue += errorCheck(getamax(axis, mcStatus, valueS1, DWTIMEOUT ));
        m_params["aMax" + QString::number(axis+1)].setVal<double>( 360 * valueS1 / ( steps * microSteps ) );

        //coil currents
        retValue += errorCheck(getcoilcurrent(axis, mcStatus, valueC1, valueC2, valueC3, valueS1, DWTIMEOUT ));
        m_params["coilCurrentHigh" + QString::number(axis+1)].setVal<double>( valueC1 == 0 ? 100.0 : 12.5 * (double)valueC1 );
        m_params["coilCurrentLow" + QString::number(axis+1)].setVal<double>( valueC1 == 0 ? 100.0 : 12.5 * (double)valueC1 );
        m_params["coilCurrentRest" + QString::number(axis+1)].setVal<double>( valueC1 == 0 ? 100.0 : 12.5 * (double)valueC1 );
        m_params["coilCurrentThreshold" + QString::number(axis+1)].setVal<double>( 360 * valueS1 / ( steps * microSteps ) );
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    RetVal retValue(retOk);
    char *temp = NULL;

	unsigned int errorCode = 0;
    
    QString serialName = (*paramsMand)[0].getVal<char*>(); //borrowed reference

    if (initusbmc == 0)
    {
        retValue += ito::RetVal(retError, 0, tr("DLL USB3xIII.dll (USB3xIII64.dll for 64bit) not loaded").toAscii().data());
    }
    else if (enumdevices() <= 0)
    {
        retValue += ito::RetVal(retError, 1, tr("no motor driver devices are currently connected to this PC").toAscii().data());
    }
	else
	{

#if defined Q_OS_WIN64
		/*if(enumdevices() > 1)
		{
			retValue += ito::RetVal(retWarning,0,tr("64bit problem: currently only the first connected controller").toAscii().data());
		}
		errorCode = opendevicebyindex(0);*/
        int s = serialName.size();
        wchar_t* wserial = new wchar_t[ s + 1];
        serialName.toWCharArray( wserial );
        wserial[s] = 0;
        errorCode = opendevicebyserial( wserial );
        delete wserial;
        wserial = NULL;
#elif defined Q_OS_WIN32
		errorCode = opendevicebyserial( serialName.toAscii().data() );
#endif
		retValue += errorCheck(errorCode);
		if(!retValue.containsError())
		{
			m_curDeviceIndex = currentdeviceindex();

			if (m_curDeviceIndex < 0)
			{
				retValue += ito::RetVal(retError, 2, tr("device could not be opened").toAscii().data());
			}
			else
			{
                QString str = RETSTRING( getserialnumber(m_curDeviceIndex) );
				m_params["connected"].setVal<int>(1);
				m_params["serialNumber"].setVal<char*>( str.toAscii().data() );

				m_identifier = RETSTRING(getserialnumber(m_curDeviceIndex));

                str = RETSTRING( getproductversion(m_curDeviceIndex) );
				m_params["productVersion"].setVal<char*>( str.toAscii().data() );
                str = RETSTRING( getvendorname(m_curDeviceIndex) );
				m_params["vendorName"].setVal<char*>( str.toAscii().data() );
                str = RETSTRING( getproductname(m_curDeviceIndex) );
				m_params["productName"].setVal<char*>( str.toAscii().data() );

				/*int test = paramsOpt->value(0).getVal<int>();
				test = paramsOpt->value(1).getVal<int>();
				test = paramsOpt->value(2).getVal<int>();*/
				m_params["axisSteps1"].setVal<int>( paramsOpt->value(0).getVal<int>() );
				m_params["axisSteps2"].setVal<int>( paramsOpt->value(1).getVal<int>() );
				m_params["axisSteps3"].setVal<int>( paramsOpt->value(2).getVal<int>() );

				if (m_params["axisSteps1"].getVal<int>() > 0) m_availableAxis.append(0);
				if (m_params["axisSteps2"].getVal<int>() > 0) m_availableAxis.append(1);
				if (m_params["axisSteps3"].getVal<int>() > 0) m_availableAxis.append(2);

				//if (initparamfromeep( DWTIMEOUT) ) // load values from EEPROM or default values
				//{
					retValue += errorCheck(initparamtodefault( DWTIMEOUT ));
				//}

				unsigned char mcStatus;
            
				retValue += errorCheck(setmode(0, 0x00, mcStatus, DWTIMEOUT));
				retValue += errorCheck(setmode(1, 0x00, mcStatus, DWTIMEOUT));
				retValue += errorCheck(setmode(2, 0x00, mcStatus, DWTIMEOUT));
				retValue += loadDriverSettingsToParams();

			}
		}
		else
		{
			qDebug() << enumdevices() << " devices of type USB Motion 3X III detected at this computer";
			QString serialNumber;
			for(unsigned int i = 0 ; i < enumdevices() ; i++)
			{
				serialNumber = RETSTRING(getserialnumber(0));
				qDebug() << "serial " << i << ": " << serialNumber;
			}
		}
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

	if(!retValue.containsError())
	{

		if (USBMotion3XIIIWid)
		{
			temp = m_params["name"].getVal<char*>(); //borrowed reference
			QString name = temp;
			temp = m_params["serialNumber"].getVal<char*>(); //borrowed reference
			QString id = temp;

			QString axis = "";
			QVector<bool> available = QVector<bool>() << false << false << false;
			if (m_availableAxis.contains(0)) 
			{
				axis.append("x (0) ");
				available[0] = true;
			}
			if (m_availableAxis.contains(1)) 
			{
				axis.append("y (1) ");
				available[1] = true;
			}
			if (m_availableAxis.contains(2)) 
			{
				axis.append("z (2) ");
				available[2] = true;
			}


			QMetaObject::invokeMethod(USBMotion3XIIIWid, "basicInformationChanged", Q_ARG(QString,name), Q_ARG(QString,id), Q_ARG(QString,axis), Q_ARG(QVector<bool>,available));
		}
	}

	setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
USBMotion3XIII::~USBMotion3XIII()
{
    if (m_timerId > 0)
    {
        killTimer(m_timerId);
        m_timerId = 0;
        m_timerInterval = 0;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_timerId > 0)
    {
        killTimer(m_timerId);
        m_timerId = 0;
        m_timerInterval = 0;
    }


    if (!checkConnection().containsError() && m_curDeviceIndex >= 0 )
    {
        //retValue += errorCheck( saveparamtoeep(0, DWTIMEOUT) );
        
        if (closedevice() == 0)
        {
            m_curDeviceIndex = -1;
            m_params["connected"].setVal<int>(0);
        }
        else
        {
            retValue += ito::RetVal(retError, 1, tr("device could not be closed").toAscii().data());
        }

    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
		waitCond->release();        
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(key);
        if (paramIt != m_params.constEnd())
        {
            *val = paramIt.value();
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toAscii().data());
        }
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

   return retValue;
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();
    int iTemp;
    int axis;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Motor is moving. Parameters cannot be set.").toAscii().data());
    }
    else if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end() && (paramIt->getType() == val->getType() || (paramIt->isNumeric() && val->isNumeric())))
        {
            //check read-only
            if ((*paramIt).getFlags() & ParamBase::Readonly)
            {
                retValue += ito::RetVal(retError, 0, tr("this parameter is only readable").toAscii().data());
            }
            else if (key == "async")
            {
                iTemp = val->getVal<int>();
                if (iTemp == 0 || iTemp == 1)
                {
                    (*paramIt).setVal<int>(iTemp);
                    m_async = iTemp;
                }
                else
                {
                    retValue += ito::RetVal(retError, 0, tr("async parameter needs value 0 or 1").toAscii().data());
                }
            }
            else if (key == "microSteps1" || key == "microSteps2" || key == "microSteps3")
            {
                axis = key.right(1).toInt();
                retValue += setMicroSteps( axis, val->getVal<int>() );

                //if micro-steps are changed, acceleration, speed and coil threshold value must be reset, too
                retValue += setAcceleration( axis, m_params["aMax" + QString::number(axis)].getVal<double>() );
                retValue += setSpeed( axis, 0x1 | 0x2, m_params["vMin" + QString::number(axis)].getVal<double>(), m_params["vMax" + QString::number(axis)].getVal<double>() );
                retValue += setCoilCurrents( axis, 0x8, 0.0, 0.0, 0.0, m_params["coilCurrentThreshold" + QString::number(axis)].getVal<double>() );
            }
            else if (key == "axisEnabled1" || key == "axisEnabled2" || key == "axisEnabled3")
            {
                retValue += setEnabled( key.right(1).toInt(), val->getVal<int>() );
            }
            else if (key == "aMax1" || key == "aMax2" || key == "aMax3")
            {
                retValue += setAcceleration( key.right(1).toInt(), val->getVal<double>() );
            }
            else if (key == "vMax1" || key == "vMax2" || key == "vMax3")
            {
                retValue += setSpeed( key.right(1).toInt(), 0x2, 0.0, val->getVal<double>() );
            }
            else if (key == "vMin1" || key == "vMin2" || key == "vMin3")
            {
                retValue += setSpeed( key.right(1).toInt(), 0x1,val->getVal<double>(), 0.0 );
            }
            else if (key == "coilCurrentHigh1" || key == "coilCurrentHigh2" || key == "coilCurrentHigh3")
            {
                retValue += setCoilCurrents( key.right(1).toInt(), 0x01, val->getVal<double>(), 0,0,0 );
            }
            else if (key == "coilCurrentLow1" || key == "coilCurrentLow2" || key == "coilCurrentLow3")
            {
                retValue += setCoilCurrents( key.right(1).toInt(), 0x02, 0,val->getVal<double>(), 0,0 );
            }
            else if (key == "coilCurrentRest1" || key == "coilCurrentRest2" || key == "coilCurrentRest3")
            {
                retValue += setCoilCurrents( key.right(1).toInt(), 0x04, 0,0,val->getVal<double>(), 0 );
            }
            else if (key == "coilCurrentThreshold1" || key == "coilCurrentThreshold2" || key == "coilCurrentThreshold3")
            {
                retValue += setCoilCurrents( key.right(1).toInt(), 0x08, 0,0,0,val->getVal<double>() );
            }
            else
            {
                retValue += paramIt.value().copyValueFrom( &(*val) );
            }
        }
        else if (paramIt->getType() != val->getType())
        {
            retValue += ito::RetVal(ito::retError, 0, tr("given parameter and m_param do not have the same type").toAscii().data());
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toAscii().data());
        }
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setMicroSteps(int axis, int steps) //axis = 1,2,3, steps = 1,2,4,8,16,32,64
{
    RetVal retValue(retOk);
    unsigned char msteps = 0;
    unsigned char msteps2 = 0;
    unsigned char mcStatus;

    if (axis < 1 || axis > 3) return RetVal(retError,0,"axis must be 1,2,3");
    if (!m_availableAxis.contains((unsigned char)(axis-1)))
    {
        return ito::RetVal(retError, 0, tr("this axis is not available (axisSteps = 0)").toAscii().data());
    }
    
    switch(steps)
    {
    case 1: msteps = 0; break;
    case 2: msteps = 1; break;
    case 4: msteps = 2; break;
    case 8: msteps = 3; break;
    case 16: msteps = 4; break;
    case 32: msteps = 5; break;
    case 64: msteps = 6; break;
    default:
        return ito::RetVal(retError, 0, tr("micro-steps must have one of these values: [1,2,4,8,16,32,64]").toAscii().data());
    }

    retValue += errorCheck( setmicrosteps(axis-1, msteps, mcStatus, DWTIMEOUT) );

    getmicrosteps(axis-1, msteps2, mcStatus, DWTIMEOUT);
    if (msteps2 != msteps)
    {
        steps = 1 << msteps2;
    }

    if (retValue == retOk )
    {
        retValue += m_params["microSteps" + QString::number(axis)].setVal<double>(steps);
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//axis = 1,2,3
//changeBitMask (OR-Relation): 0x1 -> change agtat, 0x2 -> change aleat, 0x4 -> change v0, 0x8 -> change threshold
ito::RetVal USBMotion3XIII::setCoilCurrents(int axis, char changeBitMask, double agtat, double aleat, double v0, double threshold) 
{
    RetVal retValue(retOk);

    unsigned char motor;
    unsigned char mcStatus;
    unsigned char driverAgtat, driverAleat, driverV0;
    unsigned short driverThreshold;
    double paramMin;
    double paramMax;
    int stepsPerTurn;

    if (axis < 1 || axis > 3)
    {
        return ito::RetVal(retError, 0, tr("axis must be 1,2,3").toAscii().data());
    }
    if (!m_availableAxis.contains((unsigned char)(axis-1)))
    {
        return ito::RetVal(retError, 0, tr("this axis is not available (axisSteps = 0)").toAscii().data());
    }

    stepsPerTurn = getTotalStepsPerTurn(axis-1);
    motor = axis-1;

    if ((changeBitMask ^ (0x1 | 0x2 | 0x4 | 0x8)) > 0) //not every value will be changed, therefore read the current values
    {
        retValue += errorCheck( getcoilcurrent(motor, mcStatus, driverAgtat, driverAleat, driverV0, driverThreshold, DWTIMEOUT ) );
    }

    if (changeBitMask & 0x1) //set coilCurrentHigh
    {
        paramMin = m_params["coilCurrentHigh" + QString::number(axis)].getMin();
        paramMax = m_params["coilCurrentHigh" + QString::number(axis)].getMax();

        if (agtat < paramMin) return RetVal(retError,0, tr("coilCurrentHigh is lower than the minimal allowed value").toAscii().data());
        if (agtat > paramMax) return RetVal(retError,0, tr("coilCurrentHigh is bigger than the maximal allowed value").toAscii().data());

        driverAgtat = agtat >= 100 ? 0 : (unsigned char)(agtat / 12.5);
    }
    if (changeBitMask & 0x2) //set coilCurrentLow
    {
        paramMin = m_params["coilCurrentLow" + QString::number(axis)].getMin();
        paramMax = m_params["coilCurrentLow" + QString::number(axis)].getMax();

        if (aleat < paramMin) return RetVal(retError,0, tr("coilCurrentLow is lower than the minimal allowed value").toAscii().data());
        if (aleat > paramMax) return RetVal(retError,0, tr("coilCurrentLow is bigger than the maximal allowed value").toAscii().data());

        driverAleat = aleat >= 100 ? 0 : (unsigned char)(aleat / 12.5);
    }
    if (changeBitMask & 0x4) //set coilCurrentRest
    {
        paramMin = m_params["coilCurrentRest" + QString::number(axis)].getMin();
        paramMax = m_params["coilCurrentRest" + QString::number(axis)].getMax();

        if (v0 < paramMin) return RetVal(retError,0, tr("coilCurrentRest is lower than the minimal allowed value").toAscii().data());
        if (v0 > paramMax) return RetVal(retError,0, tr("coilCurrentRest is bigger than the maximal allowed value").toAscii().data());

        driverV0 = v0 >= 100 ? 0 : (unsigned char)(v0 / 12.5);
    }
    if (changeBitMask & 0x8) //set coilCurrentThreshold
    {
        paramMin = m_params["coilCurrentThreshold" + QString::number(axis)].getMin();
        paramMax = m_params["coilCurrentThreshold" + QString::number(axis)].getMax();

        if (threshold < paramMin) return RetVal(retError,0, tr("coilCurrentThreshold is lower than the minimal allowed value").toAscii().data());
        if (threshold > paramMax) return RetVal(retError,0, tr("coilCurrentThreshold is bigger than the maximal allowed value").toAscii().data());

        driverThreshold = qRound(threshold * static_cast<double>(stepsPerTurn) / 360.0);
    }

    if (retValue == retOk)
    {
        retValue += errorCheck( setcoilcurrent(motor, driverAgtat, driverAleat, driverV0, driverThreshold, mcStatus, DWTIMEOUT) );

        getcoilcurrent(motor, mcStatus, driverAgtat, driverAleat, driverV0, driverThreshold, DWTIMEOUT );
        threshold = 360.0 * static_cast<double>(driverThreshold) / static_cast<double>(stepsPerTurn);
        agtat = driverAgtat == 0 ? 100.0 : 12.5 * driverAgtat;
        aleat = driverAleat == 0 ? 100.0 : 12.5 * driverAleat;
        v0    =    driverV0 == 0 ? 100.0 : 12.5 * driverV0;

        if (changeBitMask & 0x1)
        {
            retValue += m_params["coilCurrentHigh" + QString::number(axis)].setVal<double>(agtat);
        }
        if (changeBitMask & 0x2)
        {
            retValue += m_params["coilCurrentLow" + QString::number(axis)].setVal<double>(aleat);
        }
        if (changeBitMask & 0x4)
        {
            retValue += m_params["coilCurrentRest" + QString::number(axis)].setVal<double>(v0);
        }
        if (changeBitMask & 0x8)
        {
            retValue += m_params["coilCurrentThreshold" + QString::number(axis)].setVal<double>(threshold);
        }
    }

    return retValue;
}  

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setSpeed(int axis, char changeBitMask, double vmin, double vmax) //axis = 1,2,3, changeBitMask = 0x1 (change vmin), 0x2 (change vmax) or both
{
    RetVal retValue(retOk);

    unsigned char motor;
    unsigned char mcStatus;
    unsigned short driverVMin;
    unsigned short driverVMax;
    double paramMin;
    double paramMax;
    double stepsPerTurn;

    if (axis < 1 || axis > 3)
    {
        return RetVal(retError, 0, tr("axis must be 1,2,3").toAscii().data());
    }
    if (!m_availableAxis.contains((unsigned char)(axis-1)))
    {
        return ito::RetVal(retError, 0, tr("this axis is not available (axisSteps = 0)").toAscii().data());
    }

    stepsPerTurn = static_cast<double>( getTotalStepsPerTurn(axis-1) );
    motor = axis-1;

    if (changeBitMask & 0x2) //set vmax
    {
        paramMin = m_params["vMax" + QString::number(axis)].getMin();
        paramMax = m_params["vMax" + QString::number(axis)].getMax();

        if (vmax < paramMin)
        {
            return ito::RetVal(retError, 0, tr("vMax is lower than the minimal allowed value").toAscii().data());
        }
        if (vmax > paramMax)
        {
            return ito::RetVal(retError, 0, tr("vMax is bigger than the maximal allowed value").toAscii().data());
        }

        driverVMax = qRound( vmax * stepsPerTurn / 360.0 );
    
        retValue += errorCheck( setvmax( motor, driverVMax, mcStatus, DWTIMEOUT ) );

        getvmax(motor, mcStatus, driverVMax, DWTIMEOUT);
        vmax = 360.0 * static_cast<double>(driverVMax) / stepsPerTurn;

        if (retValue == retOk )
        {
            retValue += m_params["vMax" + QString::number(axis)].setVal<double>(vmax);
        }
    }

    if (changeBitMask & 0x1) //set vmin
    {
        paramMin = m_params["vMin" + QString::number(axis)].getMin();
        paramMax = m_params["vMin" + QString::number(axis)].getMax();

        if (vmin < paramMin)
        {
            return ito::RetVal(retError, 0, tr("vMin is lower than the minimal allowed value").toAscii().data());
        }
        if (vmin > paramMax)
        {
            return ito::RetVal(retError, 0, tr("vMin is bigger than the maximal allowed value").toAscii().data());
        }

        driverVMin = qRound( vmin * stepsPerTurn / 360.0 );
    
        retValue += errorCheck( setvmin( motor, driverVMin, mcStatus, DWTIMEOUT ) );

        getvmin(motor, mcStatus, driverVMin, DWTIMEOUT);
        vmin = 360.0 * static_cast<double>(driverVMin) / stepsPerTurn;

        if (retValue == retOk)
        {
            retValue += m_params["vMin" + QString::number(axis)].setVal<double>(vmin);
        }
    }

    getvmin(motor, mcStatus, driverVMin, DWTIMEOUT);
    qDebug() << "V-Min set to " << driverVMin << " steps / sek; " << (360*driverVMin)/stepsPerTurn <<" °/sek";
    getvmax(motor, mcStatus, driverVMax, DWTIMEOUT);
    qDebug() << "V-Max set to " << driverVMax << " steps / sek; " << (360*driverVMax)/stepsPerTurn << " °/sek";

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setAcceleration(int axis, double amax) //axis = 1,2,3
{
    RetVal retValue(retOk);

    unsigned char motor;
    unsigned char mcStatus;
    unsigned short driverAmax;
    double paramMin;
    double paramMax;
    int stepsPerTurn;

    if (axis < 1 || axis > 3)
    {
        return ito::RetVal(retError, 0, tr("axis must be 1,2,3").toAscii().data());
    }
    if (!m_availableAxis.contains((unsigned char)(axis-1)))
    {
        return ito::RetVal(retError, 0, tr("this axis is not available (axisSteps = 0)").toAscii().data());
    }

    paramMin = m_params["aMax" + QString::number(axis)].getMin();
    paramMax = m_params["aMax" + QString::number(axis)].getMax();

    if (amax < paramMin)
    {
        return ito::RetVal(retError, 0, tr("aMax is lower than the minimal allowed value").toAscii().data());
    }
    if (amax > paramMax)
    {
        return ito::RetVal(retError, 0, tr("aMax is bigger than the maximal allowed value").toAscii().data());
    }

    stepsPerTurn = getTotalStepsPerTurn(axis-1);
    driverAmax = qRound(amax * static_cast<double>(stepsPerTurn) / 360.0);

    motor = axis-1;
    
    retValue += errorCheck( setamax( motor, driverAmax, mcStatus, DWTIMEOUT ) );

    //check
    getamax(motor, mcStatus, driverAmax, DWTIMEOUT);
    amax = 360.0 * static_cast<double>(driverAmax) / static_cast<double>(stepsPerTurn);

    if (retValue == retOk )
    {
        retValue += m_params["aMax" + QString::number(axis)].setVal<double>(amax);
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setEnabled(int axis, int value) //axis = 1,2,3
{
    RetVal retValue(retOk);
    unsigned char intVal = static_cast<unsigned char>(value);
    unsigned char motor;
    unsigned char dvrState;

    if (axis < 1 || axis > 3)
    {
        return ito::RetVal(retError, 0, tr("axis must be 1,2,3").toAscii().data());
    }
    if (!m_availableAxis.contains((unsigned char)(axis-1)))
    {
        return ito::RetVal(retError, 0, tr("this axis is not available (axisSteps = 0)").toAscii().data());
    }
    if (intVal != 0 && intVal != 1)
    {
        return ito::RetVal(retError, 0, tr("enabled flag must be 0 or 1").toAscii().data());
    }

    motor = 1 << (axis-1);
    dvrState = intVal == 0 ? 0 : motor;


    
    retValue += errorCheck( enabledriverchain(0x01, intVal, motor,  dvrState, DWTIMEOUT) );

    if (retValue == retOk )
    {
        retValue += m_params["axisEnabled" + QString::number(axis)].setVal<int>(intVal);
        if(intVal)
        {
            m_currentStatus[axis-1] = m_currentStatus[axis-1] | ito::actuatorEnabled;
        }
        else
        {
            m_currentStatus[axis-1] = m_currentStatus[axis-1] ^ ito::actuatorEnabled;
        }
        sendStatusUpdate(true);
    }
    
    return retValue;
}




//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    return calib(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::RetVal(ito::retWarning, 0, tr("calibration not possible").toAscii().data());

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toAscii().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
		waitCond->release();
        
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    return setOrigin(QVector<int>(1,axis), waitCond);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setOrigin(QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toAscii().data());
    }
    else
    {
        foreach(const int &i,axis)
        {
            if (m_availableAxis.contains((unsigned char)i) )
            {
                unsigned char mcStatus;
                retValue += errorCheck( initposition( i , 0 , mcStatus , DWTIMEOUT) );
            }
            else
            {
                retValue += ito::RetVal(retError, 1, tr("axis not available").toAscii().data());
            }
        }

        retValue += updateStatus();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    retValue += updateStatus();
    *status = m_currentStatus;

    //unsigned char motor;
    //short speed;
    //unsigned char mcStatus;

    //*status = 0; //status 0: no available motor is running, else 1
    //unsigned char axis;

    //foreach(axis,m_availableAxis)
    //{
    //    getvactual(axis, mcStatus, speed, DWTIMEOUT);

    //    if (speed != 0)
    //    {
    //        *status = 1;
    //    }

    //}
    //

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_availableAxis.contains((unsigned char)axis) )
    {
        unsigned char mcStatus;
        int tempPos;
        retValue += errorCheck( getxactual( axis ,  mcStatus , tempPos,  DWTIMEOUT) );
        //transform tempPos to angle
        *pos = 360.0 * (double)tempPos / (double)getTotalStepsPerTurn(axis);
        m_currentPos[axis] = *pos;
    }
    else
    {
        retValue += ito::RetVal(retError, 1, tr("axis not available").toAscii().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::getPos(QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    for (int naxis = 0; naxis < axis.size(); naxis++)
    {
        if (m_availableAxis.contains((unsigned char)naxis) )
        {
            unsigned char mcStatus;
            int tempPos;
            retValue += errorCheck( getxactual( axis[naxis] ,  mcStatus , tempPos,  DWTIMEOUT) );
            //transform tempPos to angle
            m_currentPos[naxis] = 360.0 * (double)tempPos / (double)getTotalStepsPerTurn(naxis);
            (*pos)[naxis] = m_currentPos[axis[naxis]];
        }
        else
        {
            retValue += ito::RetVal(retError, 1, tr("at least one axis not available").toAscii().data());
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toAscii().data());
    }
    else if( !m_availableAxis.contains(axis) )
    {
        retValue += ito::RetVal(retError, 1, tr("axis not available").toAscii().data());
    }
    else if (m_params["axisEnabled" + QString::number(axis+1)].getVal<int>() == 0)
    {
        retValue += ito::RetVal(retError, 2, tr("axis not enabled").toAscii().data());
    }
    else
    {
        //pos is in degree
        int stepsPerTurn = getTotalStepsPerTurn(axis);
        int steps = qRound( static_cast<double>(pos * stepsPerTurn) / 360.0 );
        m_targetPos[axis] = static_cast<double>(360 * steps) / static_cast<double>(stepsPerTurn); //calc it here in order to consider round-problems
        unsigned char mcStatus;
        setStatus(m_currentStatus[axis], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
        retValue += errorCheck( setxtarget(axis, steps, mcStatus, DWTIMEOUT) );

        if(!retValue.containsError() )
        {
            sendTargetUpdate();
            changeStatusTimer(true); //set timer to short interval, since motor is running now
        }
        
        if(m_async && waitCond && !released)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
            released = true;
        }

        retValue += waitForDone(60000, QVector<int>(1,axis)); //WaitForAnswer(60000, axis);

        if(!m_async && waitCond && !released)
        {
            waitCond->returnValue = retValue;
            waitCond->release();
            released = true;
        }

    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
        released = true;
    }

    return retValue;
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setPosAbs(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    int newTargetSteps[3] = {0, 0, 0};
    char motor = 0;
    int stepsPerTurn;
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toAscii().data());
    }
    else
    {
        for (int i=0; i<axis.size(); i++)
        {
            if (!m_availableAxis.contains(axis[i]))
            {
                retValue += ito::RetVal(retError, 1, tr("axis not available").toAscii().data());
            }
            else if (m_params["axisEnabled" + QString::number(axis[i]+1)].getVal<int>() == 0)
            {
                retValue += ito::RetVal(retError, 2, tr("axis not enabled").toAscii().data());
            }
            motor |= (1 << axis[i]);
            if (axis[i] >= 0 && axis[i] < 3 )
            {
                stepsPerTurn = getTotalStepsPerTurn(axis[i]);
                newTargetSteps[axis[i]] = qRound(static_cast<double>(stepsPerTurn * pos[i]) / 360.0);
                m_targetPos[axis[i]] = static_cast<double>(360 * newTargetSteps[axis[i]]) / static_cast<double>(stepsPerTurn); //calc it here in order to consider round-problems
            }
        }

        if (retValue == ito::retOk)
        {
            unsigned char mcStatus;
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            retValue += errorCheck( setxyztarget(motor, newTargetSteps[0], newTargetSteps[1], newTargetSteps[2], mcStatus, DWTIMEOUT) );
            sendTargetUpdate();
            changeStatusTimer(true); //set timer to short interval, since motor is running now

            if(m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            retValue += waitForDone(60000, axis); //WaitForAnswer(60000, axis);

            if(!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toAscii().data());
    }
    else if (! m_availableAxis.contains(axis) )
    {
        retValue += ito::RetVal(retError, 1, tr("axis not available").toAscii().data());
    }
    else if (m_params["axisEnabled" + QString::number(axis+1)].getVal<int>() == 0)
    {
        retValue += ito::RetVal(retError, 2, tr("axis not enabled").toAscii().data());
    }
    else
    {
        QSharedPointer<double> actPos(new double);
        retValue += getPos(axis, actPos, NULL);

        if (retValue == ito::retOk)
        {
            //pos is in degree
            int stepsPerTurn = getTotalStepsPerTurn(axis);
            int steps = qRound( static_cast<double>(stepsPerTurn * (*actPos + pos)) / 360.0);
            m_targetPos[axis] = static_cast<double>(360 * steps) / static_cast<double>(stepsPerTurn); //calc it here in order to consider round-problems

            unsigned char mcStatus;
            setStatus(m_currentStatus[axis], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            retValue += errorCheck( setxtarget(axis, steps, mcStatus, DWTIMEOUT) );
            if(!retValue.containsError())
            {
                sendTargetUpdate();
                changeStatusTimer(true); //set timer to short interval, since motor is running now
            }

            if(m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            retValue += waitForDone(60000, QVector<int>(1,axis) ); //WaitForAnswer(60000, axis);

            if(!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal USBMotion3XIII::setPosRel(QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    int oldTarget[3] = {0, 0, 0};
    int newTarget[3] = {0, 0, 0};
    short s[3] = {0, 0, 0};
    char motor = 0;
    int stepsPerTurn;
    unsigned char mcStatus;
    bool released = false;

    if(isMotorMoving())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("motor is running. Further action is not possible").toAscii().data());
    }
    else
    {
        retValue += errorCheck( getxvactual(oldTarget[0], oldTarget[1], oldTarget[2], s[0], s[1], s[2], mcStatus, DWTIMEOUT) );

        for (int i=0; i<axis.size(); i++)
        {
            if (!m_availableAxis.contains(axis[i]))
            {
                retValue += ito::RetVal(retError, 1, tr("axis not available").toAscii().data());
            }
            else if (m_params["axisEnabled" + QString::number(axis[i]+1)].getVal<int>() == 0)
            {
                retValue += ito::RetVal(retError, 2, tr("axis not enabled").toAscii().data());
            }
            motor |= (1 << axis[i]);
            if (axis[i] >= 0 && axis[i] < 3 )
            {
                stepsPerTurn = getTotalStepsPerTurn(axis[i]);
                newTarget[axis[i]] = oldTarget[axis[i]] + qRound( static_cast<double>(stepsPerTurn * pos[i]) / 360.0 );
                m_currentPos[axis[i]] = newTarget[axis[i]];
                m_targetPos[axis[i]] = static_cast<double>(360 * newTarget[axis[i]]) / static_cast<double>(stepsPerTurn); //calc it here in order to consider round-problems
            }
        }

        if (retValue == ito::retOk)
        {
            setStatus(axis, ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
            retValue += errorCheck( setxyztarget(motor, newTarget[0], newTarget[1], newTarget[2], mcStatus, DWTIMEOUT) );
            sendStatusUpdate();
            sendTargetUpdate();
            changeStatusTimer(true); //set timer to short interval, since motor is running now

            if(m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }

            retValue += waitForDone(60000, axis ); //WaitForAnswer(60000, axis);

            if(!m_async && waitCond && !released)
            {
                waitCond->returnValue = retValue;
                waitCond->release();
                released = true;
            }
        }
    }

    if (waitCond && !released)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

	return retValue;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
ito::RetVal USBMotion3XIII::waitForDone(const int timeoutMS, const QVector<int> axis, const int flags)
{
    ito::RetVal retVal(ito::retOk);
    bool done = false;
    bool timeout = false;
    unsigned char mcStatus;
    short v1, v2, v3;
    int x1, x2, x3;
    char motor;
    QTime timer;
    QMutex waitMutex;
    QWaitCondition waitCondition;
    long delay = 100; //[ms]

    timer.start();

    QVector<int> _axis = axis;
    if (_axis.size() == 0) //all axis
    {
        for (int i=0;i<3;i++) 
        {
            if(m_availableAxis.contains(i)) _axis.append(i);
        }
    }

    foreach(const int &i, axis)
    {
        if (!m_availableAxis.contains(i))
        {
            return ito::RetVal(retError, 1, tr("axis not available").toAscii().data());
        }
    }

    while (!done && !timeout)
    {
        retVal += errorCheck( getxvactual(x1, x2, x3, v1, v2, v3, mcStatus, DWTIMEOUT ));

        done = true;
        motor = 0;
        foreach(const int &i,axis)
        {
            switch(i)
            {
            case 0:
                if (v1 != 0 || (mcStatus & 0x01) == 0) 
                {
                    motor |= 1;
                    setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                    done = false;
                }
                else
                {
                    setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                }
                break;
            case 1:
                if (v2 != 0 || (mcStatus & 0x04) == 0) 
                {
                    motor |= 2;
                    setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                    done = false;
                }
                else
                {
                    setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                }
                break;
            case 2:
                if (v3 != 0 || (mcStatus & 0x10) == 0) 
                {
                    motor |= 4;
                    setStatus(m_currentStatus[i], ito::actuatorMoving, ito::actSwitchesMask | ito::actStatusMask);
                    done = false;
                }
                else
                {
                    setStatus(m_currentStatus[i], ito::actuatorAtTarget, ito::actSwitchesMask | ito::actStatusMask);
                }
                break;
            }
        }

        sendStatusUpdate(true);

        if (!done && isInterrupted())
        {
            char wr = 0x01; //write access first
            char ena = 0x00; //disable motor
            unsigned char dvrState;
            // 2. disable motor, 3. read position, 4. set position as target, 5. enable motor which have been enabled before
            retVal += errorCheck( enabledriverchain(wr,ena,motor,dvrState, DWTIMEOUT) );
            retVal += errorCheck( getxvactual(x1, x2, x3, v1, v2, v3, mcStatus, DWTIMEOUT ));
            //stop motors by setting actual position to new target
            retVal += errorCheck( setxyztarget(motor, x1, x2, x3, mcStatus, DWTIMEOUT) );
            ena = 0x01;
            retVal += errorCheck( enabledriverchain(wr,ena,motor,dvrState, DWTIMEOUT) );

            retVal += errorCheck( getxvactual(x1, x2, x3, v1, v2, v3, mcStatus, DWTIMEOUT ));
            //stop motors by setting actual position to new target
            retVal += errorCheck( setxyztarget(motor, x1, x2, x3, mcStatus, DWTIMEOUT) );

            replaceStatus(_axis, ito::actuatorMoving, ito::actuatorInterrupted);
            sendStatusUpdate(true);


            //the following steps are necessary, since the motor is not stopping immediately. The motor tries to finish its ramp. Therefore we
            //continuously read the current position and set it as new target in order to force a recalculation of the ramp.
            waitMutex.lock();
            waitCondition.wait(&waitMutex, 100);
            waitMutex.unlock();
            retVal += errorCheck( getxvactual(x1, x2, x3, v1, v2, v3, mcStatus, DWTIMEOUT ));
            //stop motors by setting actual position to new target
            retVal += errorCheck( setxyztarget(motor, x1, x2, x3, mcStatus, DWTIMEOUT) );

            waitMutex.lock();
            waitCondition.wait(&waitMutex, 100);
            waitMutex.unlock();
            retVal += errorCheck( getxvactual(x1, x2, x3, v1, v2, v3, mcStatus, DWTIMEOUT ));
            //stop motors by setting actual position to new target
            retVal += errorCheck( setxyztarget(motor, x1, x2, x3, mcStatus, DWTIMEOUT) );
            
            retVal += ito::RetVal(ito::retError,0,"interrupt occurred");
            done = true;
            return retVal;
        }

        QCoreApplication::processEvents();

        //short delay
        waitMutex.lock();
        waitCondition.wait(&waitMutex, delay);
        waitMutex.unlock();
        setAlive();

        if (timeoutMS > -1)
        {
            if (timer.elapsed() > timeoutMS) timeout = true;
        }
    }

    if (timeout)
    {
        replaceStatus(_axis, ito::actuatorMoving, ito::actuatorTimeout);
        retVal += ito::RetVal(ito::retError,9999,"timeout occurred");
    }

    return retVal;
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void USBMotion3XIII::setAbsTargetDegree(double target1, double target2, double target3)
{
    QVector<int> axis;
    QVector<double> pos;

    for (int i=0 ; i<3; i++)
    {
        if (m_availableAxis.contains(i))
        {
            axis.append(i);
            switch(i)
            {
            case 0:
                pos.append(target1);
                break;
            case 1:
                pos.append(target2);
                break;
            case 2:
                pos.append(target3);
                break;
            }
        }
    }

    setPosAbs(axis,pos,NULL);
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void USBMotion3XIII::setRelTargetDegree(unsigned int axisNo, double relStepDegree)
{
    setPosRel(axisNo, relStepDegree, NULL);
}

//---------------------------------------------------------------------------------------------------------------------------------- 
void USBMotion3XIII::dockWidgetVisibilityChanged( bool visible )
{
    if(USBMotion3XIIIWid)
    {
        if(visible)
        {
            connect( this, SIGNAL( actuatorStatusChanged(QVector<int>,QVector<double>) ), USBMotion3XIIIWid, SLOT( actuatorStatusChanged(QVector<int>,QVector<double>) ) );
            connect( this, SIGNAL( targetChanged(QVector<double>) ), USBMotion3XIIIWid, SLOT( targetChanged(QVector<double>) ) );
        }
        else
        {
            disconnect( this, SIGNAL( actuatorStatusChanged(QVector<int>,QVector<double>) ), USBMotion3XIIIWid, SLOT( actuatorStatusChanged(QVector<int>,QVector<double>) ) );
            disconnect( this, SIGNAL( targetChanged(QVector<double>) ), USBMotion3XIIIWid, SLOT( targetChanged(QVector<double>) ) );
        }
    }
}