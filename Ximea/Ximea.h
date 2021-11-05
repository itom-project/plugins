/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2015, twip optical solutions GmbH
    Copyright (C) 2018, Institut fuer Technische Optik, Universitaet Stuttgart

    This file is part of a plugin for the measurement software itom.
  
    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef XIMEA_H
#define XIMEA_H

#include "common/addInGrabber.h"
#include "dialogXimea.h"
#include <qsharedpointer.h>


#include "xiApi.h"

struct SoftwareShading
{
    SoftwareShading()
    {
        valid = false;
        active = false;
        x0 = 0;
        y0 = 0;
        xsize = 0;
        ysize = 0;
        sub = NULL;
        mul = NULL;
        subBase = NULL;
        mulBase = NULL;
        m_correction.clear();
    }
    ~SoftwareShading()
    {
        active = false;
        if(sub) delete sub;
        if(mul) delete mul;
        if(subBase) delete subBase;
        if(mulBase) delete mulBase;
    }
    QMap<int, QVector< QPointF > > m_correction;
    bool valid;
    bool active;
    int x0;
    int y0;
    int xsize;
    int ysize;
    ito::uint16 *sub;
    ito::uint16 *mul;
    ito::uint16 *subBase;
    ito::uint16 *mulBase;
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    Ximea
  *\brief    class to use a Ximea camera as an ITOM-Addin. Child of AddIn - Library (DLL) - Interface
  *
  *         This class can be used to work with a Ximea USB3 camera. It grabbes datas with 8 or ?? Bit.
  *            The "m3api.dll" has to be in a subfolder .\Ximea in the plugin directory
  *
  *    \sa    AddInDataIO, DummyGrabber
  *    \date    04.07.2012
  *    \author  CK, Ly
  * \warning    NA
  *
  */
class Ximea : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~Ximea();
        //! Constructor
        Ximea();
        void dockWidgetVisibilityChanged(bool visible);

    public:
        friend class XimeaInterface;
        const ito::RetVal showConfDialog(void);    /*!< Open the config nonmodal dialog to set camera parameters */
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);    /*! <Wait for acquired picture */
        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);    /*!< Check if object has to be reallocated */

        ito::RetVal setXimeaParam(const char *paramName, int newValue);

    private:

        enum DeviceFamily
        {
            familyMR,    //scientific grade firewire
            familyMQ,    //xiQ USB3, CMOS
            familyMD,    //xiD USB3, CCD
            familyMU,    //subminiature, CMOS
            familyCB,    //xiB, PCI Express, CMOS
            familyMH,
            familyCURRERA,
            familyMT,
            familyUnknown
        };

		enum SyncParams {          
            sExposure = 0x0001, 
            sBinning = 0x0002,
            sRoi = 0x0004,
            sGain = 0x0008,
            sOffset = 0x0010,
            sTriggerMode = 0x0020,
			sTriggerSelector = 0x0040,
			sBpp = 0x0080,
			sFrameRate = 0x0100,
			sGamma = 0x0200,
			sSharpness = 0x0400,
            sGpiGpo = 0x0800,
			sLens = 0x1000,
            sAll = sExposure | sBinning | sRoi | sGain | sOffset | sTriggerMode | sTriggerSelector | sBpp | sFrameRate | sGamma | sSharpness | sGpiGpo | sLens
        };

		struct RoiMeta
		{
			int offsetXMin;
			int offsetXMax;
			int offsetXStep;
			int offsetYMin;
			int offsetYMax;
			int offsetYStep;
			int widthMin;
			int widthMax;
			int widthStep;
			int heightMin;
			int heightMax;
			int heightStep;
		};

		ito::RetVal synchronizeCameraSettings(int what = sAll);
        ito::RetVal readCameraIntParam(const char *ximeaParamName, const QString &paramName, bool mandatory = false);
        ito::RetVal readCameraFloatParam(const char *ximeaParamName, const QString &paramName, bool mandatory = false);

		inline double musecToSec(double musec) { return (double)musec * 1.0e-6; }
		inline double secToMusec(double sec) { return (double)(sec * 1.0e6); }

		RoiMeta m_roiMeta;
        DeviceFamily m_family;
        int m_numGPIPins;
        int m_numGPOPins;
        int m_numFrameBurst;
        int m_maxOutputBitDepth;

        ito::RetVal LoadLib();
        ito::RetVal getErrStr(const int error, const QString &command, const QString &value);
        ito::RetVal checkError(const XI_RETURN &error, const QString &command, const QString &value = QString());
        int m_saveParamsOnClose;
        bool m_channelNumberChanged;
        int m_originalSizeX;
        int m_originalSizeY;
        ito::DataObject m_hyperspectralCubeObj;
        
#if linux
        void *m_handle;
#else
        HANDLE m_handle;
#endif

#if linux
        void *ximeaLib;
#else
        HMODULE ximeaLib;
#endif
    
        void* m_pvShadingSettings;

        SoftwareShading m_shading;


        bool m_isgrabbing;
        ito::RetVal m_acqRetVal;
    signals:
        //void parametersChanged(QMap<QString, ito::Param> params);    /*! Signal send changed or all parameters to listeners */

    public slots:
        //! returns parameter of m_params with key name.
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        //! sets parameter of m_params with key name.
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        //! Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //! Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //! Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //! Stop the camera to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //! Softwaretrigger for the camera
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //! Calls retrieveData(), than shallow copy the picture to dObj of right type and size
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        //! Calls retrieveData(vpdObj), than deep copy the picture to dObj of right type and size
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        void updateParameters(QMap<QString, ito::Param> params);

        //! Slot to run special function
        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond);

        //! Slot to update the lightsource and integrationtime depended shading correction
        void updateShadingCorrection(int value);

        //! Slot to eanble the lightsource and integrationtime depended shading correction
        void activateShadingCorrection(bool enable);

	private slots:

};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    XimeaInterface
  *
  *\brief    Interface-Class for Ximea-Class
  *
  *    \sa    AddInDataIO, Ximea
  *    \date    04.07.2012
  *    \author  CK, Ly
  * \warning    NA
  *
  */
class XimeaInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)

    PLUGIN_ITOM_API

    protected:

    public:
        XimeaInterface();
        ~XimeaInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);


    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // Ximea_H
