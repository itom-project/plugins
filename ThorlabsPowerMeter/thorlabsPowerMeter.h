/* ********************************************************************
itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut fuer Technische Optik (ITO),,
Universit�t Stuttgart, Germany

This file is part of itom and its software development toolkit (SDK).

itom is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

In addition, as a special exception, the Institut f�r Technische
Optik (ITO) gives you certain additional rights.
These rights are described in the ITO LGPL Exception version 1.0,
which can be found in the file LGPL_EXCEPTION.txt in this package.

itom is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef THORLABSPOWERMETER_H
#define THORLABSPOWERMETER_H

#include "common/addInInterface.h"
#include "dialogThorlabsPowerMeter.h"
#include "DataObject/dataobj.h"
#if defined(USE_API_1_0_2)
    #include "visa.h"
#else
    #include "visatype.h"
#endif

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    ThorlabsPowerMeterInterface
  *
  *\brief    Interface-Class for MyGrabber-Class
  *
  *    \sa    AddInDataIO, MyGrabber
  *
  */
class ThorlabsPowerMeterInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        ThorlabsPowerMeterInterface();
        ~ThorlabsPowerMeterInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabber

  */
class ThorlabsPowerMeter : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        //! Destructor
        ~ThorlabsPowerMeter();
        //! Constructor
        ThorlabsPowerMeter();

        //ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */

    public:
        friend class ThorlabsPowerMeterInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);
        ito::RetVal checkFunctionCompatibility(bool* compatibility);

    private:
        bool m_isgrabbing; /*!< Check if acquire was executed */
        ito::RetVal checkError(ViStatus err);
        ViSession m_instrument;
        ito::DataObject m_data;

        enum SyncParams{
            bWavelength = 0x0001,
            bAttenuation = 0x0002,
            bDarkOffset = 0x0004,
            bLineFrequency = 0x008,
            bPowerRange = 0x0010,
            bAutoRange = 0x0020,
            bMeasurementMode = 0x0040,
            bPowerReference = 0x0080,
            bBandwidth = 0x0100,
            bAll = bWavelength | bAttenuation | bDarkOffset | bLineFrequency | bPowerRange | bAutoRange | bMeasurementMode | bPowerReference | bBandwidth
        };
        ito::RetVal synchronizeParams(int what = bAll);


    public slots:
        //!< Get Camera-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set Camera-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //!< Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //!< Stop the camera to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //!< Softwaretrigger for the camera
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //!< Wait for acquired picture, copy the picture to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        ito::RetVal acquireAutograbbing(QSharedPointer<double> value, ItomSharedSemaphore *waitCond);
        ito::RetVal zeroDevice(ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);

        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
    signals:
        void visibilityChanged(bool visible);
};

#endif // MYGRABBER_H
