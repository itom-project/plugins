/* ********************************************************************
    Plugin "OceanOpticsSpec" for itom software
    URL: http://www.bitbucket.org/itom/plugins
    Copyright (C) 2020, Institut fuer Technische Optik, Universitaet Stuttgart

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
//'\xa0f1' --> so compilers know its not an int

#ifndef OCEANSTS_H
#define OCEANSTS_H

#include "common/addInGrabber.h"
#include "dialogOceanOpticsSpec.h"

#include <qsharedpointer.h>

#include "OceanOpticsSpecDefines.h"


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    OceanOpticsSpec 
  */
class OceanOpticsSpec : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~OceanOpticsSpec();
        //! Constructor
        OceanOpticsSpec();

    public:
        friend class OceanOpticsSpecInterface;
        const ito::RetVal showConfDialog(void);    /*!< Open the config nonmodal dialog to set camera parameters */

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);    /*! <Wait for acquired picture */
        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);
        //        ito::RetVal checkData(void);    /*!< Check if objekt has to be reallocated */

    private:
        ito::RetVal sendCommand(unsigned char* cmd, int cmd_size, unsigned char* buf, int &buf_size, char *immediate_data = nullptr, int dlen = -1);
        ito::RetVal buildheader(char **headout, OcHeader *headin, unsigned char* cmd, int cmd_len, char *data = nullptr, int dlen = -1, bool command = true /*command or query*/);
        ito::RetVal readWithFixedLength(unsigned char* buf, int &buf_size);
        ito::RetVal checkAnswerForError(unsigned char* buf, const unsigned char &desiredCmd, bool warningNotError = false, const char *prefix = "");
        void dummyRead();

        ito::AddInDataIO *m_pUsb;
        bool m_isGrabbing;
        // DEVICE SPECIFIC read and config data
        
        OcSingleMeasdata singleMeasdata;
        //OcMultiMeasdata multiMeasdata;
        DeviceConfigType m_deviceConfig;
        MeasConfigType m_measConfig;
        OcHeader m_header;
        
        //
        ito::RetVal m_acquisitionRetVal;
        int m_numberDeadPixels; //this depends on the detector! some detectors don't have deadpixels, which are located at the start of the pixel stream. The list of detectors and numbers of dead pixels was provided by Avantes.
        int m_numberOfCorrectionValues;
        int m_startCorrectionIndex;

        static void idleCharDeleter(char* /*v*/) {};
        static void idleIntDeleter(int* /*v*/) {};

    signals:
        void parametersChanged(QMap<QString, ito::Param> params);    /*! Signal send changed or all parameters to listeners */

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
        //! Calls retrieveData(NULL), than copy the picture to dObj of right type and size
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        //! Deep copy the camera buffer to dObj. Object must be of right size and type. If liveData is running, a second deep-copy is performed to copy data to the grabber 
        ito::RetVal copyVal(void *dObj, ItomSharedSemaphore *waitCond);
        
        //! Retrieve new offset and new gain and give them to the camera dll
        void updateParameters(QMap<QString, ito::ParamBase> params);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    OceanOpticsSpecInterface_STS
  *
  *\brief    Interface-Class for STS-Class
  *
  *    \sa    AddInDataIO, STSUV
  *    \date    09.10.2020
  *    \author    Johannes Drozella
  * \warning    NA
  *
  */
class OceanOpticsSpecInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        OceanOpticsSpecInterface();
        ~OceanOpticsSpecInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
        //! auto-increment, static instance counter for all dummy-1394 instances
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // OCEANSTS_H
