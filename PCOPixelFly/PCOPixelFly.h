/* ********************************************************************
    Plugin "PcoPixelFly" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#ifndef PCOPIXELFLY_H
#define PCOPIXELFLY_H

#include "common/addInGrabber.h"
#include "dialogPCOPixelFly.h"
#include <qsharedpointer.h>

#ifdef WIN32
    #include <Windows.h>
#endif

#define BUFFERNUMBER 1 //Maximal Number of Buffers: 32

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    PCOPixelFly
  *\brief    class to use a PCO Pixelfly QE CCD as an ITOM-Addin. Child of AddIn - Library (DLL) - Interface
  *
  *         This class can be used to work with a PCO Pixelfly QE CCD. It grabbes data with 8 or 12 Bit.
  *            The camera driver "pccam.dll" has to be installed in the system32-directory and the "pcocnv.dll" has to be in a subfolder .\PCO in the plugin directory
  *
  *    \sa    AddInDataIO, DummyGrabber
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class PCOPixelFly : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~PCOPixelFly();
        //! Constructor
        PCOPixelFly();

    public:
        friend class PCOPixelFlyInterface;
        const ito::RetVal showConfDialog(void);    /*!< Open the config nonmodal dialog to set camera parameters */
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);    /*! <Wait for acquired picture */
//        ito::RetVal checkData(void);    /*!< Check if object has to be reallocated */

        ito::RetVal libraryVersionNumber(const QByteArray &fileName, QString &version);

    private:

        HANDLE m_event[BUFFERNUMBER];    /*!< Events to check if a buffer was field by acquire-function. Used in getVal-Function. */
        HANDLE m_hdriver;    /*!< Handle to the specific Driver-Board. Has to be set to zero during first init. Up to 4 Boards */

        void* m_pAdr[BUFFERNUMBER];    /*!< Pointer to the different Buffers used in getVal-Function to copy data to dObj */
        void* m_pBWlut; /*!< Special Black-White Lookup-Table */

        int m_pictimeout;    /*!< Timeout for WaitGrab in getVal-function */
        int m_bufnumber[BUFFERNUMBER]; /*!< Buffer identifier for the CCD (see SDK) */
        int m_nextbuf; /*!< Next buf to fill / create (see SDK) */

        int m_verticalBinning;
        int m_horizontalBinning;

        int m_libraryMajor;

        bool m_isgrabbing; /*!< Check if acquire was called */
        bool m_saveParamsOnClose; /*!< Check if the parameters should be saved on close */

        bool m_waited[BUFFERNUMBER]; /*!< Checkvaraible for WaitGrab in getVal-function */

        ito::RetVal PCOLoadLibrary(void);    /*!< Loads the pccam.dll and pcocnv.dll and defines sdk-function-handles */
        ito::RetVal PCOChkError(int errornumber); /*!< Map PCO-Error-Number to ITOM-Errortype and Message */

        ito::RetVal PCORemoveFromList(void);    /*!< Removes all Buffers from list */
        ito::RetVal PCORemoveThisFromListIfError(int bufnr); /*!<Removes specific Buffer from list */
        ito::RetVal PCOAddToList(void);    /*!< Adds all buffers to bufferlist */
        ito::RetVal PCOResetEvents(void);    /*!< Resets the m_event-variable */
        ito::RetVal PCOAllocateBuffer(void); /*!< Creates the buffers and get a handle to their first pixel (m_pAdr[i]) */
        ito::RetVal PCOFreeAllocatedBuffer(void); /*!< Frees all buffers */


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

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    PCOPixelFlyInterface
  *
  *\brief    Interface-Class for PCOPixelFly-Class
  *
  *    \sa    AddInDataIO, PCOPixelFly
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class PCOPixelFlyInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        PCOPixelFlyInterface();
        ~PCOPixelFlyInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);


    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PCOPIXELFLY_H
