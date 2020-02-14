/* ********************************************************************
Plugin "IntelRealSense" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut fuer Technische Optik (ITO),
Universitaet Stuttgart, Germany

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

#ifndef INTELREALSENSE_H
#define INTELREALSENSE_H

#include <rs.hpp>

#include "common/addInGrabber.h"
#include "opencv/cv.h"
#include <qsharedpointer.h>
#include "dialogIntelRealSense.h"




//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    IntelRealSenseInterface 
  *
  *\brief    Interface-Class for IntelRealSense-Class
  *
  *    \sa    AddInDataIO, IntelRealSense
  *
  */
class IntelRealSenseInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        IntelRealSenseInterface();
        ~IntelRealSenseInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
class IntelRealSense : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~IntelRealSense();
        IntelRealSense();
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        
    public:
        friend class IntelRealSenseInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog
        

    private:
        bool m_isgrabbing; /*!< Check if acquire was executed */
        char* bufferPtr; //this can be a pointer holding the image array from the camera. This buffer is then copied to the dataObject m_data (defined in AddInGrabber)

		rs2_stream *s_streamType;
		rs2::pipeline * m_pPipe = new rs2::pipeline;
        rs2::frameset * m_pFrameset = new rs2::frameset;
        rs2::frame * m_pFrame = new rs2::frame;

      /*  enum Mode {
            M_default,
            M_left,
            M_right,
            M_stereo,
            M_color
        };*/
        cv::Mat m_alphaChannel; /* simple uint8, 1-channel image with 255 values filled in case of colorMode. This is the alpha plane */

        QString * m_pMode = new QString;
        bool isFilter;


        //enum SyncParams : unsigned long {
        //    sRoi = 0x0001,
        //    sBpp = 0x0002,
        //    sFps = 0x0004,
        //    sRes = 0x0008,
        //    sDoDepth = 0x0010,
        //    sAll = sRoi | sBpp | sFps | sRes
        //};
        //int res[2];
		//ito::RetVal syncParams(SyncParams what = sAll);
		//template<typename _Tp>ito::RetVal getParamInfo(_Tp &min, _Tp &max, _Tp &inc, _Tp &value, const char* name);
		//*m_pParamsObj;

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
        
        //checkData usually need not to be overwritten (see comments in source code)
        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // IntelRealSense_H
