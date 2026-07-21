/* ********************************************************************
    Plugin "OpticalInspection" for itom software
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

#ifndef OPTICALINSPECTION_H
#define OPTICALINSPECTION_H

#include "common/addInGrabber.h"
#include "common/addInPublicGrabber.h"
#include "common/typeDefs.h"
#include "dialogOpticalInspection.h"

#include <qsharedpointer.h>
#include <qtimer.h>

//----------------------------------------------------------------------------------------------------------------------------------
class OpticalInspectionInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)                       /*!< this OpticalInspectionInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        OpticalInspectionInterface();                           /*!< Constructor */
        ~OpticalInspectionInterface();                          /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of OpticalInspection and returns this instance */

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of OpticalInspection, given by *addInInst */

};

//----------------------------------------------------------------------------------------------------------------------------------
class OpticalInspection : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~OpticalInspection();
        OpticalInspection();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class OpticalInspectionInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        ito::AddInPublicGrabber *m_pCam;
        ito::AddInActuator *m_pStage;
        bool m_isgrabbing;
        int64 m_startOfLastAcquisition;
        ito::uint8 m_totalBinning;
        bool m_lineCamera;
        int m_imageType;

        enum dummyImageType
        {
            imgTypeNoise,
            imgTypeGaussianSpot,
            imgTypeGaussianSpotArray,
        };

    signals:

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);

};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // OpticalInspection_H
