/* ********************************************************************
    Plugin "OpenCV-Filter" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#ifndef OPENCVFILTERSNONFREE_H
#define OPENCVFILTERSNONFREE_H

#include "common/addInInterface.h"

#include "pluginVersion.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <qsharedpointer.h>


//----------------------------------------------------------------------------------------------------------------------------------
class OpenCVFiltersNonFreeInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        OpenCVFiltersNonFreeInterface();       /*! <Class constructor */
        ~OpenCVFiltersNonFreeInterface();      /*! <Class destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*! <Create a new instance of OpenCVFilter-Class */

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*! <Destroy the loaded instance of OpenCVFilter-Class */

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
class OpenCVFiltersNonFree : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        OpenCVFiltersNonFree();    /*! <Class constructor */
        ~OpenCVFiltersNonFree();               /*! <Class destructor */

    public:
        friend class OpenCVFiltersNonFreeInterface;

#if (CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

        static const QString cvSiftDetectorDescriptorExtractorDoc;
        static ito::RetVal cvSiftDetectorDescriptorExtractor(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal cvSiftDetectorDescriptorExtractorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
#endif //(CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3)

    private:

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> /*val*/, ItomSharedSemaphore * /*waitCond*/) { return ito::retOk; }
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> /*val*/, ItomSharedSemaphore * /*waitCond*/) { return ito::retOk; }
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // OPENCVFILTERSNONFREE_H
