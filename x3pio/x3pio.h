/* ********************************************************************
    Plugin "x3pio" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2016, Institut für Technische Optik, Universität Stuttgart

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

#ifndef X3PIO_H
#define X3PIO_H

#include "common/addInInterface.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DataObjectIOInterface
*   @brief DataIO functions implemented in former ito mcpp measurement program
*
*   AddIn Interface for the MCPPDataIO class s. also \ref MCPPDataIO
*/
class X3pIOInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        X3pIOInterface();
        ~X3pIOInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DataObjectIO
*   @brief Algorithms used to process images and dataobjects with filters provided by openCV
*
*   In this class the algorithms used for the processing of images are implemented.
*   The filters wrap openCV-Filters to python interface. Handling of 3D-Objects differs depending on the filter.
*
*/
class X3pIO : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        X3pIO();
        ~X3pIO();

    public:
        friend class X3pIOInterface;
        static ito::RetVal saveDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static ito::RetVal loadDObj(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadDObjParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    private:
        static ito::RetVal parseUnit(const std::string &unitString, double &unitScale);

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // X3PIO_H
