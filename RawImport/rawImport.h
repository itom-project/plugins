/* ********************************************************************
    Plugin "rawImport" for itom software
    URL: http://www.lccv.ufal.br/
    Copyright (C) 2016, Laboratorio de Computacao Cientifica e Visualzacao,
    Universidade Federal de Alagoas (UFAL), Brasil

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

#ifndef RAWIMPORT_H
#define RAWIMPORT_H

#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
/** \file rawImport.h
*   \brief  Header for the rawImport filter / algorithm function(s)
*   \detail This is a plugin to load files from (dslr) cameras based on the dcraw program included in this
*           plugin. Dcraw was left as untempered as possible.
*
*   \author LCCV, Universidade Federal de Alagoas (UFAL)
*   \date 01/2016
*/

//----------------------------------------------------------------------------------------------------------------------------------
/** @class RawImportInterface
*   @brief short description
*
*   AddIn Interface for the RawImport class s. also \ref RawImport
*/
class RawImportInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        RawImportInterface();
        ~RawImportInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class RawImport
*   @brief short description
*
*   longer description
*/
class RawImport : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        RawImport();
        ~RawImport();

    public:
        friend class RawImportInterface;

    //----------------------------------------------------------------------------------------------------------------------------------
    //---------------------------------------------------------User-Defined-Content-----------------------------------------------------
        QString GetTempDir();
        static ito::RetVal loadImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal loadImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
    //---------------------------------------------------------End-User-Defined-Content-------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------------------

    private:

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // RAWIMPORT_H
