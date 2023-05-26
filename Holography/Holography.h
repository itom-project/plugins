/* ********************************************************************
    Plugin "FringeProj" for itom software
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

#ifndef HOLOGAPHY_H
#define HOLOGAPHY_H

#ifdef USEOPENMP
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

#include "common/addInInterface.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class HolographyInterface
*   @brief Algorithms used for holographic systems
*
*   AddIn Interface for the Holography class s. also \ref Holography
*/
class HolographyInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        HolographyInterface();
        ~HolographyInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class Holography
*   @brief Algorithms used for holographic systems
*
*   In this class the algorithms used for holographic optical systems. Currently this are basically propagation
    algorithms (e.g. Rayleigh-Sommerfeld and Fresnel) to gain a speed up compard to a python implementation.
    Maybe in some (soon) future other parts will be included here as phase distribution calculations for Zernike
    coefficients, different hologram optimization algorithms (dbs, gs, ...), ... ?
*/
class Holography : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        Holography();
        ~Holography();

    public:
        friend class HolographyInterface;

        static const QString FresnelCalcPropDoc;
        static ito::RetVal FresnelCalcProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal FresnelCalcPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString FresnelDoPropDoc;
        static ito::RetVal FresnelDoProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal FresnelDoPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString RSCalcPropDoc;
        static ito::RetVal RSCalcProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal RSCalcPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString RSDoPropDoc;
        static ito::RetVal RSDoProp(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal RSDoPropParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // HOLOGRAPHY_H
