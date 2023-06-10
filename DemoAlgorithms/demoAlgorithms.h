/* ********************************************************************
    Plugin "demoAlgorithms" for itom software
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

#ifndef DEMOALGORITHMS_H
#define DEMOALGORITHMS_H

#include "common/addInInterface.h"
//----------------------------------------------------------------------------------------------------------------------------------
/** \file demoAlgorithms.h
*   \brief  Header of a demoApplication to show plugIn-Developer how to write algorithms
*   \detail This file is the main header of the demoAolgorithms-DLL which is programmed to explain plugin-developers how to write an
*           algorithm. It was created with the PluginCreationWizard written by Marc Gronle. The user-defined content is highlighted.
*
*   \author Wolfram Lyda (ITO)
*   \date 04.2012
*/

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DemoAlgorithmsInterface
*   @brief short description
*
*   AddIn Interface for the DemoAlgorithms class s. also \ref DemoAlgorithms
*/
class DemoAlgorithmsInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        DemoAlgorithmsInterface();
        ~DemoAlgorithmsInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DemoAlgorithms
*   @brief short description
*
*   longer description
*/
class DemoAlgorithms : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        DemoAlgorithms();
        ~DemoAlgorithms() {};

    public:
        friend class DemoAlgorithmsInterface;

        static QWidget* dialog(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, ito::RetVal &retValue);
        static ito::RetVal dialogParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);


    //----------------------------------------------------------------------------------------------------------------------------------
    //---------------------------------------------------------User-Defined-Content-----------------------------------------------------
        static ito::RetVal demoTestActuator(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal demoTestActuatorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal demoMoveActuator(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal demoMoveActuatorParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal demoSnapImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal demoSnapImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal demoSnapMovie(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal demoSnapMovieParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static ito::RetVal demoCancellationFunction(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, QSharedPointer<ito::FunctionCancellationAndObserver> observer);
        static ito::RetVal demoCancellationFunctionParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

		static QWidget* demoWidget(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ito::RetVal &retValue);
		static ito::RetVal demoWidgetParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
        static QWidget* demoAlgoCancelWidget(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ito::RetVal &retValue);
        static ito::RetVal demoAlgoCancelWidgetParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);
    //---------------------------------------------------------End-User-Defined-Content-------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------------------

    private:

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // DEMOALGORITHMS_H
