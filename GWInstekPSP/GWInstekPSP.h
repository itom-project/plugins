/* ********************************************************************
    Plugin "GWInstekPSP" for itom software
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

#ifndef GWINSTEKPSP_H
#define GWINSTEKPSP_H

#include "common/addInInterface.h"

#include "dialogGWInstekPSP.h"
#include "dockWidgetGWInstekPSP.h"

#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    GWInstekPSP
  *\brief    This class can be used to communicate with PSP-405, PSP-603 and PSP-2010
  *
  *         This class can be used to work with PSP-405, PSP-603 and PSP-2010.
  *            This system needs a serial port which following settings:
  *            baud = 2400
  *            bits = 8
  *            parity = 0
  *            stopbits = 1
  *            flow = 16 (dtr = enabled)
  *            endline = "\r"
  *
  * \todo write driver
  *    \sa    SerialIO, AddInActuator, DummyMotor, dialogGWInstekPSP, DockWidgetGWInstekPSP
  *    \date    25.11.2011
  *    \author    Heiko Bieger
  * \warning    NA
  *
  */
class GWInstekPSP : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~GWInstekPSP();
        GWInstekPSP();

    public:
        friend class GWInstekPSPInterface;
        friend class dialogGWInstekPSP;
        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog

    private:
        ito::AddInDataIO *m_pSer;
        char m_status[38];        //! Contains status string [Vvv.vvAa.aaaWwww.wUuuIi.iiPpppFffffff]
//                                                          0000000000111111111122222222223333333
//                                                          0123456789012345678901234567890123456

        const ito::RetVal SetParams();
        const ito::RetVal ReadFromSerial(bool *state);
        const ito::RetVal WriteToSerial(const char *text, bool commandHasAnswer, bool getCurrentStatus = true);
        static void doNotDelSharedPtr(char * /*ptr*/) {} /*!<workaround for deleter for QSharedPointer, such that the pointer is NOT deleted if shared-pointer's reference drops towards zero.*/

    public slots:
        void setParamVoltageFromWgt(double voltage);

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitConde = NULL);

        ito::RetVal getVal(char *data, int *len, ItomSharedSemaphore *waitCond);
        ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);

    private slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    GWInstekPSPInterface
  *
  *\brief    Interface-Class for GWInstekPSPInterface-Class
  *
  *    \sa    AddInActuator, GWInstekPSP
  * \warning    NA
  *
  */
class GWInstekPSPInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        GWInstekPSPInterface();
        ~GWInstekPSPInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);


    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // LEICAMF_H
