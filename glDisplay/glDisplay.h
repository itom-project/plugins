/* ********************************************************************
    Plugin "GLDisplay" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#ifndef GLDISPLAY_H
#define GLDISPLAY_H

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"
#include "dialogGLDisplay.h"
#include "dockWidgetGLDisplay.h"
#include "glWindow.h"

//----------------------------------------------------------------------------------------------------------------------------------
class GLDisplayInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5,0,0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        GLDisplayInterface();
        ~GLDisplayInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
class GLDisplay : public ito::AddInDataIO //, public DummyGrabberInterface
{
    Q_OBJECT

    private:
        GLWindow *m_pWindow;

		ito::RetVal constructionResult;
        int m_nrOfTextures;

    protected:
        ~GLDisplay();
        GLDisplay();

        ito::RetVal interpolateLUT(QVector<double> &grayvalues, QVector<unsigned char> &lut);

    public:
        friend class GLDisplayInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 0; }; //!< indicates that this plugin has got a configuration dialog

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitConde = NULL);

        void numberOfImagesChanged(int numImg, int numGray, int numCos);
        
        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // GLDISPLAY_H
