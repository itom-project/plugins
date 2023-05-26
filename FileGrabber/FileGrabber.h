/* ********************************************************************
    Plugin "FileGrabber" for itom software
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

#ifndef FILEGRABBER_H
#define FILEGRABBER_H

#include "common/addInGrabber.h"
#include "dialogFileGrabber.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

//----------------------------------------------------------------------------------------------------------------------------------
class FileGrabberInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this FileGrabberInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        FileGrabberInterface();                    /*!< Constructor */
        ~FileGrabberInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of FileGrabber and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of FileGrabber, given by *addInInst */

        //! auto-increment, static instance counter for all File-grabber instances
        static int m_instCounter;
};

//----------------------------------------------------------------------------------------------------------------------------------
class FileGrabber : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~FileGrabber();
        FileGrabber();

//        ito::RetVal checkData();
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class FileGrabberInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_isgrabbing;
        ito::DataObject m_preloadedObject;
        QStringList m_fileList;
        QDir m_searchFolder;
        bool m_fromStack;

        double m_curTick;
        double m_lastTick;

        template<typename _Tp> inline ito::RetVal transferDataFromStack(const int current_image, const bool hasListeners, const bool copyExternal, ito::DataObject *externalDataObject);

    signals:

    public slots:
        /*ito::RetVal getParam(const char *name, QSharedPointer<char> val, QSharedPointer<int> len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getParam(const char *name, QSharedPointer<double> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const char *val, const int len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const double val, ItomSharedSemaphore *waitCond = NULL);*/

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        //ito::RetVal setVal(const void *dObj, const int length, ItomSharedSemaphore *waitCond);

        //void dataParametersChanged(int sizex, int sizey, int bpp);
        void GainOffsetPropertiesChanged(double gain, double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    private slots:
        ito::RetVal updateCamParams(void);


};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // FileGRABBER_H
