/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#pragma once

#include "common/addInInterface.h"
#include <qsharedpointer.h>
#include <qevent.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    ThorlabsTCubeTECInterface 
  *
  *\brief    Interface-Class for ThorlabsTCubeTEC-Class
  *
  *    \sa    AddInDataIO
  *
  */
class ThorlabsTCubeTECInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        ThorlabsTCubeTECInterface();
        ~ThorlabsTCubeTECInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    ThorlabsTCubeTEC

  */
class ThorlabsTCubeTEC : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        //! Destructor
        ~ThorlabsTCubeTEC();
        //! Constructor
        ThorlabsTCubeTEC();

        void timerEvent(QTimerEvent *event);
        
    public:
        friend class ThorlabsTCubeTECInterface;
 
    private:
        static QList<QByteArray> openedDevices;
        static int numberOfKinesisSimulatorConnections;

        bool m_opened;
        char m_serialNo[16];
        int m_updateTimerId;

        ito::RetVal checkError(short value, const char *message);
        
    public slots:
        //!< Get Camera-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set Camera-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = nullptr);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};
