/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef THORLABSFF_H
#define THORLABSFF_H

#include "common/addInInterface.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    ThorlabsFFInterface
  *
  *\brief    Interface-Class for ThorlabsFF-Class
  *
  *    \sa    AddInDataIO
  *
  */
class ThorlabsFFInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        ThorlabsFFInterface();
        ~ThorlabsFFInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    ThorlabsFF

  */
class ThorlabsFF : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        //! Destructor
        ~ThorlabsFF();
        //! Constructor
        ThorlabsFF();

    public:
        friend class ThorlabsFFInterface;

    private:
        static QList<QByteArray> openedDevices;
        bool m_opened;
        char m_serialNo[16];

        ito::RetVal checkError(short value, const char *message);

    public slots:
        //!< Get Camera-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set Camera-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // THORLABSFF_H
