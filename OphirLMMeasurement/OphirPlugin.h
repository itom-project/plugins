/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef OPHIRPLUGIN_H
#define OPHIRPLUGIN_H

#include "OphirLMMeasurement.h"

#include "common/addInInterface.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    OphirPluginInterface 
  *
  *\brief    Interface-Class for OphirPlugin-Class
  *
  *    \sa    AddInDataIO
  *
  */
class OphirPluginInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        OphirPluginInterface();
        ~OphirPluginInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    OphirPlugin

  */
class OphirPlugin : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        //! Destructor
        ~OphirPlugin();
        //! Constructor
        OphirPlugin();
        
    public:
        friend class OphirPluginInterface;
 
    private:
        OphirLMMeasurement m_OphirLM;
        static QList<std::wstring> openedDevices;
        long m_handle;
        bool m_opened;
        std::wstring m_serialNo;
        char *m_charBuffer;

        ito::RetVal checkError(const int &e, const char *message);
        char* wCharToChar(const wchar_t *input);

        void dataReadyCallback(long hDevice, long channel);

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

#endif // OPHIRPLUGIN_H
