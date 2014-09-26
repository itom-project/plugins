#define _SCL_SECURE_NO_WARNINGS (1)
#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

// main-include
#include "IDSInterface.h"

// standard-includes

// qt-core stuff
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QtPlugin>
#include <QtCore/QMetaObject>

// ids-includes
#include "IDS/uEye.h"
#include "IDS/version.h"

// project-includes
#include "pluginVersion.h"
#include "IDSuEye.h"

namespace
{
    /**
     * @brief   Extracts the given number of bits from the given position
     * @tparam  bitPosition The start position of the bit sequence.
     * @tparam  bits        The number of bits.   
     * @param   x           The value from which bits shall be extracted.
     * @returns The extracted bits shifted to lowest position, all other stuff is erased.
     **/
    template < size_t bitPosition, size_t bits >
    unsigned int get( unsigned int x )
    {
        static_assert( (bitPosition+bits) <= (sizeof(x)*8), "Accessing invalid bits!" );

        return ( x >> bitPosition ) & ( (0x1<<bits) - 1 );
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IDSInterface::IDSInterface(QObject *parent)
{
    m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("IDSuEye");

    m_description = QObject::tr("IDS uEye grabber.");

    const unsigned int libVersion = UEYE_VERSION_CODE;

    int major = get<24,8>(libVersion);
    int minor = get<16,8>(libVersion);
    

    char docstring[] = \
        "This plugin supports IDS uEye cameras and has currently been tested with the following models: \n\
- UI145xSE-C (colored, USB2). \n\
\n\
The plugin has been compiled using the IDS library version %1.%2. \n\
\n\
In order to run your camera, please install the SDK imaging software in the right version such that the necessary drivers are installed. \n\
\n\
The first draft of this plugin has been implemented by Pulsar Photonics GmbH; further work has been done by ITO, University of Stuttgart."; 
    m_detaildescription = tr(docstring).arg(major).arg(minor);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = tr("Licensed under LGPL");
    m_aboutThis = tr( "N.A." );  

    ito::Param param( "camera_id", ito::ParamBase::Int | ito::ParamBase::In, 0, 254, 0, tr("Camera ID of the camera to open (0: the next free camera will opened [default], 1-254: else)").toLatin1().data());
    m_initParamsOpt.append(param);
    

    //the following part would modify the meta information of the initialization parameter, but it requires to open the IDS dll even if it is not used.
    /*ito::RetVal retval = checkVersionConsistency();
    if (!retval.containsError())
    {
        ito::StringMeta *camerasSelectionValues = new ito::StringMeta( ito::StringMeta::String );
        IDSCameras &rCameras = IDSCameras::getInstance( );
        IDSCameras::iterator it = rCameras.begin();
        while (it != rCameras.end())
        {
            QString id = (*it)->getID().getNumericID();
            camerasSelectionValues->addItem(id.toLatin1().data());
            ++it;
        }
        param.setMeta(camerasSelectionValues, true);
    }*/
}

//----------------------------------------------------------------------------------------------------------------------------------
IDSInterface::~IDSInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IDSInterface::getAddInInst( ito::AddInBase **addInInst )
{
    if ( !addInInst )
    {
        return ito::retError;
    }

    ito::RetVal retval = checkVersionConsistency();
    if (!retval.containsError())
    {
        NEW_PLUGININSTANCE(IDSuEye)
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IDSInterface::closeThisInst( ito::AddInBase **addInInst )
{
   REMOVE_PLUGININSTANCE(IDSuEye)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IDSInterface::checkVersionConsistency()
{
    //check consistency of library version vs. dll-version:
    const unsigned int dllVersion = is_GetDLLVersion();
    const unsigned int libVersion = UEYE_VERSION_CODE;

    if (get<24,8>(dllVersion) != get<24,8>(libVersion) || get<16,8>(dllVersion) != get<16,8>(libVersion)) //build is ignored: || get<0,16>(dllVersion) != get<0,16>(libVersion))
    {
        return ito::RetVal::format(ito::retError, 0, "IDS library version mismatch. Expected version %i.%i.%i, got %i.%i.%i", \
            get<24,8>(libVersion), get<16,8>(libVersion), get<0,16>(libVersion), \
            get<24,8>(dllVersion), get<16,8>(dllVersion), get<0,16>(dllVersion));
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class IDSInterface with the name IDSuEye as plugin for the Qt-System (see Qt-DOC)
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(IDSuEye, IDSInterface)
#endif