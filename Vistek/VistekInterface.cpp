#include "Vistek.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"
#include "pluginVersion.h"
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
#include "dockWidgetVistek.h"

//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class VistekInterface
    \brief Small interface class for class Vistek. This class contains basic information about Vistek and can
        create one or more new instances of Vistek.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! Creates a new instance of Vistek and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created Vistek-instance is stored in *addInInst
    \return ito::RetVal retOk
    \sa Vistek
*/
ito::RetVal VistekInterface::getAddInInst(ito::AddInBase **addInInst)
{
    Vistek* newInst = new Vistek();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Deletes an instance of Vistek. The instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return ito::RetVal retOk
    \sa Vistek
*/
ito::RetVal VistekInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
      delete ((Vistek *)*addInInst);
      int idx = m_InstList.indexOf(*addInInst);
      m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor for VistekInterface
/*!
    Defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: Vistek) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.

    \param [in] parent is the plugin interface's parent object
*/
VistekInterface::VistekInterface(QObject *parent)
{
	m_autoLoadPolicy = ito::autoLoadNever;
    m_autoSavePolicy = ito::autoSaveNever;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("Vistek");

    m_description = QObject::tr("SVS Vistek GigE grabber.");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char* docstring = \
"itom plugin for GigE cameras from SVS Vistek. Every camera is simply initialized by the serial number of the connected SVS Vistek camera. \
(see camera housing). \n\
\n\
Some files of the SVGigE SDK are shipped within this plugin (currently 1.4.24). Please check the SVS Vistek website for newer versions of the SDK \
and replace the files if desired. Additionally, it is stated that SVS Vistek does not provide any support for this specific plugin wrapping the \
official SDK of SVS Vistek.";
	m_detaildescription = QObject::tr(docstring);

    m_detaildescription = QObject::tr("SVS Vistek GigE camera grabber.");
    m_author = "H. Gilbergs, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL, the necessary Vistek drivers, header files and libraries have their own license.");
    m_aboutThis = QObject::tr("N.A.");  
    
    m_initParamsMand.clear();
	m_initParamsOpt.clear();

	ito::Param p = ito::Param("CameraSerialNo", ito::ParamBase::String | ito::ParamBase::In, "", QObject::tr("Serial Number of the SVS Vistek camera (see camera housing)").toAscii().data());
	ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::RegExp, "^[0-9]*$");
	p.setMeta(m, true);
	m_initParamsOpt << p;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor for VistekInterface
/*!
    Clears m_initParamsMand and m_initParamsOpt.
*/
VistekInterface::~VistekInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class VistekInterface with the name Vistekinterface as plugin for the Qt-System (see Qt-DOC)
Q_EXPORT_PLUGIN2(Vistekinterface, VistekInterface)