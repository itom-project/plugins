/* ********************************************************************
    Grabber plugin for LIVOX LIDAR and its Livox SDK

*********************************************************************** */

#ifndef PLUGINVERSION_H
#define PLUGINVERSION_H

#include "itom_sdk.h"

#define PLUGIN_VERSION_MAJOR 0
#define PLUGIN_VERSION_MINOR 0
#define PLUGIN_VERSION_PATCH 1
#define PLUGIN_VERSION_REVISION 1
#define PLUGIN_VERSION        CREATE_VERSION(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_VERSION_STRING CREATE_VERSION_STRING(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_COMPANY        "ITO Universitaet Stuttgart"
#define PLUGIN_AUTHOR         "Faulhaber, A."
#define PLUGIN_COPYRIGHT      "Copyright by author"
#define PLUGIN_NAME           "Livox"

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PLUGINVERSION_H
