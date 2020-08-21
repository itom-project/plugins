/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef PLUGINVERSION_H
#define PLUGINVERSION_H

#include "itom_sdk.h"

#define PLUGIN_VERSION_MAJOR 0
#define PLUGIN_VERSION_MINOR 0
#define PLUGIN_VERSION_PATCH 1
#define PLUGIN_VERSION_REVISION 0
#define PLUGIN_VERSION        CREATE_VERSION(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_VERSION_STRING CREATE_VERSION_STRING(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_COMPANY        "Trumpf Lasersystems For Semiconductor Manufacturing Gmbh, Ditzingen"
#define PLUGIN_AUTHOR         "J. Krauter, Trumpf Lasersystems For Semiconductor Manufacturing Gmbh"
#define PLUGIN_COPYRIGHT      "(C) 2020, Trumpf Lasersystems For Semiconductor Manufacturing Gmbh"
#define PLUGIN_NAME           "OphirLMMeasurement"

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PLUGINVERSION_H
