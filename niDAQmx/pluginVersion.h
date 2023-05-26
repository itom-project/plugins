/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef PLUGINVERSION_H
#define PLUGINVERSION_H

#define PLUGIN_VERSION_MAJOR 1
#define PLUGIN_VERSION_MINOR 0
#define PLUGIN_VERSION_PATCH 1
#define PLUGIN_VERSION_REVISION 0

#define PLUGIN_VERSION        CREATE_VERSION(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_VERSION_STRING CREATE_VERSION_STRING(PLUGIN_VERSION_MAJOR,PLUGIN_VERSION_MINOR,PLUGIN_VERSION_PATCH)
#define PLUGIN_COMPANY        "ITO Uni Stuttgart"
#define PLUGIN_AUTHOR         "Martin Hoppe, ITO, University Stuttgart; Dan Nessett, Unaffiliated; Marc Gronle, Unaffiliated"
#define PLUGIN_COPYRIGHT      "(C) 2020, ITO Uni Stuttgart"
#define PLUGIN_NAME           "NI-DAQmx"

//----------------------------------------------------------------------------------------------------------------------------------

#endif // PLUGINVERSION_H
