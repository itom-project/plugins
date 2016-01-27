/* ********************************************************************
    Plugin "rawImport" for itom software
    URL: http://www.lccv.ufal.br/
    Copyright (C) 2016, Laboratorio de Computacao Cientifica e Visualzacao,
    Universidade Federal de Alagoas (UFAL), Brasil

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

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "rawImport.h"

#include "DataObject/dataobj.h"
#include "pluginVersion.h"
#include <QtCore/QtPlugin>
#include <qprocess.h>
#include <qmenu.h>
#include <qcoreapplication.h>
#include <qtemporarydir.h>
#include "common/apiFunctionsInc.h"

//----------------------------------------------------------------------------------------------------------------------------------
RawImportInterface::RawImportInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("rawImport");

    m_description       = QObject::tr("RawImport to load raw format images from (dslr) cameras");
    m_detaildescription = QObject::tr("");
    m_author            = "Laboratorio de Computacao Cientifica e Visualizacao (LCCV), Universidade Federal de Alagoas (UFAL)";
    m_license           = QObject::tr("LGPL");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr("");        
}

//----------------------------------------------------------------------------------------------------------------------------------
RawImportInterface::~RawImportInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal RawImportInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(RawImport)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal RawImportInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(RawImport)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(rawimportinterface, RawImportInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
RawImport::RawImport() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
/** initialize filter functions within this addIn
*    @param [in]    paramsMand    mandatory parameters that have to passed to the addIn on initialization
*    @param [in]    paramsOpt    optional parameters that can be passed to the addIn on initialization
*    @return                    retError in case of an error
*
*    Here are the filter functions defined that are available through this addIn.
*    These are:
*       - filterName    description for this filter
*
*   This plugin additionally makes available the following widgets, dialogs...:
*       - dialogName    description for this widget
*/
ito::RetVal RawImport::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;
    //AlgoWidgetDef *widget = NULL;

    //----------------------------------------------------------------------------------------------------------------------------------
    //---------------------------------------------------------User-Defined-Content-----------------------------------------------------
 
    filter = new FilterDef(RawImport::loadImage, RawImport::loadImageParams, tr("import filter for (dslr) raw images based on dcraw"), ito::AddInAlgo::catNone, ito::AddInAlgo::iNotSpecified);
    m_filterList.insert("loadRawImage", filter);

    //---------------------------------------------------------End-User-Defined-Content-------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------------------

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal RawImport::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval = ito::retOk;

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
QString RawImport::GetTempDir()
{
    if (!m_TempDir)
        m_TempDir = new QTemporaryDir();
    return m_TempDir->path();
}

//----------------------------------------------------------------------------------------------------------------------------------
/** parameters for calling loadRawImage copied form the template "filterParams" and modified
*    @param [in]    paramsMand    mandatory parameters for calling the corresponding filter
*    @param [in]    paramsOpt    optional parameters for calling the corresponding filter
*
*    mand. Params:
*
*   opt. Params: NONE

*/
ito::RetVal RawImport::loadImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> * paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        paramsMand->clear();
        ito::Param param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("file to open").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("image", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty object, will contain 2D image later").toLatin1().data());
        paramsMand->append(param);

        paramsOpt->clear();
        param = ito::Param("arguments", ito::ParamBase::String | ito::ParamBase::In, "-D -j -t 0", tr("commandline arguments for dcraw, default is -D -j -t0, i.e. raw data no scaling").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("usetmpdir", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("do temporal unpacking in temporary directory").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/** RawImport-filter to show how to capture a single image from a camera with an algorithm
*    @param [in]    paramsMand    mandatory parameters
*    @param [in]    paramsOpt    optional parameters
*
*    This algotihmes uses the ito::threadCamera-class defined in "common/helperGrabber.h" to get a deep copy of a camera image. The camera is living in another thread.
*
*   \author LCCV, Universidade Federal de Alagoas (UFAL)
*   \date 01.2016
*/
ito::RetVal RawImport::loadImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal retval = ito::retOk;
    QTemporaryDir *tmpDir = NULL;

    QString filename = QString::fromLatin1((*paramsMand)[0].getVal<char*>());
    QString filenameLoad;
    QFileInfo ofileinfo(filename);
    if (!ofileinfo.exists())
    {
        return ito::RetVal(ito::retError, 0, tr("File not found").toLatin1().data());
    }

    ito::DataObject *image = static_cast<ito::DataObject*>((*paramsMand)[1].getVal<void*>());
    if (image == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Image handle empty").toLatin1().data());
    }

    QString arguments = QString::fromLatin1((*paramsOpt)[0].getVal<char*>());

    ito::DataObject dObj;   // create an mepty object

    /**/
    if ((*paramsOpt)[1].getVal<int>())
    {
        tmpDir = new QTemporaryDir();
        QString tmpFilename(filename);
        if (tmpDir->path().lastIndexOf("/") < tmpDir->path().length() - 1
            && tmpDir->path().lastIndexOf("\\") < tmpDir->path().length() - 1)
        {
            filename = tmpDir->path() + "/" + ofileinfo.baseName() + "." + ofileinfo.completeSuffix();
            filenameLoad = tmpDir->path() + "/" + ofileinfo.completeBaseName() + ".pgm";
        }
        else
        {
            filename = tmpDir->path() + ofileinfo.baseName() + "." + ofileinfo.completeSuffix();
            filenameLoad = tmpDir->path() + ofileinfo.completeBaseName() + ".pgm";
        }
        QFile::copy(tmpFilename, filename);
        ofileinfo = QFileInfo(filename);
    }
    else
    {
        if (ofileinfo.path().lastIndexOf("/") < ofileinfo.path().length() - 1
            && ofileinfo.path().lastIndexOf("\\") < ofileinfo.path().length() - 1)
        {
            filenameLoad = ofileinfo.path() + "/" + ofileinfo.completeBaseName() + ".pgm";
        }
        else
        {
            filenameLoad = ofileinfo.path() + ofileinfo.completeBaseName() + ".pgm";
        }
    }

    QProcess *readProc = new QProcess(NULL);
    QString command(QCoreApplication::applicationDirPath());
#ifdef WIN32
    command += QString("/lib/dcraw.exe ") + arguments + " " + filename;
#else
        command += QString("/lib/dcraw ") + arguments + " " + filename;
#endif
    readProc->start(command);

//    readProc->setReadChannel(QProcess::StandardOutput);

//    QObject::connect(consoleProc, &QProcess::readyReadStandardOutput, &RawImport::readCmdOutput);
    readProc->waitForFinished();

    QVector<ito::ParamBase> filterParamsMand(0);
    QVector<ito::ParamBase> filterParamsOpt(0);
    QVector<ito::ParamBase> filterParamsOut(0);

    retval += apiFilterParamBase("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
    filterParamsMand[0].setVal<char*>((char*)image);
    filterParamsMand[1].setVal<char*>(filenameLoad.toLatin1().data());
    if (!retval.containsWarningOrError())
    {
        retval += apiFilterCall("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
    }

    if (tmpDir)
        delete tmpDir;

    if (!retval.containsError())
    {
        QString msg = tr("imported by dcraw import filter");
        image->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
