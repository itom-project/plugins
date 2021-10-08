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
#include "gitVersion.h"
#include <QtCore/QtPlugin>
#include <qprocess.h>
#include <qmenu.h>
#include <qcoreapplication.h>
#include <qtemporarydir.h>
#include "common/apiFunctionsInc.h"

#if WIN32
#include <Windows.h>
QString endline("\r\n");
#else
#include <unistd.h>
QString endline("\n");
#endif

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
    m_aboutThis         = QObject::tr(GITVERSION);        
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


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
RawImport::RawImport() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
RawImport::~RawImport()
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
ito::RetVal openExifTool(QString &filename, QProcess *&exifProc)
{
    QString exifCommand(QCoreApplication::applicationDirPath());
    if (exifProc == NULL)
    {
        exifProc = new QProcess(NULL);
#ifdef WIN32
        exifCommand = "\"" + exifCommand + QString("/lib/exiftool.exe\" -stay_open true -@ - ");
#else
        exifCommand += QString("/lib/exiftool -stay_open true -@ - ");
#endif
        exifProc->start(exifCommand, QList<QString>(), QIODevice::ReadWrite);
        exifProc->waitForStarted();
        // drain any possible input
        exifProc->waitForReadyRead(100);
        QString msg = exifProc->readAllStandardOutput();
        QString err = exifProc->readAllStandardError();
        if (err.length() > 0)
            return ito::RetVal(ito::retError, 0, (QObject::tr("Error opening exiftool: ").append(err)).toLatin1().data());
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
void closeExifTool(QProcess *&exifProc)
{
    if (exifProc)
    {
        QString command;
        command = "-stay_open" + endline;
        exifProc->write(command.toLatin1().data());
        command = "false" + endline;
        exifProc->write(command.toLatin1().data());
        exifProc->waitForBytesWritten();
        exifProc->close();
        Sleep(10);
        // make sure the process got killed
        exifProc->kill();
        delete exifProc;
        exifProc = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal readExifTag(QProcess *exifProc, QString filename, QString tagName, QString &Output, int readBinary = 1)
{
    QString readyStr = QString("{ready}").append(endline);
    // clear stale input
    exifProc->waitForReadyRead(100);
    Output = exifProc->readAllStandardOutput();
    QString command = "-" + tagName + endline;
    exifProc->write(command.toUtf8().data());
    exifProc->waitForBytesWritten();
    if (readBinary)
    {
        command = "-b" + endline;
        exifProc->write(command.toLatin1().data());
        exifProc->waitForBytesWritten();
    }
    command = filename + endline;
    exifProc->write(command.toUtf8().data());
    exifProc->waitForBytesWritten();
    command = "-execute" + endline;
    exifProc->write(command.toUtf8().data());
    exifProc->waitForBytesWritten();
    Output = "";

    int waited = 0;
    do
    {
        exifProc->waitForReadyRead(1000);
        //Sleep(100);
        Output += exifProc->readAllStandardOutput();
        waited++;
    } while ((Output.right(readyStr.length()) != readyStr) && (waited < 10));
    if (Output.right(readyStr.length()) == readyStr)
        Output.chop(readyStr.length());
    QString err = exifProc->readAllStandardError();
    if (err.length() > 0 || waited >= 10)
        return ito::RetVal(ito::retWarning, 0, (QObject::tr("Error reading exif-tag %1: %2").arg(tagName).arg(err)).toLatin1().data());

    return ito::retOk;
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
    

    QString filename = QString::fromLatin1((*paramsMand)[0].getVal<char*>());
    QString filenameLoad, dcrawExt(".ppm");
    QString filenamePath;
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

    if (arguments.contains(" -T"))
    {
        dcrawExt = QString(".tiff");
    }

    ito::DataObject dObj;   // create an mepty object

    /**/
    QTemporaryDir* tmpDir = new QTemporaryDir();
    if (!tmpDir->isValid())
        return ito::RetVal(ito::retError, 0, tr("could not create temporary directory").toLatin1().data());

    if ((*paramsOpt)[1].getVal<int>())
    {
        QString tmpPath = "";
        tmpPath = tmpDir->path();

        QString tmpFilename(filename);
        if (tmpPath.lastIndexOf("/") < tmpPath.length() - 1
            && tmpPath.lastIndexOf("\\") < tmpPath.length() - 1)
        {
            filename = tmpPath + "/" + ofileinfo.baseName() + "." + ofileinfo.completeSuffix();
            filenamePath = tmpPath + "/" + ofileinfo.completeBaseName();
        }
        else
        {
            filename = tmpPath + ofileinfo.baseName() + "." + ofileinfo.completeSuffix();
            filenamePath = tmpPath + ofileinfo.completeBaseName();
        }
        QFile::copy(tmpFilename, filename);
        ofileinfo = QFileInfo(filename);
    }
    else
    {
        if (ofileinfo.path().lastIndexOf("/") < ofileinfo.path().length() - 1
            && ofileinfo.path().lastIndexOf("\\") < ofileinfo.path().length() - 1)
        {
            filenamePath = ofileinfo.path() + "/" + ofileinfo.completeBaseName();
        }
        else
        {
            filenamePath = ofileinfo.path() + ofileinfo.completeBaseName();
        }
    }

    QProcess *readProc = new QProcess(NULL);
    QString command(QCoreApplication::applicationDirPath());
#ifdef WIN32
    command = "\"" + command + QString("/lib/dcraw.exe\" ") + arguments + " " + "\"" + filename + "\"";
#else
        command += QString("/lib/dcraw ") + arguments + " " + filename;
#endif
    readProc->start(command, QList<QString>(), QIODevice::ReadWrite);

//    readProc->setReadChannel(QProcess::StandardOutput);

//    QObject::connect(consoleProc, &QProcess::readyReadStandardOutput, &RawImport::readCmdOutput);
    readProc->waitForFinished();

    QVector<ito::ParamBase> filterParamsMand(0);
    QVector<ito::ParamBase> filterParamsOpt(0);
    QVector<ito::ParamBase> filterParamsOut(0);


    // did not find ppm / tiff, then check pgm
	filenameLoad = filenamePath + dcrawExt;
    if (!QFileInfo::exists(filenameLoad))
    {
        dcrawExt = ".pgm";
        filenameLoad = filenamePath + dcrawExt;
    }


    retval += apiFilterParamBase("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
    if (!retval.containsWarningOrError())
    {
		filterParamsMand[0].setVal<char*>((char*)image);
		filterParamsMand[1].setVal<char*>(filenameLoad.toLatin1().data());
        retval += apiFilterCall("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
    }
    delete readProc;

    QString exifOut;
    QProcess *exifProc = NULL;
    retval += openExifTool(filename, exifProc);
    if (!retval.containsWarningOrError())
    {
        // read some of the 'most' important tags, i.e. which usually are of interest for measurement applications
        // list is extensible, feel free to do so
        if (exifProc)
        {
            retval += readExifTag(exifProc, filename, "Aperture", exifOut);
            image->setTag("Aperture", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "FocalLength", exifOut);
            image->setTag("FocalLength", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "ExposureTime", exifOut);
            image->setTag("ExposureTime", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "SubSecDateTimeOriginal", exifOut);
            image->setTag("DateTime", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "LensID", exifOut, 0);
            image->setTag("LensID", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "VibrationReduction", exifOut);
            image->setTag("VibrationReduction", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "AutoFocus", exifOut);
            image->setTag("AutoFocus", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "AutoDistortionControl", exifOut);
            image->setTag("AutoDistortionControl", exifOut.toLatin1().data());
            retval += readExifTag(exifProc, filename, "FocalLengthIn35mmFormat", exifOut);
            image->setTag("FocalLengthIn35mmFormat", exifOut.toLatin1().data());

        }
        closeExifTool(exifProc);
    }

    if (filename.length() > 0 && QFile::exists(filename))
        QFile::remove(filename);

    if (filenameLoad.length() > 0 && QFile::exists(filenameLoad))
        QFile::remove(filenameLoad);

    delete tmpDir;

    if (!retval.containsError())
    {
        QString msg = tr("imported by dcraw import filter");
        image->addToProtocol(std::string(msg.toLatin1().data()));
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
