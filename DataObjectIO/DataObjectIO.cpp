/* ********************************************************************
    Plugin "DataObjectIO" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#include "DataObjectIO.h"

#include "transformations.h"
#include "common/numeric.h"

#include "DataObject/dataobj.h"
#include <QtCore/QtPlugin>
#include <qvariant.h>
#include <string.h>
#include <qdatetime.h>
#include <qdir.h>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    #include<qtextcodec.h>
#else
    #include<qstringconverter.h>
#endif

#include "opencv2/highgui/highgui.hpp"

#if CV_MAJOR_VERSION >= 4
    #include "opencv2/imgproc/types_c.h"
    #include "opencv2/imgproc/imgproc_c.h"
    #include "opencv2//imgcodecs/legacy/constants_c.h"
#endif

#include "common/sharedStructuresGraphics.h"
#include "common/sharedFunctionsQt.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include "qimagewriter.h"
#include <qlocale.h>

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIOInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(DataObjectIO)
    REGISTER_FILTERS_AND_WIDGETS
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIOInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(DataObjectIO)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectIOInterface::DataObjectIOInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("DataObjectIO");

    m_description = QObject::tr("import or export dataObject from/to several file formats.");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"This filter contains two different export- / import-functionalities for dataObjects, image or raw.\n\
\n\
Image-export functions converts dataObjects to image data and saves them as common image-formats.\n\
\n\
* uint8 or uint16 are saved as gray-values (8bit or if supported as 16bit) or if the image format allows color are saved according to the defined color palette.\n\
* float32 or float64 are saved as gray-values (8bit or if suppored as 16bit) or according to the defined color palette. Therefore the values must be between 0.0 and 1.0.\n\
  Values outside these borders are clipped. If the image format supports RGBA, invalid values are saved as transparent values (alpha=zero) else as black values.\n\
* rgba32 can be saved as 'rgb' (full opacity), 'rgba' (alpha channel is considered, not supported by all formats) or gray formats, where the color image is transformed to gray. \n\
  if a format from a color palette is indicated, the color image is transformed to gray first and then interpreted using the indicated color palette. \n\
\n\
Basic export-filter definition: source object, filename, palette, ...\n\
Basic import-filter definition: destination object, filename, channel-specification, ...\n\
\n\
Raw-export functions write/read the data to/from txt-based or binary file formats.";

    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr(
"This filter contains two different export- / import-functionalities for dataObjects, image or raw.\n\
\n\
Image-export functions converts dataObjects to image data and saves them as common image-formats.\n\
\n\
* uint8 or uint16 are saved as gray-values (8bit or if supported as 16bit) or if the image format allows color are saved according to the defined color palette.\n\
* float32 or float64 are saved as gray-values (8bit or if suppored as 16bit) or according to the defined color palette. Therefore the values must be between 0.0 and 1.0.\n\
  Values outside these borders are clipped. If the image format supports RGBA, invalid values are saved as transparent values (alpha=zero) else as black values.\n\
* rgba32 can be saved as 'rgb' (full opacity), 'rgba' (alpha channel is considered, not supported by all formats) or gray formats, where the color image is transformed to gray. \n\
  if a format from a color palette is indicated, the color image is transformed to gray first and then interpreted using the indicated color palette. \n\
\n\
Basic export-filter definition: source object, filename, palette, ...\n\
Basic import-filter definition: destination object, filename, channel-specification, ...\n\
\n\
Raw-export functions write/read the data to/from txt-based or binary file formats.");

    m_author            = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_license           = QObject::tr("Licensed under LPGL.");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = QObject::tr(GITVERSION);
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectIOInterface::~DataObjectIOInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
DataObjectIO::DataObjectIO() : AddInAlgo()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> * /*paramsOpt*/, ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;
    FilterDef *filter = NULL;

    filter = new FilterDef(DataObjectIO::saveDataObject, DataObjectIO::saveDataObjectParams, saveDataObjectDoc);
    m_filterList.insert("saveDataObject", filter);

    filter = new FilterDef(DataObjectIO::loadDataObject, DataObjectIO::loadDataObjectParams, loadDataObjectDoc);
    m_filterList.insert("loadDataObject", filter);

    filter = new FilterDef(DataObjectIO::loadImage, DataObjectIO::loadImageParams, loadImageDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Images (*.pgm *.pbm *.ppm *.sr *.ras *.bmp *.dib *.png *.tif *.tiff *.jpg *.jpeg *.jp2 *.gif *.xbm *.xpm)"));
    m_filterList.insert("loadAnyImage", filter);

    filter = new FilterDef(DataObjectIO::saveNistSDF, DataObjectIO::saveNistSDFParams, saveNistSDFDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("ASCII Surface Data File (*.sdf)"));
    m_filterList.insert("saveSDF", filter);

    filter = new FilterDef(DataObjectIO::loadNistSDF, DataObjectIO::loadNistSDFParams, loadNistSDFDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("ASCII or binary Surface Data File (*.sdf)"));
    m_filterList.insert("loadSDF", filter);

    filter = new FilterDef(DataObjectIO::saveTiff, DataObjectIO::saveTiffParams, saveTiffDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.tif *.tiff)"));
    m_filterList.insert("saveTiff", filter);

    filter = new FilterDef(DataObjectIO::saveJPG, DataObjectIO::saveJPGParams, saveJPGDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.jpg *.jpeg *.jp2)"));
    m_filterList.insert("saveJPG", filter);

    filter = new FilterDef(DataObjectIO::savePNG, DataObjectIO::savePNGParams, savePNGDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.png)"));
    m_filterList.insert("savePNG", filter);

    filter = new FilterDef(DataObjectIO::saveXPM, DataObjectIO::saveXPMParams, saveXPMDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.xpm *.xbm)"));
    m_filterList.insert("saveXPM", filter);

    filter = new FilterDef(DataObjectIO::saveBMP, DataObjectIO::saveBMPParams, saveBMPDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.bmp *.dib)"));
    m_filterList.insert("saveBMP", filter);

    filter = new FilterDef(DataObjectIO::saveRAS, DataObjectIO::saveRASParams, saveRASDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.sr *.ras)"));
    m_filterList.insert("saveRAS", filter);

    filter = new FilterDef(DataObjectIO::savePPM, DataObjectIO::savePPMParams, savePPMDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.ppm)"));
    m_filterList.insert("savePPM", filter);

    filter = new FilterDef(DataObjectIO::savePGM, DataObjectIO::savePGMParams, savePGMDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.pgm *.pbm)"));
    m_filterList.insert("savePGM", filter);

    filter = new FilterDef(DataObjectIO::saveItomIDO, DataObjectIO::saveItomIDOParams, saveItomIDODoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Raw-XML (*.ido *.idh)"));
    m_filterList.insert("saveIDO", filter);

    filter = new FilterDef(DataObjectIO::saveDataToTxt, DataObjectIO::saveDataToTxtParams, saveDataToTxtDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("ASCII Data (*.txt *.csv *.tsv)"));
    m_filterList.insert("saveTXT", filter);

    filter = new FilterDef(DataObjectIO::loadItomIDO, DataObjectIO::loadItomIDOParams, loadItomIDODoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Raw-XML (*.ido *.idh)"));
    m_filterList.insert("loadIDO", filter);

    filter = new FilterDef(DataObjectIO::loadDataFromTxt, DataObjectIO::loadDataFromTxtParams, loadDataFromTxtDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("ASCII Data (*.txt *.csv *.tsv)"));
    m_filterList.insert("loadTXT", filter);

    filter = new FilterDef(DataObjectIO::loadFrt, DataObjectIO::loadFrtParams, loadFrtDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("MicroProf FRT (*.frt)"));
    m_filterList.insert("loadFRT", filter);

    filter = new FilterDef(DataObjectIO::loadNanoscopeIII, DataObjectIO::loadNanoscopeIIIParams, loadNanoscopeIIIDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Veeco Nanoscope III (*.001 *.002 *.003 *.004)"));
    m_filterList.insert("loadNanoscopeIII", filter);

    filter = new FilterDef(DataObjectIO::loadAvantesRaw, DataObjectIO::loadAvantesRawParams, loadAvantesRawDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Avantes (*.raw8 *.rwd8 *.abs8 *.trm8 *.irr8 *.rfl8 *.rir8)"));
    m_filterList.insert("loadAvantesRaw", filter);

    filter = new FilterDef(DataObjectIO::loadZygoMetroPro, DataObjectIO::loadZygoMetroProParams, loadZygoMetroProDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Zygo MetroPro Data File (*.dat)"));
    m_filterList.insert("loadZygoMetroPro", filter);

    filter = new FilterDef(DataObjectIO::loadKeyenceVK4, DataObjectIO::loadKeyenceVK4Params, loadKeyenceVK4Doc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Keyence vk4 profilometry images (*.vk4)"));
    m_filterList.insert("loadKeyenceVk4", filter);

    filter = new FilterDef(DataObjectIO::savePtbPR, DataObjectIO::savePtbPRParams, savePtbPRDoc, ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("PR Line Profile (*.pr)"));
    m_filterList.insert("savePtbPR", filter);

    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::close(ItomSharedSemaphore * /*waitCond*/)
{
    ito::RetVal retval = ito::retOk;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveDataObjectParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveDataObjectDoc = QObject::tr("saves 1D and 2D dataObject to image formats via QImage (for saving images it is recommended to use the specific saveABC (savePNG, saveGIF, saveBMP...) commands).");

/** saveDataObjectParams method, specifies the parameter list for saveDataObject method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveDataObjectParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype or 3-planes DataObject of uint8").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, "", tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("format",ito::ParamBase::String | ito::ParamBase::In, "", tr("Format of the Image according to QImage: ['QImage::Format_Mono', 'QImage::Format_MonoLSB', 'QImage::Format_Indexed8', 'QImage::Format_RGB32', 'QImage::Format_ARGB32'").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("bitscaling",ito::ParamBase::Int | ito::ParamBase::In,0, 2, 1, tr("Toggle bit-scaling, 0 = off, 1 = autoscale (default), 2 = userdefined ranges").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("lowLimit",ito::ParamBase::Double | ito::ParamBase::In,-1*std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("Lowest limit. Will become 0").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("highLimit",ito::ParamBase::Double | ito::ParamBase::In,-1*std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 1.0, tr("Lowest limit. Will become max<bitdepth>").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("colorFormat",ito::ParamBase::String | ito::ParamBase::In, "", tr("When DataObject of type float32, their format should be -->Gray<-- or -->RGB<--").toLatin1().data());
        paramsOpt->append(param);

    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveDataObject
//----------------------------------------------------------------------------------------------------------------------------------
/** saveDataObject method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "saveDataObjectParams" function.
*    It converts passed DataObject into corresponding Image as per given Image Format and stores it to specific location provided on Hard drive.
*/
ito::RetVal DataObjectIO::saveDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;

    // Optional parameters (sourceImage, filename, Format, bitscaling)
    ito::DataObject *dObj = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    QString imgFilename = QLatin1String((*paramsMand)[1].getVal<char*>());
    QImage image;

    std::string imgFormat;
    imgFormat = (*paramsMand)[2].getVal<char*>();
    int toggle = (*paramsMand)[3].getVal<int>();

    // Optional parameters (lowLimit, highLimit, colorFormat)
    double lowLimit = (*paramsOpt)[0].getVal<double>();
    double highLimit = (*paramsOpt)[1].getVal<double>();

    char *color_format = NULL;
    std::string color_form;
    if ((*paramsOpt)[2].getLen()!=0)
    {
        color_format = (*paramsOpt)[1].getVal<char*>();
        color_form = color_format;
    }

    if (toggle == 2 && (fabs(lowLimit - highLimit) < std::numeric_limits<double>::epsilon() || lowLimit > highLimit))
    {
        ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: lowLimit must be unequal to highLimit and smaller than highLimit").toLatin1().data());
    }

  /*  QVariant out1 = QVariant(2.0);    // showing any desired output on itom application.
    outVals->append(out1);*/



    //Creating an Image in Mono Format
    if (imgFormat.compare("QImage::Format_Mono") == 0)
    {
        if (dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            switch(dObj->getType())
            {
                case ito::tUInt8:
                    {
                        if (toggle == 0)
                        {
                            highLimit = std::numeric_limits<ito::uint8>::max();
                            lowLimit = std::numeric_limits<ito::uint8>::min();
                        }
                        else
                        {
                            //doScaling = true;
                            ito::uint32 minPos[3] = {0, 0, 0};
                            ito::uint32 maxPos[3] = {0, 0, 0};
                            ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                        }
                        ito::uint8 tresHold = ((highLimit + lowLimit)/2.0);
                        itom::io::transformDatatoImage_Mono<ito::uint8>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tInt8:
                    {
                        if (toggle == 0)
                        {
                            highLimit = std::numeric_limits<ito::uint8>::max();
                            lowLimit = std::numeric_limits<ito::uint8>::min();
                        }
                        else
                        {
                            //doScaling = true;
                            ito::uint32 minPos[3] = {0, 0, 0};
                            ito::uint32 maxPos[3] = {0, 0, 0};
                            ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                        }
                        ito::int8 tresHold = ((highLimit + lowLimit)/2.0);
                        itom::io::transformDatatoImage_Mono<ito::int8>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tInt16:
                    {
                        if (toggle == 0)
                        {
                            highLimit = std::numeric_limits<ito::uint8>::max();
                            lowLimit = std::numeric_limits<ito::uint8>::min();
                        }
                        else
                        {
                            //doScaling = true;
                            ito::uint32 minPos[3] = {0, 0, 0};
                            ito::uint32 maxPos[3] = {0, 0, 0};
                            ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                        }
                        ito::int16 tresHold = ((highLimit + lowLimit)/2.0);
                        itom::io::transformDatatoImage_Mono<ito::int16>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tUInt16:
                    {
                        if (toggle == 0)
                        {
                            highLimit = std::numeric_limits<ito::uint8>::max();
                            lowLimit = std::numeric_limits<ito::uint8>::min();
                        }
                        else
                        {
                            //doScaling = true;
                            ito::uint32 minPos[3] = {0, 0, 0};
                            ito::uint32 maxPos[3] = {0, 0, 0};
                            ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                        }
                        ito::uint16 tresHold = ((highLimit + lowLimit)/2.0);
                        itom::io::transformDatatoImage_Mono<ito::uint16>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tInt32:
                    {
                        if (toggle == 0)
                        {
                            highLimit = std::numeric_limits<ito::uint8>::max();
                            lowLimit = std::numeric_limits<ito::uint8>::min();
                        }
                        else
                        {
                            //doScaling = true;
                            ito::uint32 minPos[3] = {0, 0, 0};
                            ito::uint32 maxPos[3] = {0, 0, 0};
                            ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                        }
                        ito::int32 tresHold = ((highLimit + lowLimit)/2.0);
                        itom::io::transformDatatoImage_Mono<ito::int32>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                //case ito::tUInt32:
                //    itom::io::transformDatatoImage_Mono<ito::uint32>(&image, dObj, imgFilename);
                //break;

                case ito::tFloat32:
                    {
                        if (toggle == 0)
                        {
                            highLimit = std::numeric_limits<ito::uint8>::max();
                            lowLimit = std::numeric_limits<ito::uint8>::min();
                        }
                        else
                        {
                            //doScaling = true;
                            ito::uint32 minPos[3] = {0, 0, 0};
                            ito::uint32 maxPos[3] = {0, 0, 0};
                            ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                        }
                        ito::float32 tresHold = ((highLimit + lowLimit)/2.0);
                        itom::io::transformDatatoImage_Mono<ito::float32>(&image, dObj, imgFilename, tresHold);
                    }
                case ito::tFloat64:
                    {
                        if (toggle == 0)
                        {
                            highLimit = std::numeric_limits<ito::uint8>::max();
                            lowLimit = std::numeric_limits<ito::uint8>::min();
                        }
                        else
                        {
                            //doScaling = true;
                            ito::uint32 minPos[3] = {0, 0, 0};
                            ito::uint32 maxPos[3] = {0, 0, 0};
                            ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                        }
                        ito::float64 tresHold = ((highLimit + lowLimit)/2.0);
                        itom::io::transformDatatoImage_Mono<ito::float64>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                default:
                    ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: DataObjectType is not supported").toLatin1().data());
            }
        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: Only 2D DataObject is supported").toLatin1().data());
        }
    }
    //Creating an Image in MonoLSB Format
    else if (imgFormat.compare("QImage::Format_MonoLSB")==0)
    {
        if (dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            if (toggle == 0)
            {
                highLimit = 255.0;
                lowLimit = 0.0;
            }
            else
            {
                //doScaling = true;
                ito::uint32 minPos[3] = {0, 0, 0};
                ito::uint32 maxPos[3] = {0, 0, 0};
                ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
            }

            switch(dObj->getType())
            {
                case ito::tUInt8:
                    {
                    if (toggle == 0)
                    {
                        highLimit = std::numeric_limits<ito::uint8>::max();
                        lowLimit = std::numeric_limits<ito::uint8>::min();
                    }
                    else
                    {
                        //doScaling = true;
                        ito::uint32 minPos[3] = {0, 0, 0};
                        ito::uint32 maxPos[3] = {0, 0, 0};
                        ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                    }
                    ito::uint8 tresHold = ((highLimit + lowLimit)/2.0);
                    itom::io::transformDatatoImage_MonoLSB<ito::uint8>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tInt8:
                    {
                    if (toggle == 0)
                    {
                        highLimit = std::numeric_limits<ito::int8>::max();
                        lowLimit = std::numeric_limits<ito::int8>::min();
                    }
                    else
                    {
                        //doScaling = true;
                        ito::uint32 minPos[3] = {0, 0, 0};
                        ito::uint32 maxPos[3] = {0, 0, 0};
                        ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                    }
                    ito::int8 tresHold = ((highLimit + lowLimit)/2.0);
                    itom::io::transformDatatoImage_MonoLSB<ito::int8>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tInt16:
                    {
                    if (toggle == 0)
                    {
                        highLimit = std::numeric_limits<ito::int16>::max();
                        lowLimit = std::numeric_limits<ito::int16>::min();
                    }
                    else
                    {
                        //doScaling = true;
                        ito::uint32 minPos[3] = {0, 0, 0};
                        ito::uint32 maxPos[3] = {0, 0, 0};
                        ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                    }
                    ito::int16 tresHold = ((highLimit + lowLimit)/2.0);
                    itom::io::transformDatatoImage_MonoLSB<ito::int16>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tUInt16:
                    {
                    if (toggle == 0)
                    {
                        highLimit = std::numeric_limits<ito::uint16>::max();
                        lowLimit = std::numeric_limits<ito::uint16>::min();
                    }
                    else
                    {
                        //doScaling = true;
                        ito::uint32 minPos[3] = {0, 0, 0};
                        ito::uint32 maxPos[3] = {0, 0, 0};
                        ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                    }
                    ito::uint16 tresHold = ((highLimit + lowLimit)/2.0);
                    itom::io::transformDatatoImage_MonoLSB<ito::uint16>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                case ito::tInt32:
                    {
                    if (toggle == 0)
                    {
                        highLimit = std::numeric_limits<ito::int32>::max();
                        lowLimit = std::numeric_limits<ito::int32>::min();
                    }
                    else
                    {
                        //doScaling = true;
                        ito::uint32 minPos[3] = {0, 0, 0};
                        ito::uint32 maxPos[3] = {0, 0, 0};
                        ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                    }
                    ito::int32 tresHold = ((highLimit + lowLimit)/2.0);
                    itom::io::transformDatatoImage_MonoLSB<ito::int32>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                //case ito::tUInt32:
                //    itom::io::transformDatatoImage_MonoLSB<ito::uint32>(&image, dObj, imgFilename);
                //break;

                case ito::tFloat32:
                    {
                    if (toggle == 0)
                    {
                        highLimit = 1.0;
                        lowLimit = 0.0;
                    }
                    else
                    {
                        //doScaling = true;
                        ito::uint32 minPos[3] = {0, 0, 0};
                        ito::uint32 maxPos[3] = {0, 0, 0};
                        ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                    }
                    ito::float32 tresHold = ((highLimit + lowLimit)/2.0);
                    itom::io::transformDatatoImage_MonoLSB<ito::float32>(&image, dObj, imgFilename, tresHold);
                    }
                case ito::tFloat64:
                    {
                    if (toggle == 0)
                    {
                        highLimit = 1.0;
                        lowLimit = 0.0;
                    }
                    else
                    {
                        //doScaling = true;
                        ito::uint32 minPos[3] = {0, 0, 0};
                        ito::uint32 maxPos[3] = {0, 0, 0};
                        ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
                    }
                    ito::float64 tresHold = ((highLimit + lowLimit)/2.0);
                    itom::io::transformDatatoImage_MonoLSB<ito::float64>(&image, dObj, imgFilename, tresHold);
                    }
                break;
                default:
                    ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: DataObjectType is not supported").toLatin1().data());
            }
        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, tr("Only 2D DataObject is supported").toLatin1().data());
        }
    }
    //Creating an Image in Indexed8 Format
    else if (imgFormat.compare("QImage::Format_Indexed8")==0)
    {
        if (dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            double scaling = 1.0;
            bool doScaling = true;
            if (toggle == 0)
            {
                doScaling = false;
                highLimit = 255.0;
                lowLimit = 0.0;
            }
            else if (toggle == 2)
            {
                //doScaling = true;
            }
            else
            {
                //doScaling = true;
                ito::uint32 minPos[3] = {0, 0, 0};
                ito::uint32 maxPos[3] = {0, 0, 0};
                ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
            }

            scaling = 255.0 / (highLimit - lowLimit);

            ito::ItomPalette newPalette;
            apiPaletteGetColorBarName("gray", newPalette);
            QVector<QRgb> colorMap(newPalette.colorVector256);

            switch(dObj->getType())
            {
                case ito::tUInt8:
                    itom::io::transformDatatoImage_Indexed8<ito::uint8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, colorMap);
                break;
                case ito::tInt8:
                    itom::io::transformDatatoImage_Indexed8<ito::int8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, colorMap);
                break;
                case ito::tInt16:
                    itom::io::transformDatatoImage_Indexed8<ito::int16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, colorMap);
                break;
                case ito::tUInt16:
                    itom::io::transformDatatoImage_Indexed8<ito::uint16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, colorMap);
                break;
                case ito::tInt32:
                    itom::io::transformDatatoImage_Indexed8<ito::int32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, colorMap);
                break;
                //case ito::tUInt32:
                //    itom::io::transformDatatoImage_MonoLSB<ito::uint32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling);
                //break;

                case ito::tFloat32:
                    itom::io::transformDatatoImage_Indexed8<ito::float32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, colorMap);
                case ito::tFloat64:
                    itom::io::transformDatatoImage_Indexed8<ito::float64>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, colorMap);
                break;
                default:
                    ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: DataObjectType is not supported").toLatin1().data());
            }
        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, tr("Only 2D DataObject is supported").toLatin1().data());
        }
    }

    //Creating an Image in RGB32 Format
    else if (imgFormat.compare("QImage::Format_RGB32")==0)
    {
        if (dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            double scaling = 1.0;
            bool doScaling = true;
            if (toggle == 0)
            {
                doScaling = false;
                highLimit = std::numeric_limits<ito::int32>::max();
                lowLimit = 0.0;
            }
            else if (toggle == 2)
            {
                //doScaling = true;
            }
            else
            {
                //doScaling = true;
                ito::uint32 minPos[3] = {0, 0, 0};
                ito::uint32 maxPos[3] = {0, 0, 0};
                ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
            }

            scaling = (std::numeric_limits<ito::int32>::max() - 1) / (highLimit - lowLimit);

            switch(dObj->getType())
            {
                case ito::tUInt8:
                    itom::io::transformDatatoImage_RGB32<ito::uint8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tInt8:
                    itom::io::transformDatatoImage_RGB32<ito::int8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tInt16:
                    itom::io::transformDatatoImage_RGB32<ito::int16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tUInt16:
                    itom::io::transformDatatoImage_RGB32<ito::uint16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tInt32:
                    itom::io::transformDatatoImage_RGB32<ito::int32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                //case ito::tUInt32:
                //    itom::io::transformDatatoImage_MonoLSB<ito::uint32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                //break;

                case ito::tFloat32:
                    itom::io::transformDatatoImage_RGB32<ito::float32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                case ito::tFloat64:
                    itom::io::transformDatatoImage_RGB32<ito::float64>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                default:
                    ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: DataObjectType is not supported").toLatin1().data());
            }
        }
        else if ((dObj->getType() == ito::tUInt8) && (dObj->getDims() > 2 && dObj->calcNumMats() == 3))   // RGB-Planes
        {
            double scaling = 1.0;
            bool doScaling = false;

            switch(dObj->getType())
            {
                case ito::tUInt8:
                    itom::io::transformDatatoImage_RGB32<ito::uint8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tInt8:
                    itom::io::transformDatatoImage_RGB32<ito::int8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tInt16:
                    itom::io::transformDatatoImage_RGB32<ito::int16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tUInt16:
                    itom::io::transformDatatoImage_RGB32<ito::uint16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tInt32:
                    itom::io::transformDatatoImage_RGB32<ito::int32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                //case ito::tUInt32:
                //    itom::io::transformDatatoImage_MonoLSB<ito::uint32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                //break;

                case ito::tFloat32:
                    itom::io::transformDatatoImage_RGB32<ito::float32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                case ito::tFloat64:
                    itom::io::transformDatatoImage_RGB32<ito::float64>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                default:
                    ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: DataObjectType is not supported").toLatin1().data());
            }
        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, tr("Only 2D DataObject is supported").toLatin1().data());
        }
    }
    //Creating an Image in ARGB32 Format
    else if (imgFormat.compare("QImage::Format_ARGB32")==0)
    {
        if (dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            double scaling = 1.0;
            bool doScaling = true;
            if (toggle == 0)
            {
                doScaling = false;
                highLimit = std::numeric_limits<ito::int32>::max();
                lowLimit = 0.0;
            }
            else if (toggle == 2)
            {
                //doScaling = true;
            }
            else
            {
                //doScaling = true;
                ito::uint32 minPos[3] = {0, 0, 0};
                ito::uint32 maxPos[3] = {0, 0, 0};
                ito::dObjHelper::minMaxValue(dObj, lowLimit, minPos, highLimit, maxPos, true, 0);
            }

            scaling = (std::numeric_limits<ito::int32>::max() - 1) / (highLimit - lowLimit);

            switch(dObj->getType())
            {
                case ito::tUInt8:
                    itom::io::transformDatatoImage_ARGB32<ito::uint8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tInt8:
                    itom::io::transformDatatoImage_ARGB32<ito::int8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tInt16:
                    itom::io::transformDatatoImage_ARGB32<ito::int16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tUInt16:
                    itom::io::transformDatatoImage_ARGB32<ito::uint16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                case ito::tInt32:
                    itom::io::transformDatatoImage_ARGB32<ito::int32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                //case ito::tUInt32:
                //    itom::io::transformDatatoImage_ARGB32<ito::uint32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                //break;

                case ito::tFloat32:
                    itom::io::transformDatatoImage_ARGB32<ito::float32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                case ito::tFloat64:
                    itom::io::transformDatatoImage_ARGB32<ito::float64>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, false);
                break;
                default:
                    ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: DataObjectType is not supported").toLatin1().data());
            }
        }
        else if ((dObj->getType() == ito::tInt8) && (dObj->getDims() > 2 && dObj->calcNumMats() == 4))   // RGBA-Planes
        {
            double scaling = 1.0;
            bool doScaling = false;

            switch(dObj->getType())
            {
                case ito::tUInt8:
                    itom::io::transformDatatoImage_ARGB32<ito::uint8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tInt8:
                    itom::io::transformDatatoImage_ARGB32<ito::int8>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tInt16:
                    itom::io::transformDatatoImage_ARGB32<ito::int16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tUInt16:
                    itom::io::transformDatatoImage_ARGB32<ito::uint16>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                case ito::tInt32:
                    itom::io::transformDatatoImage_ARGB32<ito::int32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                //case ito::tUInt32:
                //    itom::io::transformDatatoImage_ARGB32<ito::uint32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                //break;

                case ito::tFloat32:
                    itom::io::transformDatatoImage_ARGB32<ito::float32>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                case ito::tFloat64:
                    itom::io::transformDatatoImage_ARGB32<ito::float64>(&image, dObj, imgFilename, lowLimit, scaling, doScaling, true);
                break;
                default:
                    ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: DataObjectType is not supported").toLatin1().data());
            }
        }
        else if ((dObj->getType() == ito::tInt8) && (dObj->getDims() > 2 && dObj->calcNumMats() == 3))   // RGB-Planes
        {

        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, tr("Only 2D DataObject is supported").toLatin1().data());
        }
    }
    else
    {
    ret += ito::RetVal(ito::retError, 0, tr("Entered Image format is not supported").toLatin1().data());
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadDataObjectParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadDataObjectDoc = QObject::tr(
"loads a 2D data object from image formats via QImage (*.gif *.xbm *.xpm). \n\
This filter is deprecated and can also be directly used via the filter 'loadAnyImage'");

/** loadDataObjectParams method, specifies the parameter list for loadDataObject method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting the specified Image from its specified location on Hard drive into itom DataObject.
*/
ito::RetVal DataObjectIO::loadDataObjectParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObjet").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);

        ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::String, "asIs");
        m->addItem("alpha");
        m->addItem("R");
        m->addItem("G");
        m->addItem("B");
        m->addItem("RGB");
        m->addItem("RGBA");
        m->addItem("GRAY");
        param = ito::Param("colorElement", ito::ParamBase::String | ito::ParamBase::In, "asIs", tr("Color element: asIs (default) | alpha | R | G | B | RGB | ARGB | GRAY; 'asIs' ignores alpha channel from supported file types (use rgba instead).").toLatin1().data());
        param.setMeta(m,true); //takes ownership of m
        paramsOpt->append(param);
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadDataObject
//----------------------------------------------------------------------------------------------------------------------------------
/** loadDataObject method, retrieves the Image data and creates corresponding DataObject.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "loadDataObjectParams" function.
*    It retrieves the Image data from Image location passed as parameter from Hard drive and loads a corresponding Itom DataObject.
*/
ito::RetVal DataObjectIO::loadDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
    QImage image;
    QFileInfo fileinfo(QString::fromLatin1(filename));

    if (!fileinfo.exists())
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' does not exist.").toLatin1().data(), filename);
    }
    else if (!image.load(QLatin1String(filename)))
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' is no readable image file.").toLatin1().data(), filename);
    }
    else
    {
        ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
        std::string colorElem;
        if ((*paramsOpt)[0].getLen()!= 0)
        {
            colorElem = (*paramsOpt)[0].getVal<char*>();
        }

        QImage::Format imgFormat;
        imgFormat = image.format();

        if (colorElem.compare("asIs") == 0)
        {
            switch (imgFormat)
            {
            case QImage::Format_Mono:
            case QImage::Format_MonoLSB:
                colorElem = "GRAY";
                break;
            case QImage::Format_Indexed8:
                colorElem = "RGBA";
                break;
            case QImage::Format_RGB32:
                colorElem = "RGB";
                break;
            default:
                colorElem = "RGBA";
                break;
            }

        }
        if (colorElem.compare("RGB") == 0 || colorElem.compare("RGBA") == 0)
        {
            *dObjDst= ito::DataObject(image.height() ,image.width(), ito::tRGBA32);
        }
        else
        {
            *dObjDst = ito::DataObject(image.height(), image.width(), ito::tUInt8);
        }


        if (imgFormat == QImage::Format_Mono || imgFormat == QImage::Format_MonoLSB)    //Case for Format_Mono and Format_MonoLSB images. Note: All Format_MonoLSB images are detected as Format_Mono images so No special case for Format_MonoLSB.
        {
            if (dObjDst->getType() == ito::tRGBA32)
            {
                ret += itom::io::QImage_Mono_to_dataObject<ito::Rgba32>(&image, dObjDst);
            }
            else
            {
                ret += itom::io::QImage_Mono_to_dataObject<ito::uint8>(&image, dObjDst);
            }
        }
        else if (imgFormat == QImage::Format_Indexed8)   //Case for Format_Indexed8 images
        {
            if (dObjDst->getType() == ito::tRGBA32)
            {
                ret += itom::io::QImage_Indexed8_to_dataObject<ito::Rgba32>(&image, dObjDst);
            }
            else
            {
                ret += itom::io::QImage_Indexed8_to_dataObject<ito::uint8>(&image, dObjDst);
            }
        }
        else if (imgFormat == QImage::Format_RGB32)     //Case for Format_RGB32 images
        {
            if (dObjDst->getType() == ito::tRGBA32)
            {
                ret += itom::io::QImage_ARGB32_to_dataObject<ito::Rgba32>(&image, dObjDst, colorElem.data());
            }
            else
            {
                ret += itom::io::QImage_ARGB32_to_dataObject<ito::uint8>(&image, dObjDst, colorElem.data());
            }
        }
        else if (imgFormat == QImage::Format_ARGB32)     //Case for Format_ARGB32 images
        {
            if (dObjDst->getType() == ito::tRGBA32)
            {
                ret += itom::io::QImage_ARGB32_to_dataObject<ito::Rgba32>(&image, dObjDst, colorElem.data());
            }
            else
            {
                ret += itom::io::QImage_ARGB32_to_dataObject<ito::uint8>(&image, dObjDst, colorElem.data());
            }
        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, tr("Image does not have supported format").toLatin1().data());
        }
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveNistSDFParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveNistSDFDoc = QObject::tr("saves 1D and 2D dataObject to the ascii surface data file format (sdf), version aNIST-1.0 (no official standard) or aISO-1.0 (in compliance with ISO 25178-71).");

/** saveNistSDFParams method, specifies the parameter list for loadNistSDFParams method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Raw-Text data and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveNistSDFParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype or 3-planes DataObject of uint8").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("decimalSigns",ito::ParamBase::Int | ito::ParamBase::In, 0, 12, 3, tr("Number of decimal signs (default: 3). For MountainsMaps reduce total number of digits to 5").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("verticalScale",ito::ParamBase::Int | ito::ParamBase::In, -12, 12, -3, tr("Power of 10 for the zValue (Default: -3, micrometer), only for floating point objects.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("invalidHandling",ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 0, tr("Toggles NaN handling if dataObject is floating-type. 0: Write NaN (Default); 1: Skip Value, 2: Substitute by InvalidValue, 3: Substitute by BAD for MountainsMaps.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("invalidValue",ito::ParamBase::Double | ito::ParamBase::In, -1 * std::numeric_limits<double>::max(), std::numeric_limits<double>::max() , -42.0, tr("New value for invalid substitution. Default is 0.0").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("version", ito::ParamBase::String | ito::ParamBase::In, "aNIST-1.0", tr("File format version: 'aNIST-1.0' or 'aISO-1.0' for DIN EN ISO 25178-71").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String);
        sm.addItem("aNIST-1.0");
        sm.addItem("aISO-1.0");
        param.setMeta(&sm, false);
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveNistSDF
//----------------------------------------------------------------------------------------------------------------------------------
/** saveNistSDF method, writes the data from a dataObject to an ascii file.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "saveNistSDF" function.
*    It retrieves the raw-data from the dataObject and writes them as ascii-data to the location passed as parameter from Hard drive and loads a corresponding Itom DataObject.
*
*   see http://physics.nist.gov/VSC/jsp/DataFormat.jsp#a or http://resource.npl.co.uk/softgauges/Help.htm
*
*   see also K. J. Stout, P. J. Sullivan, W. P. Dong, E. Mainsah, N. Lou, T. Mathia and H. Zahouani, The Development of Methods for The Characterisation of Roughness in Three Dimensions, Report EUR 15178 EN. EC Brussels, 1993
*/
ito::RetVal DataObjectIO::saveNistSDF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    QString filename_ = QLatin1String((*paramsMand)[1].getVal<char*>());

    if (!filename_.endsWith(".sdf", Qt::CaseInsensitive))
    {
        filename_.append(".sdf");
    }

    QFileInfo fileinfo(filename_);
    QFile dataOut(fileinfo.absoluteFilePath());

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();

    ret += ito::dObjHelper::verify2DDataObject(dObjSrc, "sourceImage", 1, std::numeric_limits<int>::max(), 1, std::numeric_limits<int>::max(), 8, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tUInt32, ito::tFloat32, ito::tFloat64);

    double verticalScale = pow(10.0, (*paramsOpt)[1].getVal<int>());
    int decimals = (*paramsOpt)[0].getVal<int>();
    QString version = paramsOpt->at(4).getVal<char*>();
    bool is_iso25178 = (version == "aISO-1.0");

    int flags = DataObjectIO::invWrite;

    switch((*paramsOpt)[2].getVal<int>())
    {
        default:
        case DataObjectIO::invWrite:
            break;
        case DataObjectIO::invBAD:
            flags = DataObjectIO::invBAD;
            break;
        case DataObjectIO::invChange:
            flags = DataObjectIO::invChange;
            break;
        case DataObjectIO::invIgnor:
            flags = DataObjectIO::invIgnor;
            break;
    }

    if (ret.containsWarningOrError())
    {

    }
    else if (!dataOut.open(QIODevice::WriteOnly))
    {
        ret += ito::RetVal::format(ito::retError, 0, tr("The file '%s' is no writeable file.").toLatin1().data(), filename_.toLatin1().data());
    }

    if (!ret.containsWarningOrError())
    {
        QByteArray outLine(100, 0);
        ito::ByteArray tag;
        std::string unitStr;
        double value = 0.0;
        bool dummyBool;
        bool isLine = (dObjSrc->getSize(0) == 1 || dObjSrc->getSize(1) == 1) ? true : false;
        bool changeXY = dObjSrc->getSize(1) == 1 ? true : false;

        if (is_iso25178)
        {
            dataOut.write("aISO-1.0\n");
        }
        else
        {
            dataOut.write("aNIST-1.0\n");
        }

        outLine = ("ManufacID\t\t= ");
        tag = dObjSrc->getTag("ManufacID", dummyBool).getVal_ToString();
        if (tag.empty())
        {
            tag = fileinfo.fileName().toLatin1().data();
        }
        outLine.append(tag.data());
        outLine.append('\n');
        dataOut.write(outLine);

        outLine = ("CreateDate\t\t= ");
        tag = dObjSrc->getTag("CreateDate", dummyBool).getVal_ToString();
        if (tag.empty())
        {
            tag = "000000000000";
        }
        outLine.append(tag.data());
        outLine.append('\n');
        dataOut.write(outLine);

        outLine = ("ModDate\t\t= ");
        tag = dObjSrc->getTag("ModDate", dummyBool).getVal_ToString();
        if (tag.empty())
        {
            tag = "000000000000";
        }
        outLine.append(tag.data());
        outLine.append('\n');
        dataOut.write(outLine);

        double xScale = dObjSrc->getAxisScale(1);
        unitStr = dObjSrc->getAxisUnit(1, dummyBool);
        if (unitStr == "mm")
        {
            xScale *= 1/1000.0;
        }
        else if (unitStr == "µm")
        {
            xScale *= 1/1000000.0;
        }
        else if (unitStr == "km")
        {
            xScale *= 1000.0;
        }

        double yScale = dObjSrc->getAxisScale(0);
        unitStr = dObjSrc->getAxisUnit(0, dummyBool);
        if (unitStr == "mm")
        {
            yScale *= 1/1000.0;
        }
        else if (unitStr == "µm")
        {
            yScale *= 1/1000000.0;
        }
        else if (unitStr == "km")
        {
            yScale *= 1000.0;
        }

        if (changeXY)
        {
            outLine = ("NumPoints\t\t= ");
            outLine.append(QByteArray::number((ito::int32)dObjSrc->getSize(0)));
            outLine.append('\n');
            dataOut.write(outLine);
            outLine = ("NumProfiles\t\t= ");
            outLine.append(QByteArray::number((ito::int32)dObjSrc->getSize(1)));
            outLine.append('\n');
            dataOut.write(outLine);
            outLine = ("Xscale\t\t= ");
            outLine.append(QByteArray::number(yScale));
            outLine.append('\n');
            dataOut.write(outLine);
            outLine = ("Yscale\t\t= ");
            outLine.append(QByteArray::number(xScale));
            outLine.append('\n');
            dataOut.write(outLine);
        }
        else
        {
            outLine = ("NumPoints\t\t= ");
            outLine.append(QByteArray::number((ito::int32)dObjSrc->getSize(1)));
            outLine.append('\n');
            dataOut.write(outLine);
            outLine = ("NumProfiles\t\t= ");
            outLine.append(QByteArray::number((ito::int32)dObjSrc->getSize(0)));
            outLine.append('\n');
            dataOut.write(outLine);
            outLine = ("Xscale\t\t= ");
            outLine.append(QByteArray::number(xScale));
            outLine.append('\n');
            dataOut.write(outLine);
            outLine = ("Yscale\t\t= ");
            outLine.append(QByteArray::number(yScale));
            outLine.append('\n');
            dataOut.write(outLine);
        }

        double zScale = dObjSrc->getValueScale();

        if (dObjSrc->getType() != ito::tFloat32 &&  dObjSrc->getType() != ito::tFloat64)
        {
            verticalScale = 1.0;
        }
        else
        {
            zScale *= verticalScale;
        }

        unitStr = dObjSrc->getValueUnit();
        if (unitStr == "mm")
        {
            zScale *= 1/1000.0;
        }
        else if (unitStr == "µm")
        {
            zScale *= 1/1000000.0;
        }
        else if (unitStr == "km")
        {
            zScale *= 1000;
        }

        outLine = ("Zscale\t\t= ");
        outLine.append(QByteArray::number(zScale));
        outLine.append('\n');
        dataOut.write(outLine);

        tag = dObjSrc->getTag("Zresolution", dummyBool).getVal_ToString();
        if (tag.empty())
        {
            outLine = ("Zresolution\t\t= -1\n"); //negative is unknown concerning aBCR1.0 or aISO1.0 standard
        }
        else
        {
            outLine = ("Zresolution\t\t= " + QByteArray(tag.data()) + "\n");
        }
        dataOut.write(outLine);

        outLine = ("Compression\t\t= 0\n");
        dataOut.write(outLine);

        if (is_iso25178)
        {
            switch (dObjSrc->getType())
            {
            case ito::tUInt8:
            case ito::tUInt16:
            case ito::tUInt32:
            case ito::tInt8:
            case ito::tInt16:
            case ito::tInt32:
                outLine = ("DataType\t\t= 6");
                break;
            case ito::tFloat32:
            case ito::tFloat64:
                outLine = ("DataType\t\t= 7");
                break;
            }
        }
        else
        {
            switch (dObjSrc->getType())
            {
            case ito::tUInt8:
                outLine = ("DataType\t\t= 0");
                break;
            case ito::tUInt16:
                outLine = ("DataType\t\t= 1");
                break;
            case ito::tUInt32:
                outLine = ("DataType\t\t= 2");
                break;
            case ito::tInt8:
                outLine = ("DataType\t\t= 4");
                break;
            case ito::tInt16:
                outLine = ("DataType\t\t= 5");
                break;
            case ito::tInt32:
                outLine = ("DataType\t\t= 6");
                break;
            case ito::tFloat32:
                outLine = ("DataType\t\t= 3");
                break;
            case ito::tFloat64:
                outLine = ("DataType\t\t= 7");
                break;
            }
        }
        outLine.append('\n');
        dataOut.write(outLine);

        outLine = ("CheckType\t\t= 0\n");
        dataOut.write(outLine);

        outLine = "*\n";
        dataOut.write(outLine);
        outLine = "\n";
        dataOut.write(outLine);

        double newInvalidValue = (*paramsOpt)[3].getVal<double>() / verticalScale;

        switch(dObjSrc->getType())
        {
            case ito::tUInt8:
                writeDataBlock<ito::uint8>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
            case ito::tUInt16:
                writeDataBlock<ito::uint16>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
            case ito::tUInt32:
                writeDataBlock<ito::uint32>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
            case ito::tInt8:
                writeDataBlock<ito::int8>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
            case ito::tInt16:
                writeDataBlock<ito::int16>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
            case ito::tInt32:
                writeDataBlock<ito::int32>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
            case ito::tFloat32:
                writeDataBlock<ito::float32>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
            case ito::tFloat64:
                writeDataBlock<ito::float64>(dataOut, dObjSrc, 1/verticalScale, decimals, flags, ' ', newInvalidValue);
                break;
        }

        outLine = "*\n";
        dataOut.write(outLine);
    }

    if (dataOut.isOpen())
    {
        dataOut.close();
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! writeDataBlock
//----------------------------------------------------------------------------------------------------------------------------------
/** readDataBlock method, read the data-block of a .sdf-nist metrology file and write data to the corresponding dataObject.
*   @param [in]     inFile      Readonly File
*   @param [in|out] newObject   The dataObject to be created
*    @param [in]     zscale      The zscale, will be multiplied with the extracted data
*   @param [int]    flags       For later improvements, not used yet
*/
template<typename _Tp> ito::RetVal DataObjectIO::writeDataBlock(QFile &outFile, const ito::DataObject *scrObject, const double zScale, const int decimals, const int flags, const char seperator, const double nanValue)
{
    ito::RetVal ret(ito::retOk);

    int x = 0, y = 0;

    int xsize = scrObject->getSize(1);
    int ysize = scrObject->getSize(0);

    const cv::Mat* dstMat = (const cv::Mat*)(scrObject->get_mdata()[0]);
    const _Tp *p_Dst = NULL;
    QByteArray curLine;
    curLine.reserve(30000);
    if (xsize == 1)
    {
        if (std::numeric_limits<_Tp>::is_exact)
        {
            for (y = 0; y < ysize; y ++)
            {
                p_Dst = dstMat->ptr<_Tp>(y);
                curLine = QByteArray::number(p_Dst[0]);
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
        else
        {
            for (y = 0; y < ysize; y ++)
            {
                p_Dst = dstMat->ptr<_Tp>(y);
                curLine = QByteArray::number(p_Dst[0]*zScale, 'f', decimals);
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
    }
    else if (ysize == 1)
    {
        p_Dst = dstMat->ptr<_Tp>(0);
        if (std::numeric_limits<_Tp>::is_exact)
        {
            for (x = 0; x < xsize; x ++)
            {
                curLine = QByteArray::number(p_Dst[x]);
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
        else
        {
            for (x = 0; x < xsize; x ++)
            {
                curLine = QByteArray::number(p_Dst[x]*zScale, 'f', decimals);
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
    }
    else
    {

        if (std::numeric_limits<_Tp>::is_exact)
        {
            for (y = 0; y < ysize; y ++)
            {
                p_Dst = dstMat->ptr<_Tp>(y);
                curLine.clear();
                for (x = 0; x < xsize - 1; x ++)
                {
                    curLine.append(QByteArray::number(p_Dst[x]));
                    curLine.append(seperator);
                }
                curLine.append(QByteArray::number(p_Dst[xsize - 1]));
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
        else
        {
            switch((flags & invHandlingMask))
            {
                default:
                for (y = 0; y < ysize; y ++)
                {
                    p_Dst = dstMat->ptr<_Tp>(y);
                    curLine.clear();
                    for (x = 0; x < xsize - 1; x ++)
                    {
                        curLine.append(QByteArray::number(p_Dst[x]*zScale, 'f', decimals));
                        curLine.append(seperator);
                    }
                    curLine.append(QByteArray::number(p_Dst[xsize - 1]*zScale, 'f', decimals));
                    curLine.append('\n');
                    outFile.write(curLine);
                }
                break;
                case DataObjectIO::invIgnor:
                for (y = 0; y < ysize; y ++)
                {
                    p_Dst = dstMat->ptr<_Tp>(y);
                    curLine.clear();
                    for (x = 0; x < xsize - 1; x ++)
                    {
                        if (ito::isFinite<_Tp>(p_Dst[x])) curLine.append(QByteArray::number(p_Dst[x]*zScale, 'f', decimals));
                        curLine.append(seperator);
                    }
                    if (ito::isFinite<_Tp>(p_Dst[xsize - 1])) curLine.append(QByteArray::number(p_Dst[xsize - 1]*zScale, 'f', decimals));
                    curLine.append('\n');
                    outFile.write(curLine);
                }
                break;
                case DataObjectIO::invChange:
                for (y = 0; y < ysize; y ++)
                {
                    p_Dst = dstMat->ptr<_Tp>(y);
                    curLine.clear();
                    for (x = 0; x < xsize - 1; x ++)
                    {
                        if (ito::isFinite<_Tp>(p_Dst[x])) curLine.append(QByteArray::number(p_Dst[x]*zScale, 'f', decimals));
                        else curLine.append(QByteArray::number(nanValue, 'f', decimals));
                        curLine.append(seperator);
                    }
                    if (ito::isFinite<_Tp>(p_Dst[xsize - 1])) curLine.append(QByteArray::number(p_Dst[xsize - 1]*zScale, 'f', decimals));
                    else curLine.append(QByteArray::number(nanValue, 'f', decimals));
                    curLine.append('\n');
                    outFile.write(curLine);
                }
                break;
                case DataObjectIO::invBAD:
                for (y = 0; y < ysize; y ++)
                {
                    p_Dst = dstMat->ptr<_Tp>(y);
                    curLine.clear();
                    for (x = 0; x < xsize - 1; x ++)
                    {
                        if (ito::isFinite<_Tp>(p_Dst[x])) curLine.append(QByteArray::number(p_Dst[x]*zScale, 'f', decimals));
                        else curLine.append("BAD");
                        curLine.append(seperator);
                    }
                    if (ito::isFinite<_Tp>(p_Dst[xsize - 1])) curLine.append(QByteArray::number(p_Dst[xsize - 1]*zScale, 'f', decimals));
                    else curLine.append("BAD");
                    curLine.append('\n');
                    outFile.write(curLine);
                }
                break;
            }
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadNistSDFParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadNistSDFDoc = QObject::tr("loads an ascii or binary-based surface data file to a 1D or 2D data object (sdf), versions aNIST-1.0, aISO-1.0, bISO-1.0 (ISO 25178-71) or aBCR-1.0 (e.g. Zygo export format) are supported.");

/** loadNistSDFParams method, specifies the parameter list for loadNistSDFParams method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Raw-Text data and save it into Hard drive.
*/
ito::RetVal DataObjectIO::loadNistSDFParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("data object where the file data is loaded to. The type of destinationObject corresponds to the type of the data block in the sdf file (uint8, uint16, uint32, int8, int16, int32, float32, float64) or always float64 if the header value Zscale is != 1.0 or the valueUnit is != 'm'").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("filename of the sdf file (aISO-1.0, aNIST-1.0, aBCR-1.0 or bISO-1.0 format)").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("xyUnit", ito::ParamBase::String | ito::ParamBase::In, "mm", tr("Unit of x and y axes. Nist or BCR sdf files assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than 'm' lead to a multiplication of all values that might exceed the data type limit.)").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String, "m");
        sm.addItem("cm");
        sm.addItem("mm");
        sm.addItem(QString("_m").replace("_", QLatin1String("\u00B5")).toLatin1().data());
        sm.addItem("nm");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("valueUnit", ito::ParamBase::String | ito::ParamBase::In, "mm", tr("Unit of value axis. x3p assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than 'm' lead to a multiplication of all values that might exceed the data type limit.)").toLatin1().data());
        ito::StringMeta sm2(ito::StringMeta::String, "m");
        sm2.addItem("cm");
        sm2.addItem("mm");
        sm2.addItem(QString("_m").replace("_", QLatin1String("\u00B5")).toLatin1().data());
        sm2.addItem("nm");
        param.setMeta(&sm2, false);
        paramsOpt->append(param);

        param = ito::Param("nanValue", ito::ParamBase::String | ito::ParamBase::In, "BAD", tr("If this string occurs in the data block, the value will be replaced by NaN if float32 or float64 as output format. If '<minrange>' or <maxrange> the minimum or maximum value of the data type in the data block is assumed (e.g. <maxrange> is used by Zygo to describe NaN values). MountainsMap writes 'BAD' as invalid value (following ISO25178-71). This value is ignored for binary input since invalid values are directly encoded in floating point values.").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadNistSDF
//----------------------------------------------------------------------------------------------------------------------------------
/** loadNistSDF method, retrieves the ascii data and creates corresponding DataObject.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "loadNistSDF" function.
*    It retrieves the ascii-data from file location passed as parameter from Hard drive and loads a corresponding Itom DataObject.
*
*   see http://physics.nist.gov/VSC/jsp/DataFormat.jsp#a or http://resource.npl.co.uk/softgauges/Help.htm
*
*   see also K. J. Stout, P. J. Sullivan, W. P. Dong, E. Mainsah, N. Lou, T. Mathia and H. Zahouani, The Development of Methods for The Characterisation of Roughness in Three Dimensions, Report EUR 15178 EN. EC Brussels, 1993

*/
ito::RetVal DataObjectIO::loadNistSDF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    const char *filename = (*paramsMand)[1].getVal<char*>();
    QFileInfo fileinfo(QString::fromLatin1(filename));
    QFile dataIn(fileinfo.canonicalFilePath());
    std::string xyUnit = paramsOpt->at(0).getVal<char*>();
    std::string valueUnit = paramsOpt->at(1).getVal<char*>();
    QByteArray nanValue = paramsOpt->at(2).getVal<char*>();
    ito::float64 zscale = 1.0;

    ito::DataObject *dObjDst = (*paramsMand)[0].getVal<ito::DataObject*>();

    if (dObjDst == NULL)
    {
        ret += ito::RetVal::format(ito::retError,0,tr("Dataobject not initialized").toLatin1().data(), filename);
    }
    else if (!fileinfo.exists())
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' does not exist.").toLatin1().data(), filename);
    }
    else if (!dataIn.open(QIODevice::ReadOnly))
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' is no readable file.").toLatin1().data(), filename);
    }
    else
    {
        //check if binary or ascii format
        char version[8];
        int versionLen = dataIn.read(version, 8);
        bool ascii = false;

        if (versionLen > 0)
        {
            if (version[0] == 'b')
            {
                ascii = false;
            }
            else
            {
                ascii = true;
            }
        }
        else
        {
            ret += ito::RetVal(ito::retError, 0, tr("No version tag could be found in file.").toLatin1().data());
        }

        //move file pointer back to start
        dataIn.seek(0);

        if (ascii)
        {
            ret += readNistHeader(dataIn, *dObjDst, zscale, 0, xyUnit, valueUnit, nanValue);
            if (!ret.containsError())
            {
                switch (dObjDst->getType())
                {
                case ito::tInt8:
                    ret += readDataBlock<ito::int8>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                case ito::tUInt8:
                    ret += readDataBlock<ito::uint8>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                case ito::tInt16:
                    ret += readDataBlock<ito::int16>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                case ito::tUInt16:
                    ret += readDataBlock<ito::uint16>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                case ito::tInt32:
                    ret += readDataBlock<ito::int32>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                case ito::tUInt32:
                    ret += readDataBlock<ito::uint32>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                case ito::tFloat32:
                    ret += readDataBlock<ito::float32>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                case ito::tFloat64:
                    ret += readDataBlock<ito::float64>(dataIn, *dObjDst, zscale, 0, nanValue);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, "DataType not supported");
                    break;
                }
            }
        }
        else
        {
            ret += readNistHeaderBinary(dataIn, *dObjDst, zscale, 0, xyUnit, valueUnit, nanValue);
        }
    }

    if (dataIn.isOpen())
    {
        dataIn.close();
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! readNistHeader
//----------------------------------------------------------------------------------------------------------------------------------
/** readNistHeader method, read the header of a .sdf-nist metrology file and create corresponding dataObject.
*   @param [in] inFile      Readonly File
*   @param [out] newObject  The dataObject to be created
*    @param [out] zscale     The zscale, will be multiplied with the extracted data later
*   @param [int] flags      For later improvements, not used yet
*   @param [in/out] nanString if nanString is auto, it will be set to a byte array with the highest possible value for the type. If this value occurs, it will be set to NaN further on (only in float32 or float64)
*/
ito::RetVal DataObjectIO::readNistHeader(QFile &inFile, ito::DataObject &newObject, double &zscale,const int /*flags*/, const std::string &xyUnit, const std::string &valueUnit, QByteArray &nanString)
{
    ito::RetVal retValue;
    ito::float64 xscale = 1.0, yscale = 1.0, zRes = 1.0;
    int xsize = 0, ysize = 0;
    int dataType = -1;
    QByteArray curLine;
    std::map<std::string, std::string> metaData;
    QMap<QString, QByteArray> rawMetaData;
    QMap<QString, QByteArray>::const_iterator it;

    curLine = inFile.readLine();

    zscale = 1.0;

    QString stringLine = curLine;

    //First line Version Number (a: ASCII, b: binary), Unsigned Char, 8-bytes
    if (!stringLine.contains("aNIST-1.0", Qt::CaseInsensitive) && !stringLine.contains("aBCR-1.0", Qt::CaseInsensitive) && !stringLine.contains("aISO-1.0", Qt::CaseInsensitive))
    {
        return ito::RetVal::format(ito::retError,0,tr("The file '%s' does not contain an 'aNIST-1.0', 'aISO-1.0' or 'aBCR-1.0'-header.").toLatin1().data(), inFile.fileName().toLatin1().data());
    }

    curLine = inFile.readLine();
    while(!curLine.contains("*") && !inFile.atEnd())
    {
        QList<QByteArray> list = curLine.split('=');
        if (list.size() == 2)
        {
            rawMetaData[list[0].simplified().toLower()] = list[1].simplified();
        }

        curLine = inFile.readLine();
    }

    //check if everything is available and load it:
    it = rawMetaData.constFind("manufacid");
    if (it != rawMetaData.constEnd()) //Manufacturer ID, Unsigned Char, 10-bytes
    {
        metaData["ManufacID"] = it->data();
        if (it->length() > 10)
        {
            retValue += ito::RetVal(ito::retWarning, 0, "Header ManufacID is longer than 10 characters");
        }
    }

    it = rawMetaData.constFind("createdate");
    if (it != rawMetaData.constEnd()) //Create Date and Time, Unsigned Char, 12-bytes
    {
        if (it->length() != 12)
        {
            retValue += ito::RetVal(ito::retWarning, 0, "Length of header CreateDate is different than 12 characters");
        }
        else
        {
            QDateTime date = QDateTime::fromString(*it, "ddMMyyyyHHmm");
            if (date.isValid())
            {
                metaData["CreateDate"] = date.toString("yyyy-MM-ddTHH:mm:00.0+00:00").toStdString(); //same format than x3p format
            }
            else
            {
                retValue += ito::RetVal(ito::retWarning, 0, "Header CreateDate can not be parsed in format DDMMYYYYHHMM");
            }
        }
    }

    it = rawMetaData.constFind("moddate");
    if (it != rawMetaData.constEnd()) //Modified Date and Time, Unsigned Char, 12-bytes
    {
        if (it->length() != 12)
        {
            retValue += ito::RetVal(ito::retWarning, 0, "Length of header ModDate is different than 12 characters");
        }
        else
        {
            QDateTime date = QDateTime::fromString(*it, "ddMMyyyyHHmm");
            if (date.isValid())
            {
                metaData["ModDate"] = date.toString("yyyy-MM-ddTHH:mm:00.0+00:00").toStdString(); //same format than x3p format
            }
            else
            {
                retValue += ito::RetVal(ito::retWarning, 0, "Header ModDate can not be parsed in format DDMMYYYYHHMM");
            }
        }
    }

    bool ok;
    it = rawMetaData.constFind("numpoints");
    if (it != rawMetaData.constEnd()) //Number of points in a profile, Unsigned Int, 2-bytes
    {
        xsize = it->toInt(&ok);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, "Header NumPoints can not be interpreted as integer");
        }
        else if (xsize <= 0 || xsize > 65535)
        {
            //limit 65535 is given in ISO 25178-71
            retValue += ito::RetVal(ito::retError, 0, "Header NumPoints out of range (0, 65535]");
        }
    }

    it = rawMetaData.constFind("numprofiles");
    if (it != rawMetaData.constEnd()) //Number of profiles in a data file, Unsigned Int, 2-bytes
    {
        ysize = it->toInt(&ok);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, "Header NumProfiles can not be interpreted as integer");
        }
        else if (ysize <= 0 || ysize > 65535)
        {
            //limit 65535 is given in ISO 25178-71
            retValue += ito::RetVal(ito::retError, 0, "Header NumProfiles out of range (0, 65535]");
        }
    }

    if (xsize < 0 || ysize < 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "height or width of file is <= 0. No data is loaded.");
    }

    it = rawMetaData.constFind("xscale");
    if (it != rawMetaData.constEnd()) //X-scale. A x-scale value of 1.00 E-6 represents a sample spacing of 1 micrometer, Double, 8-bytes
    {
        xscale = it->toDouble(&ok);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, "Header Xscale can not be interpreted as double");
        }
    }

    it = rawMetaData.constFind("yscale");
    if (it != rawMetaData.constEnd()) //Y-scale. A y-scale value of 1.00 E-6 represents a sample spacing of 1 micrometer, Double, 8-bytes
    {
        yscale = it->toDouble(&ok);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, "Header Yscale can not be interpreted as double");
        }
    }

    it = rawMetaData.constFind("zscale");
    if (it != rawMetaData.constEnd()) //Z-scale. A z-scale value of 1.00 E-6 represents a height of 1 micrometer, Double, 8-bytes
    {
        zscale = it->toDouble(&ok);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, "Header Zscale can not be interpreted as double");
        }
    }

    it = rawMetaData.constFind("zresolution");
    if (it != rawMetaData.constEnd()) //Z- resolution, Double, 8-bytes
    {
        zRes = it->toDouble(&ok);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, "Header Zresolution can not be interpreted as double");
        }
        else if (zRes < 0.0)
        {
            zRes = 1.0; //BCR Format 1.0 says: if zResolution negative, it is unknown
        }
        else
        {
            metaData["Zresolution"] = it->data();
        }
    }

    it = rawMetaData.constFind("compression"); //Compression Type, Unsigned Char, 1-bytes
    if (it != rawMetaData.constEnd())
    {
        if (it->toLower() != "null" && it->toLower() != "0")
        {
            retValue += ito::RetVal(ito::retError, 0, "Header Compression must be 0 or NULL. Other compressions not supported by this filter and not specified in ISO25178-71.");
        }
    }

    it = rawMetaData.constFind("checktype"); //Check Sum Type, Unsigned Char, 1-bytes
    if (it != rawMetaData.constEnd())
    {
        if (it->toLower() != "null" && it->toLower() != "0")
        {
            retValue += ito::RetVal(ito::retWarning, 0, "Header CheckType must be 0 or NULL. Other check sum types are ignored by this filter and not specified in ISO25178-71.");
        }
    }

    it = rawMetaData.constFind("datatype");
    if (it != rawMetaData.constEnd())
    {
        dataType = it->toInt(&ok);
        if (!ok)
        {
            retValue += ito::RetVal(ito::retError, 0, "Header DataType cannot be interpreted as integer");
        }
        else
        {
            switch(dataType)
            {
                case 0:
                    dataType = ito::tUInt8;
                    if (nanString == "<maxrange>") nanString = "255";
                    if (nanString == "<minrange>") nanString = "0";
                    break;
                case 1:
                    dataType = ito::tUInt16;
                    if (nanString == "<maxrange>") nanString = "65535";
                    if (nanString == "<minrange>") nanString = "0";
                    break;
                case 2:
                    dataType = ito::tUInt32;
                    if (nanString == "<maxrange>") nanString = "4294967295";
                    if (nanString == "<minrange>") nanString = "0";
                    break;
                case 3:
                    dataType = ito::tFloat32;
                    if (nanString == "<maxrange>") nanString = "3.4E+38";
                    if (nanString == "<minrange>") nanString = "-3.4E+38";
                    break;
                case 4:
                    dataType = ito::tInt8;
                    if (nanString == "<maxrange>") nanString = "127";
                    if (nanString == "<minrange>") nanString = "-128";
                    break;
                case 5:
                    dataType = ito::tInt16;
                    if (nanString == "<maxrange>") nanString = "32767";
                    if (nanString == "<minrange>") nanString = "-32768";
                    break;
                case 6:
                    dataType = ito::tInt32;
                    if (nanString == "<maxrange>") nanString = "2147483647";
                    if (nanString == "<minrange>") nanString = "-2147483648";
                    break;
                case 7:
                    dataType = ito::tFloat64;
                    if (nanString == "<maxrange>") nanString = "1.7E+308";
                    if (nanString == "<minrange>") nanString = "-1.7E+308";
                    break;
                default:
                    retValue += ito::RetVal(ito::retError, 0, "DataType not supported!");
                    dataType = -1;
                    break;
            }

            if (dataType != -1 && (valueUnit != "m" || std::abs(zscale-1.0) > std::numeric_limits<double>::epsilon()))
            {
                dataType = ito::tFloat64;
            }
        }
    }

    if (!retValue.containsError() && dataType > -1 && xsize > 0 && ysize > 0)
    {
        if (xsize == 1)
        {
            std::swap(xsize, ysize);
        }

        double xyFactor;

        if (xyUnit == "m")
        {
            xyFactor = 1.0;
        }
        else if (xyUnit == "cm")
        {
            xyFactor = 100.0;
        }
        else if (xyUnit == "mm")
        {
            xyFactor = 1000.0;
        }
        else if (xyUnit == "µm")
        {
            xyFactor = 1.0e6;
        }
        else if (xyUnit == "nm")
        {
            xyFactor = 1.0e9;
        }

        newObject = ito::DataObject(ysize, xsize, dataType);

        if (std::abs(yscale) > std::numeric_limits<ito::float64>::epsilon())
        {
            newObject.setAxisScale(0, yscale * xyFactor);
        }
        else
        {
            newObject.setAxisScale(0, 1.0);
        }
        newObject.setAxisUnit(0, xyUnit.data());

        if (std::abs(xscale) > std::numeric_limits<ito::float64>::epsilon())
        {
            newObject.setAxisScale(1, xscale * xyFactor);
        }
        else
        {
            newObject.setAxisScale(1, 1.0);
        }
        newObject.setAxisUnit(1, xyUnit.data());

        std::map<std::string, std::string>::iterator it = metaData.begin();
        while (it != metaData.end())
        {
            newObject.setTag(it->first, it->second);
            ++it;
        }

        //zscale *= zRes; //the resolution is not multiplied to the values since it only indicates the uncertainty of each discretized value (according to aBCR1.0 standard, basis for and referenced by aNIST1.0 standard)

        if (xyUnit == "m")
        {
            //zscale *= 1.0;
        }
        else if (xyUnit == "cm")
        {
            zscale *= 100.0;
        }
        else if (xyUnit == "mm")
        {
            zscale *= 1000.0;
        }
        else if (xyUnit == "µm")
        {
            zscale *= 1.0e6;
        }
        else if (xyUnit == "nm")
        {
            zscale *= 1.0e9;
        }

        newObject.setValueUnit(valueUnit.data());
    }

    return retValue;
};

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::readNistHeaderBinary(QFile &inFile, ito::DataObject &newObject, double &zscale, const int /*flags*/, const std::string &xyUnit, const std::string &valueUnit, QByteArray &nanString)
{
    inFile.seek(0);

    int read;
    char header[81];
    memset(header, 0, sizeof(header));
    read = inFile.read(header, 81);
    if (read != 81)
    {
        return ito::RetVal(ito::retError, 0, "file too short for header information.");
    }

    std::string version(header, 8);
    std::string manufacID(&(header[8]), 10);
    std::string createDate(&(header[18]), 12);
    std::string modDate(&(header[30]), 12);
    ito::uint16 numPoints = *((ito::uint16*)(&header[42]));
    ito::uint16 numProfiles = *((ito::uint16*)(&header[44]));
    ito::float64 scaleX = *((ito::float64*)(&header[46]));
    ito::float64 scaleY = *((ito::float64*)(&header[54]));
    zscale = *((ito::float64*)(&header[62]));
    ito::float64 zResolution = *((ito::float64*)(&header[70]));
    char compression = header[78];
    char dataTypeRaw = header[79];
    char checkType = header[80];

    if (version != "bISO-1.0")
    {
        return ito::RetVal(ito::retError, 0, "Version tag bISO-1.0 could not be found.");
    }

    ito::RetVal retValue;
    ito::float64 xscale = scaleX, yscale = scaleY, zRes = 1.0;
    int xsize = numPoints, ysize = numProfiles;
    int dataType = -1;
    std::map<std::string, std::string> metaData;

    metaData["ManufacID"] = manufacID;

    QDateTime date = QDateTime::fromString(QString::fromStdString(createDate), "ddMMyyyyHHmm");
    if (date.isValid())
    {
        metaData["CreateDate"] = date.toString("yyyy-MM-ddTHH:mm:00.0+00:00").toStdString(); //same format than x3p format
    }
    else
    {
        retValue += ito::RetVal(ito::retWarning, 0, "Header CreateDate can not be parsed in format DDMMYYYYHHMM");
    }

    date = QDateTime::fromString(QString::fromStdString(modDate), "ddMMyyyyHHmm");
    if (date.isValid())
    {
        metaData["ModDate"] = date.toString("yyyy-MM-ddTHH:mm:00.0+00:00").toStdString(); //same format than x3p format
    }
    else
    {
        retValue += ito::RetVal(ito::retWarning, 0, "Header ModDate can not be parsed in format DDMMYYYYHHMM");
    }

    if (numPoints <= 0 || numPoints > 65535)
    {
        //limit 65535 is given in ISO 25178-71
        retValue += ito::RetVal(ito::retError, 0, "Header NumPoints out of range (0, 65535]");
    }

    if (numProfiles <= 0 || numProfiles > 65535)
    {
        //limit 65535 is given in ISO 25178-71
        retValue += ito::RetVal(ito::retError, 0, "Header NumProfiles out of range (0, 65535]");
    }

    if (zResolution < 0.0)
    {
        zResolution = 1.0; //BCR Format 1.0 says: if zResolution negative, it is unknown
    }
    metaData["Zresolution"] = zResolution;

    if (compression != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "compression value must be 0 in file format.");
    }

    if (checkType != 0)
    {
        retValue += ito::RetVal(ito::retError, 0, "checksum value must be 0 in file format.");
    }

    int size = 0;

    switch (dataTypeRaw)
    {
    case 5:
        dataType = ito::tInt16;
        size = xsize * ysize * sizeof(ito::int16);
        break;
    case 6:
        dataType = ito::tInt32;
        size = xsize * ysize * sizeof(ito::int32);
        break;
    case 7:
        dataType = ito::tFloat64;
        size = xsize * ysize * sizeof(ito::float64);
        break;
    default:
        retValue += ito::RetVal(ito::retError, 0, "DataType not supported!");
        dataType = -1;
        break;
    }

    if (!retValue.containsError())
    {
        newObject = ito::DataObject(ysize, xsize, dataType);
        read = inFile.read((char*)newObject.rowPtr(0, 0), size);
        if (read < size)
        {
            retValue += ito::RetVal(ito::retError, 0, "file size is too small for all required data.");
        }
    }

    if (!retValue.containsError())
    {
        if (dataType != -1 && (valueUnit != "m" || std::abs(zscale - 1.0) > std::numeric_limits<double>::epsilon()) && newObject.getType() != ito::tFloat64)
        {
            ito::DataObject temp;
            newObject.copyTo(temp);
            retValue = temp.convertTo(newObject ,ito::tFloat64);
        }
    }

    if (!retValue.containsError() && dataType > -1 && xsize > 0 && ysize > 0)
    {
        if (xsize == 1)
        {
            std::swap(xsize, ysize);
        }

        double xyFactor;

        if (xyUnit == "m")
        {
            xyFactor = 1.0;
        }
        else if (xyUnit == "cm")
        {
            xyFactor = 100.0;
        }
        else if (xyUnit == "mm")
        {
            xyFactor = 1000.0;
        }
        else if (xyUnit == "µm")
        {
            xyFactor = 1.0e6;
        }
        else if (xyUnit == "nm")
        {
            xyFactor = 1.0e9;
        }

        if (std::abs(yscale) > std::numeric_limits<ito::float64>::epsilon())
        {
            newObject.setAxisScale(0, yscale * xyFactor);
        }
        else
        {
            newObject.setAxisScale(0, 1.0);
        }
        newObject.setAxisUnit(0, xyUnit.data());

        if (std::abs(xscale) > std::numeric_limits<ito::float64>::epsilon())
        {
            newObject.setAxisScale(1, xscale * xyFactor);
        }
        else
        {
            newObject.setAxisScale(1, 1.0);
        }
        newObject.setAxisUnit(1, xyUnit.data());

        std::map<std::string, std::string>::iterator it = metaData.begin();
        while (it != metaData.end())
        {
            newObject.setTag(it->first, it->second);
            ++it;
        }

        //zscale *= zRes; //the resolution is not multiplied to the values since it only indicates the uncertainty of each discretized value (according to aBCR1.0 standard, basis for and referenced by aNIST1.0 standard)

        if (xyUnit == "m")
        {
            //zscale *= 1.0;
        }
        else if (xyUnit == "cm")
        {
            zscale *= 100.0;
        }
        else if (xyUnit == "mm")
        {
            zscale *= 1000.0;
        }
        else if (xyUnit == "µm")
        {
            zscale *= 1.0e6;
        }
        else if (xyUnit == "nm")
        {
            zscale *= 1.0e9;
        }

        newObject *= zscale;


        newObject.setValueUnit(valueUnit.data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! readDataBlock
//----------------------------------------------------------------------------------------------------------------------------------
/** readDataBlock method, read the data-block of a .sdf-nist metrology file and write data to the corresponding dataObject.
*   @param [in]     inFile      Readonly File
*   @param [in|out] newObject   The dataObject to be created
*    @param [in]     zscale      The zscale, will be multiplied with the extracted data
*   @param [int]    flags       For later improvements, not used yet
*/
template<typename _Tp> ito::RetVal DataObjectIO::readDataBlock(QFile &inFile, ito::DataObject &newObject, const double zScale, const int /*flags*/, const QByteArray &nanString)
{
    ito::RetVal ret(ito::retOk);

    int x = 0, y = 0;

    int xsize = newObject.getSize(1);
    int ysize = newObject.getSize(0);
    int total = xsize * ysize;
    int c = 0; //total counter

    cv::Mat* dstMat = newObject.getCvPlaneMat(0);
    _Tp *p_Dst = NULL;

    int start = 0; //first index of number
    int end = 0; //last index of number
    int len = 0;
    QByteArray ba;
    QByteArray curLine = inFile.readLine();
    const char *curLineData;

    //data block must not be organized such that every row in matrix corresponds to one row in data block (see aBCR-1.0 standard)
    while (!curLine.contains("*") && !inFile.atEnd() && c < total)
    {
        //this loop gets every line in the file and extracts ranges of characters separated by one or multiple space characters (' ', tab, \r, \n...)
        //every range item is then converted to a number and set into the array.

        len = curLine.length();
        curLineData = curLine.data();

        while (end < len) //there are still characters left
        {
            while (start < len && isspace(curLineData[start]))
            {
                start++;
            }

            end = start + 1;
            while (end < len && !isspace(curLineData[end]))
            {
                end++;
            }

            if (end <= len && c < total)
            {
                ba = curLine.mid(start, end - start);

                if (c % xsize == 0)
                {
                    //next row
                    p_Dst = dstMat->ptr<_Tp>(c / xsize);
                }

                if (std::numeric_limits<_Tp>::is_exact)
                {
                    //exact data type is only possible, if zScale is equal to 1, else the data type is automatically switched to float64 in read-header-method
                    *p_Dst = cv::saturate_cast<_Tp>(ba.toDouble());
                    p_Dst++;
                }
                else
                {
                    if (ba == nanString)
                    {
                        *p_Dst = std::numeric_limits<_Tp>::quiet_NaN();
                    }
                    else
                    {
                        *p_Dst = cv::saturate_cast<_Tp>(ba.toDouble() * zScale);
                    }
                    p_Dst++;
                }
                c++;
            }

            start = end + 1;
        }

        curLine = inFile.readLine();
        end = start = 0;
    }

    if (c != total)
    {
        ret += ito::RetVal(ito::retWarning, 0, tr("number of values in data block does not correspond to NumPoints * NumProfiles of header block").toLatin1().data());
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveTifParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveTiffDoc = QObject::tr(
"Saves a real, 2D dataObject as tiff-file (tagged image format, 8bit and 16bit supported). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] or 16bit range [0, 65535] and saved using the given color palette \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] or 16bit range [0, 65535] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. They can be saved as colored image \n\
  without transparency ('rgb') or with considering the alpha channel as transparent values ('rgba'). If another \n\
  color palette than 'rgb' or 'rgba' is indicated, the object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. \n\
\n\
The following base color palettes exist (further, user-defined palettes can be used, too): \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* gray16: [0,65535] gray-values, 16bit \n\
* rgba: color with transparency, 8bit (rgba32 input only) \n\
* rgb: color without transparency, 8bit (rgba32 input only) \n\
* grayMarked: [0,255], gray-values, 0: violet, 255: red, 8bit \n\
* falseColor, falseColorIR: [0,255], (inverse) false colors, 8bit \n\
* hotIron: [0,255], hot iron map, 8bit \n\
* red: [0,255], from black to red, 8bit \n\
* green: [0,255], from black to green, 8bit \n\
* blue: [0,255], from black to blue, 8bit \n\
\n\
This filters uses OpenCV to save the image.");

/** saveTiffParams method, specifies the parameter list for saveDataObjectOpenCV as .Tiff images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveTiffParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Real, 2D data object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "gray16" , tr("Color palette name [gray, gray16, ...]").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveTiff
//----------------------------------------------------------------------------------------------------------------------------------
/** saveTiff method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "saveTiffParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::saveTiff(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retVal(ito::retOk);
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::tiffFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveJPGParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveJPGDoc = QObject::tr(
"Saves a real, 2D dataObject as jpg-file or jp2-file (only jp2 supports 16bit but is not much known). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] and saved using the given color palette \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. They can be saved as colored image \n\
  without transparency ('rgb') or with considering the alpha channel as transparent values ('rgba'). If another \n\
  color palette than 'rgb' or 'rgba' is indicated, the object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. \n\
\n\
The following base color palettes exist (further, user-defined palettes can be used, too): \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* rgba: color with transparency (converted to gray-levels however), 8bit (rgba32 input only) \n\
* rgb: color without transparency, 8bit (rgba32 input only) \n\
* grayMarked: [0,255], gray-values, 0: violet, 255: red, 8bit \n\
* falseColor, falseColorIR: [0,255], (inverse) false colors, 8bit \n\
* hotIron: [0,255], hot iron map, 8bit \n\
* red: [0,255], from black to red, 8bit \n\
* green: [0,255], from black to green, 8bit \n\
* blue: [0,255], from black to blue, 8bit \n\
\n\
This filters uses OpenCV to save the image.");

/** saveJPGParams method, specifies the parameter list for saveDataObjectOpenCV as .jpg and .jpg2000 images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveJPGParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Real, 2D data object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "falseColor" , tr("Color palette name [gray, gray16, rgb, rgba, <any-name-of-a-color-palette>]. 'gray16' supported for '.jp2' only").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("compression",ito::ParamBase::Int | ito::ParamBase::In, 0, 100, 90, tr("Compression rate, 0: high, 100: low").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveJPG
//----------------------------------------------------------------------------------------------------------------------------------
/** saveJPG method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "saveJPGParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::saveJPG(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::jpgFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePNGParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::savePNGDoc = QObject::tr(
"Saves a real, 2D dataObject as png-file (portable network graphic). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] or 16bit range [0, 65535] and saved using the given color palette \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] or 16bit range [0, 65535] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. They can be saved as colored image \n\
  without transparency ('rgb') or with considering the alpha channel as transparent values ('rgba'). If another \n\
  color palette than 'rgb' or 'rgba' is indicated, the object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. \n\
\n\
The following base color palettes exist (further, user-defined palettes can be used, too): \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* gray16: [0,65535] gray-values, 16bit \n\
* rgba: color with transparency, 8bit (rgba32 input only) \n\
* rgb: color without transparency, 8bit (rgba32 input only) \n\
* grayMarked: [0,255], gray-values, 0: violet, 255: red, 8bit \n\
* falseColor, falseColorIR: [0,255], (inverse) false colors, 8bit \n\
* hotIron: [0,255], hot iron map, 8bit \n\
* red: [0,255], from black to red, 8bit \n\
* green: [0,255], from black to green, 8bit \n\
* blue: [0,255], from black to blue, 8bit \n\
\n\
This filters uses OpenCV to save the image.");

/** savePNGParams method, specifies the parameter list for saveDataObjectOpenCV as .png images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::savePNGParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Real, 2D data object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "falseColor", tr("Color palette name [gray, gray16, rgba, rgb, <any-name-of-a-color-palette>].").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("compression",ito::ParamBase::Int | ito::ParamBase::In, 0, 9, 9, tr("Compression rate, 0: high, 9: low").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("addAlphaChannel",ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("If enabled and in case of floating point values, invalid values will we transparent (alpha=0), else full opacity.").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePNG
//----------------------------------------------------------------------------------------------------------------------------------
/** savePNG method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "savePNGParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::savePNG(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::pngFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveXPMParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveXPMDoc = QObject::tr(
"Saves a real, 2D dataObject as xpm-file (X PixMap) or xbm (X BitMap, black-white only). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] or 16bit range [0, 65535] and saved using the given color palette \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] or 16bit range [0, 65535] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. They can be saved as colored image \n\
  without transparency ('rgb') or with considering the alpha channel as transparent values ('rgba'). If another \n\
  color palette than 'rgb' or 'rgba' is indicated, the object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. \n\
\n\
The following base color palettes exist (further, user-defined palettes can be used, too): \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* rgba: color with transparency, 8bit (rgba32 input only) \n\
* rgb: color without transparency, 8bit (rgba32 input only) \n\
* grayMarked: [0,255], gray-values, 0: violet, 255: red, 8bit \n\
* falseColor, falseColorIR: [0,255], (inverse) false colors, 8bit \n\
* hotIron: [0,255], hot iron map, 8bit \n\
* red: [0,255], from black to red, 8bit \n\
* green: [0,255], from black to green, 8bit \n\
* blue: [0,255], from black to blue, 8bit \n\
\n\
This filters uses Qt to save the image. In the format xbm, a threshold at 128 is applied to separate black and white.");

/** savePNGParams method, specifies the parameter list for saveDataObjectOpenCV as .png images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveXPMParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Real, 2D data object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "gray", tr("Color palette name [gray, rgba, rgb, <any-name-of-a-color-palette>].").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveXPM
//----------------------------------------------------------------------------------------------------------------------------------
/** saveXPM method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "savePNGParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::saveXPM(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return saveDataObjectQt(paramsMand, paramsOpt, paramsOut, DataObjectIO::xpmFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveBMPParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveBMPDoc = QObject::tr(
"Saves a real, 2D dataObject as bmp-file or dip-file (bitmap, 8bit only). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] and saved using the given color palette \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. They can be saved as colored image \n\
  without transparency ('rgb') or with considering the alpha channel as transparent values ('rgba'). If another \n\
  color palette than 'rgb' or 'rgba' is indicated, the object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. \n\
\n\
The following base color palettes exist (further, user-defined palettes can be used, too): \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* rgba: color with transparency (converted to gray-levels however), 8bit (rgba32 input only) \n\
* rgb: color without transparency, 8bit (rgba32 input only) \n\
* grayMarked: [0,255], gray-values, 0: violet, 255: red, 8bit \n\
* falseColor, falseColorIR: [0,255], (inverse) false colors, 8bit \n\
* hotIron: [0,255], hot iron map, 8bit \n\
* red: [0,255], from black to red, 8bit \n\
* green: [0,255], from black to green, 8bit \n\
* blue: [0,255], from black to blue, 8bit \n\
\n\
This filters uses OpenCV to save the image.");

/** saveBMPParams method, specifies the parameter list for saveDataObjectOpenCV as .bmp images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveBMPParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Real, 2D data object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In,"falseColor", tr("Color palette name [gray, rgb, <any-name-of-a-color-palette>, ...]").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveBMP
//----------------------------------------------------------------------------------------------------------------------------------
/** saveBMP method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "saveBMPParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::saveBMP(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::bmpFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePPMParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::savePPMDoc = QObject::tr(
"Saves a real, 2D dataObject as ppm-file (portable pixel map, 8bit only). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] and saved using the given color palette \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. They can be saved as colored image \n\
  without transparency ('rgb') or with considering the alpha channel as transparent values ('rgba'). If another \n\
  color palette than 'rgb' or 'rgba' is indicated, the object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. \n\
\n\
The following base color palettes exist (further, user-defined palettes can be used, too): \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* rgba: color with transparency (converted to gray-levels however), 8bit (rgba32 input only) \n\
* rgb: color without transparency, 8bit (rgba32 input only) \n\
* grayMarked: [0,255], gray-values, 0: violet, 255: red, 8bit \n\
* falseColor, falseColorIR: [0,255], (inverse) false colors, 8bit \n\
* hotIron: [0,255], hot iron map, 8bit \n\
* red: [0,255], from black to red, 8bit \n\
* green: [0,255], from black to green, 8bit \n\
* blue: [0,255], from black to blue, 8bit \n\
\n\
This filters uses OpenCV to save the image.");

/** savePPMParams method, specifies the parameter list for saveDataObjectOpenCV as .ppm, .pgm, .pbm images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::savePPMParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Real, 2D data object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "falseColor", tr("Color palette name [gray, <any-name-of-a-color-palette>, ...]").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("binaryOut",ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Enable binary coding").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePPM
//----------------------------------------------------------------------------------------------------------------------------------
/** savePPM method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "savePPMParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::savePPM(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::ppmFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePGMParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::savePGMDoc = QObject::tr(
"Saves a real, 2D dataObject as pgm-file or pbm-file (portable gray map or portable bit map, 16bit only for pgm-format). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] or 16bit range [0, 65535] and saved using the given color palette (only gray-valued maps are allowed) \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] or 16bit range [0, 65535] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. The object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. Coloured images are not supported by this format.\n\
\n\
The following base color palettes are supported: \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* gray16: [0,65535] gray-values, 16bit \n\
\n\
This filters uses OpenCV to save the image.");

/** savePGMParams method, specifies the parameter list for saveDataObjectOpenCV as .ppm, .pgm, .pbm images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::savePGMParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Real, 2D data object").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "gray16", tr("Color palette name [gray, gray16]").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("binaryOut",ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Enable binary coding").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePGM
//----------------------------------------------------------------------------------------------------------------------------------
/** savePGM method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "savePPMParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::savePGM(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::pgmFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveRASParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveRASDoc = QObject::tr(
"Saves a real, 2D dataObject as sr-file or ras-file (Sun Raster, 8bit only). \n\
\n\
The following conventions hold for saving the image: \n\
\n\
* fixed-point data types (uint8, uint16, uint32, int8, int16, int32) will be scaled to the \n\
  supported 8bit range [0,255] and saved using the given color palette \n\
* float32 or float64 data types are scaled from [0.0, 1.0] to the supported 8bit range [0,255] \n\
  and saved using the given color palette. Values outside of [0.0, 1.0] are clipped to the boundary \n\
  value. Invalid values are saved as transparent values (alpha = 0). \n\
* rgba32 data type are saved depending on the given color palette. They can be saved as colored image \n\
  without transparency ('rgb') or with considering the alpha channel as transparent values ('rgba'). If another \n\
  color palette than 'rgb' or 'rgba' is indicated, the object is at first transformed to gray and than handled \n\
  as uint8 or uint16 data type. \n\
\n\
The following base color palettes exist (further, user-defined palettes can be used, too): \n\
\n\
* gray: [0,255] gray-values, 8bit \n\
* rgba: color with transparency (converted to gray-levels however), 8bit (rgba32 input only) \n\
* rgb: color without transparency, 8bit (rgba32 input only) \n\
* grayMarked: [0,255], gray-values, 0: violet, 255: red, 8bit \n\
* falseColor, falseColorIR: [0,255], (inverse) false colors, 8bit \n\
* hotIron: [0,255], hot iron map, 8bit \n\
* red: [0,255], from black to red, 8bit \n\
* green: [0,255], from black to green, 8bit \n\
* blue: [0,255], from black to blue, 8bit \n\
\n\
This filters uses OpenCV to save the image.");

/** savePPMParams method, specifies the parameter list for saveDataObjectOpenCV as .ras, .su images method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Image and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveRASParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In,"falseColor", tr("Color palette name [gray, ...]").toLatin1().data());
        paramsMand->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveRAS
//----------------------------------------------------------------------------------------------------------------------------------
/** saveRAS method, saves the DataObject into Hard drive as Image.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "saveRASParams" function.
*    Redirected to "saveDataObjectOpenCV"
*/
ito::RetVal DataObjectIO::saveRAS(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::sunFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DataObjectIO::checkAndModifyFilenameSuffix(QFileInfo &file, const QString &desiredAndAllowedSuffix, const QString &allowedSuffix2 /*= QString()*/, const QString &allowedSuffix3 /*= QString()*/)
{
    QString suffix = file.suffix();
    bool wrongSuffix = true;
    if (!desiredAndAllowedSuffix.isNull() && suffix.compare(desiredAndAllowedSuffix, Qt::CaseInsensitive) == 0)
    {
        wrongSuffix = false;
    }
    else if (!allowedSuffix2.isNull() && suffix.compare(allowedSuffix2, Qt::CaseInsensitive) == 0)
    {
        wrongSuffix = false;
    }
    else if (!allowedSuffix3.isNull() && suffix.compare(allowedSuffix3, Qt::CaseInsensitive) == 0)
    {
        wrongSuffix = false;
    }

    if (wrongSuffix)
    {
        QString fixedName = QString("%1/%2.%3").arg(file.absolutePath()).arg(file.completeBaseName()).arg(desiredAndAllowedSuffix);
        file.setFile(QDir::toNativeSeparators(fixedName));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveDataObjectOpenCV
//----------------------------------------------------------------------------------------------------------------------------------
/** saveDataObjectOpenCV method, saves the DataObject into Hard drive as Image using openCV.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "save-Format-Params" function.
*    It converts passed DataObject into corresponding Image as per given Image Format and stores it to specific location provided on Hard drive.
*/
ito::RetVal DataObjectIO::saveDataObjectOpenCV(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/, const ImageFormat &imageFormat)
{
    ito::RetVal ret = ito::retOk;

    // Optional parameters (sourceImage, filename, Format, bitscaling)
    const ito::DataObject *dObj = (*paramsMand)[0].getVal<ito::DataObject*>();
    const char *filename = (*paramsMand)[1].getVal<char*>();

    const char *palette = (*paramsMand)[2].getVal<char*>();

    if (filename == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("File name empty.").toLatin1().data());
    }
    if (palette == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Palette name empty.").toLatin1().data());
    }

    //allow all 1-plane data objects (independent on dimensions), but no complex ones
    if (dObj->calcNumMats() != 1)
    {
        ret += ito::RetVal(ito::retError, 0, tr("The given dataObject must have exactly one plane").toLatin1().data());
    }
    else if (dObj->getType() == ito::tComplex128 || dObj->getType() == ito::tComplex64)
    {
        ret += ito::RetVal(ito::retError, 0, tr("Complex data types not supported").toLatin1().data());
    }

    if (ret.containsError())
    {
        return ret;
    }

    cv::Mat saveMat;
    const cv::Mat *srcData = NULL;

    QString imgPalette(palette);
    QFileInfo fileName(QString::fromLatin1(filename));
    QDir folder = fileName.absoluteDir();

    if (fileName.fileName().isEmpty())
    {
        return ito::RetVal(ito::retError, 0, tr("Filename not valid.").toLatin1().data());
    }

    if (fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("File is not writeable.").toLatin1().data());
    }

    if (!folder.exists())
    {
        return ito::RetVal(ito::retError, 0, tr("The target directory '%1' does not exist.").arg(folder.absolutePath()).toLatin1().data());
    }

    std::vector<int> save_params;
    save_params.clear();

    bool addAlpha = false;
    bool gray16Supported = false;
    bool colorSupported = false;

    switch(imageFormat)
    {
        case DataObjectIO::tiffFormat:
            gray16Supported = true;
            colorSupported = true;
            checkAndModifyFilenameSuffix(fileName, "tif", "tiff");
            break;

        case DataObjectIO::ppmFormat:
            save_params.push_back(CV_IMWRITE_PXM_BINARY);
            save_params.push_back((*paramsOpt)[0].getVal<int>());
            checkAndModifyFilenameSuffix(fileName, "ppm");
            gray16Supported = false;
            colorSupported = true;
            break;

        case DataObjectIO::pgmFormat:

            save_params.push_back(CV_IMWRITE_PXM_BINARY);
            save_params.push_back((*paramsOpt)[0].getVal<int>());
            gray16Supported = true;
            colorSupported = false;
            checkAndModifyFilenameSuffix(fileName, "pgm", "pbm");
            if (fileName.suffix().compare("pbm", Qt::CaseInsensitive) == 0)
            {
                gray16Supported = false;
            }
            break;

        case DataObjectIO::jpgFormat:
        case DataObjectIO::jp2000Format:
            save_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            save_params.push_back((*paramsOpt)[0].getVal<int>());
            checkAndModifyFilenameSuffix(fileName, "jpg", "jpeg", "jp2");

            if (fileName.suffix().compare("jp2", Qt::CaseInsensitive) == 0)
            {
                gray16Supported = true;
            }
            else
            {
                gray16Supported = false;
            }
            colorSupported = true;
            break;

        case DataObjectIO::bmpFormat:
            checkAndModifyFilenameSuffix(fileName, "bmp", "dib");
            gray16Supported = false;
            colorSupported = true;
            break;

        case DataObjectIO::pngFormat:
            save_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            save_params.push_back((*paramsOpt)[0].getVal<int>());
            checkAndModifyFilenameSuffix(fileName, "png");
            addAlpha = (*paramsOpt)[1].getVal<int>() != 0 ? true : false;
            gray16Supported = true;
            colorSupported = true;
            break;

        case DataObjectIO::sunFormat:
            checkAndModifyFilenameSuffix(fileName, "ras", "sr");
            gray16Supported = false;
            colorSupported = true;
            break;

        default:
            return ito::RetVal(ito::retError, 0, tr("Image format not valid.").toLatin1().data());
    }

    //Creating an Image in Mono Format
    if (imgPalette.compare("gray") == 0)
    {
        srcData = dObj->getCvPlaneMat(0);

        switch(dObj->getType())
        {
            case ito::tUInt8:
                saveMat = *srcData;
            break;

            case ito::tInt8:
                ret += itom::io::transformScaledToUInt8<ito::int8>(saveMat, srcData);
            break;

            case ito::tUInt16:
                ret += itom::io::transformScaledToUInt8<ito::uint16>(saveMat, srcData);
            break;

            case ito::tInt16:
                ret += itom::io::transformScaledToUInt8<ito::int16>(saveMat, srcData);
            break;

            case ito::tUInt32:
                ret += itom::io::transformScaledToUInt8<ito::uint32>(saveMat, srcData);
            break;

            case ito::tInt32:
                ret += itom::io::transformScaledToUInt8< ito::int32>(saveMat, srcData);
            break;

            case ito::tFloat32:
                ret += itom::io::transformScaledToUInt8<ito::float32>(saveMat, srcData);
            break;

            case ito::tFloat64:
                ret += itom::io::transformScaledToUInt8<ito::float64>(saveMat, srcData);
            break;

            case ito::tRGBA32:
            {
                // we need a cast to remove constantness here
                ito::DataObject gray = ((ito::DataObject*)dObj)->toGray(ito::tUInt8);
                saveMat = *(gray.getCvPlaneMat(0));
            }
            break;

            default:
                return ito::RetVal(ito::retError, 0, tr("DataObject-Type could not be converted to unsigned int 8-bit.").toLatin1().data());
        }
    }
    else if (imgPalette.compare("gray16") == 0)
    {
        if (gray16Supported)
        {
            srcData = dObj->getCvPlaneMat(0);

            switch(dObj->getType())
            {
                case ito::tUInt8:
                    ret += itom::io::transformScaledToUInt16<ito::uint8>(saveMat, srcData);
                break;

                case ito::tInt8:
                    ret += itom::io::transformScaledToUInt16<ito::int8>(saveMat, srcData);
                break;

                case ito::tUInt16:
                    saveMat = *srcData;
                break;

                case ito::tInt16:
                    ret += itom::io::transformScaledToUInt16<ito::int16>(saveMat, srcData);
                break;

                case ito::tUInt32:
                    ret += itom::io::transformScaledToUInt16<ito::uint32>(saveMat, srcData);
                break;

                case ito::tInt32:
                    ret += itom::io::transformScaledToUInt16<ito::int32>(saveMat, srcData);
                break;

                case ito::tFloat32:
                    ret += itom::io::transformScaledToUInt16<ito::float32>(saveMat, srcData);
                break;

                case ito::tFloat64:
                    ret += itom::io::transformScaledToUInt16<ito::float64>(saveMat, srcData);
                break;

                case ito::tRGBA32:
                {
                    // we need a cast to removed constantness here
                    ito::DataObject gray = ((ito::DataObject*)dObj)->toGray(ito::tUInt16);
                    saveMat = *(gray.getCvPlaneMat(0));
                }
                break;

                default:
                    return ito::RetVal(ito::retError, 0, tr("DataObject-Type could not be converted to unsigned int 16-bit.").toLatin1().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support 16-Bit gray values.").toLatin1().data());
        }
    }
    else if (imgPalette.compare("rgba", Qt::CaseInsensitive) == 0)
    {
        if (colorSupported)
        {
            switch(dObj->getType())
            {
                case ito::tRGBA32:
                    {
                        srcData = dObj->getCvPlaneMat(0);
                        //saveMat = new cv::Mat(srcData->rows, srcData->cols, CV_8UC4);
                        //int from_to[] = {0,0, 1,1, 2,2, 3,3};
                        //cv::mixChannels(srcData, 1, saveMat, 1, from_to, 4);
                        saveMat = *srcData;
                        break;
                    }
                default:
                    return ito::RetVal(ito::retError, 0, tr("DataObject must be of type rgba32 in order to save it to the rgba color format.").toLatin1().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support color values.").toLatin1().data());
        }
    }
    else if (imgPalette.compare("rgb", Qt::CaseInsensitive) == 0)
    {
        if (colorSupported)
        {
            switch(dObj->getType())
            {
                case ito::tRGBA32:
                    {
                        srcData = dObj->getCvPlaneMat(0);
                        std::vector<cv::Mat> srcDataChannels;
                        srcDataChannels.resize(4);
                        cv::split(*srcData, srcDataChannels);
                        srcDataChannels[3] = 255; //set alpha to full opacity
                        cv::merge(srcDataChannels, saveMat);
                        break;
                    }
                default:
                    return ito::RetVal(ito::retError, 0, tr("DataObject must be of type rgba32 in order to save it to the rgb color format.").toLatin1().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support color values.").toLatin1().data());
        }
    }
    else if (!imgPalette.isEmpty())
    {
        ito::ItomPalette newPalette;
        ret += apiPaletteGetColorBarName(imgPalette, newPalette);

        if (ret.containsError())
        {
            return ret;
        }

        srcData = dObj->getCvPlaneMat(0);

        if (colorSupported)
        {
            if (addAlpha)
            {
                switch(dObj->getType())
                {
                    case ito::tUInt8:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint8>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt8:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int8>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt16:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint16>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt16:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int16>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint32>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int32>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::float32>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat64:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::float64>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tRGBA32:
                    {
                        // we need a cast to removed constantness here
                        ito::DataObject gray = ((ito::DataObject*)dObj)->toGray(ito::tUInt8);
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint8>(saveMat, gray.getCvPlaneMat(0), newPalette.colorVector256);
                    }
                    break;

                    default:
                        return ito::RetVal(ito::retError, 0, tr("Data object could not be converted to a gray image with 256 values (indexed8). No color palette could be applied.").toLatin1().data());
                }
            }
            else
            {
                switch(dObj->getType())
                {
                    case ito::tUInt8:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint8>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt8:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int8>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt16:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint16>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt16:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int16>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint32>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int32>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::float32>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat64:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::float64>(saveMat, srcData, newPalette.colorVector256);
                    break;

                    case ito::tRGBA32:
                    {
                        // we need a cast to removed constantness here
                        ito::DataObject gray = ((ito::DataObject*)dObj)->toGray(ito::tUInt8);
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint8>(saveMat, gray.getCvPlaneMat(0), newPalette.colorVector256);
                    }
                    break;

                    default:
                        return ito::RetVal(ito::retError, 0, tr("Data object could not be converted to a gray image with 256 values (indexed8). No color palette could be applied.").toLatin1().data());
                }
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support color values.").toLatin1().data());
        }
    }
    else
    {
        ret += ito::RetVal(ito::retError, 0, tr("Entered Image format is not supported").toLatin1().data());
    }

    if (!ret.containsError())
    {
        try
        {
            bool test = cv::imwrite(fileName.absoluteFilePath().toLatin1().data(), saveMat, save_params);
            if (test == false)
            {
                ret += ito::RetVal(ito::retError, 0, tr("cv::imwrite exited with false!").toLatin1().data());
            }
        }
        catch (cv::Exception exc)
        {
            ret += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveDataObjectQt
//----------------------------------------------------------------------------------------------------------------------------------
/** saveDataObjectQt method, saves the DataObject into Hard drive as Image using Qt (gif, xbm, xpm only).
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "save-Format-Params" function.
*    It converts passed DataObject into corresponding Image as per given Image Format and stores it to specific location provided on Hard drive.
*/
ito::RetVal DataObjectIO::saveDataObjectQt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/, const ImageFormat &imageFormat)
{
    ito::RetVal ret = ito::retOk;

    const ito::DataObject *dObj = (*paramsMand)[0].getVal<ito::DataObject*>();
    const char *filename = (*paramsMand)[1].getVal<char*>();

    const char *palette = (*paramsMand)[2].getVal<char*>();

    if (filename == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("File name empty.").toLatin1().data());
    }
    if (palette == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("Palette name empty.").toLatin1().data());
    }

    //allow all 1-plane data objects (independent on dimensions), but no complex ones
    if (dObj->calcNumMats() != 1)
    {
        ret += ito::RetVal(ito::retError, 0, tr("The given dataObject must have exactly one plane").toLatin1().data());
    }
    else if (dObj->getType() == ito::tComplex128 || dObj->getType() == ito::tComplex64)
    {
        ret += ito::RetVal(ito::retError, 0, tr("Complex data types not supported").toLatin1().data());
    }

    if (ret.containsError())
    {
        return ret;
    }

    cv::Mat *saveMat = NULL;
    const cv::Mat *srcData = NULL;

    QString imgPalette(palette);
    QFileInfo fileName(QString::fromLatin1(filename));

    if (fileName.fileName().isEmpty())
    {
        return ito::RetVal(ito::retError, 0, tr("Filename not valid.").toLatin1().data());
    }

    if (fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("File is not writeable.").toLatin1().data());
    }

    bool addAlpha = false;
    bool gray16Supported = false;
    bool colorSupported = false;
    QImage image;

    switch(imageFormat)
    {
        case DataObjectIO::gifFormat:
            checkAndModifyFilenameSuffix(fileName, "gif");
            gray16Supported = false;
            colorSupported = true;
            break;

        case DataObjectIO::xpmFormat:
            checkAndModifyFilenameSuffix(fileName, "xpm", "xbm");
            gray16Supported = false;
            colorSupported = true;
            break;

        default:
            return ito::RetVal(ito::retError, 0, tr("Image format not valid.").toLatin1().data());
    }

    if (imgPalette.compare("gray") == 0)
    {
        srcData = dObj->getCvPlaneMat(0);

        switch(dObj->getType())
        {
            case ito::tUInt8:
                ret += itom::io::transformScaledToUInt8<ito::uint8>(image, srcData);
            break;

            case ito::tInt8:
                ret += itom::io::transformScaledToUInt8<ito::int8>(image, srcData);
            break;

            case ito::tUInt16:
                ret += itom::io::transformScaledToUInt8<ito::uint16>(image, srcData);
            break;

            case ito::tInt16:
                ret += itom::io::transformScaledToUInt8<ito::int16>(image, srcData);
            break;

            case ito::tUInt32:
                ret += itom::io::transformScaledToUInt8<ito::uint32>(image, srcData);
            break;

            case ito::tInt32:
                ret += itom::io::transformScaledToUInt8< ito::int32>(image, srcData);
            break;

            case ito::tFloat32:
                ret += itom::io::transformScaledToUInt8<ito::float32>(image, srcData);
            break;

            case ito::tFloat64:
                ret += itom::io::transformScaledToUInt8<ito::float64>(image, srcData);
            break;

            case ito::tRGBA32:
            {
                // we need a cast to remove constantness here
                ito::DataObject gray = ((ito::DataObject*)dObj)->toGray(ito::tUInt8);
                ret += itom::io::transformScaledToUInt8<ito::uint8>(image, gray.getCvPlaneMat(0));
            }
            break;

            default:
                return ito::RetVal(ito::retError, 0, tr("DataObject-Type could not be converted to unsigned int 8-bit.").toLatin1().data());
        }
    }
    else if (imgPalette.compare("rgba", Qt::CaseInsensitive) == 0)
    {
        if (colorSupported)
        {
            switch(dObj->getType())
            {
                case ito::tRGBA32:
                    {
                        srcData = dObj->getCvPlaneMat(0);
                        image = QImage(srcData->cols, srcData->rows, QImage::Format_ARGB32);
                        const ito::Rgba32 *srcLine;
                        ito::uint32 *dstLine;
                        for (int row = 0; row < srcData->rows; ++row)
                        {
                            srcLine = srcData->ptr<ito::Rgba32>(row);
                            dstLine = (ito::uint32*)image.scanLine(0);
                            for (int col = 0; col < srcData->cols; ++col)
                            {
                                dstLine[col] = qRgba(srcLine[col].r, srcLine[col].g, srcLine[col].b, srcLine[col].a);
                            }
                        }
                        break;
                    }
                default:
                    return ito::RetVal(ito::retError, 0, tr("DataObject must be of type rgba32 in order to save it to the rgba color format.").toLatin1().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support color values.").toLatin1().data());
        }
    }
    else if (imgPalette.compare("rgb", Qt::CaseInsensitive) == 0)
    {
        if (colorSupported)
        {
            switch(dObj->getType())
            {
                case ito::tRGBA32:
                    {
                        srcData = dObj->getCvPlaneMat(0);
                        image = QImage(srcData->cols, srcData->rows, QImage::Format_RGB32);
                        const ito::Rgba32 *srcLine;
                        ito::uint32 *dstLine;
                        for (int row = 0; row < srcData->rows; ++row)
                        {
                            srcLine = srcData->ptr<ito::Rgba32>(row);
                            dstLine = (ito::uint32*)image.scanLine(0);
                            for (int col = 0; col < srcData->cols; ++col)
                            {
                                dstLine[col] = qRgb(srcLine[col].r, srcLine[col].g, srcLine[col].b);
                            }
                        }
                        break;
                    }
                default:
                    return ito::RetVal(ito::retError, 0, tr("DataObject must be of type rgba32 in order to save it to the rgba color format.").toLatin1().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support color values.").toLatin1().data());
        }
    }
    else if (!imgPalette.isEmpty())
    {
        ito::ItomPalette newPalette;
        ret += apiPaletteGetColorBarName(imgPalette, newPalette);

        if (ret.containsError())
        {
            return ret;
        }

        srcData = dObj->getCvPlaneMat(0);

        if (colorSupported)
        {
            if (addAlpha)
            {
                switch(dObj->getType())
                {
                    case ito::tUInt8:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint8>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt8:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int8>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt16:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint16>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt16:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int16>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint32>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int32>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::float32>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat64:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::float64>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tRGBA32:
                    {
                        // we need a cast to removed constantness here
                        ito::DataObject gray = ((ito::DataObject*)dObj)->toGray(ito::tUInt8);
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint8>(image, gray.getCvPlaneMat(0), newPalette.colorVector256);
                    }
                    break;

                    default:
                        return ito::RetVal(ito::retError, 0, tr("Data object could not be converted to a gray image with 256 values (indexed8). No color palette could be applied.").toLatin1().data());
                }
            }
            else
            {
                switch(dObj->getType())
                {
                    case ito::tUInt8:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint8>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt8:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int8>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt16:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint16>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt16:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int16>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tUInt32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint32>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tInt32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int32>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::float32>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tFloat64:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::float64>(image, srcData, newPalette.colorVector256);
                    break;

                    case ito::tRGBA32:
                    {
                        // we need a cast to removed constantness here
                        ito::DataObject gray = ((ito::DataObject*)dObj)->toGray(ito::tUInt8);
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint8>(image, gray.getCvPlaneMat(0), newPalette.colorVector256);
                    }
                    break;

                    default:
                        return ito::RetVal(ito::retError, 0, tr("Data object could not be converted to a gray image with 256 values (indexed8). No color palette could be applied.").toLatin1().data());
                }
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support color values.").toLatin1().data());
        }
    }
    else
    {
        ret += ito::RetVal(ito::retError, 0, tr("Image format is not supported").toLatin1().data());
    }

    if (!ret.containsError())
    {
        QImageWriter writer(fileName.absoluteFilePath(), fileName.suffix().toLatin1().data());
        bool test = writer.write(image);
        if (test == false)
        {
            ret += ito::RetVal(ito::retError, 0, tr("error saving the image file (%1)!").arg(writer.errorString()).toLatin1().data());
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadImageParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadImageDoc = QObject::tr(
"load the following image formats from a file to a 2D data object: \n\
\n\
* png: 8bit/16bit, color available, transparency available \n\
* tiff/tif: 8bit/16bit, color available \n\
* jpg: 8bit, color available\n\
* jp2: 8bit/16bit, color available\n\
* ras/sr: 8bit, color available \n\
* ppm: 8bit, color available \n\
* pgm: 8bit/16bit \n\
* pbm: 8bit, color available \n\
* bmp/dip: 8bit, color available \n\
* gif: 8bit, color available, transparency available \n\
* xbm: 8bit, color available, transparency available (X BitMap)\n\
* xpm: 8bit, color available, transparency available (X PixMap)\n\
\n\
The file format only provides color or transparency data if indicated above. \n\
\n\
You can choose which channel (colorElement) of the loaded file should be used for the data object. \n\
`asIs` loads the file as it is: monochrome formats are loaded as uint8 or uint16 data object, while \n\
colored file formats are loaded as rgba32 data object. The color elements `alpha`, `R`, `G`, `B`, `RGB`, \n\
`RGBA` are only available for colored file formats are only load the selected channels to either a uint8/uint16 data object \n\
or a rgba32 data object (rgb and rgba only). `GRAY` converts a colored file format to grayscale before loading it \n\
to an uint8 or uint16 data object. \n\
\n\
All file formats are loaded using OpenCV, besides gif, xbm, xpm which are loaded via Qt.");

/** loadImageParams method, specifies the parameter list for loadImage method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting the specified Image from its specified location on Hard drive into itom DataObject.
*/
ito::RetVal DataObjectIO::loadImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("destinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("DataObject that will be filled with the loaded file content").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);

        ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::String, "asIs");
        m->addItem("alpha");
        m->addItem("R");
        m->addItem("G");
        m->addItem("B");
        m->addItem("RGB");
        m->addItem("RGBA");
        m->addItem("GRAY");
        param = ito::Param("colorElement", ito::ParamBase::String | ito::ParamBase::In, "asIs", tr("Color element: asIs (default) | alpha | R | G | B | RGB | ARGB | GRAY; 'asIs' ignores alpha channel from supported file types (use rgba instead).").toLatin1().data());
        param.setMeta(m,true); //takes ownership of m
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadDataObject
//----------------------------------------------------------------------------------------------------------------------------------
/** loadDataObject method, retrieves the Image data and creates corresponding DataObject.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "loadDataObjectParams" function.
*    It retrieves the Image data from Image location passed as parameter from Hard drive and loads a corresponding Itom DataObject.
*/
ito::RetVal DataObjectIO::loadImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * paramsOut)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
    QFileInfo fileinfo(QString::fromLatin1(filename));

    if (fileinfo.suffix().compare("gif", Qt::CaseInsensitive) == 0 || fileinfo.suffix().compare("xbm", Qt::CaseInsensitive) == 0 || fileinfo.suffix().compare("xpm", Qt::CaseInsensitive) == 0)
    {
        ret += loadDataObject(paramsMand, paramsOpt, paramsOut);
    }
    else
    {
        char *colorElement = NULL;
        colorElement = (*paramsOpt)[0].getVal<char*>();

        ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();

        if (dObjDst == NULL)
        {
            return ito::RetVal(ito::retError,0,tr("Destination dataObject is invalid.").toLatin1().data());
        }

        ito::DataObject tempObject;

        int flags = 0;
        QString colorFormat(colorElement);

        bool reduceChannel = false;

        if (colorFormat.isEmpty() || colorFormat.compare("asIs", Qt::CaseInsensitive) == 0)
        {
            flags = CV_LOAD_IMAGE_ANYDEPTH;
            flags *= -1;
            reduceChannel = false;
        }
        else if (colorFormat.compare("alpha", Qt::CaseInsensitive) == 0)
        {
            flags = CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH;
            flags *= -1;
            reduceChannel = true;
        }
        else if (colorFormat.compare("R", Qt::CaseInsensitive) == 0 || colorFormat.compare("G", Qt::CaseInsensitive) == 0 || colorFormat.compare("B", Qt::CaseInsensitive) == 0)
        {
            flags = CV_LOAD_IMAGE_COLOR | CV_LOAD_IMAGE_ANYDEPTH;
            flags *= -1;
            reduceChannel = true;
        }
        else if (colorFormat.compare("gray", Qt::CaseInsensitive) == 0 || colorFormat.compare("grey", Qt::CaseInsensitive) == 0)
        {
            flags = CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH;

            reduceChannel = false;
        }
        else
        {
            reduceChannel = false;
            flags = CV_LOAD_IMAGE_COLOR;
            flags *= -1;
        }

        cv::Mat image;

        if (!fileinfo.exists())
        {
            ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' does not exist.").toLatin1().data(), filename);
        }
        else
        {
            try
            {
                image = cv::imread(filename, flags);
            }
            catch (cv::Exception exc)
            {
                ret += ito::RetVal(ito::retError, 0, tr("%1").arg((exc.err).c_str()).toLatin1().data());
            }
        }

        if (!ret.containsError())
        {
            if (image.cols == 0 || image.rows == 0)
            {
                ret += ito::RetVal(ito::retError, 0, "Error while reading image. Probably, the bit depth, compression... is not supported");
            }
        }

        if (!ret.containsError())
        {
            int imageType = 0;

            switch(image.type())
            {
                case CV_8U:
                    imageType = ito::tUInt8;
                    break;
                case CV_16U:
                    imageType = ito::tUInt16;
                    break;
                case CV_8UC3:
                    if (reduceChannel)
                    {
                        imageType = ito::tUInt8;
                    }
                    else
                    {
                        imageType = ito::tRGBA32;
                    }
                    break;
                case CV_8UC4:
                    if (reduceChannel)
                    {
                        imageType = ito::tUInt8;
                    }
                    else
                    {
                        imageType = ito::tRGBA32;
                    }
                    break;
                case CV_16UC3:
                case CV_16UC4:
                    if (reduceChannel)
                    {
                        imageType = ito::tUInt16;
                    }
                    else
                    {
                        return ito::RetVal(ito::retError,0,tr("The loaded image has multiple channels with a bitdepth of uint16 each. This is currently unsupported.").toLatin1().data());
                    }
                    break;
                default:
                    return ito::RetVal(ito::retError,0,tr("Format of the image is currently not supported by itom.").toLatin1().data());
            }

            tempObject = ito::DataObject(image.rows, image.cols, imageType);
            cv::Mat* tempObject_ = tempObject.getCvPlaneMat(0);

            if (!reduceChannel)
            {
                if (image.type() == CV_8UC3)
                {
                    ito::Rgba32* dst = NULL;
                    const cv::Vec3b* scr = NULL;

                    for (int y = 0; y < image.rows; y++)
                    {
                        dst = ((cv::Mat*)(tempObject.get_mdata()[0]))->ptr<ito::Rgba32>(y);
                        scr = image.ptr<const cv::Vec3b>(y);
                        for (int x = 0; x < image.cols; x++)
                        {
                            dst[x].alpha() = 255;
                            dst[x].red()   = scr[x][2];
                            dst[x].green() = scr[x][1];
                            dst[x].blue()  = scr[x][0];
                        }
                    }
                }
                else if (image.type() == CV_16UC3)
                {
                    ret += ito::RetVal(ito::retError,0,tr("Color import of channels with uint16 bitdepth not supported.").toLatin1().data());
                }
                else if (image.type() == CV_8UC4)
                {
                    ito::Rgba32* dst = NULL;
                    const cv::Vec4b* scr = NULL;

                    if (colorFormat.compare("rgb", Qt::CaseInsensitive) == 0)
                    {
                        for (int y = 0; y < image.rows; y++)
                        {
                            dst = tempObject_->ptr<ito::Rgba32>(y);
                            scr = image.ptr<const cv::Vec4b>(y);
                            for (int x = 0; x < image.cols; x++)
                            {
                                dst[x].alpha() = 255;
                                dst[x].red()   = scr[x][2];
                                dst[x].green() = scr[x][1];
                                dst[x].blue()  = scr[x][0];
                            }
                        }
                    }
                    else
                    {
                        for (int y = 0; y < image.rows; y++)
                        {
                            dst = tempObject_->ptr<ito::Rgba32>(y);
                            scr = image.ptr<const cv::Vec4b>(y);
                            for (int x = 0; x < image.cols; x++)
                            {
                                dst[x].alpha() = scr[x][3];
                                dst[x].red()   = scr[x][2];
                                dst[x].green() = scr[x][1];
                                dst[x].blue()  = scr[x][0];
                            }
                        }
                    }
                }
                else if (image.type() == CV_16UC4)
                {
                    ret += ito::RetVal(ito::retError,0,tr("Color import of channels with uint16 bitdepth not supported.").toLatin1().data());
                }
                else
                {
                    image.copyTo(*tempObject_);
                }
            }
            else
            {
                int colIndex;
                if (colorFormat.compare("R", Qt::CaseInsensitive) == 0)
                {
                    colIndex = 2;
                }
                else if (colorFormat.compare("G", Qt::CaseInsensitive) == 0)
                {
                    colIndex = 1;
                }
                else if (colorFormat.compare("B", Qt::CaseInsensitive) == 0)
                {
                    colIndex = 0;
                }
                else if (colorFormat.compare("alpha", Qt::CaseInsensitive) == 0)
                {
                    colIndex = 3;
                }
                else
                {
                    colIndex = -1;
                    ret += ito::RetVal(ito::retError,0,tr("Color format of the file is currently not compatible with itom. Wait for next itom version.").toLatin1().data());
                }

                if (!ret.containsError())
                {
                    if (image.type() == CV_8UC3)
                    {
                        if (colIndex < 0 || colIndex > 3)
                        {
                            ret += ito::RetVal(ito::retError, 0, "loaded image does not contain the required channel");
                        }
                        else
                        {
                            ito::uint8* dst = NULL;
                            const cv::Vec3b* scr = NULL;

                            for (int y = 0; y < image.rows; y++)
                            {
                                dst = tempObject_->ptr<ito::uint8>(y);
                                scr = image.ptr<const cv::Vec3b>(y);
                                for (int x = 0; x < image.cols; x++)
                                {
                                    dst[x] = scr[x][colIndex];
                                }
                            }
                        }
                    }
                    else if (image.type() == CV_8UC4)
                    {
                        ito::uint8* dst = NULL;
                        const cv::Vec4b* scr = NULL;

                        for (int y = 0; y < image.rows; y++)
                        {
                            dst = tempObject_->ptr<ito::uint8>(y);
                            scr = image.ptr<const cv::Vec4b>(y);
                            for (int x = 0; x < image.cols; x++)
                            {
                                dst[x] = scr[x][colIndex];
                            }
                        }
                    }
                    else if (image.type() == CV_16UC3)
                    {
                        if (colIndex < 0 || colIndex > 3)
                        {
                            ret += ito::RetVal(ito::retError, 0, "loaded image does not contain the required channel");
                        }
                        else
                        {
                            ito::uint16* dst = NULL;
                            const cv::Vec3w* scr = NULL;

                            for (int y = 0; y < image.rows; y++)
                            {
                                dst = tempObject_->ptr<ito::uint16>(y);
                                scr = image.ptr<const cv::Vec3w>(y);
                                for (int x = 0; x < image.cols; x++)
                                {
                                    dst[x] = scr[x][colIndex];
                                }
                            }
                        }
                    }
                    else if (image.type() == CV_16UC4)
                    {
                        ito::uint16* dst = NULL;
                        const cv::Vec4w* scr = NULL;

                        for (int y = 0; y < image.rows; y++)
                        {
                            dst = tempObject_->ptr<ito::uint16>(y);
                            scr = image.ptr<const cv::Vec4w>(y);
                            for (int x = 0; x < image.cols; x++)
                            {
                                dst[x] = scr[x][colIndex];
                            }
                        }
                    }
                    else
                    {
                        ret += ito::RetVal(ito::retError,0,tr("No coloured data object with 3 or 4 color channels has been loaded.").toLatin1().data());
                    }
                }
            }
        }

        if (!ret.containsError())
        {
            *dObjDst = tempObject;
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadItomIDO
//----------------------------------------------------------------------------------------------------------------------------------
/** loadDataObject method, retrieves xml-compatible raw data and creates corresponding DataObject.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "loadItomIDOParams" function.
*    It retrieves the xml-compatible raw data from file location passed as parameter from Hard drive and loads a corresponding Itom DataObject.
*/
ito::RetVal DataObjectIO::loadItomIDO(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * /*paramsOpt*/, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();

    ito::DataObject *dObjDst = (*paramsMand)[0].getVal<ito::DataObject*>();

    if (dObjDst == NULL)
    {
        return ito::RetVal(ito::retError,0,tr("Destination dataObject is invalid.").toLatin1().data());
    }

    ito::DataObject tempObject;
    QString fileNameQt = QLatin1String(filename);
    ret += ito::loadXML2DOBJ(&tempObject, fileNameQt, false);

    if (!ret.containsError())
    {
        *dObjDst = tempObject;
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadItomIDOParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadItomIDODoc = QObject::tr(
"Tries to load the given ido or idh-file as itom dataObject. Data in the given file \n\
must have been saved using `saveIDO` in a xml-compatible file format, whereas *.idh only \n\
contains meta information of a data object and *.ido contains the full object.");

/** loadItomIDOParams method, specifies the parameter list for loadItomIDO method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for reading a xml-compatible binary and converting into DataObject from Hard drive.
*/
ito::RetVal DataObjectIO::loadItomIDOParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("DataObject that will be filled with the loaded file content").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveItomIDOParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveItomIDODoc = QObject::tr(
"The given dataObject of any size and data format is saved in an itom-specific xml-file (suffix: ido, idh). \n\
\n\
It is possible to define if only meta information (tags, scaling, offset...) should be saved in the xml-file (suffix: idh, itom data header) \n\
or if the array and meta information should be saved (suffix: ido, itom data object). Data is always saved in a base64-encoded format while the header text \n\
is mostly written in plain, ascii text. Using this filter, the data object including all its meta information is saved and \n\
can be reload using `loadIDO`.");

/** saveItomIDOParams method, specifies the parameter list for saveItomIDO method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into the itom native xml-compatible binary and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveItomIDOParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Any type of dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("headerOnly", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 0 the complete dataObject is saved (data + metadata) [default], else the filter saves only metadata").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("tags2Binary", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 0 meta information are saved as clear text, else double tags are saved as binary").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveItomIDO
//----------------------------------------------------------------------------------------------------------------------------------
/** saveItomIDO method, saves the DataObject into Hard drive as xml-compatible raw file using QT XML-Parser.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "saveItomIDOParams" function.
*    It converts passed DataObject into xml-compatible data and stores it to specific location provided on Hard drive.
*/
ito::RetVal DataObjectIO::saveItomIDO(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();

    bool onlyHeader = (*paramsOpt)[0].getVal<int>() > 0 ? true : false;
    bool asBinary   = (*paramsOpt)[1].getVal<int>() > 0 ? true : false;

    ito::DataObject *dObjDst = (*paramsMand)[0].getVal<ito::DataObject*>();

    if (dObjDst == NULL)
    {
        return ito::RetVal(ito::retError,0,tr("Destination dataObject is invalid.").toLatin1().data());
    }

    ito::DataObject tempObject;
    QString fileNameQt = QLatin1String(filename);
    ret += ito::saveDOBJ2XML(dObjDst, fileNameQt, onlyHeader, asBinary);

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveDataToTxt
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::saveDataToTxtDoc = QObject::tr("saves data to an ascii-based file like txt, csv, tsv or space separated values.\n\n\
");


template<typename _Tp> ito::RetVal doWriteDataD(ito::DataObject *dObjSrc, QTextStream *dataOut, const int asTuple, const int noPhys,
    QChar wrapSign, QChar separatorSign, QString separatorLines, QString separatorMatrices, const int precision)
{
    ito::RetVal retval(ito::retOk);

    ito::float64 zscale(1.0);
    ito::float64 zoffset(0.0);
    if (!noPhys)
    {
        zscale = dObjSrc->getValueScale();
        zoffset = dObjSrc->getValueOffset();
    }


    if (!asTuple)
    {
        if (!wrapSign.isNull())
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << wrapSign << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale), 'g', precision) << wrapSign;
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1) - 1)
                            *dataOut << separatorSign;
                    }
                    if (ny < dObjSrc->getSize(dObjSrc->getDims() - 2) - 1)
                        *dataOut << separatorLines;
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
        else
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale), 'g', precision);
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1) - 1)
                            *dataOut << separatorSign;
                    }
                    if (ny < dObjSrc->getSize(dObjSrc->getDims() - 2) - 1)
                        *dataOut << separatorLines;
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
    }
    else
    {
        ito::float64 xscale(1.0);
        ito::float64 xoffset(0.0);
        ito::float64 yscale(1.0);
        ito::float64 yoffset(0.0);

        if (!noPhys)
        {
            xscale = dObjSrc->getAxisScale(1);
            xoffset = dObjSrc->getAxisOffset(1);
            yscale = dObjSrc->getAxisScale(0);
            yoffset = dObjSrc->getAxisOffset(0);
        }

        if (!wrapSign.isNull())
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << wrapSign << QString::number((nx - xoffset) * xscale, 'g', precision) << wrapSign << separatorSign;
                        *dataOut << wrapSign << QString::number((ny - xoffset) * yscale, 'g', precision) << wrapSign << separatorSign;
                        *dataOut << wrapSign << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale), 'g', precision) << wrapSign;
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1) - 1)
                            *dataOut << separatorLines;
                    }
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
        else
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << QString::number((nx - xoffset) * xscale, 'g', precision) << separatorSign;
                        *dataOut << QString::number((ny - xoffset) * yscale, 'g', precision) << separatorSign;
                        *dataOut << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale), 'g', precision);
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1) - 1)
                            *dataOut << separatorLines;
                    }
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
    }

    return retval;
}

template<typename _Tp> ito::RetVal doWriteData(const ito::DataObject *dObjSrc, QTextStream *dataOut, const int asTuple, const int noPhys,
    QChar wrapSign, QChar separatorSign, QString separatorLines, QString separatorMatrices)
{
    ito::RetVal retval(ito::retOk);

    ito::float64 zscale(1.0);
    ito::float64 zoffset(0.0);
    if (!noPhys)
    {
        zscale = dObjSrc->getValueScale();
        zoffset = dObjSrc->getValueOffset();
    }

    if (!asTuple)
    {
        if (!wrapSign.isNull())
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << wrapSign << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale)) << wrapSign;
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1) -1 )
                            *dataOut << separatorSign;
                    }
                    if (ny < dObjSrc->getSize(dObjSrc->getDims() - 2) -1 )
                        *dataOut << separatorLines;
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
        else
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale));
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1) - 1 )
                            *dataOut << separatorSign;
                    }
                    if (ny < dObjSrc->getSize(dObjSrc->getDims() - 2) - 1 )
                        *dataOut << separatorLines;
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
    }
    else
    {
        ito::float64 xscale(1.0);
        ito::float64 xoffset(0.0);
        ito::float64 yscale(1.0);
        ito::float64 yoffset(0.0);

        if (!noPhys)
        {
            xscale = dObjSrc->getAxisScale(1);
            xoffset = dObjSrc->getAxisOffset(1);
            yscale = dObjSrc->getAxisScale(0);
            yoffset = dObjSrc->getAxisOffset(0);
        }

        if (!wrapSign.isNull())
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << wrapSign << QString::number((nx - xoffset) * xscale) << wrapSign << separatorSign;
                        *dataOut << wrapSign << QString::number((ny - xoffset) * yscale) << wrapSign << separatorSign;
                        *dataOut << wrapSign << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale)) << wrapSign;
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1) )
                            if ((nm != dObjSrc->getNumPlanes() - 1) ||
                                (ny != dObjSrc->getSize(dObjSrc->getDims() - 2) - 1) ||
                                (nx != dObjSrc->getSize(dObjSrc->getDims() - 1) - 1))
                                *dataOut << separatorLines;
                    }
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
        else
        {
            for (int nm = 0; nm < dObjSrc->getNumPlanes(); nm++)
            {
                for (int ny = 0; ny < dObjSrc->getSize(dObjSrc->getDims() - 2); ny++)
                {
                    _Tp *srcPtr = (_Tp*)dObjSrc->rowPtr(nm, ny);
                    for (int nx = 0; nx < dObjSrc->getSize(dObjSrc->getDims() - 1); nx++)
                    {
                        *dataOut << QString::number((nx - xoffset) * xscale) << separatorSign;
                        *dataOut << QString::number((ny - xoffset) * yscale) << separatorSign;
                        *dataOut << QString::number(cv::saturate_cast<_Tp>((srcPtr[nx] - zoffset) * zscale));
                        if (nx < dObjSrc->getSize(dObjSrc->getDims() - 1))
                            if ((nm != dObjSrc->getNumPlanes() - 1) ||
                                (ny != dObjSrc->getSize(dObjSrc->getDims() - 2) - 1) ||
                                (nx != dObjSrc->getSize(dObjSrc->getDims() - 1) - 1))
                                *dataOut << separatorLines;
                    }
                }
                if (nm < dObjSrc->getNumPlanes() - 1)
                    *dataOut << separatorMatrices;
            }
        }
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/**
*   saveDataToTxtParams method, specifies the parameter list for saveDataToTxtParams method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*   @param [out] outVals    optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Raw-Text data and save it into Hard drive.
*/
ito::RetVal DataObjectIO::saveDataToTxtParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("dataObject holding data to save").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination file name").toLatin1().data());
        paramsMand->append(param);

//        param = ito::Param("ignoreLines", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("Ignore the first n-lines.").toLatin1().data());
//        paramsOpt->append(param);

//        param = ito::Param("asMatrix", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("(1) Try to interprete list elements with 3 elements per row as a matrix or (0) load as written.").toLatin1().data());
//        paramsOpt->append(param);

        param = ito::Param("saveAsTuple", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("(1) save values as list of points (x, y, z).").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("noPhysicalValues", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("(1) ignore scale and offset").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("separatorSign", ito::ParamBase::String | ito::ParamBase::In, " ", tr("Uses this as the separator between elements. Default is space \" \".").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("separatorLines", ito::ParamBase::String | ito::ParamBase::In, "\r\n", tr("Uses this as the separator between lines of a matrix. Default is \\r\\n.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("separatorPlanes", ito::ParamBase::String | ito::ParamBase::In, "\r\n", tr("Uses this as the separator between matrix planes. Default is \\r\\n.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("decimalSign", ito::ParamBase::String | ito::ParamBase::In, "<guess>", tr("Uses this as the sign for decimal numbers. If <guess>, default is system default.").toLatin1().data());
        ito::StringMeta sm = ito::StringMeta(ito::StringMeta::String, "<guess>");
        sm.addItem(".");
        sm.addItem(",");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("wrapSign", ito::ParamBase::String | ito::ParamBase::In, "", tr("Sometimes numbers are wrapped by a sign (.e.g '2.3' or \"4.5\"). If so, indicate the character(s) that wrap the numbers.").toLatin1().data());
        sm.addItem("");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("encoding", ito::ParamBase::String | ito::ParamBase::In, "UTF-8", tr("encoding of text file, e.g. UTF-8, UTF-16, ISO 8859-1... Default: empty string -> the encoding is guessed due to a auto-detection of the first 64 bytes in the text file (using the BOM (Byte Order Mark)).").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
/**
*   saveDataToTxt method, writes the ascii data and creates corresponding DataObject.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*   @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "loadNistSDF" function.
*    It retrieves the ascii-data from file location passed as parameter from Hard drive and loads a corresponding Itom DataObject.
*/
ito::RetVal DataObjectIO::saveDataToTxt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
//    QFileInfo fileinfo(QString::fromLatin1(filename));
//    QFile dataOut(fileinfo.canonicalFilePath());
    QString filename_ = QLatin1String(filename);
    QFileInfo fileinfo(filename_);
    QFile dataOut(filename_);

    ito::DataObject *dObjSrc = (*paramsMand)[0].getVal<ito::DataObject*>();

    if (dObjSrc == NULL)
    {
        ret += ito::RetVal::format(ito::retError, 0, tr("DataObject not initialized").toLatin1().data(), filename);
    }
    else if (!dataOut.open(QIODevice::WriteOnly))
    {
        ret += ito::RetVal::format(ito::retError, 0, tr("The file '%s' is no readable file.").toLatin1().data(), filename);
    }
    else
    {
        QChar separatorSign(' ');
        QString separatorLines("\r\n");
        QString separatorMatrices("\r\n");
        QChar decimalSign(0);

        int asTuple = (*paramsOpt)[0].getVal<int>();
        int noPhys = (*paramsOpt)[1].getVal<int>();

        if ((*paramsOpt)[2].getVal<char*>() != NULL)
        {
            separatorSign = paramsOpt->at(2).getVal<char*>()[0];
        }

        if ((*paramsOpt)[3].getVal<char*>() != NULL)
        {
            separatorLines = paramsOpt->at(3).getVal<char*>();
        }

        if ((*paramsOpt)[4].getVal<char*>() != NULL)
        {
            separatorMatrices = paramsOpt->at(4).getVal<char*>();
        }

        if ((*paramsOpt)[5].getVal<char*>()[0] != '<') //!= <guess>
        {
            decimalSign = paramsOpt->at(5).getVal<char*>()[0];
        }

        QString wrapSign_ = QString::fromLatin1(paramsOpt->at(6).getVal<char*>());
        QChar wrapSign = wrapSign_.size() > 0 ? wrapSign_[0] : QChar();
        QString encoding = paramsOpt->at(7).getVal<char*>();

        if (encoding != "")
        {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
            if (QTextCodec::codecForName(encoding.toLatin1()) == NULL)
#else
            if (!QStringConverter::encodingForName(encoding.toLatin1()).has_value())
#endif
            {
                ret += ito::RetVal::format(ito::retError, 0, "encoding '%s' is unknown", encoding.toLatin1().data());
            }
        }
        else
        {
            // default to UTF-8
            encoding = "UTF-8";
        }

        QLocale local(QLocale::C); //per default a decimal sign is a dot (.) and the thousands group separator is (,)
        if (decimalSign == ',')
        {
            local = QLocale(QLocale::German); //if the decimal sign is a comma (,), the thousands group separator is assumed to be a dot (.)
        }

        QLocale::setDefault(local);
        QTextStream textStream(&dataOut);

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
        textStream.setCodec(encoding.toLatin1().data());
#else
        textStream.setEncoding(QStringConverter::encodingForName(encoding.toLatin1().data()).value());
#endif

        switch (dObjSrc->getType())
        {
            case ito::tUInt8:
                ret += doWriteData<ito::uint8>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;
            case ito::tInt8:
                ret += doWriteData<ito::int8>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;
            case ito::tUInt16:
                ret += doWriteData<ito::uint16>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;
            case ito::tInt16:
                ret += doWriteData<ito::int16>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;
            case ito::tUInt32:
                ret += doWriteData<ito::uint32>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;
            case ito::tInt32:
                ret += doWriteData<ito::int32>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;

            case ito::tFloat32:
                ret += doWriteData<ito::float32>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;
            case ito::tFloat64:
                ret += doWriteData<ito::float64>(dObjSrc, &textStream, asTuple, noPhys, wrapSign, separatorSign, separatorLines, separatorMatrices);
            break;
            default:
                ret += ito::RetVal(ito::retError, 0, tr("data format not supported").toLatin1().data());
            break;
        }
    }

    if (dataOut.isOpen())
    {
        dataOut.close();
    }

    return ret;
}


//----------------------------------------------------------------------------------------------------------------------------------
//! loadDataFromTxtParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadDataFromTxtDoc = QObject::tr("loads an ascii-based data file like txt, csv, tsv or space separated values. \n\
\n\
The text file is loaded line by line and tried to be interpreted as an array. It is possible to ignore the first n lines (optional parameter ignoreLines). \n\
The full content of these ignored lines is then saved in the tag 'ignoredLines' of the destinationObject.");

/** loadNistSDFParams method, specifies the parameter list for loadNistSDFParams method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Raw-Text data and save it into Hard drive.
*/
ito::RetVal DataObjectIO::loadDataFromTxtParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("ignoreLines", ito::ParamBase::Int | ito::ParamBase::In, 0, std::numeric_limits<int>::max(), 0, tr("Ignore the first n-lines.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("asMatrix", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("(1) Try to interprete list elements with 3 elements per row as a matrix or (0) load as written.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("separatorSign", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Uses this as the separator between elements. If NULL, try to guess.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("decimalSign", ito::ParamBase::String | ito::ParamBase::In, "<guess>", tr("Uses this as the sign for decimal numbers. If <guess>, try to guess if the decimal sign is a dot (.) or comma (,).").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String, "<guess>");
        sm.addItem(".");
        sm.addItem(",");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("wrapSign", ito::ParamBase::String | ito::ParamBase::In, "", tr("Sometimes numbers are wrapped by a sign (.e.g '2.3' or \"4.5\"). If so, indicate the character(s) that wrap the numbers.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("encoding", ito::ParamBase::String | ito::ParamBase::In, "", tr("encoding of text file, e.g. UTF-8, UTF-16, ISO 8859-1... Default: empty string -> the encoding is guessed due to a auto-detection of the first 64 bytes in the text file (using the BOM (Byte Order Mark)).").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadDataFromTxt
//----------------------------------------------------------------------------------------------------------------------------------
/** loadNistSDF method, retrieves the ascii data and creates corresponding DataObject.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function accepts parameters from itom Python application according to specification provided by "loadNistSDF" function.
*    It retrieves the ascii-data from file location passed as parameter from Hard drive and loads a corresponding Itom DataObject.
*/
ito::RetVal DataObjectIO::loadDataFromTxt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
    QFileInfo fileinfo(QString::fromLatin1(filename));
    QFile dataIn(fileinfo.canonicalFilePath());

    ito::DataObject *dObjDst = (*paramsMand)[0].getVal<ito::DataObject*>();

    if (dObjDst == NULL)
    {
        ret += ito::RetVal::format(ito::retError,0,tr("DataObject not initialized").toLatin1().data(), filename);
    }
    else if (!fileinfo.exists())
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' does not exist.").toLatin1().data(), filename);
    }
    else if (!dataIn.open(QIODevice::ReadOnly))
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' is no readable file.").toLatin1().data(), filename);
    }
    else
    {
        QChar separatorSign(0);
        QChar decimalSign(0);

        int ignoreLines = (*paramsOpt)[0].getVal<int>();

        int readFlag = 0;
        readFlag += (*paramsOpt)[1].getVal<int>();

        if ((*paramsOpt)[2].getVal<char*>() != NULL)
        {
            separatorSign = (*paramsOpt)[2].getVal<char*>()[0];
        }

        if ((*paramsOpt)[3].getVal<char*>()[0] != '<') //!= <guess>
        {
            decimalSign = (*paramsOpt)[3].getVal<char*>()[0];
        }

        QString wrapSign = paramsOpt->at(4).getVal<char*>();
        QString encoding = paramsOpt->at(5).getVal<char*>();

        if (encoding != "")
        {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
            if (QTextCodec::codecForName(encoding.toLatin1()) == NULL)
#else
            if (!QStringConverter::encodingForName(encoding.toLatin1()).has_value())
#endif
            {
                ret += ito::RetVal::format(ito::retError, 0, "encoding '%s' is unknown", encoding.toLatin1().data());
            }
        }
        else
        {
            // try to guess encoding
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
            QTextCodec* tc = QTextCodec::codecForUtfText(dataIn.read(64));
            dataIn.seek(0);
            if (tc)
            {
                encoding = tc->name();
            }
#else
            auto tc = QStringConverter::encodingForData(dataIn.read(64));
            dataIn.seek(0);
            if (tc.has_value())
            {
                encoding = QStringConverter::nameForEncoding(tc.value());
            }
#endif
            else
            {
                ret += ito::RetVal(ito::retWarning, 0, "encoding of file can not be guessed. UTF-8 is assumed.");
                encoding = "UTF-8";
            }
        }

        if (!ret.containsError())
        {
            ito::float64 zscale(0.0);

            QTextStream textStream(&dataIn);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
            textStream.setCodec(encoding.toLatin1().data());
#else
            textStream.setEncoding(QStringConverter::encodingForName(encoding.toLatin1().data()).value());
#endif
            ret += analyseTXTData(textStream, *dObjDst, separatorSign, decimalSign, readFlag, ignoreLines);
            if (!ret.containsError())
            {
                if (!dataIn.seek(0))
                {
                    dataIn.reset();
                }

                ret += readTXTDataBlock(textStream, *dObjDst, separatorSign, decimalSign, readFlag, ignoreLines, wrapSign);
            }
        }
    }

    if (dataIn.isOpen())
    {
        dataIn.close();
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::analyseTXTData(QTextStream &inFile, ito::DataObject &newObject, QChar &separator, QChar &decimalSign, const int flags, const int ignoreLines)
{
    ito::RetVal ret(ito::retOk);

    bool guessSeparator = (separator == 0);
    bool guessDecimal = (decimalSign == 0);

    int tabs = 0;
    int points = 0;
    int space = 0;
    int comma = 0;
    int sim = 0;

    int lines = 1;
    int cols = 1;

    QString strIgnoredLines("");
    QString curLine("");
    QStringList elem;
    for (int i = 0; i < ignoreLines; i++)
    {
        if (inFile.atEnd())
        {
            ret += ito::RetVal(ito::retError, 0, tr("Unexpected end of file").toLatin1().data());
            break;
        }
        strIgnoredLines.append(inFile.readLine());
    }

    if (!ret.containsError())
    {
        if (inFile.atEnd())
        {
            ret += ito::RetVal(ito::retError, 0, tr("Unexpected end of file").toLatin1().data());
        }
        curLine = inFile.readLine().trimmed();

        tabs = curLine.count('\t');
        points = curLine.count('.');
        space = curLine.count(' ');
        comma = curLine.count(',');
        sim = curLine.count(';');

        if (decimalSign != 0 && decimalSign != '.' && decimalSign != ',')
        {
            ret += ito::RetVal(ito::retError, 0, tr("The decimal sign must be '<guess>', '.' or ','.").toLatin1().data());
        }
        else if (decimalSign == separator && !guessDecimal)
        {
            ret += ito::RetVal(ito::retError, 0, tr("The decimal sign and the separator must differ.").toLatin1().data());
        }
    }

    if (!ret.containsError())
    {
        if (separator == '.' && guessDecimal)
        {
            decimalSign = ',';
            guessDecimal = false;
        }
        if (separator != ',' && !guessSeparator && guessDecimal)
        {
            if (comma == 0) decimalSign = '.';
            else decimalSign = ',';

            guessDecimal = false;
        }
        else if (separator != '.' && !guessSeparator && guessDecimal)
        {
            if (points == 0) decimalSign = '.';
            else if (separator == ',') decimalSign = '.';
            else
            {
                decimalSign = ',';
            }

            guessDecimal = false;
        }
        else if (decimalSign == ',' && guessSeparator)
        {
            if (tabs != 0) separator = '\t';
            else if (space != 0) separator = ' ';
            else if (sim != 0) separator = ';';
            else if (points != 0) separator = '.'; //dot is the last check, since it might also be a thousand-separator
            else
            {
                separator = ' ';
                ret += ito::RetVal(ito::retWarning, 0, tr("No separator could be guessed. Maybe, there is only one value per row. The separator is set to a space character.").toLatin1().data());
            }
        }
        else if (decimalSign == '.' && guessSeparator)
        {
            if (tabs != 0) separator = '\t';
            else if (space != 0) separator = ' ';
            else if (sim != 0) separator = ';';
            else if (comma != 0) separator = ','; //comma is the last check, since it might also be a thousand-separator
            else
            {
                separator = ' ';
                ret += ito::RetVal(ito::retWarning, 0, tr("No separator could be guessed. Maybe, there is only one value per row. The separator is set to a space character.").toLatin1().data());
            }
        }
        else if (guessSeparator && guessDecimal)
        {
            if (comma == 0 && points == 0)
            {
                decimalSign = '.';
                separator = (tabs > 0) ? '\t' : ' ';
            }
            else if (tabs == 0 && space == 0 && sim == 0 && comma == 0)
            {
                decimalSign = '.';
                separator = ' ';
            }
            else if (comma != 0 && points != 0)
            {
                decimalSign = '.';
                separator = ',';
            }
            else if (comma != 0)
            {
                decimalSign = ',';
                if (tabs != 0) separator = '\t';
                else if (space != 0) separator = ' ';
                else if (sim != 0) separator = ';';
                else if (points != 0) separator = '.';
                else
                {
                    decimalSign = '.';
                    separator = ',';
                }
            }
            else if (points != 0)
            {
                decimalSign = '.';
                if (tabs != 0) separator = '\t';
                else if (space != 0) separator = ' ';
                else if (sim != 0) separator = ';';
                else if (comma != 0) separator = ',';
                else separator = ' ';
            }
            guessDecimal = false;
        }
    }

    if (!ret.containsError())
    {
        if (guessSeparator)
        {
            int remove = 0;

            if (decimalSign == ',')
            {
                remove = comma;
                if (separator != '.')
                {
                    remove += points; //possible thousand group separator
                }
            }
            else
            {
                remove = points;
                if (separator != ',')
                {
                    remove += comma; //possible thousand group separator
                }
            }

            if (separator == '.')
            {
                if ((tabs + space + sim + comma - remove) > 0)
                {
                    ret += ito::RetVal(ito::retWarning, 0, tr("The separator was specified as (.) but other separator signs are also possible.").toLatin1().data());
                }
            }
            else if (separator == ',')
            {
                if ((tabs + space + sim + points - remove) > 0)
                {
                    ret += ito::RetVal(ito::retWarning, 0, tr("The separator was specified as (,) but other separator signs are also possible.").toLatin1().data());
                }
            }
            else if (separator == '\t')
            {
                if ((points + space + sim + comma - remove) > 0)
                {
                    ret += ito::RetVal(ito::retWarning, 0, tr("The separator was specified as (tab) but other separator signs are also possible.").toLatin1().data());
                }
            }
            else if (separator == ' ')
            {
                if ((tabs + comma + sim + points - remove) > 0)
                {
                    ret += ito::RetVal(ito::retWarning, 0, tr("The separator was specified as (space) but other separator signs are also possible.").toLatin1().data());
                }
            }
            else if (separator == ';')
            {
                if ((tabs + space + comma + points - remove) > 0)
                {
                    ret += ito::RetVal(ito::retWarning, 0, tr("The separator was specified as (;) but other separator signs are also possible.").toLatin1().data());
                }
            }
        }
    }

    if (!ret.containsError())
    {
        //Check if data is a list with 3 columns and n-Row
        if (flags & 0x01)
        {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
            if (curLine.split(separator, Qt::SkipEmptyParts).size() != 3)
#else
            if (curLine.split(separator, QString::SkipEmptyParts).size() != 3)
#endif
            {
                ret += ito::RetVal(ito::retError, 0, tr("The file is no list with 3 columns and N rows or contains invalid separators.").toLatin1().data());
            }
            else
            {
                cols = 3;
                while(!inFile.atEnd())
                {
                    inFile.readLine();
                    lines++;
                }
            }
        }
        else
        {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
            cols = curLine.split(separator, Qt::KeepEmptyParts).size();
#else
            cols = curLine.split(separator, QString::KeepEmptyParts).size();
#endif
            while(!inFile.atEnd())
            {
                inFile.readLine();
                lines++;
            }
        }
    }

    if (!ret.containsError())
    {
        newObject = ito::DataObject(lines, cols, ito::tFloat32);
        if (ignoreLines)
        {
            ito::DataObjectTagType ignoreTag(strIgnoredLines.toLatin1().data());
            newObject.setTag("ignoredLines", ignoreTag);
        }
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::readTXTDataBlock(QTextStream &inFile, ito::DataObject &newObject, const QChar &separator, const QChar &decimalSign, const int flags, const int ignoreLines, const QString &wrapSign)
{
    ito::RetVal ret(ito::retOk);
    ito::float32* rowPtr = NULL;
    cv::Mat* myMat = (cv::Mat*)(newObject.get_mdata()[newObject.seekMat(0)]);
    int ysize = newObject.getSize(newObject.getDims() - 2);
    int xsize = newObject.getSize(newObject.getDims() - 1);
    int xsizetmp = 0;
    bool check;
    QByteArray curline;
    QList<QByteArray> curLineData;
    curLineData.reserve(xsize);
    const char sep = separator.toLatin1();
    QByteArray wrapSign_ = wrapSign.toLatin1();
    int wrapSignLen = wrapSign_.size();

    for (int i = 0; i < ignoreLines; i++)
    {
        inFile.readLine();
    }

    QLocale local(QLocale::C); //per default a decimal sign is a dot (.) and the thousands group separator is (,)

    if (decimalSign == ',')
    {
        local = QLocale(QLocale::German); //if the decimal sign is a comma (,), the thousands group separator is assumed to be a dot (.)
    }



    for (int y = 0; y < ysize; ++y)
    {
        if (inFile.atEnd())
        {
            ret += ito::RetVal(ito::retError, 0, tr("Unexpected end of file").toLatin1().data());
            break;
        }

        curline = inFile.readLine().toLatin1();
        curLineData = curline.trimmed().split(sep);

        if (curLineData.size() > xsize)
        {
            ret += ito::RetVal::format(ito::retError, 0, tr("Line %i contains more elements than the first analyzed line").toLatin1().data(), y + ignoreLines);
            break;
        }

        rowPtr = myMat->ptr<ito::float32>(y);
        xsizetmp = std::min(xsize, (int)curLineData.size());

        for (int x = 0; x < xsizetmp; x++)
        {
            curline = curLineData[x].trimmed();
            if (wrapSign_.isEmpty())
            {
                rowPtr[x] = local.toFloat(curline, &check);
                if (!check)
                {
                    rowPtr[x] = std::numeric_limits<ito::float32>::quiet_NaN();
                }
            }
            else
            {
                if (curline.startsWith(wrapSign_) && curline.endsWith(wrapSign_))
                {
                    rowPtr[x] = local.toFloat(curline.mid(wrapSignLen, curline.size() - 2 * wrapSignLen), &check);
                    if (!check)
                    {
                        rowPtr[x] = std::numeric_limits<ito::float32>::quiet_NaN();
                    }
                }
                else
                {
                    rowPtr[x] = std::numeric_limits<ito::float32>::quiet_NaN();
                }
            }
        }


    }

    if (flags & 0x01)
    {
        // resort to funny matrix

        QVector<ito::float32> yCords;
        QVector<ito::float32> xCords;

        rowPtr = myMat->ptr<ito::float32>(0);

        ito::float32 lastY = rowPtr[1];
        yCords.append(rowPtr[1]);

        for (int y = 0; y < ysize; y++)
        {
            rowPtr = myMat->ptr<ito::float32>(y);

            if (ito::isFinite(rowPtr[1]) && ito::isNotZero(rowPtr[1] - lastY))
            {
                yCords.append(rowPtr[1]);
                lastY = rowPtr[1];
            }

            if (ito::isFinite(rowPtr[0]) && !xCords.contains(rowPtr[0]))
            {
                xCords.append(rowPtr[0]);
            }
        }
        std::sort(yCords.begin(), yCords.end());
        std::sort(xCords.begin(), xCords.end());

        int newXSize = xCords.count();
        int newYSize = yCords.count();

        ito::DataObject tmpObj;
        tmpObj.ones(newYSize, newXSize, ito::tFloat32);
        tmpObj *= std::numeric_limits<ito::float32>::quiet_NaN();

        cv::Mat* dstMat = (cv::Mat*)(tmpObj.get_mdata()[tmpObj.seekMat(0)]);

        rowPtr = myMat->ptr<ito::float32>(0);

        lastY = rowPtr[1];
        int yt = yCords.indexOf(rowPtr[1]);
        yt = std::min(yt, newYSize);
        yt = std::max(yt, 0);
        ito::float32* dstPtr = dstMat->ptr<ito::float32>(yt);

        int xt = -1;
        int ytDst = 0;

        for (int y = 0; y < ysize; y++)
        {
            rowPtr = myMat->ptr<ito::float32>(y);

            if (lastY != rowPtr[1])
            {
                lastY = rowPtr[1];
                yt = yCords.indexOf(rowPtr[1], yt);
                //ytDst = yt * newXSize;
                dstPtr = dstMat->ptr<ito::float32>(yt);
                xt = -1;
            }

            xt++;
            if (xt >= newXSize)
            {
                xt = xCords.indexOf(rowPtr[0]);
            }
            else
            {
                xt = xCords.indexOf(rowPtr[0], xt);
                if (xt < 0) xt = xCords.indexOf(rowPtr[0]);
            }

            //dstPtr[yt + xt] = rowPtr[2];
            dstPtr[xt] = rowPtr[2];
        }

        newObject.copyTagMapTo(tmpObj);
        newObject = tmpObj;

        double meanStepY = 0.0;
        for (int i = 1; i < newYSize; i++)
        {
            meanStepY += yCords[i] - yCords[i - 1];
        }

        if (newYSize < 2)
        {
            meanStepY = 1.0;
        }
        else
        {
            meanStepY /= newYSize - 1;
        }

        double meanStepX = 0;
        for (int i = 1; i < newXSize; i++)
        {
            meanStepX += xCords[i] - xCords[i - 1];
        }

        if (newXSize < 2)
        {
            meanStepX = 1.0;
        }
        else
        {
            meanStepX /= newXSize - 1;
        }

        newObject.setAxisScale(0, meanStepY);
        newObject.setAxisOffset(0, yCords[0] / meanStepY);
        newObject.setAxisScale(1, meanStepX);
        newObject.setAxisOffset(1, xCords[0] / meanStepX);
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
//! saveNistSDFParams
//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::savePtbPRDoc = QObject::tr( \
"saves a 1D data object to the PR format used for the reference software for roughness metrology (https://www.ptb.de/rptb) of PTB (Physikalisch Technische Bundesanstalt).\n\
\n\
The .pr format requires the lateral scaling values in mm. If another metric unit (m, cm, mm, _m, nm) is given in the axis unit tag, an automatic conversion is applied. Else a \n\
warning is returned. The same holds for the values (ordinate). You can choose if the .pr format should contain the ordinate values in nm or _m. An auto-conversion is implemented, too. \n\
\n\
This filter uses the hex-code DF for the german Umlaut 'oe' and F6 for 'sz' like required by the input file format description of the RPTB tool (since RPTB Version 2.01).").replace("_", QLatin1String("\u00B5"));

/** saveNistSDFParams method, specifies the parameter list for loadNistSDFParams method.
*   @param [in] paramsMand  mandatory argument parameters
*   @param [in] paramsOpt   optional argument parameters
*    @param [out] outVals   optional output parameters
*
*   This Function interacts with itom Python application, constructs plugin functionality, creates necessary parameters (eg. Mandatory and Optional parameters)
*    and their specifications as required for converting DataObject into Raw-Text data and save it into Hard drive.
*/
ito::RetVal DataObjectIO::savePtbPRParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand,paramsOpt,paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("1D data object of any real data type. No invalid values are allowed.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("decimalSigns",ito::ParamBase::Int | ito::ParamBase::In, 0, 12, 6, tr("Number of decimal signs (default: 6).").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("ordinateUnit", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("unit of ordinate (0: nm [default], 1: _m)").replace("_", QLatin1String("\u00B5")).toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePtbPR
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::savePtbPR(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
    QString filename_ = QLatin1String(filename);

    if (!filename_.endsWith(".pr", Qt::CaseInsensitive))
    {
        filename_.append(".pr");
    }

    QFileInfo fileinfo(filename_);
    QFile dataOut(filename_);

    ito::DataObject source = ito::dObjHelper::squeezeConvertCheck2DDataObject(paramsMand->at(0).getVal<ito::DataObject*>(), "sourceObject", ito::Range(1,1), ito::Range::all(), ret, ito::tFloat64, 8, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tUInt32, ito::tFloat32, ito::tFloat64);

    int decimals = (*paramsOpt)[0].getVal<int>();
    int ordinateUnit = paramsOpt->at(1).getVal<int>(); //0: nm, 1: µm
    int flags = DataObjectIO::invWrite;

    if (ret.containsError())
    {

    }
    else if (!dataOut.open(QIODevice::WriteOnly))
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' is no writeable file.").toLatin1().data(), filename);
    }

    if (!ret.containsError())
    {
        int len = source.getSize(1);
        double valueScale = 1.0;

        std::string valueUnit = source.getValueUnit();
        if (valueUnit == "mm")
        {
            if (ordinateUnit == 0) //nm
            {
                valueScale = 1.e6;
            }
            else //1, µm
            {
                valueScale = 1.e3;
            }
        }
        else if (valueUnit == "µm")
        {
            if (ordinateUnit == 0) //nm
            {
                valueScale = 1.e3;
            }
        }
        else if (valueUnit == "nm")
        {
            if (ordinateUnit == 0) //nm
            {
            }
            else //1, µm
            {
                valueScale = 1.e-3;
            }
        }
        else if (valueUnit == "cm")
        {
            if (ordinateUnit == 0) //nm
            {
                valueScale = 1.e7;
            }
            else //1, µm
            {
                valueScale = 1.e4;
            }
        }
        else if (valueUnit == "m")
        {
            if (ordinateUnit == 0) //nm
            {
                valueScale = 1.e9;
            }
            else //1, µm
            {
                valueScale = 1.e6;
            }
        }
        else
        {
            ret += ito::RetVal(ito::retWarning, 0, "The given input object does not have a metric value unit defined (m, cm, mm, µm, nm). No correct value transformation to µm or nm can be applied.");
        }

        double length = len * source.getAxisScale(1); //length is assumed to be in mm
        bool valid;
        std::string axisUnit = source.getAxisUnit(1, valid);
        if (axisUnit == "mm")
        {

        }
        else if (axisUnit == "µm")
        {
            length *= 1.e-3;
        }
        else if (axisUnit == "nm")
        {
            length *= 1.e-6;
        }
        else if (axisUnit == "cm")
        {
            length *= 10.0;
        }
        else if (axisUnit == "m")
        {
            length *= 1.e3;
        }
        else
        {
            ret += ito::RetVal(ito::retWarning, 0, "The given input object does not have a metric axis unit defined (m, cm, mm, µm, nm). No correct lateral unit value can be read. Values are assumed to be in mm.");
        }

        unsigned char key1[] = { 'X', '-', 'M', 'a', 0xDF, ' ', '=', ' ', '\0' };
        unsigned char key2[] = { ' ', 'X', '-', 'A', 'u', 'f', 'l', 0xF6, 's', 'u', 'n', 'g', ' ', '\0' };

        //write header
        dataOut.write("Profil ");
        dataOut.write(fileinfo.fileName().replace(" ", "_").toLatin1());
        dataOut.write("\n");
        dataOut.write((char*)key1);
        dataOut.write(QByteArray::number(length, 'f', decimals));
        dataOut.write((char*)key2);
        if (length != 0.0)
        {
            dataOut.write(QByteArray::number((double)len / length, 'f', decimals));
        }
        else
        {
            dataOut.write("0.0");
            ret += ito::RetVal(ito::retWarning, 0, "length of data set is 0.0");
        }
        dataOut.write(" Punkte/Zeile: ");
        dataOut.write(QByteArray::number(len));
        dataOut.write("\n");

        //write data
        const ito::float64 *row = (const ito::float64*)source.rowPtr(0, 0);
        for (int i = 0; i < len; ++i)
        {
            if (ito::isFinite(row[i]))
            {
                dataOut.write(QByteArray::number(row[i] * valueScale, 'f', decimals));
                dataOut.write("\n");
            }
            else
            {
                dataOut.write(QByteArray::number(0.0, 'f', decimals));
                dataOut.write("\n");
                ret += ito::RetVal::format(ito::retWarning, 0, "invalid value encountered in column %i", (i + 1));
            }
        }
    }

    if (dataOut.isOpen())
    {
        dataOut.close();
    }

    return ret;
}


//----------------------------------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadFrtDoc = QObject::tr(\
"Loads MicroProf FRT data from profilometers (based on FRT File Format specification 4.0.1.0). \n\
\n\
The files are loaded in a unit16, int32 or float64 data format. The FRT file format provides further \n\
information about the measurement conditions and system settings. Several of these settings are saved \n\
in tags of the resulting dataObject. These are among others (if available): \n\
\n\
* comment: optional multiline comment of the file \n\
* scanDirection: string with the scan direction for the data acquisition \n\
* measureRange: total measurement range \n\
* startTime: start time of measurement (seconds from 1970) \n\
* duration: duration of measurement in seconds \n\
* zTableType: string with the type of the used z-scanning stage \n\
* xyTableType: string with the type of the used xy-scanning stage \n\
* hardware: name of the measurement device (str) \n\
* speedX: speed of the x-stage in m/s (only given if overrideSpeed is false) \n\
* speedY: speed of the y-stage in m/s (only given if overrideSpeed is false) \n\
* sensorDelay: wait at each point so many ms (only given if overrideSpeed is true) \n\
* checkSensorError: during measurement check the error state of the sensor \n\
* sensorErrorTime: wait max. so many ms for non-error state of the sensor \n\
* scanBackMeas: during scan: measure when scanning back \n\
* title: name of dataset if given \n\
* heatingChamber: temperature of heating chamber if given \n\
* xyStitchingActive : 'true' if xy stitching was active, else 'false' \n\
* xyStitchingResolutionDivisor : only given if xyStitchingActive is 'true' \n\
\n\
Some frt files can contain more than one dataset. Multiple datasets can represent different types (topology, intensity, phases...), \n\
they can come from different sensors (e.g. upside and downside sensor) or they can be acquired at different levels. \n\
Per default, the standard dataset (e.g. topology) is loaded. Set 'printAllBufferTypes' to 1 in order to get a list of available \n\
buffers printed to the command line (only if more than one buffer is available). While buffers at different levels are automatically \n\
loaded to a 3D data object instead of a 2D one, the user can select the type and sensor counter that should be loaded. Possible values are among others: \n\
\n\
**bufferType** \n\
\n\
* 0x0001: piezo \n\
* 0x0002: intensity \n\
* 0x0004: topography \n\
* 0x0008: re_part \n\
* 0x0010: im_part \n\
* 0x0040: camera \n\
* 0x0080: thickness \n\
* 0x0100: dibfromfile \n\
* 0x0200: abs_val \n\
* 0x0400: phase \n\
* 0x0800: samplethickness \n\
* 0x1000: afm \n\
* 0x0200: quality \n\
* 0x0401: fit \n\
* 0x0402: slope \n\
\n\
**sensorCounter** \n\
\n\
Currently, only 0 (sensor 1 - top) and 1 (sensor 2 - bottom) seems to be implemented in files. In future sensor 3 and sensor 4 (indices 2 and 3) might follow.");

ito::RetVal DataObjectIO::loadFrtParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Destination dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source filename").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("xyUnit", ito::ParamBase::String | ito::ParamBase::In, "mm", tr("Unit of x and y axes. Nist or BCR sdf files assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than 'm' lead to a multiplication of all values that might exceed the data type limit.)").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String, "m");
        sm.addItem("cm");
        sm.addItem("mm");
        sm.addItem(QString("_m").replace("_", QLatin1String("\u00B5")).toLatin1().data());
        sm.addItem("nm");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("valueUnit", ito::ParamBase::String | ito::ParamBase::In, "mm", tr("Unit of value axis. x3p assumes to have m as default unit, this can be scaled using other values than m. Default: m (Be careful that other units than 'm' lead to a multiplication of all values that might exceed the data type limit.)").toLatin1().data());
        ito::StringMeta sm2(ito::StringMeta::String, "m");
        sm2.addItem("cm");
        sm2.addItem("mm");
        sm2.addItem(QString("_m").replace("_", QLatin1String("\u00B5")).toLatin1().data());
        sm2.addItem("nm");
        param.setMeta(&sm2, false);
        paramsOpt->append(param);

        param = ito::Param("bufferType", ito::ParamBase::Int | ito::ParamBase::In, 0, 0x0000ffdf, 0, tr("some files contain more than one dataset. Then pass the bufferType here (its mask is 0x0000ffdf), if 0 is given, the default buffer type is used.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("sensorCounter", ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 0, tr("some files contain more than one dataset. Its type is selected by 'bufferType'. Sometimes the result can come from different sensors, then select the sensor index here.").toLatin1().data());
        paramsOpt->append(param);

        param = ito::Param("printAllBufferTypes", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 1 and different buffer types are available, they are printed to the command line.").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

struct Buffer
{
    int alias;
    int bufferType; //something within the mask 0x0000ffdf
    int sensorCounter; //e.g. 0x10000000 or 0x00000000
    ito::int32 sizex;
    ito::int32 sizey;
    ito::int32 bpp;
    ito::int32 bytesPerBuffer;
    std::vector<std::pair<int, const char*> > pointers; //buffer counter vs. start pointer to buffer
};

/*static*/ ito::RetVal DataObjectIO::loadFrt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    const char fileID[] = { 'F', 'R', 'T', 'M', '_', 'G', 'L', 'I', 'D', 'E', 'R', 'V', '1', '.', '0', '0' }; //16 unsigned char
    ito::int16 noOfBlocks = 0; //signed short (2 byte)
    ito::uint32 fileSize = 0;

    QFileInfo info(QLatin1String(paramsMand->at(1).getVal<char*>()));
    fileSize = info.size();

    std::string xyUnit = (paramsOpt->at(0).getVal<char*>());
    std::string valueUnit = (paramsOpt->at(1).getVal<char*>());
    int desiredBufferType = paramsOpt->at(2).getVal<int>();
    int desiredSensorCounter = paramsOpt->at(3).getVal<int>();
    bool printBufferTypes = paramsOpt->at(4).getVal<int>() > 0;


    if (!info.exists())
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' does not exist", paramsMand->at(1).getVal<char*>());
    }
    else if (fileSize < sizeof(fileID) + sizeof(noOfBlocks))
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' is no valid FRT file", paramsMand->at(1).getVal<char*>());
    }
    else
    {
        QFile file(info.absoluteFilePath());

        if (!file.open(QIODevice::ReadOnly))
        {
            retval += ito::RetVal::format(ito::retError, 0, "Error opening file '%s'", paramsMand->at(1).getVal<char*>());
        }
        else
        {
            QByteArray content = file.readAll();
            const char* data = content.data();
            const char* data_part;

            ito::int16 blockType;
            ito::int32 blockSize;

            //mandatory blocks
            bool block102detected = false;
            bool block103detected = false;
            bool block108detected = false;
            bool block11detected = false;
            bool block171detected = false;

            struct
            {
                ito::float64 rangex; //in m
                ito::float64 rangey; //in m
                ito::float64 offsetx; //in m
                ito::float64 offsety; //in m
                ito::float64 factorrangey; //in m
                ito::int32 scandir;
            } scanSize;

            struct
            {
                ito::int32 measurerange;
                ito::float64 zscaling; //in m / bit
            } sensor;



            std::vector<Buffer> buffers;
            Buffer data01buffer;
            data01buffer.sizex = data01buffer.sizey = -1; //not initialized
            int selectedBufferIndex = -1; //-1 not yet selected, this means take data01buffer, else take buffers[selectedBufferIndex]

            ito::float64 zOffset = 0.0;

            const char * scanDirections[] = {
                "",
                "pos x, low y",
                "neg x, low y",
                "pos y, low x",
                "neg y, low x",
                "pos x, high y",
                "neg x, high y",
                "pos y, high x",
                "neg y, high x",
                "meander start pos x, low y",
                "meander start pos y, low x",
                "meander start pos x, high y",
                "meander start pos y, high x",
                "unknown"
            };

            const char * measureRange[] = {
                "+- 8 micrometer",
                "+- 80 micrometer",
                "+- 800 micrometer",
                "+- 800 micrometer",
                "+- 1 micrometer",
                "+- 10 micrometer",
                "+- 100 micrometer",
                "+- 200 micrometer (uncalib)",
                "unknown"
            };

            const char * hardware[] = {
                "",
                "200 micrometer Sensor",
                "5 mm Sensor",
                "CWL Sensor",
                "700 micrometer Sensor with IF",
                "ME300 ADC",
                "Eddy Current Sensor 1",
                "PCL A/D 16 Bit Card",
                "CWL with IF",
                "Camera",
                "Data from script",
                "Sios Stylus",
                "Auto Focus Sensor",
                "CWL FT",
                "Eddy Current Sensor 2",
                "Conoscopic Sensor",
                "CWL F",
                "CWL X",
                "No Sensor",
                "Konfocal Sensor",
                "CWL F FT",
                "FTR",
                "WLI",
                "PCL1741",
                "Infared Interferometric",
                "CFM",
                "AFM SIS",
                "WLI PL 2"
                "unknown"
            };

            const char * xyTableType[] = {
                "",
                "400 micrometer2",
                "5 mm2",
                "100 mm2, mech. bear. piezo",
                "100 mm2, air bearing xy",
                "100 mm2 mech. bear",
                "unknown",
                "unknown",
                "unknown",
                "unknown",
                "Turbo PMac",
                "unknown"
            }; //special is 41: 100 mm, air bearing linear

            const char * zTableType[] = {
                "",
                "40 mm Stepper",
                "40 mm DC",
                "PMAC Controlled",
                "C842 Controlled",
                "Manual Z-Table",
                "unknown",
                "unknown",
                "unknown",
                "unknown",
                "Turbo PMac",
                "unknown"
            };

            ito::int32 firstValidValue = -1;
            ito::int32 lastValidValue = -1;

            int bufferTypeMask = 0x0000ffdf;
            int sensorCounterMask = 0x30000000;
            int bufferCounterMask = 0x0f000000;

            if (memcmp(data, fileID, sizeof(fileID)) != 0)
            {
                retval += ito::RetVal::format(ito::retError, 0, "The file '%s' is no valid FRT file", paramsMand->at(1).getVal<char*>());
            }

            if (!retval.containsError())
            {
                data_part = data + sizeof(fileID);
                noOfBlocks = *((ito::int16*)data_part);
                data_part += sizeof(ito::int16);

                if (noOfBlocks <= 1)
                {
                    retval += ito::RetVal(ito::retError, 0, "invalid block size in FRT file");
                }
            }

            if (!retval.containsError())
            {
                //verify if the file is valid and looks for block 102 (Imagesize_01).
                const char* data_part_temp = data_part;
                ito::tDataType dataType;

                for (int block = 0; block < noOfBlocks; ++block)
                {
                    if (data_part + 2 + 4 > data + fileSize)
                    {
                        retval += ito::RetVal(ito::retError, 0, "file size of FRT file is too short for the pretended number of blocks.");
                        break;
                    }

                    blockType = *((ito::int16*)data_part);
                    data_part += sizeof(ito::int16);
                    blockSize = *((ito::int32*)data_part);
                    data_part += sizeof(ito::int32);

                    if (data_part + blockSize > data + fileSize)
                    {
                        retval += ito::RetVal(ito::retError, 0, "file size of FRT file is too short for the pretended block data size.");
                        break;
                    }

                    if (blockType == 11) //case 11: /*Data_01*/
                    {
                        block11detected = true;
                        data01buffer.alias = 0;
                        data01buffer.pointers.push_back(std::pair<int, const char*>(0x00000000, data_part));
                        data01buffer.bytesPerBuffer = (data01buffer.sizex * data01buffer.sizey * (data01buffer.bpp == 16 ? 2 : 4));

                        if (data01buffer.sizex >= 0) //size already initialized by block type 102, check the buffer size here now
                        {
                            if (blockSize < data01buffer.bytesPerBuffer)
                            {
                                retval += ito::RetVal(ito::retError, 0, "invalid block size of Data_01 block in FRT file.");
                            }
                        }
                    }
                    else if (blockType == 102)
                    {
                        if (blockSize == 12)
                        {
                            data01buffer.alias = 0;
                            data01buffer.sizex = *((ito::int32*)(&data_part[0]));
                            data01buffer.sizey = *((ito::int32*)(&data_part[4]));
                            data01buffer.bpp = *((ito::int32*)(&data_part[8]));
                            block102detected = true;

                            if (data01buffer.bpp != 16 && data01buffer.bpp != 32)
                            {
                                retval += ito::RetVal(ito::retError, 0, "unknown bit depth in FRT file.");
                            }

                            if (data01buffer.pointers.size() > 0) //already initialized pointers, check buffer size now
                            {
                                if ((data01buffer.sizex * data01buffer.sizey * (data01buffer.bpp == 16 ? 2 : 4)) > data01buffer.bytesPerBuffer)
                                {
                                    retval += ito::RetVal(ito::retError, 0, "invalid block size of Data_01 block in FRT file.");
                                }
                            }
                        }
                        else
                        {
                            retval += ito::RetVal(ito::retError, 0, "wrong size of Imagesize block in FRT file.");
                        }
                    }
                    else if (blockType == 125) /*Multibuffer_01*/
                    {
                        //read all buffers and put them into the buffers vector
                        const char *data_temp = &(data_part[8]);
                        int alias;
                        bool found = false;
                        std::vector<Buffer>::iterator it;

                        int numBuffers = *((ito::int32*)(&data_part[0])); //zero-based
                        int defaultAlias = *((ito::int32*)(&data_part[4])); //alias from current buffer, corresponds to buffer type 11

                        if ((defaultAlias & bufferTypeMask) == 0x0100 /*DIB -> last image, use topography instead */)
                        {
                            defaultAlias = 0x004 | (defaultAlias & (~bufferTypeMask));
                        }

                        int counter;

                        if (desiredBufferType == 0)
                        {
                            desiredBufferType = defaultAlias & bufferTypeMask;
                            desiredSensorCounter = defaultAlias & sensorCounterMask;
                        }
                        else
                        {
                            desiredSensorCounter <<= 32;
                        }

                        //search for the right buffer
                        for (int buf = 0; buf < numBuffers; ++buf)
                        {
                            alias = *((ito::int32*)(&data_temp[0]));
                            it = buffers.begin();

                            counter = -1;

                            //search for an existing buffer
                            while (it != buffers.end())
                            {
                                counter++;

                                if (it->bufferType == (alias & bufferTypeMask) && \
                                    it->sensorCounter == (alias & sensorCounterMask))
                                {
                                    break;
                                }

                                it++;
                            }

                            if (it == buffers.end())
                            {
                                counter++;
                                Buffer newbuffer;
                                //create new one
                                buffers.push_back(newbuffer);
                                it = buffers.end() - 1;
                            }


                            it->alias = alias;
                            it->sizey = *((ito::int32*)(&data_temp[8])); //lines
                            it->sizex = *((ito::int32*)(&data_temp[4])); //rows
                            it->bpp = *((ito::int32*)(&data_temp[12])); //depth
                            it->bytesPerBuffer = (it->sizex * it->sizey * (it->bpp == 16 ? 2 : 4));
                            it->bufferType = alias & bufferTypeMask;
                            it->sensorCounter = alias & sensorCounterMask;
                            it->pointers.push_back(std::pair<int, const char*>(alias & bufferCounterMask, &data_temp[16]));
                            data_temp += 16;
                            data_temp += it->bytesPerBuffer;

                            if (desiredBufferType == it->bufferType && desiredSensorCounter == it->sensorCounter)
                            {
                                selectedBufferIndex = counter;

                                if (it->bpp != 16 && it->bpp != 32)
                                {
                                    retval += ito::RetVal(ito::retError, 0, "unknown bit depth of selected buffer in FRT file.");
                                }
                            }
                        }

                        if (selectedBufferIndex == -1)
                        {
                            retval += ito::RetVal(ito::retError, 0, "the selected bufferType in combination with the sensorCounter could not be found in the FRT data file.");
                        }

                        if (printBufferTypes)
                        {
                            std::cout << "Available buffer types in the frt file '" << paramsMand->at(1).getVal<char*>() << "'\n";
                            std::cout << "------------------------------------------------------------------------------------------------------------------------------\n\n";

                            for (int i = 0; i < buffers.size(); ++i)
                            {
                                std::cout << "type: " << QByteArray::number(buffers[i].bufferType).data() << ", sensor counter: " << QByteArray::number(buffers[i].sensorCounter >> 32).data() << ", num buffers (planes): " << QByteArray::number((unsigned int)(buffers[i].pointers.size())).data();

                                if (i == selectedBufferIndex)
                                {
                                    std::cout << " (selected)\n" << std::endl;
                                }
                                else
                                {
                                    std::cout << "\n" << std::endl;
                                }
                            }
                        }
                    }

                    data_part += blockSize;
                }

                if (!block102detected)
                {
                    retval += ito::RetVal(ito::retError, 0, "mandatory block Imagesize is missing in FRT file.");
                }

                if (!retval.containsError())
                {
                    data_part = data_part_temp;
                    ito::DataObject obj;
                    int axisX = 1;
                    int axisY = 0;
                    const Buffer *usedBuffer = NULL;

                    if (selectedBufferIndex == -1) //take data 01
                    {
                        if (data01buffer.bpp == 16)
                        {
                            dataType = ito::tUInt16;
                        }
                        else if (data01buffer.bpp == 32)
                        {
                            dataType = ito::tInt32;
                        }

                        obj = ito::DataObject(data01buffer.sizey, data01buffer.sizex, dataType);
                        memcpy(obj.rowPtr(0, 0), data01buffer.pointers[0].second, data01buffer.bytesPerBuffer);
                        usedBuffer = &data01buffer;
                    }
                    else
                    {
                        usedBuffer = &(buffers[selectedBufferIndex]);
                        if (usedBuffer->bpp == 16)
                        {
                            dataType = ito::tUInt16;
                        }
                        else if (usedBuffer->bpp == 32)
                        {
                            dataType = ito::tInt32;
                        }

                        if (usedBuffer->pointers.size() == 1)
                        {
                            obj = ito::DataObject(usedBuffer->sizey, usedBuffer->sizex, dataType); //2d object
                            memcpy(obj.rowPtr(0, 0), usedBuffer->pointers[0].second, usedBuffer->bytesPerBuffer);
                        }
                        else
                        {
                            axisX = 2;
                            axisY = 1;
                            obj = ito::DataObject(usedBuffer->pointers.size(), usedBuffer->sizey, usedBuffer->sizex, dataType); //3d object
                            int currentPlane = -1;

                            for (int z = 0; z < usedBuffer->pointers.size(); ++z)
                            {
                                for (int z2 = 0; z2 < usedBuffer->pointers.size(); ++z2)
                                {
                                    //search for the lowest, still unused buffer counter and copy it to sort for the buffer counters
                                    if (usedBuffer->pointers[z2].first > currentPlane)
                                    {
                                        currentPlane = usedBuffer->pointers[z2].first;
                                        memcpy(obj.rowPtr(z, 0), usedBuffer->pointers[z2].second, usedBuffer->bytesPerBuffer);
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    bool error = false;
                    QStringList warnings;

                    //data_part always points to the start of the next block
                    for (int block = 0; block < noOfBlocks && !error; ++block)
                    {
                        blockType = *((ito::int16*)data_part);
                        data_part += sizeof(ito::int16);
                        blockSize = *((ito::int32*)data_part);
                        data_part += sizeof(ito::int32);

                        switch (blockType)
                        {
                        case 101: /*Description_01*/
                        {
                            if (blockSize > 1 && data_part[blockSize - 1] == '\0')
                            {
                                obj.setTag("comment", data_part);
                            }
                            break;
                        }
                        case 102: /*Imagesize_01*/
                            //already handled
                            break;
                        case 103: /*Scansize_01*/
                            if (blockSize == 44)
                            {
                                scanSize.rangex = *((ito::float64*)(&data_part[0]));
                                scanSize.rangey = *((ito::float64*)(&data_part[8])); //sometimes rangey is not defined in older files, factorrangey should always be preferred.
                                scanSize.offsetx = *((ito::float64*)(&data_part[16]));
                                scanSize.offsety = *((ito::float64*)(&data_part[24]));
                                scanSize.factorrangey = *((ito::float64*)(&data_part[32]));

                                //usually rangey should be rangex * factorrangey. If this is not the case,
                                //we warn the use and reset reangey.
                                if (ito::isZeroValue<ito::float64>(scanSize.rangey - scanSize.rangex * scanSize.factorrangey, std::numeric_limits<ito::float64>::epsilon()) == false)
                                {
                                    retval += ito::RetVal(ito::retWarning, 0, "rangey in block Scansize did not correspond to specs and was modified to (rangex * factorrangey)");
                                    scanSize.rangey = scanSize.rangex * scanSize.factorrangey;
                                }

                                scanSize.scandir = *((ito::int32*)(&data_part[40]));
                                block103detected = true;

                                if (usedBuffer->sizey > 1)
                                {
                                    ito::float64 scale = scanSize.rangey / usedBuffer->sizey;

                                    if (xyUnit == "m")
                                    {
                                        //nothing
                                    }
                                    else if (xyUnit == "cm")
                                    {
                                        scale *= 1e2;
                                    }
                                    else if (xyUnit == "mm")
                                    {
                                        scale *= 1e3;
                                    }
                                    else if (xyUnit == "nm")
                                    {
                                        scale *= 1e9;
                                    }
                                    else //micrometer
                                    {
                                        scale *= 1e6;
                                    }

                                    obj.setAxisScale(axisY, scale);
                                    obj.setAxisOffset(axisY, scanSize.offsety / scale);
                                    obj.setAxisUnit(axisY, xyUnit);
                                }

                                if (usedBuffer->sizex > 1)
                                {
                                    ito::float64 scale = scanSize.rangex / usedBuffer->sizex;

                                    if (xyUnit == "m")
                                    {
                                        //nothing
                                    }
                                    else if (xyUnit == "cm")
                                    {
                                        scale *= 1e2;
                                    }
                                    else if (xyUnit == "mm")
                                    {
                                        scale *= 1e3;
                                    }
                                    else if (xyUnit == "nm")
                                    {
                                        scale *= 1e9;
                                    }
                                    else //micrometer
                                    {
                                        scale *= 1e6;
                                    }

                                    obj.setAxisScale(axisX, scale);
                                    obj.setAxisOffset(axisX, scanSize.offsetx / scale);
                                    obj.setAxisUnit(axisX, xyUnit);
                                }
                                const char* temp = scanDirections[qBound(1, scanSize.scandir, 13)];
                                obj.setTag("scanDirection", temp);
                            }
                            else
                            {
                                retval += ito::RetVal(ito::retError, 0, "wrong size of Scansize block in FRT file.");
                                error = true;
                            }
                            break;
                        case 104: /* Scanspeed_01*/
                        {
                            if (blockSize == 36)
                            {
                                ito::float64 speedx = *((ito::float64*)(&data_part[0])); //m/s
                                ito::float64 speedy = *((ito::float64*)(&data_part[8])); //m/s
                                ito::int32 overridespeed = *((ito::int32*)(&data_part[16]));
                                ito::int32 checksensorerror = *((ito::int32*)(&data_part[20]));
                                ito::int32 scanbackmeas = *((ito::int32*)(&data_part[24]));
                                ito::int32 sensordelay = *((ito::int32*)(&data_part[28])); //ms
                                ito::int32 sensorerrortime = *((ito::int32*)(&data_part[32])); //ms

                                obj.setTag("sensorErrorTime", (double)sensorerrortime * 1e-3);
                                if (overridespeed)
                                {
                                    obj.setTag("sensorDelay", (double)sensordelay * 1e-3);
                                }
                                else
                                {
                                    obj.setTag("speedX", speedx);
                                    obj.setTag("speedY", speedy);
                                }

                                obj.setTag("checkSensorError", checksensorerror ? "true" : "false");
                                obj.setTag("scanBackMeas", scanbackmeas ? "true" : "false");
                            }
                            else
                            {
                                warnings << "wrong size of Scanspeed block in FRT file.";
                            }
                            break;
                        }
                        case 108: /*Sensor_01*/
                            if (blockSize == 12)
                            {
                                sensor.measurerange = *((ito::int32*)(&data_part[0]));
                                block108detected = true;
                                const char* temp = measureRange[qBound(0, sensor.measurerange, 7)];
                                obj.setTag("measureRange", temp);

                                if (!block171detected) //zscaling and offset not yet loaded from block171
                                {
                                    sensor.zscaling = *((ito::float64*)(&data_part[4]));
                                    if (valueUnit == "mm")
                                    {
                                        sensor.zscaling *= 1e3;
                                    }
                                    else if (valueUnit == "cm")
                                    {
                                        sensor.zscaling *= 1e2;
                                    }
                                    else if (valueUnit == "nm")
                                    {
                                        sensor.zscaling *= 1e9;
                                    }
                                    else if (valueUnit == "m")
                                    {
                                        //
                                    }
                                    else //micrometer
                                    {
                                        sensor.zscaling *= 1e6;
                                    }

                                    obj.setValueUnit(valueUnit);
                                }
                            }
                            else
                            {
                                retval += ito::RetVal(ito::retError, 0, "wrong size of Scansize block in FRT file.");
                                error = true;
                            }
                            break;
                        case 110: /*Hardware_01*/
                            if (blockSize == 12)
                            {
                                int sensorType = *((ito::int32*)(&data_part[0]));
                                int scanXYType = *((ito::int32*)(&data_part[4]));
                                int scanZType = *((ito::int32*)(&data_part[8]));

                                const char* temp = hardware[qBound(0, sensorType, 28)];
                                obj.setTag("hardware", temp);
                                temp = scanXYType == 41 ? "100 mm, air bearing linear" : xyTableType[qBound(0, scanXYType, 11)];
                                obj.setTag("xyTableType", temp);
                                temp = zTableType[qBound(0, scanZType, 11)];
                                obj.setTag("zTableType", temp);
                            }
                            else
                            {
                                warnings << "wrong size of Hardware block in FRT file.";
                            }
                            break;
                        case 112: /*Validval_01*/
                            if (blockSize == 8)
                            {
                                firstValidValue = *((ito::int32*)(&data_part[0]));
                                lastValidValue = *((ito::int32*)(&data_part[4]));
                            }
                            else
                            {
                                warnings << "wrong size of Validval block in FRT file.";
                            }
                            break;
                        case 113: /*zoffset_01*/
                        {
                            if (blockSize == sizeof(ito::float64))
                            {
                                if (!block171detected) //zscaling and offset not yet loaded from block171
                                {
                                    zOffset = *((ito::float64*)data_part);

                                    if (valueUnit == "mm")
                                    {
                                        zOffset *= 1e3;
                                    }
                                    else if (valueUnit == "cm")
                                    {
                                        zOffset *= 1e2;
                                    }
                                    else if (valueUnit == "nm")
                                    {
                                        zOffset *= 1e9;
                                    }
                                    else if (valueUnit == "m")
                                    {
                                        //
                                    }
                                    else //micrometer
                                    {
                                        zOffset *= 1e6;
                                    }
                                }
                            }
                            else
                            {
                                warnings << "wrong block size for 'zoffset' block. This block is ignored.";
                            }
                            break;
                        }
                        case 114: /*Time_01*/
                            if (blockSize == 12)
                            {
                                ito::int32 start = *((ito::int32*)(&data_part[0]));
                                ito::int32 duration = *((ito::int32*)(&data_part[8]));
                                obj.setTag("startTime", start);
                                obj.setTag("duration", duration);
                            }
                            else
                            {
                                warnings << "wrong size of Time block in FRT file.";
                            }
                            break;
                        case 119: /*NameOfSet_01*/
                        {
                            if (blockSize > 1 && data_part[blockSize - 1] == '\0')
                            {
                                obj.setTag("title", data_part);
                            }
                            break;
                        }
                        case 129: /*LAYER_01*/
                        {
                            ito::float64 r0 = *((ito::float64*)(&data_part[4]));
                            ito::float64 rRest = *((ito::float64*)(&data_part[8]));

                            if (valueUnit == "mm")
                            {
                                r0 *= 1e3;
                                rRest *= 1e3;
                            }
                            else if (valueUnit == "cm")
                            {
                                r0 *= 1e2;
                                rRest *= 1e2;
                            }
                            else if (valueUnit == "nm")
                            {
                                r0 *= 1e9;
                                rRest *= 1e9;
                            }
                            else if (valueUnit == "m")
                            {
                                //
                            }
                            else //micrometer
                            {
                                r0 *= 1e6;
                                rRest *= 1e6;
                            }

                            if (obj.getDims() == 3)
                            {
                                obj.setAxisScale(0, rRest); //step between two layers
                                obj.setAxisOffset(0, -(0.5 * r0) / rRest); //center point of first layer
                            }
                            break;
                        }
                        case 147: /*DEFINED_COLORS*/
                            if (blockSize == 12)
                            {
                                int colorInvalidValues = *((ito::int32*)(&data_part[0]));
                                int colorLowerValues = *((ito::int32*)(&data_part[4]));
                                int colorUpperValues = *((ito::int32*)(&data_part[8]));
                            }
                            else
                            {
                                warnings << "wrong size of DEFINED_COLORS block in FRT file.";
                            }
                            break;
                        case 171: /*MULTIBUFFER_02*/
                        {
                            if (usedBuffer->alias != 0) //the buffer comes from a multibuffer item
                            {
                                //read all buffers and put them into the buffers vector
                                const char *data_temp = &(data_part[4]);
                                int alias;
                                int absolute;
                                int numBuffers = *((ito::int32*)(&data_part[0])); //zero-based
                                for (int buf = 0; buf < numBuffers; ++buf)
                                {
                                    alias = *((ito::int32*)(&data_temp[0]));

                                    if (alias == usedBuffer->alias)
                                    {
                                        sensor.zscaling = *((ito::float64*)(&data_temp[4]));
                                        if (valueUnit == "mm")
                                        {
                                            sensor.zscaling *= 1e3;
                                        }
                                        else if (valueUnit == "cm")
                                        {
                                            sensor.zscaling *= 1e2;
                                        }
                                        else if (valueUnit == "nm")
                                        {
                                            sensor.zscaling *= 1e9;
                                        }
                                        else if (valueUnit == "m")
                                        {
                                            //
                                        }
                                        else //micrometer
                                        {
                                            sensor.zscaling *= 1e6;
                                        }

                                        zOffset = *((ito::float64*)(&data_temp[12]));

                                        if (valueUnit == "mm")
                                        {
                                            zOffset *= 1e3;
                                        }
                                        else if (valueUnit == "cm")
                                        {
                                            zOffset *= 1e2;
                                        }
                                        else if (valueUnit == "nm")
                                        {
                                            zOffset *= 1e9;
                                        }
                                        else if (valueUnit == "m")
                                        {
                                            //
                                        }
                                        else //micrometer
                                        {
                                            zOffset *= 1e6;
                                        }

                                        absolute = *((ito::float64*)(&data_temp[20]));

                                        if (absolute)
                                        {
                                            sensor.zscaling = 1.0;
                                            zOffset = 0.0;
                                            obj.setValueUnit("");
                                        }
                                        else
                                        {
                                            obj.setValueUnit(valueUnit);
                                        }

                                        block171detected = true;
                                        break;
                                    }
                                    data_temp += 24;
                                }
                            }
                            break;
                        }
                        case 178: /*XYSTITCHING*/
                            if (blockSize == 8)
                            {
                                int stitchingActive = *((ito::int32*)(&data_part[0]));
                                int resolutionDivisor = *((ito::int32*)(&data_part[4]));
                                obj.setTag("xyStitchingActive", stitchingActive ? "true": "false");
                                if (stitchingActive)
                                {
                                    obj.setTag("xyStitchingResolutionDivisor", resolutionDivisor);
                                }
                            }
                            else
                            {
                                warnings << "wrong size of XYSTITCHING block in FRT file.";
                            }
                            break;
                        case 179: /*HEATING_CHAMBER*/
                        {
                            if (blockSize == sizeof(ito::float64))
                            {
                                ito::float64 temp = *((ito::float64*)data_part);
                                if (temp > -200.0 && temp < 1000.0)
                                {
                                    obj.setTag("heatingChamber", temp);
                                }
                            }
                            else
                            {
                                warnings << "wrong block size for 'heating_chamber' block. This block is ignored.";
                            }
                            break;
                        }
                        default:
                            //warnings << QString("block type %1 unknown in FRT file. It will be ignored.").arg(blockType);
                            break;
                        }

                        data_part += blockSize;
                    }

                    if (warnings.size() > 0)
                    {
                        retval += ito::RetVal(ito::retWarning, 0, warnings.join("\n").toLatin1().data());
                    }

                    if (!block102detected || !block103detected || \
                        !block108detected || !block11detected)
                    {
                        retval += ito::RetVal(ito::retError, 0, "missing mandatory blocks in FRT file.");
                    }

                    if (!retval.containsError() && usedBuffer->alias != 0 && !block171detected)
                    {
                        //retval += ito::RetVal(ito::retWarning, 0, "Data from a multibuffer section was loaded, but no scale or offset information found in Multibuffer_02 block found. Default values are taken.");
                        //the warning was commented, since many *.frt files have block 125 but not block 171.

                        //the following types are integer-based without physical unit!
                        switch (usedBuffer->alias & bufferTypeMask)
                        {
                        case 0x00000002: /*INTENSITY*/
                        case 0x00000008: /*RE_PART*/
                        case 0x00000010: /*IM_PART*/
                        case 0x00000040: /*CAMERA*/
                        case 0x00000100: /*DIBFROMFILE*/
                        case 0x00000200: /*ABS_VAL*/
                        case 0x00000400: /*PHASE*/
                        case 0x00002000: /*QUALITY*/
                        case 0x00004001: /*FIT*/
                        case 0x00004002: /*SLOPE*/
                            obj.setValueUnit("");
                            sensor.zscaling = 1.0;
                            zOffset = 0.0;
                        }
                    }


                    if (!retval.containsError())
                    {
                        if (sensor.zscaling != 1.0 || zOffset != 0)
                        {
                            ito::DataObject objConverted;
                            retval += obj.convertTo(objConverted, ito::tFloat64, sensor.zscaling, zOffset);
                            objConverted.setValueUnit(valueUnit);

                            if (firstValidValue >= 0 && lastValidValue >= 0)
                            {
                                int total = obj.getSize(axisX) * obj.getSize(axisY);
                                ito::float64 *destPtr;

                                for (int z = 0; z < obj.getNumPlanes(); ++z)
                                {
                                    destPtr = objConverted.rowPtr<ito::float64>(z, 0);

                                    if (obj.getType() == ito::tUInt16)
                                    {
                                        const ito::uint16 *srcPtr = obj.rowPtr<ito::uint16>(z, 0);
                                        for (int i = 0; i < total; ++i)
                                        {
                                            if (srcPtr[i] < firstValidValue || srcPtr[i] > lastValidValue)
                                            {
                                                //usually scrPtr[i] == 0: not measured, srcPtr[i] == 1: measured, but invalid
                                                destPtr[i] = std::numeric_limits<ito::float64>::quiet_NaN();
                                            }
                                        }
                                    }
                                    else if (obj.getType() == ito::tInt32)
                                    {
                                        const ito::int32 *srcPtr = obj.rowPtr<ito::int32>(z, 0);
                                        for (int i = 0; i < total; ++i)
                                        {
                                            if (srcPtr[i] < firstValidValue || srcPtr[i] > lastValidValue)
                                            {
                                                destPtr[i] = std::numeric_limits<ito::float64>::quiet_NaN();
                                            }
                                        }
                                    }
                                }
                            }

                            *((*paramsMand)[0].getVal<ito::DataObject*>()) = objConverted;
                        }
                        else
                        {
                            *((*paramsMand)[0].getVal<ito::DataObject*>()) = obj;
                        }
                    }
                }
            }

            file.close();
        }

    }


    return retval;
}
