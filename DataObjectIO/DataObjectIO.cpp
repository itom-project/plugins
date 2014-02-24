/* ********************************************************************
    Plugin "DataObjectIO" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#include "DataObject/dataobj.h"
#include <QtCore/QtPlugin>
#include <qvariant.h>
#include <QtGui>
#include <string.h>

#include "opencv2/highgui/highgui.hpp"

#include "common/sharedStructuresGraphics.h"
#include "common/sharedFunctionsQt.h"
#include "pluginVersion.h"

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIOInterface::getAddInInst(ito::AddInBase **addInInst)
{
    DataObjectIO* newInst = new DataObjectIO();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);
    QList<QString> keyList = newInst->m_filterList.keys();
    for (int i = 0; i < newInst->m_filterList.size(); i++)
    {
        newInst->m_filterList[keyList[i]]->m_pBasePlugin = this;
    }

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIOInterface::closeThisInst(ito::AddInBase **addInInst)
{
    if (*addInInst)
    {
        delete ((DataObjectIO *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectIOInterface::DataObjectIOInterface()
{
    m_type = ito::typeAlgo;
    setObjectName("DataObjectIO");

    m_description = QObject::tr("import or export dataObject from/to several file formats.");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This filter contains two different export- / import-functionalities for dataObjects, image or raw.\n\
\n\
Image-export functions converts dataObjects to image data and saves them as common image-formats.\n\
uint8 or uint16 are saved as gray-values (8bit or if suppored as 16bit) or if the image format allows color are saved according to the defined color palette.\n\
float32 or float64 are saved as gray-values (8bit or if suppored as 16bit) or according to the defined color palette. Therefore the values must be between 0.0 and 1.0.\n\
Values outside these borders are clipped. If the image format supports RGBA, invalid values are saved as 00 00 00 FF else as 00 00 00.\n\
\n\
Basic export-filter definition: source-obj, filename, palette, ...\n\
Basic import-filter definition: dst-obj, filename, chanel-specification, ...\n\
\n\
Raw-export functions write/read the data to/from txt-based or binary file formats.";

    m_detaildescription = QObject::tr(docstring);
    m_author            = "W. Lyda, M. Gronle, ITO, University Stuttgart";
    m_license           = QObject::tr("Licensed under LPGL.");
    m_version           = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer        = MINVERSION;
    m_maxItomVer        = MAXVERSION;
    m_aboutThis         = tr("Fill in about dialog content");         
    
 //    ito::tParam paramVal = ito::tParam("Number of Axis", ito::ParamBase::Int, 0, 10, 6, "Number of axis for this Motor");
//    m_initParamsOpt.append(paramVal);

    //m_initParamsMand.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
DataObjectIOInterface::~DataObjectIOInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(DataObjectIOInterface, DataObjectIOInterface)
#endif

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

    filter = new FilterDef(DataObjectIO::saveDataObject, DataObjectIO::saveDataObjectParams, tr("saves 1D and 2D dataObject to image formats via QImage (*.xbm *.xpm)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.xbm *.xpm)"));
    m_filterList.insert("saveDataObject", filter);

    filter = new FilterDef(DataObjectIO::loadDataObject, DataObjectIO::loadDataObjectParams, tr("loads 1D and 2D dataObject from image formats via QImage  (*.gif *.xbm *.xpm)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Images (*.gif *.xbm *.xpm)"));
    m_filterList.insert("loadDataObject", filter);

    filter = new FilterDef(DataObjectIO::loadImage, DataObjectIO::loadImageParams, tr("loads 1D and 2D dataObject from common image formats via openCV"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Images (*.pgm *.pbm *.ppm *.sr *.ras *.bmp *.dib *.png *.tif *.tiff *.jpg *.jpeg *.jp2 *.gif *.xbm *.xpm)"));
    m_filterList.insert("loadAnyImage", filter);

    filter = new FilterDef(DataObjectIO::saveNistSDF, DataObjectIO::saveNistSDFParams, tr("saves 1D and 2D dataObject to the ascii surface data file format (sdf), version aNIST-1.0"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("ASCII Surface Data File (*.sdf)"));
    m_filterList.insert("saveSDF", filter);

    filter = new FilterDef(DataObjectIO::loadNistSDF, DataObjectIO::loadNistSDFParams, tr("loads an ascii-based surface data file to a 1D or 2D data object (sdf), version aNIST-1.0"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("ASCII Surface Data File (*.sdf)"));
    m_filterList.insert("loadSDF", filter);

    filter = new FilterDef(DataObjectIO::saveTiff, DataObjectIO::saveTiffParams, tr("saveObject as tiff via openCV (tif)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.tif *.tiff)"));
    m_filterList.insert("saveTiff", filter);

    filter = new FilterDef(DataObjectIO::saveJPG, DataObjectIO::saveJPGParams, tr("saveObject as jpg via openCV (jpg)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.jpg *.jpeg *.jp2)"));
    m_filterList.insert("saveJPG", filter);

    filter = new FilterDef(DataObjectIO::savePNG, DataObjectIO::savePNGParams, tr("saveObject as portabel networl graphic via openCV (png)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.png)"));
    m_filterList.insert("savePNG", filter);

    filter = new FilterDef(DataObjectIO::saveBMP, DataObjectIO::saveBMPParams, tr("saveObject as windows bitmap via openCV (bmp, dib)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.bmp *.dib)"));
    m_filterList.insert("saveBMP", filter);

    filter = new FilterDef(DataObjectIO::saveRAS, DataObjectIO::saveRASParams, tr("saveObject as sun raster via openCV (sr, ras)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.sr *.ras)"));
    m_filterList.insert("saveRAS", filter);

    filter = new FilterDef(DataObjectIO::savePPM, DataObjectIO::savePPMParams, tr("saveObject as portabel pixel map via openCV (ppm)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.ppm)"));
    m_filterList.insert("savePPM", filter);

    filter = new FilterDef(DataObjectIO::savePGM, DataObjectIO::savePGMParams, tr("saveObject as portabel gray map via openCV (pgm, pbm)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Images (*.pgm *.pbm)"));
    m_filterList.insert("savePGM", filter);

    filter = new FilterDef(DataObjectIO::saveItomIDO, DataObjectIO::saveItomIDOParams, tr("saveObject or only its header as xml-compatible file (ido, idh)"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iWriteDataObject, tr("Raw-XML (*.ido *.idh)"));
    m_filterList.insert("saveIDO", filter);

    filter = new FilterDef(DataObjectIO::loadItomIDO, DataObjectIO::loadItomIDOParams, tr("loadOject or its header from an xml-compatible file (ido, idh) into a dataObject"), ito::AddInAlgo::catDiskIO, ito::AddInAlgo::iReadDataObject, tr("Raw-XML (*.ido *.idh)"));
    m_filterList.insert("loadIDO", filter);

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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype or 3-planes DataObject of uint8").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("Format",ito::ParamBase::String | ito::ParamBase::In,NULL, tr("Format of the Image according to QImage: ['QImage::Format_Mono', 'QImage::Format_MonoLSB', 'QImage::Format_Indexed8', 'QImage::Format_RGB32', 'QImage::Format_ARGB32'").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("bitscaling",ito::ParamBase::Int | ito::ParamBase::In,0, 2, 1, tr("Toggle bit-scaling, 0 = off, 1 = autoscale (default), 2 = userdefined ranges").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("lowLimit",ito::ParamBase::Double | ito::ParamBase::In,-1*std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0, tr("Lowest limit. Will become 0").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("highLimit",ito::ParamBase::Double | ito::ParamBase::In,-1*std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 1.0, tr("Lowest limit. Will become max<bitdepth>").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("Color_Format",ito::ParamBase::String | ito::ParamBase::In, NULL, tr("When DataObject of type float32, their format should be -->Gray<-- or -->RGB<--").toLatin1().data());
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
    char *filename = NULL;
    filename = (*paramsMand)[1].getVal<char*>();
    QImage image;

    std::string imgFormat;
    imgFormat = (*paramsMand)[2].getVal<char*>();
    int toogle = (*paramsMand)[3].getVal<int>();

    // Optional parameters (lowLimit, highLimit, colorFormat)
    double lowLimit = (*paramsOpt)[0].getVal<double>();
    double highLimit = (*paramsOpt)[1].getVal<double>();

    char *color_format = NULL;    
    std::string color_form;
    if((*paramsOpt)[2].getLen()!=0) 
    {
        color_format = (*paramsOpt)[1].getVal<char*>();
        color_form = color_format;
    }

    if(toogle == 2 && (fabs(lowLimit - highLimit) < std::numeric_limits<double>::epsilon() || lowLimit > highLimit))
    {
        ret += ito::RetVal(ito::retError, 0, tr("Save to image failed: lowLimit must be unequal to highLimit and smaller than highLimit").toLatin1().data());
    }

  /*  QVariant out1 = QVariant(2.0);    // showing any desired output on itom application.
    outVals->append(out1);*/
            
    QString imgFilename(filename);

    //Creating an Image in Mono Format
    if(imgFormat.compare("QImage::Format_Mono") == 0)
    {
        if(dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            switch(dObj->getType())
            {
                case ito::tUInt8:
                    {
                        if(toogle == 0)
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
                        if(toogle == 0)
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
                        if(toogle == 0)
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
                        if(toogle == 0)
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
                        if(toogle == 0)
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
                        if(toogle == 0)
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
                        if(toogle == 0)
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
    else if(imgFormat.compare("QImage::Format_MonoLSB")==0)
    {
        if(dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            if(toogle == 0)
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
                    if(toogle == 0)
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
                    if(toogle == 0)
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
                    if(toogle == 0)
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
                    if(toogle == 0)
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
                    if(toogle == 0)
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
                    if(toogle == 0)
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
                    if(toogle == 0)
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
    else if(imgFormat.compare("QImage::Format_Indexed8")==0)
    {    
        if(dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            double scaling = 1.0;
            bool doScaling = true;
            if(toogle == 0)
            {
                doScaling = false;
                highLimit = 255.0;
                lowLimit = 0.0;
            }
            else if(toogle == 2)
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
    else if(imgFormat.compare("QImage::Format_RGB32")==0)
    {        
        if(dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            double scaling = 1.0;
            bool doScaling = true;
            if(toogle == 0)
            {
                doScaling = false;
                highLimit = std::numeric_limits<ito::int32>::max();
                lowLimit = 0.0;
            }
            else if(toogle == 2)
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
        else if((dObj->getType() == ito::tUInt8) && (dObj->getDims() > 2 && dObj->calcNumMats() == 3))   // RGB-Planes
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
    else if(imgFormat.compare("QImage::Format_ARGB32")==0)
    {
        if(dObj->getDims() == 2 || (dObj->getDims() > 2 && dObj->calcNumMats() == 1))   // 2D or 1x1x1x...x2D
        {
            double scaling = 1.0;
            bool doScaling = true;
            if(toogle == 0)
            {
                doScaling = false;
                highLimit = std::numeric_limits<ito::int32>::max();
                lowLimit = 0.0;
            }
            else if(toogle == 2)
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
        else if((dObj->getType() == ito::tInt8) && (dObj->getDims() > 2 && dObj->calcNumMats() == 4))   // RGBA-Planes
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
        else if((dObj->getType() == ito::tInt8) && (dObj->getDims() > 2 && dObj->calcNumMats() == 3))   // RGB-Planes
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
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("DestinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObjet").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);

        ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::String, "R");
        m->addItem("G");
        m->addItem("B");
        m->addItem("RGB");
        param = ito::Param("ColorElement", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Color element character: R or G or B or RGB").toLatin1().data());
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
    QFileInfo fileinfo(filename);

    if(!fileinfo.exists())
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' does not exist.").toLatin1().data(), filename);
    }    
    else if( !image.load(filename) )
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' is no readable image file.").toLatin1().data(), filename);
    }
    else
    {
        ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
        char *colorElement;
        std::string colorElem;
        if ((*paramsOpt)[0].getLen()!= 0) 
        {
            colorElement = (*paramsOpt)[0].getVal<char*>();
            colorElem = colorElement;
        }
        if (colorElem.compare("RGB") == 0) 
        {
            *dObjDst= ito::DataObject(3, image.height() ,image.width(), ito::tUInt8);
        }
        else 
        {
            *dObjDst = ito::DataObject(image.height(), image.width(), ito::tUInt8);
        }
        QImage::Format imgFormat;
        imgFormat = image.format();
    
        if (imgFormat == QImage::Format_Mono)    //Case for Format_Mono and Format_MonoLSB images. Note: All Format_MonoLSB images are detected as Format_Mono images so No special case for Format_MonoLSB. 
        {                                        
            itom::io::transformImagetoData_Mono<ito::uint8>(&image, dObjDst);
        }
        else if (imgFormat == QImage::Format_Indexed8)   //Case for Format_Indexed8 images
        {
            itom::io::transformImagetoData_Indexed8<ito::uint8>(&image, dObjDst);
        }
        else if (imgFormat == QImage::Format_RGB32)     //Case for Format_RGB32 images
        {
            if (colorElem.compare("R") == 0) itom::io::transformImagetoData_RGB32<ito::uint8>(&image, dObjDst, colorElement);
            else if (colorElem.compare("G") == 0) itom::io::transformImagetoData_RGB32<ito::uint8>(&image, dObjDst, colorElement);
            else if (colorElem.compare("B") == 0) itom::io::transformImagetoData_RGB32<ito::uint8>(&image, dObjDst, colorElement);
            else if (colorElem.compare("RGB") == 0) itom::io::transformImagetoData_RGB32<ito::uint8>(&image, dObjDst, colorElement);
            else itom::io::transformImagetoData_RGB32<ito::uint8>(&image, dObjDst);
        }
        else if (imgFormat == QImage::Format_ARGB32)     //Case for Format_ARGB32 images
        {    
            if (colorElem.compare("R") == 0) itom::io::transformImagetoData_ARGB32<ito::uint8>(&image, dObjDst, colorElement);
            else if (colorElem.compare("G") == 0) itom::io::transformImagetoData_ARGB32<ito::uint8>(&image, dObjDst, colorElement);
            else if (colorElem.compare("B") == 0) itom::io::transformImagetoData_ARGB32<ito::uint8>(&image, dObjDst, colorElement);
            else if (colorElem.compare("RGB") == 0) itom::io::transformImagetoData_ARGB32<ito::uint8>(&image, dObjDst, colorElement);
            else itom::io::transformImagetoData_ARGB32<ito::uint8>(&image, dObjDst);
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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype or 3-planes DataObject of uint8").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("decimalSigns",ito::ParamBase::Int | ito::ParamBase::In, 0, 12, 3, tr("Number of decimal signs (default: 3).").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("verticalScale",ito::ParamBase::Int | ito::ParamBase::In, -12, 12, -3, tr("Power of 10 for the zValue (Default: -3, micrometer), only for floating point objects.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("invalidHandling",ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("Toggles NaN handling if dataObject is floating-type. 0: Write NaN (Default); 1: Substitut by InvalidValue.").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("invalidValue",ito::ParamBase::Double | ito::ParamBase::In, 0.0, -1 * std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), tr("New value for invalid substitution. Default is 0.0").toLatin1().data());
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
*/
ito::RetVal DataObjectIO::saveNistSDF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
    QFileInfo fileinfo(filename);
    QFile dataOut(filename);

    ito::DataObject *dObjSrc = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();

    ret += ito::dObjHelper::verify2DDataObject(dObjSrc, "dObjectIn", 1, std::numeric_limits<int>::max(), 1, std::numeric_limits<int>::max(), 8, ito::tInt8, ito::tUInt8, ito::tInt16, ito::tUInt16, ito::tInt32, ito::tUInt32, ito::tFloat32, ito::tFloat64);

    double verticalScale = pow(10.0, (*paramsOpt)[1].getVal<int>());
    int decimals = (*paramsOpt)[0].getVal<int>();

    if(ret.containsWarningOrError())
    {
    
    }
    else if( !dataOut.open(QIODevice::WriteOnly) )
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' is no writeable file.").toLatin1().data(), filename);
    }

    if(!ret.containsWarningOrError())
    {   
        QByteArray outLine(100, 0);
        ito::ByteArray tag;
        std::string unitStr;
        double value = 0.0;
        bool dummyBool;
        bool isLine = (dObjSrc->getSize(0) == 1 || dObjSrc->getSize(1) == 1) ? true : false;
        bool changeXY = dObjSrc->getSize(1) == 1 ? true : false;

        dataOut.write("aNIST-1.0\n");
        
        outLine = ("ManufacID\t\t= ");
        tag = dObjSrc->getTag("ManufacID", dummyBool).getVal_ToString();
        if(tag.empty())
        {
            tag = fileinfo.fileName().toLatin1().data();
        }
        outLine.append(tag.data());
        outLine.append('\n');
        dataOut.write(outLine);

        outLine = ("CreateDate\t\t= ");
        tag = dObjSrc->getTag("CreateDate", dummyBool).getVal_ToString();
        if(tag.empty())
        {
            tag = "000000000000";
        }
        outLine.append(tag.data());
        outLine.append('\n');
        dataOut.write(outLine);

        outLine = ("ModDate\t\t= ");
        tag = dObjSrc->getTag("ModDate", dummyBool).getVal_ToString();
        if(tag.empty())
        {
            tag = "000000000000";
        }
        outLine.append(tag.data());
        outLine.append('\n');
        dataOut.write(outLine);

        double xScale = dObjSrc->getAxisScale(1);
        unitStr = dObjSrc->getAxisUnit(1, dummyBool);
        if(unitStr == "mm")
        {
            xScale *= 1/1000.0;
        }
        else if(unitStr == "µm")
        {
            xScale *= 1/1000000.0;
        }
        else if(unitStr == "km")
        {
            xScale *= 1000.0;
        }

        double yScale = dObjSrc->getAxisScale(0);
        unitStr = dObjSrc->getAxisUnit(0, dummyBool);
        if(unitStr == "mm")
        {
            yScale *= 1/1000.0;
        }
        else if(unitStr == "µm")
        {
            yScale *= 1/1000000.0;
        }
        else if(unitStr == "km")
        {
            yScale *= 1000.0;
        }

        if(changeXY)
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

        if(dObjSrc->getType() != ito::tFloat32 &&  dObjSrc->getType() != ito::tFloat64)
        {
            verticalScale = 1.0;
        }
        else
        {
            zScale *= verticalScale;
        }

        unitStr = dObjSrc->getValueUnit();
        if(unitStr == "mm")
        {
            zScale *= 1/1000.0;
        }
        else if(unitStr == "µm")
        {
            zScale *= 1/1000000.0;
        }
        else if(unitStr == "km")
        {
            zScale *= 1000;
        }

        outLine = ("Zscale\t\t= ");
        outLine.append(QByteArray::number(zScale));
        outLine.append('\n');
        dataOut.write(outLine);

        outLine = ("Zresolution\t\t= 1\n");
        dataOut.write(outLine);
        
        outLine = ("Compression\t\t= 0\n");
        dataOut.write(outLine);

        switch(dObjSrc->getType())
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
        outLine.append('\n');
        dataOut.write(outLine);

        outLine = ("CheckType\t\t= 0\n");
        dataOut.write(outLine);

        outLine = "*\n";
        dataOut.write(outLine);
        outLine = "\n";
        dataOut.write(outLine);

        switch(dObjSrc->getType())
        {
            case ito::tUInt8:
                writeDataBlock<ito::uint8>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
            case ito::tUInt16:
                writeDataBlock<ito::uint16>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
            case ito::tUInt32:
                writeDataBlock<ito::uint32>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
            case ito::tInt8:
                writeDataBlock<ito::int8>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
            case ito::tInt16:
                writeDataBlock<ito::int16>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
            case ito::tInt32:
                writeDataBlock<ito::int32>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
            case ito::tFloat32:
                writeDataBlock<ito::float32>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
            case ito::tFloat64:
                writeDataBlock<ito::float64>(dataOut, dObjSrc, 1/verticalScale, decimals, 0, ' ');
                break;
        }

        outLine = "*\n";
        dataOut.write(outLine);

    }

    if(dataOut.isOpen())
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
template<typename _Tp> ito::RetVal DataObjectIO::writeDataBlock(QFile &outFile, const ito::DataObject *scrObject, const double zScale, const int decimals, const int flags, const char seperator)
{
    ito::RetVal ret(ito::retOk);

    int x = 0, y = 0;

    int xsize = scrObject->getSize(1);
    int ysize = scrObject->getSize(0);

    const cv::Mat* dstMat = (const cv::Mat*)(scrObject->get_mdata()[0]);
    const _Tp *p_Dst = NULL;
    QByteArray curLine;
    curLine.reserve(30000);
    if(xsize == 1)
    {
        if(std::numeric_limits<_Tp>::is_exact)
        {
            for(y = 0; y < ysize; y ++)
            {
                p_Dst = dstMat->ptr<_Tp>(y);
                curLine = QByteArray::number(p_Dst[0]);
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
        else
        {
            for(y = 0; y < ysize; y ++)
            {
                p_Dst = dstMat->ptr<_Tp>(y);
                curLine = QByteArray::number(p_Dst[0]*zScale, 'f', decimals);
                curLine.append('\n');
                outFile.write(curLine);
            }        
        }
    }
    else if(ysize == 1)
    {
        p_Dst = dstMat->ptr<_Tp>(0);
        if(std::numeric_limits<_Tp>::is_exact)
        {
            for(x = 0; x < xsize; x ++)
            {
                curLine = QByteArray::number(p_Dst[x]);
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
        else
        {
            for(x = 0; x < xsize; x ++)
            {
                curLine = QByteArray::number(p_Dst[x]*zScale, 'f', decimals);
                curLine.append('\n');
                outFile.write(curLine);
            }        
        }    
    }
    else
    {
        
        if(std::numeric_limits<_Tp>::is_exact)
        {
            for(y = 0; y < ysize; y ++)
            {
                p_Dst = dstMat->ptr<_Tp>(y);
                curLine.clear();
                for(x = 0; x < xsize - 1; x ++)
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
            for(y = 0; y < ysize; y ++)
            {
                p_Dst = dstMat->ptr<_Tp>(y);
                curLine.clear();
                for(x = 0; x < xsize - 1; x ++)
                {
                    curLine.append(QByteArray::number(p_Dst[x]*zScale, 'f', decimals));
                    curLine.append(seperator);
                }
                curLine.append(QByteArray::number(p_Dst[xsize - 1]*zScale, 'f', decimals));
                curLine.append('\n');
                outFile.write(curLine);
            }
        }
    }

    return ret;
}
//----------------------------------------------------------------------------------------------------------------------------------
//! loadNistSDFParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("DestinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObjet").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);

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
*/
ito::RetVal DataObjectIO::loadNistSDF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
    QFileInfo fileinfo(filename);
    QFile dataIn(fileinfo.canonicalFilePath());

    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();

    if(dObjDst == NULL)
    {
        ret += ito::RetVal::format(ito::retError,0,tr("Dataobject not initialized").toLatin1().data(), filename);
    }
    else if(!fileinfo.exists())
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' does not exist.").toLatin1().data(), filename);
    }    
    else if( !dataIn.open(QIODevice::ReadOnly) )
    {
        ret += ito::RetVal::format(ito::retError,0,tr("The file '%s' is no readable file.").toLatin1().data(), filename);
    }
    else
    {     
        ito::float64 zscale(0.0);
        ret += readNistHeader(dataIn, *dObjDst, zscale, 0);
        if(!ret.containsError())
        {
            switch(dObjDst->getType())
            {
            case ito::tInt8:
                ret += readDataBlock<ito::int8>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            case ito::tUInt8:
                ret += readDataBlock<ito::uint8>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            case ito::tInt16:
                ret += readDataBlock<ito::int16>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            case ito::tUInt16:
                ret += readDataBlock<ito::uint16>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            case ito::tInt32:
                ret += readDataBlock<ito::int32>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            case ito::tUInt32:
                ret += readDataBlock<ito::uint32>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            case ito::tFloat32:
                ret += readDataBlock<ito::float32>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            case ito::tFloat64:
                ret += readDataBlock<ito::float64>(dataIn, *dObjDst, zscale, 0, ' ');
                break;
            default:
                return ito::RetVal(ito::retError, 0, "DataType not supported");
                break;
            }
            
        }
    }

    if(dataIn.isOpen())
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
*/
ito::RetVal DataObjectIO::readNistHeader(QFile &inFile, ito::DataObject &newObject, double &zscale,const int /*flags*/)
{
    ito::RetVal ret(ito::retOk);
    ito::float64 xscale = 1.0, yscale = 1.0, zRes = 1.0;
    int xsize = 0, ysize = 0;
    int dataType = -1;
    QByteArray curLine;
    std::map<std::string, std::string> metaData;
    curLine = inFile.readLine();
    
    zscale = 1.0;

    //First line Version Number (a: ASCII, b: binary), Unsigned Char, 8-bytes
    if(!curLine.contains("aNIST-1.0"))
    {
        return ito::RetVal::format(ito::retError,0,tr("The file '%s' did not contain 'aNIST-1.0'-header.").toLatin1().data(), inFile.fileName().toLatin1().data());
    }
    curLine = inFile.readLine();
    while(!curLine.contains("*") && !inFile.atEnd())
    {

        if(curLine.contains("ManufacID")) //Manufacturer ID, Unsigned Char, 10-bytes
        {
            metaData["ManufacID"] = (std::string)curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified();
        }
        else if(curLine.contains("CreateDate")) //Create Date and Time, Unsigned Char, 12-bytes
        {
            metaData["CreateDate"] = (std::string)curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified();
        }
        else if(curLine.contains("ModDate")) //Modified Date and Time, Unsigned Char, 12-bytes
        {
            metaData["ModDate"] = (std::string)curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified();
        }
        else if(curLine.contains("NumPoints")) //Number of points in a profile, Unsigned Int, 2-bytes
        {
            xsize = curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified().toInt();
        }
        else if(curLine.contains("NumProfiles")) //Number of profiles in a data file, Unsigned Int, 2-bytes
        {
            ysize = curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified().toInt();
        }
        else if(curLine.contains("Xscale"))     //X-scale. A x-scale value of 1.00 E-6 represents a sample spacing of 1 micrometer, Double, 8-bytes
        {
            xscale = curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified().toDouble();
        }
        else if(curLine.contains("Yscale"))     //Y-scale. A y-scale value of 1.00 E-6 represents a sample spacing of 1 micrometer, Double, 8-bytes
        {
            yscale = curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified().toDouble();
        }  
        else if(curLine.contains("Zscale"))     //Z-scale. A z-scale value of 1.00 E-6 represents a height of 1 micrometer, Double, 8-bytes
        {
            zscale = curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified().toDouble();
        }  
        else if(curLine.contains("Zresolution"))     //Z- resolution, Double, 8-bytes
        {
            zRes = curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified().toDouble();
        }   
        else if(curLine.contains("Compression"))     //Compression Type, Unsigned Char, 1-bytes
        {
            // documentation missing
        } 
        else if(curLine.contains("DataType"))     //Data Type (0: unsigned char, 1: unsigned integer, 2: unsigned long, 3: float, 4: signed char, 5: signed integer, 6 signed long, 7: double), Unsigned Char, 1-bytes
        {
            dataType = curLine.mid(curLine.lastIndexOf("=")+1, -1).simplified().toInt();
            switch(dataType)
            {
                case 0:
                    dataType = ito::tUInt8;
                    break;
                case 1:
                    dataType = ito::tUInt16;
                    break;
                case 2:
                    dataType = ito::tUInt32;
                    break;
                case 3:
                    dataType = ito::tFloat32;
                    break;
                case 4:
                    dataType = ito::tInt8;
                    break;
                case 5:
                    dataType = ito::tInt16;
                    break;
                case 6:
                    dataType = ito::tInt32;
                    break;
                case 7:
                    dataType = ito::tFloat64;
                    break;
                default:
                    ret = ito::RetVal(ito::retError, 0, "DataType not supported!");
                    dataType = -1;
                    break;
            }
        }
        else if(curLine.contains("CheckType"))      //Check Sum Type, Unsigned Char, 1-bytes
        {
            // documentation missing
        }

        curLine = inFile.readLine();
    }

    if(dataType > -1 && xsize > 0 && ysize > 0)
    {
        if(xsize == 1)
        {
            xsize = ysize;
            ysize = 1;
        }

        newObject = ito::DataObject(ysize, xsize, dataType);
        newObject.setAxisScale(0, yscale * 1000);
        newObject.setAxisUnit(0, "mm");
        newObject.setAxisScale(1, xscale * 1000);
        newObject.setAxisUnit(1, "mm");
        std::map<std::string, std::string>::iterator it = metaData.begin();
        for(unsigned int i = 0; i < metaData.size(); i++)
        {
            newObject.setTag((*it).first, (*it).second);
            ++it;
        }

        zscale *= zRes;
        zscale *= 1000;
        newObject.setValueUnit("mm");
    }
    else
    {
        ret = ito::RetVal(ito::retError, 0, "DataObject was not created!");
    }


    return ret;
};
//----------------------------------------------------------------------------------------------------------------------------------
//! readDataBlock
//----------------------------------------------------------------------------------------------------------------------------------
/** readDataBlock method, read the data-block of a .sdf-nist metrology file and write data to the corresponding dataObject.
*   @param [in]     inFile      Readonly File
*   @param [in|out] newObject   The dataObject to be created
*    @param [in]     zscale      The zscale, will be multiplied with the extracted data
*   @param [int]    flags       For later improvements, not used yet
*/
template<typename _Tp> ito::RetVal DataObjectIO::readDataBlock(QFile &inFile, ito::DataObject &newObject, const double zScale, const int /*flags*/, const char seperator)
{
    ito::RetVal ret(ito::retOk);

    int x = 0, y = 0;

    int xsize = newObject.getSize(1);
    int ysize = newObject.getSize(0);

    

    cv::Mat* dstMat = (cv::Mat*)(newObject.get_mdata()[0]);
    _Tp *p_Dst = NULL;

    if(ysize == 1)
    {
        QByteArray curLine = inFile.readLine().simplified();
        p_Dst = dstMat->ptr<_Tp>(0);
        while(!curLine.contains("*") && !inFile.atEnd() && x < xsize)
        {
            if(curLine.length() > 1)
            {       
                if(std::numeric_limits<_Tp>::is_exact)
                {
                    p_Dst[x] = cv::saturate_cast<_Tp>(curLine.toDouble() * zScale);

                }
                else
                {
                    p_Dst[x] = (_Tp)(curLine.toDouble() * zScale);               
                }
                x++;
            }

            curLine = inFile.readLine().simplified();
        }
    }
    else
    {
        QByteArray curLine = inFile.readLine().simplified();
        QList<QByteArray> dest;
        dest.reserve(xsize);
        while(!curLine.contains("*") && !inFile.atEnd() && y < ysize)
        {
            if(curLine.length() > 1)
            {
                dest = curLine.split(seperator);
                if(dest.size() != xsize)
                {
                    ret = ito::RetVal(ito::retError, 0, "Read-Dataline failed!");
                }
                else
                {
                    p_Dst = dstMat->ptr<_Tp>(y);
                
                    if(std::numeric_limits<_Tp>::is_exact)
                    {
                        for(x = 0; x < xsize; x++)
                        {
                            p_Dst[x] = cv::saturate_cast<_Tp>(dest[x].toDouble() * zScale);
                        } 
                    }
                    else
                    {
                        for(x = 0; x < xsize; x++)
                        {
                            p_Dst[x] = (_Tp)(dest[x].toDouble() * zScale);
                        }                   
                    }
                    y++;
                }
            
            }

            curLine = inFile.readLine().simplified();
        }
    }

    return ret;
}


//----------------------------------------------------------------------------------------------------------------------------------
//! saveData2Txt
//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
//! loadDataFromTxt
//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
//! saveTifParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype").toLatin1().data());
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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "falseColor" , tr("Color palette name [gray, gray16, ...]. 'gray16' supported for '.jp2' only").toLatin1().data());
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
    ito::RetVal retVal(ito::retOk);
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::jpgFormat);
}
//----------------------------------------------------------------------------------------------------------------------------------
//! savePNGParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "falseColor", tr("Color palette name [gray, gray16, ...].").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("compression",ito::ParamBase::Int | ito::ParamBase::In, 0, 9, 9, tr("Compression rate, 0: high, 9: low").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("addAlphaChannel",ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("If enabled and in case of floating point values, invalid will have alpha low else high").toLatin1().data());
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
    ito::RetVal retVal(ito::retOk);
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::pngFormat);
}
//----------------------------------------------------------------------------------------------------------------------------------
//! saveBMPParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
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
    ito::RetVal retVal(ito::retOk);
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::bmpFormat);
}
//----------------------------------------------------------------------------------------------------------------------------------
//! savePPMParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("palette",ito::ParamBase::String | ito::ParamBase::In, "falseColor", tr("Color palette name [gray, ...]").toLatin1().data());
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
    ito::RetVal retVal(ito::retOk);
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::ppmFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! savePGMParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceImage", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("2D-DataObject of anytype").toLatin1().data());
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
    ito::RetVal retVal(ito::retOk);
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::pgmFormat);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveRASParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
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
    ito::RetVal retVal(ito::retOk);
    return saveDataObjectOpenCV(paramsMand, paramsOpt, paramsOut, DataObjectIO::sunFormat);
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
ito::RetVal DataObjectIO::saveDataObjectOpenCV(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/, const unsigned char imageFormat)
{
    ito::RetVal ret = ito::retOk;
    
    // Optional parameters (sourceImage, filename, Format, bitscaling)
    ito::DataObject *dObj = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();
    char *filename = NULL;
    filename = (*paramsMand)[1].getVal<char*>();

    char *palette = NULL;
    palette = (*paramsMand)[2].getVal<char*>();

    if(filename == NULL)
    {
        return ito::RetVal(ito::retError, 0, tr("File name empty.").toLatin1().data());
    }
    if(palette == NULL)
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

    //ito::dObjHelper::verify2DDataObject(dObj, "sourceImage", 1, std::numeric_limits<ito::int16>::max(), 1, std::numeric_limits<ito::int16>::max(), 8, ito::tUInt8, ito::tInt8, ito::tUInt16, ito::tInt16, ito::tUInt8, ito::tInt8, ito::tUInt32, ito::tInt32, ito::tUInt8, ito::tInt8, ito::tFloat32, ito::tFloat64);
    
    if(ret.containsError())
    {
        return ret;
    }

    cv::Mat saveMat;
    cv::Mat *scrData = NULL;

    QString imgPalette(palette);
    QFileInfo fileName(filename);

    if(fileName.fileName().isEmpty())
    {
        return ito::RetVal(ito::retError, 0, tr("Filename not valid.").toLatin1().data());
    }

    if(fileName.exists() && !fileName.isWritable())
    {
        return ito::RetVal(ito::retError, 0, tr("File is not writeable.").toLatin1().data());
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
            if(fileName.suffix().compare("tif", Qt::CaseInsensitive) != 0 && fileName.suffix().compare("tiff", Qt::CaseInsensitive) != 0)
            {
                QString fixedName(fileName.absolutePath());
                fixedName.append("/");
                fixedName.append(fileName.completeBaseName());
                fixedName.append(".tif");
                fileName = fixedName;
            }

            break;
        case DataObjectIO::ppmFormat:
            save_params.push_back(CV_IMWRITE_PXM_BINARY);
            save_params.push_back((*paramsOpt)[0].getVal<int>());

            if(fileName.suffix().compare("ppm", Qt::CaseInsensitive) != 0)
            {
                QString fixedName(fileName.absolutePath());
                fixedName.append("/");
                fixedName.append(fileName.completeBaseName());                
                fileName = fixedName;                
            }

            gray16Supported = false;
            colorSupported = true;
            break;
        case DataObjectIO::pgmFormat:
            
            save_params.push_back(CV_IMWRITE_PXM_BINARY);
            save_params.push_back((*paramsOpt)[0].getVal<int>());

            gray16Supported = true;
            colorSupported = false;

            if(fileName.suffix().compare("pgm", Qt::CaseInsensitive) != 0 && fileName.suffix().compare("pbm", Qt::CaseInsensitive) != 0)
            {
                QString fixedName(fileName.absolutePath());
                fixedName.append("/");
                fixedName.append(fileName.completeBaseName());

                fixedName.append(".pgm");
                
                fileName = fixedName;
                
            }
            else if(fileName.suffix().compare("pbm", Qt::CaseInsensitive) == 0)
            {
                gray16Supported = false;
            }

            break;

        case DataObjectIO::jpgFormat:
        case DataObjectIO::jp2000Format:

            if(fileName.suffix().compare("jpg", Qt::CaseInsensitive) != 0 && fileName.suffix().compare("jpeg", Qt::CaseInsensitive) != 0 && fileName.suffix().compare("jp2", Qt::CaseInsensitive) != 0)
            {
                QString fixedName(fileName.absolutePath());
                fixedName.append("/");
                fixedName.append(fileName.completeBaseName());
                fixedName.append(".jpg");
                fileName = fixedName;
                gray16Supported = false;
            }

            if(fileName.suffix().compare("jp2", Qt::CaseInsensitive) == 0)
            {
                gray16Supported = true;
            }

            colorSupported = true;
            save_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            save_params.push_back((*paramsOpt)[0].getVal<int>());
            
            break;
        case DataObjectIO::bmpFormat:
            if(fileName.suffix().compare("bmp", Qt::CaseInsensitive) != 0 && fileName.suffix().compare("dib", Qt::CaseInsensitive) != 0)
            {
                QString fixedName(fileName.absolutePath());
                fixedName.append("/");
                fixedName.append(fileName.completeBaseName());

                fixedName.append(".bmp");
                
                fileName = fixedName;
                
            }
            gray16Supported = false;
            colorSupported = true;

            break;
        case DataObjectIO::pngFormat:

            if(fileName.suffix().compare("png", Qt::CaseInsensitive) != 0)
            {
                QString fixedName(fileName.absolutePath());
                fixedName.append("/");
                fixedName.append(fileName.completeBaseName());

                fixedName.append(".png");
                
                fileName = fixedName;
                
            }

            save_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            save_params.push_back((*paramsOpt)[0].getVal<int>());
            addAlpha = (*paramsOpt)[1].getVal<int>() != 0 ? true : false;
            gray16Supported = true;
            colorSupported = true;
            break;

        case DataObjectIO::sunFormat:
            if(fileName.suffix().compare("sr", Qt::CaseInsensitive) != 0 && fileName.suffix().compare("ras", Qt::CaseInsensitive) != 0)
            {
                QString fixedName(fileName.absolutePath());
                fixedName.append("/");
                fixedName.append(fileName.completeBaseName());

                fixedName.append(".ras");
                
                fileName = fixedName;
                
            }
            gray16Supported = false;
            colorSupported = true;
            break;
        default:
            return ito::RetVal(ito::retError, 0, tr("Image Format not valid.").toLatin1().data());
    }

    //Creating an Image in Mono Format
    if(imgPalette.compare("gray") == 0)
    {
        scrData = (cv::Mat *)(dObj->get_mdata()[dObj->seekMat(0)]);
        saveMat.create(scrData->rows, scrData->cols, CV_8U);

        switch(dObj->getType())
        {
            case ito::tUInt8:
                scrData->copyTo(saveMat);
                break;
            case ito::tInt8:
                ret += itom::io::transformScaled<ito::int8 , ito::uint8>(saveMat, scrData);
                break;
            case ito::tUInt16:
                ret += itom::io::transformScaled<ito::uint16, ito::uint8>(saveMat, scrData);
                break;
            case ito::tInt16:
                ret += itom::io::transformScaled<ito::int16, ito::uint8>(saveMat, scrData);
                break;
            case ito::tUInt32:
                ret += itom::io::transformScaled<ito::uint32, ito::uint8>(saveMat, scrData);
                break;
            case ito::tInt32:
                ret += itom::io::transformScaled< ito::int32, ito::uint8>(saveMat, scrData);
                break;
            case ito::tFloat32:
                ret += itom::io::transformScaled<ito::float32, ito::uint8>(saveMat, scrData);
                break;
            case ito::tFloat64:
                ret += itom::io::transformScaled<ito::float64, ito::uint8>(saveMat, scrData);
                break;
            default:
                return ito::RetVal(ito::retError, 0, tr("DataObject-Type could not be converted to unsigned int 8-bit.").toLatin1().data());
        }
    }
    else if(imgPalette.compare("gray16") == 0 )
    {
        if(gray16Supported)
        {
            scrData = (cv::Mat *)(dObj->get_mdata()[dObj->seekMat(0)]);
            saveMat.create(scrData->rows, scrData->cols, CV_16U);

            switch(dObj->getType())
            {
                case ito::tUInt16:
                    scrData->copyTo(saveMat);
                    break;
                case ito::tInt16:
                    ret += itom::io::transformScaled<ito::int16, ito::uint16>(saveMat, scrData);
                    break;
                case ito::tUInt32:
                    ret += itom::io::transformScaled<ito::uint32, ito::uint16>(saveMat, scrData);
                    break;
                case ito::tInt32:
                    ret += itom::io::transformScaled<ito::int32, ito::uint16>(saveMat, scrData);
                    break;
                case ito::tFloat32:
                    ret += itom::io::transformScaled<ito::float32, ito::uint16>(saveMat, scrData);
                    break;
                case ito::tFloat64:
                    ret += itom::io::transformScaled<ito::float64, ito::uint16>(saveMat, scrData);
                    break;
                default:
                    return ito::RetVal(ito::retError, 0, tr("DataObject-Type could not be converted to unsigned int 16-bit.").toLatin1().data());
            }
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support 16-Bit Gray value.").toLatin1().data());
        }
    }
    else if(imgPalette.compare("RGBA") == 0)
    {
        if(colorSupported)
        {
            return ito::RetVal(ito::retError, 0, tr("Saving RGB-Image currently not supported.").toLatin1().data());
        }
        else
        {
            return ito::RetVal(ito::retError, 0, tr("Image format does not support color values.").toLatin1().data());
        }
        
    }
    else if(!imgPalette.isEmpty())
    {
        ito::ItomPalette newPalette;
        ret += apiPaletteGetColorBarName(imgPalette, newPalette);

        if (ret.containsError())
        {
            return ret;
        }

        scrData = (cv::Mat *)(dObj->get_mdata()[dObj->seekMat(0)]);

        if(colorSupported)
        {
            if(addAlpha)
            {
                saveMat.create(scrData->rows, scrData->cols, CV_8UC4);
                switch(dObj->getType())
                {
                    case ito::tUInt8:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint8>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tInt8:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int8>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tUInt16:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint16>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tInt16:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int16>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tUInt32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::uint32>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tInt32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::int32>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tFloat32:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::float32>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tFloat64:
                        ret += itom::io::transformScaledIndex8ToRGBA<ito::float64>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    default:
                        return ito::RetVal(ito::retError, 0, tr("DataObject-Type could not be converted to index8 scaled images.").toLatin1().data());
                }
            }
            else
            {
                saveMat.create(scrData->rows, scrData->cols, CV_8UC3);
                switch(dObj->getType())
                {
                    case ito::tUInt8:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint8>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tInt8:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int8>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tUInt16:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint16>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tInt16:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int16>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tUInt32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::uint32>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tInt32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::int32>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tFloat32:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::float32>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    case ito::tFloat64:
                        ret += itom::io::transformScaledIndex8ToRGB<ito::float64>(saveMat, scrData, newPalette.colorVector256);
                        break;
                    default:
                        return ito::RetVal(ito::retError, 0, tr("DataObject-Type could not be converted to index8 scaled images.").toLatin1().data());
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

    if(!ret.containsError())
    {
        try 
        {
            bool test = cv::imwrite(fileName.absoluteFilePath().toLatin1().data(), saveMat, save_params);
            if(test == false)
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
//! loadImageParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("DestinationImage", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObjet").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);

        ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::String, "asIs");
        m->addItem("alpha");
        m->addItem("R");
        m->addItem("G");
        m->addItem("B");
        m->addItem("RGB");
        m->addItem("ARGB");
        m->addItem("GRAY");
        param = ito::Param("ColorElement", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Color element character: asIs (default) | alpha | R | G | B | RGB | ARGB | GRAY").toLatin1().data());
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
ito::RetVal DataObjectIO::loadImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> * /*paramsOut*/)
{
    ito::RetVal ret = ito::retOk;
    char *filename = (*paramsMand)[1].getVal<char*>();
    char *colorElement = NULL;
    colorElement = (*paramsOpt)[0].getVal<char*>();

    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();

    if(dObjDst == NULL)
    {
        return ito::RetVal(ito::retError,0,tr("Destination dataObject is invalid.").toLatin1().data());
    }

    ito::DataObject tempObject;

    int flags = 0;
    QString colorFormat(colorElement);

    bool reduceChanel = false;

    if(colorFormat.isEmpty() || colorFormat.compare("asIs", Qt::CaseInsensitive) == 0)
    {
        flags = CV_LOAD_IMAGE_ANYDEPTH;
        flags *= -1;
        reduceChanel = false;
    }
    else if(colorFormat.compare("alpha", Qt::CaseInsensitive) == 0)
    {
        flags = CV_LOAD_IMAGE_COLOR;
        flags *= -1;
        reduceChanel = true;
    }
    else if(colorFormat.compare("R", Qt::CaseInsensitive) == 0 || colorFormat.compare("G", Qt::CaseInsensitive) == 0 || colorFormat.compare("B", Qt::CaseInsensitive) == 0)
    {
        flags = CV_LOAD_IMAGE_COLOR;
        flags *= -1;
        reduceChanel = true;
    }
    else if(colorFormat.compare("gray", Qt::CaseInsensitive) == 0 || colorFormat.compare("grey", Qt::CaseInsensitive) == 0)
    {
        flags = CV_LOAD_IMAGE_GRAYSCALE;

        reduceChanel = false;
    }
    else
    {
        reduceChanel = false;
        flags = CV_LOAD_IMAGE_COLOR;
        flags *= -1;        
    }

    cv::Mat image;
    QFileInfo fileinfo(filename);

    if(!fileinfo.exists())
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

    if(!ret.containsError())
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
            case CV_8UC4:
                if(reduceChanel) imageType = ito::tUInt8;
                else imageType = ito::tRGBA32;
                break;
            case CV_16UC3:
            case CV_16UC4:
                if(reduceChanel) imageType = ito::tUInt16;
                else return ito::RetVal(ito::retError,0,tr("Color import is currently not managed. Wait for next itom version (approx end of 2014).").toLatin1().data());
                break;
            default:
                return ito::RetVal(ito::retError,0,tr("Color format of the file is currently not compatible with itom. Wait for next itom version.").toLatin1().data());
        }

        tempObject = ito::DataObject(image.rows, image.cols, imageType);

        if(!reduceChanel)
        {

            if(image.type() == CV_8UC3)
            {
                ito::Rgba32* dst = NULL;
                cv::Vec3b* scr = NULL;

                for(int y = 0; y < image.rows; y++)
                {
                    dst = ((cv::Mat*)(tempObject.get_mdata()[0]))->ptr<ito::Rgba32>(y);
                    scr = image.ptr<cv::Vec3b>(y);
                    for(int x = 0; x < image.cols; x++)
                    {
                        dst[x].alpha() = 255;
                        dst[x].red()   = scr[x][2];
                        dst[x].green() = scr[x][1];
                        dst[x].blue()  = scr[x][0];
                    }
                }
            }
            else if(image.type() == CV_16UC3)
            {
                ret += ito::RetVal(ito::retError,0,tr("Color import is currently not managed. Wait for next itom version (approx end of 2014).").toLatin1().data());
            }
            else if(image.type() == CV_8UC4)
            {
                ito::Rgba32* dst = NULL;
                cv::Vec4b* scr = NULL;

                for(int y = 0; y < image.rows; y++)
                {
                    dst = ((cv::Mat*)(tempObject.get_mdata()[0]))->ptr<ito::Rgba32>(y);
                    scr = image.ptr<cv::Vec4b>(y);
                    for(int x = 0; x < image.cols; x++)
                    {
                        dst[x].alpha() = scr[x][0];
                        dst[x].red()   = scr[x][3];
                        dst[x].green() = scr[x][2];
                        dst[x].blue()  = scr[x][1];
                    }
                }
            }
            else if(image.type() == CV_16UC4)
            {
                ret += ito::RetVal(ito::retError,0,tr("Color import is currently not managed. Wait for next itom version (approx end of 2014).").toLatin1().data());
            }
            else
            {
                image.copyTo(*((cv::Mat*)(tempObject.get_mdata()[0])));
            }
            
        }
        else
        {
            int colIndex;
            if(colorFormat.compare("R", Qt::CaseInsensitive) == 0)
            {
                colIndex = 3;
            }
            else if(colorFormat.compare("G", Qt::CaseInsensitive) == 0)
            {
                colIndex = 2;
            }
            else if(colorFormat.compare("B", Qt::CaseInsensitive) == 0)
            {
                colIndex = 1;
            }
            else if(colorFormat.compare("alpha", Qt::CaseInsensitive) == 0)
            {
                colIndex = 0;
            }
            else
            {
                colIndex = -1;
                return ito::RetVal(ito::retError,0,tr("Color format of the file is currently not compatible with itom. Wait for next itom version.").toLatin1().data());
            }

            if(image.type() == CV_8UC3)
            {
                colIndex--;

                ito::uint8* dst = NULL;
                cv::Vec3b* scr = NULL;

                for(int y = 0; y < image.rows; y++)
                {
                    dst = ((cv::Mat*)(tempObject.get_mdata()[0]))->ptr<ito::uint8>(y);
                    scr = image.ptr<cv::Vec3b>(y);
                    for(int x = 0; x < image.cols; x++)
                    {
                        dst[x] = scr[x][colIndex];
                    }
                }
            }
            else if(image.type() == CV_8UC4)
            {
                colIndex--;

                ito::uint8* dst = NULL;
                cv::Vec4b* scr = NULL;

                for(int y = 0; y < image.rows; y++)
                {
                    dst = ((cv::Mat*)(tempObject.get_mdata()[0]))->ptr<ito::uint8>(y);
                    scr = image.ptr<cv::Vec4b>(y);
                    for(int x = 0; x < image.cols; x++)
                    {
                        dst[x] = scr[x][colIndex];
                    }
                }
            }
            else if(image.type() == CV_16UC3)
            {
                colIndex--;

                ito::uint16* dst = NULL;
                cv::Vec3w* scr = NULL;

                for(int y = 0; y < image.rows; y++)
                {
                    dst = ((cv::Mat*)(tempObject.get_mdata()[0]))->ptr<ito::uint16>(y);
                    scr = image.ptr<cv::Vec3w>(y);
                    for(int x = 0; x < image.cols; x++)
                    {
                        dst[x] = scr[x][colIndex];
                    }
                }
            }
            else if(image.type() == CV_16UC4)
            {
                colIndex--;

                ito::uint16* dst = NULL;
                cv::Vec4w* scr = NULL;

                for(int y = 0; y < image.rows; y++)
                {
                    dst = ((cv::Mat*)(tempObject.get_mdata()[0]))->ptr<ito::uint16>(y);
                    scr = image.ptr<cv::Vec4w>(y);
                    for(int x = 0; x < image.cols; x++)
                    {
                        dst[x] = scr[x][colIndex];
                    }
                }
            }
            else
            {
                return ito::RetVal(ito::retError,0,tr("Color channel not supported for extraction!").toLatin1().data());
            }
        }
    }

    if(!ret.containsError())
    {
        *dObjDst = tempObject;
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

    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();

    if(dObjDst == NULL)
    {
        return ito::RetVal(ito::retError,0,tr("Destination dataObject is invalid.").toLatin1().data());
    }

    ito::DataObject tempObject;
    QString fileNameQt(filename);
    ret += ito::loadXML2DOBJ(&tempObject, fileNameQt, false);

    if(!ret.containsError())
    {
        *dObjDst = tempObject;
    }

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! loadItomIDOParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObjet").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source file name").toLatin1().data());
        paramsMand->append(param);
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

    ito::DataObject *dObjDst = (ito::DataObject*)(*paramsMand)[0].getVal<void*>();

    if(dObjDst == NULL)
    {
        return ito::RetVal(ito::retError,0,tr("Destination dataObject is invalid.").toLatin1().data());
    }

    ito::DataObject tempObject;
    QString fileNameQt(filename);
    ret += ito::saveDOBJ2XML(dObjDst, fileNameQt, onlyHeader, asBinary);

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! saveItomIDOParams
//----------------------------------------------------------------------------------------------------------------------------------
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
    if(!retval.containsError())
    {
        ito::Param  param = ito::Param("sourceObject", ito::ParamBase::DObjPtr | ito::ParamBase::In, NULL, tr("Any type of dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Destination filename").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("headerOnly", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 0 the complete dataObject is saved (data + metadata), else the filter saves only metadata").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("tags2Binary", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("If 0 the metaData is saved as clear text, else double tags are saved as binary").toLatin1().data());
        paramsOpt->append(param);
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
