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

#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include "DataObject/dataobj.h"
#include "common/sharedStructures.h"

#include "DataObject/dataObjectFuncs.h"

#include <qimage.h>
#include <qstring.h>
#include <qobject.h>

namespace itom
{
namespace io
{


//----------------------------------------------------------------------------------------------------------------------------------
//transformDatatoImage_Mono
/*!
    This Function transforms the DataObject passed to saveDataObject function via itom application into Image of Format_Mono.
*/
template<typename _Tp> ito::RetVal transformDatatoImage_Mono(QImage *image, ito::DataObject *dObj, QString imgFilename, const _Tp tresHold)
{
    ito::RetVal ret = ito::retOk;
    int monoIndex;

    cv::Mat_<_Tp>* MAT0 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[0]);
    _Tp *linePtr0 = NULL;

    *image = QImage(MAT0->cols, MAT0->rows, QImage::Format_Mono);
    for (int row = 0; row < MAT0->rows; row++)
    {
        linePtr0 = (_Tp*)(_Tp*)MAT0->ptr(row);
        for (int col=0; col<MAT0->cols; col++)
        {
            monoIndex = linePtr0[col] > tresHold ? 1 : 0;
            image->setPixel(col, row, monoIndex);
        }
    }
    bool result = image->save(imgFilename);
    if (result == false)
    {
        ret += ito::RetVal(ito::retError, 0, QObject::tr("image could not be saved to hard drive").toLatin1().data());
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------------------
//transformDatatoImage_MonoLSB
/*!
    This Function transforms the DataObject passed to saveDataObject function via itom application into Image of Format_MonoLSB.
*/
template<typename _Tp> ito::RetVal transformDatatoImage_MonoLSB(QImage *image, ito::DataObject *dObj, QString imgFilename, const _Tp tresHold)
{
    ito::RetVal ret = ito::retOk;
    int monoIndex;

    cv::Mat_<_Tp>* MAT0 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[0]);
    _Tp *linePtr0 = NULL;

    *image = QImage(MAT0->cols, MAT0->rows, QImage::Format_MonoLSB);
    for (int row=0;row<MAT0->rows;row++)
    {
        linePtr0 = (_Tp*)MAT0->ptr(row);
        for (int col=0; col < MAT0->cols; col++)
        {
            monoIndex = linePtr0[col] > tresHold ? 1 : 0;
            image->setPixel(col, row, monoIndex);
        }
    }
    bool result = image->save(imgFilename);
    if (result == false)
    {
        ret += ito::RetVal(ito::retError, 0, QObject::tr("image could not be saved to hard drive").toLatin1().data());
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------------------
//transformDatatoImage_Indexed8
/*!
    This Function transforms the DataObject passed to saveDataObject function via itom application into Image of Format_Indexed8.
*/
template<typename _Tp> ito::RetVal transformDatatoImage_Indexed8(QImage *image, ito::DataObject *dObj, QString imgFilename, const double lowVal, const double scaling, const bool doScaling, const QVector<QRgb> &colorMap)
{
    ito::RetVal ret = ito::retOk;

    cv::Mat_<_Tp>* MAT0 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[0]);
    _Tp *linePtr0 = NULL;

    *image = QImage( MAT0->cols, MAT0->rows, QImage::Format_Indexed8);

    image->setColorTable( colorMap );
    uchar *destPtr = NULL;
    if (doScaling)
    {
        for (int row=0; row < MAT0->rows; row++)
        {
            destPtr = image->scanLine(row);
            linePtr0 = (_Tp*)MAT0->ptr(row);
            for (int col=0; col<MAT0->cols; col++)
            {
                destPtr[col] = cv::saturate_cast<ito::uint8>((linePtr0[col] - lowVal) * scaling);        
            }
        }
    }
    else
    {
        for (int row=0; row < MAT0->rows; row++)
        {
            destPtr = image->scanLine(row);
            linePtr0 = (_Tp*)MAT0->ptr(row);
            for (int col=0; col < MAT0->cols; col++)
            {
                destPtr[col] = cv::saturate_cast<ito::uint8>(linePtr0[col]);        
            }
        }
    }
    bool result = image->save(imgFilename);
    if (result == false)
    {
        ret += ito::RetVal(ito::retError, 0, QObject::tr("image could not be saved to hard drive").toLatin1().data());
    }
    return ret;
}
//----------------------------------------------------------------------------------------------------------------------------------
//transformDatatoImage_RGB32
/*!
    This Function transforms the DataObject passed to saveDataObject function via itom application into Image of Format_RGB32.
*/
template<typename _Tp> ito::RetVal transformDatatoImage_RGB32(QImage *image, ito::DataObject *dObj, QString imgFilename, const double lowVal, const double scaling, const bool doScaling, const bool multPlanes)
{    
    ito::RetVal ret = ito::retOk;

    ito::uint32 *destPtr = NULL;

    _Tp *linePtr0 = NULL;
    cv::Mat_<_Tp>* MAT0 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[0]);

    *image = QImage(MAT0->cols, MAT0->rows, QImage::Format_RGB32);

    if (multPlanes)
    {
        ito::uint32 value;
        
        _Tp *linePtr1 = NULL;
        _Tp *linePtr2 = NULL;
        
        cv::Mat_<_Tp>* MAT1 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[1]);
        cv::Mat_<_Tp>* MAT2 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[2]);

        for (int row=0; row < MAT0->rows; row++)
        {
            destPtr = (ito::uint32 *)image->scanLine(row);
            linePtr0 = (_Tp*)MAT0->ptr(row);
            linePtr1 = (_Tp*)MAT1->ptr(row);
            linePtr2 = (_Tp*)MAT2->ptr(row);

            for (int col=0; col < MAT0->cols; col++)
            {
                value  = ((ito::uint32)(linePtr0[col] > 255 ? 255 : linePtr0[col])) << 16;
                value += ((ito::uint32)(linePtr1[col] > 255 ? 255 : linePtr1[col])) <<  8;
                value += ((ito::uint32)(linePtr2[col] > 255 ? 255 : linePtr2[col])) ;

                destPtr[col] = value;        
            }
        }
    }
    else
    {
        if (doScaling)
        {
            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::uint32 *)image->scanLine(row);
                linePtr0 = (_Tp*)MAT0->ptr(row);
                for (int col=0; col<MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::uint32>((linePtr0[col] - lowVal) * scaling) & 0x00FFFFFF;        
                }
            }
        }
        else
        {
            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::uint32 *)image->scanLine(row);
                linePtr0 = (_Tp*)MAT0->ptr(row);
                for (int col=0; col<MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::uint32>(linePtr0[col]) & 0x00FFFFFF;;        
                }
            }
        }
    }
    bool result = image->save(imgFilename);
    if (result == false)
    {
        ret += ito::RetVal(ito::retError, 0, QObject::tr("image could not be saved to hard drive").toLatin1().data());
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformDatatoImage_RGB32<ito::int32>
/*!
    This Function transforms the DataObject of int32 datatype passed to saveDataObject function via itom application into Image of Format_RGB32.
*/
template<> ito::RetVal transformDatatoImage_RGB32<ito::int32>(QImage *image, ito::DataObject *dObj, QString imgFilename, const double lowVal, const double scaling, const bool doScaling, const bool multPlanes)
{
    ito::RetVal ret = ito::retOk;

    ito::int32 *linePtr0 = NULL;
    cv::Mat_<ito::int32>* MAT0 = (cv::Mat_<ito::int32>*)(dObj->get_mdata()[0]);    

    ito::uint32 *destPtr = NULL;
    *image = QImage(MAT0->cols, MAT0->rows, QImage::Format_RGB32);

    if (multPlanes)
    {
        ito::uint32 value;
        ito::int32 *linePtr1 = NULL;
        ito::int32 *linePtr2 = NULL;
        cv::Mat_<ito::int32>* MAT1 = (cv::Mat_<ito::int32>*)(dObj->get_mdata()[1]);
        cv::Mat_<ito::int32>* MAT2 = (cv::Mat_<ito::int32>*)(dObj->get_mdata()[2]);

        for (int row=0; row < MAT0->rows; row++)
        {
            destPtr = (ito::uint32 *)image->scanLine(row);
            linePtr0 = MAT0->ptr<ito::int32>(row);
            linePtr1 = MAT1->ptr<ito::int32>(row);
            linePtr2 = MAT2->ptr<ito::int32>(row);

            for (int col=0; col<MAT0->cols; col++)
            {
                value  = ((ito::uint32)(linePtr0[col] > 255 ? 255 : linePtr0[col] < 0 ? 0 : linePtr0[col])) << 16;
                value += ((ito::uint32)(linePtr1[col] > 255 ? 255 : linePtr1[col] < 0 ? 0 : linePtr1[col])) <<  8;
                value += ((ito::uint32)(linePtr2[col] > 255 ? 255 : linePtr2[col] < 0 ? 0 : linePtr2[col])) ;

                destPtr[col] = value;        
            }
        }
    }
    else
    {
        ito::int32 *destPtr = NULL;
        if (doScaling)
        {

            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::int32 *)image->scanLine(row);
                linePtr0 = MAT0->ptr<ito::int32>(row);
                for (int col=0; col<MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::int32>((linePtr0[col] - lowVal) * scaling) & 0x00FFFFFF;        
                }
            }
        }
        else
        {
            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::int32 *)image->scanLine(row);
                linePtr0 = MAT0->ptr<ito::int32>(row);
                for (int col=0; col<MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::int32>(linePtr0[col]) & 0x00FFFFFF;        
                }
            }
        }
    }

    bool result = image->save(imgFilename);
    if (result == false)
    {
        ret += ito::RetVal(ito::retError, 0, QObject::tr("image could not be saved to hard drive").toLatin1().data());
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformDatatoImage_ARGB32
/*!
    This Function transforms the DataObject passed to saveDataObject function via itom application into Image of Format_ARGB32.
*/
template<typename _Tp> ito::RetVal transformDatatoImage_ARGB32(QImage *image, ito::DataObject *dObj, QString imgFilename, const double lowVal, const double scaling, const bool doScaling, const bool multPlanes)
{
    ito::RetVal ret = ito::retOk;

    _Tp *linePtr0 = NULL;
    cv::Mat_<_Tp>* MAT0 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[0]);

    ito::uint32 *destPtr = NULL;
    *image = QImage(MAT0->cols, MAT0->rows, QImage::Format_ARGB32);

    if (multPlanes)
    {
        ito::uint32 value;
        
        _Tp *linePtr1 = NULL;
        _Tp *linePtr2 = NULL;
        _Tp *linePtr3 = NULL;

        cv::Mat_<_Tp>* MAT1 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[1]);
        cv::Mat_<_Tp>* MAT2 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[2]);
        cv::Mat_<_Tp>* MAT3 = (cv::Mat_<_Tp>*)(dObj->get_mdata()[3]);

        for (int row=0; row < MAT0->rows; row++)
        {
            destPtr = (ito::uint32 *)image->scanLine(row);
            linePtr0 = (_Tp*)MAT0->ptr(row);
            linePtr1 = (_Tp*)MAT1->ptr(row);
            linePtr2 = (_Tp*)MAT2->ptr(row);
            linePtr3 = (_Tp*)MAT3->ptr(row);

            for (int col=0; col<MAT0->cols; col++)
            {
                value  = ((ito::uint32)(linePtr0[col] > 255 ? 255 : linePtr0[col])) << 24;
                value += ((ito::uint32)(linePtr1[col] > 255 ? 255 : linePtr1[col])) << 16;
                value += ((ito::uint32)(linePtr2[col] > 255 ? 255 : linePtr2[col])) <<  8;
                value += ((ito::uint32)(linePtr3[col] > 255 ? 255 : linePtr3[col])) ;

                destPtr[col] = value;        
            }
        }
    }
    else
    {

        if (doScaling)
        {
            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::uint32 *)image->scanLine(row);
                linePtr0 = (_Tp*)MAT0->ptr(row);
                for (int col=0; col < MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::uint32>((linePtr0[col] - lowVal) * scaling);        
                }
            }
        }
        else
        {
            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::uint32 *)image->scanLine(row);
                linePtr0 = (_Tp*)MAT0->ptr(row);
                for (int col=0; col < MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::uint32>(linePtr0[col]);        
                }
            }
        }   
    }
    bool result = image->save(imgFilename);
    if (result == false)
    {
        ret += ito::RetVal(ito::retError, 0, QObject::tr("image could not be saved to hard drive").toLatin1().data());
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformDatatoImage_ARGB32<ito::int32>
/*!
    This Function transforms the DataObject of int32 datatype passed to saveDataObject function via itom application into Image of Format_RGB32.
*/
template<> ito::RetVal transformDatatoImage_ARGB32<ito::int32>(QImage *image, ito::DataObject *dObj, QString imgFilename, const double lowVal, const double scaling, const bool doScaling, const bool multPlanes)
{
    ito::RetVal ret = ito::retOk;

    ito::int32 *linePtr0 = NULL;
    cv::Mat_<ito::int32>* MAT0 = (cv::Mat_<ito::int32>*)(dObj->get_mdata()[0]);

    ito::int32 *destPtr = NULL;
    *image = QImage(MAT0->cols, MAT0->rows, QImage::Format_ARGB32);

    if (multPlanes)
    {
        ito::uint32 value = 0;
        
        ito::int32 *linePtr1 = NULL;
        ito::int32 *linePtr2 = NULL;
        ito::int32 *linePtr3 = NULL;
        
        cv::Mat_<ito::int32>* MAT1 = (cv::Mat_<ito::int32>*)(dObj->get_mdata()[1]);
        cv::Mat_<ito::int32>* MAT2 = (cv::Mat_<ito::int32>*)(dObj->get_mdata()[2]);
        cv::Mat_<ito::int32>* MAT3 = (cv::Mat_<ito::int32>*)(dObj->get_mdata()[3]);

        for (int row=0; row < MAT0->rows; row++)
        {
            destPtr = (ito::int32 *)image->scanLine(row);
            linePtr0 = MAT0->ptr<ito::int32>(row);
            linePtr1 = MAT1->ptr<ito::int32>(row);
            linePtr2 = MAT2->ptr<ito::int32>(row);
            linePtr3 = MAT3->ptr<ito::int32>(row);

            for (int col=0; col<MAT0->cols; col++)
            {
                value  = ((ito::uint32)(linePtr0[col] > 127 ? 255 : linePtr0[col] < -128 ? 0 : linePtr0[col] + 128)) << 24;
                value += ((ito::uint32)(linePtr1[col] > 127 ? 255 : linePtr1[col] < -128 ? 0 : linePtr1[col] + 128)) << 16;
                value += ((ito::uint32)(linePtr2[col] > 127 ? 255 : linePtr2[col] < -128 ? 0 : linePtr2[col] + 128)) <<  8;
                value += ((ito::uint32)(linePtr3[col] > 127 ? 255 : linePtr3[col] < -128 ? 0 : linePtr3[col] + 128));

                destPtr[col] = value;        
            }
        }
    }
    else
    {
        if (doScaling)
        {
            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::int32 *)image->scanLine(row);
                linePtr0 = MAT0->ptr<ito::int32>(row);
                for (int col=0; col<MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::int32>((linePtr0[col] - lowVal) * scaling);        
                }
            }
        }
        else
        {
            for (int row=0; row < MAT0->rows; row++)
            {
                destPtr = (ito::int32 *)image->scanLine(row);
                linePtr0 = MAT0->ptr<ito::int32>(row);
                for (int col=0; col<MAT0->cols; col++)
                {
                    destPtr[col] = cv::saturate_cast<ito::int32>(linePtr0[col]);        
                }
            }
        }      
    }

    bool result = image->save(imgFilename);
    if (result == false)
    {
        ret += ito::RetVal(ito::retError, 0, QObject::tr("image could not be saved to hard drive").toLatin1().data());
    }
    return ret;
}















//----------------------------------------------------------------------------------------------------------------------------------
//transformImagetoData_Mono
/*!
    This Function transforms the pixel value of Format_Mono type Image into respective DataObject.
*/
template<typename _Tp> ito::RetVal transformImagetoData_Mono(QImage *image, ito::DataObject *dObj, ito::DataObject *dObj_red=NULL, ito::DataObject *dObj_green=NULL, ito::DataObject *dObj_blue=NULL)
{
    ito::RetVal ret = ito::retOk;
    QImage::Format imgFormat;
    imgFormat = image->format();
    ito::uint8 *linePtr2 = NULL;
//    ito::uint8 *linePtr_red = NULL;
//    ito::uint8 *linePtr_green = NULL;
//    ito::uint8 *linePtr_blue = NULL;
    cv::Mat *MAT1 = (cv::Mat*)(dObj->get_mdata()[0]);
    uchar *orgPtr;    
    int byteIndex;
    int bitIndexInByte;
    
    for (int row=0; row<image->height(); row++)
    {
        linePtr2 =(_Tp*)MAT1->ptr(row);
        orgPtr = image->scanLine(row);
        for (int col=0; col<image->width(); col++)
            {
                if (1)
                {
                    bitIndexInByte = col % 8;                        /*!Each pixel represents one bit data in Format_Mono type Image. So this code snippet converts 1 bit data into equivalent 1 byte value to be filled as each element of DataObject matrix.*/
                    byteIndex = (col - bitIndexInByte) / 8;
                    bitIndexInByte = 7 - bitIndexInByte;
                }
                else
                {
                    bitIndexInByte = col % 8;
                    byteIndex = (col - bitIndexInByte) / 8;
                }
                linePtr2[col] = (orgPtr[byteIndex] & (1 << bitIndexInByte)) ? 255 : 0;  /*! Transfering the value of the pixel to respective element of DataObject matrix. */
                qDebug() << linePtr2[col];
            }    
    }    
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformImagetoData_Indexed8
/*!
    This Function transforms the pixel value of Format_Indexed8 type Image into respective DataObject.
*/
template<typename _Tp> ito::RetVal transformImagetoData_Indexed8(QImage *image, ito::DataObject *dObj, ito::DataObject *dObj_red=NULL, ito::DataObject *dObj_green=NULL, ito::DataObject *dObj_blue=NULL)
{
    ito::RetVal ret = ito::retOk;
    ito::uint8 *linePtr2 = NULL;
//    ito::uint8 *linePtr_red = NULL;
//    ito::uint8 *linePtr_green = NULL;
//    ito::uint8 *linePtr_blue = NULL;
    cv::Mat *MAT1 = (cv::Mat*)(dObj->get_mdata()[0]);
//    cv::Mat *MAT1R;
//    cv::Mat *MAT1G;
//    cv::Mat *MAT1B;
    QRgb rgb1;

    for (int row=0; row<image->height(); row++)
    {
        linePtr2 =(_Tp*)MAT1->ptr(row);
        for (int col=0; col<image->width(); col++)
        {
            rgb1 = image->pixel(col, row);
            linePtr2[col] = qGray(rgb1);        /*! Transfering Gray value of the pixel to respective element of DataObject matrix. */
        }            
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformImagetoData_RGB32
/*!
    This Function transforms the pixel value of Format_RGB32 type Image into respective DataObject.
*/
template<typename _Tp> ito::RetVal transformImagetoData_RGB32(QImage *image, ito::DataObject *dObj, char *colorElement=NULL)   
{
    ito::RetVal ret = ito::retOk;

    std::string colorElem;
    if (colorElement != NULL)
    {
        colorElem = colorElement;
    }
    ito::uint8 *linePtr2 = NULL;
    ito::uint8 *linePtr_red = NULL;
    ito::uint8 *linePtr_green = NULL;
    ito::uint8 *linePtr_blue = NULL;
    cv::Mat *MAT1 = (cv::Mat*)(dObj->get_mdata()[0]);
    cv::Mat *MAT1R;
    cv::Mat *MAT1G;
    cv::Mat *MAT1B;
    uchar *orgPtr;

    if (colorElem.compare("RGB") == 0)        //Case when ColorElement parameter is passed as "RGB"
    {
        MAT1R = (cv::Mat*)(dObj->get_mdata()[dObj->seekMat(0)]);    //Reference to 1st layer of dObj using seekMat() method. 
        MAT1G = (cv::Mat*)(dObj->get_mdata()[dObj->seekMat(1)]);    //Reference to 2nd layer of dObj using seekMat() method. 
        MAT1B = (cv::Mat*)(dObj->get_mdata()[dObj->seekMat(2)]);    //Reference to 3rd layer of dObj using seekMat() method. 
        for (int row=0; row<image->height(); row++)
        {    
            linePtr_red =MAT1R->ptr(row);
            linePtr_green =MAT1G->ptr(row);
            linePtr_blue =MAT1B->ptr(row);
            orgPtr = image->scanLine(row);    
            for (int col=3; col<((image->width())*4); col+=4)
            {
                linePtr_red[col/4] = orgPtr[col-1];
                linePtr_green[col/4] = orgPtr[col-2];        
                linePtr_blue[col/4] = orgPtr[col-3];
            }
        }
    }
    else            //Case when ColorElement parameter is passed as either "R" , "G" or "B".
    {
        for (int row=0; row<image->height(); row++)
        {
            linePtr2 =(_Tp*)MAT1->ptr(row);
            orgPtr = image->scanLine(row);                            
            for (int col=3; col<((image->width())*4); col+=4)
            {
                if (colorElem.compare("R") == 0) linePtr2[col/4] = orgPtr[col-1];        /*! Extracting only Red Color Element. */
                else if (colorElem.compare("G") == 0) linePtr2[col/4] = orgPtr[col-2];    /*! Extracting only Red Color Element. */
                else if (colorElem.compare("B") == 0) linePtr2[col/4] = orgPtr[col-3];    /*! Extracting only Red Color Element. */
                else linePtr2[col/4] = 0.299 * orgPtr[col - 1] + 0.587 * orgPtr[col - 2] + 0.114 * orgPtr[col - 3];        /*! Converting the RGB values into equivalent Gray value using formula " GrayElement = 0.299*RedElemet + 0.587*GreenElement + 0.114*BlueElement */
            }
        }
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformImagetoData_ARGB32
/*!
    This Function transforms the pixel value of Format_ARGB32 type Image into respective DataObject.
*/
template<typename _Tp> ito::RetVal transformImagetoData_ARGB32(QImage *image, ito::DataObject *dObj, char *colorElement=NULL)  
{
    ito::RetVal ret = ito::retOk;
    std::string colorElem;
    if (colorElement!=NULL)    colorElem= colorElement;
    ito::uint8 *linePtr2 = NULL;
    ito::uint8 *linePtr_red = NULL;
    ito::uint8 *linePtr_green = NULL;
    ito::uint8 *linePtr_blue = NULL;
    cv::Mat *MAT1 = (cv::Mat*)(dObj->get_mdata()[0]);
    cv::Mat *MAT1R;
    cv::Mat *MAT1G;
    cv::Mat *MAT1B;
    uchar *orgPtr;

    if (colorElem.compare("RGB") == 0)
    {
        MAT1R = (cv::Mat*)(dObj->get_mdata()[dObj->seekMat(0)]);
        MAT1G = (cv::Mat*)(dObj->get_mdata()[dObj->seekMat(1)]);
        MAT1B = (cv::Mat*)(dObj->get_mdata()[dObj->seekMat(2)]);
        for (int row=0; row<image->height(); row++)
        {    
            linePtr_red = MAT1R->ptr(row);
            linePtr_green = MAT1G->ptr(row);
            linePtr_blue = MAT1B->ptr(row);
            orgPtr = image->scanLine(row);    
            for (int col=3; col<((image->width())*4); col+=4)
            {
                if (orgPtr[col] != 0)        /*! Check if alfa value (transparency) of ARGB32 Image is not 0, otherwise the respective pixel value is discarded. */
                {
                    linePtr_red[col/4] = orgPtr[col-1];
                    linePtr_green[col/4] = orgPtr[col-2];        
                    linePtr_blue[col/4] = orgPtr[col-3];
                }
            }
        }
    }
    else
    {
        for (int row=0; row<image->height(); row++)
        {
            linePtr2 = (_Tp*)MAT1->ptr(row);
            orgPtr = image->scanLine(row);                            
            for (int col=3; col<((image->width())*4); col+=4)
            {
                if (orgPtr[col] != 0)    /*! Check if alfa value (transparency) of ARGB32 Image is not 0, otherwise the respective pixel value is discarded. */
                {
                    if (colorElem.compare("R") == 0) linePtr2[col/4] = orgPtr[col-1];    
                    else if (colorElem.compare("G") == 0) linePtr2[col/4] = orgPtr[col-2];
                    else if (colorElem.compare("B") == 0) linePtr2[col/4] = orgPtr[col-3];
                    else linePtr2[col/4] = 0.299 * orgPtr[col - 1] + 0.587 * orgPtr[col - 2] + 0.114 * orgPtr[col - 3];    /*! Converting the RGB values into equivalent Gray value using formula " GrayElement = 0.299*RedElemet + 0.587*GreenElement + 0.114*BlueElement */
                }
                else
                {
                    if (colorElem.compare("R") == 0) linePtr2[col/4] = orgPtr[col-1];    
                    else if (colorElem.compare("G") == 0) linePtr2[col/4] = orgPtr[col-2];
                    else if (colorElem.compare("B") == 0) linePtr2[col/4] = orgPtr[col-3];
                }
            }
        }        
    }                                
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformScaled
/*!
    
*/
template<typename _TpSrc, typename _TpDest> ito::RetVal transformScaled(cv::Mat *image, const cv::Mat *scrMat)  
{
    const _TpSrc *linePtr = NULL;  
    _TpDest *destPtr = NULL;  

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        //TODO: bitshift might get negativ - at least in compilers point of view
        //int bitShift = (sizeof(_TpSrc) - sizeof(_TpDest)) * 8;
        _TpSrc bitShift = 1 << ((sizeof(_TpSrc) - sizeof(_TpDest)) * 8);
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        for (int row = 0; row < scrMat->rows; row++)
        {
            destPtr = (_TpDest*)image->ptr<_TpDest>(row);
            linePtr = scrMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                //destPtr[col] = (_TpDest)(linePtr[col] >> bitShift) +  signCorrection;    
                destPtr[col] = (_TpDest)(linePtr[col] / bitShift) +  signCorrection;
            }
        }
    }
    else
    {
        _TpSrc scaling = (double) (1 << (sizeof(_TpDest) * 8));
        for (int row = 0; row < scrMat->rows; row++)
        {
            destPtr = image->ptr<_TpDest>(row);
            linePtr = scrMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                destPtr[col] = cv::saturate_cast<_TpDest>(linePtr[col] * scaling);        
            }
        }
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformScaledIndex8ToRGB
/*!
    This Function transforms the pixel value of Format_ARGB32 type Image into respective DataObject.
*/
template<typename _TpSrc> ito::RetVal transformScaledIndex8ToRGB(cv::Mat *image, const cv::Mat *scrMat, const QVector<QRgb> &colorMap)  
{
    
    const _TpSrc *linePtr = NULL;
    cv::Vec3b* destPtr = NULL;

    unsigned char index;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        //int bitShift = (sizeof(_TpSrc) - 1) * 8;
        _TpSrc bitShift = 1 << ((sizeof(_TpSrc) - 1) * 8);
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        unsigned char index = 0;

        for (int row = 0; row < scrMat->rows; row++)
        {
            linePtr = scrMat->ptr<const _TpSrc>(row);
            destPtr = image->ptr<cv::Vec3b>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                //index = (ito::uint8)(linePtr[col] >> bitShift) +  signCorrection;
                index = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
                destPtr[col][2] = (colorMap[index] >> 16) & 0xFF;
                destPtr[col][1] = (colorMap[index] >> 8) & 0xFF;
                destPtr[col][0] = colorMap[index] & 0xFF;
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < scrMat->rows; row++)
        {
            linePtr = scrMat->ptr<const _TpSrc>(row);
            destPtr = image->ptr<cv::Vec3b>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                index = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);    
                destPtr[col][2] = (colorMap[index] >> 16) & 0xFF;
                destPtr[col][1] = (colorMap[index] >> 8) & 0xFF;
                destPtr[col][0] = colorMap[index] & 0xFF;
            }
        }
    }
    
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformScaledIndex8ToRGB
/*!
    This Function transforms the pixel value of Format_ARGB32 type Image into respective DataObject.
*/
template<typename _TpSrc> ito::RetVal transformScaledIndex8ToRGBA(cv::Mat *image, const cv::Mat *scrMat, const QVector<QRgb> &colorMap)  
{
    
    const _TpSrc *linePtr = NULL;
    cv::Vec4b* destPtr = NULL;
    unsigned char index;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        //int bitShift = (sizeof(_TpSrc) - 1) * 8;
        _TpSrc bitShift = 1 << ((sizeof(_TpSrc) - 1) * 8);
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        unsigned char index = 0;

        for (int row = 0; row < scrMat->rows; row++)
        {
            destPtr = image->ptr<cv::Vec4b>(row);
            linePtr = scrMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                //index = (ito::uint8)(linePtr[col] >> bitShift) +  signCorrection;
                index = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
                destPtr[col][2] = (colorMap[index] >> 16) & 0xFF;
                destPtr[col][1] = (colorMap[index] >> 8) & 0xFF;
                destPtr[col][0] = colorMap[index] & 0xFF;
                destPtr[col][3] = UCHAR_MAX;
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < scrMat->rows; row++)
        {
            destPtr = image->ptr<cv::Vec4b>(row);
            linePtr = scrMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                index = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);    
                destPtr[col][2] = (colorMap[index] >> 16) & 0xFF;
                destPtr[col][1] = (colorMap[index] >> 8) & 0xFF;
                destPtr[col][0] = colorMap[index] & 0xFF;
                

                if (ito::dObjHelper::isFinite<_TpSrc>(linePtr[col]))
                {
                    destPtr[col][3] = UCHAR_MAX;
                }
                else
                {
                    destPtr[col][3] = 0;
                }
            }
        }
    }
    
    return ito::retOk;
}

} //namespace io
} //namespace itom

#endif
