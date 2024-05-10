/* ********************************************************************
    Plugin "DataObjectIO" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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
#include "common/numeric.h"

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
template<typename _Tp> ito::RetVal transformDatatoImage_Mono(QImage *image, ito::DataObject *dObj, QString imgFilename, const _Tp threshold)
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
            monoIndex = linePtr0[col] > threshold ? 1 : 0;
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
template<typename _Tp> ito::RetVal transformDatatoImage_MonoLSB(QImage *image, ito::DataObject *dObj, QString imgFilename, const _Tp threshold)
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
            monoIndex = linePtr0[col] > threshold ? 1 : 0;
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
//QImage_Mono_to_dataObject
/*!
    This Function transforms the pixel value of Format_Mono type Image into respective DataObject.
*/
template<typename _Tp> ito::RetVal QImage_Mono_to_dataObject(const QImage *image, ito::DataObject *dObj)
{
    bool lsb = image->format() == QImage::Format_MonoLSB ? true : false;
    _Tp *linePtr2 = NULL;
    cv::Mat *MAT1 = dObj->getCvPlaneMat(0);
    const uchar *orgPtr;
    int byteIndex;
    int bitIndexInByte;

    QVector<QRgb> colorTable = image->colorTable();
    QVector<ito::uint8> grayTable;
    grayTable.reserve(colorTable.size());
    foreach(const QRgb &c, colorTable)
    {
        grayTable.append(qGray(c));
    }

    if (grayTable.size() < 2)
    {
        grayTable.clear();
        grayTable << 0 << 255;
    }

    for (int row = 0; row < image->height(); row++)
    {
        linePtr2 =(_Tp*)MAT1->ptr(row);
        orgPtr = image->scanLine(row);

        for (int col=0; col<image->width(); col++)
        {
            if (!lsb)
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
            linePtr2[col] = (grayTable[(orgPtr[byteIndex] & (1 << bitIndexInByte)) ? 1 : 0] > 0) ? std::numeric_limits<_Tp>::max() : 0;  /*! Transferring the value of the pixel to respective element of DataObject matrix. */
        }
    }
    return ito::retOk;
}

template<> ito::RetVal QImage_Mono_to_dataObject<ito::Rgba32>(const QImage *image, ito::DataObject *dObj)
{
    bool lsb = image->format() == QImage::Format_MonoLSB ? true : false;
    ito::Rgba32 *linePtr2 = NULL;
    cv::Mat *MAT1 = dObj->getCvPlaneMat(0);
    const uchar *orgPtr;
    int byteIndex;
    int bitIndexInByte;

    QVector<QRgb> colorTable = image->colorTable();
    QVector<ito::uint8> grayTable;
    grayTable.reserve(colorTable.size());
    foreach(const QRgb &c, colorTable)
    {
        grayTable.append(qGray(c));
    }

    if (grayTable.size() < 2)
    {
        grayTable.clear();
        grayTable << 0 << 255;
    }

    for (int row = 0; row < image->height(); row++)
    {
        linePtr2 =(ito::Rgba32*)MAT1->ptr(row);
        orgPtr = image->scanLine(row);

        for (int col = 0; col < image->width(); col++)
        {
            if (!lsb)
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
            linePtr2[col] = (grayTable[((orgPtr[byteIndex] & (1 << bitIndexInByte)) > 0) ? 1 : 0] > 0) ? ito::Rgba32(255) : ito::Rgba32(0);  /*! Transferring the value of the pixel to respective element of DataObject matrix. */
        }
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//QImage_Indexed8_to_dataObject
/*!
    This Function transforms the pixel value of Format_Indexed8 type Image into respective DataObject.
*/
template<typename _Tp> ito::RetVal QImage_Indexed8_to_dataObject(const QImage *image, ito::DataObject *dObj)
{
    ito::uint8 *linePtr2 = NULL;
    cv::Mat *MAT1 = dObj->getCvPlaneMat(0);
    const uchar* srcLine = NULL;
    _Tp* dstLine = NULL;
    QVector<QRgb> colorTable = image->colorTable();

    for (int row = 0; row < image->height(); row++)
    {
        srcLine = image->scanLine(row);
        dstLine = (_Tp*)MAT1->ptr(row);

        for (int col = 0; col < image->width(); col++)
        {
            dstLine[col] = cv::saturate_cast<_Tp>(qGray(colorTable[srcLine[col]]));
        }
    }

    return ito::retOk;
}

template<> ito::RetVal QImage_Indexed8_to_dataObject<ito::Rgba32>(const QImage *image, ito::DataObject *dObj)
{
    ito::uint8 *linePtr2 = NULL;
    cv::Mat *MAT1 = dObj->getCvPlaneMat(0);
    const uchar* srcLine = NULL;
    ito::Rgba32* dstLine = NULL;
    QVector<QRgb> colorTable = image->colorTable();
    QRgb c;

    for (int row = 0; row < image->height(); row++)
    {
        srcLine = image->scanLine(row);
        dstLine = (ito::Rgba32*)MAT1->ptr(row);

        for (int col = 0; col < image->width(); col++)
        {
            c = colorTable[srcLine[col]];
            dstLine[col] = ito::Rgba32(qAlpha(c), qRed(c), qGreen(c), qBlue(c));
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformImagetoData_RGB32
/*!
    This Function transforms the pixel value of Format_RGB32 type Image into respective DataObject.
*/
template<typename _Tp> ito::RetVal QImage_ARGB32_to_dataObject(QImage *image, ito::DataObject *dObj, const char *colorElement)
{
    ito::uint8 *linePtr2 = NULL;
    cv::Mat *MAT1 = dObj->getCvPlaneMat(0);
    const QRgb* srcLine = NULL;
    _Tp* dstLine = NULL;

    if (strcmp(colorElement, "R") == 0)
    {
        for (int row = 0; row < image->height(); row++)
        {
            srcLine = (const QRgb*)image->scanLine(row);
            dstLine = (_Tp*)MAT1->ptr(row);

            for (int col = 0; col < image->width(); col++)
            {
                dstLine[col] = cv::saturate_cast<_Tp>(qRed(srcLine[col]));
            }
        }
    }
    else if (strcmp(colorElement, "G") == 0)
    {
        for (int row = 0; row < image->height(); row++)
        {
            srcLine = (const QRgb*)image->scanLine(row);
            dstLine = (_Tp*)MAT1->ptr(row);

            for (int col = 0; col < image->width(); col++)
            {
                dstLine[col] = cv::saturate_cast<_Tp>(qGreen(srcLine[col]));
            }
        }
    }
    else if (strcmp(colorElement, "B") == 0)
    {
        for (int row = 0; row < image->height(); row++)
        {
            srcLine = (const QRgb*)image->scanLine(row);
            dstLine = (_Tp*)MAT1->ptr(row);

            for (int col = 0; col < image->width(); col++)
            {
                dstLine[col] = cv::saturate_cast<_Tp>(qBlue(srcLine[col]));
            }
        }
    }
    else if (strcmp(colorElement, "GRAY") == 0)
    {
        for (int row = 0; row < image->height(); row++)
        {
            srcLine = (const QRgb*)image->scanLine(row);
            dstLine = (_Tp*)MAT1->ptr(row);

            for (int col = 0; col < image->width(); col++)
            {
                dstLine[col] = cv::saturate_cast<_Tp>(qGray(srcLine[col]));
            }
        }
    }
    else
    {
        return ito::RetVal(ito::retError, 0, "invalid color channel");
    }

    return ito::retOk;
}

template<> ito::RetVal QImage_ARGB32_to_dataObject<ito::Rgba32>(QImage *image, ito::DataObject *dObj, const char *colorElement)
{
    ito::uint8 *linePtr2 = NULL;
    cv::Mat *MAT1 = dObj->getCvPlaneMat(0);
    const QRgb* srcLine = NULL;
    ito::Rgba32* dstLine = NULL;

    if (strcmp(colorElement, "RGB") == 0)
    {
        for (int row = 0; row < image->height(); row++)
        {
            srcLine = (const QRgb*)image->scanLine(row);
            dstLine = (ito::Rgba32*)MAT1->ptr(row);

            for (int col = 0; col < image->width(); col++)
            {
                dstLine[col] = ito::Rgba32(255, qRed(srcLine[col]), qGreen(srcLine[col]), qBlue(srcLine[col]));
            }
        }
    }
    else if (strcmp(colorElement, "RGBA") == 0)
    {
        for (int row = 0; row < image->height(); row++)
        {
            srcLine = (const QRgb*)image->scanLine(row);
            dstLine = (ito::Rgba32*)MAT1->ptr(row);

            for (int col = 0; col < image->width(); col++)
            {
                dstLine[col] = ito::Rgba32(qAlpha(srcLine[col]), qRed(srcLine[col]), qGreen(srcLine[col]), qBlue(srcLine[col]));
            }
        }
    }
    else
    {
        return ito::RetVal(ito::retError, 0, "invalid color channel");
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformScaled
/*!

*/
template<typename _TpSrc> ito::RetVal transformScaledToUInt8(cv::Mat &dstMat, const cv::Mat *scrMat)
{
    dstMat = cv::Mat(scrMat->rows, scrMat->cols, CV_8U);
    const _TpSrc *linePtr = NULL;
    ito::uint8 *destPtr = NULL;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        _TpSrc bitShift = 1 << qBound<ito::uint8>(0, ((sizeof(_TpSrc) - 1 /*sizeof(ito::uint8)*/) * 8), 32); //qBound in order to avoid compiler warning for negative or too big shift values
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        for (int row = 0; row < scrMat->rows; row++)
        {
            destPtr = (ito::uint8*)dstMat.ptr<ito::uint8>(row);
            linePtr = scrMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                destPtr[col] = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < scrMat->rows; row++)
        {
            destPtr = dstMat.ptr<ito::uint8>(row);
            linePtr = scrMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                destPtr[col] = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);
            }
        }
    }
    return ito::retOk;
}

template<typename _TpSrc> ito::RetVal transformScaledToUInt8(QImage &dstImg, const cv::Mat *srcMat)
{
    dstImg = QImage(srcMat->cols, srcMat->rows, QImage::Format_Indexed8);
    QVector<QRgb> colors(256);
    for (int i = 0; i < 256; ++i)
    {
        colors[i] = qRgb(i,i,i);
    }

    dstImg.setColorTable(colors);

    const _TpSrc *linePtr = NULL;
    ito::uint8 *destPtr = NULL;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        _TpSrc bitShift = 1 << qBound<ito::uint8>(0, ((sizeof(_TpSrc) - 1 /*sizeof(ito::uint8)*/) * 8), 32); //qBound in order to avoid compiler warning for negative or too big shift values
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = (ito::uint8*)dstImg.scanLine(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                destPtr[col] = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = (ito::uint8*)dstImg.scanLine(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                destPtr[col] = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);
            }
        }
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformScaled
/*!

*/
template<typename _TpSrc> ito::RetVal transformScaledToUInt16(cv::Mat &dstMat, const cv::Mat *scrMat)
{
    dstMat = cv::Mat(scrMat->rows, scrMat->cols, CV_16U);
    const _TpSrc *linePtr = NULL;
    ito::uint16 *destPtr = NULL;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        if (sizeof(_TpSrc) >= sizeof(ito::uint16))
        {
            _TpSrc bitShift = 1 << qBound<ito::uint8>(0, ((sizeof(_TpSrc) - 2 /*sizeof(ito::uint16)*/) * 8), 32); //qBound in order to avoid compiler warning for negative or too big shift values
            unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
            for (int row = 0; row < scrMat->rows; row++)
            {
                destPtr = (ito::uint16*)dstMat.ptr<ito::uint16>(row);
                linePtr = scrMat->ptr<const _TpSrc>(row);
                for (int col = 0; col < scrMat->cols; col++)
                {
                    destPtr[col] = (ito::uint16)(linePtr[col] / bitShift) +  signCorrection;
                }
            }
        }
        else //convert from 8bit to 16bit
        {
            unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
            for (int row = 0; row < scrMat->rows; row++)
            {
                destPtr = (ito::uint16*)dstMat.ptr<ito::uint16>(row);
                linePtr = scrMat->ptr<const _TpSrc>(row);
                for (int col = 0; col < scrMat->cols; col++)
                {
                    destPtr[col] = (ito::uint16)(linePtr[col]) + signCorrection;
                }
            }
        }
    }
    else
    {
        _TpSrc scaling = 65535.0;
        for (int row = 0; row < scrMat->rows; row++)
        {
            destPtr = dstMat.ptr<ito::uint16>(row);
            linePtr = scrMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < scrMat->cols; col++)
            {
                destPtr[col] = cv::saturate_cast<ito::uint16>(linePtr[col] * scaling);
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
template<typename _TpSrc> ito::RetVal transformScaledIndex8ToRGB(cv::Mat &dstMat, const cv::Mat *srcMat, const QVector<QRgb> &colorMap)
{
    dstMat = cv::Mat(srcMat->rows, srcMat->cols, CV_8UC3);

    const _TpSrc *linePtr = NULL;
    cv::Vec3b* destPtr = NULL;

    unsigned char index;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        _TpSrc bitShift = 1 << qBound<ito::uint8>(0, ((sizeof(_TpSrc) - 1) * 8), 32); //qBound in order to avoid compiler warning for negative or too big shift values
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        unsigned char index = 0;

        for (int row = 0; row < srcMat->rows; row++)
        {
            linePtr = srcMat->ptr<const _TpSrc>(row);
            destPtr = dstMat.ptr<cv::Vec3b>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
                destPtr[col][2] = qRed(colorMap[index]);
                destPtr[col][1] = qGreen(colorMap[index]);
                destPtr[col][0] = qBlue(colorMap[index]);
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < srcMat->rows; row++)
        {
            linePtr = srcMat->ptr<const _TpSrc>(row);
            destPtr = dstMat.ptr<cv::Vec3b>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);
                destPtr[col][2] = qRed(colorMap[index]);
                destPtr[col][1] = qGreen(colorMap[index]);
                destPtr[col][0] = qBlue(colorMap[index]);
            }
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//transformScaledIndex8ToRGB
template<typename _TpSrc> ito::RetVal transformScaledIndex8ToRGB(QImage &dstImg, const cv::Mat *srcMat, const QVector<QRgb> &colorMap)
{
    dstImg = QImage(srcMat->cols, srcMat->rows, QImage::Format_RGB32);

    const _TpSrc *linePtr = NULL;
    QRgb *destPtr = NULL;
    unsigned char index;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        _TpSrc bitShift = 1 << qBound<ito::uint8>(0, ((sizeof(_TpSrc) - 1) * 8), 32); //qBound in order to avoid compiler warning for negative or too big shift values
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        unsigned char index = 0;

        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = (QRgb*)dstImg.scanLine(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
                destPtr[col] = colorMap[index];
                destPtr[col] |= 0xff000000; //set alpha to 0xff
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = (QRgb*)dstImg.scanLine(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);
                destPtr[col] = colorMap[index];
                destPtr[col] |= 0xff000000; //set alpha to 0xff
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
template<typename _TpSrc> ito::RetVal transformScaledIndex8ToRGBA(cv::Mat &dstMat, const cv::Mat *srcMat, const QVector<QRgb> &colorMap)
{
    dstMat = cv::Mat(srcMat->rows, srcMat->cols, CV_8UC4);

    const _TpSrc *linePtr = NULL;
    cv::Vec4b* destPtr = NULL;
    unsigned char index;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        _TpSrc bitShift = 1 << qBound<ito::uint8>(0, ((sizeof(_TpSrc) - 1) * 8), 32);  //qBound in order to avoid compiler warning for negative or too big shift values
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        unsigned char index = 0;

        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = dstMat.ptr<cv::Vec4b>(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
                destPtr[col][2] = qRed(colorMap[index]);
                destPtr[col][1] = qGreen(colorMap[index]);
                destPtr[col][0] = qBlue(colorMap[index]);
                destPtr[col][3] = UCHAR_MAX;
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = dstMat.ptr<cv::Vec4b>(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);
                destPtr[col][2] = qRed(colorMap[index]);
                destPtr[col][1] = qGreen(colorMap[index]);
                destPtr[col][0] = qBlue(colorMap[index]);


                if (ito::isFinite<_TpSrc>(linePtr[col]))
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

//-----------------------------------------------------------------------------------------------------------------------------
template<typename _TpSrc> ito::RetVal transformScaledIndex8ToRGBA(QImage &dstImg, const cv::Mat *srcMat, const QVector<QRgb> &colorMap)
{
    dstImg = QImage(srcMat->cols, srcMat->rows, QImage::Format_ARGB32);

    const _TpSrc *linePtr = NULL;
    QRgb *destPtr = NULL;
    unsigned char index;

    if (std::numeric_limits<_TpSrc>::is_exact)
    {
        _TpSrc bitShift = 1 << qBound<ito::uint8>(0, ((sizeof(_TpSrc) - 1) * 8), 32);  //qBound in order to avoid compiler warning for negative or too big shift values
        unsigned char signCorrection = std::numeric_limits<_TpSrc>::is_signed ? 128 : 0;
        unsigned char index = 0;

        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = (QRgb*)dstImg.scanLine(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = (ito::uint8)(linePtr[col] / bitShift) +  signCorrection;
                destPtr[col] = colorMap[index];
                destPtr[col] |= 0xff000000; //set alpha to 0xff
            }
        }
    }
    else
    {
        _TpSrc scaling = 255.0;
        for (int row = 0; row < srcMat->rows; row++)
        {
            destPtr = (QRgb*)dstImg.scanLine(row);
            linePtr = srcMat->ptr<const _TpSrc>(row);
            for (int col = 0; col < srcMat->cols; col++)
            {
                index = cv::saturate_cast<ito::uint8>(linePtr[col] * scaling);
                destPtr[col] = colorMap[index];

                if (ito::isFinite<_TpSrc>(linePtr[col]))
                {
                    destPtr[col] |= 0xff000000; //set alpha to 0xff
                }
                else
                {
                    destPtr[col] &= 0x00ffffff; //set alpha to 0x00
                }
            }
        }
    }

    return ito::retOk;
}

} //namespace io
} //namespace itom

#endif
