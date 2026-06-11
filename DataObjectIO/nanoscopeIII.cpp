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

#include "DataObjectIO.h"

#include "common/typeDefs.h"
#include "DataObject/dataobj.h"
#include <math.h>

/*static*/ const QString DataObjectIO::loadNanoscopeIIIDoc = QObject::tr(\
"This filter loads NanoscopeIII data from Veeco devices. This filter was tested with files \
from an AFM device 'Dimension3100' using the file format 0x05310001. \
If a file of a other version is used no guarantee about scaling can be given.\
If a file of Version 4.3 is loaded, no information about the value units are available.");


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::loadNanoscopeIIIParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Empty dataObject where the image will be saved to. The destinationObject will be of type float32.").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Absolute or relative path to the Veeco nanoscope file (suffix 001, 002, 003, ...).").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("numberOfImage", ito::ParamBase::Int | ito::ParamBase::In, 0, new ito::IntMeta(-1, std::numeric_limits<int>::max()), tr("defines the index of the image which will be loaded (default: 0). If the parameter is set to -1 a list of all images included in the file will be printed.").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::loadNanoscopeIII(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;
    ito::uint32 fileSize = 0;

    QFileInfo info(QLatin1String(paramsMand->at(1).getVal<char*>()));
    fileSize = info.size();
    ito::DataObject *dObjDst = paramsMand->at(0).getVal<ito::DataObject*>();

    if (dObjDst == NULL)
    {
        retval += ito::RetVal::format(ito::retError, 0, tr("Dataobject not initialized").toLatin1().data());
    }
    if (!info.exists())
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' does not exist", paramsMand->at(1).getVal<char*>());
    }
    else
    {
        int numImage = (*paramsOpt)[0].getVal <int>();
        QFile file(info.absoluteFilePath());
        if (!file.open(QIODevice::ReadOnly))
        {
            retval += ito::RetVal::format(ito::retError, 0, "'%s' is no readable file", paramsMand->at(1).getVal<char*>());
        }
        else
        {
            float scalingFactor;
            unsigned long startImage;
            unsigned long imageLength;
            retval += readNanoscopeIIIHeader(file, *dObjDst, scalingFactor, startImage, imageLength, numImage);


        }

    }

    return retval;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::readNanoscopeIIIHeader(QFile &inFile, ito::DataObject &outObj, float &scalingFactor, unsigned long &startImage, unsigned long &imageLength, const int &numImage)
{
    ito::RetVal retval;
    QMap<QString, QString> metaDataObj;
    QMap<QString, QByteArray> rawMeta;
    QMap<QByteArray, QMap<QByteArray, QByteArray> > headerParts; //maps section to a map of key->value pairs of properties
    bool check(true);


    const char* magic_bin = "\\*File list\r\n";
    QByteArray header = inFile.readLine();
    if (header != magic_bin)
    {
        retval += ito::RetVal(ito::retError, 0, "File is no valid Veeco, Nanoscope III file format. Currently, only the binary file format is supported.");
    }

    if (!retval.containsError())
    {
        inFile.seek(0);
        QByteArray curLine = inFile.readLine();
        QList<QByteArray> list;
        QMap<QByteArray, QByteArray> *properties = NULL;
        QByteArray currentSection;
        QByteArray key, value;
        unsigned short numRows(0);
        unsigned short numColumns(0);
        bool isnonSquare;
        int bpp, startValue, imageCount(0);
        char* col;
        QString keyString, unit;
        double zScale;
        QList<QPair<QByteArray, QMap<QByteArray,QByteArray>*> > orderList;


        while (!curLine.contains((char)0x1A))//marks the end of the header
        {
            if (curLine.startsWith("\\*"))
            {
                key = curLine.mid(2);
                if (currentSection != key)
                {


                    currentSection = key;
                    if (key.contains("Ciao image list"))
                    {
                        QByteArray tem(currentSection);
                        tem = tem.append(QByteArray::number(imageCount));

                        headerParts[tem] = QMap<QByteArray, QByteArray>();
                        properties = &(headerParts[tem]);
                        orderList.append(QPair<QByteArray, QMap<QByteArray, QByteArray>*>(tem, properties));
                        imageCount += 1;
                    }
                    else
                    {
                        headerParts[currentSection] = QMap<QByteArray, QByteArray>();
                        properties = &(headerParts[currentSection]);
                        orderList.append(QPair<QByteArray, QMap<QByteArray, QByteArray>*>(currentSection, properties));
                    }
                }
            }
            else if (curLine.startsWith("\\@") && isdigit(curLine[2]) && curLine[3] == ':')
            {
                col = curLine.data()+4;
                col = strchr(col, ':');
                if (!col || !isspace(col[1]))//there must be a space after the colon
                {
                    retval += ito::RetVal(ito::retError, 0, "Missing colon in header line");
                    break;
                }
                *col = '\0';
                keyString = curLine; //since a string ends with \0 the key is now saved in keyString
                int id(0), step(0);
                while (*col != '\r')//search where the last bracket is closing
                {
                    col += 1;
                    step += 1;

                    if (*col == ']' || *col == ')' || *col == '}')
                    {
                        id = step;
                    }

                }
                col = col+(id-step);//go to last bracket
                do{
                    col += 1;
                } while (isspace(*col));//go to first value
                startValue = col - curLine.constData();
                while (*col != '\r' && !isspace(*col) )//search where the value ends
                {
                    col += 1;
                }
                value = curLine.mid(startValue, col - curLine.constData() - startValue);
                if (properties)
                {
                    (*properties)[QByteArray(keyString.toLatin1()).remove(0,1)] = value;
                }

            }
            else if (curLine.startsWith('\\') && curLine.contains(":"))
            {
                int idx = curLine.indexOf(':');
                key = curLine.mid(1, idx - 1);
                value = curLine.mid(idx + 2);
                value = value.left(value.size() - 2);// cut of endline character

                if (properties)
                {
                    (*properties)[key] = value;
                }
            }

            curLine = inFile.readLine();
        }
        if (numImage > imageCount - 1)
        {
            retval += ito::RetVal(ito::retError, 0, "The optional parameter 'numberOfImage' is bigger than the number of images included in the file.");
        }
        if (numImage == -1)//print images in command of itom and exit
        {
            std::cout << QString("Max. index of included images: %1\n").arg(imageCount-1).toLatin1().data();
            headerParts.erase(headerParts.find("File list end\r\n"));//remove end key
            orderList.removeLast();
            QPair<QByteArray, QMap<QByteArray, QByteArray>*> iter;
            QByteArray val;
            int count(0);
            foreach(iter, orderList)
            {
                val = iter.first;
                if (val.contains("Ciao image list"))//pick the selected Image
                {
                    retval += printOutInformation(&headerParts[val], count++);
                }
            }
            return retval;
        }
        if (!retval.containsError())
        {
            headerParts.erase(headerParts.find("File list end\r\n"));//remove end key
            orderList.removeLast();
            QPair<QByteArray, QMap<QByteArray, QByteArray>*> iter;
            QMap<QByteArray, QByteArray>::const_iterator it;
            QByteArray val;
            it = headerParts["File list\r\n"].constFind("Version");
            if (it != headerParts["File list\r\n"].constEnd())
            {
                if (strtol(it->data(), NULL, 16) >= 0x09200000)//if true we are assuming that we have 32bit data
                {
                    bpp = 4;
                }
                else
                {
                    bpp = 2;
                }

            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, "File has no version number. Maybe the file is broken");
            }
            it = headerParts["Scanner list\r\n"].constFind("@Sens. Zscan");//get zScale value
            if (it != headerParts["Scanner list\r\n"].constEnd())
            {
                const QByteArray byteScale(it.value());
                const char * ptr;
                int start, end;
                ptr = byteScale.data();
                while (!isdigit(*ptr))
                {
                    ++ptr;
                }
                start = ptr - byteScale.constData();
                while (!isspace(*ptr))
                {
                    ++ptr;
                }
                end = ptr - byteScale.constData() -1;
                zScale = strtod(byteScale.mid(start, end),NULL);
                while (*ptr!= '/')//get unit
                {
                    ptr++;
                }
                unit = byteScale.mid(end + 2, ptr - byteScale.constData() - end - 2);


            }
            else
            {
                retval += ito::RetVal(ito::retError, 0, "Can not evaluate zScaling value in 'Scanner list'");
            }
            if (!retval.containsError())
            {
                int ciaoImageCount(0);
                foreach(iter, orderList)
                {
                    val = iter.first;



                        if (val == "Scanner list\r\n" || val == "Microscope list\r\n" || val == "Equipment list\r\n" || val == "File list\r\n" || val == "Controller list\r\n")
                        {
                            continue;
                        }
                        if (val == "Ciao scan list\r\n" || val == "Afm list\r\n" || val == "Stm list\r\n" || val == "NC Afm list\r\n")
                        {
                            retval += readSize(&headerParts[val], numColumns, numRows);
                            retval += readIsNonSquareAspect(&headerParts[val], isnonSquare);
                        }
                        if (!val.contains("Ciao image list") || val == "AFM image list\r\n" || val == "STM image list\r\n" || val == "NCAFM image list\r\n" || val == "Ciao force image list\r\n" || val == "Image list\r\n")
                        {
                            continue;
                        }
                        if (val.contains("Ciao image list") && ciaoImageCount == numImage)//pick the selected Image
                        {
                            retval += mapToDataField(&headerParts[val], outObj, numColumns, numRows, isnonSquare, bpp, inFile, zScale, unit);
                        }
                        else
                        {
                            ++ciaoImageCount;
                        }


                }
                if (!retval.containsError())
                {
                    addTags(orderList, outObj, numImage);
                }

            }
        }

    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::readSize(const QMap<QByteArray, QByteArray>* map, unsigned short &x, unsigned short &y)
{
    ito::RetVal retval;
    bool check(true);
    QMap<QByteArray, QByteArray>::const_iterator it;
    it = map->constFind("Samps/line");
    if (it != map->constEnd())
    {
        x = it->toUShort(&check);

        if (!check)
        {
            retval += ito::RetVal(ito::retError, 0, "Can not convert Samps/line to an unsigned short. Maybe the file is broken");
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "Can not find Samps/line in header. Maybe the file is broken");
    }
    if (!retval.containsError())
    {


        it = map->constFind("Lines");
        if (it != map->constEnd())
        {
            y = it->toUShort(&check);
            if (!check)
            {
                retval += ito::RetVal(ito::retError, 0, "Can not convert Lines to an unsigned short. Maybe the file is broken");
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "Can not find Lines in header. Maybe the file is broken");
        }
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------

ito::RetVal DataObjectIO::readIsNonSquareAspect(const QMap<QByteArray, QByteArray>* map, bool &aspect)
{
    bool done(false);
    double temp;
    ito::RetVal retval;
    QMap<QByteArray, QByteArray>::const_iterator it;
    it = map->constFind("Aspect ratio");
    if (it != map->constEnd())
    {
        if (it->data() != QByteArray("1:1"))
        {
            aspect = false;
            done = true;
        }
        if (!done)
        {
            temp = strtod(it->data(), NULL);
            if (temp > 0.0 && temp != 1.0)
            {
                aspect = true;
            }
            aspect = false;
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, "Can not find Aspect ratio in header. Maybe the file is broken");
    }
    return retval;

}
//----------------------------------------------------------------------------------------------------------------------------------

ito::RetVal DataObjectIO::mapToDataField(const QMap<QByteArray, QByteArray>* map, ito::DataObject &outObj, unsigned short &gx, unsigned short &gy, bool &gNoneSquare, const int &bpp, QFile &inFile, const double &gzScale, const QString &unitStr)
{
    ito::RetVal retval;
    unsigned short x, y;
    unsigned long size;
    int offset;
    bool check, nonSquare, sizeOK(false), useGlobal(false), old(false);
    double fieldX, fieldY, scale;
    char *end, *s, un[5];




    QList<QByteArray> keyList(map->keys());
    if (!keyList.contains(QByteArray("Samps/line")) || !keyList.contains(QByteArray("Number of lines")) || !keyList.contains(QByteArray("Aspect ratio")) || !keyList.contains(QByteArray("Scan size")) || !keyList.contains("Data offset") || !keyList.contains("Data length") || !keyList.contains("@2:Z scale") )
    {    //version = 4.2
        old = true;
        if (!keyList.contains(QByteArray("Samps/line")) || !keyList.contains(QByteArray("Number of lines")) || !keyList.contains(QByteArray("Aspect ratio")) || !keyList.contains(QByteArray("Scan size")) || !keyList.contains("Data offset") || !keyList.contains("Data length") || !keyList.contains("Z scale"))
        {
            retval += ito::RetVal(ito::retError, 0, "Header does not provide all required information. Maybe the file is broken");
            return retval;
        }
    }

        QMap<QByteArray, QByteArray>::const_iterator it;
        it = map->constFind("Samps/line");
        x = it->toUShort(&check);
        if (!check)
        {
            retval += ito::RetVal(ito::retError, 0, "Can not convert Samps/line to an unsigned short. Maybe the file is broken");
        }
        it = map->constFind("Number of lines");
        y = it->toUShort(&check);
        if (!check)
        {
            retval += ito::RetVal(ito::retError, 0, "Can not convert Number of Lines to an unsigned short. Maybe the file is broken");
        }
        if (!retval.containsError())
        {
            retval += readIsNonSquareAspect(map, nonSquare);
            fieldX = strtod(map->constFind("Scan size")->data(),&end);
            if (*end != ' ')
            {
                retval += ito::RetVal(ito::retError, 0, "Cannot parse `Scan size' field. Maybe the file is broken");
            }
            s = end + 1;
            fieldY = strtod(s, &end);
            if (*end != ' ')
            {
                //old files do not have two numbers... we assume equal dimensions
                fieldY = fieldX;
                end = s;
            }
            while (isspace(*end))
                end++;
            if (sscanf(end, "%4s", un) != 1)
            {
                retval += ito::RetVal(ito::retError, 0, "Cannot parse `Scan size' field.");

            }
            if (!retval.containsError())
            {
                //TODO unit convert like in 672ff in nanoscope.c
                if (!gx)
                {
                    gx = x;
                }
                if (!gy)
                {
                    gy = y;
                }
                offset = map->constFind("Data offset")->toInt(&check); // not sure of a int is big enough but later the mid function is called and this only takes an int
                if (!check)
                {
                    retval += ito::RetVal(ito::retError, 0, "Cannot convert 'Data offset'.");
                }
                size =  map->constFind("Data length")->toULong(&check);
                if (!check)
                {
                    retval += ito::RetVal(ito::retError, 0, "Cannot convert 'Data length'.");
                }
                if (!old)
                {
                    scale = map->constFind("@2:Z scale")->toDouble(&check);
                    if (!check)
                    {
                        retval += ito::RetVal(ito::retError, 0, "Cannot convert '@2:Z scale'.");
                    }
                }
                else
                {
                    scale = map->constFind("Z scale")->toDouble(&check);
                    if (!check)
                    {
                        retval += ito::RetVal(ito::retError, 0, "Cannot convert 'Z scale'.");
                    }
                }

                //Try if the size fits to data
                if (!sizeOK && size == bpp*x*y)
                {
                    sizeOK = true;
                }
                if (!sizeOK && size == bpp*gx*gy)
                {
                    sizeOK = true;
                    useGlobal = true;
                }
                // if nothing fits lets try the best fitting possibility
                if (!sizeOK && size > bpp*MAX(x*y, gx*gy))
                {
                    sizeOK = true;
                    useGlobal = (x*y < gx*gy);
                }
                if (!sizeOK && size > bpp*MIN(x*y, gx*gy))
                {
                    sizeOK = true;
                    useGlobal = (x*y > gx*gy);
                }
                if (!sizeOK)
                {
                    retval += ito::RetVal(ito::retError, 0, "Cannot find a matching size of the data field to the data.");
                }
                if (!retval.containsError())
                {
                    if (useGlobal)
                    {
                        if (gx)
                        {
                            fieldX *= (double) gx / x;
                            x = gx;
                        }
                        if (gy)
                        {
                            fieldY *= (double) gy / y;
                            y = gy;
                        }
                    }
                    else if (nonSquare)
                    {//this could be wrong
                        if (nonSquare)
                        {
                            fieldY *= y;
                            fieldY /= x;
                        }
                        else
                        {
                            fieldY *= y;
                            fieldY /= gy;
                        }
                    }
                    if (!((fieldX = fabs(fieldX)) > 0)) //fix sizes if 0
                    {
                        fieldX = 1.0;
                    }
                    if (!((fieldY = fabs(fieldY)) > 0))
                    {
                        fieldY = 1.0;
                    }
                    //todo z scaling
                    switch (bpp)
                    {
                    case 4:
                        outObj = ito::DataObject(y, x, ito::tInt32);
                        break;
                    case 2:
                        outObj = ito::DataObject(y, x, ito::tInt16);
                    }
                    outObj.setAxisScale(0, fieldY / y);
                    outObj.setAxisScale(1, fieldX / x);
                    outObj.setAxisUnit(0, "\u00B5m"); // \mu m
                    outObj.setAxisUnit(1, "\u00B5m"); // \mu m


                    retval += readNanoscopeIIIData(inFile, &outObj, pow(1.0 / 256.0, bpp), offset, bpp);
                    if (!retval.containsError())
                    {
                        outObj *= scale*gzScale;
                        if (!old)// not clear what the unit for old data
                        {
                            outObj.setValueUnit(std::string(unitStr.toLatin1()));
                        }
                        if (keyList.contains("@2:Image Data"))
                        {
                            outObj.setValueDescription(map->constFind("@2:Image Data").value().mid(1, map->constFind("@2:Image Data").value().size() - 2).data()); //parse value description
                        }
                    }
                }

            }





        }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::readNanoscopeIIIData(QFile &inFile, ito::DataObject *outObj,const float &scalingFactor, const int &start, const int &bpp)
{
    ito::RetVal retval;
    inFile.seek(0);
    QByteArray file = inFile.readAll().mid(start);
    const char *startPtr = file.constData();
    ito::int32 *rowPtr = (ito::int32*)outObj->rowPtr(0, 0);
    memcpy(rowPtr, startPtr, bpp*outObj->getSize(0)*outObj->getSize(1));
    ito::DataObject rhs;
    outObj->convertTo(rhs, ito::tFloat32);
    rhs *= scalingFactor;
    *outObj = rhs;

    return retval;

}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::printOutInformation(const QMap<QByteArray, QByteArray>* map,const int& idx)
{
    ito::RetVal retval;
    QList<QByteArray> keyList(map->keys());
    if (keyList.contains("@2:Image Data"))
    {
        std::cout << idx << " :" << map->constFind("@2:Image Data").value().mid(1, map->constFind("@2:Image Data").value().size() - 2).data()<< "\n" << std::endl;
    }
    else
    {
        retval += ito::RetVal(ito::retWarning, 0, "Can not find description of Values in 'Ciao image list'");
    }
    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::addTags(const QList<QPair<QByteArray, QMap<QByteArray, QByteArray>*> > orderList, ito::DataObject &outObj, const int & numImage)
{
    ito::RetVal retval;
    QPair<QByteArray, QMap<QByteArray, QByteArray>*> item;
    QMap<QByteArray, QByteArray>::iterator iter;
    QByteArray val;
    int count(0);
    foreach(item, orderList)
    {
        val = item.first;
        if (val == "Scanner list\r\n" || val == "File list\r\n")
        {
            for (iter = item.second->begin(); iter != item.second->end(); ++iter)
            {
                outObj.setTag(iter.key().data(), iter.value().data());
            }
        }
        if (val.contains("Ciao image list"))
        {
            if (numImage == count)//choose the right image
            {
                for (iter = item.second->begin(); iter != item.second->end(); ++iter)
                {
                    outObj.setTag(iter.key().data(), iter.value().data());
                }
            }
            else
            {
                ++count;
            }
        }
    }
    return retval;
}
