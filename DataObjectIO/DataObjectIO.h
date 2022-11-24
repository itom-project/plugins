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

#ifndef DATAOBJECTIO_H
#define DATAOBJECTIO_H

#include "common/addInInterface.h"
#include <qsharedpointer.h>
#include <qfile.h>
#include <qfileinfo.h>
#include <qtextstream.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DataObjectIOInterface
*   @brief DataIO functions implemented in former ito mcpp measurement programm
*
*   AddIn Interface for the MCPPDataIO class s. also \ref MCPPDataIO
*/
class DataObjectIOInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        DataObjectIOInterface();
        ~DataObjectIOInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class DataObjectIO
*   @brief Algorithms used to process images and dataobjects with filters provided by openCV
*
*   In this class the algorithms used for the processing of images are implemented.
*   The filters wrapp openCV-Filters to python interface. Handling of 3D-Objects differs depending on the filter.
*
*/
class DataObjectIO : public ito::AddInAlgo
{
    Q_OBJECT

    protected:
        DataObjectIO();
        ~DataObjectIO() {};

    public:
        friend class DataObjectIOInterface;

        enum ImageFormat
        {
            noFormat = 0x00,
            tiffFormat = 0x01,
            pgmFormat = 0x02,
            ppmFormat = 0x03,
            jpgFormat = 0x04,
            jp2000Format = 0x05,
            bmpFormat = 0x06,
            pngFormat = 0x07,
            sunFormat = 0x08,
            gifFormat = 0x09,
            xpmFormat = 0x0a
        };

        enum
        {
            invWrite = 0x00,
            invIgnor = 0x01,
            invChange = 0x02,
            invBAD = 0x03,
            invHandlingMask = 0x0F
        } tInvalidHandling;

        static const QString saveDataObjectDoc;
        static ito::RetVal saveDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveDataObjectParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadDataObjectDoc;
        static ito::RetVal loadDataObject(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadDataObjectParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadDataFromTxtDoc;
        static ito::RetVal loadDataFromTxt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadDataFromTxtParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveDataToTxtDoc;
        static ito::RetVal saveDataToTxt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveDataToTxtParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveNistSDFDoc;
        static ito::RetVal saveNistSDF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveNistSDFParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveItomIDODoc;
        static ito::RetVal saveItomIDO(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveItomIDOParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveTiffDoc;
        static ito::RetVal saveTiff(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveTiffParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveJPGDoc;
        static ito::RetVal saveJPG(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveJPGParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString savePNGDoc;
        static ito::RetVal savePNG(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePNGParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveGIFDoc;
        static ito::RetVal saveGIF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveGIFParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveXPMDoc;
        static ito::RetVal saveXPM(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveXPMParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveBMPDoc;
        static ito::RetVal saveBMP(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveBMPParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString savePPMDoc;
        static ito::RetVal savePPM(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePPMParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString savePGMDoc;
        static ito::RetVal savePGM(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePGMParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString saveRASDoc;
        static ito::RetVal saveRAS(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal saveRASParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadImageDoc;
        static ito::RetVal loadImage(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadImageParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadNistSDFDoc;
        static ito::RetVal loadNistSDF(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadNistSDFParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadItomIDODoc;
        static ito::RetVal loadItomIDO(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadItomIDOParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadFrtDoc;
        static ito::RetVal loadFrt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadFrtParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString savePtbPRDoc;
        static ito::RetVal savePtbPR(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal savePtbPRParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadAvantesRawDoc;
        static ito::RetVal loadAvantesRaw(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadAvantesRawParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadZygoMetroProDoc;
        static ito::RetVal loadZygoMetroPro(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadZygoMetroProParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadKeyenceVK4Doc;
        static ito::RetVal loadKeyenceVK4(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static ito::RetVal loadKeyenceVK4Params(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);

        static const QString loadNanoscopeIIIDoc;
        static  ito::RetVal loadNanoscopeIII(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut);
        static  ito::RetVal loadNanoscopeIIIParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut);


    private:
        static ito::RetVal saveDataObjectOpenCV(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, const ImageFormat &imageFormat);
        static ito::RetVal saveDataObjectQt(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut, const ImageFormat &imageFormat);

        static ito::RetVal analyseTXTData(QTextStream &inFile, ito::DataObject &newObject, QChar &separator, QChar &decimalSign, const int flags, const int ignoreLines);
        static ito::RetVal readTXTDataBlock(QTextStream &inFile, ito::DataObject &newObject, const QChar &separator, const QChar &decimalSign, const int flags, const int ignoreLines, const QString &wrapSign);

        template<typename _Tp> static ito::RetVal writeDataBlock(QFile &outFile, const ito::DataObject *scrObject, const double zScale, const int decimals, const int flags, const char seperator, const double nanValue);
        template<typename _Tp> static ito::RetVal readDataBlock(QFile &inFile, ito::DataObject &newObject, const double zScale, const int flags, const QByteArray &nanString);
        static ito::RetVal readNistHeader(QFile &inFile, ito::DataObject &newObject, double &zscale, const int flags, const std::string &xyUnit, const std::string &valueUnit, QByteArray &nanString);
        static ito::RetVal readNistHeaderBinary(QFile &inFile, ito::DataObject &newObject, double &zscale, const int flags, const std::string &xyUnit, const std::string &valueUnit, QByteArray &nanString);

        static ito::RetVal readNanoscopeIIIHeader(QFile &inFile, ito::DataObject &outObj, float &scalingFactor, unsigned long &startImage, unsigned long &imageLength, const int &numImage);
        static ito::RetVal readNanoscopeIIIData(QFile &inFile, ito::DataObject *outObj, const float &scalingFactor, const int &start, const int &bpp);
        static ito::RetVal readSize(const QMap<QByteArray, QByteArray>* map, unsigned short &x, unsigned short &y);
        static ito::RetVal readIsNonSquareAspect(const QMap<QByteArray, QByteArray>* map, bool &aspect);
        static ito::RetVal printOutInformation(const QMap<QByteArray, QByteArray>* map, const int& idx);
        static ito::RetVal addTags(const QList<QPair<QByteArray, QMap<QByteArray, QByteArray>*> > orderList, ito::DataObject &outObj, const int & numImage);
        static ito::RetVal mapToDataField(const QMap<QByteArray, QByteArray>* map, ito::DataObject &outObj, unsigned short &gx, unsigned short &gy, bool &gNoneSquare, const int &bpp, QFile &inFile, const double &gzScale, const QString &unitStr);

        static void checkAndModifyFilenameSuffix(QFileInfo &file, const QString &desiredAndAllowedSuffix, const QString &allowedSuffix2 = QString(), const QString &allowedSuffix3 = QString());

    public slots:
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // DATAOBJECTIO_H
