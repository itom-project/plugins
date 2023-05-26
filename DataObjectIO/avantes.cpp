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

#include "DataObjectIO.h"

#include "common/typeDefs.h"
#include "DataObject/dataobj.h"

//----------------------------------------------------------------------------------------------------------------------------------


typedef struct
{
    char serialNumber[10]; // array[0..9] of AnsiChar;
    char userFriendlyName[64]; //array[0..63] of AnsiChar;
    ito::uint8 status;
} AvantesIdentityType;

typedef struct
{
    ito::uint8 enable;
    ito::uint8 forgetPercentage;
} AvantesDarkCorrectionType;

typedef struct
{
    ito::uint16 smoothPix;
    ito::uint8 smoothModel;
} AvantesSmoothingType;

typedef struct
{
    ito::uint8 mode;
    ito::uint8 source;
    ito::uint8 sourceType;
} AvantesTriggerType;

typedef struct
{
    ito::uint16 strobeControl;
    ito::uint32 laserDelay;
    ito::uint32 laserWidth;
    ito::float32 laserWaveLength;
    ito::uint16 storeToRam;
} AvantesControlSettingsType;

typedef struct
{
    ito::uint16 startPixel;
    ito::uint16 stopPixel;
    ito::float32 integrationTime;
    ito::uint32 integrationDelay;
    ito::uint32 nrAverages;
    AvantesDarkCorrectionType corDynDark;
    AvantesSmoothingType smoothing;
    ito::uint8 saturationDetection;
    AvantesTriggerType trigger;
    AvantesControlSettingsType control;
} AvantesMeasConfigType;

typedef struct
{
    char                    marker[5]; // 'AVS82' independent of measure mode
    ito::uint8              numspectra; // number of spectra in file
    ito::uint32             length; // total length of this subfile, depends on number of pixels used
    ito::uint8              seqnum; // sequence number of spectrum in file, starting at 0
    ito::uint8              measmode; // Measurement Mode, 0=scope
                                        // 1=absorbance
                                        // 2=scope corrected for dark
                                        // 3=transmission
                                        // 4=reflectance
                                        // 5=irradiance
                                        // 6=relative irradiance
                                        // 7=temperature
    ito::uint8              bitness; // 14 or 16 bit AD used (0=14 and 1=16 bit)
    ito::uint8              SDmarker; // 1 if file originated from SDcard, 0 otherwise
    AvantesIdentityType     identity; // 75 byte structure with serial number and friendly name
    AvantesMeasConfigType   measconf;
    ito::uint32             timestamp;
    ito::uint32             SPCfiledate;
    ito::float32            detectortemp;
    ito::float32            boardtemp;
    ito::float32            NTC2volt; // raw voltage from NTC2
    ito::float32            colorTemp; // used in calculation of relative irradiance, without lampfile
    ito::float32            calIntTime; // integration time used in intensity calibration
    ito::float64            fitdata[5]; // array of 5 doubles, the wavelength calibration polynomial
    char                    comment[130]; // same size as in SPC
} AvantesRawFormat;


/*static*/ const QString DataObjectIO::loadAvantesRawDoc = QObject::tr(\
"load binary files from Avantes devices. The destinationObject is a RxN float32 data object, where R is the \
number of spectas in the file and N the number of pixels. Use the optional parameter 'dataType' to decide \
which type of data should be loaded (including the calibration wavelength table of the grating). \n\
\n\
The description of the file format has been taken from 'AvaSoft 8 File Formats.pdf'");

ito::RetVal DataObjectIO::loadAvantesRawParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Destination dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source filename").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("dataType", ito::ParamBase::Int | ito::ParamBase::In, 0, 3, 1, tr("type of data to be loaded: 0 fully calculated array of wavelengths, 1 scopemode spectrum, 2 dark spectrum, 3 reference spectrum").toLatin1().data());
        paramsOpt->append(param);
    }

    return retval;
}

/*static*/ ito::RetVal DataObjectIO::loadAvantesRaw(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    AvantesRawFormat metaBlock;
    ito::uint32 fileSize = 0;

    QFileInfo info(QLatin1String(paramsMand->at(1).getVal<char*>()));
    fileSize = info.size();

    int dataType = paramsOpt->at(0).getVal<int>();

    if (!info.exists())
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' does not exist", paramsMand->at(1).getVal<char*>());
    }
    else if (fileSize < sizeof(metaBlock))
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' is no valid Avantes file", paramsMand->at(1).getVal<char*>());
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
            const char* data = (char*)content.data();

            //unfortunately, items in a struct must not lie in neighbouring cells in memory, therey can be alignment cells in between.
            memcpy(&metaBlock.marker, data + 0, sizeof(metaBlock.marker));
            memcpy(&metaBlock.numspectra, data + 5, sizeof(metaBlock.numspectra));
            memcpy(&metaBlock.length, data + 6, sizeof(metaBlock.length));
            memcpy(&metaBlock.seqnum, data + 10, sizeof(metaBlock.seqnum));
            memcpy(&metaBlock.measmode, data + 11, sizeof(metaBlock.measmode));
            memcpy(&metaBlock.bitness, data + 12, sizeof(metaBlock.bitness));
            memcpy(&metaBlock.SDmarker, data + 13, sizeof(metaBlock.SDmarker));
            memcpy(&metaBlock.identity, data + 14, sizeof(metaBlock.identity));

            memcpy(&metaBlock.measconf.startPixel, data + 14 + 75 + 0, sizeof(metaBlock.measconf.startPixel));
            memcpy(&metaBlock.measconf.stopPixel, data + 14 + 75 + 2, sizeof(metaBlock.measconf.stopPixel));
            memcpy(&metaBlock.measconf.integrationTime, data + 14 + 75 + 4, sizeof(metaBlock.measconf.integrationTime));
            memcpy(&metaBlock.measconf.integrationDelay, data + 14 + 75 + 8, sizeof(metaBlock.measconf.integrationDelay));
            memcpy(&metaBlock.measconf.nrAverages, data + 14 + 75 + 12, sizeof(metaBlock.measconf.nrAverages));
            memcpy(&metaBlock.measconf.corDynDark, data + 14 + 75 + 16, sizeof(metaBlock.measconf.corDynDark));
            memcpy(&metaBlock.measconf.smoothing.smoothPix, data + 14 + 75 + 18, sizeof(metaBlock.measconf.smoothing.smoothPix));
            memcpy(&metaBlock.measconf.smoothing.smoothModel, data + 14 + 75 + 20, sizeof(metaBlock.measconf.smoothing.smoothModel));
            memcpy(&metaBlock.measconf.saturationDetection, data + 14 + 75 + 21, sizeof(metaBlock.measconf.saturationDetection));
            memcpy(&metaBlock.measconf.trigger, data + 14 + 75 + 22, sizeof(metaBlock.measconf.trigger));
            memcpy(&metaBlock.measconf.control.strobeControl, data + 14 + 75 + 25, sizeof(metaBlock.measconf.control.strobeControl));
            memcpy(&metaBlock.measconf.control.laserDelay, data + 14 + 75 + 27, sizeof(metaBlock.measconf.control.laserDelay));
            memcpy(&metaBlock.measconf.control.laserWidth, data + 14 + 75 + 31, sizeof(metaBlock.measconf.control.laserWidth));
            memcpy(&metaBlock.measconf.control.laserWaveLength, data + 14 + 75 + 35, sizeof(metaBlock.measconf.control.laserWaveLength));
            memcpy(&metaBlock.measconf.control.storeToRam, data + 14 + 75 + 39, sizeof(metaBlock.measconf.control.storeToRam));

            memcpy(&metaBlock.timestamp, data + 14 + 75 + 41, sizeof(metaBlock.timestamp));
            memcpy(&metaBlock.SPCfiledate, data + 14 + 75 + 41 + 4, sizeof(metaBlock.SPCfiledate));
            memcpy(&metaBlock.detectortemp, data + 14 + 75 + 41 + 8, sizeof(metaBlock.detectortemp));
            memcpy(&metaBlock.boardtemp, data + 14 + 75 + 41 + 12, sizeof(metaBlock.boardtemp));
            memcpy(&metaBlock.NTC2volt, data + 14 + 75 + 41 + 16, sizeof(metaBlock.NTC2volt));
            memcpy(&metaBlock.colorTemp, data + 14 + 75 + 41 + 20, sizeof(metaBlock.colorTemp));
            memcpy(&metaBlock.calIntTime, data + 14 + 75 + 41 + 24, sizeof(metaBlock.calIntTime));
            memcpy(&metaBlock.fitdata, data + 14 + 75 + 41 + 28, sizeof(metaBlock.fitdata));
            memcpy(&metaBlock.comment, data + 14 + 75 + 41 + 28 + 5 * 8, sizeof(metaBlock.comment));

            if (metaBlock.marker[0] != 'A' || metaBlock.marker[1] != 'V' || metaBlock.marker[3] != '8' || metaBlock.marker[4] != '2')
            {
                retval += ito::RetVal::format(ito::retError, 0, "The file '%s' is no valid Avantes file. Init marker wrong.", paramsMand->at(1).getVal<char*>());
            }
            else
            {
                int pixels = metaBlock.measconf.stopPixel - metaBlock.measconf.startPixel + 1;
                int bpp = metaBlock.bitness == 0 ? 14 : 16;

                if (pixels > 0)
                {
                    ito::DataObject obj(metaBlock.numspectra, pixels, ito::tFloat32);
                    obj.setTag("seqnum", metaBlock.seqnum);
                    switch (metaBlock.measmode)
                    {
                    case 0:
                        obj.setTag("measmode", "scope");
                        break;
                    case 1:
                        obj.setTag("measmode", "absorbance");
                        break;
                    case 2:
                        obj.setTag("measmode", "scope corrected for dark");
                        break;
                    case 3:
                        obj.setTag("measmode", "transmission");
                        break;
                    case 4:
                        obj.setTag("measmode", "reflectance");
                        break;
                    case 5:
                        obj.setTag("measmode", "irradiance");
                        break;
                    case 6:
                        obj.setTag("measmode", "relative irradiance");
                        break;
                    case 7:
                        obj.setTag("measmode", "temperature");
                        break;
                    }
                    obj.setTag("bpp", bpp);
                    obj.setTag("deviceSerialNumber", metaBlock.identity.serialNumber);
                    obj.setTag("deviceName", metaBlock.identity.userFriendlyName);
                    obj.setTag("timestamp", metaBlock.timestamp * 1e-5);
                    obj.setTag("detectorTemperature", metaBlock.detectortemp);
                    obj.setTag("boardTemperature", metaBlock.boardtemp);
                    obj.setTag("wavelengthCalibPolynomial0", metaBlock.fitdata[0]);
                    obj.setTag("wavelengthCalibPolynomial1", metaBlock.fitdata[1]);
                    obj.setTag("wavelengthCalibPolynomial2", metaBlock.fitdata[2]);
                    obj.setTag("wavelengthCalibPolynomial3", metaBlock.fitdata[3]);
                    obj.setTag("wavelengthCalibPolynomial4", metaBlock.fitdata[4]);
                    obj.setTag("comment", metaBlock.comment);
                    obj.setTag("startPixel", metaBlock.measconf.startPixel);
                    obj.setTag("stopPixel", metaBlock.measconf.stopPixel);
                    obj.setTag("integrationTime", metaBlock.measconf.integrationTime);
                    obj.setTag("integrationDelay", metaBlock.measconf.integrationDelay);
                    obj.setTag("nrAverages", metaBlock.measconf.nrAverages);

                    ito::float32 *rowPtr = obj.rowPtr<ito::float32>(0, 0);
                    const char *startdata = data + 14 + 75 + 41 + 28 + 5 * 8 + 130 + dataType * metaBlock.numspectra * pixels * sizeof(ito::float32);
                    memcpy(rowPtr, startdata, metaBlock.numspectra * pixels * sizeof(ito::float32));


                    *((*paramsMand)[0].getVal<ito::DataObject*>()) = obj;
                }
                else
                {
                    *((*paramsMand)[0].getVal<ito::DataObject*>()) = ito::DataObject();
                }
            }

            file.close();
        }

    }


    return retval;
}
