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
#include "common/fileUtils.h"
#include "DataObject/dataobj.h"
#include <qmath.h>


#pragma pack(push, 1)
typedef struct
{
    char magic1[4];
    char dll_version[4];
    char magic2[4];

} Vk4Header;

typedef struct
{
    ito::uint32 setting;
    ito::uint32 color_peak;
    ito::uint32 color_light;
    ito::uint32 light[3];
    ito::uint32 height[3];
    ito::uint32 color_peak_thumbnail;
    ito::uint32 color_thumbnail;
    ito::uint32 light_thumbnail;
    ito::uint32 height_thumbnail;
    ito::uint32 assemble;
    ito::uint32 line_measure;
    ito::uint32 line_thickness;
    ito::uint32 string_data;
    ito::uint32 reserved;
} Vk4OffsetTable;

typedef struct
{
    ito::uint32 size;
    ito::uint32 year;
    ito::uint32 month;
    ito::uint32 day;
    ito::uint32 hour;
    ito::uint32 minute;
    ito::uint32 second;
    ito::int32 diff_utc_by_minutes;
    ito::uint32 image_attributes;
    ito::uint32 user_interface_mode;
    ito::uint32 color_composite_mode;
    ito::uint32 num_layer;
    ito::uint32 run_mode;
    ito::uint32 peak_mode;
    ito::uint32 sharpening_level;
    ito::uint32 speed;
    ito::uint32 distance;
    ito::uint32 pitch;
    ito::uint32 optical_zoom;
    ito::uint32 num_line;
    ito::uint32 line0_pos;
    ito::uint32 reserved1[3];
    ito::uint32 lens_mag;
    ito::uint32 pmt_gain_mode;
    ito::uint32 pmt_gain;
    ito::uint32 pmt_offset;
    ito::uint32 nd_filter;
    ito::uint32 reserved2;
    ito::uint32 persist_count;
    ito::uint32 shutter_speed_mode;
    ito::uint32 shutter_speed;
    ito::uint32 white_balance_mode;
    ito::uint32 white_balance_red;
    ito::uint32 white_balance_blue;
    ito::uint32 camera_gain;
    ito::uint32 plane_compensation;
    ito::uint32 xy_length_unit;
    ito::uint32 z_length_unit;
    ito::uint32 xy_decimal_place;
    ito::uint32 z_decimal_place;
    ito::uint32 x_length_per_pixel;
    ito::uint32 y_length_per_pixel;
    ito::uint32 z_length_per_digit;
    ito::uint32 reserved3[5];
    ito::uint32 light_filter_type;
    ito::uint32 reserved4;
    ito::uint32 gamma_reverse;
    ito::uint32 gamma;
    ito::uint32 gamma_offset;
    ito::uint32 ccd_bw_offset;
    ito::uint32 numerical_aperture;
    ito::uint32 head_type;
    ito::uint32 pmt_gain2;
    ito::uint32 omit_color_image;
    ito::uint32 lens_id;
    ito::uint32 light_lut_mode;
    ito::uint32 light_lut_in0;
    ito::uint32 light_lut_out0;
    ito::uint32 light_lut_in1;
    ito::uint32 light_lut_out1;
    ito::uint32 light_lut_in2;
    ito::uint32 light_lut_out2;
    ito::uint32 light_lut_in3;
    ito::uint32 light_lut_out3;
    ito::uint32 light_lut_in4;
    ito::uint32 light_lut_out4;
    ito::uint32 upper_position;
    ito::uint32 lower_position;
    ito::uint32 light_effective_bit_depth;
    ito::uint32 height_effective_bit_depth;
    /* more values in original file */
} Vk4MeasurementConditions;

typedef struct
{
    ito::uint32 width;
    ito::uint32 height;
    ito::uint32 bit_depth;
    ito::uint32 compression;
    ito::uint32 byte_size;
    ito::uint32 palette_range_min;
    ito::uint32 palette_range_max;
    ito::uint8 palette[0x300];
} Vk4TopoIntensityImage;

typedef struct
{
    ito::uint32 width;
    ito::uint32 height;
    ito::uint32 bit_depth;
    ito::uint32 compression;
    ito::uint32 byte_size;
} Vk4ColorImage;
#pragma pack(pop)

#define PICOMETRE (1.e-12)

//-------------------------------------------------------------------------------------------------------
ito::RetVal readDataImage(QFile &file, const QByteArray &setname, const Vk4OffsetTable &offsets, const Vk4MeasurementConditions &measconds, ito::DataObject &dataobj)
{
    int index = 0;
    ito::uint32 offset;
    bool topoNotColor = false;
    bool isHeight = false;
    bool considerNaN = false;
    ito::RetVal retval;

    if (setname.startsWith("topo"))
    {
        index = setname.mid(4).toInt();
        offset = offsets.height[index];
        topoNotColor = true;
        isHeight = true;
        considerNaN = true;
    }
    else if (setname.startsWith("intensity"))
    {
        index = setname.mid(4).toInt();
        offset = offsets.light[index];
        topoNotColor = true;
    }
    else if (setname.startsWith("peak"))
    {
        offset = offsets.color_peak;
        topoNotColor = false;
    }
    else if (setname.startsWith("color"))
    {
        offset = offsets.color_light;
        topoNotColor = false;
    }


    if (offset == 0)
    {
        retval += ito::RetVal::format(ito::retError, 0, "requested data '%s' could not be found in file", setname.data());
    }
    else
    {
        if (file.seek(offset))
        {
            if (topoNotColor)
            {
                Vk4TopoIntensityImage header;
                retval += ito::readFromDevice(&file, (char*)(&header), sizeof(Vk4TopoIntensityImage));

                if (header.width <= 0 || header.height <= 0 || \
                    (header.bit_depth != 8 && header.bit_depth != 16 && header.bit_depth != 32))
                {
                    retval += ito::RetVal(ito::retError, 0, "wrong dimensions or bit depth in requested data");
                }
                else if (header.byte_size != (header.width * header.height * header.bit_depth / 8))
                {
                    retval += ito::RetVal(ito::retError, 0, "image data does not correspond to given meta information");
                }

                if (!retval.containsError())
                {
                    ito::tDataType datatype;
                    switch (header.bit_depth)
                    {
                    case 8:
                        datatype = ito::tUInt8;
                        break;
                    case 16:
                        datatype = ito::tUInt16;
                        break;
                    case 32:
                        datatype = ito::tInt32;
                        break;
                    }

                    double scale = (isHeight) ? measconds.z_length_per_digit * PICOMETRE : qPow(0.5, header.bit_depth);

                    if (datatype != ito::tInt32)
                    {
                        ito::DataObject temp(header.height, header.width, datatype);
                        char* ptr = (char*)temp.rowPtr(0, 0);
                        retval += ito::readFromDevice(&file, ptr, header.byte_size);
                        retval += temp.convertTo(dataobj, ito::tFloat64, scale, 0.0);

                        if (considerNaN && datatype == ito::tUInt8)
                        {
                            const ito::uint8* source = (const ito::uint8*)temp.rowPtr(0, 0);
                            ito::float64* dest = (ito::float64*)dataobj.rowPtr(0, 0);

                            for (size_t i = 0; i < header.height * header.width; ++i)
                            {
                                if (source[i] == 0)
                                {
                                    dest[i] = std::numeric_limits<ito::float64>::quiet_NaN();
                                }
                            }
                        }
                        else if (considerNaN && datatype == ito::tUInt16)
                        {
                            const ito::uint16* source = (const ito::uint16*)temp.rowPtr(0, 0);
                            ito::float64* dest = (ito::float64*)dataobj.rowPtr(0, 0);

                            for (size_t i = 0; i < header.height * header.width; ++i)
                            {
                                if (source[i] == 0)
                                {
                                    dest[i] = std::numeric_limits<ito::float64>::quiet_NaN();
                                }
                            }
                        }
                    }
                    else
                    {
                        //dataobject does not support uint32
                        ito::DataObject temp(header.height, header.width, ito::tFloat64);
                        ito::float64 *ptr = (ito::float64*)temp.rowPtr(0, 0);
                        char *buf = new char[header.byte_size];
                        retval += ito::readFromDevice(&file, buf, header.byte_size);
                        const ito::uint32 *buf_ = (const ito::uint32*)buf;

                        if (considerNaN)
                        {
                            for (size_t i = 0; i < header.height * header.width; ++i)
                            {
                                ptr[i] = buf_[i] == 0 ? std::numeric_limits<ito::float64>::quiet_NaN() : (ito::float64)buf_[i] * scale;
                            }
                        }
                        else
                        {
                            for (size_t i = 0; i < header.height * header.width; ++i)
                            {
                                ptr[i] = (ito::float64)buf_[i] * scale;
                            }
                        }
                        DELETE_AND_SET_NULL(buf);
                        dataobj = temp;
                    }

                    dataobj.setAxisUnit(0, "m");
                    dataobj.setAxisUnit(1, "m");
                    dataobj.setAxisScale(0, measconds.y_length_per_pixel * PICOMETRE);
                    dataobj.setAxisScale(1, measconds.x_length_per_pixel * PICOMETRE);

                    if (isHeight)
                    {
                        dataobj.setValueUnit("m");
                    }
                }
            }
            else
            {
                Vk4ColorImage header;
                retval += ito::readFromDevice(&file, (char*)(&header), sizeof(Vk4ColorImage));

                if (header.width <= 0 || header.height <= 0 || \
                    (header.bit_depth != 24))
                {
                    retval += ito::RetVal(ito::retError, 0, "wrong dimensions or bit depth in requested data");
                }
                else if (header.byte_size != (header.width * header.height * header.bit_depth / 8))
                {
                    retval += ito::RetVal(ito::retError, 0, "image data does not correspond to given meta information");
                }

                if (!retval.containsError())
                {
                    dataobj = ito::DataObject(header.height, header.width, ito::tRGBA32);
                    ito::Rgba32* ptr = dataobj.rowPtr<ito::Rgba32>(0, 0);
                    char* buf = new char[header.byte_size];
                    retval += ito::readFromDevice(&file, buf, header.byte_size);
                    const char* buf_ = const_cast<char*>(buf); //running variable

                    if (!retval.containsError())
                    {
                        for (ito::uint32 i = 0; i < header.width * header.height; ++i)
                        {
                            ptr->a = 255;
                            ptr->r = buf_[0];
                            ptr->g = buf_[1];
                            ptr->b = buf_[2];
                            ++ptr;
                            buf_ += 3;
                        }
                    }

                    DELETE_AND_SET_NULL_ARRAY(buf);

                    dataobj.setAxisUnit(0, "m");
                    dataobj.setAxisUnit(1, "m");
                    dataobj.setAxisScale(0, measconds.y_length_per_pixel * PICOMETRE);
                    dataobj.setAxisScale(1, measconds.x_length_per_pixel * PICOMETRE);
                }
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, "file size too small to access requested data");
        }
    }

    return retval;

}


//-------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadKeyenceVK4Doc = QObject::tr(\
"This filter loads Keyence VK4 profilometry images from Keyence devices (e.g. laser scanning microscope).");

//-------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::loadKeyenceVK4Params(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Destination dataObject").toLatin1().data());
        paramsMand->append(param);

        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("setname", ito::ParamBase::String | ito::ParamBase::In, "topo0", tr("name of data set to load, empty string prints a list of all available dataset names. Possible values are: topo0, topo1, ..., intensity0, intensity1, ..., peak, color").toLatin1().data());
        ito::StringMeta *smset = new ito::StringMeta(ito::StringMeta::String);
        smset->addItem("topo0");
        smset->addItem("topo1");
        smset->addItem("topo2");
        smset->addItem("intensity0");
        smset->addItem("intensity1");
        smset->addItem("intensity2");
        smset->addItem("peak");
        smset->addItem("color");
        param.setMeta(smset, true);
        paramsOpt->append(param);

        param = ito::Param("xyUnit", ito::ParamBase::String | ito::ParamBase::In, "m", tr("Unit of x and y axes. VK4 assumes to have m as default unit, this can be scaled using other values than m. Default: mm.").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String, "mm");
        sm.addItem("cm");
        sm.addItem("mm");
        sm.addItem("\u00B5m");  // mu m
        sm.addItem("nm");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("valueUnit", ito::ParamBase::String | ito::ParamBase::In, "m", tr("Unit of value axis (only valid for topography data channels). VK4 assumes to have m as default unit, this can be scaled using other values than m. Default: mm.").toLatin1().data());
        ito::StringMeta sm2(ito::StringMeta::String, "mm");
        sm2.addItem("cm");
        sm2.addItem("mm");
        sm2.addItem("\u00B5m");  // mu m
        sm2.addItem("nm");
        param.setMeta(&sm2, false);
        paramsOpt->append(param);
    }

    return retval;
}

//-------------------------------------------------------------------------------------------------------
/*static*/ ito::RetVal DataObjectIO::loadKeyenceVK4(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    QFileInfo info(QLatin1String(paramsMand->at(1).getVal<char*>()));
    ito::uint32 fileSize = info.size();
    QByteArray setname = paramsOpt->at(0).getVal<const char*>();
    std::string xyUnit = paramsOpt->at(1).getVal<char*>();
    std::string valueUnit = paramsOpt->at(2).getVal<char*>();
    QList<QByteArray> availableSetnames;

    double xyScaleFactor = 1.0;

    if (xyUnit == "m")
    {
        xyScaleFactor = 1.0; //m is the default unit of Keyence
    }
    else if (xyUnit == "cm")
    {
        xyScaleFactor = 100.0;
    }
    else if (xyUnit == "mm")
    {
        xyScaleFactor = 1000.0;
    }
    else if (xyUnit == "\u00B5m" || (xyUnit.size() == 2 && xyUnit.data()[0] == -75 && xyUnit.data()[1] == 'm'))  // mu m
    {
        xyScaleFactor = 1.0e6;
    }
    else if (xyUnit == "nm")
    {
        xyScaleFactor = 1.0e9;
    }

    double valueScaleFactor = 1.0;

    if (valueUnit == "m")
    {
        valueScaleFactor = 1.0; //m is the default unit of Keyence
    }
    else if (valueUnit == "cm")
    {
        valueScaleFactor = 100.0;
    }
    else if (valueUnit == "mm")
    {
        valueScaleFactor = 1000.0;
    }
    else if (valueUnit == " \u00B5m" || (valueUnit.size() == 2 && valueUnit.data()[0] == -75 && valueUnit.data()[1] == 'm'))  // mu m
    {
        valueScaleFactor = 1.0e6;
    }
    else if (valueUnit == "nm")
    {
        valueScaleFactor = 1.0e9;
    }

    if (!info.exists())
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' does not exist", paramsMand->at(1).getVal<char*>());
    }
    else if (fileSize < sizeof(Vk4Header))
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' is no valid Keyence vk4 file", paramsMand->at(1).getVal<char*>());
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
            Vk4Header header;
            Vk4OffsetTable offsetTable;
            Vk4MeasurementConditions measCond;

            retval += ito::readFromDevice(&file, (char*)(&header), sizeof(Vk4Header));

            if (!retval.containsError() && \
                (memcmp(header.magic1, "VK4_", 4) != 0 || \
                memcmp(header.magic2, "\x00\x00\x00\x00", 4) != 0))
            {
                retval += ito::RetVal::format(ito::retError, 0, "The file '%s' is no valid Keyence vk4 file (wrong magic sequence).", paramsMand->at(1).getVal<char*>());
            }

            if (!retval.containsError())
            {
                retval += ito::readFromDevice(&file, (char*)(&offsetTable), sizeof(Vk4OffsetTable));
            }

            if (!retval.containsError())
            {
                //get available set names
                if (offsetTable.height[0] > 0) availableSetnames << "topo0";
                if (offsetTable.height[1] > 0) availableSetnames << "topo1";
                if (offsetTable.height[2] > 0) availableSetnames << "topo2";
                if (offsetTable.light[0] > 0) availableSetnames << "intensity0";
                if (offsetTable.light[1] > 0) availableSetnames << "intensity1";
                if (offsetTable.light[2] > 0) availableSetnames << "intensity2";
                if (offsetTable.color_peak > 0) availableSetnames << "peak";
                if (offsetTable.color_light > 0) availableSetnames << "color";

                if (setname == "")
                {
                    std::cout << "The keyence vk4 file contains the following datasets:\n";
                    foreach (const QByteArray &ba, availableSetnames)
                    {
                        std::cout << "- " << ba.data() << "\n";
                    }
                    std::cout << std::endl;
                    retval += ito::RetVal(ito::retError, 0, "abort loading keyence vk4 file (information request only)");
                }
                else if (availableSetnames.contains(setname) == false)
                {
                    retval += ito::RetVal::format(ito::retError, 0, "desired data setname '%s' not contained in file. Call filter with empty setname to get a list of available dataset names", setname.data());
                }
            }

            if (!retval.containsError())
            {
                retval += ito::readFromDevice(&file, (char*)(&measCond), sizeof(Vk4MeasurementConditions));
            }

            if (!retval.containsError())
            {
                ito::DataObject dataobj;

                retval += readDataImage(file, setname, offsetTable, measCond, dataobj);

                if (!retval.containsError())
                {
                    dataobj.setAxisScale(0, dataobj.getAxisScale(0) * xyScaleFactor);
                    dataobj.setAxisScale(1, dataobj.getAxisScale(1) * xyScaleFactor);
                    dataobj.setAxisUnit(0, xyUnit);
                    dataobj.setAxisUnit(1, xyUnit);

                    if (dataobj.getType() != ito::tRGBA32 && dataobj.getValueUnit() != "") //if valueUnit() is empty, we have intensity data
                    {
                        dataobj.setValueUnit(valueUnit);
                        if (valueScaleFactor != 1.0)
                        {
                            dataobj *= valueScaleFactor;
                        }
                    }
                }

                if (!retval.containsError())
                {
                    //add meta information
                    char buf[48];

                    sprintf_s(buf, sizeof(buf), "%u.%u.%u.%u",
                               header.dll_version[3], header.dll_version[2],
                               header.dll_version[1], header.dll_version[0]);
                    dataobj.setTag("dll_version", buf);

                    sprintf_s(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
                               measCond.year, measCond.month, measCond.day,
                               measCond.hour, measCond.minute, measCond.second);
                    dataobj.setTag("date", buf);

                    dataobj.setTag("time_diff_to_utc_in_minutes", measCond.diff_utc_by_minutes);
                    dataobj.setTag("image_attributes", measCond.image_attributes);
                    dataobj.setTag("ui_mode", measCond.user_interface_mode);
                    dataobj.setTag("color_composition_mode", measCond.color_composite_mode);
                    dataobj.setTag("image_layer_number", measCond.num_layer);
                    dataobj.setTag("run_mode", measCond.run_mode);
                    dataobj.setTag("peak_mode", measCond.peak_mode);
                    dataobj.setTag("sharpening_level", measCond.sharpening_level);
                    dataobj.setTag("speed", measCond.speed);
                    dataobj.setTag("distance", measCond.distance * 1.0e-6); //in mm (original in nm)
                    dataobj.setTag("pitch", measCond.pitch * 1.0e-6); //in mm (original in nm)
                    dataobj.setTag("optical_zoom", measCond.optical_zoom/10.0);
                    dataobj.setTag("number_of_lines", measCond.num_line);
                    dataobj.setTag("first_line_position", measCond.line0_pos);
                    dataobj.setTag("lens_magnification", measCond.lens_mag/10.0);
                    dataobj.setTag("PMT_gain mode", measCond.pmt_gain_mode);
                    dataobj.setTag("PMT_gain", measCond.pmt_gain);
                    dataobj.setTag("PMT_offset", measCond.pmt_offset);
                    dataobj.setTag("ND_filter", measCond.nd_filter);
                    dataobj.setTag("image_average_frequency", measCond.persist_count);
                    dataobj.setTag("shutter_speed_mode", measCond.shutter_speed_mode);
                    dataobj.setTag("shutter_speed", measCond.shutter_speed);
                    dataobj.setTag("white_balance_mode", measCond.white_balance_mode);
                    dataobj.setTag("white_balance_red", measCond.white_balance_red);
                    dataobj.setTag("white_balance_blue", measCond.white_balance_blue);
                    dataobj.setTag("camera_gain_in_dB", 6*measCond.camera_gain);
                    dataobj.setTag("plane_compensation", measCond.plane_compensation);
                    dataobj.setTag("light_filter_type", measCond.light_filter_type);
                    dataobj.setTag("gamma_reverse", measCond.gamma_reverse);
                    dataobj.setTag("gamma", measCond.gamma/100.0);
                    dataobj.setTag("gamma_correction_offset", measCond.gamma_offset/65536.0);
                    dataobj.setTag("CCD_BW_offset", measCond.ccd_bw_offset/100.0);
                    dataobj.setTag("numerical_aperture", measCond.numerical_aperture/1000.0);
                    dataobj.setTag("head_type", measCond.head_type);
                    dataobj.setTag("PMT_gain_2", measCond.pmt_gain2);
                    dataobj.setTag("omit_color_image", measCond.omit_color_image);
                    dataobj.setTag("lens_ID", measCond.lens_id);
                    dataobj.setTag("light_lut_mode", measCond.light_lut_mode);
                    dataobj.setTag("light_lut_input 0", measCond.light_lut_in0);
                    dataobj.setTag("light_lut_output 0", measCond.light_lut_out0);
                    dataobj.setTag("light_lut_input 1", measCond.light_lut_in1);
                    dataobj.setTag("light_lut_output 1", measCond.light_lut_out1);
                    dataobj.setTag("light_lut_input 2", measCond.light_lut_in2);
                    dataobj.setTag("light_lut_output 2", measCond.light_lut_out2);
                    dataobj.setTag("light_lut_input 3", measCond.light_lut_in3);
                    dataobj.setTag("light_lut_output 3", measCond.light_lut_out3);
                    dataobj.setTag("light_lut_input 4", measCond.light_lut_in4);
                    dataobj.setTag("light_lut_output 4", measCond.light_lut_out4);
                    dataobj.setTag("upper_position", measCond.upper_position * 1.0e-6); //in mm (original in nm)
                    dataobj.setTag("lower_position", measCond.lower_position * 1.0e-6); //in mm (original in nm)
                    dataobj.setTag("light_effective_bit_depth", measCond.light_effective_bit_depth);
                    dataobj.setTag("height_effective_bit_depth", measCond.height_effective_bit_depth);

                    if (offsetTable.string_data > 0)
                    {
                        if (!file.seek((qint64)(offsetTable.string_data)))
                        {
                            retval += ito::RetVal(ito::retWarning, 0, "Further string meta information could not be read from file");
                        }
                        else
                        {
                            ito::uint32 len;

                            //try to read title
                            retval += ito::readFromDevice(&file, (char*)(&len), sizeof(ito::uint32));
                            if (!retval.containsError() && len > 0)
                            {
                                ushort *title_ = new ushort[len];
                                retval += ito::readFromDevice(&file, (char*)title_, len * sizeof(ushort));
                                QString title = QString::fromUtf16(title_, len);
                                dataobj.setTag("title", title.toLatin1().data());
                                DELETE_AND_SET_NULL_ARRAY(title_);
                            }


                            //try to read lens name
                            if (!retval.containsError())
                            {
                                retval += ito::readFromDevice(&file, (char*)(&len), sizeof(ito::uint32));
                            }

                            if (!retval.containsError() && len > 0)
                            {
                                ushort *lensname_ = new ushort[len];
                                retval += ito::readFromDevice(&file, (char*)lensname_, len * sizeof(ushort));
                                QString lensname = QString::fromUtf16(lensname_, len);
                                dataobj.setTag("lens_name", lensname.toLatin1().data());
                                DELETE_AND_SET_NULL_ARRAY(lensname_);
                            }
                        }

                    }
                }

                if (!retval.containsError())
                {
                    *((*paramsMand)[0].getVal<ito::DataObject*>()) = dataobj;
                }
            }

            file.close();
        }
    }

    if (retval.containsError())
    {
        *((*paramsMand)[0].getVal<ito::DataObject*>()) = ito::DataObject();
    }


    return retval;
}
