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

#include <qdatetime.h>

#pragma pack(push, 1)
typedef struct
{
    //only Magic 1
    ito::int32 magic_number;
    ito::int16 header_format;
    ito::int32 header_size;
    ito::int16 swinfo_type;
    char swinfo_date[30];
    ito::int16 swinfo_vers_maj;
    ito::int16 swinfo_vers_min;
    ito::int16 swinfo_vers_bug;
    ito::int16 intens_org_x;
    ito::int16 intens_org_y;
    ito::int16 intens_width;
    ito::int16 intens_height;
    ito::int16 intens_n_buckets;
    ito::uint16 intens_range;
    ito::int32 intens_n_bytes;
    ito::int16 phase_org_x;
    ito::int16 phase_org_y;
    ito::int16 phase_width;
    ito::int16 phase_height;
    ito::int32 phase_n_bytes;
    ito::int32 time_stamp;
    char comment[82];
    ito::int16 source;
    ito::float32 intf_scale_factor;
    ito::float32 wavelength_in;
    ito::float32 num_aperture;
    ito::float32 obliquity_factor;
    ito::float32 magnification;
    ito::float32 lateral_res;
    ito::int16 acq_type;
    ito::int16 intens_avg_cnt;
    ito::int16 pzt_cal;
    ito::int16 pzt_gain_tolerance;
    ito::int16 pzt_gain;
    ito::float32 part_thickness;
    ito::int16 agc;
    ito::float32 target_range;
    ito::int16 rad_crv_veasure_eeq;
    ito::int32 min_mod;
    ito::int32 min_mod_count;
    ito::int16 phase_res;
    ito::int32 min_area;
    ito::int16 discon_action;
    ito::float32 discon_filter;
    ito::int16 connect_order;
    ito::int16 sign;
    ito::int16 camera_width;
    ito::int16 camera_height;
    ito::int16 sys_type;
    ito::int16 sys_board;
    ito::int16 sys_serial;
    ito::int16 inst_id;
    char obj_name[12];
    char part_name[40];
    ito::int16 codev_type;
    ito::int16 phase_avg_cnt;
    ito::int16 sub_sys_err;
    char unused[16]; /* Unused 16 bytes. */
    char part_ser_num[40];
    ito::float32 refractive_index;
    ito::int16 rem_tilt_bias;
    ito::int16 rem_fringes;
    ito::int32 max_area;
    ito::int16 setup_type;
    ito::int16 internal_use;
    ito::float32 pre_connect_filter;
    ito::float32 wavelength_in_2;
    ito::int16 wavelength_fold;
    ito::float32 wavelength_in_1;
    ito::float32 wavelength_in_3;
    ito::float32 wavelength_in_4;
    char wavelen_select[8];
    ito::int16 fda_res;
    char scan_descr[20];
    ito::int16 n_fiducials_a;
    ito::float32 fiducials_a[14];
    ito::float32 pixel_width;
    ito::float32 pixel_height;
    ito::float32 exit_pupil_diam;
    ito::float32 light_level_pct;
    ito::int32 coords_state;
    ito::float32 coords_x_pos;
    ito::float32 coords_y_pos;
    ito::float32 coords_z_pos;
    ito::float32 coords_x_rot;
    ito::float32 coords_y_rot;
    ito::float32 coords_z_rot;
    ito::int16 coherence_mode;
    ito::int16 surface_filter;
    char sys_err_file_name[28];
    char zoom_descr[8];
    ito::float32 alpha_part;
    ito::float32 beta_part;
    ito::float32 dist_part;
    ito::int16 cam_split_loc_x;
    ito::int16 cam_split_loc_y;
    ito::int16 cam_split_trans_x;
    ito::int16 cam_split_trans_y;
    char material_a[24];
    char material_b[24];
    ito::int16 cam_split_unused;
    //ito::int16 cam_split_trans_y2;
    ito::int16 unused2; /* Unused 2 bytes. */
    ito::float32 dmi_ctr_x;
    ito::float32 dmi_ctr_y;
    ito::int16 sph_dist_corr;
    ito::int16 unused3; /* Unused 2 bytes. */
    ito::float32 sph_dist_part_na;
    ito::float32 sph_dist_part_radius;
    ito::float32 sph_dist_cal_na;
    ito::float32 sph_dist_cal_radius;
    ito::int16 surface_type;
    ito::int16 intens_surface_type;
    ito::float32 z_position;
    ito::float32 power_multiplier;
    ito::float32 focus_multiplier;
    ito::float32 rad_crv_vocus_sal_lactor;
    ito::float32 rad_crv_vower_ral_lactor;
    ito::float32 ftp_left_pos;
    ito::float32 ftp_right_pos;
    ito::float32 ftp_pitch_pos;
    ito::float32 ftp_roll_pos;
    ito::float32 min_mod_pct;
    ito::int32 max_inten;
    ito::int16 ring_of_fire;
    ito::int8 unused4; /* Unused 1 byte. */
    ito::int8 rc_orientation;
    ito::float32 rc_distance;
    ito::float32 rc_angle;
    ito::float32 rc_diameter;
    ito::int16 rem_fringes_mode;
    ito::int16 unused5; /* Unused 2 byte. */
    ito::int16 frames_acquired;
    ito::int16 cavity_type;
    ito::float32 cam_frame_rate;
    ito::float32 tune_range;
    ito::int16 cal_pix_loc_x;
    ito::int16 cal_pix_loc_y;
    ito::int16 n_tst_cal_pts;
    ito::int16 n_ref_cal_pts;
    ito::float32 tst_cal_pts[4];
    ito::float32 ref_cal_pts[4];
    ito::float32 tst_cal_pix_opd;
    ito::float32 ref_cal_pix_opd;
    ito::int32 sys_serial2;
    char unused6[40];

} ZygoMetroProHeader;
#pragma pack(pop)

const struct
{
    unsigned int format;
    unsigned int magic;
    unsigned int size;
}
header_formats[] =
{
    { 1, 0x881b036f, 834, },
    { 2, 0x881b0370, 834, },
    { 3, 0x881b0371, 4096, },
    { 0, 0x0, 0}
};

//--------------------------------------------------------------------------------------------
ito::uint32 swap32(ito::uint32 uint32in)
{
    union s
    {
        char sa[4];
        ito::uint32 res;
    } temp;
    temp.res = uint32in;
    s uint32out;
    for (int teller = 0; teller<4; ++teller){
        uint32out.sa[teller] = temp.sa[3 - teller];
    }
    return uint32out.res;
}

//--------------------------------------------------------------------------------------------
ito::int32 swap32(ito::int32 int32in)
{
    union s
    {
        char sa[4];
        ito::int32 res;
    } temp;
    temp.res = int32in;
    s int32out;
    for (int teller = 0; teller<4; ++teller){
        int32out.sa[teller] = temp.sa[3 - teller];
    }
    return int32out.res;
}

//--------------------------------------------------------------------------------------------
ito::uint16 swap16(ito::uint16 uint16in)
{
    union s
    {
        char sa[2];
        ito::uint16 res;
    } temp;
    temp.res = uint16in;
    s uint16out;
    for (int teller = 0; teller<2; ++teller){
        uint16out.sa[teller] = temp.sa[1 - teller];
    }
    return uint16out.res;
}

//--------------------------------------------------------------------------------------------
ito::int16 swap16(ito::int16 int16in)
{
    union s
    {
        char sa[2];
        ito::int16 res;
    } temp;
    temp.res = int16in;
    s int16out;
    for (int teller = 0; teller<2; ++teller){
        int16out.sa[teller] = temp.sa[1 - teller];
    }
    return int16out.res;
}

//--------------------------------------------------------------------------------------------
ito::float32 swap_float32(ito::float32 float32in)
{
    union s
    {
        char pp[4];
        ito::float32 f;
    } temp;

    temp.f = float32in;
    s float32out;
    float32out.pp[0] = temp.pp[3];
    float32out.pp[1] = temp.pp[2];
    float32out.pp[2] = temp.pp[1];
    float32out.pp[3] = temp.pp[0];
    return float32out.f;
}


//-------------------------------------------------------------------------------------------------------
/*static*/ const QString DataObjectIO::loadZygoMetroProDoc = QObject::tr(\
"load binary MetroPro files from Zygo. This import filter is mainly tested with the first generation of MetroPro files. \n\
However, newer files should also be loaded with the limitation, that only the tags from the first generation are saved \n\
into the 'destinationObject'.\n\
\n\
Per default, a topography is loaded from the binary file into a float32 dataObject. If available, the optional parameter \n\
'topography' can be set to 0 in order to load the intensity data to a uint16 dataObject. The 'destinationObject' contains \n\
many tags that are directly obtained from the binary files. \n\
\n\
Use the optional parameters 'xyUnit' and 'valueUnit' to get the result in a desired unit. 'valueUnit' is ignored for intensity data. \n\
\n\
The file format has been implemented based on the MetroPro Reference Guide, version OMP-0347K.");

//-------------------------------------------------------------------------------------------------------
ito::RetVal DataObjectIO::loadZygoMetroProParams(QVector<ito::Param> *paramsMand, QVector<ito::Param> *paramsOpt, QVector<ito::Param> *paramsOut)
{
    ito::RetVal retval = prepareParamVectors(paramsMand, paramsOpt, paramsOut);
    if (!retval.containsError())
    {
        ito::Param  param = ito::Param("destinationObject", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, tr("Destination dataObject").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("filename", ito::ParamBase::String | ito::ParamBase::In, NULL, tr("Source filename").toLatin1().data());
        paramsMand->append(param);
        param = ito::Param("topography", ito::ParamBase::Int, 0, 1, 1, tr("load topography data (1, default), else intensity data (0)").toLatin1().data());
        paramsOpt->append(param);
        param = ito::Param("xyUnit", ito::ParamBase::String | ito::ParamBase::In, "m", tr("Unit of x and y axes. MetroPro assumes to have m as default unit, this can be scaled using other values than m. Default: mm.").toLatin1().data());
        ito::StringMeta sm(ito::StringMeta::String, "mm");
        sm.addItem("cm");
        sm.addItem("mm");
        sm.addItem("\u00B5m");  // mu m
        sm.addItem("nm");
        param.setMeta(&sm, false);
        paramsOpt->append(param);

        param = ito::Param("valueUnit", ito::ParamBase::String | ito::ParamBase::In, "m", tr("Unit of value axis. MetroPro assumes to have m as default unit, this can be scaled using other values than m. Default: mm.").toLatin1().data());
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
/*static*/ ito::RetVal DataObjectIO::loadZygoMetroPro(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, QVector<ito::ParamBase> *paramsOut)
{
    ito::RetVal retval;

    QFileInfo info(QLatin1String(paramsMand->at(1).getVal<char*>()));
    ito::uint32 fileSize = info.size();
    bool topo = paramsOpt->at(0).getVal<int>() > 0;
    std::string xyUnit = paramsOpt->at(1).getVal<char*>();
    std::string valueUnit = paramsOpt->at(2).getVal<char*>();

    double xyScaleFactor = 1.0;

    if (xyUnit == "m")
    {
        xyScaleFactor = 1.0; //m is the default unit of MetroPro
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
        valueScaleFactor = 1.0; //m is the default unit of MetroPro
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
    else if (fileSize < sizeof(ZygoMetroProHeader))
    {
        retval += ito::RetVal::format(ito::retError, 0, "The file '%s' is no valid Zygo MetroPro file", paramsMand->at(1).getVal<char*>());
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

            ZygoMetroProHeader *header_ref = (ZygoMetroProHeader*)data;
            ZygoMetroProHeader header = *(ZygoMetroProHeader*)data;

            //especially the first numbers in the struct are all saved in big-edian. At the end, most of the entries are saved in litte-endian.
            //swap all big-endian values (all with respect to guide 'MetroPro Reference Guide OMP-0347K'):
            header.magic_number = swap32(header.magic_number);
            header.header_format = swap16(header_ref->header_format);
            header.header_size = swap32(header_ref->header_size);
            header.swinfo_type = swap16(header_ref->swinfo_type);
            header.swinfo_vers_maj = swap16(header_ref->swinfo_vers_maj);
            header.swinfo_vers_min = swap16(header_ref->swinfo_vers_min);
            header.swinfo_vers_bug = swap16(header_ref->swinfo_vers_bug);
            header.intens_org_x = swap16(header_ref->intens_org_x);
            header.intens_org_y = swap16(header_ref->intens_org_y);
            header.intens_width = swap16(header_ref->intens_width);
            header.intens_height = swap16(header_ref->intens_height);
            header.intens_n_buckets = swap16(header_ref->intens_n_buckets);
            header.intens_range = swap16(header_ref->intens_range);
            header.intens_n_bytes = swap32(header_ref->intens_n_bytes);
            header.phase_org_x = swap16(header_ref->phase_org_x);
            header.phase_org_y = swap16(header_ref->phase_org_y);
            header.phase_width = swap16(header_ref->phase_width);
            header.phase_height = swap16(header_ref->phase_height);
            header.phase_n_bytes = swap32(header_ref->phase_n_bytes);
            header.time_stamp = swap32(header_ref->time_stamp);
            header.source = swap16(header_ref->source);
            header.intf_scale_factor = swap_float32(header_ref->intf_scale_factor);
            header.wavelength_in = swap_float32(header_ref->wavelength_in);
            header.num_aperture = swap_float32(header_ref->num_aperture);
            header.obliquity_factor = swap_float32(header_ref->obliquity_factor);
            header.magnification = swap_float32(header_ref->magnification);
            header.lateral_res = swap_float32(header_ref->lateral_res);
            header.acq_type = swap16(header_ref->acq_type);
            header.intens_avg_cnt = swap16(header_ref->intens_avg_cnt);
            header.pzt_cal = swap16(header_ref->pzt_cal);
            header.pzt_gain_tolerance = swap16(header_ref->pzt_gain_tolerance);
            header.pzt_gain = swap16(header_ref->pzt_gain);
            header.part_thickness = swap_float32(header_ref->part_thickness);
            header.agc = swap16(header_ref->agc);
            header.target_range = swap_float32(header_ref->target_range);
            header.rad_crv_veasure_eeq = swap16(header_ref->rad_crv_veasure_eeq);
            header.min_mod = swap32(header_ref->min_mod);
            header.min_mod_count = swap32(header_ref->min_mod_count);
            header.phase_res = swap16(header_ref->phase_res);
            header.min_area = swap32(header_ref->min_area);
            header.discon_action = swap16(header_ref->discon_action);
            header.discon_filter = swap_float32(header_ref->discon_filter);
            header.connect_order = swap16(header_ref->connect_order);
            header.sign = swap16(header_ref->sign);
            header.camera_width = swap16(header_ref->camera_width);
            header.camera_height = swap16(header_ref->camera_height);
            header.sys_type = swap16(header_ref->sys_type);
            header.sys_board = swap16(header_ref->sys_board);
            header.sys_serial = swap16(header_ref->sys_serial);
            header.inst_id = swap16(header_ref->inst_id);
            header.codev_type = swap16(header_ref->codev_type);
            header.phase_avg_cnt = swap16(header_ref->phase_avg_cnt);
            header.sub_sys_err = swap16(header_ref->sub_sys_err);
            header.refractive_index = swap_float32(header_ref->refractive_index);
            header.rem_tilt_bias = swap16(header_ref->rem_tilt_bias);
            header.rem_fringes = swap16(header_ref->rem_fringes);
            header.max_area = swap32(header_ref->max_area);
            header.setup_type = swap16(header_ref->setup_type);
            header.pre_connect_filter = swap_float32(header_ref->pre_connect_filter);
            header.wavelength_in_2 = swap_float32(header_ref->wavelength_in_2);
            header.wavelength_fold = swap16(header_ref->wavelength_fold);
            header.wavelength_in_1 = swap_float32(header_ref->wavelength_in_1);
            header.wavelength_in_3 = swap_float32(header_ref->wavelength_in_3);
            header.wavelength_in_4 = swap_float32(header_ref->wavelength_in_4);
            header.fda_res = swap16(header_ref->fda_res);
            header.n_fiducials_a = swap16(header_ref->n_fiducials_a);
            for (int i = 0; i < 14; ++i)
            {
                header.fiducials_a[i] = swap_float32(header_ref->fiducials_a[i]);
            }
            header.pixel_width = swap_float32(header_ref->pixel_width);
            header.pixel_height = swap_float32(header_ref->pixel_height);
            header.exit_pupil_diam = swap_float32(header_ref->exit_pupil_diam);
            header.light_level_pct = swap_float32(header_ref->light_level_pct);
            header.rem_fringes_mode = swap16(header_ref->rem_fringes_mode); //this is big-endian!


            bool found = false;
            for (int i = 0; i < 3; ++i)
            {
                if (header.magic_number == header_formats[i].magic && \
                    header.header_format == header_formats[i].format && \
                    header.header_size == header_formats[i].size)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                retval += ito::RetVal(ito::retError, 0, "The given file is no valid Zygo MetroPro file format.");
            }
            else
            {
                ito::DataObject obj;

                if (topo && header.phase_width != 0 && header.phase_height != 0)
                {
                    obj = ito::DataObject(header.phase_height, header.phase_width, ito::tFloat32);
                    obj.setAxisOffset(0, header.phase_org_y);
                    obj.setAxisOffset(1, header.phase_org_x);
                    obj.setValueUnit(valueUnit);
                    obj.setValueDescription("Height");

                    ito::int32* ptr = (ito::int32*)(data + header.header_size + header.intens_n_bytes);
                    ito::int32 val;
                    ito::float32* obj_ptr = obj.rowPtr<ito::float32>(0, 0);
                    ito::float32 factor = header.intf_scale_factor * header.obliquity_factor * header.wavelength_in * valueScaleFactor;
                    if (header.phase_res == 0) /*normal, each fringe represented by 4096 counts*/
                    {
                        factor /= 4096.0;
                    }
                    else if (header.phase_res == 1) /*1, high, each fringe represented by 32768 counts*/
                    {
                        factor /= 32768.0;
                    }
                    else /*2, very high, each fringe represented by 131072 counts*/
                    {
                        factor /= 131072.0;
                    }

                    if (header.sign == 1)
                    {
                        factor *= -1;
                    }

                    for (int i = 0; i < header.phase_width * header.phase_height; ++i)
                    {
                        val = swap32(*ptr);
                        if (val >= 2147483640)
                        {
                            *obj_ptr = std::numeric_limits<ito::float32>::quiet_NaN();
                        }
                        else
                        {
                            *obj_ptr = val * factor;
                        }
                        obj_ptr++;
                        ptr++;
                    }
                }
                else if (!topo && header.intens_width != 0 && header.intens_height != 0)
                {
                    obj = ito::DataObject(header.intens_height, header.intens_width, ito::tUInt16);
                    obj.setAxisOffset(0, header.intens_org_y);
                    obj.setAxisOffset(1, header.intens_org_x);
                    obj.setValueDescription("Intensity");

                    ito::uint16* ptr = (ito::uint16*)(data + header.header_size);
                    ito::uint16* obj_ptr = obj.rowPtr<ito::uint16>(0, 0);

                    for (int i = 0; i < header.intens_width * header.intens_height; ++i)
                    {
                        *obj_ptr = swap16(*ptr);
                        obj_ptr++;
                        ptr++;
                    }

                    obj.setTag("invalid", 65535);
                    if (header.sign == 1)
                    {
                        obj.setTag("inversData", 1);
                    }

                }
                else if (topo)
                {
                    retval += ito::RetVal(ito::retError, 0, "file does not contain topography data.");
                }
                else
                {
                    retval += ito::RetVal(ito::retError, 0, "file does not contain intensity data.");
                }

                if (!retval.containsError())
                {
                    obj.setAxisScale(0, header.lateral_res != 0.0 ? header.lateral_res * xyScaleFactor : 1.0);
                    obj.setAxisScale(1, header.lateral_res != 0.0 ? header.lateral_res * xyScaleFactor : 1.0);
                    obj.setAxisUnit(0, xyUnit);
                    obj.setAxisUnit(1, xyUnit);
                    obj.setAxisDescription(0, "y");
                    obj.setAxisDescription(1, "x");

                    obj.setTag("comment", header.comment);
                    obj.setTag("wavelength", header.wavelength_in);
                    obj.setTag("NA", header.num_aperture);
                    if (!topo)
                    {
                        obj.setTag("intensityAverages", header.intens_avg_cnt);
                    }
                    else
                    {
                        obj.setTag("phaseAverages", header.phase_avg_cnt);
                    }

                    switch (header.sys_type)
                    {
                    case 0:
                        obj.setTag("systemType", "Software");
                        break;
                    case 1:
                        obj.setTag("systemType", "Mark IVxp");
                        break;
                    case 2:
                        obj.setTag("systemType", "Maxim 3D");
                        break;
                    case 3:
                        obj.setTag("systemType", "Maxim NT");
                        break;
                    case 4:
                        obj.setTag("systemType", "GPI-XP");
                        break;
                    case 5:
                        obj.setTag("systemType", "NewView");
                        break;
                    case 6:
                        obj.setTag("systemType", "Maxim GP");
                        break;
                    case 7:
                        obj.setTag("systemType", "NewView/GP");
                        break;
                    case 8:
                        obj.setTag("systemType", "Mark to GPI conversion");
                        break;
                    default:
                        obj.setTag("systemType", "Unknown");
                        break;
                    }

                    obj.setTag("objective", QByteArray(header.obj_name).trimmed().data());
                    obj.setTag("scanDescription", header.scan_descr);
                    obj.setTag("intensityRange", header.intens_range);
                    QDateTime timestamp;
#if QT_VERSION >= QT_VERSION_CHECK(5, 8, 0)
                    timestamp.setSecsSinceEpoch(header.time_stamp);
#else
                    timestamp.setTime_t(header.time_stamp);
#endif
                    obj.setTag("timestamp", timestamp.toString().toLatin1().data());
                    obj.setTag("automaticGainControl", header.agc);
                    obj.setTag("exitPupilDiameter", header.exit_pupil_diam);
                    obj.setTag("lightLevelPercentage", header.light_level_pct);
                    obj.setTag("zoomDescription", QByteArray(header.zoom_descr).trimmed().data());
                    obj.setTag("interferometricScaleFactor", header.intf_scale_factor);
                    obj.setTag("phaseCorrectionFactor", header.obliquity_factor);
                    obj.setTag("acquireMode", header.acq_type == 0 ? "phase" : (header.acq_type == 1 ? "fringe" : "scan"));
                    obj.setTag("lateralResolution", header.lateral_res);
                    obj.setTag("pixelWidth", header.pixel_width);
                    obj.setTag("pixelHeight", header.pixel_height);
                    obj.setTag("removeRegressionPlane", header.rem_tilt_bias);
                    obj.setTag("removeFringesFromIntensity", header.rem_fringes);
                    obj.setTag("xPos", header.coords_x_pos);
                    obj.setTag("yPos", header.coords_y_pos);
                    obj.setTag("zPos", header.coords_z_pos);
                    obj.setTag("xRot", header.coords_x_rot);
                    obj.setTag("yRot", header.coords_y_rot);
                    obj.setTag("zRot", header.coords_z_rot);
                }

                *((*paramsMand)[0].getVal<ito::DataObject*>()) = obj;
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
