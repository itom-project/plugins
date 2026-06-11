/* ********************************************************************
    Plugin "V4L2" for itom software
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

#ifndef V4L2_ITOM_API_H
#define V4L2_ITOM_API_H

#include <QDir>
#include <QMap>
#include <QSharedPointer>
#include <linux/videodev2.h>
#include "common/sharedStructures.h"

typedef struct {
    void *start;
    size_t length;
} v4l2_buffer_t;

typedef struct {
    unsigned flag;
    const char *str;
} flag_def;


//string methods to convert
static std::string num2s(unsigned num);
std::string fcc2s(unsigned int val);
std::string buftype2s(int type);
static std::string frmtype2s(unsigned type);
static std::string fract2sec(const struct v4l2_fract &f);
static std::string fract2fps(const struct v4l2_fract &f);
std::string field2s(int val);
std::string colorspace2s(int val);
static std::string name2var(unsigned char *name);
static std::string safename(const unsigned char *name);
static std::string safename(const char *name);
static std::string ctrlflags2s(__u32 flags);
std::string flags2s(unsigned val, const flag_def *def);

class V4L2Format
{
public:
    V4L2Format(unsigned int width, unsigned int height, unsigned int pxformat);
    unsigned int m_width;
    unsigned int m_height;
    unsigned int m_pxformat;
    std::string m_spxformat;
};

class V4L2Ctrl;

class Device
{
private:
    friend class V4L2Ctrl;
    int m_fd;
    unsigned int m_nbuffers;
    v4l2_buffer_t* m_buffers;
    unsigned char* m_data;
    bool m_mustConvert;
    struct v4lconvert_data *m_convertData;
    QMap<QString, QSharedPointer<V4L2Ctrl> > m_controls;

    ito::RetVal close_fd();
    ito::RetVal open_fd();
    ito::RetVal init_buffer();
    ito::RetVal delete_buffer();
    ito::RetVal load_cap();
    ito::RetVal crawl_fmts(int &number, QString &outstr);
    ito::RetVal crawl_frmsizes(v4l2_fmtdesc &fmt, int &number, QString &outstr);
    ito::RetVal check_opened();
    ito::RetVal check_convert();
    ito::RetVal fetch_ctrls();
    ito::RetVal enumerate_menu(struct v4l2_queryctrl &qctrl);
    QString to_itom_name(const struct v4l2_queryctrl &queryctrl);

public:
    Device() : m_dev_name("Empty"), m_nbuffers(0),m_dev_id(-1), m_opened(0),m_mustConvert(false){}
    QString m_dev_name;
    //List of supported formats
    QList<V4L2Format> m_formatlist;
    int m_dev_id;
    bool m_opened;
    struct v4l2_capability m_cap;
    //video format set
    struct v4l2_format m_vfmt;
    //destination video format to be set
    struct v4l2_format m_dvfmt;

    int xioctl(int fd, int request, void *arg);

    ito::RetVal open_dev();
    ito::RetVal    close_dev();

    ito::RetVal start_capture();
    ito::RetVal stop_capture();
    ito::RetVal grab_frame(unsigned char* dest);

    ito::RetVal print_cap();
    ito::RetVal print_videofmts();
    ito::RetVal    print_fmt(const v4l2_format &vfmt);
    ito::RetVal print_ctrls();

    ito::RetVal get_fmt();
    QString get_pixelformat_str();
    QString get_fmt_str(const v4l2_format &vfmt);
    QString get_cap_string();
    QMap<QString, QSharedPointer<V4L2Ctrl> >* get_parameters();
    size_t get_buf_size();
    int get_count_fmts();
    QSharedPointer<V4L2Ctrl> get_ctrl_by_name(QString &name);

    ito::RetVal set_fmt(int number);
};

class  DeviceList
{
private:
    QDir m_deviceFolder;
    QList<QString> m_deviceList;
public:
    Device m_device;
    ito::RetVal set_active_dev(int deviceID);
    ito::RetVal refresh_dev_list();
    int get_number_of_dev(ito::RetVal &retValue);
    ito::RetVal check_if_dev_id_exists(int deviceID);
    QString list_to_str();
};


class V4L2Ctrl
{
private:
    ito::RetVal enumerate_menu();
    Device m_dev;

public:
    V4L2Ctrl(Device const *dev, struct v4l2_queryctrl &qctrl);
    struct v4l2_queryctrl m_qctrl;
    struct v4l2_control m_ctrl;

    //refresh data in m_ctrl, contains current value of ctrl
    ito::RetVal refresh();

    //print all information and data of ctrl
    ito::RetVal print();

    //set ctrl

    ito::RetVal set();
    //methods to get values
    int max();
    int min();
    int step();
    int value();
    int default_value();
    QString get_name();
    QString get_type();
};

#endif // V4L2_ITOM_API_H
