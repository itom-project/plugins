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

#include <cstdlib>
#include <qstring.h>
#include <qstringlist.h>
#include <iostream>
#include <string>
#include <cerrno>
#include <QMap>
#include <QSharedPointer>

#include <v4l2_itom_api.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "libv4lconvert.h"

/*
 * Code based on:
 * https://github.com/jrimclean/libv4l2/blob/master/src/v4l2.c
 * https://github.com/philips/libv4l
 * http://git.linuxtv.org/v4l-utils.git/tree/HEAD
 */

// Constructor for a simple V4L2Format container
V4L2Format::V4L2Format(unsigned int width, unsigned int height, unsigned int pxformat)
{
    m_width=width;
    m_height=height;
    m_pxformat=pxformat;
    //corresponding string representation of pxformat (YUYV,etc)
    m_spxformat=fcc2s(pxformat);
}

//get list of device (/dev/videoX)
ito::RetVal DeviceList::refresh_dev_list()
{
    ito::RetVal retValue(ito::retOk);
    //get all video* files in /dev
    m_deviceFolder.setPath("/dev");
    m_deviceFolder.setNameFilters(QStringList()<<"video*");

    //fill devicelist with all available video devices
    if(m_deviceFolder.exists() && m_deviceFolder.isReadable())
    {
        QStringList deviceNameList = m_deviceFolder.entryList(QDir::System | QDir::Readable | QDir::Writable, QDir::Name);
        for (int i = 0; i < deviceNameList.count(); i++) {
            QString file = deviceNameList.at(i);
            if (file.length() > 5)
            {
                int val = file.mid(10).toInt();
                if (val >= 0) {
                    QString m_dev_name = m_deviceFolder.filePath(file);
                    m_deviceList.append(m_dev_name);
                }
            }
        }
        if (m_deviceList.count() == 0)
        {
            return retValue += ito::RetVal(ito::retError, 0, QObject::tr("No accessible video device could be found in search folder:  %1").arg( m_deviceFolder.absolutePath() ).toLatin1().data());
        }
    }
    else
    {
        return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Video device search folder %1 does not exist or is not readable").arg( m_deviceFolder.absolutePath() ).toLatin1().data());
    }
    return retValue;
}

//set active device, 0 -> /dev/video0
ito::RetVal DeviceList::set_active_dev(int deviceID)
{
    ito::RetVal retValue(ito::retOk);
    bool found = 0;
    for (int i = 0; i < get_number_of_dev(retValue); i++)
    {
        QString devname = m_deviceList.at(i);
        int val = devname.mid(10).toInt();
        if (val == deviceID)
        {
            m_device.m_dev_id=deviceID;
            m_device.m_dev_name=devname;
            found = 1;
            break;
        }
    }
    if (!found){
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Video device with id: %1 not found").arg(deviceID).toLatin1().data());
    }
    return retValue;
}

//return number of devices in list
int DeviceList::get_number_of_dev(ito::RetVal &retValue)
{
    int n = m_deviceList.count();
    retValue+=ito::retOk;
    return n;
}


ito::RetVal DeviceList::check_if_dev_id_exists(int deviceID)
{
    ito::RetVal retValue(ito::retOk);
    bool found = 0;
    for (int i = 0; i < get_number_of_dev(retValue); i++)
    {
        QString devname = m_deviceList.at(i);
        int val = devname.mid(10).toInt();
        if (val == deviceID)
        {
            //m_device.m_dev_id=deviceID;
            //m_device.m_dev_name=devname;
            found = 1;
            break;
        }
    }
    if (!found){
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Video device with id: %1 not found").arg(deviceID).toLatin1().data());
    }
    return retValue;
}

QString DeviceList::list_to_str()
{
    QString retstr="";
    ito::RetVal retValue(ito::retOk);
    for (int i = 0; i < get_number_of_dev(retValue); i++)
    {
        QString devname = m_deviceList.at(i);
        int val = devname.mid(10).toInt();
        QString addstr = QString("[%1]: '%2'\n").arg(val).arg(m_deviceList.at(i));
            retstr.append(addstr);
    }
    return retstr;

}

//fetch capabilities of device and save them in m_cap
ito::RetVal Device::open_fd()
{
    ito::RetVal retValue(ito::retOk);
    QString fileName = m_dev_name;
    if (!m_opened)
    {
        m_fd = open(fileName.toLatin1().data(), O_RDWR | O_NONBLOCK, 0);
        if(m_fd < 0)
        {
            retValue += ito::RetVal(ito::retError, 0, QObject::tr("Video device with id: %1 not found").arg(fileName).toLatin1().data());
        }
        m_opened=1;
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Video device: %1 already open").arg(fileName).toLatin1().data());
    }
    return retValue;
}

//close the fd
ito::RetVal Device::close_fd()
{
    ito::RetVal retValue(ito::retOk);
    if (m_opened){
        if (-1 == close(m_fd))
        {
            retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unable to close device: %1").arg(m_dev_name).toLatin1().data());
        }
        else
        {
            m_opened=0;
            m_fd=0;
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("Device: %1 not open").arg(m_dev_name).toLatin1().data());
    }
    return retValue;
}

//open device to be able to get and change parameters
ito::RetVal Device::open_dev()
{
    ito::RetVal retValue(ito::retOk);
    retValue+=open_fd();
    retValue += check_opened();
    //query capabilities
    if (retValue.containsError()) {
        return retValue;
    }
    retValue += load_cap();
    retValue+=get_fmt();
    retValue +=fetch_ctrls();
    return retValue;
}

//close device
ito::RetVal Device::close_dev()
{
    ito::RetVal retValue(ito::retOk);
    retValue+=close_fd();
    return retValue;
}

//wrapper for ioctl used to talk to v4l2
int Device::xioctl(int fd, int request, void *arg)
{
    int r;
    int no;
    do
    {
        r = ioctl(fd, request, arg);
        no = errno;
    } while (-1 == r && EINTR == errno);
    return r;
}

ito::RetVal    Device::check_opened(){
    ito::RetVal retValue(ito::retOk);
    if (!m_opened)
    {
        retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error %1 device is not open").arg(m_dev_name).toLatin1().data());
    }
    return retValue;
}

//check if we have to convert the video format selected by the user to get BGR24 needed for itom
ito::RetVal Device::check_convert()
{
    ito::RetVal retValue(ito::retOk);
    if(m_vfmt.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24)
    {
        m_mustConvert=true;
        //copy inputformat
        m_dvfmt = m_vfmt;
        //set output to BGR24
        m_convertData = v4lconvert_create(m_fd);
        m_dvfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
        v4l2_format copy = m_vfmt;
        int err;
        err = v4lconvert_try_format(m_convertData, &m_dvfmt, &m_vfmt);
        if (err !=0)
        {
            return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Could not convert from %1 to %2 format. E: %3").arg(fcc2s(m_vfmt.fmt.pix.pixelformat).c_str()).arg(fcc2s(m_dvfmt.fmt.pix.pixelformat).c_str()).arg(err).toLatin1().data());
        }
        m_vfmt = copy;
    }
    return retValue;
}

//grab a frame
ito::RetVal Device::grab_frame(unsigned char* dest)
{
    ito::RetVal retValue(ito::retOk);
    struct v4l2_buffer frame_buffer;

    memset(&frame_buffer, 0, sizeof(frame_buffer));

    frame_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frame_buffer.memory = V4L2_MEMORY_MMAP;

    //dequeue buffer
    if (-1 == xioctl(m_fd, VIDIOC_DQBUF, &frame_buffer))
    {
        //this case happens very often, dequeue is erroneous, why? we ignore it ...
        return retValue;
        //return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Could not dequeue buffer on device: %1. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());
    }

    int err = 0;
    if (m_mustConvert)
    {
        err=v4lconvert_convert(m_convertData, &m_vfmt, &m_dvfmt, (unsigned char *)m_buffers[frame_buffer.index].start, frame_buffer.bytesused, dest, m_dvfmt.fmt.pix.sizeimage);
    }
    if (!m_mustConvert)
    {
        memcpy(dest, m_buffers[frame_buffer.index].start, frame_buffer.bytesused);
    }

    if (err == -1)
    {
        std::cout << v4lconvert_get_error_message(m_convertData);
    }
    //requeue buffer
    if (-1 == xioctl(m_fd, VIDIOC_QBUF, &frame_buffer))
    {
        return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Could not requeue buffer on device: %1. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());
    }
    return retValue;
}

ito::RetVal Device::start_capture()
{
    ito::RetVal retValue(ito::retOk);
    retValue+=check_convert();
    retValue +=init_buffer();
    if (!retValue.containsError())
    {
        //allocate data buffer
        if (m_mustConvert)
        {
            m_data = new unsigned char[m_dvfmt.fmt.pix.sizeimage];
        }
        else{
            m_data = new unsigned char[m_buffers[0].length];
        }

        //queue buffers
        for (int i = 0; i < m_nbuffers; i++)
        {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf))
            {
                return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unable to queue buffers on device: %1. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());
            }
        }

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        //turn on stream
        if (-1 == xioctl(m_fd, VIDIOC_STREAMON, &type))
        {
            return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unable to turn on stream on device: %1. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());

        }
    }

    return retValue;
}


ito::RetVal Device::stop_capture()
{
    ito::RetVal retValue(ito::retOk);
    //free data buffer
    delete[] m_data;

    //turn off stream
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(m_fd, VIDIOC_STREAMOFF, &type))
    {
        return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unable to turn off stream on device: %1. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());
    }
    retValue+=delete_buffer();
    return retValue;
}


//return pointer to the complete ctrl map
QMap<QString, QSharedPointer<V4L2Ctrl> >* Device::get_parameters()
{
    QMap<QString, QSharedPointer<V4L2Ctrl> > *p_mctrls = NULL;
    p_mctrls = &m_controls;
    return p_mctrls;
}

//get only a qspointer to the ctrl defined by name in the qmap
QSharedPointer<V4L2Ctrl> Device::get_ctrl_by_name(QString &name)
{
    QSharedPointer<V4L2Ctrl> rctrl;
    if (m_controls.contains(name))
    {
        rctrl=m_controls[name];
    }

    return rctrl;
}

//query the capabilities of the device
ito::RetVal Device::load_cap()
{
    ito::RetVal retValue(ito::retOk);
    QString fileName = m_dev_name;

    //retValue+=check_opened();
    //if (retValue.containsError()){
    //    return retValue;
    //}
    //open_fd();
    //query capabilities
    if (-1 == Device::xioctl(m_fd, VIDIOC_QUERYCAP, &m_cap))
    {
        retValue += ito::RetVal(ito::retError, 0, QObject::tr("%1 not a V4L2 device. E: %2").arg(fileName).arg(errno).toLatin1().data());
    }
    if (!(m_cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error %1 is no video capture device. E: %2").arg(fileName).arg(errno).toLatin1().data());
    }
    if (!(m_cap.capabilities & V4L2_CAP_STREAMING))
    {
        retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error %1 does not support streaming. E: %2").arg(fileName).arg(errno).toLatin1().data());
    }
    //close_fd();
    return retValue;
}

//print out info of device stored in m_cap
ito::RetVal Device::print_cap()
{
    ito::RetVal retValue(ito::retOk);

    QString mystr=get_cap_string();
    std::cout << mystr.toLatin1().data();
    return ito::retOk;
}

//get the string containing the m_cap data
QString Device::get_cap_string()
{
    ito::RetVal retValue(ito::retOk);
    QString deviceName = m_dev_name;
    QString deviceCard = (const char *)m_cap.card;
    QString deviceBus = (const char *)m_cap.bus_info;
    QString deviceVersion = QString("%1.%2.%3").arg(m_cap.version>>16).arg((m_cap.version>>8)&0xff).arg(m_cap.version&0xff);
    QString deviceDriver = (const char *)m_cap.driver;
    QString response="";
    response.append(QString("Device:\t\t'%1'\nCamera:\t\t'%2'\nBus:    \t\t'%3'\nVersion:\t\t'%4'\nDriver:\t\t'%5'\n").arg(deviceName).arg(deviceCard).arg(deviceBus).arg(deviceVersion).arg(deviceDriver));
    return response;
}

//print all available mediaformats to let the user select
//print out the Videoformat List which is numbered so that user can select desired format
ito::RetVal Device::print_videofmts()
{
    ito::RetVal retValue(ito::retOk);
    QString outstr = "";
    int number = 0;
    retValue+=crawl_fmts(number, outstr);
    if (retValue.containsError()){
        return retValue;
    }

    std::cout << "Formats:\t'" << number <<"'"
            <<"\n\nAvailable mediaTypeIDs:" << outstr.toLatin1().data() <<std::endl;

    return retValue;
}

//get buffer size
size_t Device::get_buf_size()
{
    if (m_nbuffers > 0)
    {
        return m_buffers[0].length;
    }
    else
    {
        return 0;
    }
}

ito::RetVal Device::init_buffer()
{
    ito::RetVal retValue(ito::retOk);

    //init mmap
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(m_fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            retValue +=
                    ito::RetVal(ito::retError, 0,
                            QObject::tr("Error: %1 does not support memory mapping. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0,
                    QObject::tr("Error requesting buffers on device: %1. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());
        }
        return retValue;
    }

    if (req.count < 2)
    {
        return retValue +=
                ito::RetVal(ito::retError, 0,
                        QObject::tr("Insufficient buffer memory on: %1").arg(m_dev_name).toLatin1().data());
    }

    //allocate buffers
    try
    {
        m_buffers = new v4l2_buffer_t[req.count];
    }
    catch (...)
    {
        return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Out of memory").toLatin1().data());
    }

    //map buffers
    for (int i = 0; i < req.count; i++)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(m_fd, VIDIOC_QUERYBUF, &buf))
        {
            return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unable to query buffers on %1. E: %2").arg(m_dev_name).arg(errno).toLatin1().data());
        }

        m_buffers[i].length = buf.length;
        m_buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, buf.m.offset);

        if (MAP_FAILED == m_buffers[i].start)
        {
            return retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unable to map buffers on %1").arg(m_dev_name).toLatin1().data());
        }
    }
    m_nbuffers = req.count;
    return retValue;

}

ito::RetVal Device::delete_buffer()
{
    ito::RetVal retValue(ito::retOk);
    for (int i = 0; i < m_nbuffers; i++)
    {
        if (-1 == munmap(m_buffers[i].start, m_buffers[i].length))
        {
            retValue += ito::RetVal(ito::retError, 0, QObject::tr("Unable to unmap buffers on %1").arg(m_dev_name).toLatin1().data());
        }
    }
    return retValue;
}

//return number of formats supported by device
int Device::get_count_fmts()
{
    ito::RetVal retValue(ito::retOk);
    QString outstr = "";
    int number = 0;
    retValue+=crawl_fmts(number, outstr);
    if (retValue.containsError()){
        return -1;
    }
    return number;
}

//runs through all formats and increases number if valid format found, it also appends the format to outstr
// which is used by printVideoFormats
ito::RetVal Device::crawl_fmts(int &number, QString &outstr)
{
    ito::RetVal retValue(ito::retOk);
    struct v4l2_fmtdesc fmt;

    //retValue+=check_opened();
    //if (retValue.containsError()){
    //        return retValue;
    //}
    //open_fd();
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;


    while (xioctl(m_fd, VIDIOC_ENUM_FMT, &fmt)>=0)
    {
        QString cformat = QString::fromStdString(fcc2s(fmt.pixelformat));

        retValue+=crawl_frmsizes(fmt, number, outstr);
        if (retValue.containsError())
        {
            //close_fd();
            return retValue;
        }
        fmt.index++;
    }
    //close_fd();
    return retValue;
}

//for given format go through all supported framesizes, increase number of valid format found and append to outstr
// fill m_formatlist
ito::RetVal Device::crawl_frmsizes(v4l2_fmtdesc &fmt, int &number, QString &outstr){
    ito::RetVal retValue(ito::retOk);
    struct v4l2_frmsizeenum frmsize;
    struct v4l2_frmivalenum frmival;

    //retValue+=check_opened();
    //if (retValue.containsError()){
    //            return retValue;
    //}
    frmsize.pixel_format = fmt.pixelformat;
    frmsize.index = 0;
    while (xioctl(m_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0)
    {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            frmival.index = 0;
            frmival.pixel_format = fmt.pixelformat;
            frmival.width = frmsize.discrete.width;
            frmival.height = frmsize.discrete.height;

            if (xioctl(m_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) >= 0)
            {
                if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
                {
                    outstr = outstr +
                            QString("\nID %1: %2 x %3 px ").arg(number).arg(frmsize.discrete.width).arg(frmsize.discrete.height) +
                            QString::fromStdString(fract2fps(frmival.discrete) + " fps (" + fcc2s(fmt.pixelformat)+" Video)");
                }

                else if (frmival.type == V4L2_FRMIVAL_TYPE_STEPWISE)
                {
                    outstr = outstr +
                            QString("\nID %1: %2 x %3 px ").arg(number).arg(frmsize.discrete.width).arg(frmsize.discrete.width) +
                            QString::fromStdString(fract2fps(frmival.stepwise.max) + " fps ("+fcc2s(fmt.pixelformat)+" Video)");
                }
                //fill m_formatlist
                V4L2Format vformat(frmsize.discrete.width, frmsize.discrete.height,fmt.pixelformat);
                m_formatlist.append(vformat);
                number++;
            }
            else
            {
                return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("No valid frame interval found").toLatin1().data());
            }
        }
        else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {

            outstr = outstr +
                    QString("\nID %1: from %2 x %3 px to %4 x %5 px in steps of %6/%7,  ? fps (").arg(number).arg(frmsize.stepwise.min_width).arg(frmsize.stepwise.min_height).arg(frmsize.stepwise.max_width).arg(frmsize.stepwise.max_height).arg(frmsize.stepwise.step_width).arg(frmsize.stepwise.step_height)
                    +QString::fromStdString(fcc2s(fmt.pixelformat)+" Video)");
            V4L2Format vformat(frmsize.stepwise.max_width, frmsize.stepwise.max_height,fmt.pixelformat);
            m_formatlist.append(vformat);
            number++;
        }
        frmsize.index++;
    }
    return retValue;
}

//set format to the chosen number by the user
ito::RetVal Device::set_fmt(int number)
{
    ito::RetVal retValue(ito::retOk);

    v4l2_format fmt;

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //get the new values from the m_formatlist at position number chosen by the user
    if (number != -1){
        fmt.fmt.pix.width = m_formatlist.at(number).m_width;
        fmt.fmt.pix.height = m_formatlist.at(number).m_height;
        fmt.fmt.pix.pixelformat = m_formatlist.at(number).m_pxformat;
    }

    fmt.fmt.pix.bytesperline = 0;
    //fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (-1 == xioctl(m_fd, VIDIOC_S_FMT, &fmt))
    {
        //retValue+=close_fd();
        return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Could not set video format. E: %1").arg(strerror(errno)).toLatin1().data());
    }
    //close_fd();
    retValue+=get_fmt();
    std::cout <<"\n---------------------------V4L2------------------------------------" <<std::endl;
    retValue+=print_fmt(m_vfmt);
    std::cout <<"---------------------------V4L2------------------------------------\n"<< std::endl;

    //close_fd();
    return retValue;
}

//get current format set on device and save it  to m_vfmt
ito::RetVal Device::get_fmt()
{
    ito::RetVal retValue(ito::retOk);

    m_vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //open_fd();
    if (-1 == xioctl(m_fd, VIDIOC_G_FMT, &m_vfmt))
    {
        //retValue+=close_fd();
        return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Could not get video format. E: %1").arg(errno).toLatin1().data());
    }
    else
    {
        //retValue+=print_fmt(m_vfmt);
    }
    //close_fd();
    return retValue;
}

//get current format set on device and save it  to m_vfmt
QString Device::get_pixelformat_str()
{
    return fcc2s(m_vfmt.fmt.pix.pixelformat).c_str();
}

//print information about given format
ito::RetVal Device::print_fmt(const v4l2_format &vfmt)
{
    ito::RetVal retValue(ito::retOk);
    QString resp=get_fmt_str(vfmt);

    std::cout << resp.toLatin1().data();

    return retValue;
}

//get string of format
QString Device::get_fmt_str(const v4l2_format &vfmt)
{
    ito::RetVal retValue(ito::retOk);

    QString response="";
        response.append(QString("Format:\t\t'%1'\nWidth/Height:\t'%2 x %3'\nPixel Format:\t'%4'\nField:  \t\t'%5'\nBytes per Line:\t'%6'\nSize Image:\t'%7'\nColorspace:\t'%8'\n")
                .arg(buftype2s(vfmt.type).c_str()).arg(vfmt.fmt.pix.width).arg(vfmt.fmt.pix.height)
                .arg(fcc2s(vfmt.fmt.pix.pixelformat).c_str()).arg(field2s(vfmt.fmt.pix.field).c_str()).arg(vfmt.fmt.pix.bytesperline).arg(vfmt.fmt.pix.sizeimage)
                .arg(colorspace2s(vfmt.fmt.pix.colorspace).c_str()));
    return response;
}

//fetch all the ctrls of device (brightness, etc) and save the name and a pointer to each in the m_controls qmap
ito::RetVal Device::fetch_ctrls()
{
    ito::RetVal retValue(ito::retOk);
    struct v4l2_queryctrl queryctrl;

    //retValue += open_fd();
    //retValue += check_opened();
    memset (&queryctrl, 0, sizeof (queryctrl));

    for (queryctrl.id = V4L2_CID_BASE; queryctrl.id < V4L2_CID_LASTP1; queryctrl.id++)
    {
        if ( xioctl(m_fd, VIDIOC_QUERYCTRL, &queryctrl)>=0)
        {
            if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
            {
                continue;
            }
            QSharedPointer<V4L2Ctrl> ctrl(new V4L2Ctrl(this,queryctrl));
            m_controls.insert(to_itom_name(queryctrl), ctrl);
        }
        else
        {
            if (errno == EINVAL)
            {
                continue;
            }
            std::cout << "Error  VIDIOC_QUERYCTRL"<<std::endl;
            return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error  VIDIOC_QUERYCTRL").toLatin1().data());
        }
    }
    //close_fd();
    return retValue;
}

//print all available ctrls with their data, i.e. brightness, with min, max,step, value, default
ito::RetVal Device::print_ctrls()
{
    ito::RetVal retValue(ito::retOk);
    QMap<QString, QSharedPointer<V4L2Ctrl> >::iterator i;
    std::cout << "\nAvailable Controls:"<<std::endl;
    for(i=m_controls.begin();i!=m_controls.end();++i)
    {
        //QString name=i.key();
        //std::cout<<name.toLatin1().data()<<std::endl;;
        i.value()->print();
    }
    return retValue;
}

//if itom or any widget expects a special name for the ctrl you can set it here so that it is mapped to that name
// use the ctrl id list that can be found in videodev2.h
QString Device::to_itom_name(const struct v4l2_queryctrl &queryctrl)
{
    QString name;
    ito::RetVal retValue(ito::retOk);
    switch (queryctrl.type)
    {
    case V4L2_CID_BRIGHTNESS:
        name="brightness";
        break;
    case V4L2_CID_AUTOBRIGHTNESS:
        name="brightnessAuto";
        break;
    case V4L2_CID_CONTRAST:
        name="contrast";
        break;
    case V4L2_CID_SATURATION:
        name="saturation";
        break;
    case V4L2_CID_GAIN:
        name="gain";
        break;
    case V4L2_CID_AUTOGAIN:
        name="gainAuto";
        break;
    case V4L2_CID_SHARPNESS:
        name="sharpness";
        break;
    default:
        name=name2var((unsigned char*)queryctrl.name).c_str();
        break;
    }
    return name;
}

//Constructor for V4L2Ctrl
V4L2Ctrl::V4L2Ctrl(Device const *dev, struct v4l2_queryctrl &qctrl)
{
    m_dev=*dev;
    m_qctrl=qctrl;
    refresh();
}

//set value of ctrl, the value has to be set first with m_ctrl.value = X then call set();
ito::RetVal V4L2Ctrl::set()
{
    ito::RetVal retValue(ito::retOk);
    int err = m_dev.xioctl(m_dev.m_fd, VIDIOC_S_CTRL, &m_ctrl);
    int no = errno;
    if ( err==-1 && errno == ERANGE)
    {
        return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error  VIDIOC_S_CTRL, value out of range! E: %1").arg(strerror(errno)).toLatin1().data());
    }
    else if (err==-1)
    {
        return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error  VIDIOC_S_CTRL E: %1").arg(strerror(errno)).toLatin1().data());
    }
    retValue+=refresh();
    return retValue;
}

//return name of ctrl
QString V4L2Ctrl::get_name()
{
    return (const char*)m_qctrl.name;
}

//return type of ctrl, we only support int and bool at the moment
QString V4L2Ctrl::get_type()
{
    QString rtype="None";
    switch (m_qctrl.type)
    {
    case V4L2_CTRL_TYPE_INTEGER:
        rtype="Int";
        break;

    case V4L2_CTRL_TYPE_INTEGER64:
        break;

    case V4L2_CTRL_TYPE_STRING:
        break;
    case V4L2_CTRL_TYPE_BOOLEAN:
        rtype="Bool";
        break;
    case V4L2_CTRL_TYPE_MENU:
        break;
    case V4L2_CTRL_TYPE_BUTTON:
        break;
    case V4L2_CTRL_TYPE_BITMASK:
        break;
    default:
        break;
    }
    return rtype;
}

int V4L2Ctrl::min()
{
    return m_qctrl.minimum;
}

int V4L2Ctrl::max()
{
    return m_qctrl.maximum;
}

int V4L2Ctrl::step()
{
    return m_qctrl.step;
}

int V4L2Ctrl::default_value()
{
    return m_qctrl.default_value;
}

int V4L2Ctrl::value()
{
    return m_ctrl.value;
}

//print info for selected ctrl
ito::RetVal V4L2Ctrl::print()
{
    ito::RetVal retValue(ito::retOk);
    switch (m_qctrl.type)
    {
    case V4L2_CTRL_TYPE_INTEGER:
        std::cout <<m_qctrl.name
        <<" (int):\tmin="<<m_qctrl.minimum
        <<" max=" << m_qctrl.maximum
        <<" step=" << m_qctrl.step
        <<" default=" << m_qctrl.default_value
        <<" value=" << m_ctrl.value<<std::endl;
        break;

    case V4L2_CTRL_TYPE_INTEGER64:
        std::cout <<m_qctrl.name
        <<" (int64):\tvalue="<< m_ctrl.value<<std::endl;
        //<<ctrl->value64
        break;

    case V4L2_CTRL_TYPE_STRING:
        std::cout <<m_qctrl.name
        <<" (string):\tmin="<<m_qctrl.minimum
        <<" max=" << m_qctrl.maximum
        <<" step=" << m_qctrl.step
        <<" value=" << m_ctrl.value<<std::endl;
        //
        break;
    case V4L2_CTRL_TYPE_BOOLEAN:
        std::cout <<m_qctrl.name
        <<" (bool):\tdefault="<<m_qctrl.default_value
        <<" value="<<m_ctrl.value
        <<std::endl;
        //
        break;
    case V4L2_CTRL_TYPE_MENU:
        std::cout <<m_qctrl.name <<" (menu):\t";
        retValue+=enumerate_menu();
        //<<" max=" << m_qctrl.maximum
        //<<" default=" << m_qctrl.default_value<<std::endl;
        std::cout <<" value=" << m_ctrl.value<<std::endl;
        break;
    case V4L2_CTRL_TYPE_BUTTON:
        std::cout <<m_qctrl.name
        <<" (button):\t"<<m_qctrl.name<<std::endl;
        break;
    case V4L2_CTRL_TYPE_BITMASK:
        std::cout <<m_qctrl.name
        <<" (bitmask):\tmax="<<m_qctrl.minimum
        <<" default=" << m_qctrl.default_value
        <<"value=" << m_ctrl.value <<std::endl;
        break;

    default:
        std::cout << "Not implemented type for ctr: "<< m_qctrl.name <<std::endl;
        break;
    }
    return retValue;
}

//used to get information if ctrl is of type menu
ito::RetVal V4L2Ctrl::enumerate_menu()
{
    ito::RetVal retValue(ito::retOk);
    struct v4l2_querymenu querymenu;

    std::cout << "[";

    memset (&querymenu, 0, sizeof (querymenu));
    querymenu.id = m_qctrl.id;
    //retValue+=m_dev.open_fd();
    //retValue+=m_dev.check_opened();

    for (querymenu.index = m_qctrl.minimum; querymenu.index <= m_qctrl.maximum; querymenu.index++)
    {
        if (m_dev.xioctl(m_dev.m_fd, VIDIOC_QUERYMENU, &querymenu) >=0)

        {
            std::cout << " "<<querymenu.name << "|";
        }
        else
        {
            std::cout << "Error: VIDIOC_QUERYMENU E:"<<strerror(errno) << std::endl;
            //retValue+=m_dev.close_fd();
            return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error: VIDIOC_QUERYMENU: E: %1").arg(strerror(errno)).toLatin1().data());
        }
    }
    //retValue+=m_dev.close_fd();
    std::cout << "]";
    return retValue;
}

//refresh the m_ctrl data, specially the value if it has changed
ito::RetVal V4L2Ctrl::refresh()
{
    ito::RetVal retValue(ito::retOk);
    memset (&m_ctrl, 0, sizeof (m_ctrl));
    m_ctrl.id=m_qctrl.id;
    if ( m_dev.xioctl(m_dev.m_fd, VIDIOC_G_CTRL, &m_ctrl)==-1)
    {
        return retValue+=ito::RetVal(ito::retError, 0, QObject::tr("Error  VIDIOC_G_CTRL E:%1").arg(strerror(errno)).toLatin1().data());
    }
    return retValue;
}


// string conversion methods based on v4l2-ctrl
static std::string num2s(unsigned num)
{
    char buf[10];

    sprintf(buf, "%08x", num);
    return buf;
}

std::string buftype2s(int type)
{
    switch (type) {
    case 0:
        return "Invalid";
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        return "Video Capture";
    case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
        return "Video Capture Multiplanar";
    case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        return "Video Output";
    case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
        return "Video Output Multiplanar";
    case V4L2_BUF_TYPE_VIDEO_OVERLAY:
        return "Video Overlay";
    case V4L2_BUF_TYPE_VBI_CAPTURE:
        return "VBI Capture";
    case V4L2_BUF_TYPE_VBI_OUTPUT:
        return "VBI Output";
    case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
        return "Sliced VBI Capture";
    case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
        return "Sliced VBI Output";
    case V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY:
        return "Video Output Overlay";
    default:
        return "Unknown (" + num2s(type) + ")";
    }
}

std::string fcc2s(unsigned int val)
{
    std::string s;

    s += val & 0xff;
    s += (val >> 8) & 0xff;
    s += (val >> 16) & 0xff;
    s += (val >> 24) & 0xff;
    return s;
}

static std::string frmtype2s(unsigned type)
{
    static const char *types[] = {
            "Unknown",
            "Discrete",
            "Continuous",
            "Stepwise"
    };

    if (type > 3)
        type = 0;
    return types[type];
}

static std::string fract2sec(const struct v4l2_fract &f)
{
    char buf[100];

    sprintf(buf, "%.3f", (1.0 * f.numerator) / f.denominator);
    return buf;
}

static std::string fract2fps(const struct v4l2_fract &f)
{
    char buf[100];

    sprintf(buf, "%.3f", (1.0 * f.denominator) / f.numerator);
    return buf;
}

std::string field2s(int val)
{
    switch (val) {
    case V4L2_FIELD_ANY:
        return "Any";
    case V4L2_FIELD_NONE:
        return "None";
    case V4L2_FIELD_TOP:
        return "Top";
    case V4L2_FIELD_BOTTOM:
        return "Bottom";
    case V4L2_FIELD_INTERLACED:
        return "Interlaced";
    case V4L2_FIELD_SEQ_TB:
        return "Sequential Top-Bottom";
    case V4L2_FIELD_SEQ_BT:
        return "Sequential Bottom-Top";
    case V4L2_FIELD_ALTERNATE:
        return "Alternating";
    case V4L2_FIELD_INTERLACED_TB:
        return "Interlaced Top-Bottom";
    case V4L2_FIELD_INTERLACED_BT:
        return "Interlaced Bottom-Top";
    default:
        return "Unknown (" + num2s(val) + ")";
    }
}

std::string colorspace2s(int val)
{
    switch (val) {
    case V4L2_COLORSPACE_SMPTE170M:
        return "Broadcast NTSC/PAL (SMPTE170M/ITU601)";
    case V4L2_COLORSPACE_SMPTE240M:
        return "1125-Line (US) HDTV (SMPTE240M)";
    case V4L2_COLORSPACE_REC709:
        return "HDTV and modern devices (ITU709)";
    case V4L2_COLORSPACE_BT878:
        return "Broken Bt878";
    case V4L2_COLORSPACE_470_SYSTEM_M:
        return "NTSC/M (ITU470/ITU601)";
    case V4L2_COLORSPACE_470_SYSTEM_BG:
        return "PAL/SECAM BG (ITU470/ITU601)";
    case V4L2_COLORSPACE_JPEG:
        return "JPEG (JFIF/ITU601)";
    case V4L2_COLORSPACE_SRGB:
        return "SRGB";
    default:
        return "Unknown (" + num2s(val) + ")";
    }
}

static std::string name2var(unsigned char *name)
{
    std::string s;
    int add_underscore = 0;

    while (*name) {
        std::string b = s;
        int l = b.length();
        if (isalnum(*name)) {
            if (add_underscore)
                s += '_';
            add_underscore = 0;
            s += std::string(1, tolower(*name));
        }
        else if (s.length()) add_underscore = 1;
        name++;
    }
    return s;
}

static std::string safename(const unsigned char *name)
{
    std::string s;

    while (*name) {
        if (*name == '\n') {
            s += "\\n";
        }
        else if (*name == '\r') {
            s += "\\r";
        }
        else if (*name == '\f') {
            s += "\\f";
        }
        else if (*name == '\\') {
            s += "\\\\";
        }
        else if ((*name & 0x7f) < 0x20) {
            char buf[3];

            sprintf(buf, "%02x", *name);
            s += "\\x";
            s += buf;
        }
        else {
            s += *name;
        }
        name++;
    }
    return s;
}
static std::string safename(const char *name)
{
    return safename((const unsigned char *)name);
}

static std::string ctrlflags2s(__u32 flags)
{
    static const flag_def def[] = {
            { V4L2_CTRL_FLAG_GRABBED,    "grabbed" },
            { V4L2_CTRL_FLAG_DISABLED,   "disabled" },
            { V4L2_CTRL_FLAG_READ_ONLY,  "read-only" },
            { V4L2_CTRL_FLAG_UPDATE,     "update" },
            { V4L2_CTRL_FLAG_INACTIVE,   "inactive" },
            { V4L2_CTRL_FLAG_SLIDER,     "slider" },
            { V4L2_CTRL_FLAG_WRITE_ONLY, "write-only" },
            { V4L2_CTRL_FLAG_VOLATILE,   "volatile" },
            { 0, NULL }
    };
    return flags2s(flags, def);
}

std::string flags2s(unsigned val, const flag_def *def)
{
    std::string s;

    while (def->flag) {
        if (val & def->flag) {
            if (s.length()) s += ", ";
            s += def->str;
            val &= ~def->flag;
        }
        def++;
    }
    if (val) {
        if (s.length()) s += ", ";
        s += num2s(val);
    }
    return s;
}
