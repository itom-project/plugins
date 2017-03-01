/* ********************************************************************
Plugin "DslrRemote2" for itom software
URL: http://lccv.ufal.br/
Copyright (C) 2017, Universidade Federal de Alagoas (UFAL), Brazil

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

#ifndef PTPCAM_H
#define PTPCAM_H

#include "libptp2-1.2.0/src/ptp.h"
#include "libusb.h"
#include <qvector.h>
#include <qmap.h>

typedef struct _PTP_USB PTP_USB;
struct _PTP_USB {
    libusb_device_handle *handle;
    int inep;
    int outep;
    int intep;
};

class DslrRemote; // forward declaration

class PtpCam
{
public:
    PtpCam() {};
    ~PtpCam() {};

    ito::RetVal init_ptp_usb(PTPParams* params, PTP_USB* ptp_usb, struct libusb_device* dev);
    ito::RetVal clear_stall(PTP_USB* ptp_usb);
    ito::RetVal close_usb(PTP_USB* ptp_usb, struct libusb_device* dev);
    //struct libusb_device* find_device(int busn, int devn, short force);
    libusb_device* find_device(int portnum, short force);
    ito::RetVal find_endpoints(struct libusb_device *dev, int* inep, int* outep, int* intep);
    ito::RetVal open_camera(int portnum, short force, PTP_USB *ptp_usb, PTPParams *params, struct libusb_device **dev);
    ito::RetVal close_camera(PTP_USB *ptp_usb, PTPParams *params, struct libusb_device *dev);
    ito::RetVal list_devices(short force, QMap<int, QString> &deviceList);
    ito::RetVal show_info(int portnum, short force, QString &info);
    ito::RetVal capture_image(int portnum, short force);
    static void sig_alrm(int signo);
    ito::RetVal loop_capture(int portnum, short force, int n, int interval, int overwrite);
    ito::RetVal nikon_initiate_dc(int portnum, short force);
    ito::RetVal nikon_direct_capture(int portnum, short force, char* filename, int overwrite);
    ito::RetVal nikon_direct_capture2(int portnum, short force, char* filename, int overwrite);
    ito::RetVal get_last_file_handle(int portnum, short force, int &imgNum, uint32_t &handle, int &fileType, DslrRemote *parentHandle);
    ito::RetVal list_files(int portnum, short force, QVector<QString> &fileList);
    ito::RetVal delete_object(int portnum, short force, uint32_t handle);
    ito::RetVal delete_all_files(int portnum, short force);
    ito::RetVal save_object(PTPParams *params, uint32_t handle, char* filename, PTPObjectInfo oi, int overwrite);
    ito::RetVal get_save_object(PTPParams *params, uint32_t handle, char* filename, int overwrite);
    ito::RetVal get_file(int portnum, short force, uint32_t handle, char* filename, int overwrite);
    ito::RetVal get_all_files(int portnum, short force, int overwrite);
    ito::RetVal send_generic_request(int portnum, uint16_t reqCode, uint32_t *reqParams, uint32_t direction, char *data_file);
    ito::RetVal list_operations(int portnum, short force, QVector<QString> &operations);
    ito::RetVal list_properties(int portnum, short force, QVector<QString> &properties);
    ito::RetVal set_property(PTPParams* params, uint16_t property, const char* value, uint16_t datatype);
    ito::RetVal getset_property_internal(PTPParams* params, uint16_t property, char** value, short force);
    ito::RetVal getset_propertybyname(int portnum, char* property, char** value, short force);
    ito::RetVal getset_property(int portnum, uint16_t property, char** value, short force);
    ito::RetVal getset_property_value(int portnum, uint16_t property, char** value, short force);
    ito::RetVal show_all_properties(int portnum, short force, int unknown, QMap<QString, QString> &properties);
    int usb_ptp_get_device_status(PTP_USB* ptp_usb, uint16_t* devstatus);

private:
    void ptpcam_siginthandler(int signum);
    ito::RetVal init_usb(struct libusb_device ***devlist, int &cnt);

    static short ptp_read_func(unsigned char *bytes, unsigned int size, void *data);
    static short ptp_write_func(unsigned char *bytes, unsigned int size, void *data);
    static short ptp_check_int(unsigned char *bytes, unsigned int size, void *data);
    static void ptpcam_debug(void *data, const char *format, va_list args);
    static void ptpcam_error(void *data, const char *format, va_list args);

    //        uint16_t ptp_transaction_nodata(PTPParams* params, PTPContainer* ptp);
    //        uint16_t ptp_transaction_getdata(PTPParams* params, PTPContainer* ptp, unsigned int *getlen, char** data);
    //        uint16_t ptp_transaction_senddata(PTPParams* params, PTPContainer* ptp, unsigned int sendlen, char* data);

    int usb_get_endpoint_status(PTP_USB* ptp_usb, int ep, uint16_t* status);
    int usb_clear_stall_feature(PTP_USB* ptp_usb, int ep);
    int usb_ptp_device_reset(PTP_USB* ptp_usb);
    ito::RetVal reset_device(int portnum, short force);

    PTPParams *m_globalparams; //!> we need it for a proper signal handling :/
    static int m_ptpcam_usb_timeout;
    static int m_verbose;
};

#endif //PTPCAM_H