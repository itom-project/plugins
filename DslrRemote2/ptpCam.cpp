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
#include "common/retVal.h"
#include "DslrRemote.h"
#include "ptpCam.h"

#include <qobject.h>
#ifdef WIN32
#include <iostream>
#include <sys/utime.h>
#include <time.h>
#else
#include <unistd.h>
#include <sys/mman.h>
#include <signal.h>
#include <fcntl.h>
#include <utime.h>
#endif
//#include <usb.h>


#ifdef ENABLE_NLS
#  include <libintl.h>
#  undef _
#  define _(String) dgettext (GETTEXT_PACKAGE, String)
#  ifdef gettext_noop
#    define N_(String) gettext_noop (String)
#  else
#    define N_(String) (String)
#  endif
#else
#  define textdomain(String) (String)
#  define gettext(String) (String)
#  define dgettext(Domain,Message) (Message)
#  define dcgettext(Domain,Message,Type) (Message)
#  define bindtextdomain(Domain,Directory) (Domain)
#  define _(String) (String)
#  define N_(String) (String)
#endif

#ifndef USB_CLASS_PTP
#define USB_CLASS_PTP		6
#endif

// USB control message data phase direction
#ifndef USB_DP_HTD
#define USB_DP_HTD		(0x00 << 7)	// host to device
#endif
#ifndef USB_DP_DTH
#define USB_DP_DTH		(0x01 << 7)	// device to host
#endif

// PTP class specific requests
#ifndef USB_REQ_DEVICE_RESET
#define USB_REQ_DEVICE_RESET		0x66
#endif
#ifndef USB_REQ_GET_DEVICE_STATUS
#define USB_REQ_GET_DEVICE_STATUS	0x67
#endif

// USB Feature selector HALT
#ifndef USB_FEATURE_HALT
#define USB_FEATURE_HALT	0x00
#endif

// OUR APPLICATION USB URB (2MB) ;)
#define PTPCAM_USB_URB		2097152

#define USB_TIMEOUT		5000
#define USB_CAPTURE_TIMEOUT	20000

// the other one, it sucks definitely ;)
//int ptpcam_usb_timeout = USB_TIMEOUT;

int PtpCam::m_verbose = 0;
int PtpCam::m_ptpcam_usb_timeout = 3000;

//----------------------------------------------------------------------------------------------------------------------------------
void PtpCam::ptpcam_siginthandler(int signum)
{
    PTP_USB* ptp_usb = (PTP_USB *)m_globalparams->data;

#ifndef WIN32
    // this should not happen on windows, as there a new thread is created handling SIGINT
    if (signum == SIGINT)
    {
        /* hey it's not that easy though... but at least we can try! */
        printf("Got SIGINT, trying to clean up and close...\n");
        //usleep(5000);
        Sleep(5);
        if (m_pdev != NULL)
            close_camera(ptp_usb, m_globalparams, m_pdev);
        exit(-1);
    }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
short PtpCam::ptp_read_func(unsigned char *bytes, unsigned int size, void *data)
{
    int result = -1;
    PTP_USB *ptp_usb = (PTP_USB *)data;
    int toread = 0;
    signed long int rbytes = size;

    do {
        bytes += toread;
        if (rbytes>PTPCAM_USB_URB)
            toread = PTPCAM_USB_URB;
        else
            toread = rbytes;
        result = libusb_bulk_transfer(ptp_usb->handle, ptp_usb->inep, (unsigned char*)bytes, toread, &toread, m_ptpcam_usb_timeout);
        // sometimes retry might help
        if (result != 0)
            result = libusb_bulk_transfer(ptp_usb->handle, ptp_usb->inep, (unsigned char*)bytes, toread, &toread, m_ptpcam_usb_timeout);
        if (result < 0)
            break;
        rbytes -= PTPCAM_USB_URB;
    } while (rbytes>0);

    if (result >= 0) {
        return PTP_RC_OK;
    }
    else
    {
        if (m_verbose) perror("usb_bulk_read");
            return PTP_ERROR_IO;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
short PtpCam::ptp_write_func(unsigned char *bytes, unsigned int size, void *data)
{
    int result;
    PTP_USB *ptp_usb = (PTP_USB *)data;
    int written;
    result = libusb_bulk_transfer(ptp_usb->handle, ptp_usb->outep, bytes, size, &written, m_ptpcam_usb_timeout);

    if (result >= 0)
        return PTP_RC_OK;
    else
    {
        if (m_verbose) perror("usb_bulk_write");
            return PTP_ERROR_IO;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
// XXX this one is suposed to return the number of bytes read!!!
short PtpCam::ptp_check_int(unsigned char *bytes, unsigned int size, void *data)
{
    int result;
    PTP_USB *ptp_usb = (PTP_USB *)data;
    int written;

    result = libusb_bulk_transfer(ptp_usb->handle, ptp_usb->inep, bytes, size, &written, m_ptpcam_usb_timeout);
    if (result != 0)
        result = libusb_bulk_transfer(ptp_usb->handle, ptp_usb->inep, bytes, size, (int*)&size, m_ptpcam_usb_timeout);
    if (m_verbose > 2) fprintf(stderr, "USB_BULK_READ returned %i, size=%i\n", result, size);

    if (result >= 0)
    {
        return result;
    }
    else
    {
        if (m_verbose) perror("ptp_check_int");
            return result;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//void ptpcam_debug(void *data, const char *format, va_list args);
void PtpCam::ptpcam_debug(void *data, const char *format, va_list args)
{
    if (m_verbose < 2) return;
    vfprintf(stderr, format, args);
    fprintf(stderr, "\n");
    fflush(stderr);
}

//----------------------------------------------------------------------------------------------------------------------------------
//void ptpcam_error(void *data, const char *format, va_list args);
void PtpCam::ptpcam_error(void *data, const char *format, va_list args)
{
    vfprintf(stderr, format, args);
    fprintf(stderr, "\n");
    fflush(stderr);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::init_ptp_usb(PTPParams* params, PTP_USB* ptp_usb, struct libusb_device* dev)
{
    ito::RetVal retval;
    libusb_device_handle *device_handle;
    int ret;

    params->write_func = ptp_write_func;
    params->read_func = ptp_read_func;
    params->check_int_func = ptp_check_int;
    params->check_int_fast_func = ptp_check_int;
    params->error_func = ptpcam_error;
    params->debug_func = ptpcam_debug;
    params->sendreq_func = ptp_usb_sendreq;
    params->senddata_func = ptp_usb_senddata;
    params->getresp_func = ptp_usb_getresp;
    params->getdata_func = ptp_usb_getdata;

    params->data = ptp_usb;
    params->transaction_id = 0;
    params->byteorder = PTP_DL_LE;

    if (!libusb_open(dev, &device_handle))
    {
        if (!device_handle)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("usb_open()").toLatin1().data());
        }

        ptp_usb->handle = device_handle;
        struct libusb_config_descriptor *config = NULL;

        if (ret = libusb_get_config_descriptor(dev, 0, &config))
        {
            if (ret == LIBUSB_ERROR_ACCESS)
                retval += ito::RetVal(ito::retError, 9999, QObject::tr("access denied error, retrieving config descriptor, usb err: %1.\nTry changing usb driver for camera divce using zadig (http://zadig.akeo.ie/)").arg(ret).toLatin1().data());
            else
                retval += ito::RetVal(ito::retError, 0, QObject::tr("Error retrieving config descriptor, usb err: %1").arg(ret).toLatin1().data());
        }
        uint8_t iface = config->interface->altsetting->bInterfaceNumber;
        uint8_t cfg = config->bConfigurationValue;
        libusb_free_config_descriptor(config);

        if (libusb_detach_kernel_driver(device_handle, iface) != 0)
        {
            //retval += ito::RetVal(ito::retWarning, 0, QObject::tr("could not detach kernel driver").toLatin1().data());
        }

        if (config)
        {
            if (ret = libusb_set_configuration(device_handle, cfg))
            {
                if (ret == LIBUSB_ERROR_ACCESS)
                    retval += ito::RetVal(ito::retWarning, 9999, QObject::tr("access denied error, setting device configuration, usb err: %1.\nTry changing usb driver for camera divce using zadig (http://zadig.akeo.ie/)").arg(ret).toLatin1().data());
                else
                    retval += ito::RetVal(ito::retWarning, 0, QObject::tr("error setting device configuration, usb err: %1").arg(ret).toLatin1().data());
            }

            if (ret = libusb_claim_interface(device_handle, iface))
            {
                if (ret == LIBUSB_ERROR_ACCESS)
                    retval += ito::RetVal(ito::retError, 9999, QObject::tr("access denied error, claiming interface, usb err: %1.\nTry changing usb driver for camera divce using zadig (http://zadig.akeo.ie/)").arg(ret).toLatin1().data());
                else
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("Error claiming interface, usb err: %1").arg(ret).toLatin1().data());
            }
        }
    }
    else
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Error opening usb device ptp_usb").toLatin1().data());
    }
    m_globalparams = params;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::clear_stall(PTP_USB* ptp_usb)
{
    ito::RetVal retval;
    uint16_t status = 0;
//    int ret;

    // check the inep status
    retval += usb_get_endpoint_status(ptp_usb, ptp_usb->inep, &status);
    if (retval.containsError())
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("inep: usb_get_endpoint_status()").toLatin1().data());
        perror("inep: usb_get_endpoint_status()");
    }
    // and clear the HALT condition if happend
    else if (status)
    {
        // printf("Resetting input pipe!\n");
        retval = usb_clear_stall_feature(ptp_usb, ptp_usb->inep);
        // usb_clear_halt(ptp_usb->handle,ptp_usb->inep);
        if (retval.containsError())
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_clear_stall_feature()").toLatin1().data());
            perror("usb_clear_stall_feature()");
        }
    }
    status = 0;

    // check the outep status
    retval += usb_get_endpoint_status(ptp_usb, ptp_usb->outep, &status);
    if (retval.containsError())
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("outep: usb_get_endpoint_status()").toLatin1().data());
        perror("outep: usb_get_endpoint_status()");
    }
    // and clear the HALT condition if happend
    else if (status)
    {
        //printf("Resetting output pipe!\n");
        retval += usb_clear_stall_feature(ptp_usb, ptp_usb->outep);
        // usb_clear_halt(ptp_usb->handle,ptp_usb->outep);
        if (retval.containsError())
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_clear_stall_feature()").toLatin1().data());
            perror("usb_clear_stall_feature()");
        }
    }

    // usb_clear_halt(ptp_usb->handle,ptp_usb->intep);
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::close_usb(PTP_USB* ptp_usb, struct libusb_device* dev)
{
    ito::RetVal retval;

    //clear_stall(ptp_usb);
    if (!ptp_usb || !dev)
        return ito::retOk;

    struct libusb_config_descriptor *config = NULL;
    libusb_get_config_descriptor(dev, 0, &config);
    if (!config)
        return ito::RetVal(ito::retError, 0, QObject::tr("Could not read configuration, close_usb").toLatin1().data());

    if (libusb_release_interface(ptp_usb->handle,
        config->interface->altsetting->bInterfaceNumber))
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Error closing usb interface").toLatin1().data());
    libusb_free_config_descriptor(config);
    libusb_reset_device(ptp_usb->handle);
    libusb_close(ptp_usb->handle);
//    retval += usb_release_interface(ptp_usb->handle,
//        dev->config->interface->altsetting->bInterfaceNumber);
//    retval += usb_reset(ptp_usb->handle);
//    retval += usb_close(ptp_usb->handle);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::init_usb(struct libusb_device ***devlist, int &cnt)
{
    if (libusb_init(NULL))
        return ito::RetVal(ito::retError, 0, QObject::tr("Error initializing usb").toLatin1().data());
    cnt = libusb_get_device_list(NULL, devlist);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
*  find_device() returns the pointer to a usb_device structure matching
*  given busn, devicen numbers. If any or both of arguments are 0 then the
*  first matching PTP device structure is returned.
*/
//struct libusb_device* find_device(int busn, int devicen, short force);
struct libusb_device* PtpCam::find_device(int portnum, short force)
{
    ito::RetVal retval;
    libusb_device *foundDev = NULL;
    libusb_device **devlist = NULL, *dev = NULL;
    int devcnt, i = 0;

    retval += init_usb(&devlist, devcnt);
    if (retval.containsError())
        goto end;

    while ((dev = devlist[i++]) != NULL)
    {
        struct libusb_config_descriptor *config = NULL;
        if (!libusb_get_config_descriptor(dev, 0, &config) && config)
        {
            if ((config->interface->altsetting->bInterfaceClass ==
                USB_CLASS_PTP) || force)
            {
                struct libusb_device_descriptor desc;
                if (!libusb_get_device_descriptor(dev, &desc))
                {
                    if (desc.bDeviceClass != LIBUSB_CLASS_HUB)
                    {
                        //int curbusn, curdevn;

                        //curbusn = strtol(bus->dirname, NULL, 10);
                        //curdevn = strtol(dev->filename, NULL, 10);
                        int port = libusb_get_port_number(dev);

                        if (port == portnum)
                        {
                            libusb_ref_device(dev);
                            foundDev = dev;
                            goto end;
                        }
                    }
                }
            }
        }
        libusb_free_config_descriptor(config);
    }

end:
    libusb_free_device_list(devlist, 1);
    return foundDev;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void find_endpoints(struct libusb_device *dev, int* inep, int* outep, int* intep);
ito::RetVal PtpCam::find_endpoints(struct libusb_device *dev, int* inep, int* outep, int* intep)
{
    ito::RetVal retval;
    int i, n;
    const struct libusb_endpoint_descriptor *ep = NULL;
    struct libusb_config_descriptor *config = NULL;

    libusb_get_config_descriptor(dev, 0, &config);
    if (!config)
        return ito::RetVal(ito::retWarning, 0, QObject::tr("could not load device config descriptor").toLatin1().data());

    ep = config->interface->altsetting->endpoint;
    n = config->interface->altsetting->bNumEndpoints;

    for (i = 0; i<n; i++)
    {
        if (ep[i].bmAttributes == LIBUSB_TRANSFER_TYPE_BULK)
        {
            if ((ep[i].bEndpointAddress&LIBUSB_ENDPOINT_IN) ==
                LIBUSB_ENDPOINT_IN)
            {
                *inep = ep[i].bEndpointAddress;
                if (m_verbose > 1)
                {
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("Found inep: 0x%1").arg(QString::number(*inep)).toLatin1().data());
                    //fprintf(stderr, "Found inep: 0x%02x\n", *inep);
                }
            }
            if ((ep[i].bEndpointAddress&LIBUSB_ENDPOINT_OUT) == LIBUSB_ENDPOINT_OUT)
            {
                *outep = ep[i].bEndpointAddress;
                if (m_verbose > 1)
                {
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("Found outep: 0x%1").arg(QString::number(*outep)).toLatin1().data());
                    //fprintf(stderr, "Found outep: 0x%02x\n", *outep);
                }
            }
        }
        else if ((ep[i].bmAttributes == LIBUSB_TRANSFER_TYPE_INTERRUPT) &&
            ((ep[i].bEndpointAddress&LIBUSB_ENDPOINT_IN) ==
            LIBUSB_ENDPOINT_IN))
        {
            *intep = ep[i].bEndpointAddress;
            if (m_verbose > 1)
            {
                retval += ito::RetVal(ito::retError, 0, QObject::tr("Found intep: 0x%1").arg(QString::number(*intep)).toLatin1().data());
                //fprintf(stderr, "Found intep: 0x%02x\n", *intep);
            }
        }
    }
    libusb_free_config_descriptor(config);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::open_camera(int portnum, short force, PTP_USB *ptp_usb, PTPParams *params, struct libusb_device **dev)
{
    ito::RetVal retval;
#ifdef DEBUG
    //printf("dev %i\tbus %i\n", devn, busn);
    qDebug("port %1").arg(QString::number(portnum));
#endif

    *dev = find_device(portnum, force);
    if (*dev == NULL)
    {
        fprintf(stderr, "could not find any device matching given bus/dev numbers");
        return ito::RetVal(ito::retError, 0, QObject::tr("could not find any device matching given bus/dev numbers").toLatin1().data());
        //exit(-1);
    }
    retval += find_endpoints(*dev, &ptp_usb->inep, &ptp_usb->outep, &ptp_usb->intep);

    retval += init_ptp_usb(params, ptp_usb, *dev);
    if (ptp_opensession(params, 1) != PTP_RC_OK)
    {
        fprintf(stderr, "ERROR: Could not open session!");
        retval += close_usb(ptp_usb, *dev);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not open session!").toLatin1().data());
        return retval;
        // return -1;
    }
    if (ptp_getdeviceinfo(params, &params->deviceinfo) != PTP_RC_OK) {
        fprintf(stderr, "ERROR: Could not get device info!");
        retval += close_usb(ptp_usb, *dev);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not get device info!").toLatin1().data());
        // return -1;
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::close_camera(PTP_USB *ptp_usb, PTPParams *params, struct libusb_device *dev)
{
    ito::RetVal retval;

    if (ptp_closesession(params) != PTP_RC_OK)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not close session!").toLatin1().data());
        //fprintf(stderr, "ERROR: Could not close session!");
    }
    retval += close_usb(ptp_usb, dev);
    libusb_unref_device(dev);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::list_devices(short force, QMap<int, QString> &deviceList)
{
    ito::RetVal retval;
    struct libusb_device **devs = NULL, *dev = NULL;
    int found = 0, cnt = 0, i = 0;

    retval += init_usb(&devs, cnt);
    if (retval.containsError())
        goto end;

    while ((dev = devs[i++]) != NULL)
    {
        // if it's a PTP device try to talk to it
        struct libusb_config_descriptor *config = NULL;
        if (!libusb_get_config_descriptor(dev, 0, &config) && config)
        //if (dev->config)
        {
            if ((config->interface->altsetting->bInterfaceClass ==
          //  if ((dev->config->interface->altsetting->bInterfaceClass ==
                USB_CLASS_PTP) || force)
            {
                struct libusb_device_descriptor desc;
                if (!libusb_get_device_descriptor(dev, &desc))
                {
                    if (desc.bDeviceClass != LIBUSB_DT_HUB)
                    {
                        PTPParams params;
                        PTP_USB ptp_usb;
                        PTPDeviceInfo deviceinfo;

                        if (!found)
                        {
                            // printf("\nListing devices...\n");
                            // printf("bus/dev\tvendorID/prodID\tdevice model\n");
                            found = 1;
                        }

                        retval += find_endpoints(dev, &ptp_usb.inep, &ptp_usb.outep,
                            &ptp_usb.intep);
                        retval += init_ptp_usb(&params, &ptp_usb, dev);

                        // CC(ptp_opensession(&params, 1),
                        //    "Could not open session!\n"
                        //    "Try to reset the camera.\n");
                        if (ptp_opensession(&params, 1) != PTP_RC_OK)
                        {
                            retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not open session! Try to reset the camera.").toLatin1().data());
                        }

                        // CC(ptp_getdeviceinfo(&params, &deviceinfo),
                        // "Could not get device info!\n");

                        //                    deviceList.append(QString("%1/%2\t0x%3/0x%4\t%5").arg(bus->dirname, dev->filename,
                        //                        dev->discriptor.idVendor, dev->descriptor.idProduct, deviceinfo.Model);
                        if (ptp_getdeviceinfo(&params, &deviceinfo) == PTP_RC_OK)
                        {
                            int port = libusb_get_port_number(dev);
                            deviceList.insert(port, QString("%1\t0x%2/0x%3\t%4").arg(QString::number(port),
                                QString::number(desc.idVendor), QString::number(desc.idProduct), QString(deviceinfo.Model)));
                            //                        printf("%s/%s\t0x%04X/0x%04X\t%s\n",
                            //                            bus->dirname, dev->filename,
                            //                            dev->descriptor.idVendor,
                            //                            dev->descriptor.idProduct, deviceinfo.Model);

                            //                        CC(ptp_closesession(&params),
                            //                            "Could not close session!\n");
                        }

                        if (ptp_closesession(&params) != PTP_RC_OK)
                        {
                            retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not close session!").toLatin1().data());
                        }
                        retval += close_usb(&ptp_usb, dev);
                    }
                }
            }
        }
        libusb_free_config_descriptor(config);
    }
    if (!found) printf("\nFound no PTP devices\n");
        printf("\n");

end:
    libusb_free_device_list(devs, 1);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::show_info(int portnum, short force, QString &info)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;

    info = QString();
    info.append("Camera information\n");
    info.append("==================\n");

    // printf("\nCamera information\n");
    // printf("==================\n");
    // if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    //    return;
    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;

    info.append("Model: %1\n").arg(params.deviceinfo.Model);
    info.append("  Manufacturer: %1\n").arg(params.deviceinfo.Manufacturer);
    info.append("  Serial Number: %1\n").arg(params.deviceinfo.SerialNumber);
    info.append("  Device Version: %1\n").arg(params.deviceinfo.DeviceVersion);
    info.append("  Extension ID: 0x%1\n").arg(params.deviceinfo.VendorExtensionID);
    info.append("  Extension Description: %1\n").arg(params.deviceinfo.VendorExtensionDesc);
    info.append("  Extension Version: %1\n").arg(params.deviceinfo.VendorExtensionVersion);
    // printf("Model: %s\n", params.deviceinfo.Model);
    // printf("  manufacturer: %s\n", params.deviceinfo.Manufacturer);
    // printf("  serial number: '%s'\n", params.deviceinfo.SerialNumber);
    // printf("  device version: %s\n", params.deviceinfo.DeviceVersion);
    // printf("  extension ID: 0x%08lx\n", (long unsigned)
    //    params.deviceinfo.VendorExtensionID);
    // printf("  extension description: %s\n",
    //    params.deviceinfo.VendorExtensionDesc);
    // printf("  extension version: 0x%04x\n",
    //    params.deviceinfo.VendorExtensionVersion);
    //printf("\n");
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::capture_image(int portnum, short force)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    PTPContainer event;
    int ExposureTime = 0;
    short ret;

    // printf("\nInitiating captue...\n");
    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    //if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    if (retval.containsError())
        return retval;

    if (!ptp_operation_issupported(&params, PTP_OC_InitiateCapture))
    {
        // printf("Your camera does not support InitiateCapture operation!\nSorry, blame the %s!\n", params.deviceinfo.Manufacturer);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Your camera does not support InitiateCapture operation! Sorry, blame the %1!").arg(params.deviceinfo.Manufacturer).toLatin1().data());
        goto out;
    }

    // obtain exposure time in miliseconds
    if (ptp_property_issupported(&params, PTP_DPC_ExposureTime))
    {
        PTPDevicePropDesc dpd;
        memset(&dpd, 0, sizeof(dpd));
        ret = ptp_getdevicepropdesc(&params, PTP_DPC_ExposureTime, &dpd);
        if (ret == PTP_RC_OK)
            ExposureTime = (*(int32_t*)(dpd.CurrentValue)) / 10;
        else
            retval += ito::RetVal(ito::retWarning, 0, QObject::tr("Could not read exposure time.").toLatin1().data());
    }

    // adjust USB timeout
    if (ExposureTime > USB_TIMEOUT) m_ptpcam_usb_timeout = ExposureTime;

    //CR(ptp_initiatecapture(&params, 0x0, 0), "Could not capture.\n");
    if (ret = ptp_initiatecapture(&params, 0x0, 0) != PTP_RC_OK)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not capture.").toLatin1().data());
    /*
    // wait for event is currently not working
    ret = ptp_usb_event_wait(&params, &event);
    if (ret != PTP_RC_OK)
        goto err;

    if (m_verbose)
#ifdef WIN32
        std::cout << printf("Event received %08x, ret=%x\n", event.Code, ret);
#else
        stdout << printf("Event received %08x, ret=%x\n", event.Code, ret);
#endif
    if (event.Code == PTP_EC_CaptureComplete)
    {
        perror("Camera reported 'capture completed' but the object information is missing.\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Camera reported 'capture completed' but the object information is missing.").toLatin1().data());
        goto out;
    }

    while (event.Code == PTP_EC_ObjectAdded)
    {
        // printf("Object added 0x%08lx\n", (long unsigned)event.Param1);
        if (ptp_usb_event_wait(&params, &event) != PTP_RC_OK)
            goto err;

        if (m_verbose)
#ifdef WIN32
            std::cout << printf("Event received %08x, ret=%x\n", event.Code, ret);
#else
            stdout << printf("Event received %08x, ret=%x\n", event.Code, ret);
#endif
        if (event.Code == PTP_EC_CaptureComplete)
        {
            // printf("Capture completed successfully!\n");
            goto out;
        }
    }
    */

err:
    // printf("Events receiving error. Capture status unknown.\n");
out:

    m_ptpcam_usb_timeout = USB_TIMEOUT;
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PtpCam::sig_alrm(int signo)
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::loop_capture(int portnum, short force, int n, int interval, int overwrite)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    PTPContainer event;
#ifdef WIN32
    HANDLE fh;
    HANDLE fmh;
#else
    int file;
#endif
    PTPObjectInfo oi;
    uint32_t handle = 0;
    char *image;
    int ret;
    char *filename;
    time_t start_time;
    time_t rest_time;

    // Catch the SIGALARM
    //signal(SIGALRM, sig_alrm);

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
//    if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    if (retval.containsError())
        return retval;

    // capture timeout should be longe
    m_ptpcam_usb_timeout = USB_CAPTURE_TIMEOUT;

    // printf("Camera: %s\n", params.deviceinfo.Model);

    // local loop
    while (n > 0)
    {
        // capture
        time(&start_time);
        //printf("\nInitiating captue...\n");
        //CR(ptp_initiatecapture(&params, 0x0, 0), "Could not capture\n");
        retval += ptp_initiatecapture(&params, 0x0, 0);
        n--;

        ret = ptp_usb_event_wait(&params, &event);

        if (m_verbose)
            std::cout << printf("Event received %08x, ret=%x\n", event.Code, ret);
        if (ret != PTP_RC_OK) goto err;
        if (event.Code == PTP_EC_CaptureComplete)
        {
            // printf("CANNOT DOWNLOAD: got 'capture completed' but the object information is missing.\n");
            retval += ito::RetVal(ito::retError, 0, QObject::tr("CANNOT DOWNLOAD: got 'capture completed' but the object information is missing.").toLatin1().data());
            goto out;
        }

        while (event.Code == PTP_EC_ObjectAdded)
        {
            // printf("Object added 0x%08lx\n", (long unsigned)event.Param1);
            handle = event.Param1;
            if (ptp_usb_event_wait(&params, &event) != PTP_RC_OK)
                goto err;
            if (m_verbose)
                std::cout << printf("Event received %08x, ret=%x\n", event.Code, ret);
            if (event.Code == PTP_EC_CaptureComplete)
                goto download;
        }

    download:
        memset(&oi, 0, sizeof(PTPObjectInfo));
        if (m_verbose)
            std::cout << printf("Downloading: 0x%08lx\n", (long unsigned)handle);

        if ((ret = ptp_getobjectinfo(&params, handle, &oi)) != PTP_RC_OK)
        {
            fprintf(stderr, "ERROR: Could not get object info\n");
            ptp_perror(&params, ret);
            if (ret == PTP_ERROR_IO)
            {
                retval += clear_stall(&ptp_usb);
            }
            continue;
        }

        if (oi.ObjectFormat == PTP_OFC_Association)
            goto out;
        filename = (oi.Filename);
#ifdef WIN32
        fh = CreateFileA(filename, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (!fh)
#else
        file = open(filename, (overwrite == 1 ? 0 : O_EXCL) | O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRGRP);
        if (file == -1)
#endif
        {
            if (errno == EEXIST)
            {
                std::cout << printf("Skipping file: \"%s\", file exists!\n", filename);
                retval += ito::RetVal(ito::retError, 0, QObject::tr("Skipping file: \"%s\", file exists!").arg(filename).toLatin1().data());
                goto out;
            }
            perror("open");
            goto out;
        }
#ifdef WIN32
        SetFilePointer(fh, oi.ObjectCompressedSize - 1, NULL, FILE_BEGIN);
        ret = WriteFile(fh, "", 1, 0, 0);
#else
        lseek(file, oi.ObjectCompressedSize - 1, SEEK_SET);
        ret = write(file, "", 1);
#endif
        if (ret == -1)
        {
            perror("write");
            retval += ito::RetVal(ito::retError, 0, QObject::tr("Error writing file!").toLatin1().data());
            goto out;
        }
#ifdef WIN32
        fmh = CreateFileMappingA(fh, NULL, PAGE_READWRITE, 0, 0, filename);
        image = (char*)MapViewOfFile(fmh, FILE_MAP_ALL_ACCESS, 0, 0, 0);
        if (!fh || !fmh || !image)
        {
            UnmapViewOfFile((LPCVOID)image);
            CloseHandle(fmh);
            CloseHandle(fh);
            retval += ito::RetVal(ito::retError, 0, QObject::tr("Error mmap!").toLatin1().data());
            goto out;
        }
#else
        image = (char*)mmap(0, oi.ObjectCompressedSize, PROT_READ | PROT_WRITE, MAP_SHARED,
            file, 0);
        if (image == MAP_FAILED)
        {
            perror("mmap");
            retval += ito::RetVal(ito::retError, 0, QObject::tr("Error mmap!").toLatin1().data());
            close(file);
            goto out;
        }
#endif
        // printf("Saving file: \"%s\" ", filename);
        fflush(NULL);
        ret = ptp_getobject(&params, handle, &image);

#ifdef WIN32
        UnmapViewOfFile((LPCVOID)image);
        CloseHandle(fmh);
        CloseHandle(fh);
#else
        munmap(image, oi.ObjectCompressedSize);
        close(file);
#endif
        if (ret != PTP_RC_OK)
        {
            printf("error!\n");
            ptp_perror(&params, ret);
            if (ret == PTP_ERROR_IO)
                retval += clear_stall(&ptp_usb);
        }
        else
        {
            // and delete from camera!
            // printf("is done...\nDeleting from camera.\n");
            // CR(ptp_deleteobject(&params, handle, 0),
            //    "Could not delete object\n");
            retval += ptp_deleteobject(&params, handle, 0);
            // printf("Object 0x%08lx (%s) deleted.\n", (long unsigned)handle, oi.Filename);
        }
    out:
        rest_time = interval - (time(NULL) - start_time);
        if (rest_time>0 && n>0)
        {
            // printf("Sleeping for remaining %u seconds.\n", rest_time);
#ifdef WIN32
            Sleep(rest_time);
#else
            alarm(rest_time);
            pause();
#endif
        }
    }
err:

    m_ptpcam_usb_timeout = USB_TIMEOUT;
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::nikon_initiate_dc(int portnum, short force)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    uint16_t result;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;

    // printf("Camera: %s\n", params.deviceinfo.Model);
    // printf("\nInitiating direct captue...\n");

    if (params.deviceinfo.VendorExtensionID != PTP_VENDOR_NIKON)
    {
        // printf("Your camera is not Nikon!\nDo not buy from %s!\n", params.deviceinfo.Manufacturer);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Your camera is not Nikon! Do not buy from %1!").arg(params.deviceinfo.Manufacturer).toLatin1().data());
        goto out;
    }

    if (!ptp_operation_issupported(&params, PTP_OC_NIKON_DirectCapture))
    {
        // printf("Sorry, your camera dows not support Nikon DirectCapture!\nDo not buy from %s!\n", params.deviceinfo.Manufacturer);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Sorry, your camera does not support Nikon DirectCapture! Do not buy from %1!").arg(params.deviceinfo.Manufacturer).toLatin1().data());
        goto out;
    }

    // perform direct capture
    result = ptp_nikon_directcapture(&params, 0xffffffff);
    if (result != PTP_RC_OK)
    {
        ptp_perror(&params, result);
        // fprintf(stderr, "ERROR: Could not capture.\n");
        if (result != PTP_RC_StoreFull)
        {
            retval += close_camera(&ptp_usb, &params, m_pdev);
            m_pdev = NULL;
            return retval;
        }
    }
    //usleep(300 * 1000);
    Sleep(300);

out:
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::nikon_direct_capture(int portnum, short force, char* filename, int overwrite)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    uint16_t result;
    uint16_t nevent = 0;
    PTPUSBEventContainer* events = NULL;
    int ExposureTime = 0;	// exposure time in miliseconds
    int BurstNumber = 1;
    PTPDevicePropDesc dpd;
    PTPObjectInfo oi;
    int i;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;

    // printf("Camera: %s\n", params.deviceinfo.Model);

    if ((result = ptp_getobjectinfo(&params, 0xffff0001, &oi)) == PTP_RC_OK)
    {
        if (filename == NULL) filename = oi.Filename;
        save_object(&params, 0xffff0001, filename, oi, overwrite);
        goto out;
    }

    // printf("\nInitiating direct captue...\n");

    if (params.deviceinfo.VendorExtensionID != PTP_VENDOR_NIKON)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Your camera is not Nikon! Do not buy from %1!").arg(params.deviceinfo.Manufacturer).toLatin1().data());
        // printf("Your camera is not Nikon!\nDo not buy from %s!\n", params.deviceinfo.Manufacturer);
        goto out;
    }

    if (!ptp_operation_issupported(&params, PTP_OC_NIKON_DirectCapture))
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Sorry, your camera dows not support Nikon DirectCapture! Do not buy from %1!").arg(params.deviceinfo.Manufacturer).toLatin1().data());
        // printf("Sorry, your camera dows not support Nikon DirectCapture!\nDo not buy from %s!\n", params.deviceinfo.Manufacturer);
        goto out;
    }

    // obtain exposure time in miliseconds
    memset(&dpd, 0, sizeof(dpd));
    result = ptp_getdevicepropdesc(&params, PTP_DPC_ExposureTime, &dpd);
    if (result == PTP_RC_OK) ExposureTime = (*(int32_t*)(dpd.CurrentValue)) / 10;

    // obtain burst number
    memset(&dpd, 0, sizeof(dpd));
    result = ptp_getdevicepropdesc(&params, PTP_DPC_BurstNumber, &dpd);
    if (result == PTP_RC_OK) BurstNumber = *(uint16_t*)(dpd.CurrentValue);
    /*
    if ((result=ptp_getobjectinfo(&params,0xffff0001, &oi))==PTP_RC_OK)
    {
    if (filename==NULL) filename=oi.Filename;
    save_object(&params, 0xffff0001, filename, oi, overwrite);
    ptp_nikon_keepalive(&params);
    ptp_nikon_keepalive(&params);
    ptp_nikon_keepalive(&params);
    ptp_nikon_keepalive(&params);
    }
    */

    // perform direct capture
    result = ptp_nikon_directcapture(&params, 0xffffffff);
    if (result != PTP_RC_OK)
    {
        ptp_perror(&params, result);
        // fprintf(stderr, "ERROR: Could not capture.\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not capture.").toLatin1().data());
        if (result != PTP_RC_StoreFull)
        {
            retval += close_camera(&ptp_usb, &params, m_pdev);
            m_pdev = NULL;
            return retval;
        }
    }
//    if (BurstNumber>1)
//        printf("Capturing %i frames in burst.\n", BurstNumber);

    // sleep in case of exposure longer than 1/100
    //if (ExposureTime>10) {
//    printf("sleeping %i miliseconds\n", 500 + ExposureTime);
    //usleep(ExposureTime * 1000 + 500000);
    Sleep(ExposureTime + 500);
    //}

    while (BurstNumber>0)
    {

#if 0
        // Is this really needed???
        ptp_nikon_keepalive(&params);
#endif

        result = ptp_nikon_checkevent(&params, &events, &nevent);
        if (result != PTP_RC_OK)
        {
            fprintf(stderr, "Error checking Nikon events\n");
            ptp_perror(&params, result);
            goto out;
        }
        for (i = 0; i<nevent; i++)
        {
            ptp_nikon_keepalive(&params);
            void *prop;
            if (events[i].code == PTP_EC_DevicePropChanged)
            {
                // printf("Checking: %s\n", ptp_prop_getname(&params, events[i].param1));
                ptp_getdevicepropvalue(&params, events[i].param1, &prop, PTP_DTC_UINT64);
            }

            // printf("Event [%i] = 0x%04x,\t param: %08x\n", i, events[i].code, events[i].param1);
            if (events[i].code == PTP_EC_NIKON_CaptureOverflow)
            {
                // printf("Ram cache overflow? Shooting to fast!\n");
                if ((result = ptp_getobjectinfo(&params, 0xffff0001, &oi)) != PTP_RC_OK)
                {
                    // fprintf(stderr, "Could not get object info\n");
                    ptp_perror(&params, result);
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object info").toLatin1().data());
                    goto out;
                }
                if (filename == NULL) filename = oi.Filename;
                retval += save_object(&params, 0xffff0001, filename, oi, overwrite);
                BurstNumber = 0;
                //usleep(100);
                Sleep(1);
            }
            else
                if (events[i].code == PTP_EC_NIKON_ObjectReady)
                {
                    if ((result = ptp_getobjectinfo(&params, 0xffff0001, &oi)) != PTP_RC_OK)
                    {
                        //fprintf(stderr, "Could not get object info\n");
                        ptp_perror(&params, result);
                        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object info").toLatin1().data());
                        goto out;
                    }
                    if (filename == NULL) filename = oi.Filename;
                    save_object(&params, 0xffff0001, filename, oi, overwrite);
                    BurstNumber--;
                }
        }
        free(events);
    }

out:
    m_ptpcam_usb_timeout = USB_TIMEOUT;
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::nikon_direct_capture2(int portnum, short force, char* filename, int overwrite)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    uint16_t result;
    PTPObjectInfo oi;

    m_pdev = find_device(portnum, force);
    if (m_pdev == NULL)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("could not find any device matching given bus/dev numbers").toLatin1().data());
//        fprintf(stderr, "could not find any device matching given "
//            "bus/dev numbers\n");
//        exit(-1);
        return retval;
    }
    retval += find_endpoints(m_pdev, &ptp_usb.inep, &ptp_usb.outep, &ptp_usb.intep);

    retval += init_ptp_usb(&params, &ptp_usb, m_pdev);

    if (ptp_opensession(&params, 1) != PTP_RC_OK)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not open session!").toLatin1().data());
        //fprintf(stderr, "ERROR: Could not open session!\n");
        retval += close_usb(&ptp_usb, m_pdev);
        m_pdev = NULL;
        return retval;
    }
    /*
    memset(&dpd,0,sizeof(dpd));
    result=ptp_getdevicepropdesc(&params,PTP_DPC_BurstNumber,&dpd);
    memset(&dpd,0,sizeof(dpd));
    result=ptp_getdevicepropdesc(&params,PTP_DPC_ExposureTime,&dpd);
    */

    // perform direct capture
    result = ptp_nikon_directcapture(&params, 0xffffffff);
    if (result != PTP_RC_OK)
    {
        ptp_perror(&params, result);
        // fprintf(stderr, "ERROR: Could not capture.\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not capture.").toLatin1().data());
        if (result != PTP_RC_StoreFull)
        {
            retval += close_camera(&ptp_usb, &params, m_pdev);
            m_pdev = NULL;
            return retval;
        }
    }

    if (ptp_closesession(&params) != PTP_RC_OK)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not close session!").toLatin1().data());
        // fprintf(stderr, "ERROR: Could not close session!\n");
        return retval;
    }

    //usleep(300 * 1000);
    Sleep(300);

    if (ptp_opensession(&params, 1) != PTP_RC_OK)
    {
        // fprintf(stderr, "ERROR: Could not open session!\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not open session!").toLatin1().data());
        retval += close_usb(&ptp_usb, m_pdev);
        m_pdev = NULL;
        return retval;
    }
loop:
    if ((result = ptp_getobjectinfo(&params, 0xffff0001, &oi)) == PTP_RC_OK)
    {
        if (filename == NULL) filename = oi.Filename;
        retval += save_object(&params, 0xffff0001, filename, oi, overwrite);
    }
    else
    {
        ptp_nikon_keepalive(&params);
        goto loop;
    }

    // out:
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

#if 0
    PTPParams params;
    PTP_USB ptp_usb;
    struct libusb_device *dev;
    uint16_t result;
    uint16_t nevent = 0;
    PTPUSBEventContainer* events = NULL;
    int ExposureTime = 0;	// exposure time in miliseconds
    int BurstNumber = 1;
    PTPDevicePropDesc dpd;
    PTPObjectInfo oi;
    int i;
    char *filename = NULL;

    retval += open_camera(busn, devn, force, &ptp_usb, &params, &dev);
    if (retval.containsError())
        return retval;
//    if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
//        return;

    // printf("Camera: %s\n", params.deviceinfo.Model);
    // printf("\nInitiating direct captue...\n");

    if (params.deviceinfo.VendorExtensionID != PTP_VENDOR_NIKON)
    {
        // printf("Your camera is not Nikon!\nDo not buy from %s!\n", params.deviceinfo.Manufacturer);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Your camera is not Nikon! Do not buy from %1!").arg(params.deviceinfo.Manufacturer).toLatin1().data());
        goto out;
    }

    if (!ptp_operation_issupported(&params, PTP_OC_NIKON_DirectCapture))
    {
        // printf("Sorry, your camera dows not support Nikon DirectCapture!\nDo not buy from %s!\n", params.deviceinfo.Manufacturer);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Sorry, your camera dows not support Nikon DirectCapture! Do not buy from %1!").arg(params.deviceinfo.Manufacturer).toLatin1().data());
        goto out;
    }

    // obtain exposure time in miliseconds
    memset(&dpd, 0, sizeof(dpd));
    result = ptp_getdevicepropdesc(&params, PTP_DPC_ExposureTime, &dpd);
    if (result == PTP_RC_OK) ExposureTime = (*(int32_t*)(dpd.CurrentValue)) / 10;

    // obtain burst number
    memset(&dpd, 0, sizeof(dpd));
    result = ptp_getdevicepropdesc(&params, PTP_DPC_BurstNumber, &dpd);
    if (result == PTP_RC_OK) BurstNumber = *(uint16_t*)(dpd.CurrentValue);

//    if (BurstNumber > 1) printf("Capturing %i frames in burst.\n", BurstNumber);
#if 0
    /* sleep in case of exposure longer than 1/100 */
    if (ExposureTime > 10)
    {
        // printf("sleeping %i miliseconds\n", ExposureTime);
        //usleep(ExposureTime * 1000);
        Sleep(ExposureTime);
    }
#endif

    while (num>0)
    {
        // perform direct capture
        result = ptp_nikon_directcapture(&params, 0xffffffff);
        if (result != PTP_RC_OK)
        {
            if (result == PTP_ERROR_IO)
            {
                retval += close_camera(&ptp_usb, &params, dev);
                return retval;
            }
        }

#if 0
        // Is this really needed???
        ptp_nikon_keepalive(&params);
#endif
        ptp_nikon_keepalive(&params);
        ptp_nikon_keepalive(&params);
        ptp_nikon_keepalive(&params);
        ptp_nikon_keepalive(&params);

        result = ptp_nikon_checkevent(&params, &events, &nevent);
        if (result != PTP_RC_OK) goto out;

        for (i = 0; i<nevent; i++)
        {
            // printf("Event [%i] = 0x%04x,\t param: %08x\n", i, events[i].code, events[i].param1);
            if (events[i].code == PTP_EC_NIKON_ObjectReady)
            {
                num--;
                if ((result = ptp_getobjectinfo(&params, 0xffff0001, &oi)) != PTP_RC_OK)
                {
                    // fprintf(stderr, "Could not get object info\n");
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object info").toLatin1().data());
                    ptp_perror(&params, result);
                    goto out;
                }
                if (filename == NULL) filename = oi.Filename;
                retval += save_object(&params, 0xffff0001, filename, oi, overwrite);
            }
            if (events[i].code == PTP_EC_NIKON_CaptureOverflow)
            {
                // printf("Ram cache overflow, capture terminated\n");
                retval += ito::RetVal(ito::retError, 0, QObject::tr("Ram cache overflow, capture terminated").toLatin1().data());
                //BurstNumber=0;
            }
        }
        free(events);
    }

out:
    ptpcam_usb_timeout = USB_TIMEOUT;
    retval += close_camera(&ptp_usb, &params, dev);
#endif

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::get_last_file_handle(int portnum, short force, int &imgNum, uint32_t &handle, int &fileType, DslrRemote *parentHandle)
{
    ito::RetVal retval;
    PTP_USB ptp_usb;
    PTPParams params;
    PTPObjectInfo oiF, oiF1, oiL;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
    {
        return retval;
    }

    if (ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000, &params.handles) != PTP_RC_OK)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object handles").toLatin1().data());

    if (imgNum < 0 || imgNum >= params.handles.n)
    {
        int found = 0, lastF = params.handles.n - 1;
        handle = 0;
        while (!found && lastF >= 0)
        {
            if (parentHandle)
                parentHandle->setAlive();
            ptp_getobjectinfo(&params, params.handles.Handler[lastF], &oiL);
            if (oiL.ObjectFormat == 14337 || oiL.ObjectFormat == 12288)
            {
                break;
            }
            lastF--;
        }
        found = 0;
        int firstF = 0;
        ptp_getobjectinfo(&params, params.handles.Handler[firstF], &oiF);
        while (!found && firstF < params.handles.n - 1)
        {
            if (parentHandle)
                parentHandle->setAlive();
            ptp_getobjectinfo(&params, params.handles.Handler[firstF + 1], &oiF1);
            if (oiF.ObjectFormat == 14337 || oiF.ObjectFormat == 12288 && oiF.CaptureDate > oiF1.CaptureDate)
            {
                break;
            }
            oiF = oiF1;
            firstF++;
        }
        if (oiF.CaptureDate > oiL.CaptureDate)
        {
            handle = params.handles.Handler[firstF];
            fileType = oiF.ObjectFormat;
            imgNum = firstF;
        }
        else
        {
            handle = params.handles.Handler[lastF];
            fileType = oiL.ObjectFormat;
            imgNum = lastF;
        }
    }
    else
    {
        handle = params.handles.Handler[imgNum];
        ptp_getobjectinfo(&params, params.handles.Handler[imgNum], &oiL);
        fileType = oiL.ObjectFormat;
    }
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::list_files(int portnum, short force, QVector<QString> &fileList, DslrRemote *parentHandle)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    int i;
    PTPObjectInfo oi;
    struct tm *tm;

    // printf("\nListing files...\n");
    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
    {
        return retval;
    }
    // if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    //    return;
    // printf("Camera: %s\n", params.deviceinfo.Model);
    if (ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000, &params.handles) != PTP_RC_OK)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object handles").toLatin1().data());

    // CR(ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000,
    //    &params.handles), "Could not get object handles\n");
    // printf("Handler:           Size: \tCaptured:      \tname:\n");
    for (i = 0; i < params.handles.n; i++)
    {
        parentHandle->setAlive();
        // CR(ptp_getobjectinfo(&params, params.handles.Handler[i],
        //    &oi), "Could not get object info\n");
        if (ptp_getobjectinfo(&params, params.handles.Handler[i], &oi) != PTP_RC_OK)
            retval += ito::RetVal(ito::retWarning, 0, QObject::tr("Could not get object info").toLatin1().data());
        if (oi.ObjectFormat == PTP_OFC_Association)
            continue;
        tm = gmtime(&oi.CaptureDate);
        fileList.append(QString("0x%1 %2 %3/%4/%5 %6:%7 %8;").arg(QString::number(params.handles.Handler[i]),
            QString::number(oi.ObjectCompressedSize), QString::number(tm->tm_year + 1900),
            QString::number(tm->tm_mon + 1), QString::number(tm->tm_mday),
            QString::number(tm->tm_hour), QString::number(tm->tm_min), QString(oi.Filename)));
//        printf("0x%08lx: %12u\t%4i-%02i-%02i %02i:%02i\t%s\n",
//            (long unsigned)params.handles.Handler[i],
//            (unsigned)oi.ObjectCompressedSize,
//            tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
//            tm->tm_hour, tm->tm_min,
//            oi.Filename);
    }
    // printf("\n");
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::delete_object(int portnum, short force, uint32_t handle)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    PTPObjectInfo oi;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
//    if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
//        return;
    if (ptp_getobjectinfo(&params, handle, &oi))
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object info").toLatin1().data());
//    CR(ptp_getobjectinfo(&params, handle, &oi),
//        "Could not get object info\n");
    if (ptp_deleteobject(&params, handle, 0))
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not delete object").toLatin1().data());
//    CR(ptp_deleteobject(&params, handle, 0), "Could not delete object\n");
//    printf("\nObject 0x%08lx (%s) deleted.\n", (long unsigned)handle, oi.Filename);
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::delete_all_files(int portnum, short force)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    PTPObjectInfo oi;
    uint32_t handle;
    int i;
    int ret;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
//    if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
//        return;
    // printf("Camera: %s\n", params.deviceinfo.Model);
    if (ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000, &params.handles))
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object handles").toLatin1().data());
    // CR(ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000,
    //    &params.handles), "Could not get object handles\n");

    for (i = 0; i < params.handles.n; i++)
    {
        handle = params.handles.Handler[i];
        if ((ret = ptp_getobjectinfo(&params, handle, &oi)) != PTP_RC_OK)
        {
            // fprintf(stderr, "Handle: 0x%08lx\n", (long unsigned)handle);
            // fprintf(stderr, "ERROR: Could not get object info\n");
            ptp_perror(&params, ret);
            retval += ito::RetVal(ito::retError, 0, QObject::tr("Handle: 0x %1 ERROR: Could not get object info").arg(handle).toLatin1().data());
            if (ret == PTP_ERROR_IO) clear_stall(&ptp_usb);
            continue;
        }
        if (oi.ObjectFormat == PTP_OFC_Association)
            continue;
        if (ptp_deleteobject(&params, handle, 0))
        {
            retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not delete object").arg(handle).toLatin1().data());
        }
        // CR(ptp_deleteobject(&params, handle, 0),
        //    "Could not delete object\n");
        // printf("Object 0x%08lx (%s) deleted.\n", (long unsigned)handle, oi.Filename);
    }
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::save_object(PTPParams *params, uint32_t handle, char* filename, PTPObjectInfo oi, int overwrite)
{
    ito::RetVal retval;
    char *image;
    int ret;

#ifdef WIN32
    utimbuf timebuf;
    HANDLE fh;
    HANDLE fmh;
    fh = CreateFileA(filename, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (!fh)
#else
    struct utimbuf timebuf;
    int file;
    file = open(filename, (overwrite == 1 ? 0 : O_EXCL) | O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRGRP);
    if (file == -1)
#endif
    {
        if (errno == EEXIST)
        {
            // printf("Skipping file: \"%s\", file exists!\n", filename);
            retval += ito::RetVal(ito::retError, 0, QObject::tr("Skipping file: \"%1\", file exists!").arg(filename).toLatin1().data());
            goto out;
        }
        perror("open");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("open").toLatin1().data());
        goto out;
    }
#ifdef WIN32
    SetFilePointer(fh, oi.ObjectCompressedSize - 1, NULL, FILE_BEGIN);
    ret = WriteFile(fh, "", 1, 0, 0);
#else
    lseek(file, oi.ObjectCompressedSize - 1, SEEK_SET);
    ret = write(file, "", 1);
#endif
    if (ret == -1)
    {
        perror("write");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("write").toLatin1().data());
        goto out;
    }
#ifdef WIN32
    fmh = CreateFileMappingA(fh, NULL, PAGE_READWRITE, 0, 0, filename);
    image = (char*)MapViewOfFile(fmh, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (!fh || !fmh || !image)
    {
        UnmapViewOfFile((LPCVOID)image);
        CloseHandle(fmh);
        CloseHandle(fh);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Error mmap!").toLatin1().data());
        goto out;
    }
#else
    image = (char*)mmap(0, oi.ObjectCompressedSize, PROT_READ | PROT_WRITE, MAP_SHARED,
        file, 0);
    if (image == MAP_FAILED)
    {
        perror("mmap");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("mmap").toLatin1().data());
        close(file);
        goto out;
    }
#endif
    //printf("Saving file: \"%s\" ", filename);
    fflush(NULL);
    ret = ptp_getobject(params, handle, &image);

#ifdef WIN32
    UnmapViewOfFile((LPCVOID)image);
    CloseHandle(fmh);
    CloseHandle(fh);
#else
    munmap(image, oi.ObjectCompressedSize);
    if (close(file) == -1)
    {
        perror("close");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("close").toLatin1().data());
    }
#endif

    timebuf.actime = oi.ModificationDate;
    timebuf.modtime = oi.CaptureDate;
    utime(filename, &timebuf);
    if (ret != PTP_RC_OK)
    {
        // printf("error!\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("error").toLatin1().data());
        ptp_perror(params, ret);
        if (ret == PTP_ERROR_IO) clear_stall((PTP_USB *)(params->data));
    }
    else
    {
        // printf("is done.\n");
    }

out:
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::get_save_object(PTPParams *params, uint32_t handle, char* filename, int overwrite)
{
    ito::RetVal retval;
    PTPObjectInfo oi;
    int ret;

    memset(&oi, 0, sizeof(PTPObjectInfo));
    if (m_verbose)
        std::cout << printf("Handle: 0x%08lx\n", (long unsigned)handle);

    if ((ret = ptp_getobjectinfo(params, handle, &oi)) != PTP_RC_OK)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object info").toLatin1().data());
        // fprintf(stderr, "Could not get object info\n");
        ptp_perror(params, ret);
        if (ret == PTP_ERROR_IO) clear_stall((PTP_USB *)(params->data));
        goto out;
    }
    if (oi.ObjectFormat == PTP_OFC_Association)
        goto out;
    if (filename == NULL) filename = (oi.Filename);

    retval += save_object(params, handle, filename, oi, overwrite);

out:
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::get_filehandlebyname(int portnum, short force, char *camfilename, uint32_t &fhandle, uint32_t &ftype, DslrRemote *parentHandle)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    PTPObjectInfo oi;
    int i;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;

    if (ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000, &params.handles) != PTP_RC_OK)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object handles").toLatin1().data());

    // CR(ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000,
    //    &params.handles), "Could not get object handles\n");
    // printf("Handler:           Size: \tCaptured:      \tname:\n");
    for (i = 0; i < params.handles.n; i++)
    {
        parentHandle->setAlive();
        // CR(ptp_getobjectinfo(&params, params.handles.Handler[i],
        //    &oi), "Could not get object info\n");
        if (ptp_getobjectinfo(&params, params.handles.Handler[i], &oi) != PTP_RC_OK)
            retval += ito::RetVal(ito::retWarning, 0, QObject::tr("Could not get object info").toLatin1().data());
        if (oi.ObjectFormat == PTP_OFC_Association)
            continue;
#if WIN32
        if (stricmp(camfilename, oi.Filename) == 0)
#else
        if (strcasecmp(camfilename, oi.Filename) == 0)
#endif
            break;
    }

    if (i < params.handles.n)
    {
        fhandle = params.handles.Handler[i];
        ftype = oi.ObjectFormat;
    }

    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::get_file(int portnum, short force, uint32_t handle, char* filename, int overwrite)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
//    if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
//        return;
    // printf("Camera: %s\n", params.deviceinfo.Model);

    retval += get_save_object(&params, handle, filename, overwrite);

    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::get_all_files(int portnum, short force, int overwrite)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    int i;

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
    // if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    //    return;
    // printf("Camera: %s\n", params.deviceinfo.Model);

    if (ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000,
        &params.handles))
        retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get object handles").toLatin1().data());
//    CR(ptp_getobjecthandles(&params, 0xffffffff, 0x000000, 0x000000,
//        &params.handles), "Could not get object handles\n");

    for (i = 0; i<params.handles.n; i++)
    {
        retval += get_save_object(&params, params.handles.Handler[i], NULL,
            overwrite);
    }
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::send_generic_request(int portnum, uint16_t reqCode, uint32_t *reqParams, uint32_t direction,
    char *data_file)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    char *data = NULL;
    long fsize = 0;

    if (direction == PTP_DP_SENDDATA)
    {
        // read data from file
        if (strncmp(data_file, "0x", 2) == 0)
        {
            uint len = strlen(data_file);
            char num[3];
            uint i;
            data = (char*)calloc(1, len / 2);

            num[2] = 0;
            for (i = 2; i < len; i += 2)
            {
                num[1] = data_file[i];
                if (i < len - 1)
                {
                    num[0] = data_file[i];
                    num[1] = data_file[i + 1];
                }
                else
                {
                    num[0] = data_file[i];
                    num[1] = 0;
                }
                data[fsize] = (char)strtol(num, NULL, 16);
                ++fsize;
            }
        }
        else
        {

            FILE *f = fopen(data_file, "r");
            if (f)
            {
                fseek(f, 0, SEEK_END);
                fsize = ftell(f);
                fseek(f, 0, SEEK_SET);
                data = (char*)calloc(1, fsize + 1);
                if (fread(data, 1, fsize, f) != fsize)
                {
                    // fprintf(stderr, "PTP: ERROR: can't read data to send from file '%s'\n", data_file);
                    retval += ito::RetVal(ito::retError, 0, QObject::tr("PTP: ERROR: can't read data to send from file '%s'").arg(data_file).toLatin1().data());
                    free(data);
                    return retval;
                }
                else
                {
                    // printf("--- data to send ---\n");
                    // display_hexdump(data, fsize);
                    // printf("--------------------\n");
                }
            }
            else
            {
                // error no data to send
                // fprintf(stderr, "PTP: ERROR: file not found '%s'\n", data_file);
                retval += ito::RetVal(ito::retError, 0, QObject::tr("PTP: ERROR: can't read data to send from file '%s'").arg(data_file).toLatin1().data());
                return retval;
            }
        }
    }

    retval += open_camera(portnum, 0, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
//    if (open_camera(busn, devn, 0, &ptp_usb, &params, &dev)<0)
//        return;
//    printf("Camera: %s\n", params.deviceinfo.Model);

//    printf("Sending generic request: reqCode=0x%04x, params=[0x%08x,0x%08x,0x%08x,0x%08x,0x%08x]\n",
//        (uint)reqCode, reqParams[0], reqParams[1], reqParams[2], reqParams[3], reqParams[4]);
    uint16_t result = ptp_sendgenericrequest(&params, reqCode, reqParams, &data, direction, fsize);
    if ((result) != PTP_RC_OK)
    {
        ptp_perror(&params, result);
        if (result > 0x2000)
            retval += ito::RetVal(ito::retError, 0, QObject::tr("PTP: ERROR: response 0x%1").arg(QString::number(result)).toLatin1().data());
//            fprintf(stderr, "PTP: ERROR: response 0x%04x\n", result);
    }
    else
    {
        if (data != NULL && direction == PTP_DP_GETDATA)
        {
            // display_hexdump(data, malloc_usable_size((void*)data));
            free(data);
        }
        // printf("PTP: response OK\n");
    }
    if (data != NULL && direction == PTP_DP_SENDDATA)
    {
        free(data);
    }
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::list_operations(int portnum, short force, QVector<QString> &operations)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    int i;
    const char* name;

    // printf("\nListing supported operations...\n");

    operations.clear();
    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
//    if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
//        return;
    // printf("Camera: %s\n", params.deviceinfo.Model);
    for (i = 0; i<params.deviceinfo.OperationsSupported_len; i++)
    {
        name = ptp_get_operation_name(&params,
            params.deviceinfo.OperationsSupported[i]);

        if (name == NULL)
        {
            retval += ito::RetVal(ito::retWarning, 0, QObject::tr("0x%1: UNKNOWN").arg(QString::number(params.deviceinfo.OperationsSupported[i])).toLatin1().data());
            // printf("  0x%04x: UNKNOWN\n",
            // params.deviceinfo.OperationsSupported[i]);
        }
        else
        {
            // printf("  0x%04x: %s\n",
            //    params.deviceinfo.OperationsSupported[i], name);
            operations.append(QString("0x%1: %2").arg(QString::number(params.deviceinfo.OperationsSupported[i]), QString(name)));
        }
    }
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PtpCam::list_properties(int portnum, short force, QVector<QString> &properties)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    const char* propname;
    int i;

    // printf("\nListing properties...\n");
    properties.clear();
    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
    // printf("Camera: %s\n", params.deviceinfo.Model);
    for (i = 0; i<params.deviceinfo.DevicePropertiesSupported_len; i++)
    {
        propname = ptp_prop_getname(&params,
            params.deviceinfo.DevicePropertiesSupported[i]);
        if (propname != NULL)
        {
            properties.append(QString("0x%1: %2").arg((ushort)params.deviceinfo.DevicePropertiesSupported[i], 4, 16, QChar('0')).arg(QString(propname)));
//            printf("  0x%04x: %s\n",
//                params.deviceinfo.DevicePropertiesSupported[i],
//                propname);

        }
        else
        {
            retval += ito::RetVal(ito::retWarning, 0, QObject::tr("0x%1: UNKNOWN").arg(QString::number(params.deviceinfo.DevicePropertiesSupported[i])).toLatin1().data());
            printf("  0x%04x: UNKNOWN\n",
                params.deviceinfo.DevicePropertiesSupported[i]);
        }
    }
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//short print_propval(uint16_t datatype, void* value, short hex);
/*
short print_propval(uint16_t datatype, void* value, short hex)
{
    switch (datatype) {
    case PTP_DTC_INT8:
        printf("%hhi", *(char*)value);
        return 0;
    case PTP_DTC_UINT8:
        printf("%hhu", *(unsigned char*)value);
        return 0;
    case PTP_DTC_INT16:
        printf("%hi", *(int16_t*)value);
        return 0;
    case PTP_DTC_UINT16:
        if (hex == PTPCAM_PRINT_HEX)
            printf("0x%04hX (%hi)", *(uint16_t*)value,
            *(uint16_t*)value);
        else
            printf("%hi", *(uint16_t*)value);
        return 0;
    case PTP_DTC_INT32:
        printf("%li", (long int)*(int32_t*)value);
        return 0;
    case PTP_DTC_UINT32:
        if (hex == PTPCAM_PRINT_HEX)
            printf("0x%08lX (%lu)",
            (long unsigned)*(uint32_t*)value,
            (long unsigned)*(uint32_t*)value);
        else
            printf("%lu", (long unsigned)*(uint32_t*)value);
        return 0;
    case PTP_DTC_STR:
        printf("\"%s\"", (char *)value);
    }
    return -1;
}
*/

//----------------------------------------------------------------------------------------------------------------------------------
// uint16_t set_property(PTPParams* params, uint16_t property, const char* value, uint16_t datatype);
ito::RetVal PtpCam::set_property(PTPParams* params, uint16_t property, const char* value, uint16_t datatype)
{
    ito::RetVal retval;
    void* val = NULL;

    switch (datatype)
    {
        case PTP_DTC_INT8:
            val = malloc(sizeof(int8_t));
            *(int8_t*)val = (int8_t)strtol(value, NULL, 0);
            break;
        case PTP_DTC_UINT8:
            val = malloc(sizeof(uint8_t));
            *(uint8_t*)val = (uint8_t)strtol(value, NULL, 0);
            break;
        case PTP_DTC_INT16:
            val = malloc(sizeof(int16_t));
            *(int16_t*)val = (int16_t)strtol(value, NULL, 0);
            break;
        case PTP_DTC_UINT16:
            val = malloc(sizeof(uint16_t));
            *(uint16_t*)val = (uint16_t)strtol(value, NULL, 0);
            break;
        case PTP_DTC_INT32:
            val = malloc(sizeof(int32_t));
            *(int32_t*)val = (int32_t)strtol(value, NULL, 0);
            break;
        case PTP_DTC_UINT32:
            val = malloc(sizeof(uint32_t));
            *(uint32_t*)val = (uint32_t)strtol(value, NULL, 0);
            break;
        case PTP_DTC_STR:
            val = (void *)value;
    }

    if (ptp_setdevicepropvalue(params, property, val, datatype))
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("error setting property").toLatin1().data());
    }
    free(val);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void getset_property_internal(PTPParams* params, uint16_t property, const char* value, short force);
ito::RetVal PtpCam::getset_property_internal(PTPParams* params, uint16_t property, char** value, short force)
{
    ito::RetVal retval;
    PTPDevicePropDesc dpd;
    const char* propname;
    const char *propdesc;
    uint16_t result;

    memset(&dpd, 0, sizeof(dpd));
    result = ptp_getdevicepropdesc(params, property, &dpd);
    if (result != PTP_RC_OK&&!force)
    {
        ptp_perror(params, result);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not get device property description! Try to reset the camera.").toLatin1().data());
        // fprintf(stderr, "ERROR: "
        //    "Could not get device property description!\n"
        //    "Try to reset the camera.\n");
        return retval;
    }
    // until this point dpd has to be free()ed
    propdesc = ptp_prop_getdesc(params, &dpd, NULL);
    propname = ptp_prop_getname(params, property);

    if (*value == NULL)
    {
        *value = (char*)calloc(1024, sizeof(unsigned char));
        // property GET
        if (!m_verbose)
        {
            switch (dpd.DataType)
            {
                case PTP_DTC_INT8:
                    _snprintf(*value, 1024, "%hhi", *(char*)dpd.CurrentValue);
                break;
                case PTP_DTC_UINT8:
                    _snprintf(*value, 1024, "%hhu", *(unsigned char*)dpd.CurrentValue);
                break;
                case PTP_DTC_INT16:
                    _snprintf(*value, 1024, "%hi", *(int16_t*)dpd.CurrentValue);
                break;
                case PTP_DTC_UINT16:
                    if (dpd.FormFlag == PTP_DPFF_Enumeration)
                        _snprintf(*value, 1024, "0x%04hX (%hi)", *(uint16_t*)dpd.CurrentValue, *(uint16_t*)dpd.CurrentValue);
                    else
                        _snprintf(*value, 1024, "%hi", *(uint16_t*)dpd.CurrentValue);
                break;
                case PTP_DTC_INT32:
                    _snprintf(*value, 1024, "%li", (long int)*(int32_t*)dpd.CurrentValue);
                break;
                case PTP_DTC_UINT32:
                    if (dpd.FormFlag == PTP_DPFF_Enumeration)
                        _snprintf(*value, 1024, "0x%08lX (%lu)", (long unsigned)*(uint32_t*)dpd.CurrentValue,
                        (long unsigned)*(uint32_t*)dpd.CurrentValue);
                    else
                        _snprintf(*value, 1024, "%lu", (long unsigned)*(uint32_t*)dpd.CurrentValue);
                break;
                case PTP_DTC_STR:
                    _snprintf(*value, 1024, "%s", (char *)dpd.CurrentValue);
            }
            // short output, default
            // printf("'%s' is set to: ", propname == NULL ? "UNKNOWN" : propname);
/*
            if (propdesc != NULL)
                printf("[%s]", propdesc);
            else
            {
                if (dpd.FormFlag == PTP_DPFF_Enumeration)
                    PRINT_PROPVAL_HEX(dpd.CurrentValue);
                else
                    PRINT_PROPVAL_DEC(dpd.CurrentValue);
            }
*/
            // printf("\n");
        }
        else
        {
/*
            // verbose output
            printf("%s: [0x%04x, ", propname == NULL ? "UNKNOWN" : propname,
                property);
            if (dpd.GetSet == PTP_DPGS_Get)
                printf("readonly, ");
            else
                printf("readwrite, ");
            printf("%s] ",
                ptp_get_datatype_name(params, dpd.DataType));

            printf("\n  Current value: ");
            if (dpd.FormFlag == PTP_DPFF_Enumeration)
                PRINT_PROPVAL_HEX(dpd.CurrentValue);
            else
                PRINT_PROPVAL_DEC(dpd.CurrentValue);

            if (propdesc != NULL)
                printf(" [%s]", propdesc);
            printf("\n  Factory value: ");
            if (dpd.FormFlag == PTP_DPFF_Enumeration)
                PRINT_PROPVAL_HEX(dpd.FactoryDefaultValue);
            else
                PRINT_PROPVAL_DEC(dpd.FactoryDefaultValue);
            propdesc = ptp_prop_getdesc(params, &dpd,
                dpd.FactoryDefaultValue);
            if (propdesc != NULL)
                printf(" [%s]", propdesc);
            printf("\n");

            switch (dpd.FormFlag)
            {
                case PTP_DPFF_Enumeration:
                {
                    int i;
                    printf("Enumerated:\n");
                    for (i = 0; i<dpd.FORM.Enum.NumberOfValues; i++)
                    {
                        PRINT_PROPVAL_HEX(
                            dpd.FORM.Enum.SupportedValue[i]);
                        propdesc = ptp_prop_getdesc(params, &dpd, dpd.FORM.Enum.SupportedValue[i]);
                        if (propdesc != NULL) printf("\t[%s]", propdesc);
                        printf("\n");
                    }
                }
                break;

                case PTP_DPFF_Range:
                    printf("Range [");
                    PRINT_PROPVAL_DEC(dpd.FORM.Range.MinimumValue);
                    printf(" - ");
                    PRINT_PROPVAL_DEC(dpd.FORM.Range.MaximumValue);
                    printf("; step ");
                    PRINT_PROPVAL_DEC(dpd.FORM.Range.StepSize);
                    printf("]\n");
                break;

                case PTP_DPFF_None:
                break;
            }
*/
        }
    }
    else
    {
        uint16_t r;
        propdesc = ptp_prop_getdesc(params, &dpd, NULL);
/*
        printf("'%s' is set to: ", propname == NULL ? "UNKNOWN" : propname);
        if (propdesc != NULL)
            printf("[%s]", propdesc);
        else
        {
            if (dpd.FormFlag == PTP_DPFF_Enumeration)
                PRINT_PROPVAL_HEX(dpd.CurrentValue);
            else
                PRINT_PROPVAL_DEC(dpd.CurrentValue);
        }
        printf("\n");
*/
        propdesc = ptp_prop_getdescbystring(params, &dpd, *value);
        /*
        if (propdesc==NULL)
        {
            fprintf(stderr, "ERROR: Unable to set property to unidentified value: '%s'\n",
            value);
            goto out;
        }
        */
//        printf("Changing property value to %s [%s] ",
//            value, propdesc);
        retval += set_property(params, property, *value, dpd.DataType);
        if (retval.containsError())
        {
            // printf("FAILED!!!\n");
            fflush(NULL);
            retval += ito::RetVal(ito::retError, 0, QObject::tr("FAILED!!!").toLatin1().data());
            ptp_perror(params, r);
        }
//        else
//            printf("succeeded.\n");
    }
    /*	out: */

    ptp_free_devicepropdesc(&dpd);

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
// void getset_propertybyname(int busn, int devn, char* property, char* value, short force);
ito::RetVal PtpCam::getset_propertybyname(int portnum, char* property, char** value, short force)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    char *p;
    uint16_t dpc;
    const char *propval = NULL;

    // printf("\n");

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
    {
        return retval;
    }
    // if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    //    return;

    // printf("Camera: %s", params.deviceinfo.Model);
    // if ((devn != 0) || (busn != 0))
    //    printf(" (bus %i, dev %i)\n", busn, devn);
    // else
    //    printf("\n");

    if (property == NULL)
    {
        // fprintf(stderr, "ERROR: no such property\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: no such property").toLatin1().data());
        return retval;
    }


    // 1. change all '-' in property and value to ' '
    // 2. change all '  ' in property and value to '-'
    // 3. get property code by name
    // 4. get value code by name
    // 5. set property
#ifdef WIN32
    while ((p = strchr(property, '-')) != NULL)
#else
    while ((p = index(property, '-')) != NULL)
#endif
    {
        *p = ' ';
    }

    dpc = ptp_prop_getcodebyname(&params, property);
    if (dpc == 0)
    {
        // fprintf(stderr, "ERROR: Could not find property '%s'\n",
        //    property);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not find property %1").arg(property).toLatin1().data());
        retval += close_camera(&ptp_usb, &params, m_pdev);
        m_pdev = NULL;
        return retval;
    }

    if (!ptp_property_issupported(&params, dpc))
    {
        // fprintf(stderr, "The device does not support this property!\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("The device does not support this property!").toLatin1().data());
        retval += close_camera(&ptp_usb, &params, m_pdev);
        m_pdev = NULL;
        return retval;
    }

    if (value != NULL)
    {
#ifdef WIN32
        while ((p = strchr(*value, '-')) != NULL)
#else
        while ((p = index(*value, '-')) != NULL)
#endif
        {
            *p = ' ';
        }
        propval = ptp_prop_getvalbyname(&params, *value, dpc);
        if (propval == NULL) propval = *value;
    }

    retval += getset_property_internal(&params, dpc, (char**)&propval, force);
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void getset_property(int busn, int devn, uint16_t property, char* value, short force);
ito::RetVal PtpCam::getset_property(int portnum, uint16_t property, char** value, short force)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;

    // printf("\n");

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    // if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    //    return;

    // printf("Camera: %s", params.deviceinfo.Model);
    // if ((devn != 0) || (busn != 0))
    //    printf(" (bus %i, dev %i)\n", busn, devn);
    // else
    //    printf("\n");

    if (!ptp_property_issupported(&params, property) && !force)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("The device does not support this property!").toLatin1().data());
        // fprintf(stderr, "The device does not support this property!\n");
        retval += close_camera(&ptp_usb, &params, m_pdev);
        m_pdev = NULL;
        return retval;
    }

    retval += getset_property_internal(&params, property, value, force);
    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void getset_property_value(int busn, int devn, uint16_t property, char* value, short force);
ito::RetVal PtpCam::getset_property_value(int portnum, uint16_t property, char** value, short force)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;

    // printf("\n");

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
    // if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    //    return;

    // printf("Camera: %s", params.deviceinfo.Model);
    // if ((devn != 0) || (busn != 0))
    //    printf(" (bus %i, dev %i)\n", busn, devn);
    // else
    //    printf("\n");
    if (!ptp_property_issupported(&params, property) && !force)
    {
        retval += ito::RetVal(ito::retError, 0, QObject::tr("The device does not support this property!").toLatin1().data());
        //fprintf(stderr, "The device does not support this property!\n");
        retval += close_camera(&ptp_usb, &params, m_pdev);
        m_pdev = NULL;
        return retval;
    }

    // Transaction data phase description
    #define PTP_DP_NODATA		0x0000	// no data phase
    #define PTP_DP_SENDDATA		0x0001	// sending data
    #define PTP_DP_GETDATA		0x0002	// receiving data
    #define PTP_DP_DATA_MASK	0x00ff	// data phase mask
    #define PTP_CNT_INIT(cnt) { memset(&cnt,0,sizeof(cnt)); }

    PTPContainer ptp;
    uint16_t ret;
    char* dpv = NULL;

    PTP_CNT_INIT(ptp);
    ptp.Code = PTP_OC_GetDevicePropValue;
    ptp.Param1 = property;
    ptp.Nparam = 1;
    //ret = ptp_transaction(&params, &ptp, PTP_DP_GETDATA, 0, &dpv);
    //result =ptp_getdevicepropvalue (&params, property, &prop, PTP_DTC_UINT8);
    ret = ptp_getdevicepropvalue(&params, ptp.Code, (void**)&property, PTP_DTC_UINT8);

    if (ret != PTP_RC_OK)
    {
        ptp_perror(&params, ret);
        // fprintf(stderr, "ERROR: "
        //    "Could not get device property description!\n"
        //    "Try to reset the camera.\n");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("ERROR: Could not get device property description! Try to reset the camera.").toLatin1().data());
        if (!force) return retval;
    }
    // if (dpv) printf("%x is set to: %08x\n", property, *((uint8_t*)dpv));

    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
//void show_all_properties(int busn, int devn, short force, int unknown);
ito::RetVal PtpCam::show_all_properties(int portnum, short force, int unknown, QMap<QString, QString> &properties)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    PTPDevicePropDesc dpd;
    const char* propname;
    const char *propdesc;
    int i;

    // printf("\n");

    retval += open_camera(portnum, force, &ptp_usb, &params, &m_pdev);
    if (retval.containsError())
        return retval;
    // if (open_camera(busn, devn, force, &ptp_usb, &params, &dev)<0)
    //    return;

    // printf("Camera: %s", params.deviceinfo.Model);
    // if ((devn != 0) || (busn != 0))
    //    printf(" (bus %i, dev %i)\n", busn, devn);
    // else
    //    printf("\n");

    for (i = 0; i<params.deviceinfo.DevicePropertiesSupported_len; i++)
    {
        propname = ptp_prop_getname(&params,
            params.deviceinfo.DevicePropertiesSupported[i]);
        if ((unknown) && (propname != NULL)) continue;

        // printf("  0x%04x: ",
        //    params.deviceinfo.DevicePropertiesSupported[i]);
        memset(&dpd, 0, sizeof(dpd));
        if (ptp_getdevicepropdesc(&params,
            params.deviceinfo.DevicePropertiesSupported[i], &dpd))
            retval += ito::RetVal(ito::retError, 0, QObject::tr("Could not get device property description! Try to reset the camera.").toLatin1().data());
        // CR(ptp_getdevicepropdesc(&params,
        //    params.deviceinfo.DevicePropertiesSupported[i], &dpd),
        //    "Could not get device property description!\n"
        //    "Try to reset the camera.\n");
        propdesc = ptp_prop_getdesc(&params, &dpd, NULL);

        if (m_verbose)
        {
            // printf("(%s", propname == NULL ? "UNKNOWN" : propname);
            // if (propdesc != NULL)
            //    printf(": %s) ", propdesc);
            // else
            //    printf(") ");
        }
        // PRINT_PROPVAL_DEC(dpd.CurrentValue);
        if (propname != NULL)
            properties.insert(QString(propname), QString::number(*(int*)dpd.CurrentValue));

        // printf("\n");
        ptp_free_devicepropdesc(&dpd);
    }

    retval += close_camera(&ptp_usb, &params, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
int PtpCam::usb_get_endpoint_status(PTP_USB* ptp_usb, int ep, uint16_t* status)
{
//    return (usb_control_msg(ptp_usb->handle,
//        USB_DP_DTH | USB_RECIP_ENDPOINT, USB_REQ_GET_STATUS,
//        USB_FEATURE_HALT, ep, (char *)status, 2, 3000));
    return (libusb_control_transfer(ptp_usb->handle, USB_DP_DTH | LIBUSB_RECIPIENT_ENDPOINT
        | LIBUSB_REQUEST_GET_STATUS,
        USB_FEATURE_HALT, ep, 0, (unsigned char*)status, 2, 3000));
}

//----------------------------------------------------------------------------------------------------------------------------------
int PtpCam::usb_clear_stall_feature(PTP_USB* ptp_usb, int ep)
{

//    return (usb_control_msg(ptp_usb->handle,
//        USB_RECIP_ENDPOINT, USB_REQ_CLEAR_FEATURE, USB_FEATURE_HALT,
//        ep, NULL, 0, 3000));
    return (libusb_control_transfer(ptp_usb->handle, LIBUSB_REQUEST_CLEAR_FEATURE | LIBUSB_RECIPIENT_ENDPOINT,
        USB_FEATURE_HALT, ep, 0, NULL, 0, 3000));
}

//----------------------------------------------------------------------------------------------------------------------------------
// int usb_ptp_get_device_status(PTP_USB* ptp_usb, uint16_t* devstatus);
int PtpCam::usb_ptp_get_device_status(PTP_USB* ptp_usb, uint16_t* devstatus)
{
//    return (usb_control_msg(ptp_usb->handle,
//        USB_DP_DTH | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
//        USB_REQ_GET_DEVICE_STATUS, 0, 0,
//        (char *)devstatus, 4, 3000));
    return (libusb_control_transfer(ptp_usb->handle, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        USB_REQ_GET_DEVICE_STATUS, 0, 0, (unsigned char*)devstatus, 4, 3000));
}

//----------------------------------------------------------------------------------------------------------------------------------
// int usb_ptp_device_reset(PTP_USB* ptp_usb);
int PtpCam::usb_ptp_device_reset(PTP_USB* ptp_usb)
{
//    return (usb_control_msg(ptp_usb->handle,
//        USB_TYPE_CLASS | USB_RECIP_INTERFACE,
//        USB_REQ_DEVICE_RESET, 0, 0, NULL, 0, 3000));
    return (libusb_control_transfer(ptp_usb->handle, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        USB_REQ_DEVICE_RESET, 0, 0, NULL, 0, 3000));
}

//----------------------------------------------------------------------------------------------------------------------------------
// void reset_device(int busn, int devn, short force);
ito::RetVal PtpCam::reset_device(int portnum, short force)
{
    ito::RetVal retval;
    PTPParams params;
    PTP_USB ptp_usb;
    uint16_t status;
    uint16_t devstatus[2] = { 0, 0 };
    int ret;

#ifdef DEBUG
    stdout << printf("dev %i\tbus %i\n", devn, busn);
#endif
    m_pdev = find_device(portnum, force);
    if (m_pdev == NULL)
    {
        // fprintf(stderr, "could not find any device matching given "
        //    "bus/dev numbers\n");
        // exit(-1);
        retval += ito::RetVal(ito::retError, 0, QObject::tr("could not find any device matching given bus/dev numbers").toLatin1().data());
        return retval;
    }
    retval += find_endpoints(m_pdev, &ptp_usb.inep, &ptp_usb.outep, &ptp_usb.intep);

    retval += init_ptp_usb(&params, &ptp_usb, m_pdev);

    // get device status (devices likes that regardless of its result)
    retval += usb_ptp_get_device_status(&ptp_usb, devstatus);

    // check the in endpoint status
    ret = usb_get_endpoint_status(&ptp_usb, ptp_usb.inep, &status);
    //if (ret<0) perror("usb_get_endpoint_status()");
    if (ret < 0)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_get_endpoint_status()").toLatin1().data());

    // and clear the HALT condition if happend
    if (status)
    {
        // printf("Resetting input pipe!\n");
        ret = usb_clear_stall_feature(&ptp_usb, ptp_usb.inep);
        //if (ret<0) perror("usb_clear_stall_feature()");
        if (ret < 0)
            retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_clear_stall_feature()").toLatin1().data());
    }
    status = 0;
    // check the out endpoint status
    ret = usb_get_endpoint_status(&ptp_usb, ptp_usb.outep, &status);
    // if (ret<0) perror("usb_get_endpoint_status()");
    if (ret < 0)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_get_endpoint_status()").toLatin1().data());

    // and clear the HALT condition if happend
    if (status)
    {
        // printf("Resetting output pipe!\n");
        ret = usb_clear_stall_feature(&ptp_usb, ptp_usb.outep);
        //if (ret<0)perror("usb_clear_stall_feature()");
        if (ret < 0)
            retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_clear_stall_feature()").toLatin1().data());
    }

    status = 0;
    // check the interrupt endpoint status
    ret = usb_get_endpoint_status(&ptp_usb, ptp_usb.intep, &status);
    //if (ret<0)perror("usb_get_endpoint_status()");
    if (ret < 0)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_get_endpoint_status()").toLatin1().data());

    // and clear the HALT condition if happend
    if (status)
    {
        // printf("Resetting interrupt pipe!\n");
        ret = usb_clear_stall_feature(&ptp_usb, ptp_usb.intep);
        //if (ret<0)perror("usb_clear_stall_feature()");
        if (ret < 0)
            retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_clear_stall_feature()").toLatin1().data());
    }

    // get device status (now there should be some results)
    ret = usb_ptp_get_device_status(&ptp_usb, devstatus);
    if (ret<0)
        // perror("usb_ptp_get_device_status()");
        retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_ptp_get_device_status()").toLatin1().data());
    else
    {
        if (devstatus[1] == PTP_RC_OK)
            printf("Device status OK\n");
        else
            printf("Device status 0x%04x\n", devstatus[1]);
    }

    // finally reset the device (that clears prevoiusly opened sessions)
    ret = usb_ptp_device_reset(&ptp_usb);
    // if (ret<0)perror("usb_ptp_device_reset()");
    if (ret < 0)
        retval += ito::RetVal(ito::retError, 0, QObject::tr("usb_ptp_device_reset()").toLatin1().data());
    // get device status (devices likes that regardless of its result)
    usb_ptp_get_device_status(&ptp_usb, devstatus);

    retval += close_usb(&ptp_usb, m_pdev);
    m_pdev = NULL;

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
// uint16_t ptp_transaction_nodata(PTPParams* params, PTPContainer* ptp);
/*
uint16_t PtpCam::ptp_transaction_nodata(PTPParams* params, PTPContainer* ptp)
{
    return (ptp_transaction(params, ptp, PTP_DP_NODATA, 0, 0));
}

//----------------------------------------------------------------------------------------------------------------------------------
//uint16_t ptp_transaction_getdata(PTPParams* params, PTPContainer* ptp, unsigned int *getlen, char** data);
uint16_t PtpCam::ptp_transaction_getdata(PTPParams* params, PTPContainer* ptp, unsigned int *getlen, char** data)
{
    return (ptp_transaction(params, ptp, PTP_DP_GETDATA, (unsigned int)getlen, data));
}

//----------------------------------------------------------------------------------------------------------------------------------
// uint16_t ptp_transaction_senddata(PTPParams* params, PTPContainer* ptp, unsigned int sendlen, char* data);
uint16_t PtpCam::ptp_transaction_senddata(PTPParams* params, PTPContainer* ptp, unsigned int sendlen, char* data)
{
    return (ptp_transaction(params, ptp, PTP_DP_SENDDATA, sendlen, &data));
}
*/
//----------------------------------------------------------------------------------------------------------------------------------
/*
void ptphack()
{
    PTPParams params = {};
    PTP_USB ptp_usb = {};
    struct libusb_device *dev;
    PTPContainer ptp = {};
    int getlen = 0;

    char* data = NULL;

    PTPDeviceInfo 	PTPDeviceInfo = {};
    PTPStorageIDs 	PTPStorageIDs = {};
    PTPStorageInfo 	PTPStorageInfo = {};
    PTPObjectHandles 	PTPObjectHandles = {};
    PTPObjectInfo	PTPObjectInfo = {};
    PTPPropDescRangeForm 	PTPPropDescRangeForm = {};
    PTPPropDescEnumForm 	PTPPropDescEnumForm = {};
    PTPDevicePropDesc 	PTPDevicePropDesc = {};
    PTPCANONFolderEntry 	PTPCANONFolderEntry = {};

    if (open_camera(0, 0, 0, &ptp_usb, &params, &dev)<0)
        return;

    raise(SIGINT);

    //ptp_transaction(&params, &ptp, PTP_DP_GETDATA, 0, &dpv);

    // ptp_transaction_nodata(&params, &ptp)
    // ptp_transaction_getdata(&params, &ptp, &getlen, &data)
    // ptp_transaction_senddata(&params, &ptp, sendlen, data)

    close_camera(&ptp_usb, &params, dev);
}
*/

// main program
/*
int
main(int argc, char ** argv)
{
    int busn = 0, devn = 0;
    int action = 0;
    short force = 0;
    int overwrite = SKIP_IF_EXISTS;
    uint16_t property = 0;
    char* value = NULL;
    char* propstr = NULL;
    uint32_t handle = 0;
    char *filename = NULL;
    int num = 0;
    int interval = 0;
    uint16_t reqCode = 0;
    uint32_t reqParams[5];
    uint32_t direction = PTP_DP_GETDATA;
    char data_file[256];
    // parse options
    int option_index = 0, opt;
    static struct option loptions[] = {
        { "help", 0, 0, 'h' },
        { "bus", 1, 0, 0 },
        { "dev", 1, 0, 0 },
        { "reset", 0, 0, 'r' },
        { "list-devices", 0, 0, 'l' },
        { "list-files", 0, 0, 'L' },
        { "list-operations", 1, 0, 'o' },
        { "list-properties", 0, 0, 'p' },
        { "show-all-properties", 0, 0, 0 },
        { "show-unknown-properties", 0, 0, 0 },
        { "show-property", 1, 0, 's' },
        { "set-property", 1, 0, 's' },
        { "set", 1, 0, 0 },
        { "get-file", 1, 0, 'g' },
        { "get-all-files", 0, 0, 'G' },
        { "capture", 0, 0, 'c' },
        { "nikon-dc", 0, 0, 0 },
        { "ndc", 0, 0, 0 },
        { "nikon-ic", 0, 0, 0 },
        { "nic", 0, 0, 0 },
        { "nikon-dc2", 0, 0, 0 },
        { "ndc2", 0, 0, 0 },
        { "loop-capture", 1, 0, 0 },
        { "interval", 1, 0, 0 },
        { "delete-object", 1, 0, 'd' },
        { "delete-all-files", 1, 0, 'D' },
        { "info", 0, 0, 'i' },
        { "val", 1, 0, 0 },
        { "filename", 1, 0, 0 },
        { "overwrite", 0, 0, 0 },
        { "force", 0, 0, 'f' },
        { "verbose", 2, 0, 'v' },
        { 0, 0, 0, 0 }
    };

    // register signal handlers
    signal(SIGINT, ptpcam_siginthandler);

    while (1) {
        opt = getopt_long(argc, argv, "LhlcipfroGg:Dd:s:v::R:", loptions, &option_index);
        if (opt == -1) break;

        switch (opt) {
            // set parameters
        case 0:
            if (!(strcmp("val", loptions[option_index].name)))
                value = strdup(optarg);
            if (!(strcmp("filename", loptions[option_index].name)))
                filename = strdup(optarg);
            if (!(strcmp("overwrite", loptions[option_index].name)))
                overwrite = OVERWRITE_EXISTING;
            if (!(strcmp("bus", loptions[option_index].name)))
                busn = strtol(optarg, NULL, 10);
            if (!(strcmp("dev", loptions[option_index].name)))
                devn = strtol(optarg, NULL, 10);
            if (!(strcmp("loop-capture", loptions[option_index].name)))
            {
                action = ACT_LOOP_CAPTURE;
                num = strtol(optarg, NULL, 10);
            }
            if (!(strcmp("show-all-properties", loptions[option_index].name)))
                action = ACT_SHOW_ALL_PROPERTIES;
            if (!(strcmp("show-unknown-properties", loptions[option_index].name)))
                action = ACT_SHOW_UNKNOWN_PROPERTIES;
            if (!(strcmp("set", loptions[option_index].name)))
            {
                propstr = strdup(optarg);
                action = ACT_SET_PROPBYNAME;
            }
            if (!(strcmp("interval", loptions[option_index].name)))
                interval = strtol(optarg, NULL, 10);
            if (!strcmp("nikon-dc", loptions[option_index].name) ||
                !strcmp("ndc", loptions[option_index].name))
            {
                action = ACT_NIKON_DC;
            }
            if (!strcmp("nikon-ic", loptions[option_index].name) ||
                !strcmp("nic", loptions[option_index].name))
            {
                action = ACT_NIKON_IC;
            }
            if (!strcmp("nikon-dc2", loptions[option_index].name) ||
                !strcmp("ndc2", loptions[option_index].name))
            {
                action = ACT_NIKON_DC2;
            }
            break;
        case 'f':
            force = ~force;
            break;
        case 'v':
            if (optarg)
                verbose = strtol(optarg, NULL, 10);
            else
                verbose = 1;
            // printf("VERBOSE LEVEL  = %i\n",verbose);
            break;
            // actions
        case 'h':
            help();
            break;
        case 'r':
            action = ACT_DEVICE_RESET;
            break;
        case 'l':
            action = ACT_LIST_DEVICES;
            break;
        case 'p':
            action = ACT_LIST_PROPERTIES;
            break;
        case 's':
            action = ACT_GETSET_PROPERTY;
            property = strtol(optarg, NULL, 16);
            break;
        case 'o':
            action = ACT_LIST_OPERATIONS;
            break;
        case 'i':
            action = ACT_SHOW_INFO;
            break;
        case 'c':
            action = ACT_CAPTURE;
            break;
        case 'L':
            action = ACT_LIST_FILES;
            break;
        case 'g':
            action = ACT_GET_FILE;
            handle = strtol(optarg, NULL, 16);
            break;
        case 'G':
            action = ACT_GET_ALL_FILES;
            break;
        case 'd':
            action = ACT_DELETE_OBJECT;
            handle = strtol(optarg, NULL, 16);
            break;
        case 'D':
            action = ACT_DELETE_ALL_FILES;
            break;
        case 'R':
            action = ACT_GENERIC_REQ;
            memset((void*)reqParams, 0, sizeof(uint32_t) * 5);
            memset((void*)data_file, 0, 256);
            sscanf(optarg, "%x,%x,%x,%x,%x,%x,%s", (uint*)&reqCode, &reqParams[0], &reqParams[1],
                &reqParams[2], &reqParams[3], &reqParams[4], data_file);
            if (data_file[0] == 'r') {
                direction = PTP_DP_GETDATA;
            }
            else if (data_file[0] == 'n') {
                direction = PTP_DP_NODATA;
            }
            else if (strlen(data_file)>0) {
                direction = PTP_DP_SENDDATA;
                printf("data to send : '%s'\n", data_file);
            }
            break;
        case '?':
            break;
        default:
            fprintf(stderr, "getopt returned character code 0%o\n",
                opt);
            break;
        }
    }
    if (argc == 1) {
        usage();
        return 0;
    }
    switch (action) {
    case ACT_DEVICE_RESET:
        reset_device(busn, devn, force);
        break;
    case ACT_LIST_DEVICES:
        list_devices(force);
        break;
    case ACT_LIST_PROPERTIES:
        list_properties(busn, devn, force);
        break;
    case ACT_GETSET_PROPERTY:
        //getset_property_value(busn,devn,property,value,force);
        getset_property(busn, devn, property, value, force);
        break;
    case ACT_SHOW_INFO:
        show_info(busn, devn, force);
        break;
    case ACT_LIST_OPERATIONS:
        list_operations(busn, devn, force);
        break;
    case ACT_LIST_FILES:
        list_files(busn, devn, force);
        break;
    case ACT_GET_FILE:
        get_file(busn, devn, force, handle, filename, overwrite);
        break;
    case ACT_GENERIC_REQ:
        send_generic_request(busn, devn, reqCode, reqParams, direction, data_file);
        break;
    case ACT_GET_ALL_FILES:
        get_all_files(busn, devn, force, overwrite);
        break;
    case ACT_CAPTURE:
        capture_image(busn, devn, force);
        break;
    case ACT_DELETE_OBJECT:
        delete_object(busn, devn, force, handle);
        break;
    case ACT_DELETE_ALL_FILES:
        delete_all_files(busn, devn, force);
        break;
    case ACT_LOOP_CAPTURE:
        loop_capture(busn, devn, force, num, interval, overwrite);
        break;
    case ACT_SHOW_ALL_PROPERTIES:
        show_all_properties(busn, devn, force, 0);
        break;
    case ACT_SHOW_UNKNOWN_PROPERTIES:
        show_all_properties(busn, devn, force, 1);
        break;
    case ACT_SET_PROPBYNAME:
        getset_propertybyname(busn, devn, propstr, value, force);
        break;
    case ACT_NIKON_DC:
        nikon_direct_capture(busn, devn, force, filename, overwrite);
        break;
    case ACT_NIKON_IC:
        nikon_initiate_dc(busn, devn, force);
        break;
    case ACT_NIKON_DC2:
        nikon_direct_capture2(busn, devn, force, filename, overwrite);
    }

    return 0;
}
*/
