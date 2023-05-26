/*
 * check_libusb.c
 *
 *  Created on: Jun 2, 2014
 *      Author: Steve
 */

#include <stdio.h>
#include <stdlib.h>
#include <libusb.h>
#include <errno.h>

#define VID 0x0403
#define PID 0x7C38

#define INTERFACE_A     0
#define INTERFACE_B     1

#define SIO_RESET          0 /* Reset the port */
#define FTDI_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT)
#define SIO_RESET_REQUEST             SIO_RESET
#define SIO_RESET_SIO 0
#define EPERM 1

static int find_device(libusb_context *ctx, int vid, int pid, int interface, libusb_device_handle **foundDev);
static int open_device(libusb_device *dev, libusb_device_handle **hDev, int interface);

int main(int argc, char **argv) {
    int i, err;
    libusb_context *ctx;
    libusb_device_handle *foundDev;

    for (i=0; i<50; i++) {
        printf("Open count %d of 50\n", i);

        err = libusb_init(&ctx);
        if (err < 0) {
            printf("Error initializing libusb: %s\n", libusb_strerror((libusb_error)err));
            ctx = NULL;
            break;
        }

        // find and open the device
        err = find_device(ctx, VID, PID, INTERFACE_A, &foundDev);
        if (err == 0) {
            if (foundDev) {
                libusb_close(foundDev);
                foundDev = NULL;
            }
            else {
                printf("No device found!\n");
                break;
            }
        }
        else {
            printf("Error finding device!\n");
            break;
        }

        // exit libusb
        if (ctx) {
            libusb_exit(ctx);
            ctx = NULL;
        }
    }

    // exit libusb if not done earlier
    if (ctx) {
        libusb_exit(ctx);
        ctx = NULL;
    }


    return err;
}

static int find_device(libusb_context *ctx, int vid, int pid, int interface, libusb_device_handle **foundDev) {
    int i = 0, err;
    libusb_device *dev;
    libusb_device **devs;
    libusb_device_handle *hDev;

    // get the list of devices
    err = libusb_get_device_list(ctx, &devs);
    if (err < 0) {
        printf("Error getting device list: %s\n", libusb_strerror((libusb_error)err));
        devs = NULL;
        return err;
    }

    // find our device
    while ((dev = devs[i++]) != NULL) {
        struct libusb_device_descriptor desc;
        int res;

        if (libusb_get_device_descriptor(dev, &desc) < 0) {
            printf("Error getting device descriptor: %s\n", libusb_strerror((libusb_error)err));
            break;
        }

        struct libusb_config_descriptor *config = NULL;
        if (libusb_get_config_descriptor(dev, 0, &config)) {
            printf("Error getting config descriptor: %s\n", libusb_strerror((libusb_error)err));
            break;
        }

        // if this isn't our device, just continue
        /*
        if (desc.idVendor != vid || desc.idProduct != pid) {
            continue;
        }
        */
        #define USB_CLASS_PTP 6
        if (config->interface->altsetting->bInterfaceClass != USB_CLASS_PTP)
            continue;

        libusb_free_config_descriptor(config);

        // open the device to check for serial number match
        err = libusb_open(dev, &hDev);
        if (err < 0) {
            printf("Error opening device for serial number check: %s\n", libusb_strerror((libusb_error)err));
            break;
        }

        // we're not checking for matching product and serial numbers,
        // so just close the device
        libusb_close(hDev);

        // this device matches our VID and PID, so open it
        err = open_device(dev, foundDev, interface);
        if (err < 0) {
            printf("Error getting device list: %s\n", libusb_strerror((libusb_error)err));
        }
        break;
    }

    // free the device list
    libusb_free_device_list(devs, 1);

    return err;
}

static int open_device(libusb_device *dev, libusb_device_handle **hDev, int interface) {
    int err;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor *config0;
    int cfg, cfg0, detach_errno = 0;

    err = libusb_open(dev, hDev);
    if (err < 0) {
        *hDev = NULL;
        puts(libusb_strerror((libusb_error)err));
        return err;
    }

    err = libusb_get_device_descriptor(dev, &desc);
    if (err < 0) {
        printf("Error getting device descriptor: %s\n", libusb_strerror((libusb_error)err));
        return err;
    }

    err = libusb_get_config_descriptor(dev, 0, &config0);
    if (err < 0) {
        printf("Error getting config descriptor: %s\n", libusb_strerror((libusb_error)err));
        return err;
    }
    cfg0 = config0->bConfigurationValue;
    libusb_free_config_descriptor(config0);

    // Try to detach ftdi_sio kernel module.
    //
    // The return code is kept in a separate variable and only parsed
    // if usb_set_configuration() or usb_claim_interface() fails as the
    // detach operation might be denied and everything still works fine.
    // Likely scenario is a static ftdi_sio kernel module.
//  if (ftdi->module_detach_mode == AUTO_DETACH_SIO_MODULE)
    if (1) {
        if (libusb_detach_kernel_driver(*hDev, interface) !=0) {
            detach_errno = errno;
        }
    }

    err = libusb_get_configuration(*hDev, &cfg);
    if (err < 0) {
        printf("Error getting configuration: %s\n", libusb_strerror((libusb_error)err));
        return err;
    }

    // set configuration (needed especially for windows)
    // tolerate EBUSY: one device with one configuration, but two interfaces
    //    and libftdi sessions to both interfaces (e.g. FT2232)
    if (desc.bNumConfigurations > 0 && cfg != cfg0) {
        err = libusb_set_configuration(*hDev, cfg0);
        if (err < 0) {
            libusb_close (*hDev);
            *hDev = NULL;
            if (detach_errno == EPERM) {
                printf("Error setting configuration (EPERM): %s\n", libusb_strerror((libusb_error)err));
                return err;
            }
            else {
                printf("Error setting configuration: %s\n", libusb_strerror((libusb_error)err));
                return err;
            }
        }
    }

    err = libusb_claim_interface(*hDev, interface);
    if (err < 0) {
        libusb_close (*hDev);
        *hDev = NULL;
        if (detach_errno == EPERM) {
            printf("Error claiming interface (EPERM): %s\n", libusb_strerror((libusb_error)err));
            return err;
        }
        else {
            printf("Error claiming interface: %s\n", libusb_strerror((libusb_error)err));
            return err;
        }
    }

    err = libusb_control_transfer(*hDev, FTDI_DEVICE_OUT_REQTYPE,
                                SIO_RESET_REQUEST, SIO_RESET_SIO,
                                interface+1, NULL, 0, 5000);
    if (err < 0) {
        libusb_close (*hDev);
        *hDev = NULL;
        printf("Error resetting device: %s\n", libusb_strerror((libusb_error)err));
        return err;
    }

    // skipped determining max packet size

    // skipped setting buadrate

    return err;
}
