/*
 * 1394-Based Digital Camera Control Library
 *
 * USB backend for dc1394
 *
 * Written by David Moore <dcm@acm.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __DC1394_USB_H__
#define __DC1394_USB_H__

#include <libusb.h>
#include "config.h"
#include "internal.h"
#include "register.h"
#include "offsets.h"

struct _platform_t {
    libusb_context *context;
};

struct _platform_camera_t {
    libusb_device_handle * handle;
    dc1394camera_t * camera;

    struct usb_frame        * frames;
    unsigned char        * buffer;
    size_t buffer_size;
    uint32_t flags;
    unsigned int num_frames;
    int current;
    int frames_ready;

    uint8_t bus;
    uint8_t addr;
    int notify_pipe[2];
    pthread_t thread;
    int thread_created;
    pthread_mutex_t mutex;
    int mutex_created;
    libusb_context *thread_context;
    libusb_device_handle *thread_handle;
    int kill_thread;

    int capture_is_set;
    int iso_auto_started;
};

typedef enum {
    BUFFER_EMPTY,
    BUFFER_FILLED,
    BUFFER_CORRUPT,
} usb_frame_status;

struct usb_frame {
    dc1394video_frame_t frame;
    struct libusb_transfer * transfer;
    platform_camera_t * pcam;
    usb_frame_status status;
};


dc1394error_t
dc1394_usb_capture_setup(platform_camera_t *craw, uint32_t num_dma_buffers,
        uint32_t flags);

dc1394error_t
dc1394_usb_capture_stop(platform_camera_t *craw);

dc1394error_t
dc1394_usb_capture_dequeue (platform_camera_t * craw,
        dc1394capture_policy_t policy, dc1394video_frame_t **frame_return);

dc1394error_t
dc1394_usb_capture_enqueue (platform_camera_t * craw,
        dc1394video_frame_t * frame);

int
dc1394_usb_capture_get_fileno (platform_camera_t * craw);

dc1394bool_t
dc1394_usb_capture_is_frame_corrupt (platform_camera_t * craw,
        dc1394video_frame_t * frame);

#endif
