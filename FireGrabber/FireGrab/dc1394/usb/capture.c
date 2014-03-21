/*
 * 1394-Based Digital Camera Control Library
 *
 * IIDC-over-USB using libusb backend for dc1394
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>
#include <unistd.h>

#include "usb/usb.h"

/* Callback whenever a bulk transfer finishes. */
static void
callback (struct libusb_transfer * transfer)
{
    struct usb_frame * f = transfer->user_data;
    platform_camera_t * craw = f->pcam;

    if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        dc1394_log_warning ("usb: Bulk transfer %d cancelled", f->frame.id);
        return;
    }

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
        dc1394_log_error ("usb: Bulk transfer %d failed with code %d",
                f->frame.id, transfer->status);

    dc1394_log_debug ("usb: Bulk transfer %d complete, %d of %d bytes",
            f->frame.id, transfer->actual_length, transfer->length);
    int status = BUFFER_FILLED;
    if (transfer->actual_length < transfer->length)
        status = BUFFER_CORRUPT;
    pthread_mutex_lock (&craw->mutex);
    f->status = status;
    craw->frames_ready++;
    pthread_mutex_unlock (&craw->mutex);

    write (craw->notify_pipe[1], "+", 1);
}

static void *
capture_thread (void * arg)
{
    platform_camera_t * craw = arg;

    dc1394_log_debug ("usb: Helper thread starting");

    while (1) {
        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = 100000,
        };
        libusb_handle_events_timeout(craw->thread_context, &tv);
        pthread_mutex_lock (&craw->mutex);
        if (craw->kill_thread)
            break;
        pthread_mutex_unlock (&craw->mutex);
    }
    pthread_mutex_unlock (&craw->mutex);
    dc1394_log_debug ("usb: Helper thread ending");
    return NULL;
}

static dc1394error_t
init_frame(platform_camera_t *craw, int index, dc1394video_frame_t *proto)
{
    struct usb_frame *f = craw->frames + index;

    memcpy (&f->frame, proto, sizeof f->frame);
    f->frame.image = craw->buffer + index * proto->total_bytes;
    f->frame.id = index;
    f->transfer = libusb_alloc_transfer (0);
    f->pcam = craw;
    f->status = BUFFER_EMPTY;
    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_usb_capture_setup(platform_camera_t *craw, uint32_t num_dma_buffers,
        uint32_t flags)
{
    dc1394video_frame_t proto;
    int i;
    dc1394camera_t * camera = craw->camera;

    // if capture is already set, abort
    if (craw->capture_is_set > 0)
        return DC1394_CAPTURE_IS_RUNNING;

    craw->capture_is_set = 1;

    if (flags & DC1394_CAPTURE_FLAGS_DEFAULT)
        flags = DC1394_CAPTURE_FLAGS_CHANNEL_ALLOC |
            DC1394_CAPTURE_FLAGS_BANDWIDTH_ALLOC;

    craw->flags = flags;

    if (capture_basic_setup(camera, &proto) != DC1394_SUCCESS) {
        dc1394_log_error("usb: Basic capture setup failed");
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }

    if (pipe (craw->notify_pipe) < 0) {
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }

    dc1394_log_debug ("usb: Frame size is %"PRId64, proto.total_bytes);

    craw->num_frames = num_dma_buffers;
    craw->current = -1;
    craw->frames_ready = 0;
    craw->buffer_size = proto.total_bytes * num_dma_buffers;
    craw->buffer = malloc (craw->buffer_size);
    if (craw->buffer == NULL) {
        dc1394_usb_capture_stop (craw);
        return DC1394_MEMORY_ALLOCATION_FAILURE;
    }

    craw->frames = calloc (num_dma_buffers, sizeof *craw->frames);
    if (craw->frames == NULL) {
        dc1394_usb_capture_stop (craw);
        return DC1394_MEMORY_ALLOCATION_FAILURE;
    }

    for (i = 0; i < num_dma_buffers; i++)
        init_frame(craw, i, &proto);

    if (libusb_init(&craw->thread_context) != 0) {
        dc1394_log_error ("usb: Failed to create thread USB context");
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }

    uint8_t bus = libusb_get_bus_number (libusb_get_device (craw->handle));
    uint8_t addr = libusb_get_device_address (libusb_get_device (craw->handle));

    libusb_device **list, *dev;
    libusb_get_device_list (craw->thread_context, &list);
    for (i = 0, dev = list[0]; dev; dev = list[++i]) {
        if (libusb_get_bus_number (dev) == bus &&
                libusb_get_device_address (dev) == addr)
            break;
    }
    if (!dev) {
        libusb_free_device_list (list, 1);
        dc1394_log_error ("usb: capture thread failed to find device");
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }

    if (libusb_open (dev, &craw->thread_handle) < 0) {
        libusb_free_device_list (list, 1);
        dc1394_log_error ("usb: capture thread failed to open device");
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }
    libusb_free_device_list (list, 1);

    if (libusb_claim_interface (craw->thread_handle, 0) < 0) {
        dc1394_log_error ("usb: capture thread failed to claim interface");
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }

    for (i = 0; i < craw->num_frames; i++) {
        struct usb_frame *f = craw->frames + i;
        libusb_fill_bulk_transfer (f->transfer, craw->thread_handle,
                0x81, f->frame.image, f->frame.total_bytes,
                callback, f, 0);
    }
    for (i = 0; i < craw->num_frames; i++) {
        if (libusb_submit_transfer (craw->frames[i].transfer) < 0) {
            dc1394_log_error ("usb: Failed to submit initial transfer %d", i);
            dc1394_usb_capture_stop (craw);
            return DC1394_FAILURE;
        }
    }

    if (pthread_mutex_init (&craw->mutex, NULL) < 0) {
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }
    craw->mutex_created = 1;
    if (pthread_create (&craw->thread, NULL, capture_thread, craw) < 0) {
        dc1394_log_error ("usb: Failed to launch helper thread");
        dc1394_usb_capture_stop (craw);
        return DC1394_FAILURE;
    }
    craw->thread_created = 1;

    // if auto iso is requested, start ISO
    if (flags & DC1394_CAPTURE_FLAGS_AUTO_ISO) {
        dc1394_video_set_transmission(camera, DC1394_ON);
        craw->iso_auto_started = 1;
    }

    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_usb_capture_stop(platform_camera_t *craw)
{
    dc1394camera_t * camera = craw->camera;
    int i;

    if (craw->capture_is_set == 0)
        return DC1394_CAPTURE_IS_NOT_SET;

    dc1394_log_debug ("usb: Capture stopping");

    // stop ISO if it was started automatically
    if (craw->iso_auto_started > 0) {
        dc1394_video_set_transmission(camera, DC1394_OFF);
        craw->iso_auto_started = 0;
    }

    if (craw->thread_created) {
#if 0
        for (i = 0; i < craw->num_frames; i++) {
            libusb_cancel_transfer (craw->frames[i].transfer);
        }
#endif
        pthread_mutex_lock (&craw->mutex);
        craw->kill_thread = 1;
        pthread_mutex_unlock (&craw->mutex);
        pthread_join (craw->thread, NULL);
        dc1394_log_debug ("usb: Joined with helper thread");
        craw->kill_thread = 0;
        craw->thread_created = 0;
    }

    if (craw->mutex_created) {
        pthread_mutex_destroy (&craw->mutex);
        craw->mutex_created = 0;
    }

    if (craw->thread_handle) {
        libusb_release_interface (craw->thread_handle, 0);
        libusb_close (craw->thread_handle);
        craw->thread_handle = NULL;
    }

    if (craw->thread_context) {
        libusb_exit (craw->thread_context);
        craw->thread_context = NULL;
    }

    if (craw->frames) {
        for (i = 0; i < craw->num_frames; i++) {
            libusb_free_transfer (craw->frames[i].transfer);
        }
        free (craw->frames);
        craw->frames = NULL;
    }

    free (craw->buffer);
    craw->buffer = NULL;

    if (craw->notify_pipe[0] != 0 || craw->notify_pipe[1] != 0) {
        close (craw->notify_pipe[0]);
        close (craw->notify_pipe[1]);
    }
    craw->notify_pipe[0] = 0;
    craw->notify_pipe[1] = 0;

    craw->capture_is_set = 0;

    return DC1394_SUCCESS;
}

#define NEXT_BUFFER(c,i) (((i) == -1) ? 0 : ((i)+1)%(c)->num_frames)

dc1394error_t
dc1394_usb_capture_dequeue (platform_camera_t * craw,
        dc1394capture_policy_t policy, dc1394video_frame_t **frame_return)
{
    int next = NEXT_BUFFER (craw, craw->current);
    struct usb_frame * f = craw->frames + next;

    if ((policy < DC1394_CAPTURE_POLICY_MIN)
            || (policy > DC1394_CAPTURE_POLICY_MAX))
        return DC1394_INVALID_CAPTURE_POLICY;

    /* default: return NULL in case of failures or lack of frames */
    *frame_return = NULL;

    if (policy == DC1394_CAPTURE_POLICY_POLL) {
        int status;
        pthread_mutex_lock (&craw->mutex);
        status = f->status;
        pthread_mutex_unlock (&craw->mutex);
        if (status != BUFFER_FILLED && status != BUFFER_CORRUPT)
            return DC1394_SUCCESS;
    }

    char ch;
    read (craw->notify_pipe[0], &ch, 1);

    pthread_mutex_lock (&craw->mutex);
    if (f->status != BUFFER_FILLED && f->status != BUFFER_CORRUPT) {
        dc1394_log_error ("usb: Expected filled buffer");
        pthread_mutex_unlock (&craw->mutex);
        return DC1394_FAILURE;
    }
    craw->frames_ready--;
    f->frame.frames_behind = craw->frames_ready;
    pthread_mutex_unlock (&craw->mutex);

    craw->current = next;

    *frame_return = &f->frame;

    return DC1394_SUCCESS;
}

dc1394error_t
dc1394_usb_capture_enqueue (platform_camera_t * craw,
        dc1394video_frame_t * frame)
{
    dc1394camera_t * camera = craw->camera;
    struct usb_frame * f = (struct usb_frame *) frame;

    if (frame->camera != camera) {
        dc1394_log_error("usb: Camera does not match frame's camera");
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    if (f->status != BUFFER_FILLED && f->status != BUFFER_CORRUPT) {
        dc1394_log_error ("usb: Frame is not enqueuable");
        return DC1394_FAILURE;
    }

    f->status = BUFFER_EMPTY;
    libusb_submit_transfer (f->transfer);

    return DC1394_SUCCESS;
}

int
dc1394_usb_capture_get_fileno (platform_camera_t * craw)
{
    if (craw->notify_pipe[0] == 0 && craw->notify_pipe[1] == 0)
        return -1;

    return craw->notify_pipe[0];
}

dc1394bool_t
dc1394_usb_capture_is_frame_corrupt (platform_camera_t * craw,
        dc1394video_frame_t * frame)
{
    struct usb_frame * f = (struct usb_frame *) frame;

    if (f->status == BUFFER_CORRUPT)
        return DC1394_TRUE;

    return DC1394_FALSE;
}

