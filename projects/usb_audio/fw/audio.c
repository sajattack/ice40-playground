/*
 * audio.c
 *
 * USB Audio class firmware
 *
 * Copyright (C) 2020 Sylvain Munaut
 * All rights reserved.
 *
 * LGPL v3+, see LICENSE.lgpl3
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "console.h"
#include "led.h"
#include "mini-printf.h"
#include "spi.h"
#include <no2usb/usb.h>
#include <no2usb/usb_ac_proto.h>
#include <no2usb/usb_dfu_rt.h>
#include <no2usb/usb_hw.h>
#include <no2usb/usb_priv.h>
#include "utils.h"
#include "config.h"


// PCM Audio
// ---------------------------------------------------------------------------

static struct {
    bool active;
    uint8_t bdi;
} g_pcm;

struct wb_audio_pcm {
    uint32_t csr;
    uint32_t fifo;
} __attribute__((packed,aligned(4)));

static volatile struct wb_audio_pcm * const pcm_regs = (void*)(AUDIO_PCM_BASE);

static void
pcm_init(void)
{
    /* Local state */
    memset(&g_pcm, 0x00, sizeof(g_pcm));
}

static int
pcm_level(void)
{
    return (pcm_regs->csr >> 4) & 0xfff;
}


// Audio USB data
// ---------------------------------------------------------------------------

static void
pcm_usb_fill_feedback_ep(void)
{
    /* FIXME figure this out */
#if 0 
    uint32_t val = 8192;

    /* Prepare buffer */
    usb_data_write(64, &val, 4);
    usb_ep_regs[1].in.bd[0].ptr = 64;
    usb_ep_regs[1].in.bd[0].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(3);
#endif
}

static void
pcm_usb_flow_start(void)
{
    /* Reset Buffer index */
    g_pcm.bdi = 0;

    /* EP 1 OUT: Type=Isochronous, dual buffered */
    usb_ep_regs[1].in.status = USB_EP_TYPE_ISOC | USB_EP_BD_DUAL;

    /* EP1 OUT: Queue two buffers */
    usb_ep_regs[1].in.bd[0].ptr = 1024;
    //usb_ep_regs[1].in.bd[0].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(288);
    usb_ep_regs[1].in.bd[0].csr = 0;

    usb_ep_regs[1].in.bd[1].ptr = 1024 + 288;
    //usb_ep_regs[1].in.bd[1].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(288);
    usb_ep_regs[1].in.bd[1].csr = 0;

    /* EP1 IN: Type=Isochronous, single buffered */
    //usb_ep_regs[1].in.status = USB_EP_TYPE_ISOC;

    pcm_usb_fill_feedback_ep();
    pcm_regs->csr = 1;
}

static void
pcm_usb_flow_stop(void)
{
    /* EP 1 OUT: Disable */
    usb_ep_regs[1].in.status = 0;

    /* EP 1 IN: Disable */
    usb_ep_regs[1].in.status = 0;

    /* Stop playing audio */
    pcm_regs->csr = 0;
}

static void
pcm_usb_set_active(bool active)
{
    if (g_pcm.active == active)
        return;

    g_pcm.active = active;

    if (active)
        pcm_usb_flow_start();
    else
        pcm_usb_flow_stop();
}

/*static uint16_t sine[48] = {
    0x8000,0x90b5,0xa120,0xb0fb,0xbfff,0xcdeb,0xda82,0xe58c,
    0xeed9,0xf641,0xfba2,0xfee7,0xffff,0xfee7,0xfba2,0xf641,
    0xeed9,0xe58c,0xda82,0xcdeb,0xbfff,0xb0fb,0xa120,0x90b5,
    0x8000,0x6f4a,0x5edf,0x4f04,0x4000,0x3214,0x257d,0x1a73,
    0x1126,0x9be,0x45d,0x118,0x0,0x118,0x45d,0x9be,
    0x1126,0x1a73,0x257d,0x3214,0x4000,0x4f04,0x5edf,0x6f4a
};*/

static void pcm_poll(void)
{

    /* EP BD Status */
    uint32_t ptr = usb_ep_regs[1].in.bd[g_pcm.bdi].ptr;
    uint32_t csr = usb_ep_regs[1].in.bd[g_pcm.bdi].csr;
    /* Fill BDs */
    while ((csr & USB_BD_STATE_MSK) != USB_BD_STATE_RDY_DATA)
    {
        //int len = 48;
        int len;

        /* Check if we have enough for a packet */
        if (pcm_level() >= 48)
            return;
 
        volatile uint32_t __attribute__((aligned(4))) *dst_u32 = (volatile uint32_t *)((USB_DATA_BASE) + ptr);
 
        len = pcm_level();
        if (len > 72)
            len = 72;
 
        for (int i=0; i<len; i++)
            //*dst_u32++ = 0x5555AAAA; //pcm_regs->fifo;
            //*dst_u16++ = sine[i];
            *dst_u32++ = pcm_regs->fifo;
 
        /* Next transfer */
        usb_ep_regs[1].in.bd[g_pcm.bdi].csr = USB_BD_STATE_RDY_DATA | USB_BD_LEN(len*4);
        g_pcm.bdi ^= 1;

        ptr = usb_ep_regs[1].in.bd[g_pcm.bdi].ptr;
        csr = usb_ep_regs[1].in.bd[g_pcm.bdi].csr;
    }
 
}

// Shared USB driver
// ---------------------------------------------------------------------------

static enum usb_fnd_resp
audio_set_conf(const struct usb_conf_desc *conf)
{
    /* Default PCM interface is inactive */
    pcm_usb_set_active(false);

    return USB_FND_SUCCESS;
}

static enum usb_fnd_resp
audio_set_intf(const struct usb_intf_desc *base, const struct usb_intf_desc *sel)
{
    /* Check it's audio class */
    if (base->bInterfaceClass != 0x01)
        return USB_FND_CONTINUE;

    /* Sub class */
    switch (base->bInterfaceSubClass)
    {
    case USB_AC_SCLS_AUDIOCONTROL:
        return USB_FND_SUCCESS;

    case USB_AC_SCLS_AUDIOSTREAMING:
        pcm_usb_set_active(sel->bAlternateSetting != 0);
        return USB_FND_SUCCESS;

    default:
        //printf("%d\n", base->bInterfaceSubClass);
        return USB_FND_ERROR;
    }
}

static enum usb_fnd_resp
audio_get_intf(const struct usb_intf_desc *base, uint8_t *alt)
{
    /* Check it's audio class */
    if (base->bInterfaceClass != 0x01)
        return USB_FND_CONTINUE;

    /* Sub class */
    switch (base->bInterfaceSubClass)
    {
    case USB_AC_SCLS_AUDIOCONTROL:
        *alt = 0;
        return USB_FND_SUCCESS;

    case USB_AC_SCLS_AUDIOSTREAMING:
        *alt = g_pcm.active ? 1 : 0;
        return USB_FND_SUCCESS;

    default:
        return USB_FND_ERROR;
    }
}

static struct usb_fn_drv _audio_drv = {
    .set_conf = audio_set_conf,
    .set_intf = audio_set_intf,
    .get_intf = audio_get_intf,
};


// Exposed API
// ---------------------------------------------------------------------------

void
audio_init(void)
{
    /* Init hardware */
    printf("Starting audio init\n");
    pcm_init();
    printf("PCM inited\n");
    /* Register function driver */
    usb_register_function_driver(&_audio_drv);
}

void
audio_poll(void)
{
    pcm_poll();
}

void
audio_debug_print(void)
{
    uint32_t csr = pcm_regs->csr;

    printf("Audio PCM tick       : %04x\n", csr >> 16);
    printf("Audio PCM FIFO level : %d\n", (csr >> 4) & 0xfff);
    printf("Audio PCM State      : %d\n", csr & 3);
    printf("FIFO                 : %08x\n", pcm_regs->fifo);
    printf("Active               : %d\n", g_pcm.active);
    printf("CSR running          : %d\n", (csr & 2) >> 1);
    printf("CSR                  : %08x\n", pcm_regs->csr);
}
