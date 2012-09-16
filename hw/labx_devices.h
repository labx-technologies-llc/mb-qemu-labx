/*
 * Lab X device types header.
 *
 * Copyright (c) 2010 Lab X Technologies, LLC
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
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#ifndef _LABX_DEVICES_INCLUDED_
#define _LABX_DEVICES_INCLUDED_

#include <net.h>

/* Audio packetizer  */
static inline DeviceState *
labx_audio_packetizer_create(target_phys_addr_t base, qemu_irq irq,
                             int clockDomains, int cacheDataWords)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,audio-packetizer");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_prop_set_uint32(dev, "clockDomains", clockDomains);
    qdev_prop_set_uint32(dev, "cacheDataWords", cacheDataWords);
    qdev_init_nofail(dev);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, irq);
    return dev;
}

/* Audio depacketizer  */
static inline DeviceState *
labx_audio_depacketizer_create(target_phys_addr_t base, qemu_irq irq,
                               int clockDomains, int cacheDataWords, int hasDMA)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,audio-depacketizer");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_prop_set_uint32(dev, "clockDomains", clockDomains);
    qdev_prop_set_uint32(dev, "cacheDataWords", cacheDataWords);
    qdev_prop_set_uint32(dev, "hasDMA", hasDMA);
    qdev_init_nofail(dev);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, irq);
    return dev;
}

/* DMA */
static inline DeviceState *
labx_dma_create(target_phys_addr_t base, int microcodeWords)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,dma");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_prop_set_uint32(dev, "microcodeWords", microcodeWords);
    qdev_init_nofail(dev);
    return dev;
}

/* Ethernet */
static inline DeviceState *
labx_ethernet_create(NICInfo *nd, target_phys_addr_t base, qemu_irq hostIrq,
                     qemu_irq fifoIrq, qemu_irq phyIrq)
{
    DeviceState *dev;
    SysBusDevice *s;

    qemu_check_nic_model(nd, "labx-ethernet");

    dev = qdev_create(NULL, "labx,ethernet");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_set_nic_properties(dev, nd);
    qdev_init_nofail(dev);

    s = sysbus_from_qdev(dev);
    sysbus_connect_irq(s, 0, hostIrq);
    sysbus_connect_irq(s, 1, fifoIrq);
    sysbus_connect_irq(s, 2, phyIrq);

    return dev;
}

/* PTP */
static inline DeviceState *
labx_ptp_create(target_phys_addr_t base)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,ptp");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_init_nofail(dev);
    return dev;
}

#endif /* _LABX_DEVICES_INCLUDED_ */
