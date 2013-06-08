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

/* DMA */
static inline DeviceState *
labx_dma_create(hwaddr base, int microcodeWords)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx.dma");
    qdev_prop_set_uint32(dev, "reg",             base);
    qdev_prop_set_uint32(dev, "microcode-words", microcodeWords);
    qdev_init_nofail(dev);
    return dev;
}

#endif /* _LABX_DEVICES_INCLUDED_ */
