/*
 * QEMU Altera Nios II CPU interrupt wrapper logic.
 *
 * Copyright (c) 2012 Chris Wulff <crwulff@gmail.com>
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

#include "hw.h"
#include "pc.h"
#include "nios2.h"

void pic_info(Monitor *mon)
{
}

void irq_info(Monitor *mon)
{
}

static void nios2_pic_cpu_handler(void *opaque, int irq, int level)
{
    CPUNios2State *env = (CPUNios2State *)opaque;
    int type = irq ? CPU_INTERRUPT_NMI : CPU_INTERRUPT_HARD;

    if (level) {
        cpu_interrupt(env, type);
    } else {
        cpu_reset_interrupt(env, type);
    }
}

qemu_irq *nios2_pic_init_cpu(CPUNios2State *env)
{
    return qemu_allocate_irqs(nios2_pic_cpu_handler, env, 2);
}
