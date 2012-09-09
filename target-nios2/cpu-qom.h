/*
 * QEMU Nios II CPU
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
#ifndef QEMU_NIOS2_CPU_QOM_H
#define QEMU_NIOS2_CPU_QOM_H

#include "qemu/cpu.h"

#define TYPE_NIOS2_CPU "nios2-cpu"

#define NIOS2_CPU_CLASS(klass) \
    OBJECT_CLASS_CHECK(Nios2CPUClass, (klass), TYPE_NIOS2_CPU)
#define NIOS2_CPU(obj) \
    OBJECT_CHECK(Nios2CPU, (obj), TYPE_NIOS2_CPU)
#define NIOS2_CPU_GET_CLASS(obj) \
    OBJECT_GET_CLASS(Nios2CPUClass, (obj), TYPE_NIOS2_CPU)

/**
 * Nios2CPUClass:
 * @parent_reset: The parent class' reset handler.
 *
 * A Nios2 CPU model.
 */
typedef struct Nios2CPUClass {
    /*< private >*/
    CPUClass parent_class;
    /*< public >*/

    void (*parent_reset)(CPUState *cpu);
} Nios2CPUClass;

/**
 * Nios2CPU:
 * @env: #CPUNios2State
 *
 * A Nios2 CPU.
 */
typedef struct Nios2CPU {
    /*< private >*/
    CPUState parent_obj;
    /*< public >*/

    CPUNios2State env;
} Nios2CPU;

static inline Nios2CPU *nios2_env_get_cpu(CPUNios2State *env)
{
    return NIOS2_CPU(container_of(env, Nios2CPU, env));
}

#define ENV_GET_CPU(e) CPU(nios2_env_get_cpu(e))

#endif /* QEMU_NIOS2_CPU_QOM_H */
