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

#include "cpu.h"
#include "qemu-common.h"


/* CPUClass::reset() */
static void nios2_cpu_reset(CPUState *s)
{
    Nios2CPU *cpu = NIOS2_CPU(s);
    Nios2CPUClass *mcc = NIOS2_CPU_GET_CLASS(cpu);
    CPUNios2State *env = &cpu->env;

    if (qemu_loglevel_mask(CPU_LOG_RESET)) {
        qemu_log("CPU Reset (CPU %d)\n", s->cpu_index);
        log_cpu_state(env, 0);
    }

    mcc->parent_reset(s);

    tlb_flush(env, 1);

    memset(env->regs, 0, sizeof(uint32_t) * NUM_CORE_REGS);
    env->regs[R_PC] = env->reset_addr;

#if defined(CONFIG_USER_ONLY)
    /* start in user mode with interrupts enabled.  */
    env->regs[CR_STATUS] = CR_STATUS_U | CR_STATUS_PIE;
#else
    mmu_init(&env->mmu);
#endif
}

static void nios2_cpu_initfn(Object *obj)
{
    CPUState *cs = CPU(obj);
    Nios2CPU *cpu = NIOS2_CPU(obj);
    CPUNios2State *env = &cpu->env;

    cs->env_ptr = env;
    cpu_exec_init(env);
}

static void nios2_cpu_class_init(ObjectClass *oc, void *data)
{
    CPUClass *cc = CPU_CLASS(oc);
    Nios2CPUClass *mcc = NIOS2_CPU_CLASS(oc);

    mcc->parent_reset = cc->reset;
    cc->reset = nios2_cpu_reset;
    cc->do_interrupt = nios2_cpu_do_interrupt;
}

static const TypeInfo nios2_cpu_type_info = {
    .name = TYPE_NIOS2_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(Nios2CPU),
    .instance_init = nios2_cpu_initfn,
    .class_size = sizeof(Nios2CPUClass),
    .class_init = nios2_cpu_class_init,
};

static void nios2_cpu_register_types(void)
{
    type_register_static(&nios2_cpu_type_info);
}

type_init(nios2_cpu_register_types)
