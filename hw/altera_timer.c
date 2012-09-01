/*
 * QEMU model of the Altera timer.
 *
 * Copyright (c) 2012 Chris Wulff
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 */

#include "sysbus.h"
#include "sysemu.h"
#include "qemu-timer.h"

#define R_STATUS     0
#define R_CONTROL    1
#define R_PERIODL    2
#define R_PERIODH    3
#define R_SNAPL      4
#define R_SNAPH      5
#define R_MAX        6

#define STATUS_TO  0x0001
#define STATUS_RUN 0x0002

#define CONTROL_ITO   0x0001
#define CONTROL_CONT  0x0002
#define CONTROL_START 0x0004
#define CONTROL_STOP  0x0008

struct altera_timer {
  SysBusDevice  busdev;
  qemu_irq      irq;
  uint32_t      freq_hz;
  QEMUBH       *bh;
  ptimer_state *ptimer;
  uint32_t      regs[R_MAX];
};

static uint32_t timer_readl(void *opaque, target_phys_addr_t addr) {
  struct altera_timer *t = opaque;
  uint32_t r = 0;

  addr >>= 2;
  addr &= 0x7;
  switch (addr) {
    case R_STATUS:
      r = t->regs[R_STATUS];
      break;

    default:
      if (addr < ARRAY_SIZE(t->regs)) {
        r = t->regs[addr];
      }
      break;
  }

  qemu_set_irq(t->irq, t->regs[R_STATUS] & t->regs[R_CONTROL] & CONTROL_ITO);

  return r;
}

static void timer_start(struct altera_timer *t) {
  ptimer_stop(t->ptimer);
  ptimer_set_count(t->ptimer, (t->regs[R_PERIODH]<<16) | t->regs[R_PERIODL]);
  ptimer_run(t->ptimer, 1);
}

static void timer_writel(void *opaque, target_phys_addr_t addr, uint32_t value)
{
  struct altera_timer *t = opaque;
  uint32_t count = 0;

  addr >>= 2;
  addr &= 0x7;
  switch (addr) {
    case R_STATUS:
      /* Writing zero clears the timeout */
      t->regs[R_STATUS] &= ~STATUS_TO;
      break;

    case R_CONTROL:
      t->regs[R_CONTROL] = value & (CONTROL_ITO | CONTROL_CONT);
      if ((value & CONTROL_START) && ((t->regs[R_STATUS] & STATUS_RUN) == 0)) {
        timer_start(t);
      }
      if ((value & CONTROL_STOP) && (t->regs[R_STATUS] & STATUS_RUN)) {
        ptimer_stop(t->ptimer);
      }
      break;

    case R_PERIODL:
    case R_PERIODH:
      t->regs[addr] = value & 0xFFFF;
      if (t->regs[R_STATUS] & STATUS_RUN) {
        timer_start(t);
      }
      break;

    case R_SNAPL:
    case R_SNAPH:
      count = ptimer_get_count(t->ptimer);
      t->regs[R_SNAPL] = count & 0xFFFF;
      t->regs[R_SNAPH] = (count>>16) & 0xFFFF;
      break;
 
    default:
      if (addr < ARRAY_SIZE(t->regs)) {
        t->regs[addr] = value;
      }
      break;
  }

  qemu_set_irq(t->irq, t->regs[R_STATUS] & t->regs[R_CONTROL] & CONTROL_ITO);
}

static CPUReadMemoryFunc * const timer_read[] = {
  &timer_readl,
  &timer_readl,
  &timer_readl,
};

static CPUWriteMemoryFunc * const timer_write[] = {
  &timer_writel,
  &timer_writel,
  &timer_writel,
};

static void timer_hit(void *opaque) {
  struct altera_timer *t = opaque;
  t->regs[R_STATUS] |= STATUS_TO;

  if (t->regs[R_CONTROL] & CONTROL_CONT) {
    timer_start(t);
  }
  qemu_set_irq(t->irq, t->regs[R_STATUS] & t->regs[R_CONTROL] & CONTROL_ITO);
}

static int altera_timer_init(SysBusDevice *dev) {
  struct altera_timer *t = FROM_SYSBUS(typeof (*t), dev);
  int timer_regs;

  sysbus_init_irq(dev, &t->irq);

  t->bh = qemu_bh_new(timer_hit, t);
  t->ptimer = ptimer_init(t->bh);
  ptimer_set_freq(t->ptimer, t->freq_hz);

  timer_regs = cpu_register_io_memory(timer_read, timer_write, t);
  sysbus_init_mmio(dev, R_MAX * 4, timer_regs);
  return 0;
}

static SysBusDeviceInfo altera_timer_info = {
  .init = altera_timer_init,
  .qdev.name  = "altera,timer",
  .qdev.size  = sizeof(struct altera_timer),
  .qdev.props = (Property[]) {
    DEFINE_PROP_UINT32("frequency", struct altera_timer, freq_hz, 0),
    DEFINE_PROP_END_OF_LIST(),
  }
};

static void altera_timer_register(void)
{
    sysbus_register_withprop(&altera_timer_info);
}

device_init(altera_timer_register)
