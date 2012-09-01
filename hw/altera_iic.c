/*
 * QEMU Altera Internal Interrupt Controller.
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
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "sysbus.h"
#include "hw.h"

extern CPUState *env;

struct altera_iic {
    SysBusDevice busdev;
    qemu_irq parent_irq;
};

static void update_irq(struct altera_iic *pv) {
  uint32_t i;

  if ((env->regs[CR_STATUS] & CR_STATUS_PIE) == 0) {
    qemu_irq_lower(pv->parent_irq);
    return;
  }

  for (i = 0; i < 32; i++) {
    if (env->regs[CR_IPENDING] & env->regs[CR_IENABLE] & (1 << i)) break;
  }
  if (i == 32) {
    qemu_irq_lower(pv->parent_irq);
  } else {
    qemu_irq_raise(pv->parent_irq);
  }
}

static void irq_handler(void *opaque, int irq, int level) {
  struct altera_iic *pv = opaque;

  env->regs[CR_IPENDING] &= ~(1 << irq);
  env->regs[CR_IPENDING] |= level << irq;

  update_irq(pv);
}

static int altera_iic_init(SysBusDevice *dev) {
  struct altera_iic *pv = FROM_SYSBUS(typeof (*pv), dev);

  qdev_init_gpio_in(&dev->qdev, irq_handler, 32);
  sysbus_init_irq(dev, &pv->parent_irq);

  return 0;
}

static SysBusDeviceInfo altera_iic_info = {
  .init = altera_iic_init,
  .qdev.name  = "altera,iic",
  .qdev.size  = sizeof(struct altera_iic),
  .qdev.props = (Property[]) {
    DEFINE_PROP_END_OF_LIST(),
  }
};

static void altera_iic_register(void) {
  sysbus_register_withprop(&altera_iic_info);
}

device_init(altera_iic_register)

