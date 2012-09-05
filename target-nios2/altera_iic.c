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

#include "hw/sysbus.h"
#include "dyngen-exec.h"
#include "cpu.h"

struct altera_iic {
    SysBusDevice busdev;
    Nios2CPU *cpu;
    qemu_irq parent_irq;
};

static void update_irq(struct altera_iic *pv) {
  uint32_t i;

  if ((pv->cpu->env.regs[CR_STATUS] & CR_STATUS_PIE) == 0) {
    qemu_irq_lower(pv->parent_irq);
    return;
  }

  for (i = 0; i < 32; i++) {
    if (pv->cpu->env.regs[CR_IPENDING] & pv->cpu->env.regs[CR_IENABLE] & (1 << i)) break;
  }
  if (i == 32) {
    qemu_irq_lower(pv->parent_irq);
  } else {
    qemu_irq_raise(pv->parent_irq);
  }
}

static void irq_handler(void *opaque, int irq, int level) {
  struct altera_iic *pv = opaque;

  pv->cpu->env.regs[CR_IPENDING] &= ~(1 << irq);
  pv->cpu->env.regs[CR_IPENDING] |= level << irq;

  update_irq(pv);
}

extern Nios2CPU *g_cpu;

static int altera_iic_init(SysBusDevice *dev) {
  struct altera_iic *pv = FROM_SYSBUS(typeof (*pv), dev);
  pv->cpu = g_cpu; // TODO: Get attached CPU instead somehow...

  qdev_init_gpio_in(&dev->qdev, irq_handler, 32);
  sysbus_init_irq(dev, &pv->parent_irq);

  return 0;
}

static Property altera_iic_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void altera_iic_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = altera_iic_init;
    dc->props = altera_iic_properties;
}

static TypeInfo altera_iic_info = {
    .name          = "altera,iic",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct altera_iic),
    .class_init    = altera_iic_class_init,
};

static void altera_iic_register(void) {
    type_register_static(&altera_iic_info);
}

type_init(altera_iic_register)

