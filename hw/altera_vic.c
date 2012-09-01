/*
 * QEMU Altera Vectored Interrupt Controller.
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

#define R_INT_CONFIG_0      0
#define R_INT_ENABLE       32
#define R_INT_ENABLE_SET   33
#define R_INT_ENABLE_CLR   34
#define R_INT_PENDING      35
#define R_INT_RAW_STATUS   36
#define R_SW_INTERRUPT     37
#define R_SW_INTERRUPT_SET 38
#define R_SW_INTERRUPT_CLR 39
#define R_VIC_CONFIG       40
#define R_VIC_STATUS       41
#define R_VEC_TBL_BASE     42
#define R_VEC_TBL_ADDR     43
#define R_MAX              44

struct altera_vic {
    SysBusDevice busdev;
    qemu_irq parent_irq;

    /* Runtime control registers.  */
    uint32_t regs[R_MAX];
};

static void update_irq(struct altera_vic *pv) {
  uint32_t i;
  pv->regs[R_INT_PENDING] = (pv->regs[R_INT_RAW_STATUS] | pv->regs[R_SW_INTERRUPT]) & pv->regs[R_INT_ENABLE];

  for (i = 0; i < 32; i++) {
    if (pv->regs[R_INT_PENDING] & (1 << i)) break;
  }
  if (i == 32) {
    pv->regs[R_VEC_TBL_ADDR] = 0;
    pv->regs[R_VIC_STATUS] = 0;
    qemu_irq_lower(pv->parent_irq);
  } else {
    pv->regs[R_VEC_TBL_ADDR] = pv->regs[R_VEC_TBL_BASE] + i*(4 << (pv->regs[R_VIC_CONFIG] & 7));
    pv->regs[R_VIC_STATUS] = 0x80000000 | i;
    qemu_irq_raise(pv->parent_irq);
  }
}

static uint32_t pic_readl(void *opaque, target_phys_addr_t addr) {
  struct altera_vic *pv = opaque;
  uint32_t r = 0;

  addr >>= 2;
  if (addr < R_MAX) {
    r = pv->regs[addr];
  }

  return r;
}

static void pic_writel(void *opaque, target_phys_addr_t addr, uint32_t value)
{
  struct altera_vic *pv = opaque;

  addr >>= 2;
  if (addr < R_INT_ENABLE) {
    // R_INT_CONFIG_XX
    pv->regs[addr] = value & 0x00001FFF;
  } else {
    switch (addr) {
      case R_INT_PENDING:
      case R_INT_RAW_STATUS:
      case R_VIC_STATUS:
      case R_VEC_TBL_ADDR:
        /* read only */
        break;
      case R_INT_ENABLE_SET:
        pv->regs[R_INT_ENABLE] |= value;
	break;
      case R_SW_INTERRUPT_SET:
        pv->regs[R_SW_INTERRUPT] |= value;
	break;
      case R_INT_ENABLE_CLR:
	pv->regs[R_INT_ENABLE] &= ~value;
	break;
      case R_SW_INTERRUPT_CLR:
	pv->regs[R_SW_INTERRUPT] &= ~value;
	break;
      case R_VIC_CONFIG:
	pv->regs[addr] = value & 0x0000000F;
	break;
      default:
        if (addr < ARRAY_SIZE(pv->regs)) {
          pv->regs[addr] = value;
	}
        break;
    }
  }
  update_irq(pv);
}

static CPUReadMemoryFunc * const pic_read[] = {
    NULL, NULL,
    &pic_readl,
};

static CPUWriteMemoryFunc * const pic_write[] = {
    NULL, NULL,
    &pic_writel,
};

static void irq_handler(void *opaque, int irq, int level) {
  struct altera_vic *pv = opaque;

  pv->regs[R_INT_RAW_STATUS] &= ~(1 << irq);
  pv->regs[R_INT_RAW_STATUS] |= level << irq;

  update_irq(pv);
}

static int altera_vic_init(SysBusDevice *dev) {
  struct altera_vic *pv = FROM_SYSBUS(typeof (*pv), dev);
  int pic_regs;

  qdev_init_gpio_in(&dev->qdev, irq_handler, 32);
  sysbus_init_irq(dev, &pv->parent_irq);

  memset(pv->regs, 0, sizeof(uint32_t) * R_MAX);
  pic_regs = cpu_register_io_memory(pic_read, pic_write, pv);
  sysbus_init_mmio(dev, R_MAX * sizeof(uint32_t), pic_regs);
  return 0;
}

static SysBusDeviceInfo altera_vic_info = {
  .init = altera_vic_init,
  .qdev.name  = "altera,vic",
  .qdev.size  = sizeof(struct altera_vic),
  .qdev.props = (Property[]) {
    DEFINE_PROP_END_OF_LIST(),
  }
};

static void altera_vic_register(void) {
  sysbus_register_withprop(&altera_vic_info);
}

device_init(altera_vic_register)

