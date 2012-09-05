
#include "sysbus.h"

qemu_irq *nios2_pic_init_cpu(CPUNios2State *env);

static inline DeviceState *
altera_vic_create(target_phys_addr_t base, qemu_irq irq, int kind_of_intr)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "altera,vic");
    qdev_init_nofail(dev);
    sysbus_mmio_map(sysbus_from_qdev(dev), 0, base);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, irq);
    return dev;
}

static inline DeviceState *
altera_iic_create(qemu_irq irq, int kind_of_intr)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "altera,iic");
    qdev_init_nofail(dev);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, irq);
    return dev;
}
