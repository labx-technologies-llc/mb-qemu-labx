
/* timer */
static inline DeviceState* altera_timer_create(target_phys_addr_t base, qemu_irq irq, int freq)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "altera,timer");
    qdev_prop_set_uint32(dev, "frequency", freq);
    qdev_init_nofail(dev);
    sysbus_mmio_map(sysbus_from_qdev(dev), 0, base);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, irq);
    return dev;
}

