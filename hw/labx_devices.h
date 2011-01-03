
/* Audio packetizer  */
static inline DeviceState *
labx_audio_packetizer_create(target_phys_addr_t base, qemu_irq irq, int clockDomains, int cacheDataWords)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,audio-packetizer");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_prop_set_uint32(dev, "clockDomains", clockDomains);
    qdev_prop_set_uint32(dev, "cacheDataWords", cacheDataWords);
    qdev_init_nofail(dev);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, irq);
    return dev;
}
