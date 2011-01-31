
#include <net.h>

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

/* Audio depacketizer  */
static inline DeviceState *
labx_audio_depacketizer_create(target_phys_addr_t base, qemu_irq irq, int clockDomains, int cacheDataWords, int hasDMA)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,audio-depacketizer");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_prop_set_uint32(dev, "clockDomains", clockDomains);
    qdev_prop_set_uint32(dev, "cacheDataWords", cacheDataWords);
    qdev_prop_set_uint32(dev, "hasDMA", hasDMA);
    qdev_init_nofail(dev);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, irq);
    return dev;
}

/* DMA */
static inline DeviceState *
labx_dma_create(target_phys_addr_t base, int microcodeWords)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,dma");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_prop_set_uint32(dev, "microcodeWords", microcodeWords);
    qdev_init_nofail(dev);
    return dev;
}

/* Ethernet */
static inline DeviceState *
labx_ethernet_create(NICInfo *nd, target_phys_addr_t base, qemu_irq hostIrq, qemu_irq fifoIrq, qemu_irq phyIrq)
{
    DeviceState *dev;
    SysBusDevice *s;

    qemu_check_nic_model(nd, "labx-ethernet");

    dev = qdev_create(NULL, "labx,ethernet");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_set_nic_properties(dev, nd);
    qdev_init_nofail(dev);

    s = sysbus_from_qdev(dev);
    sysbus_connect_irq(s, 0, hostIrq);
    sysbus_connect_irq(s, 1, fifoIrq);
    sysbus_connect_irq(s, 2, phyIrq);
    
    return dev;
}

/* PTP */
static inline DeviceState *
labx_ptp_create(target_phys_addr_t base)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "labx,ptp");
    qdev_prop_set_uint32(dev, "baseAddress", base);
    qdev_init_nofail(dev);
    return dev;
}

