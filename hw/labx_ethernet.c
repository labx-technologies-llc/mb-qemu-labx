
/*
 * QEMU model of the LabX legacy ethernet core.
 *
 * Copyright (c) 2010 Lab X Technologies, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "sysbus.h"
#include "sysemu.h"
#include "net.h"

#define FIFO_RAM_BYTES 2048

struct labx_ethernet
{
    SysBusDevice busdev;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    /* Device Configuration */
    uint32_t baseAddress;

    /* Values set by drivers */

    /* Tx buffers */
    uint32_t* txBuffer;

    /* Rx buffers */
    uint32_t* rxBuffer;
};

/*
 * Legacy ethernet registers
 */
static uint32_t ethernet_regs_readl (void *opaque, target_phys_addr_t addr)
{
    //struct labx_ethernet *p = opaque;

    uint32_t retval = 0;
   
    switch ((addr>>2) & 0x0F)
    {
    	case 0x00: // mdio control
            break;

        case 0x01: // mdio data
            break;

        case 0x02: // irq mask
            break;

        case 0x03: // irq flags
            break;

        case 0x04: // vlan mask
            break;

        case 0x0F: // revision
                retval = 0x00000010;
                break;

        default:
            printf("labx-ethernet: Read of unknown register %08X\n", addr);
            break;
    }

    return retval;
}

static void ethernet_regs_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    //struct labx_ethernet *p = opaque;

    switch ((addr>>2) & 0x0F)
    {
    	case 0x00: // mdio control
            break;

        case 0x01: // mdio data
            break;

        case 0x02: // irq mask
            break;

        case 0x03: // irq flags
            break;

        case 0x04: // vlan mask
            break;

        case 0x0F: // revision
            break;

        default:
            printf("labx-ethernet: Write of unknown register %08X = %08X\n", addr, value);
            break;
    }
}

static CPUReadMemoryFunc * const ethernet_regs_read[] = {
    NULL, NULL,
    &ethernet_regs_readl,
};

static CPUWriteMemoryFunc * const ethernet_regs_write[] = {
    NULL, NULL,
    &ethernet_regs_writel,
};


/*
 * MAC registers
 */
static uint32_t mac_regs_readl (void *opaque, target_phys_addr_t addr)
{
    //struct labx_ethernet *p = opaque;

    uint32_t retval = 0;
   
    switch ((addr>>2) & 0x0F)
    {
    	case 0x01: // host rx config
            break;

        case 0x02: // host tx config
            break;

        case 0x04: // host speed config
            break;

        case 0x05: // host mdio config
            break;

        default:
            printf("labx-ethernet: Read of unknown mac register %08X\n", addr);
            break;
    }

    return retval;
}

static void mac_regs_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    //struct labx_ethernet *p = opaque;

    switch ((addr>>2) & 0x0F)
    {
    	case 0x01: // host rx config
            break;

        case 0x02: // host tx config
            break;

        case 0x04: // host speed config
            break;

        case 0x05: // host mdio config
            break;

        default:
            printf("labx-ethernet: Write of unknown mac register %08X = %08X\n", addr, value);
            break;
    }
}

static CPUReadMemoryFunc * const mac_regs_read[] = {
    NULL, NULL,
    &mac_regs_readl,
};

static CPUWriteMemoryFunc * const mac_regs_write[] = {
    NULL, NULL,
    &mac_regs_writel,
};


/*
 * FIFO registers
 */
static uint32_t fifo_regs_readl (void *opaque, target_phys_addr_t addr)
{
    //struct labx_ethernet *p = opaque;

    uint32_t retval = 0;
   
    switch ((addr>>2) & 0x0F)
    {
    	case 0x00: // fifo int status
            break;

        case 0x01: // fifo int enable
            break;

        case 0x02: // fifo tx reset
            break;

        case 0x03: // fifo tx vacancy
            break;

        case 0x04: // fifo tx data
            break;

        case 0x05: // fifo tx length
            break;

        case 0x06: // fifo rx reset
            break;

        case 0x07: // fifo rx occupancy
            break;

        case 0x08: // fifo rx data
            break;

        case 0x09: // fifo rx length
            break;

        default:
            printf("labx-ethernet: Read of unknown fifo register %08X\n", addr);
            break;
    }

    return retval;
}

static void fifo_regs_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    //struct labx_ethernet *p = opaque;

    switch ((addr>>2) & 0x0F)
    {
    	case 0x00: // fifo int status
            break;

        case 0x01: // fifo int enable
            break;

        case 0x02: // fifo tx reset
            break;

        case 0x03: // fifo tx vacancy
            break;

        case 0x04: // fifo tx data
            break;

        case 0x05: // fifo tx length
            break;

        case 0x06: // fifo rx reset
            break;

        case 0x07: // fifo rx occupancy
            break;

        case 0x08: // fifo rx data
            break;

        case 0x09: // fifo rx length
            break;

        default:
            printf("labx-ethernet: Write of unknown fifo register %08X = %08X\n", addr, value);
            break;
    }
}

static CPUReadMemoryFunc * const fifo_regs_read[] = {
    NULL, NULL,
    &fifo_regs_readl,
};

static CPUWriteMemoryFunc * const fifo_regs_write[] = {
    NULL, NULL,
    &fifo_regs_writel,
};


static int eth_can_rx(VLANClientState *nc)
{
    //struct labx_ethernet *s = DO_UPCAST(NICState, nc, nc)->opaque;

    return 0;
}

static ssize_t eth_rx(VLANClientState *nc, const uint8_t *buf, size_t size)
{
    // struct labx_ethernet *s = DO_UPCAST(NICState, nc, nc)->opaque;

    return -1;
}

static void eth_cleanup(VLANClientState *nc)
{
    struct labx_ethernet *s = DO_UPCAST(NICState, nc, nc)->opaque;

    s->nic = NULL;
}

static NetClientInfo net_labx_ethernet_info = {
    .type = NET_CLIENT_TYPE_NIC,
    .size = sizeof(NICState),
    .can_receive = eth_can_rx,
    .receive = eth_rx,
    .cleanup = eth_cleanup,
};

static int labx_ethernet_init(SysBusDevice *dev)
{
    struct labx_ethernet *p = FROM_SYSBUS(typeof (*p), dev);
    int ethernet_regs;
    int mac_regs;
    int fifo_regs;

    /* Initialize defaults */
    p->txBuffer = qemu_malloc(FIFO_RAM_BYTES);
    p->rxBuffer = qemu_malloc(FIFO_RAM_BYTES);

    /* Set up memory regions */
    ethernet_regs = cpu_register_io_memory(ethernet_regs_read, ethernet_regs_write, p);
    mac_regs = cpu_register_io_memory(mac_regs_read, mac_regs_write, p);
    fifo_regs = cpu_register_io_memory(fifo_regs_read, fifo_regs_write, p);

    sysbus_init_mmio(dev, 0x10 * 4, ethernet_regs);
    sysbus_init_mmio(dev, 0x10 * 4, mac_regs);
    sysbus_init_mmio(dev, 0x10 * 4, fifo_regs);

    sysbus_mmio_map(dev, 0, p->baseAddress);
    sysbus_mmio_map(dev, 1, p->baseAddress + (1 << (9+2)));
    sysbus_mmio_map(dev, 2, p->baseAddress + (2 << (9+2)));

    /* Initialize the irq */
    sysbus_init_irq(dev, &p->irq);

    /* Set up the NIC */
    qemu_macaddr_default_if_unset(&p->conf.macaddr);
    p->nic = qemu_new_nic(&net_labx_ethernet_info, &p->conf,
                          dev->qdev.info->name, dev->qdev.id, p);
    qemu_format_nic_info_str(&p->nic->nc, p->conf.macaddr.a);
    return 0;
}

static SysBusDeviceInfo labx_ethernet_info = {
    .init = labx_ethernet_init,
    .qdev.name  = "labx,ethernet",
    .qdev.size  = sizeof(struct labx_ethernet),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("baseAddress", struct labx_ethernet, baseAddress, 0),
        DEFINE_NIC_PROPERTIES(struct labx_ethernet, conf),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void labx_ethernet_register(void)
{
    sysbus_register_withprop(&labx_ethernet_info);
}

device_init(labx_ethernet_register)

