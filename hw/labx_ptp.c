
/*
 * QEMU model of the LabX PTP.
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

#define min_bits qemu_fls
#define RAM_INDEX(addr, size) (((addr)>>2)&((1<<min_bits((size)-1))-1))

#define PTP_MAX_PACKETS      8
#define PTP_MAX_PACKET_BYTES 256
#define PTP_RAM_BYTES        (PTP_MAX_PACKETS*PTP_MAX_PACKET_BYTES)
#define PTP_HOST_RAM_WORDS   (PTP_RAM_BYTES/4)

struct labx_ptp
{
    SysBusDevice busdev;

    MemoryRegion  mmio_ptp;
    MemoryRegion  mmio_tx;
    MemoryRegion  mmio_rx;

    /* Device Configuration */
    uint32_t baseAddress;

    /* Values set by drivers */

    /* Tx buffers */
    uint32_t* txRam;

    /* Rx buffers */
    uint32_t* rxRam;
};

/*
 * PTP registers
 */
static uint64_t ptp_regs_read(void *opaque, target_phys_addr_t addr, unsigned int size)
{
    //struct labx_ptp *p = opaque;

    uint32_t retval = 0;
   
    switch ((addr>>2) & 0x0F)
    {
    	case 0x00: // rx
            break;

        case 0x01: // tx
            break;

        case 0x02: // irq mask
            break;

        case 0x03: // irq flags
            break;

        case 0x04: // rtc increment
            break;

        case 0x05: // seconds high
            break;

        case 0x06: // seconds low
            break;

        case 0x07: // nanoseconds
            break;

        case 0x08: // timer
            break;

        case 0x09: // local seconds high
            break;

        case 0x0A: // local seconds low
            break;

        case 0x0B: // local nanoseconds
            break;

        case 0x0F: // revision
                retval = 0x00000111; // Report 1 port, revision 1.1
                break;

        default:
            printf("labx-ptp: Read of unknown register %08X\n", addr);
            break;
    }

    return retval;
}

static void ptp_regs_write(void *opaque, target_phys_addr_t addr, uint64_t val64, unsigned int size)
{
    //struct labx_ptp *p = opaque;
    uint32_t value = val64;

    switch ((addr>>2) & 0x0F)
    {
    	case 0x00: // rx
            break;

        case 0x01: // tx
            break;

        case 0x02: // irq mask
            break;

        case 0x03: // irq flags
            break;

        case 0x04: // rtc increment
            break;

        case 0x05: // seconds high
            break;

        case 0x06: // seconds low
            break;

        case 0x07: // nanoseconds
            break;

        case 0x08: // timer
            break;

        case 0x09: // local seconds high
            break;

        case 0x0A: // local seconds low
            break;

        case 0x0B: // local nanoseconds
            break;

        case 0x0F: // revision
            break;

        default:
            printf("labx-ptp: Write of unknown register %08X = %08X\n", addr, value);
            break;
    }
}

static const MemoryRegionOps ptp_regs_ops = {
    .read = ptp_regs_read,
    .write = ptp_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};


/*
 * Tx Ram
 */
static uint64_t tx_ram_read(void *opaque, target_phys_addr_t addr, unsigned int size)
{
    struct labx_ptp *p = opaque;

    return p->txRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)];
}

static void tx_ram_write(void *opaque, target_phys_addr_t addr, uint64_t val64, unsigned int size)
{
    struct labx_ptp *p = opaque;
    uint32_t value = val64;

    p->txRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)] = value;
}

static const MemoryRegionOps tx_ram_ops = {
    .read = tx_ram_read,
    .write = tx_ram_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};


/*
 * Rx Ram
 */
static uint64_t rx_ram_read(void *opaque, target_phys_addr_t addr, unsigned int size)
{
    struct labx_ptp *p = opaque;

    return p->rxRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)];
}

static void rx_ram_write(void *opaque, target_phys_addr_t addr, uint64_t val64, unsigned int size)
{
    struct labx_ptp *p = opaque;
    uint32_t value = val64;

    p->rxRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)] = value;
}

static const MemoryRegionOps rx_ram_ops = {
    .read = rx_ram_read,
    .write = rx_ram_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};


static int labx_ptp_init(SysBusDevice *dev)
{
    struct labx_ptp *p = FROM_SYSBUS(typeof (*p), dev);

    /* Initialize defaults */
    p->txRam = g_malloc0(PTP_RAM_BYTES);
    p->rxRam = g_malloc0(PTP_RAM_BYTES);

    /* Set up memory regions */
    memory_region_init_io(&p->mmio_ptp, &ptp_regs_ops, p, "labx,ptp-regs", 0x100 * 4);
    memory_region_init_io(&p->mmio_tx,  &tx_ram_ops,   p, "labx,ptp-tx",   PTP_RAM_BYTES);
    memory_region_init_io(&p->mmio_rx,  &rx_ram_ops,   p, "labx,ptp-rx",   PTP_RAM_BYTES);

    sysbus_init_mmio(dev, &p->mmio_ptp);
    sysbus_init_mmio(dev, &p->mmio_tx);
    sysbus_init_mmio(dev, &p->mmio_rx);

    sysbus_mmio_map(dev, 0, p->baseAddress);
    sysbus_mmio_map(dev, 1, p->baseAddress + (1 << min_bits(PTP_RAM_BYTES-1)));
    sysbus_mmio_map(dev, 2, p->baseAddress + (2 << min_bits(PTP_RAM_BYTES-1)));

    return 0;
}

static Property labx_ptp_properties[] = {
    DEFINE_PROP_UINT32("baseAddress",        struct labx_ptp, baseAddress,        0),
    DEFINE_PROP_END_OF_LIST(),
};

static void labx_ptp_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = labx_ptp_init;
    dc->props = labx_ptp_properties;
}

static TypeInfo labx_ptp_info = {
    .name          = "labx,ptp",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct labx_ptp),
    .class_init    = labx_ptp_class_init,
};

static void labx_ptp_register(void) {
    type_register_static(&labx_ptp_info);
}

type_init(labx_ptp_register)

