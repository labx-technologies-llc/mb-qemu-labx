
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
static uint32_t ptp_regs_readl (void *opaque, target_phys_addr_t addr)
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

static void ptp_regs_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    //struct labx_ptp *p = opaque;

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

static CPUReadMemoryFunc * const ptp_regs_read[] = {
    NULL, NULL,
    &ptp_regs_readl,
};

static CPUWriteMemoryFunc * const ptp_regs_write[] = {
    NULL, NULL,
    &ptp_regs_writel,
};


/*
 * Tx Ram
 */
static uint32_t tx_ram_readl (void *opaque, target_phys_addr_t addr)
{
    struct labx_ptp *p = opaque;

    return p->txRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)];
}

static void tx_ram_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct labx_ptp *p = opaque;

    p->txRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)] = value;
}

static CPUReadMemoryFunc * const tx_ram_read[] = {
    NULL, NULL,
    &tx_ram_readl,
};

static CPUWriteMemoryFunc * const tx_ram_write[] = {
    NULL, NULL,
    &tx_ram_writel,
};

/*
 * Rx Ram
 */
static uint32_t rx_ram_readl (void *opaque, target_phys_addr_t addr)
{
    struct labx_ptp *p = opaque;

    return p->rxRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)];
}

static void rx_ram_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct labx_ptp *p = opaque;

    p->rxRam[RAM_INDEX(addr, PTP_HOST_RAM_WORDS)] = value;
}

static CPUReadMemoryFunc * const rx_ram_read[] = {
    NULL, NULL,
    &rx_ram_readl,
};

static CPUWriteMemoryFunc * const rx_ram_write[] = {
    NULL, NULL,
    &rx_ram_writel,
};

static int labx_ptp_init(SysBusDevice *dev)
{
    struct labx_ptp *p = FROM_SYSBUS(typeof (*p), dev);
    int ptp_regs;
    int tx_ram;
    int rx_ram;

    /* Initialize defaults */
    p->txRam = qemu_malloc(PTP_RAM_BYTES);
    p->rxRam = qemu_malloc(PTP_RAM_BYTES);

    /* Set up memory regions */
    ptp_regs = cpu_register_io_memory(ptp_regs_read, ptp_regs_write, p);
    tx_ram = cpu_register_io_memory(tx_ram_read, tx_ram_write, p);
    rx_ram = cpu_register_io_memory(rx_ram_read, rx_ram_write, p);

    sysbus_init_mmio(dev, 0x100 * 4, ptp_regs);
    sysbus_init_mmio(dev, PTP_RAM_BYTES, tx_ram);
    sysbus_init_mmio(dev, PTP_RAM_BYTES, rx_ram);

    sysbus_mmio_map(dev, 0, p->baseAddress);
    sysbus_mmio_map(dev, 1, p->baseAddress + (1 << min_bits(PTP_RAM_BYTES-1)));
    sysbus_mmio_map(dev, 2, p->baseAddress + (2 << min_bits(PTP_RAM_BYTES-1)));

    return 0;
}

static SysBusDeviceInfo labx_ptp_info = {
    .init = labx_ptp_init,
    .qdev.name  = "labx,ptp",
    .qdev.size  = sizeof(struct labx_ptp),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("baseAddress",        struct labx_ptp, baseAddress,        0),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void labx_ptp_register(void)
{
    sysbus_register_withprop(&labx_ptp_info);
}

device_init(labx_ptp_register)

