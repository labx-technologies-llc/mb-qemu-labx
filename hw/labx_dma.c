
/*
 * QEMU model of the LabX DMA Engine.
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


struct labx_dma
{
    SysBusDevice busdev;

    /* Device Configuration */
    uint32_t baseAddress;
    uint32_t paramWords;
    uint32_t microcodeWords;
    uint32_t numIndexRegs;
    uint32_t numChannels;
    uint32_t numAlus;

    /* Values set by drivers */

    /* Microcode buffer */
    uint32_t* microcodeRam;
};

/*
 * DMA registers
 */
static uint32_t dma_regs_readl (void *opaque, target_phys_addr_t addr)
{
    struct labx_dma *p = opaque;

    uint32_t retval = 0;
   
    if ((addr>>2) & 0x80)
    {
        // vector
    } 
    else
    {
        switch ((addr>>2) & 0x7F)
        {
            case 0x00: // control
                break;

            case 0x01: // channel enable
                break;

            case 0x02: // channel start
                break;

            case 0x03: // channel irq enable
                break;

            case 0x04: // channel irq
                break;

            case 0x05: // sync
                break;

            case 0x7E: // capabilities
                retval = ((p->numIndexRegs               & 0x0F) << 12) |
                         ((p->numChannels                & 0x03) << 10) |
                         ((p->numAlus                    & 0x03) << 8) |
                         ((min_bits(p->paramWords-1)     & 0x0F) << 4) |
                         ((min_bits(p->microcodeWords-1) & 0x0F));
                break;

            case 0x7F: // revision
                retval = 0x00000011;
                break;

            default:
                printf("labx-dma: Read of unknown register %08X\n", addr);
                break;
        }
    }

    return retval;
}

static void dma_regs_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    //struct labx_dma *p = opaque;

    if ((addr>>2) & 0x80)
    {
        // vector
    } 
    else
    {
        switch ((addr>>2) & 0x7F)
        {
            case 0x00: // control
                break;

            case 0x01: // channel enable
                break;

            case 0x02: // channel start
                break;

            case 0x03: // channel irq enable
                break;

            case 0x04: // channel irq
                break;

            case 0x05: // sync
                break;

            case 0x7E: // capabilities
                break;

            case 0x7F: // revision
                break;

            default:
                printf("labx-dma: Write of unknown register %08X = %08X\n", addr, value);
                break;
        }
    }
}

static CPUReadMemoryFunc * const dma_regs_read[] = {
    NULL, NULL,
    &dma_regs_readl,
};

static CPUWriteMemoryFunc * const dma_regs_write[] = {
    NULL, NULL,
    &dma_regs_writel,
};


/*
 * Microcode RAM
 */
static uint32_t microcode_ram_readl (void *opaque, target_phys_addr_t addr)
{
    struct labx_dma *p = opaque;

    return p->microcodeRam[RAM_INDEX(addr, p->microcodeWords)];
}

static void microcode_ram_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct labx_dma *p = opaque;

    p->microcodeRam[RAM_INDEX(addr, p->microcodeWords)] = value;
}

static CPUReadMemoryFunc * const microcode_ram_read[] = {
    NULL, NULL,
    &microcode_ram_readl,
};

static CPUWriteMemoryFunc * const microcode_ram_write[] = {
    NULL, NULL,
    &microcode_ram_writel,
};

static int labx_dma_init(SysBusDevice *dev)
{
    struct labx_dma *p = FROM_SYSBUS(typeof (*p), dev);
    int dma_regs;
    int microcode_ram;

    /* Initialize defaults */
    p->microcodeRam = qemu_malloc(p->microcodeWords*4);

    /* Set up memory regions */
    dma_regs = cpu_register_io_memory(dma_regs_read, dma_regs_write, p);
    microcode_ram = cpu_register_io_memory(microcode_ram_read, microcode_ram_write, p);

    sysbus_init_mmio(dev, 0x100 * 4, dma_regs);
    sysbus_init_mmio(dev, 4 * p->microcodeWords, microcode_ram);

    sysbus_mmio_map(dev, 0, p->baseAddress);
    sysbus_mmio_map(dev, 1, p->baseAddress + (1 << (min_bits(p->microcodeWords-1)+2)));

    return 0;
}

static SysBusDeviceInfo labx_dma_info = {
    .init = labx_dma_init,
    .qdev.name  = "labx,dma",
    .qdev.size  = sizeof(struct labx_dma),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("baseAddress",        struct labx_dma, baseAddress,        0),
        DEFINE_PROP_UINT32("paramWords",         struct labx_dma, paramWords,         1024),
        DEFINE_PROP_UINT32("microcodeWords",     struct labx_dma, microcodeWords,     1024),
        DEFINE_PROP_UINT32("numIndexRegs",       struct labx_dma, numIndexRegs,       4),
        DEFINE_PROP_UINT32("numChannels",        struct labx_dma, numChannels,        1),
        DEFINE_PROP_UINT32("numAlus",            struct labx_dma, numAlus,            1),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void labx_dma_register(void)
{
    sysbus_register_withprop(&labx_dma_info);
}

device_init(labx_dma_register)

