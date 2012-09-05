
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

    MemoryRegion  mmio_dma;
    MemoryRegion  mmio_microcode;

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
static uint64_t dma_regs_read(void *opaque, target_phys_addr_t addr, unsigned int size)
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

static void dma_regs_write(void *opaque, target_phys_addr_t addr, uint64_t val64, unsigned int size)
{
    //struct labx_dma *p = opaque;
    uint32_t value = val64;

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

static const MemoryRegionOps dma_regs_ops = {
    .read = dma_regs_read,
    .write = dma_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};


/*
 * Microcode RAM
 */
static uint64_t microcode_ram_read(void *opaque, target_phys_addr_t addr, unsigned int size) {
    struct labx_dma *p = opaque;

    return p->microcodeRam[RAM_INDEX(addr, p->microcodeWords)];
}

static void microcode_ram_write(void *opaque, target_phys_addr_t addr, uint64_t val64, unsigned int size) {
    struct labx_dma *p = opaque;
    uint32_t value = val64;

    p->microcodeRam[RAM_INDEX(addr, p->microcodeWords)] = value;
}

static const MemoryRegionOps microcode_ram_ops = {
    .read = microcode_ram_read,
    .write = microcode_ram_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};


static int labx_dma_init(SysBusDevice *dev)
{
    struct labx_dma *p = FROM_SYSBUS(typeof (*p), dev);

    /* Initialize defaults */
    p->microcodeRam = g_malloc0(p->microcodeWords*4);

    /* Set up memory regions */
    memory_region_init_io(&p->mmio_dma,       &dma_regs_ops,      p, "labx,dma-regs",      0x100 * 4);
    memory_region_init_io(&p->mmio_microcode, &microcode_ram_ops, p, "labx,dma-microcode", 4 * p->microcodeWords);

    sysbus_init_mmio(dev, &p->mmio_dma);
    sysbus_init_mmio(dev, &p->mmio_microcode);

    sysbus_mmio_map(dev, 0, p->baseAddress);
    sysbus_mmio_map(dev, 1, p->baseAddress + (1 << (min_bits(p->microcodeWords-1)+2)));

    return 0;
}

static Property labx_dma_properties[] = {
    DEFINE_PROP_UINT32("baseAddress",        struct labx_dma, baseAddress,        0),
    DEFINE_PROP_UINT32("paramWords",         struct labx_dma, paramWords,         1024),
    DEFINE_PROP_UINT32("microcodeWords",     struct labx_dma, microcodeWords,     1024),
    DEFINE_PROP_UINT32("numIndexRegs",       struct labx_dma, numIndexRegs,       4),
    DEFINE_PROP_UINT32("numChannels",        struct labx_dma, numChannels,        1),
    DEFINE_PROP_UINT32("numAlus",            struct labx_dma, numAlus,            1),
    DEFINE_PROP_END_OF_LIST(),
};

static void labx_dma_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = labx_dma_init;
    dc->props = labx_dma_properties;
}

static TypeInfo labx_dma_info = {
    .name          = "labx,dma",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct labx_dma),
    .class_init    = labx_dma_class_init,
};

static void labx_dma_register(void) {
    type_register_static(&labx_dma_info);
}

type_init(labx_dma_register)

