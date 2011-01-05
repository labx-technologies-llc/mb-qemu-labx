
/*
 * QEMU model of the LabX audio depacketizer.
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
#include "labx_devices.h"

#define min_bits qemu_fls
#define RAM_INDEX(addr, size) (((addr)>>2)&((1<<min_bits((size)-1))-1))

struct clock_domain_info
{
    uint32_t tsInterval;
};

struct audio_depacketizer
{
    SysBusDevice busdev;

    /* Device Configuration */
    uint32_t baseAddress;
    uint32_t clockDomains;
    uint32_t cacheDataWords;
    uint32_t paramWords;
    uint32_t microcodeWords;
    uint32_t maxStreamSlots;
    uint32_t maxStreams;
    uint32_t hasDMA;
    uint32_t matchArch;

    /* IRQ */
    qemu_irq irq;

    /* Values set by drivers */

    /* Microcode buffer */
    uint32_t* microcodeRam;

    /* Clock domain information */
    struct clock_domain_info *clockDomainInfo;

    /* Attached DMA (if hasDMA > 0) */
    DeviceState *dma;
};

/*
 * Depacketizer registers
 */
static uint32_t depacketizer_regs_readl (void *opaque, target_phys_addr_t addr)
{
    struct audio_depacketizer *p = opaque;

    uint32_t retval = 0;
    
    switch ((addr>>2) & 0xFF)
    {
        case 0x00: // control
            break;

        case 0x01: // vector bar
            break;

        case 0x02: // id select 0
            break;

        case 0x03: // id select 1
            break;

        case 0x04: // id select 2
            break;

        case 0x05: // id select 3
            break;

        case 0x06: // id config data
            break;

        case 0x08: // irq mask
            break;

        case 0x09: // irq flags
            break;

        case 0x0A: // sync
            break;

        case 0x0B: // relocate
            break;

        case 0x0C: // stream status 0
            break;

        case 0x0D: // stream status 1
            break;

        case 0x0E: // stream status 2
            break;

        case 0x0F: // stream status 3
            break;

        case 0xFD: // capabilities a
            retval = (p->maxStreamSlots & 0x7F);
            break;

        case 0xFE: // capabilities b
            retval = ((p->matchArch & 0xFF) << 24) |
                     ((p->maxStreams & 0xFF) << 16) |
                     ((p->clockDomains & 0xFF) << 8) |
                     ((min_bits(p->paramWords-1) & 0x0F) << 4) |
                     ((min_bits(p->microcodeWords-1) & 0x0F));
            break;

        case 0xFF: // revision
            retval = 0x00000014;
            break;

        default:
            printf("labx-audio-depacketizer: Read of unknown register %08X\n", addr);
            break;
    }

    return retval;
}

static void depacketizer_regs_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    //struct audio_depacketizer *p = opaque;

    switch ((addr>>2) & 0xFF)
    {
        case 0x00: // control
            break;

        case 0x01: // vector bar
            break;

        case 0x02: // id select 0
            break;

        case 0x03: // id select 1
            break;

        case 0x04: // id select 2
            break;

        case 0x05: // id select 3
            break;

        case 0x06: // id config data
            break;

        case 0x08: // irq mask
            break;

        case 0x09: // irq flags
            break;

        case 0x0A: // sync
            break;

        case 0x0B: // relocate
            break;

        case 0x0C: // stream status 0
            break;

        case 0x0D: // stream status 1
            break;

        case 0x0E: // stream status 2
            break;

        case 0x0F: // stream status 3
            break;

        case 0xFD: // capabilities a
            break;

        case 0xFE: // capabilities b
            break;

        case 0xFF: // revision
            break;

        default:
            printf("labx-audio-depacketizer: Write of unknown register %08X = %08X\n", addr, value);
            break;
    }
}

static CPUReadMemoryFunc * const depacketizer_regs_read[] = {
    NULL, NULL,
    &depacketizer_regs_readl,
};

static CPUWriteMemoryFunc * const depacketizer_regs_write[] = {
    NULL, NULL,
    &depacketizer_regs_writel,
};


/*
 * Clock domain registers
 */
static uint32_t clock_domain_regs_readl (void *opaque, target_phys_addr_t addr)
{
    struct audio_depacketizer *p = opaque;

    uint32_t retval = 0;
    int domain = (addr>>6) & ((1<<min_bits(p->clockDomains-1))-1);

    switch ((addr>>2)&0x10)
    {
        case 0x00: // recovery index
            break;

        case 0x01: // ts interval
            retval = p->clockDomainInfo[domain].tsInterval;
            break;

        case 0x08: // DAC offset
            break;

        case 0x09: // DAC P coeff
            break;

        case 0x0A: // lock count
            break;

        default:
            break;
    }

    return retval;
}

static void clock_domain_regs_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct audio_depacketizer *p = opaque;
    int domain = (addr>>6) & ((1<<min_bits(p->clockDomains-1))-1);

    switch ((addr>>2)&0x10)
    {
        case 0x00: // recovery index
            break;

        case 0x01: // ts interval
            p->clockDomainInfo[domain].tsInterval = value;
            break;

        case 0x08: // DAC offset
            break;

        case 0x09: // DAC P coeff
            break;

        case 0x0A: // lock count
            break;

        default:
            break;
    }
}

static CPUReadMemoryFunc * const clock_domain_regs_read[] = {
    NULL, NULL,
    &clock_domain_regs_readl,
};

static CPUWriteMemoryFunc * const clock_domain_regs_write[] = {
    NULL, NULL,
    &clock_domain_regs_writel,
};


/*
 * Microcode RAM
 */
static uint32_t microcode_ram_readl (void *opaque, target_phys_addr_t addr)
{
    struct audio_depacketizer *p = opaque;

    return p->microcodeRam[RAM_INDEX(addr, p->microcodeWords)];
}

static void microcode_ram_writel (void *opaque, target_phys_addr_t addr, uint32_t value)
{
    struct audio_depacketizer *p = opaque;

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

static int labx_audio_depacketizer_init(SysBusDevice *dev)
{
    struct audio_depacketizer *p = FROM_SYSBUS(typeof (*p), dev);
    int depacketizer_regs;
    int clock_domain_regs;
    int microcode_ram;

    /* Initialize defaults */
    p->microcodeRam = qemu_malloc(p->microcodeWords*4);
    p->clockDomainInfo = qemu_malloc(sizeof(struct clock_domain_info)*p->clockDomains);

    /* Set up the IRQ */
    sysbus_init_irq(dev, &p->irq);

    /* Set up memory regions */
    depacketizer_regs = cpu_register_io_memory(depacketizer_regs_read, depacketizer_regs_write, p);
    clock_domain_regs = cpu_register_io_memory(clock_domain_regs_read, clock_domain_regs_write, p);
    microcode_ram = cpu_register_io_memory(microcode_ram_read, microcode_ram_write, p);

    sysbus_init_mmio(dev, 0x100 * 4, depacketizer_regs);
    sysbus_init_mmio(dev, 0x10 * 4 * p->clockDomains, clock_domain_regs);
    sysbus_init_mmio(dev, 4 * p->microcodeWords, microcode_ram);

    sysbus_mmio_map(dev, 0, p->baseAddress);
    sysbus_mmio_map(dev, 1, p->baseAddress + (1 << (min_bits(p->microcodeWords-1)+2)));
    sysbus_mmio_map(dev, 2, p->baseAddress + (2 << (min_bits(p->microcodeWords-1)+2)));

    if (p->hasDMA)
    {
        p->dma = labx_dma_create(p->baseAddress + (4 << (min_bits(p->microcodeWords-1)+2)), 1024);
    }
    
    return 0;
}

static SysBusDeviceInfo labx_audio_depacketizer_info = {
    .init = labx_audio_depacketizer_init,
    .qdev.name  = "labx,audio-depacketizer",
    .qdev.size  = sizeof(struct audio_depacketizer),
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT32("baseAddress",        struct audio_depacketizer, baseAddress,        0),
        DEFINE_PROP_UINT32("clockDomains",       struct audio_depacketizer, clockDomains,       1),
        DEFINE_PROP_UINT32("cacheDataWords",     struct audio_depacketizer, cacheDataWords,     1024),
        DEFINE_PROP_UINT32("paramWords",         struct audio_depacketizer, paramWords,         1024),
        DEFINE_PROP_UINT32("microcodeWords",     struct audio_depacketizer, microcodeWords,     1024),
        DEFINE_PROP_UINT32("maxStreamSlots",     struct audio_depacketizer, maxStreamSlots,     32),
        DEFINE_PROP_UINT32("maxStreams",         struct audio_depacketizer, maxStreams,         128),
        DEFINE_PROP_UINT32("hasDMA",             struct audio_depacketizer, hasDMA,             1),
        DEFINE_PROP_UINT32("matchArch",          struct audio_depacketizer, matchArch,          255),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void labx_audio_depacketizer_register(void)
{
    sysbus_register_withprop(&labx_audio_depacketizer_info);
}

device_init(labx_audio_depacketizer_register)

