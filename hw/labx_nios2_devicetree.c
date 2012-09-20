/*
 * Flexible model of nios2 designs that use a device-tree to determine
 * the hardware configuration.
 *
 * Copyright (c) 2012 Chris Wulff <crwulff@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include "sysbus.h"
#include "hw.h"
#include "net.h"
#include "flash.h"
#include "sysemu.h"
#include "devices.h"
#include "boards.h"
#include "device_tree.h"
#include "nios2.h"
#include "altera.h"
#include "labx_devices.h"
#include "loader.h"
#include "elf.h"
#include "blockdev.h"
#include "exec-memory.h"

#include <libfdt.h>

#define LMB_BRAM_SIZE  (128 * 1024)

static int sopc_device_probe(void *fdt, const char *node_path, int pass, uint32_t offset);

static struct {
    uint32_t bootstrap_pc;
    uint32_t cmdline;
    uint32_t initrd;
    uint32_t fdt;
} boot_info;

/* Current ethernet device index for multiple network interfaces */
static int eth_dev_index = -1;

static void main_cpu_reset(void *opaque)
{
    Nios2CPU *cpu = opaque;
    CPUNios2State *env = &cpu->env;

    cpu_reset(CPU(cpu));

    env->regs[R_ARG0] = boot_info.cmdline;
    env->regs[R_ARG1] = boot_info.initrd;
    env->regs[R_ARG2] = boot_info.fdt;
    env->regs[R_PC]   = boot_info.bootstrap_pc;
}

#ifndef CONFIG_FDT
#error "Device-tree support is required for this target to function"
#endif

#define BINARY_DEVICE_TREE_FILE "labx-nios2.dtb"
static void *get_device_tree(int *fdt_size)
{
    char *path;
    void *fdt;

    /* Try the local "local.dtb" override.  */
    fdt = load_device_tree("local.dtb", fdt_size);
    if (!fdt) {
        path = qemu_find_file(QEMU_FILE_TYPE_BIOS, BINARY_DEVICE_TREE_FILE);
        if (path) {
            fdt = load_device_tree(path, fdt_size);
            g_free(path);
        }
    }

    return fdt;
}

static int labx_load_device_tree(target_phys_addr_t addr,
                                 uint32_t ramsize,
                                 target_phys_addr_t initrd_base,
                                 target_phys_addr_t initrd_size,
                                 const char *kernel_cmdline)
{
    int fdt_size;
    void *fdt;
    int r;

    fdt = get_device_tree(&fdt_size);

    if (!fdt) {
        return 0;
    }

    if (kernel_cmdline && strlen(kernel_cmdline)) {
        r = qemu_devtree_setprop_string(fdt, "/chosen", "bootargs",
                                        kernel_cmdline);
        if (r < 0) {
            fprintf(stderr, "couldn't set /chosen/bootargs\n");
        }
    }
    cpu_physical_memory_write(addr, (void *)fdt, fdt_size);

    return fdt_size;
}

static uint64_t translate_kernel_address(void *opaque, uint64_t addr)
{
    return addr - 0xC0000000LL;
}

static ram_addr_t get_dram_base(void *fdt)
{
    Error *errp = NULL;

    printf("DRAM base %08X, size %08X\n",
        qemu_devtree_getprop_cell(fdt, "/memory", "reg", 0, 0, &errp),
        qemu_devtree_getprop_cell(fdt, "/memory", "reg", 1, 0, &errp));

    return qemu_devtree_getprop_cell(fdt, "/memory", "reg", 0, 0, &errp);
}

typedef void (*device_init_func_t)(void *fdt, const char *node_path, uint32_t offset);

typedef struct DevInfo {
    device_init_func_t probe;
    int pass;
    const char **compat;

} DevInfo;

/*
 * Interrupt controller device
 */
static qemu_irq irq[32] = {};
static qemu_irq *cpu_irq;

static void cpu_probe(void *fdt, const char *node_path, uint32_t offset)
{
    int i;
    DeviceState *dev;
    Error *errp = NULL;

    Nios2CPU *cpu = cpu_nios2_init("nios2");

    qemu_register_reset(main_cpu_reset, cpu);

#if 0 /* TODO: Finish off the vectored-interrupt-controller */
    int reglen;
    const void *reg = qemu_devtree_getprop_offset(fdt, node, "reg", &reglen);
    uint32_t irq_addr = qemu_devtree_int_array_index(reg, 0) + offset;
    int nrIrqLen;
    const void *nrIrq =
        qemu_devtree_getprop_offset(fdt, node, "ALTR,num-intr-inputs",
                                    &nrIrqLen);
    uint32_t nrIrqs = qemu_devtree_int_array_index(nrIrq, 0);

    printf("  IRQ BASE %08X NIRQS %d\n", irq_addr, nrIrqs);

    cpu_irq = nios2_pic_init_cpu(env);
    dev = altera_vic_create(irq_addr, cpu_irq[0], 2);
#else
    /* Internal interrupt controller (IIC) */
    cpu_irq = nios2_pic_init_cpu(cpu);
    dev = altera_iic_create(cpu, cpu_irq[0], 2);
#endif

    /* TODO: use the entrypoint of the passed in elf file or
       the device-tree one */
#if 0
    cpu->env.reset_addr =
        qemu_devtree_getprop_cell(fdt, node_path, "ALTR,reset-addr", 0, 0, &errp);
#else
    cpu->env.reset_addr = 0xc0000000;
#endif

    cpu->env.exception_addr =
        qemu_devtree_getprop_cell(fdt, node_path, "ALTR,exception-addr", 0, 0, &errp);
    cpu->env.fast_tlb_miss_addr =
        qemu_devtree_getprop_cell(fdt, node_path, "ALTR,fast-tlb-miss-addr", 0, 0, &errp);

    /* reset again to use the new reset vector */
    cpu_reset(CPU(cpu));

    for (i = 0; i < 32; i++) {
        irq[i] = qdev_get_gpio_in(dev, i);
    }
}

DevInfo cpu_device = {
    .probe = &cpu_probe,
    .pass = 0,
    .compat = (const char * []) {
        "ALTR,nios2-11.0",
        "ALTR,nios2-11.1",
        "ALTR,nios2-12.0",
        NULL
    }
};

/*
 * Flash device
 */
static void flash_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t flash_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) + offset;
    uint32_t flash_size = qemu_devtree_getprop_cell(fdt, node_path, "reg", 1, 0, &errp);

    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi01_register(flash_addr, NULL,
                          qemu_devtree_get_node_name(fdt, node_path), flash_size,
                          dinfo ? dinfo->bdrv : NULL, (64 * 1024),
                          flash_size >> 16,
                          1, 0x89, 0x18, 0x0000, 0x0, 0);
    printf("-- loaded %d bytes to %08X\n",
           load_image_targphys(qemu_devtree_get_node_name(fdt, node_path),
                               flash_addr, flash_size), flash_addr);
}

DevInfo flash_device = {
    .probe = &flash_probe,
    .pass = 1,
    .compat = (const char * []) { "cfi-flash", NULL }
};

/*
 * LabX audio packetizer device
 */
static void labx_audio_packetizer_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t packetizer_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) +
                               offset;
    uint32_t packetizer_irq =
        qemu_devtree_getprop_cell(fdt, node_path, "interrupts", 0, 0, &errp);
    uint32_t clock_domains =
        qemu_devtree_getprop_cell(fdt, node_path, "labx,num-clock-domains", 0, 0, &errp);
    uint32_t cache_words =
        qemu_devtree_getprop_cell(fdt, node_path, "labx,cache-data-words", 0, 0, &errp);

    labx_audio_packetizer_create(packetizer_addr, irq[packetizer_irq],
                                 clock_domains, cache_words);
}

DevInfo labx_audio_packetizer_device = {
    .probe = &labx_audio_packetizer_probe,
    .pass = 1,
    .compat = (const char * []) { "labx,labx_audio_packetizer-1.0", NULL }
};

/*
 * LabX audio depacketizer device
 */
static void labx_audio_depacketizer_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t depacketizer_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) +
                                 offset;
    uint32_t depacketizer_irq =
        qemu_devtree_getprop_cell(fdt, node_path, "interrupts", 0, 0, &errp);
    uint32_t clock_domains =
        qemu_devtree_getprop_cell(fdt, node_path, "labx,num-clock-domains", 0, 0, &errp);
    uint32_t cache_words =
        qemu_devtree_getprop_cell(fdt, node_path, "labx,cache-data-words", 0, 0, &errp);

    int ifLen;
    const void *ifType =
        qemu_devtree_getprop(fdt, node_path, "labx,interface-type", &ifLen, 0, &errp);
    int hasDMA = (0 != strncmp("CACHE_RAM", (const char *)ifType, ifLen));

    labx_audio_depacketizer_create(depacketizer_addr, irq[depacketizer_irq],
                                   clock_domains, cache_words, hasDMA);
}

DevInfo labx_audio_depacketizer_device = {
    .probe = &labx_audio_depacketizer_probe,
    .pass = 1,
    .compat = (const char * []) {
        "labx,labx-audio-depacketizer-1.00.a",
        "labx,labx-audio-depacketizer-1.01.a",
        NULL
    }
};

/*
 * LabX dma device
 */
static void labx_dma_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t dma_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) + offset;

    labx_dma_create(dma_addr, 1024);
}

DevInfo labx_dma_device = {
    .probe = &labx_dma_probe,
    .pass = 1,
    .compat = (const char * []) { "labx,labx_dma-1.0", NULL }
};

/*
 * LabX ethernet device
 */
static void labx_ethernet_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t ethernet_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) +
                             offset;
    uint32_t host_irq =
        qemu_devtree_getprop_cell(fdt, node_path, "interrupts", 0, 0, &errp);
    uint32_t fifo_irq = 0; /*
        qemu_devtree_getprop_cell(fdt, node_path, "interrupts", 2, 0, &errp); */
    uint32_t phy_irq = 0; /*
        qemu_devtree_getprop_cell(fdt, node_path, "interrupts", 4, 0, &errp); */

    labx_ethernet_create(&nd_table[++eth_dev_index], ethernet_addr,
                         irq[host_irq], irq[fifo_irq], irq[phy_irq]);
}

DevInfo labx_ethernet_device = {
    .probe = &labx_ethernet_probe,
    .pass = 1,
    .compat = (const char * []) { "labx,labx_ethernet-1.0", NULL }
};

/*
 * LabX ptp device
 */
static void labx_ptp_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t ptp_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) + offset;

    labx_ptp_create(ptp_addr);
}

DevInfo labx_ptp_device = {
    .probe = &labx_ptp_probe,
    .pass = 1,
    .compat = (const char * []) { "labx,labx_ptp-1.0", NULL }
};

/*
 * Altera uart device
 */
static void altera_uart_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t uart_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) + offset;
    uint32_t uart_irq = qemu_devtree_getprop_cell(fdt, node_path, "interrupts", 0, 0, &errp);

    printf("  UART BASE %08X IRQ %d\n", uart_addr, uart_irq);

    sysbus_create_simple("altera,uart", uart_addr, irq[uart_irq]);
}

DevInfo altera_uart_device = {
    .probe = &altera_uart_probe,
    .pass = 1,
    .compat = (const char * []) { "ALTR,uart-1.0", NULL }
};

/*
 * Altera timer device
 */
static void altera_timer_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t timer_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) + offset;
    uint32_t timer_irq = qemu_devtree_getprop_cell(fdt, node_path, "interrupts", 0, 0, &errp);
    uint32_t timer_freq =
        qemu_devtree_getprop_cell(fdt, node_path, "clock-frequency", 0, 0, &errp);

    printf("  TIMER BASE %08X IRQ %d FREQUENCY %d\n",
           timer_addr, timer_irq, timer_freq);

    altera_timer_create(timer_addr, irq[timer_irq], timer_freq);
}

DevInfo altera_timer_device = {
    .probe = &altera_timer_probe,
    .pass = 1,
    .compat = (const char * []) { "ALTR,timer-1.0", NULL }
};

/*
 * Altera clock domain crossing bridge
 */
static void altera_bridge_probe(void *fdt, const char *node_path, uint32_t offset)
{
    Error *errp = NULL;

    uint32_t bridge_addr = qemu_devtree_getprop_cell(fdt, node_path, "reg", 0, 0, &errp) + offset;

    printf("  BRIDGE %08X\n", bridge_addr);

    /* Do multiple passes through the devices. Some have dependencies
       on others being first */
    int num_children = qemu_devtree_get_num_children(fdt, node_path, 1);
    char **children = qemu_devtree_get_children(fdt, node_path, 1);

    int pass = 0;
    int again = 0;
    int i = 0;
    do {
        again = 0;
        for (i = 0; i < num_children; i++) {
            again |= sopc_device_probe(fdt, children[i], pass, bridge_addr);
        }

        pass++;

    } while (again);
}

DevInfo altera_bridge_device = {
    .probe = &altera_bridge_probe,
    .pass = 1,
    .compat = (const char * []) { "simple-bus", NULL }
};

/*
 * Table of available devices
 */
DevInfo *devices[] = {
    &cpu_device,
    &flash_device,
    &labx_audio_packetizer_device,
    &labx_audio_depacketizer_device,
    &labx_dma_device,
    &labx_ethernet_device,
    &labx_ptp_device,
    &altera_uart_device,
    &altera_timer_device,
    &altera_bridge_device,
    NULL
};

static int sopc_device_probe(void *fdt, const char *node_path, int pass, uint32_t offset)
{
    DevInfo **dev = &(devices[0]);

    while (*dev) {
        const char **compat = &((*dev)->compat[0]);
        while (*compat) {
            if (0 == fdt_node_check_compatible(fdt, fdt_path_offset(fdt, node_path), *compat)) {
                if (pass == (*dev)->pass) {
                    printf("Adding a device for node %s\n",
                           fdt_get_name(fdt, fdt_path_offset(fdt, node_path), NULL));

                    (*dev)->probe(fdt, node_path, offset);
                    return 0;
                }

                if (pass < (*dev)->pass) {
                    /* Probe again on the next pass */
                    return 1;
                }
            }

            compat++;
        }

        dev++;
    }

    return 0;
}

static void cpus_probe(void *fdt)
{
    int num_children = qemu_devtree_get_num_children(fdt, "/cpus", 1);
    char **children = qemu_devtree_get_children(fdt, "/cpus", 1);
    int i;

    for (i = 0; i < num_children; i++) {
        sopc_device_probe(fdt, children[i], 0, 0xE0000000);
    }
}

static void sopc_bus_probe(void *fdt)
{
    int num_children = qemu_devtree_get_num_children(fdt, "/sopc", 1);
    char **children = qemu_devtree_get_children(fdt, "/sopc", 1);

    if (num_children > 0) {
        /* Do multiple passes through the devices. Some have dependencies
           on others being first */
        int pass = 0;
        int again = 0;
        do {
            int child = 0;
            again = 0;
            do {
                again |= sopc_device_probe(fdt, children[child++], pass, 0xE0000000);
            } while (child < num_children);

            pass++;

        } while (again);
    }
}

static void
labx_nios2_init(ram_addr_t ram_size,
                const char *boot_device,
                const char *kernel_filename,
                const char *kernel_cmdline,
                const char *initrd_filename,
                const char *cpu_model)
{
    MemoryRegion *address_space_mem = get_system_memory();

    int kernel_size;
    int fdt_size;
    void *fdt = get_device_tree(&fdt_size);
    target_phys_addr_t ddr_base = get_dram_base(fdt);
    MemoryRegion *phys_lmb_bram = g_new(MemoryRegion, 1);
    MemoryRegion *phys_ram = g_new(MemoryRegion, 1);
    MemoryRegion *phys_ram_alias = g_new(MemoryRegion, 1);

    /* Attach emulated BRAM through the LMB. LMB size is not specified
       in the device-tree but there must be one to hold the vector table. */
    memory_region_init_ram(phys_lmb_bram, "nios2.lmb_bram", LMB_BRAM_SIZE);
    vmstate_register_ram_global(phys_lmb_bram);
    memory_region_add_subregion(address_space_mem, 0x00000000, phys_lmb_bram);

    memory_region_init_ram(phys_ram, "nios2.ram", ram_size);
    vmstate_register_ram_global(phys_ram);
    memory_region_add_subregion(address_space_mem, ddr_base, phys_ram);
    memory_region_init_alias(phys_ram_alias, "nios2.ram.mirror",
                             phys_ram, 0, ram_size);
    memory_region_add_subregion(address_space_mem, ddr_base + 0xc0000000,
                                phys_ram_alias);

    /* Create cpus listed in the device-tree */
    cpus_probe(fdt);

    /* Create other devices listed in the device-tree */
    sopc_bus_probe(fdt);

    if (kernel_filename) {
        uint64_t entry = 0, low = 0, high = 0;
        uint32_t base32 = 0;

        /* Boots a kernel elf binary.  */
        kernel_size = load_elf(kernel_filename, NULL, NULL,
                               &entry, &low, &high,
                               0, ELF_MACHINE, 0);
        base32 = entry;
        if (base32 == 0xc0000000) {
            kernel_size = load_elf(kernel_filename, translate_kernel_address,
                                   NULL, &entry, NULL, NULL,
                                   0, ELF_MACHINE, 0);
        }
        /* Always boot into physical ram.  */
        boot_info.bootstrap_pc = ddr_base + 0xc0000000 + (entry & 0x07ffffff);

        /* If it wasn't an ELF image, try an u-boot image.  */
        if (kernel_size < 0) {
            target_phys_addr_t uentry, loadaddr;

            kernel_size = load_uimage(kernel_filename, &uentry, &loadaddr, 0);
            boot_info.bootstrap_pc = uentry;
            high = (loadaddr + kernel_size + 3) & ~3;
        }

        /* Not an ELF image nor an u-boot image, try a RAW image.  */
        if (kernel_size < 0) {
            kernel_size = load_image_targphys(kernel_filename, ddr_base,
                                              ram_size);
            boot_info.bootstrap_pc = ddr_base;
            high = (ddr_base + kernel_size + 3) & ~3;
        }

        if (initrd_filename) {
            uint32_t initrd_base = 0x88c00000;
            uint32_t initrd_size =
                load_image_targphys(initrd_filename, initrd_base,
                                    ram_size - initrd_base);
            if (initrd_size <= 0) {
                fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
                        initrd_filename);
                exit(1);
            }

            boot_info.initrd = initrd_base;
        } else {
            boot_info.initrd = 0x00000000;
        }

        boot_info.cmdline = high + 4096;
        if (kernel_cmdline && strlen(kernel_cmdline)) {
            pstrcpy_targphys("cmdline", boot_info.cmdline, 256, kernel_cmdline);
        }
        /* Provide a device-tree.  */
        boot_info.fdt = boot_info.cmdline + 4096;
        labx_load_device_tree(boot_info.fdt, ram_size,
                              0, 0,
                              kernel_cmdline);
    }
}

static QEMUMachine labx_nios2_machine = {
    .name = "labx-nios2-devicetree",
    .desc = "Nios II design based on the peripherals specified "
            "in the device-tree.",
    .init = labx_nios2_init,
    .is_default = 1
};

static void labx_nios2_machine_init(void)
{
    qemu_register_machine(&labx_nios2_machine);
}

machine_init(labx_nios2_machine_init);
