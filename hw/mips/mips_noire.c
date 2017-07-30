/*
 * QEMU machine model for Project Noire on ThinPad
 * Based on mipssim emulation model
 *
 * Differences between mipssim:
 * - Does not load kernel into RAM, but load into flash which is at
 *   physical address 0x1E00_0000
 * - Different serial driver to match the behavior of our hardware
 * - Does not have network support (no DM9000 emulation available)
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/char/serial.h"
#include "hw/isa/isa.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/mips/bios.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "sysemu/qtest.h"

#define FLASH_SIZE (8 * 1024 * 1024)
#define FLASH_PHYADDR (0x1e000000LL)

typedef struct ResetData {
    MIPSCPU *cpu;
    uint64_t vector;
} ResetData;

static void main_cpu_reset(void *opaque)
{
    ResetData *s = (ResetData *)opaque;
    CPUMIPSState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->active_tc.PC = s->vector & ~(target_ulong)1;
    if (s->vector & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    }
}

static void
mips_mipssim_init(MachineState *machine)
{
    ram_addr_t ram_size = machine->ram_size;
    const char *cpu_model = machine->cpu_model;
    const char *kernel_filename = machine->kernel_filename;
    char *filename;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *isa = g_new(MemoryRegion, 1);
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *flash = g_new(MemoryRegion, 1);
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MIPSCPU *cpu;
    CPUMIPSState *env;
    ResetData *reset_info;
    int bios_size;
    int flash_kernel_size;

    /* Init CPUs. */
    if (cpu_model == NULL) {
#ifdef TARGET_MIPS64
        cpu_model = "5Kf";
#else
        cpu_model = "24Kf";
#endif
    }
    cpu = cpu_mips_init(cpu_model);
    if (cpu == NULL) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    env = &cpu->env;

    reset_info = g_malloc0(sizeof(ResetData));
    reset_info->cpu = cpu;
    reset_info->vector = env->active_tc.PC;
    qemu_register_reset(main_cpu_reset, reset_info);

    /* Allocate RAM. */
    memory_region_allocate_system_memory(ram, NULL, "mips_mipssim.ram",
                                         ram_size);
    memory_region_init_ram(bios, NULL, "mips_mipssim.bios", BIOS_SIZE,
                           &error_fatal);
    vmstate_register_ram_global(bios);
    memory_region_set_readonly(bios, true);

    memory_region_init_ram(flash, NULL, "mips_mipssim.flash",
                           FLASH_SIZE, &error_fatal);

    memory_region_add_subregion(address_space_mem, 0, ram);

    /* Map the BIOS / boot exception handler. */
    memory_region_add_subregion(address_space_mem, 0x1fc00000LL, bios);
    /* Load a BIOS / boot exception handler image. */
    if (bios_name == NULL)
        bios_name = BIOS_FILENAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = load_image_targphys(filename, 0x1fc00000LL, BIOS_SIZE);
        g_free(filename);
    } else {
        bios_size = -1;
    }
    if ((bios_size < 0 || bios_size > BIOS_SIZE) &&
        !kernel_filename && !qtest_enabled()) {
        /* Bail out if we have neither a kernel image nor boot vector code. */
        error_report("Could not load MIPS bios '%s', and no "
                     "-kernel argument was specified", bios_name);
        exit(1);
    } else {
        /* We have a boot vector start address. */
        env->active_tc.PC = (target_long)(int32_t)0xbfc00000;
    }

    /* Map the flash */
    memory_region_add_subregion(address_space_mem, FLASH_PHYADDR, flash);

    /* Load kernel into flash */
    if (!kernel_filename) {
        error_report("Missing -kernel argument to load into flash");
        exit(1);
    }

    flash_kernel_size = load_image_targphys(kernel_filename,
            FLASH_PHYADDR, FLASH_SIZE);
    if (flash_kernel_size < 0 || flash_kernel_size > FLASH_SIZE) {
        error_report("Could not load kernel '%s'", kernel_filename);
        exit(1);
    }
    /* But do we ever needed to reboot? */
    reset_info->vector = (target_long)(int32_t)0xbfc00000;

    /* Init CPU internal devices. */
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);

    /* Register 64 KB of ISA IO space at 0x1fd00000. */
    memory_region_init_alias(isa, NULL, "isa_mmio",
                             get_system_io(), 0, 0x00010000);
    memory_region_add_subregion(get_system_memory(), 0x1fd00000, isa);

    /* A single 16450 sits at offset 0x3f8. It is attached to
       MIPS CPU INT2, which is interrupt 4. */
    if (serial_hds[0])
        serial_init(0x3f8, env->irq[4], 115200, serial_hds[0],
                    get_system_io());
}

static void mips_mipssim_machine_init(MachineClass *mc)
{
    mc->desc = "Noire on ThinPad";
    mc->init = mips_mipssim_init;
}

DEFINE_MACHINE("noire", mips_mipssim_machine_init)
