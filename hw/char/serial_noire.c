/*
 * Simple serial controller emulation for Noire on ThinPad
 * Based on 16550A UART emulation
 *
 * See serial_noire.h for interface details
 */

#include "qemu/osdep.h"
#include "hw/char/serial_noire.h"
#include "sysemu/char.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"

// #define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "serial: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
do {} while (0)
#endif

static void serial_update_irq(SerialNoireState *s) {
    qemu_set_irq(s->irq, !fifo8_is_empty(&s->recv_fifo));
}

static uint64_t serial_ioport_read(void *opaque, hwaddr addr, unsigned size) {
    SerialNoireState *s = opaque;
    int readable, writable;
    uint32_t ret;

    readable = !fifo8_is_empty(&s->recv_fifo);

    if (addr == 0) {
        // DATA
        ret = readable ? fifo8_pop(&s->recv_fifo) : 0;
        serial_update_irq(s);
    } else {
        // STATUS
        // XXX temporary solution
        writable = 1;
        ret = (readable << 1) | writable;
    }
    DPRINTF("read addr=0x%" HWADDR_PRIx " val=0x%02x\n", addr, ret);
    return ret;
}

static void serial_ioport_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size) {
    SerialNoireState *s = opaque;
    uint8_t char_val;
    int retry_count;

    DPRINTF("write addr=0x%" HWADDR_PRIx " val=0x%" PRIx64 "\n", addr, val);

    // Can only write to DATA
    if (addr == 0) {
        char_val = val;
        // XXX temporary solution
        retry_count = 0;
        while (retry_count < 1000 && qemu_chr_fe_write(&s->chr, &char_val, 1) != 1) {
            ++retry_count;
        }
        if (retry_count == 1000) {
            fprintf(stderr, "retry count limit reached, abort\n");
            return;
        }
    }
}

static const MemoryRegionOps serial_noire_io_ops = {
    .read = serial_ioport_read,
    .write = serial_ioport_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = false,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void serial_reset(void *opaque) {
    SerialNoireState *s = opaque;
    fifo8_reset(&s->recv_fifo);
    qemu_irq_lower(s->irq);
}

static int serial_can_receive(void *opaque) {
    SerialNoireState *s = opaque;
    int ret;
    // returns 1 when FIFO is not full
    ret = !fifo8_is_full(&s->recv_fifo);
    DPRINTF("serial_can_receive: returning %d\n", ret);
    return ret;
}

static void serial_receive(void *opaque, const uint8_t *buf, int size) {
    SerialNoireState *s = opaque;
    DPRINTF("serial_receive got size = %d, buf[0] = %#02x\n", size, buf[0]);
    if (size > 1) {
        return;
    }
    if (fifo8_is_full(&s->recv_fifo)) {
        DPRINTF("serial_receive when FIFO is full\n");
        return;
    }
    fifo8_push(&s->recv_fifo, buf[0]);
    serial_update_irq(s);
}

SerialNoireState *serial_noire_init(int base, qemu_irq irq,
                                    Chardev *chr, MemoryRegion *system_io) {
    DPRINTF("serial_noire_init\n");
    SerialNoireState *s = g_malloc0(sizeof(SerialNoireState));
    s->irq = irq;
    qemu_chr_fe_init(&s->chr, chr, &error_abort);
    if (!qemu_chr_fe_get_driver(&s->chr)) {
        error_setg(&error_fatal, "Can't create serial device, empty char device");
        return s;
    }
    qemu_chr_fe_set_handlers(&s->chr, serial_can_receive, serial_receive,
                             NULL, s, NULL, true);
    fifo8_create(&s->recv_fifo, NOIRE_UART_FIFO_LENGTH);
    serial_reset(s);
    qemu_register_reset(serial_reset, s);
    memory_region_init_io(&s->io, NULL, &serial_noire_io_ops,
                          s, "serial-noire", 8);
    memory_region_add_subregion(system_io, base, &s->io);

    return s;
}
