/*
 * Simple serial controller emulation for Noire on ThinPad
 * Based on 16550A UART emulation
 *
 * - Buadrate is fixed (thus no interface to configure is provided)
 * - Only has receiving FIFO of length 16
 *
 * offset 0: DATA Read/Write
 *     writing (word, halfword, or byte) to DATA sends the lowest byte
 *     reading (word, halfword, or byte) from DATA returns first available
 *         character zero-extended to given bit width
 *
 * offset 4: STATUS Readonly
 *     reading (word, halfword, or byte) from STATUS returns the status whose
 *         bit 0: capable to receive at least one character
 *         bit 1: at least one character is pending read
 *         other: zero
 *
 * IRQ: high if at least one character is pending read
 */

#ifndef HW_SERIAL_NOIRE_H
#define HW_SERIAL_NOIRE_H

#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "sysemu/char.h"
#include "exec/memory.h"
#include "qemu/fifo8.h"
#include "sysemu/char.h"

#define NOIRE_UART_FIFO_LENGTH    16      /* Same Fifo Length as 16550A */

typedef struct SerialNoireState SerialNoireState;

struct SerialNoireState {
    qemu_irq irq;
    CharBackend chr;
    Fifo8 recv_fifo;
    MemoryRegion io;
};

SerialNoireState *serial_noire_init(int base, qemu_irq irq,
                                    Chardev *chr, MemoryRegion *system_io);

#endif  // HW_SERIAL_NOIRE_H

