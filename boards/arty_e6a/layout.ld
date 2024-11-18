/* Licensed under the Apache License, Version 2.0 or the MIT License. */
/* SPDX-License-Identifier: Apache-2.0 OR MIT                         */
/* Copyright Tock Contributors 2023.                                  */

MEMORY
{
  /* Flash size on ARTY is 16MB. The first 4MB are for FPGA configuration.
     The kernel will be flashed at 4MB offset from the start of the flash (0x40400000).
  */
  rom (rx)  : ORIGIN = 0x40400000, LENGTH = 1M
  /* Reserved for apps */
  prog (rx) : ORIGIN = 0x40400000 + LENGTH(rom), LENGTH = 1M
  ram (rwx) : ORIGIN = 0x80000000, LENGTH = 64K
}

SECTIONS {
    /* Export the start & end of SRAM and flash as symbols for setting
     * up the PMP. Flash includes rom, prog and flash storage, such
     * that we can use a single NAPOT region. The .text section will
     * be made executable by a separate PMP region.
     */
    _sflash = ORIGIN(rom);
    _eflash = ORIGIN(prog) + LENGTH(prog);

    _ssram  = ORIGIN(ram);
    _esram  = ORIGIN(ram) + LENGTH(ram);
}

INCLUDE tock_kernel_layout.ld
