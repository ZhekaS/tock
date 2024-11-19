// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Instantiation of the sifive Platform Level Interrupt Controller

use kernel::utilities::StaticRef;
use sifive::plic::{Plic, PlicRegisters};

const TOTAL_INTS: usize = 129;

pub const PLIC_BASE: StaticRef<PlicRegisters> =
    unsafe { StaticRef::new(0x0040_0000 as *const PlicRegisters) };

pub type E6APlic = Plic<TOTAL_INTS>;

pub static mut PLIC: E6APlic = Plic::new(PLIC_BASE);
