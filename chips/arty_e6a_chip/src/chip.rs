// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use core::fmt::Write;
use core::ptr::addr_of;
use kernel::debug;
use kernel::platform::chip::{Chip, InterruptService};
use kernel::utilities::registers::interfaces::Readable;
use kernel::{hil::time::Frequency, utilities::registers::interfaces::ReadWriteable};
use rv32i::csr;
use rv32i::{
    csr::{mcause, mie::mie, mip::mip, CSR},
    pmp::PMPUserMPU,
};
use sifive::{clint::Clint, uart::Uart};
use crate::plic::PLIC;

use rv32i::pmp::kernel_protection::KernelProtectionPMP;

use crate::interrupts; // {simple::SimplePMP, PMPUserMPU};

pub type PmpType = PMPUserMPU<4, KernelProtectionPMP<16>>;
pub type ClintType<'a, F> = Clint<'a, F>;

pub struct E6ACore<'a, F: Frequency, I: InterruptService + 'a> {
    userspace_kernel_boundary: rv32i::syscall::SysCall,
    pmp: PmpType,
    clint: &'a ClintType<'a, F>,
    plic: &'a crate::plic::E6APlic,
    plic_interrupt_service: &'a I, /* TODO: Use it */
}

pub struct ArtyE6ADefaultPeripherals<'a> {
    pub gpio_port: crate::gpio::Port<'a>,
    pub uart0: sifive::uart::Uart<'a>,
}

impl<'a> ArtyE6ADefaultPeripherals<'a> {
    pub fn new(clock_frequency: u32) -> Self {
        Self {
            //machinetimer: ArtyExxClint::new(&clint::CLINT_BASE),
            gpio_port: crate::gpio::Port::new(),
            uart0: Uart::new(crate::uart::UART0_BASE, clock_frequency),
        }
    }

    // Resolves any circular dependencies and sets up deferred calls
    pub fn init(&'static self) {
        kernel::deferred_call::DeferredCallClient::register(&self.uart0);
    }
}

impl<'a> InterruptService for ArtyE6ADefaultPeripherals<'a> {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            interrupts::UART0 => { self.uart0.handle_interrupt(); true }
            /*
            int_pin @ interrupts::GPIO0..=interrupts::GPIO31 => {
                let pin = &self.e310x.gpio_port[(int_pin - interrupts::GPIO0) as usize];
                pin.handle_interrupt();
            } */

            // put E310x specific interrupts here
            _ => false,
        }
    }
}

impl<'a, F: Frequency, I: InterruptService + 'a> E6ACore<'a, F, I> {
    pub unsafe fn new(
        pmp: KernelProtectionPMP<16>,
        clint: &'a ClintType<'a, F>,
        plic_interrupt_service: &'a I,
    ) -> Self {
        Self {
            userspace_kernel_boundary: rv32i::syscall::SysCall::new(),
            pmp: PMPUserMPU::new(pmp),
            clint,
            plic: &*addr_of!(PLIC),
            plic_interrupt_service,
        }
    }

    pub unsafe fn enable_plic_interrupts(&self) {
        /* E6A core manual
         * PLIC Chapter 9.4  A pending bit in the PLIC core can be cleared
         * by setting the associated enable bit then performing a claim.
         */
        // first disable interrupts globally
        let old_mie = CSR
            .mstatus
            .read_and_clear_field(csr::mstatus::mstatus::mie);

        //self.plic.enable_all();
        self.plic.enable_specific_interrupts(&[interrupts::UART0]);
        self.plic.clear_all_pending();

        // restore the old external interrupt enable bit
        CSR
            .mstatus
            .modify(csr::mstatus::mstatus::mie.val(old_mie));
    }

    unsafe fn handle_plic_interrupts(&self) {
        while let Some(interrupt) = self.plic.get_saved_interrupts() {
            if !self.plic_interrupt_service.service_interrupt(interrupt) {
                debug!("Pidx {}", interrupt);
            }
            self.atomic(|| {
                self.plic.complete(interrupt);
            });
        }
    }
}

impl<'a, F: Frequency, I: InterruptService + 'a> kernel::platform::chip::Chip
    for E6ACore<'a, F, I>
{
    type MPU = PmpType;
    type UserspaceKernelBoundary = rv32i::syscall::SysCall;

    fn mpu(&self) -> &Self::MPU {
        &self.pmp
    }

    fn userspace_kernel_boundary(&self) -> &rv32i::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    
    fn service_pending_interrupts(&self) {
        loop {
            let mip = CSR.mip.extract();

            if mip.is_set(mip::mtimer) {
                self.clint.handle_interrupt();
            }
            if self.plic.get_saved_interrupts().is_some() {
                unsafe {
                    self.handle_plic_interrupts();
                }
            }

            if !mip.any_matching_bits_set(mip::mtimer::SET)
                && self.plic.get_saved_interrupts().is_none()
            {
                break;
            }
        }

        // Re-enable all MIE interrupts that we care about. Since we looped
        // until we handled them all, we can re-enable all of them.
        CSR.mie.modify(mie::mext::SET + mie::mtimer::SET);
    }

    fn has_pending_interrupts(&self) -> bool {
        // First check if the global machine timer interrupt is set.
        // We would also need to check for additional global interrupt bits
        // if there were to be used for anything in the future.
        CSR.mip.is_set(mip::mtimer) 
        ||
        // Then we can check the PLIC.
        self.plic.get_saved_interrupts().is_some()
    }

    fn sleep(&self) {
        unsafe {
            rv32i::support::wfi();
        }
    }

    unsafe fn atomic<T, R>(&self, f: T) -> R
    where
        T: FnOnce() -> R,
    {
        rv32i::support::atomic(f)
    }

    unsafe fn print_state(&self, write: &mut dyn Write) {
        rv32i::print_riscv_state(write);
    }
}

fn handle_exception(exception: mcause::Exception) {
    match exception {
        mcause::Exception::UserEnvCall | mcause::Exception::SupervisorEnvCall => (),

        mcause::Exception::InstructionMisaligned
        | mcause::Exception::InstructionFault
        | mcause::Exception::IllegalInstruction
        | mcause::Exception::Breakpoint
        | mcause::Exception::LoadMisaligned
        | mcause::Exception::LoadFault
        | mcause::Exception::StoreMisaligned
        | mcause::Exception::StoreFault
        | mcause::Exception::MachineEnvCall
        | mcause::Exception::InstructionPageFault
        | mcause::Exception::LoadPageFault
        | mcause::Exception::StorePageFault
        | mcause::Exception::Unknown => {
            panic!("fatal exception");
        }
    }
}

unsafe fn handle_interrupt(intr: mcause::Interrupt) {
    match intr {
        mcause::Interrupt::UserSoft
        | mcause::Interrupt::UserTimer
        | mcause::Interrupt::UserExternal => {
            panic!("unexpected user-mode interrupt");
        }
        mcause::Interrupt::SupervisorExternal
        | mcause::Interrupt::SupervisorTimer
        | mcause::Interrupt::SupervisorSoft => {
            panic!("unexpected supervisor-mode interrupt");
        }

        mcause::Interrupt::MachineSoft => {
            CSR.mie.modify(mie::msoft::CLEAR);
        }
        mcause::Interrupt::MachineTimer => {
            CSR.mie.modify(mie::mtimer::CLEAR);
        }
        mcause::Interrupt::MachineExternal => {
            // We received an interrupt, disable interrupts while we handle them
            CSR.mie.modify(mie::mext::CLEAR);

            // Claim the interrupt, unwrap() as we know an interrupt exists
            // Once claimed this interrupt won't fire until it's completed
            // NOTE: The interrupt is no longer pending in the PLIC
            loop {
                let interrupt = PLIC.next_pending();

                match interrupt {
                    Some(irq) => {
                        // Safe as interrupts are disabled
                        PLIC.save_interrupt(irq);
                    }
                    None => {
                        // Enable generic interrupts
                        CSR.mie.modify(mie::mext::SET);
                        break;
                    }
                }
            }
        }

        mcause::Interrupt::Unknown => {
            panic!("interrupt of unknown cause");
        }
    }
}

/// Trap handler for board/chip specific code.
///
/// For the e310 this gets called when an interrupt occurs while the chip is
/// in kernel mode.
#[export_name = "_start_trap_rust_from_kernel"]
pub unsafe extern "C" fn start_trap_rust() {
    match mcause::Trap::from(CSR.mcause.extract()) {
        mcause::Trap::Interrupt(interrupt) => {
            handle_interrupt(interrupt);
        }
        mcause::Trap::Exception(exception) => {
            handle_exception(exception);
        }
    }
}

/// Function that gets called if an interrupt occurs while an app was running.
/// mcause is passed in, and this function should correctly handle disabling the
/// interrupt that fired so that it does not trigger again.
#[export_name = "_disable_interrupt_trap_rust_from_app"]
pub unsafe extern "C" fn disable_interrupt_trap_handler(mcause_val: u32) {
    match mcause::Trap::from(mcause_val as usize) {
        mcause::Trap::Interrupt(interrupt) => {
            handle_interrupt(interrupt);
        }
        _ => {
            panic!("unexpected non-interrupt\n");
        }
    }
}
