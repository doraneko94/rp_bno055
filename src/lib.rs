#![no_std]

use rp2040_hal::rom_data;

pub const FLASH_BLOCK_SIZE: u32 = 0xFFFF;
pub const FLASH_SECTOR_SIZE: usize = 0x1000;
pub const FLASH_PAGE_SIZE: u32 = 0x100;
pub const FLASH_BLOCK_CMD: u8 = 0x20;
pub const XIP_BASE: u32 = 0x1000_0000;
pub const FLASH_END: u32 = 0x0020_0000;

#[inline(never)]
#[link_section = ".data.ram_func"]
pub fn write_flash(data: &[u8]) {
    let size = data.len();
    let addr = FLASH_END - size as u32;
    unsafe {
        cortex_m::interrupt::free(|_cs| {
            rom_data::connect_internal_flash();
            rom_data::flash_exit_xip();
            rom_data::flash_range_erase(addr, FLASH_SECTOR_SIZE, FLASH_BLOCK_SIZE, FLASH_BLOCK_CMD);
            rom_data::flash_range_program(addr, data.as_ptr(), size);
            rom_data::flash_flush_cache(); // Get the XIP working again
            rom_data::flash_enter_cmd_xip(); // Start XIP back up
        });
    }
}

pub fn read_flash(data: &mut [u8]) {
    let size = data.len();
    let addr = XIP_BASE + FLASH_END - size as u32;
    unsafe {
        let _ = core::slice::from_raw_parts(addr as *const u8, size)
            .iter()
            .zip(
                data.iter_mut()
            )
            .map(|(&elem, save)| {
                *save = elem;
            })
            .collect::<()>();
    }
}

pub fn gen_identification(data: &mut [u8], key: u8) {
    let key_usize = key as usize;
    let _ = data.iter_mut()
        .enumerate()
        .map(|(i, save)| {
            *save = (i % key_usize) as u8;
        })
        .collect::<()>();
}

pub fn check_identification(data: &[u8], key: u8) -> bool {
    let key_usize = key as usize;
    let mut valid = true;
    for (i, &elem) in data.iter().enumerate() {
        if (i & key_usize) as u8 != elem {
            valid = false;
            break;
        }
    }
    
    valid
}