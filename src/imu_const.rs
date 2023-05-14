#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;

use rp2040_hal::clocks::Clock;

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use serial_write::Writer;

use fugit::RateExtU32;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const CALIB: bno055::BNO055Calibration = bno055::BNO055Calibration {
    acc_offset_x_lsb: 237,
    acc_offset_x_msb: 255,
    acc_offset_y_lsb: 212,
    acc_offset_y_msb: 255,
    acc_offset_z_lsb: 227,
    acc_offset_z_msb: 255,
    mag_offset_x_lsb: 72,
    mag_offset_x_msb: 0,
    mag_offset_y_lsb: 252,
    mag_offset_y_msb: 255,
    mag_offset_z_lsb: 156,
    mag_offset_z_msb: 253,
    gyr_offset_x_lsb: 255,
    gyr_offset_x_msb: 255,
    gyr_offset_y_lsb: 255,
    gyr_offset_y_msb: 255,
    gyr_offset_z_lsb: 1,
    gyr_offset_z_msb: 0,
    acc_radius_lsb: 232,
    acc_radius_msb: 3,
    mag_radius_lsb: 12,
    mag_radius_msb: 2
};

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the USB bus
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set the serial port
    let mut serial = SerialPort::new(&usb_bus);

    // Set a USB device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();
    
    let sda_pin = pins.gpio16.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio17.into_mode::<hal::gpio::FunctionI2C>();

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut writer = Writer::new();

    // Wait 1 sec after power supply to BNO055.
    for _ in 0..200 {
        delay.delay_ms(5);
        let _ = usb_dev.poll(&mut [&mut serial]);
    }

    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();
    imu.init(&mut delay).unwrap();
    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).unwrap();
    
    imu.set_calibration_profile(CALIB, &mut delay).unwrap();
    
    loop {
        for _ in 0..20 {
            delay.delay_ms(5);
            let _ = usb_dev.poll(&mut [&mut serial]);
        }
        let eular = imu.euler_angles().unwrap();
        let _ = writer.write_str("Pitch: ", &mut serial);
        let _ = writer.write_f32(eular.a, 2, &mut serial);
        let _ = writer.write_str(", Roll: ", &mut serial);
        let _ = writer.write_f32(eular.b, 2, &mut serial);
        let _ = writer.write_str(", Yaw: ", &mut serial);
        let _ = writer.writeln_f32(eular.c, 2, &mut serial);
    }
}

// End of file