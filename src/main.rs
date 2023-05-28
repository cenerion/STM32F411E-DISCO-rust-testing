#![no_main]
#![no_std]
#![allow(clippy::empty_loop)]

use cortex_m_rt::entry;
use cortex_m::asm;
use stm32f4xx_hal as hal;

// panic handlers
use rtt_target::{
    rtt_init_print,
    rprintln,
};
use panic_rtt_target as _;
//use panic_halt as _;

use hal::{
    prelude::*,
    pac,
    gpio::PinState,
    otg_fs::{UsbBus, UsbBusType, USB},
};

use l3gd20::L3gd20;
use lsm303dlhc::{Lsm303dlhc, AccelOdr};

use usb_device::prelude::*;
use usbd_serial::SerialPort;


fn _remap(val: i16, i_min: i16, i_max: i16, o_min: f32, o_max: f32) -> f32 {
    let slope = (o_max - o_min) / (i_max - i_min) as f32;
    o_min + slope * (val - i_min) as f32
}

fn normalize_a(acc:lsm303dlhc::I16x3) -> (f32, f32, f32) {
    (
        acc.x as f32 * (2.0_f32 / i16::MAX as f32),
        acc.y as f32 * (2.0_f32 / i16::MAX as f32),
        acc.z as f32 * (2.0_f32 / i16::MAX as f32),
    )
}

fn normalize_g(acc:l3gd20::I16x3) -> (f32, f32, f32) {
    (
        acc.x as f32 * (2.0_f32 / i16::MAX as f32),
        acc.y as f32 * (2.0_f32 / i16::MAX as f32),
        acc.z as f32 * (2.0_f32 / i16::MAX as f32),
    )
}


use core::fmt::Write;
struct Foo<'a, 'b, A: usb_device::bus::UsbBus + 'static>{
    serial: &'a mut SerialPort<'b, A>
}

impl<'a, 'b, A: usb_device::bus::UsbBus + 'static> Foo<'a, 'b, A>{
    fn new(serial: &'a mut SerialPort<'b, A>) -> Foo<'a, 'b, A> {
        Foo { serial }
    }
}

impl<A: usb_device::bus::UsbBus> Write for Foo<'_, '_, A> {
    fn write_str(&mut self, data: &str) -> Result<(), core::fmt::Error> { 
        match self.serial.write(data.as_bytes()) {
            Ok(_size) => Ok(()),
            Err(_) => Err(core::fmt::Error),
        }
    }
}


#[entry]
fn main() -> ! {
    if let (Some(dp), Some(_cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        rtt_init_print!();

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpiod = dp.GPIOD.split();
        let gpioe = dp.GPIOE.split();

        let mut led_green = gpiod.pd12.into_push_pull_output();
        let mut led_orange = gpiod.pd13.into_push_pull_output();
        // let mut led_red = gpiod.pd14.into_push_pull_output();
        // let mut led_blue = gpiod.pd15.into_push_pull_output();

        let sck = gpioa.pa5;
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7;
        let cs = gpioe.pe3.into_push_pull_output_in_state(PinState::High);

        let scl = gpiob.pb6;
        let sda = gpiob.pb9;

        let usb_dm = gpioa.pa11;
        let usb_dp = gpioa.pa12;


        let spi1 = dp.SPI1.spi(
            (sck, miso, mosi), 
            l3gd20::MODE,
            1000.kHz(), 
            &clocks
        );

        let i2c1 = dp.I2C1.i2c((scl,sda), 400.kHz(), &clocks);

        let usb = USB::new(
            (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK), 
            (usb_dm, usb_dp), 
            &clocks
        );



        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;
        
        unsafe{
            USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
        }

        let mut usb_serial = SerialPort::new(unsafe{USB_BUS.as_ref().unwrap()});
        //let mut usb_serial2 = SerialPort::new(unsafe{USB_BUS.as_ref().unwrap()});

        let mut usb_dev = UsbDeviceBuilder::new(
            unsafe{USB_BUS.as_ref().unwrap()}, 
            UsbVidPid(0x16c0, 0x27dd)
        )
        .manufacturer("rust embedded")
        .product("usb echo server")
        .serial_number("4242")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();



        let mut gyro = L3gd20::new(spi1, cs).expect("gyro not working");
        let mut acc = Lsm303dlhc::new(i2c1).expect("accel not working");
        
        acc.accel_odr(AccelOdr::Hz10).unwrap();
        acc.set_accel_sensitivity(lsm303dlhc::Sensitivity::G1).unwrap();

        rprintln!("who am i: {}", gyro.who_am_i().unwrap() );


        // Create a delay abstraction based on general-pupose 32-bit timer TIM5
        let mut delay = dp.TIM5.delay_us(&clocks);

        loop {
            led_green.toggle();
            //rprintln!("gyro: {:?}", gyro.gyro().unwrap());
            let measure = normalize_a(acc.accel().unwrap());
            let compass = normalize_a(acc.mag().unwrap());
            let gyroscope = normalize_g(gyro.gyro().unwrap());

            rprintln!("acce: {:-6.4} {:-6.4} {:-6.4}", measure.0, measure.1, measure.2 );

            let mut serial_writer = Foo::new(&mut usb_serial);

            if usb_dev.poll(&mut [serial_writer.serial]) {

                led_orange.set_high();
                let _ = write!(&mut serial_writer, "accel_x:{:-6.4},accel_y:{:-6.4},accel_z:{:-6.4},mag_x:{:-6.4},mag_y:{:-6.4},mag_z:{:-6.4},gyro_x:{:-6.4},gyro_y:{:-6.4},gyro_z:{:-6.4}\r\n", measure.0, measure.1, measure.2, compass.0, compass.1, compass.2, gyroscope.0, gyroscope.1, gyroscope.2 );
                led_orange.set_low();
                
                /* 
                match usb_serial.write(b"aaaaaaa") {
                    Ok(count) => {
                        led_orange.set_low();// count bytes were written
                    },
                    Err(UsbError::WouldBlock) => continue,// No data could be written (buffers full)
                    Err(err) => continue// An error occurred
                };
                */
            }

            delay.delay_ms(100_u32);
        }
    }

    loop {
        asm::bkpt();
    }
}