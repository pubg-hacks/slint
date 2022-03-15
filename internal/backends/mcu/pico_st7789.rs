// Copyright © SixtyFPS GmbH <info@slint-ui.com>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-commercial

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec;
use cortex_m::singleton;
pub use cortex_m_rt::entry;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::rate::*;
use hal::dma::{DMAExt, SingleChannel, WriteTarget};
use hal::pio::ValidStateMachine;
use rp_pico::hal::{self, pac, prelude::*, Timer};

use defmt_rtt as _; // global logger

#[cfg(feature = "panic-probe")]
use panic_probe as _;

#[alloc_error_handler]
fn oom(layout: core::alloc::Layout) -> ! {
    panic!("Out of memory {:?}", layout);
}
use alloc_cortex_m::CortexMHeap;

use crate::lengths::PhysicalLength;
use crate::{Devices, PhysicalRect, PhysicalSize};

const HEAP_SIZE: usize = 128 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

pub fn init() {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) }

    let sio = hal::sio::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI1);

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        18_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let spi = singleton!(:shared_bus::BusManagerSimple<hal::Spi<hal::spi::Enabled,  pac::SPI1, 8>> = shared_bus::BusManagerSimple::new(spi)).unwrap();

    let rst = pins.gpio15.into_push_pull_output();

    let dc = pins.gpio8.into_push_pull_output();
    let cs = pins.gpio9.into_push_pull_output();
    let di = display_interface_spi::SPIInterface::new(spi.acquire_spi(), dc, cs);

    const DISPLAY_SIZE: PhysicalSize = PhysicalSize::new(320, 240);
    let mut display =
        st7789::ST7789::new(di, rst, DISPLAY_SIZE.width as _, DISPLAY_SIZE.height as _);

    // Turn on backlight
    {
        let mut bl = pins.gpio13.into_push_pull_output();
        bl.set_low().unwrap();
        delay.delay_us(10_000);
        bl.set_high().unwrap();
    }

    display.init(&mut delay).unwrap();
    display.set_orientation(st7789::Orientation::Landscape).unwrap();

    let touch = xpt2046::XPT2046::new(
        pins.gpio17.into_pull_down_input(),
        pins.gpio16.into_push_pull_output(),
        spi.acquire_spi(),
    )
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let dma = pac.DMA.split(&mut pac.RESETS);
    // SAFETY: This is not safe :-(
    let stolen_spi = unsafe {
        hal::spi::Spi::<_, _, 8>::new(rp_pico::hal::pac::Peripherals::steal().SPI1).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            18_000_000u32.Hz(),
            &embedded_hal::spi::MODE_3,
        )
    };
    let pio = PioTransfer::Idle(
        dma.ch0,
        vec![Rgb565::default(); DISPLAY_SIZE.width as _].leak(),
        stolen_spi,
    );

    crate::init_with_display(PicoDevices {
        display,
        touch,
        last_touch: Default::default(),
        timer,
        buffer: vec![Rgb565::default(); DISPLAY_SIZE.width as _].leak(),
        pio: Some(pio),
    });
}

enum PioTransfer<TO: WriteTarget, CH: SingleChannel> {
    Idle(CH, &'static mut [super::TargetPixel], TO),
    Running(hal::dma::SingleBuffering<CH, PartialReadBuffer, TO>),
}

impl<TO: WriteTarget<TransmittedWord = u8>, CH: SingleChannel> PioTransfer<TO, CH> {
    fn wait(self) -> (CH, &'static mut [super::TargetPixel], TO) {
        match self {
            PioTransfer::Idle(a, b, c) => (a, b, c),
            PioTransfer::Running(dma) => {
                let (a, b, c) = dma.wait();
                (a, b.0, c)
            }
        }
    }
}

struct PicoDevices<Display, Touch, PioTransfer> {
    display: Display,
    touch: Touch,
    last_touch: Option<i_slint_core::graphics::Point>,
    timer: Timer,
    buffer: &'static mut [super::TargetPixel],
    pio: Option<PioTransfer>,
}

impl<
        DI: WriteOnlyDataCommand,
        RST: OutputPin,
        IRQ: InputPin,
        CS: OutputPin<Error = IRQ::Error>,
        SPI: Transfer<u8>,
        TO: WriteTarget<TransmittedWord = u8>,
        CH: SingleChannel,
    > Devices
    for PicoDevices<st7789::ST7789<DI, RST>, xpt2046::XPT2046<IRQ, CS, SPI>, PioTransfer<TO, CH>>
{
    fn screen_size(&self) -> PhysicalSize {
        self.display.screen_size()
    }

    fn render_line(
        &mut self,
        line: PhysicalLength,
        dirty_region: crate::renderer::DirtyRegion,
        fill_buffer: &mut dyn FnMut(&mut [Rgb565]),
    ) {
        //let [b1, b2] = &mut self.buffers;
        //let line_buffer = b1.take().or_else(|| b2.take()).unwrap();
        fill_buffer(self.buffer);

        let (ch, mut b, spi) = self.pio.take().unwrap().wait();
        core::mem::swap(&mut self.buffer, &mut b);
        // We send empty data
        self.display.fill_contiguous(
            &embedded_graphics::primitives::Rectangle::new(
                Point::new(dirty_region.origin.x as i32, line.get() as i32),
                Size::new(dirty_region.size.width as u32, 1),
            ),
            core::iter::empty(),
        );
        self.pio = Some(PioTransfer::Running(
            hal::dma::SingleBufferingConfig::new(
                ch,
                PartialReadBuffer(b, dirty_region.min_x() as _..dirty_region.max_x() as _),
                spi,
            )
            .start(),
        ));
    }

    fn debug(&mut self, text: &str) {
        self.display.debug(text)
    }

    fn read_touch_event(&mut self) -> Option<i_slint_core::input::MouseEvent> {
        let button = i_slint_core::items::PointerEventButton::left;
        self.touch
            .read()
            .map_err(|_| ())
            .unwrap()
            .map(|point| {
                let size = self.display.screen_size().to_f32();
                let pos = euclid::point2(point.x * size.width, point.y * size.height);
                match self.last_touch.replace(pos) {
                    Some(_) => i_slint_core::input::MouseEvent::MouseMoved { pos },
                    None => i_slint_core::input::MouseEvent::MousePressed { pos, button },
                }
            })
            .or_else(|| {
                self.last_touch
                    .take()
                    .map(|pos| i_slint_core::input::MouseEvent::MouseReleased { pos, button })
            })
    }

    fn time(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.timer.get_counter())
    }
}

struct PartialReadBuffer(&'static mut [Rgb565], core::ops::Range<usize>);
unsafe impl embedded_dma::ReadBuffer for PartialReadBuffer {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const <Self as embedded_dma::ReadBuffer>::Word, usize) {
        let act_slice = &self.0[self.1.clone()];
        (act_slice.as_ptr() as *const u8, act_slice.len() * core::mem::size_of::<Rgb565>())
    }
}

mod xpt2046 {
    use embedded_hal::blocking::spi::Transfer;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_time::rate::Extensions;
    use euclid::default::Point2D;

    pub struct XPT2046<IRQ: InputPin, CS: OutputPin, SPI: Transfer<u8>> {
        irq: IRQ,
        cs: CS,
        spi: SPI,
    }

    impl<PinE, IRQ: InputPin<Error = PinE>, CS: OutputPin<Error = PinE>, SPI: Transfer<u8>>
        XPT2046<IRQ, CS, SPI>
    {
        pub fn new(irq: IRQ, mut cs: CS, spi: SPI) -> Result<Self, PinE> {
            cs.set_high()?;
            Ok(Self { irq, cs, spi })
        }

        pub fn read(&mut self) -> Result<Option<Point2D<f32>>, Error<PinE, SPI::Error>> {
            if self.irq.is_low().map_err(|e| Error::Pin(e))? {
                const CMD_X_READ: u8 = 0b10010000;
                const CMD_Y_READ: u8 = 0b11010000;
                const CMD_Z1_READ: u8 = 0b10110001;
                //const CMD_Z2_READ: u8 = 0b11000001;

                // These numbers were measured approximately.
                const MIN_X: u32 = 1900;
                const MAX_X: u32 = 30300;
                const MIN_Y: u32 = 2300;
                const MAX_Y: u32 = 30300;
                const Z_THRESHOLD: u32 = 1000;

                // FIXME! how else set the frequency to this device
                unsafe { set_spi_freq(3_000_000u32.Hz()) };

                self.cs.set_low().map_err(|e| Error::Pin(e))?;

                macro_rules! xchg {
                    ($byte:expr) => {
                        match self
                            .spi
                            .transfer(&mut [$byte, 0, 0])
                            .map_err(|e| Error::Transfer(e))?
                        {
                            [_, h, l] => ((*h as u32) << 8) | (*l as u32),
                            _ => return Err(Error::InternalError),
                        }
                    };
                }

                let z1 = xchg!(CMD_Z1_READ);
                //let z2 = xchg!(CMD_Z2_READ);
                //let z = z1 as i32 + 4095 - z2 as i32;

                if z1 < Z_THRESHOLD {
                    xchg!(0);
                    self.cs.set_high().map_err(|e| Error::Pin(e))?;
                    unsafe { set_spi_freq(18_000_000u32.Hz()) };
                    return Ok(None);
                }

                xchg!(CMD_X_READ | 1); // Dummy read, first read is a outlier

                let mut point = Point2D::new(0u32, 0u32);
                for _ in 0..10 {
                    let y = xchg!(CMD_Y_READ);
                    let x = xchg!(CMD_X_READ);
                    point += euclid::vec2(i16::MAX as u32 - x, y)
                }
                xchg!(0);
                self.cs.set_high().map_err(|e| Error::Pin(e))?;
                unsafe { set_spi_freq(18_000_000u32.Hz()) };

                point /= 10;
                Ok(Some(euclid::point2(
                    point.x.saturating_sub(MIN_X) as f32 / (MAX_X - MIN_X) as f32,
                    point.y.saturating_sub(MIN_Y) as f32 / (MAX_Y - MIN_Y) as f32,
                )))
            } else {
                Ok(None)
            }
        }
    }

    pub enum Error<PinE, TransferE> {
        Pin(PinE),
        Transfer(TransferE),
        InternalError,
    }

    unsafe fn set_spi_freq(freq: impl Into<super::Hertz<u32>>) {
        // FIXME: the touchscreen and the LCD have different frequencies, but we cannot really set different frequencies to different SpiProxy without this hack
        rp_pico::hal::spi::Spi::<_, _, 8>::new(rp_pico::hal::pac::Peripherals::steal().SPI1)
            .set_baudrate(125_000_000u32.Hz(), freq);
    }
}

#[cfg(not(feature = "panic-probe"))]
#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Safety: it's ok to steal here since we are in the panic handler, and the rest of the code will not be run anymore
    let (mut pac, core) = unsafe { (pac::Peripherals::steal(), pac::CorePeripherals::steal()) };

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led = pins.led.into_push_pull_output();
    led.set_high().unwrap();

    // Re-init the display
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI1);
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        4_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let rst = pins.gpio15.into_push_pull_output();
    let dc = pins.gpio8.into_push_pull_output();
    let cs = pins.gpio9.into_push_pull_output();
    let di = display_interface_spi::SPIInterface::new(spi, dc, cs);
    let mut display = st7789::ST7789::new(di, rst, 320, 240);

    // Turn on backlight
    {
        let mut bl = pins.gpio13.into_push_pull_output();
        bl.set_low().unwrap();
        delay.delay_us(10_000);
        bl.set_high().unwrap();
    }

    use core::fmt::Write;
    use embedded_graphics::{
        draw_target::DrawTarget,
        mono_font::{ascii::FONT_6X10, MonoTextStyle},
        prelude::*,
        text::Text,
    };

    display.init(&mut delay).unwrap();
    display.set_orientation(st7789::Orientation::Landscape).unwrap();
    display.fill_solid(&display.bounding_box(), Rgb565::new(0x00, 0x25, 0xff)).unwrap();

    struct WriteToScreen<'a, D> {
        x: i32,
        y: i32,
        width: i32,
        style: MonoTextStyle<'a, Rgb565>,
        display: &'a mut D,
    }
    let mut writer = WriteToScreen {
        x: 0,
        y: 1,
        width: display.bounding_box().size.width as i32 / 6 - 1,
        style: MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE),
        display: &mut display,
    };
    impl<'a, D: DrawTarget<Color = Rgb565>> Write for WriteToScreen<'a, D> {
        fn write_str(&mut self, mut s: &str) -> Result<(), core::fmt::Error> {
            while !s.is_empty() {
                let (x, y) = (self.x, self.y);
                let end_of_line = s
                    .find(|c| {
                        if c == '\n' || self.x > self.width {
                            self.x = 0;
                            self.y += 1;
                            true
                        } else {
                            self.x += 1;
                            false
                        }
                    })
                    .unwrap_or(s.len());
                let (line, rest) = s.split_at(end_of_line);
                let sz = self.style.font.character_size;
                Text::new(line, Point::new(x * sz.width as i32, y * sz.height as i32), self.style)
                    .draw(self.display)
                    .map_err(|_| core::fmt::Error)?;
                s = rest.strip_prefix('\n').unwrap_or(rest);
            }
            Ok(())
        }
    }
    write!(writer, "{}", info).unwrap();

    loop {
        delay.delay_ms(100);
        led.set_low().unwrap();
        delay.delay_ms(100);
        led.set_high().unwrap();
    }
}
