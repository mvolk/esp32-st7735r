# ST7735R Prototyping Library for ESP32 & ESP-IDF

This library provides low-level support for interfacing
with a Sitronix ST7735R TFT LCD display controller IC
via 4-wire SPI on an Espressif ESP32 system using the
ESP-IDF development platform.

## For Prototyping

This library does not emphasize performance or memory
efficiency. Nor does it emphasize automated testing,
for that matter. For these reasons, this library is
best used for prototyping with specific hardware.
Which also happens to be what it was designed for. :)

This library is particularly useful for learning
how to integrate an ST7335R-controlled TFT in your
ESP-IDF ESP32 project, and for fine-tuning the
configuration parameters supplied to the TFT
controller.

## Supported Ecosystem

This library aims to support the following... and only the following:

* **Microcontroller:** Espressif [ESP32 modules](https://www.espressif.com/en/products/modules/esp32)
* **TFT Controller:** Sitronix [ST7735R](https://cdn-shop.adafruit.com/datasheets/ST7735R_V0.2.pdf) rev 0.2
* **Interface:** SPI 4-wire sans MISO, with an
  additional line for data/command signaling, and
  optionally with lines for hardware reset and backlight control
* **Development Platform:** [ESP-IDF v4.1+](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html),
  Espressif's freeRTOS-based platform for ESP32 development

## Boundaries

* **SPI Bus:** this library does not configure or
  initialize the SPI Bus itself, though it does configure
  how the ESP32 will communicate with the ST7735R over that
  bus (e.g. adding the device to the bus, setting the SPI
  mode and clock speed for communication with the device,
  etc)
* **Display Panel:** this library does not support any
  specific TFT panels
* **GFX:** this library does not provide features to draw
  geometries, text, images, etc.
* **Peripherals:** this library aims to be compatible
  with the use of additional devices, such as an SD card
  reader, on the same SPI bus, but it does not provide
  any support for such additional peripherals


## Installation

Using the modern ESP-IDF with CMake, Git and the recommended project structure (with a
`components` folder), the easiest way to install this library is to add it to the `components`
folder as a Git submodule:

```shell
cd ${YOUR_PROJECT_ROOT}/components
git submodule add https://github.com/mvolk/esp32-st7735r.git
git commit -m "Add esp32-st7735r component"

```

## API

The following lists are intended to provide quick
reference. Functions may appear in multiple lists
if they are likely to be used in multiple contexts.

See [st7735r.h](./include/st7735r.h) for detailed documentation of the following API features.

### Initialization

* `st7735r_init(..)` adds the device to the SPI
bus and initializes the device data structure

### Low-Level Commands

The following functions implement commands described
in the ST7735R datasheet. Most of these are used
primarily for configuration at startup. Those
commonly used during operation are included in the
more concise "Operation" functions list below.

* `st7735r_frmctr1(...)`
* `st7735r_frmctr2(...)`
* `st7735r_frmctr3(...)`
* `st7735r_pwctr1(...)`
* `st7735r_pwctr2(...)`
* `st7735r_pwctr3(...)`
* `st7735r_pwctr4(...)`
* `st7735r_pwctr5(...)`
* `st7735r_vmctr1(...)`
* `st7735r_invon(...)`
* `st7735r_invoff(...)`
* `st7735r_invctr(...)`
* `st7735r_madctl(...)`
* `st7735r_colmod(...)`
* `st7735r_caset(...)`
* `st7735r_raset(...)`
* `st7735r_ramwr(...)`
* `st7735r_gmctrp1(...)`
* `st7735r_gmctrn1(...)`
* `st7735r_noron(...)`
* `st7735r_ptlon(...)`
* `st7735r_dispon(...)`
* `st7735r_dispoff(...)`
* `st7735r_swreset(...)`
* `st7735r_slpin(...)`
* `st7735r_slpout(...)`

There is also an extensive set of #define'd named
values for the various command parameters. Each
command function's documentation in the
[header file](./include/st7735r.h)
includes a list of relevant named values.

### Operation

* `st7735r_hwreset(...)` hardware reset via optional `rst` pin
* `st7735r_swreset(...)` software reset via command
* `st7735r_slpin(...)` enter sleep mode
* `st7735r_slpout(...)` exit sleep mode
* `st7735r_backlight(...)` backlight level via optional
  `bckl` pin
* `st7735r_set_dispon(...)` turn the display on
* `st7735r_set_dispoff(...)` turn the display off
* `st7735r_set_caset(...)` specify where to write pixels
* `st7735r_set_raset(...)` specify which to write pixels
* `st7735r_set_ramwr(...)` write pixels
* `st7735r_paint(...)` conveniently write pixels in 1 call

### Utility

* `st7735r_dca(...)` builds `dca` arg for `pwctr[3-5]`
  calls
* `st7735r_paint(...)` conveniently write pixels in 1 call
* `st7735r_rgb565(...)` builds uint16_t color values from RGB values

### Low-Level Data Transport

These are the functions that are used internally to
transmit data via SPI. They are also exposed externally
to support use of commands and features not yet exposed in this library.

* `st7735r_cmd_init(...)` initialize an SPI command
  transaction
* `st7735r_data_init(...)` intialize an SPI data
  transaction
* `st7735r_enqueue(...)` send SPI transactions
  asynchronously
* `st7735r_await(...)` wait for SPI transactions
  to conclude
* `st7735r_exec(...)` execute SPI transactions
  synchronously
* `st7735r_send_cmd(...)` synchronously send a single
  SPI command transaction
* `st7735r_send_data(...)` synchronously send a single
  SPI data transaction

## Technical Notes

* The functions provided by this library are not thread-safe.
  External sychronization is required if multiple threads will
  be accessing the same device. See the
  [ESP-IDF SPI Master](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)
  documentation for details.

* DMA was used for testing, but the library does not
  require the use of, set up or depend in any known way on DMA.
  However, if using DMA, data buffers to be transmitted over
  SPI must be in RAM.  See the
  [ESP-IDF SPI Master](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)
  documentation for details.

* While this library is agnostic concerning SPI bus configuration,
  it is opinionated about the SPI communications settings for
  communicating with the ST7735R. These settings are specific to the
  device, and do not constrain the settings applied to other devices
  on the same bus, but may conflict with desired settings. The settings
  in question are SPI Mode (0), frequency (10MHz) and transaction
  queue depth (6).

* Because RGB/565 is stored in 2 bytes (16 bits), color values are
  represented using the type uint16_t. However, the ESP32 arranges
  multi-byte integers in memory starting from the least significant
  byte and ending with the most significant byte (little endian
  byte order, not to be confused with MSB first/LSB fist). This
  results in corruption of the color values when a uint16_t color
  value is doled out to the ST7735R byte-by-byte in the order in
  which those bytes are stored in memory. This can be solved by
  storing RGB/565 values as two-byte arrays, or by using uint16_t
  with the byte inverted. In the latter example, the apparent
  format (to ESP code) is G[2:0]R[4:0]B[4:0]G[5:3], but because
  this bytes are inverted in memory, when read bytewise this comes
  out as B[4:0]G[5:3]G[2:0]R[4:0], which is the format expected by
  the ST7735R. The st7735r_rgb565(...) function is provided to
  compensate for byte order complications.

## Support

Technical support is not available.

## Contributing

### Bug Reports

Please feel free to report bugs by [opening a new issue via Github](https://github.com/mvolk/esp32-st7735r/issues/new).

Please do not use Github issues to request technical support.

### Pull Requests

Pull requests that further the goals of this library are welcome.

The [issues list](https://github.com/mvolk/esp32-st7735r/issues) is a
good place to look for ideas if you would like to contribute but don't
have a specific contribution in mind.

If you would like to extend support to additional MCUs or development platforms,
or to specific TFT panels, please consider forking or building on top of this library.

## Credits

While this library is not a derivative of the [Adafruit-ST7735-Library](Adafruit-ST7735-Library)
for the Arduino platform, a study of the implementation of that library helped
fill in the gaps left by the terse datasheet.

The Espressif ESP-IDF [SPI master example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_master/lcd)
provided a valuable jump-start on SPI programming for the ESP32 on the ESP-IDF platform.

Espressif's [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html)
was invaluable.

A Sitronix [ST7735R datasheet](https://cdn-shop.adafruit.com/datasheets/ST7735R_V0.2.pdf)
and a few TFT breakout boards were sourced from [Adafruit](https://www.adafruit.com/)
and made this library possible.

## License & Copyright

This library is released under the MIT License.

Copyright (c) 2020 Michael Volk

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions
of the Software.

## Liability Notice

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
