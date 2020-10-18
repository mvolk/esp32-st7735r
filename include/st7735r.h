/*
 * MIT License
 *
 * Copyright (c) 2020 Michael Volk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ST7735R_H
#define ST7735R_H

#include <freertos/FreeRTOS.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>


#define ST7735R_CMD_NOP             0x00
#define ST7735R_CMD_SWRESET         0x01
#define ST7735R_CMD_RDDID           0x04
#define ST7735R_CMD_RDDST           0x09
#define ST7735R_CMD_SLPIN           0x10
#define ST7735R_CMD_SLPOUT          0x11
#define ST7735R_CMD_PTLON           0x12
#define ST7735R_CMD_NORON           0x13
#define ST7735R_CMD_INVOFF          0x20
#define ST7735R_CMD_INVON           0x21
#define ST7735R_CMD_DISPOFF         0x28
#define ST7735R_CMD_DISPON          0x29
#define ST7735R_CMD_CASET           0x2A
#define ST7735R_CMD_RASET           0x2B
#define ST7735R_CMD_RAMWR           0x2C
#define ST7735R_CMD_RAMRD           0x2E
#define ST7735R_CMD_PTLAR           0x30
#define ST7735R_CMD_TEOFF           0x34
#define ST7735R_CMD_TEON            0x35
#define ST7735R_CMD_MADCTL          0x36
#define ST7735R_CMD_COLMOD          0x3A
#define ST7735R_CMD_RDID1           0xDA
#define ST7735R_CMD_FRMCTR1         0xB1
#define ST7735R_CMD_FRMCTR2         0xB2
#define ST7735R_CMD_FRMCTR3         0xB3
#define ST7735R_CMD_INVCTR          0xB4
#define ST7735R_CMD_DISSET5         0xB6
#define ST7735R_CMD_PWCTR1          0xC0
#define ST7735R_CMD_PWCTR2          0xC1
#define ST7735R_CMD_PWCTR3          0xC2
#define ST7735R_CMD_PWCTR4          0xC3
#define ST7735R_CMD_PWCTR5          0xC4
#define ST7735R_CMD_VMCTR1          0xC5
#define ST7735R_CMD_RDID2           0xDB
#define ST7735R_CMD_RDID3           0xDC
#define ST7735R_CMD_RDID4           0xDD
#define ST7735R_CMD_GMCTRP1         0xE0
#define ST7735R_CMD_GMCTRN1         0xE1
#define ST7735R_CMD_PWCTR6          0xFC

#define ST7735R_CFG_INV_LINE_NORM   0x04
#define ST7735R_CFG_INV_LINE_IDLE   0x02
#define ST7735R_CFG_INV_LINE_PART   0x01

#define ST7735R_CFG_MIRROR_X        0x40
#define ST7735R_CFG_MIRROR_Y        0x80
#define ST7735R_CFG_EXCHANGE_XY     0x20
#define ST7735R_CFG_REFRESH_RTL     0x04
#define ST7735R_CFG_REFRESH_BTT     0x10
#define ST7735R_CFG_BGR             0x08

#define ST7735R_CFG_12_BIT_COLOR    0x03
#define ST7735R_CFG_16_BIT_COLOR    0x05
#define ST7735R_CFG_18_BIT_COLOR    0x06

#define ST7735R_CFG_BCKL_OFF        0x00
#define ST7735R_CFG_BCKL_ON         0xFF

#define ST7735R_CFG_AVDD_450        0x00
#define ST7735R_CFG_AVDD_460        0x01
#define ST7735R_CFG_AVDD_470        0x02
#define ST7735R_CFG_AVDD_480        0x03
#define ST7735R_CFG_AVDD_490        0x04
#define ST7735R_CFG_AVDD_500        0x05
#define ST7735R_CFG_AVDD_510        0x06

#define ST7735R_CFG_GVDD_470        0x00
#define ST7735R_CFG_GVDD_465        0x01
#define ST7735R_CFG_GVDD_460        0x02
#define ST7735R_CFG_GVDD_455        0x03
#define ST7735R_CFG_GVDD_450        0x04
#define ST7735R_CFG_GVDD_445        0x05
#define ST7735R_CFG_GVDD_440        0x06
#define ST7735R_CFG_GVDD_435        0x07
#define ST7735R_CFG_GVDD_430        0x08
#define ST7735R_CFG_GVDD_425        0x09
#define ST7735R_CFG_GVDD_420        0x0A
#define ST7735R_CFG_GVDD_415        0x0B
#define ST7735R_CFG_GVDD_410        0x0C
#define ST7735R_CFG_GVDD_405        0x0D
#define ST7735R_CFG_GVDD_400        0x0E
#define ST7735R_CFG_GVDD_395        0x0F
#define ST7735R_CFG_GVDD_390        0x10
#define ST7735R_CFG_GVDD_385        0x11
#define ST7735R_CFG_GVDD_380        0x12
#define ST7735R_CFG_GVDD_375        0x13
#define ST7735R_CFG_GVDD_370        0x14
#define ST7735R_CFG_GVDD_365        0x15
#define ST7735R_CFG_GVDD_360        0x16
#define ST7735R_CFG_GVDD_355        0x17
#define ST7735R_CFG_GVDD_350        0x18
#define ST7735R_CFG_GVDD_345        0x19
#define ST7735R_CFG_GVDD_340        0x1A
#define ST7735R_CFG_GVDD_335        0x1B
#define ST7735R_CFG_GVDD_330        0x1C
#define ST7735R_CFG_GVDD_325        0x1D
#define ST7735R_CFG_GVDD_320        0x1E
#define ST7735R_CFG_GVDD_315        0x1F

#define ST7735R_CFG_GVCL_NEG_470    0x00
#define ST7735R_CFG_GVCL_NEG_465    0x01
#define ST7735R_CFG_GVCL_NEG_460    0x02
#define ST7735R_CFG_GVCL_NEG_455    0x03
#define ST7735R_CFG_GVCL_NEG_450    0x04
#define ST7735R_CFG_GVCL_NEG_445    0x05
#define ST7735R_CFG_GVCL_NEG_440    0x06
#define ST7735R_CFG_GVCL_NEG_435    0x07
#define ST7735R_CFG_GVCL_NEG_430    0x08
#define ST7735R_CFG_GVCL_NEG_425    0x09
#define ST7735R_CFG_GVCL_NEG_420    0x0A
#define ST7735R_CFG_GVCL_NEG_415    0x0B
#define ST7735R_CFG_GVCL_NEG_410    0x0C
#define ST7735R_CFG_GVCL_NEG_405    0x0D
#define ST7735R_CFG_GVCL_NEG_400    0x0E
#define ST7735R_CFG_GVCL_NEG_395    0x0F
#define ST7735R_CFG_GVCL_NEG_390    0x10
#define ST7735R_CFG_GVCL_NEG_385    0x11
#define ST7735R_CFG_GVCL_NEG_380    0x12
#define ST7735R_CFG_GVCL_NEG_375    0x13
#define ST7735R_CFG_GVCL_NEG_370    0x14
#define ST7735R_CFG_GVCL_NEG_365    0x15
#define ST7735R_CFG_GVCL_NEG_360    0x16
#define ST7735R_CFG_GVCL_NEG_355    0x17
#define ST7735R_CFG_GVCL_NEG_350    0x18
#define ST7735R_CFG_GVCL_NEG_345    0x19
#define ST7735R_CFG_GVCL_NEG_340    0x1A
#define ST7735R_CFG_GVCL_NEG_335    0x1B
#define ST7735R_CFG_GVCL_NEG_330    0x1C
#define ST7735R_CFG_GVCL_NEG_325    0x1D
#define ST7735R_CFG_GVCL_NEG_320    0x1E
#define ST7735R_CFG_GVCL_NEG_315    0x1F

#define ST7735R_CFG_PWR_MODE_2X     0x00
#define ST7735R_CFG_PWR_MODE_3X     0x01
#define ST7735R_CFG_PWR_MODE_AUTO   0x02

#define ST7735R_CFG_V25_210         0x00
#define ST7735R_CFG_V25_220         0x01
#define ST7735R_CFG_V25_230         0x02
#define ST7735R_CFG_V25_240         0x03

#define ST7735R_CFG_VGL_NEG_075     0x00
#define ST7735R_CFG_VGL_NEG_100     0x01
#define ST7735R_CFG_VGL_NEG_125     0x02
#define ST7735R_CFG_VGL_NEG_130     0x03

#define ST7735R_CFG_VGH_2_X_AVDD_PLS_VHG25    0x00
#define ST7735R_CFG_VGH_3_X_AVDD              0x01
#define ST7735R_CFG_VGH_3_X_AVDD_PLS_VHG25    0x02

#define ST7735R_CFG_OPAMP_I_NONE              0x00
#define ST7735R_CFG_OPAMP_I_SMALL             0x01
#define ST7735R_CFG_OPAMP_I_MED_LOW           0x02
#define ST7735R_CFG_OPAMP_I_MED               0x03
#define ST7735R_CFG_OPAMP_I_MED_HIGH          0x04
#define ST7735R_CFG_OPAMP_I_LARGE             0x05

#define ST7735R_CFG_BCLK_DIV_10     0x00
#define ST7735R_CFG_BCLK_DIV_15     0x01
#define ST7735R_CFG_BCLK_DIV_20     0x02
#define ST7735R_CFG_BCLK_DIV_40     0x03

#define ST7735R_CFG_VCOM_NEG_0425   0x00
#define ST7735R_CFG_VCOM_NEG_0450   0x01
#define ST7735R_CFG_VCOM_NEG_0475   0x02
#define ST7735R_CFG_VCOM_NEG_0500   0x03
#define ST7735R_CFG_VCOM_NEG_0525   0x04
#define ST7735R_CFG_VCOM_NEG_0550   0x05
#define ST7735R_CFG_VCOM_NEG_0575   0x06
#define ST7735R_CFG_VCOM_NEG_0600   0x07
#define ST7735R_CFG_VCOM_NEG_0625   0x08
#define ST7735R_CFG_VCOM_NEG_0650   0x09
#define ST7735R_CFG_VCOM_NEG_0675   0x0A
#define ST7735R_CFG_VCOM_NEG_0700   0x0B
#define ST7735R_CFG_VCOM_NEG_0725   0x0C
#define ST7735R_CFG_VCOM_NEG_0750   0x0D
#define ST7735R_CFG_VCOM_NEG_0775   0x0E
#define ST7735R_CFG_VCOM_NEG_0800   0x0F
#define ST7735R_CFG_VCOM_NEG_0825   0x10
#define ST7735R_CFG_VCOM_NEG_0850   0x11
#define ST7735R_CFG_VCOM_NEG_0875   0x12
#define ST7735R_CFG_VCOM_NEG_0900   0x13
#define ST7735R_CFG_VCOM_NEG_0925   0x14
#define ST7735R_CFG_VCOM_NEG_0950   0x15
#define ST7735R_CFG_VCOM_NEG_0975   0x16
#define ST7735R_CFG_VCOM_NEG_1000   0x17
#define ST7735R_CFG_VCOM_NEG_1025   0x18
#define ST7735R_CFG_VCOM_NEG_1050   0x19
#define ST7735R_CFG_VCOM_NEG_1075   0x1A
#define ST7735R_CFG_VCOM_NEG_1100   0x1B
#define ST7735R_CFG_VCOM_NEG_1125   0x1C
#define ST7735R_CFG_VCOM_NEG_1150   0x1D
#define ST7735R_CFG_VCOM_NEG_1175   0x1E
#define ST7735R_CFG_VCOM_NEG_1200   0x1F
#define ST7735R_CFG_VCOM_NEG_1225   0x20
#define ST7735R_CFG_VCOM_NEG_1250   0x21
#define ST7735R_CFG_VCOM_NEG_1275   0x22
#define ST7735R_CFG_VCOM_NEG_1300   0x23
#define ST7735R_CFG_VCOM_NEG_1325   0x24
#define ST7735R_CFG_VCOM_NEG_1350   0x25
#define ST7735R_CFG_VCOM_NEG_1375   0x26
#define ST7735R_CFG_VCOM_NEG_1400   0x27
#define ST7735R_CFG_VCOM_NEG_1425   0x28
#define ST7735R_CFG_VCOM_NEG_1450   0x29
#define ST7735R_CFG_VCOM_NEG_1475   0x2A
#define ST7735R_CFG_VCOM_NEG_1500   0x2B
#define ST7735R_CFG_VCOM_NEG_1525   0x2C
#define ST7735R_CFG_VCOM_NEG_1550   0x2D
#define ST7735R_CFG_VCOM_NEG_1575   0x2E
#define ST7735R_CFG_VCOM_NEG_1600   0x2F
#define ST7735R_CFG_VCOM_NEG_1625   0x30
#define ST7735R_CFG_VCOM_NEG_1650   0x31
#define ST7735R_CFG_VCOM_NEG_1675   0x32
#define ST7735R_CFG_VCOM_NEG_1700   0x33
#define ST7735R_CFG_VCOM_NEG_1725   0x34
#define ST7735R_CFG_VCOM_NEG_1750   0x35
#define ST7735R_CFG_VCOM_NEG_1775   0x36
#define ST7735R_CFG_VCOM_NEG_1800   0x37
#define ST7735R_CFG_VCOM_NEG_1825   0x38
#define ST7735R_CFG_VCOM_NEG_1850   0x39
#define ST7735R_CFG_VCOM_NEG_1875   0x3A
#define ST7735R_CFG_VCOM_NEG_1900   0x3B
#define ST7735R_CFG_VCOM_NEG_1925   0x3C
#define ST7735R_CFG_VCOM_NEG_1950   0x3D
#define ST7735R_CFG_VCOM_NEG_1975   0x3E
#define ST7735R_CFG_VCOM_NEG_2000   0x3F


/**
 * @brief Initialized and used internally
 *
 * Supplies data/command line instructions in
 * SPI transactions.
 */
typedef struct {
    gpio_num_t gpio_dc;
    uint8_t setting;
} st7735r_dc_setting_t;


/**
 * @brief Device descriptor
 *
 * GPIO pins and SPI host must be configured by
 * caller. All other fields are configured by
 * st7735r_init(...).
 */
typedef struct {
    /** @brief SPI device handle */
    spi_device_handle_t spi_device;
    /** @brief SPI Host (HSPI or VSPI) */
    spi_host_device_t host;
    /** @brief SPI Chip Select GPIO pin */
    gpio_num_t gpio_cs;
    /** @brief Data/Command Select GPIO pin */
    gpio_num_t gpio_dc;
    /** @brief Internal use only */
    st7735r_dc_setting_t dc_cmd;
    /** @brief Internal use only */
    st7735r_dc_setting_t dc_data;
    /** @brief Optional Reset GPIO pin */
    gpio_num_t gpio_rst;
    /** @brief Optional Backlight GPIO pin */
    gpio_num_t gpio_bckl;
} st7735r_device_t;

/**
 * @brief Placeholder for future enhancement
 */
typedef struct {
} st7735r_spi_params_t;


/**
 * @brief Convenient way to reference the device
 */
typedef st7735r_device_t* st7735r_device_handle_t;


/**
 * @brief Initialize an SPI command transaction
 *
 * Clears the transaction and configures it to
 * transmit the provided command, but does not send
 * the transaction.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7735r device handle
 * @param transaction the SPI transaction to configure
 * @param cmd the ST7735r command to be transmitted
 */
void st7735r_cmd_init(
    st7735r_device_handle_t device,
    spi_transaction_t *transaction,
    uint8_t cmd
);


/**
 * @brief Initialize an SPI command transaction
 *
 * Clears the transaction and configures it to
 * transmit the provided command, but does not send
 * the transaction.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * NOTE 1: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 2: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 3: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7735r device handle
 * @param transaction the SPI transaction to configure
 * @param data the data, in RAM, to be transmitted
 * @param len_bytes the byte length of the data
 */
void st7735r_data_init(
    st7735r_device_handle_t device,
    spi_transaction_t *transaction,
    const uint8_t * data,
    size_t len_bytes
);


/**
 * @brief Asynchronously execute SPI transactions
 *
 * Executes transactions using interrupts rather than
 * polling. Returns immediately. Use `st7735r_await(...)`
 * to block until the transactions are complete.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7735r device handle
 * @param transactions the SPI transactions to execute
 * @param num_transactions the count of transactions to
 *        execute
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_enqueue(
    st7735r_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions
);


/**
 * @brief Block until SPI transactions are complete
 *
 * Use this method after calling `st7735_enqueue(...)`
 * to block until all of the enqueued transactions are
 * complete.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * NOTE 1: Because ticks_to_wait applies for each
 * transaction this method serially waits upon, the
 * total time before return could in the worse case
 * be num_transactions * ticks_to_wait.
 *
 * @param device the ST7735r device handle
 * @param num_transactions the count of transactions to
 *        wait for
 * @param ticks_to_wait how long to wait for any one
 *        transaction to complete
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_await(
    st7735r_device_handle_t device,
    size_t num_transactions,
    TickType_t ticks_to_wait
);


/**
 * @brief Synchronously execute SPI transactions
 *
 * This method conviently wraps serial calls to
 * st7735r_enqueue(...) and st7735r_await(...) with
 * a 10-second ticks_to_wait value.
 *
 * This method will block the task (but not the CPU)
 * until either all of the transactions complete, or
 * until the timeout is reached while waiting for any
 * one of the transactions to complete.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7735r device handle
 * @param transactions the SPI transactions to execute
 * @param num_transactions the count of transactions to
 *        wait for
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_exec(
    st7735r_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions
);


/**
 * @brief Send a command to the LCD.
 *
 * Uses spi_device_polling_transmit, which waits
 * until the transfer is complete and blocks
 * both the thread and the CPU but is more
 * efficient than interrupt-based transfers for
 * small transmissions.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * @param device the ST7735r device handle
 * @param cmd the ST7735r command to send
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_send_cmd(
    st7735r_device_handle_t device,
    const uint8_t cmd
);


/**
 * @brief Send data to the LCD.
 *
 * Uses spi_device_polling_transmit, which waits
 * until the transfer is complete and blocks
 * both the thread and the CPU but is more
 * efficient than interrupt-based transfers for
 * small transmissions.
 *
 * This is a low-level function that most callers will
 * not need to use.
 *
 * NOTE 1: This function is inappropriate for transfers
 * of large payloads, such as pixel data. Use the
 * interrupt-based methods instead (_enqueue, _await,
 * _exec).
 *
 * NOTE 2: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 3: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 4: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7735r device handle
 * @param cmd the ST7735r command to send
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_send_data(
    st7735r_device_handle_t device,
    const uint8_t *data,
    size_t len_bytes
);


/**
 * @brief Add the device to the SPI bus
 *
 * Adds the device to the SPI bus and
 * completes initialization of the device
 * descriptor.
 *
 * This method will add the device to the SPI
 * bus with a 6-transaction queue, SPI mode 0,
 * and a 10MHz clock speed. Future enhancements
 * may make this hard-coded parameters customizable.
 *
 * NOTE 1: The SPI Host must be set in the
 * device descriptor before invoking this
 * method. HSPI or VSPI are supported.
 * SPI1 is not recommended.
 *
 * Note 2: GPIO pins must be set in the
 * device descriptor before invoking this
 * method. The rst and bckl pins are
 * optional and their absence is gracefully
 * handled in all of this library's
 * functions.
 *
 * Note 3: The SPI bus itself must already
 * be initialized before calling this
 * method. This library does not configure
 * the bus itself. This library was tested
 * using the following bus config:
 *
 * SPI host: HSPI
 * mosi_io_num: GPIO 12
 * miso_io_num: GPIO 13 (not used by this library)
 * sclk_io_num: GPIO 14
 * quadwp_io_num: -1
 * quadhd_io_num: -1
 * max_transfer_sz: 4096 bytes
 * DMA Channel: 2
 *
 * @param device the ST7735r device handle, preconfigured
 *        per notes above.
 * @param spi_param placeholder for future enhancement, NULL ok
 */
void st7735r_init(
    st7735r_device_handle_t device,
    st7735r_spi_params_t *spi_params
);


/**
 * @brief Performs a hardware reset
 *
 * Sends hardware reset signal over the RST GPIO pin (if connected).
 * The entire sequence involves three separate delays totaling 500ms.
 * This method does not return until that sequence is complete.
 *
 * Returns immediately without performing any work if the RST GPIO
 * pin is GPIO_NUM_NC.
 *
 * @param device the ST7735r device handle
 */
void st7735r_hwreset(st7735r_device_handle_t device);


/**
 * @brief Performs a software reset
 *
 * Sends SWRESET (01h) in a single SPI transaction, followed by
 * a conservative 150ms delay to ensure stabilization prior to
 * subsequent commands. Does not return until this sequence is
 * complete.
 *
 * The datasheet calls for a 120ms delay.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_swreset(st7735r_device_handle_t device);


/**
 * @brief Enters sleep mode
 *
 * Sends a SLPIN (10h) command in a single SPI transaction,
 * followed by a conservative 150 ms delay to ensure
 * stabilization prior to subsequent commands. Does not
 * return until this sequence is complete.
 *
 * The datasheet calls for a 120ms delay.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_slpin(st7735r_device_handle_t device);


/**
 * @brief Exits sleep mode
 *
 * Sends a SLPOUT (11h) command in a single SPI transaction,
 * followed by a conservative 150 ms delay to ensure
 * stabilization prior to subsequent commands. Does not
 * return until this sequence is complete.
 *
 * The datasheet calls for a 120ms delay.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_slpout(st7735r_device_handle_t device);


/**
 * @brief Sets the frame rate in normal mode
 *
 * Sends FRMCTR1 (B1h) and config data two separate
 * SPI transactions, returning after both are
 * complete or an error is encountered.
 *
 * Frame rate = fosc / (rtna x 2 + 40) * (LINE + fp + bp)
 * fosc = 625kHz
 * LINE = number of displayed lines
 *
 * @param device the ST7735r device handle
 * @param rtna the RTNA value (no pre-defined values)
 * @param fp the front porch value (no pre-defined values)
 * @param bp the back porch value (no pre-defined values)
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_frmctr1(
    st7735r_device_handle_t device,
    uint8_t rtna,
    uint8_t fp,
    uint8_t bp
);


/**
 * @brief Sets the frame rate in idle mode
 *
 * Sends FRMCTR2 (B2h) and config data two separate
 * SPI transactions, returning after both are
 * complete or an error is encountered.
 *
 * Frame rate = fosc / (rtna x 2 + 40) * (LINE + fp + bp)
 * fosc = 625kHz
 * LINE = number of displayed lines
 *
 * @param device the ST7735r device handle
 * @param rtna the RTNA value (no pre-defined values)
 * @param fp the front porch value (no pre-defined values)
 * @param bp the back porch value (no pre-defined values)
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_frmctr2(
    st7735r_device_handle_t device,
    uint8_t rtna,
    uint8_t fp,
    uint8_t bp
);

/**
 * @brief Sets the frame rate in partial mode
 *
 * Sends FRMCTR3 (B3h) and config data two separate
 * SPI transactions, returning after both are
 * complete or an error is encountered.
 *
 * Frame rate = fosc / (rtna x 2 + 40) * (LINE + fp + bp)
 * fosc = 625kHz
 * LINE = number of displayed lines
 *
 * @param device the ST7735r device handle
 * @param rtna the RTNA value (no pre-defined values)
 * @param fp the front porch value (no pre-defined values)
 * @param bp the back porch value (no pre-defined values)
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_frmctr3(
    st7735r_device_handle_t device,
    uint8_t rtna,
    uint8_t fp,
    uint8_t bp
);


/**
 * @brief Sets GVDD, AVDD, GVCL and power mode
 *
 * Sends a PWCTR1 (C0h) command. See the datasheet for
 * details.
 *
 * @param device the ST7735r device handle
 * @param gvdd One of the ST7735R_CFG_GVDD_* values
 * @param avdd One of the ST7735R_CFG_AVDD_* values
 * @param gvcl One of the ST7735R_CFG_GVCL_* values
 * @param mode One of the ST7735R_CFG_PWR_MODE_* values
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_pwctr1(
    st7735r_device_handle_t device,
    uint8_t gvdd,
    uint8_t avdd,
    uint8_t gvcl,
    uint8_t mode
);


/**
 * @brief Sets the VGH and VGL power supply level.
 *
 * Sends a PWCTR2 (C1h) command. See the datasheet for
 * details.
 *
 * @param device the ST7735r device handle
 * @param device the SPI handle representing the display
 * @param v25 One of the ST7735R_CFG_V25_* values
 * @param vgh One of the ST7735R_CFG_VGH_* values
 * @param vgl One of the ST7735R_CFG_VGL_* values
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_pwctr2(
    st7735r_device_handle_t device,
    uint8_t v25,
    uint8_t vgh,
    uint8_t vgl
);


/**
 * @brief Builds a composite DCA value
 *
 * This utility function does the bitshifting
 * for you, building a DCA value that can be
 * supplied directly to st7735r_pwctr3,
 * st7735r_pwctr4 and st7735r_pwctr5.
 *
 * @param dca5 ST7735R_CFG_BCLK_* value corresponding to DCA[9:8]
 * @param dca4 ST7735R_CFG_BCLK_* value corresponding to DCA[7:6]
 * @param dca3 ST7735R_CFG_BCLK_* value corresponding to DCA[5:4]
 * @param dca2 ST7735R_CFG_BCLK_* value corresponding to DCA[3:2]
 * @param dca1 ST7735R_CFG_BCLK_* value corresponding to DCA[1:0]
 * @return composite dca value
 */
uint16_t st7735r_dca(
    uint8_t dca5,
    uint8_t dca4,
    uint8_t dca3,
    uint8_t dca2,
    uint8_t dca1
);


/**
 * @brief Sets the op amp current and boost circuit in
 * normal mode
 *
 * Sends a PWCTR3 (C2h) command. See the datasheet for
 * details.
 *
 * @param device the ST7735r device handle
 * @param ap One of the ST7735R_CFG_OPAMP_I_* values
 * @param sap One of the ST7735R_CFG_OPAMP_I_* values
 * @param dca boost frequency config constructed with st7735r_dca
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_pwctr3(
    st7735r_device_handle_t device,
    uint8_t ap,
    uint8_t sap,
    uint16_t dca
);


/**
 * @brief Sets the op amp current and boost circuit in
 * idle mode
 *
 * Sends a PWCTR4 (C3h) command. See the datasheet for
 * details.
 *
 * @param device the ST7735r device handle
 * @param ap One of the ST7735R_CFG_OPAMP_I_* values
 * @param sap One of the ST7735R_CFG_OPAMP_I_* values
 * @param dca boost frequency config constructed with st7735r_dca
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_pwctr4(
    st7735r_device_handle_t device,
    uint8_t ap,
    uint8_t sap,
    uint16_t dca
);


/**
 * @brief Sets the op amp current and boost circuit in
 * partial mode
 *
 * Sends a PWCTR5 (C4h) command. See the datasheet for
 * details.
 *
 * @param device the ST7735r device handle
 * @param ap One of the ST7735R_CFG_OPAMP_I_* values
 * @param sap One of the ST7735R_CFG_OPAMP_I_* values
 * @param dca boost frequency config constructed with st7735r_dca
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_pwctr5(
    st7735r_device_handle_t device,
    uint8_t ap,
    uint8_t sap,
    uint16_t dca
);


/**
 * @brief Sets the vcom voltage
 *
 * Sends a VMCTR1 (C5h) command. See the datasheet for
 * details.
 *
 * @param device the ST7735r device handle
 * @param vcoms one of the ST7735R_CFG_VCOM_* values
 * @return ESP_OK or an error code
 */

esp_err_t st7735r_vmctr1(
    st7735r_device_handle_t device,
    int8_t vcoms
);


/**
 * @brief Enables color inversion
 *
 * Sends the INVON (20h) command in a single SPI transaction.
 *
 * Each pixel's color value will be inverted bitwise.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_invon(st7735r_device_handle_t device);


/**
 * @brief Disables color inversion
 *
 * Sends the INVOFF (21h) command in a single SPI transaction.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_invoff(st7735r_device_handle_t device);


/**
 * @brief Sets display inversion configuration
 *
 * Sends an INVCTR (B4h) command in one SPI transaction
 * and one configuration byte in a second SPI transaction.
 *
 * For config, start with 0x00 and bitwise-or (|) with:
 * ST7735R_INV_LINE_NORM for line instead of dot inversion
 * in normal mode
 * ST7735R_INV_LINE_IDLE for line instead of dot inversion
 * in idle mode
 * ST7735R_INV_LINE_PART for line instead of dot inverstion
 * in partial mode
 *
 * Config default after power-on, hw reset and sw reset is
 * ST7735R_INV_LINE_IDLE | ST7735R_INV_LINE_PART
 *
 * @param device the ST7735r device handle
 * @param config see description above
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_invctr(
    st7735r_device_handle_t device,
    uint8_t config
);


/**
 * @brief Sets memory data access configuration
 *
 * Sends a MADCTL (36h) command in one SPI transaction and
 * one configuration byte in a second SPI transaction.
 *
 * The config value 0x00 represents:
 * - No mirroring of x or y axis
 * - No exchange of x and y axis
 * - Data transmitted from IC to LCD top-to-bottom and
 *   left-to-right
 * - LCD panel pixels are arranged in R-G-B order
 *
 * Bitwise or (|) 0x00 with the following to set up the
 * configuration:
 * - ST7735R_CFG_MIRROR_X to mirror the x axis ("MX")
 * - ST7735R_CFG_MIRROR_Y to mirror the y axis ("MY")
 * - ST7735R_CFG_EXCHANGE_XY to exchange the x and y axis
 *   ("MV")
 * - ST7735R_CFG_REFRESH_RTL to transmit data from IC to LCD
 *   right-to-left ("MH")
 * - ST7735R_CFG_REFRESH_BTT to transmit data from IC to LCD
 *   bottom-to-top ("ML")
 * - ST7735R_CFG_BGR for LCD panel pixels arranged in B-G-R
 *   order ("RGB")
 *
 * Note that names above quoted in parenthesis are the
 * associated bitfield names as described in the datasheet.
 *
 * @param device the ST7735r device handle
 * @param config see description above
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_madctl(
    st7735r_device_handle_t device,
    uint8_t config
);

/**
 * @brief Sets the color mode configuration
 *
 * Sends a COLMOD (3Ah) command in one SPI transaction and
 * one configuration byte in a second SPI transaction.
 *
 * @param device the ST7735r device handle
 * @param config ST7735R_CFG_12_BIT_COLOR,
 *        ST7735R_CFG_16_BIT_COLOR or ST7735R_CFG_18_BIT_COLOR
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_colmod(
    st7735r_device_handle_t device,
    uint8_t config
);

/**
 * @brief Sets the column (x-axis) address range
 *
 * Sends a CASET (2Ah) command in one SPI transaction
 * and four configuration bytes in a second SPI
 * transaction.
 *
 * @param device the ST7735r device handle
 * @param x_min the lowest-valued x-axis address
 * @param x_max the highest-valued x-axis address,
 *        where x_max >= x_min
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_caset(
    st7735r_device_handle_t device,
    uint8_t x_min,
    uint8_t x_max
);

/**
 * @brief Sets the row (y-axis) address range
 *
 * Sends a RASET (3Bh) command in one SPI transaction
 * and four configuration bytes in a second SPI
 * transaction.
 *
 * @param device the ST7735r device handle
 * @param y_min the lowest-valued y-axis address
 * @param y_max the highest-valued y-axis address,
 *        where y_max >= y_min
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_raset(
    st7735r_device_handle_t device,
    uint8_t y_min,
    uint8_t y_max
);

/**
 * @brief Writes pixel data to device RAM
 *
 * Sends a RAMWR (2Ch) command in one SPI transaction
 * and the buffer in a second SPI transaction.
 *
 * The location where the pixels will be written
 * must first have been set with st7735r_caset(...)
 * and st7735r_raset(...).
 *
 * For large amounts of data, the data transaction
 * will be sent using interrupts, allowing the CPU
 * to do other work while the task is blocked
 * waiting for a completion signal. Polling is used
 * for small amounts of data.
 *
 * The data in the buffer needs to be arranged such
 * that the first two bytes read represent the color
 * of the lower-left most pixel in the space, with
 * subsequent reads moving right in the space before
 * wrapping to the left-most position of the next
 * line.
 *
 * NOTE 1: Beware that the ESP32 stores uint16_t
 * values most significant byte first. Thus, a
 * RGB/565 uint16_t value is arranged in memory
 * (and sent byte-by-byte to the TFT controller) as
 * G[2:0]BRG[5:3]. Obviously, this will not produce
 * the desired effect. Pre-inverting the bytes
 * when building color values is one of several
 * possible solutions to this problem.
 *
 * NOTE 2: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 3: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 4: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7735r device handle
 * @param buffer the pixel data to write
 * @param num_pixels the number of pixels to write
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_ramwr(
    st7735r_device_handle_t device,
    uint16_t * buffer,
    size_t num_pixels
);


/**
 * @brief Sets the positive gamma config
 *
 * Sends a GMCTRP1 command (E0h) and 16
 * bytes of data from the polarity value.
 *
 * 2) Positive polarity value (16 bytes)
 * 3) ST7735R_CMD_GMCTRN1 command (E1h)
 * 4) Negative polarity value (16 bytes)
 *
 * NOTE 1: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 2: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7735r device handle
 * @param polarity 16 bytes representing positive polarity
 *        configuration; see datasheet for details.
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_gmctrp1(
    st7735r_device_handle_t device,
    uint8_t *polarity
);


/**
 * @brief Sets the gamma configuration
 *
 * Sends a GMCTRN1 command (E1h) and 16
 * bytes of data from the polarity value.
 *
 * NOTE 1: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 2: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7735r device handle
 * @param polarity 16 bytes representing negative polarity
 *        configuration; see datasheet for details.
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_gmctrn1(
    st7735r_device_handle_t device,
    uint8_t *polarity
);


/**
 * @brief Turns off partial mode.
 *
 * Sends a NORON (13h) command in a single SPI transaction,
 * followed by a brief 10ms delay for stablization.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_noron(st7735r_device_handle_t device);


/**
 * @brief Turns on partial mode.
 *
 * Sends a PTLON (12h) command in a single SPI transaction,
 * followed by a brief 10ms delay for stablization.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_ptlon(st7735r_device_handle_t device);


/**
 * @brief Sets the backlight illumination level full on or full off
 *
 * Returns without taking any action if the backlight
 * pin is not connected (GPIO_NUM_NC).
 *
 * @param device the ST7735r device handle
 * @param level st7735r_CFG_BCKL_ON, st7735r_CFG_BCKL_OFF
 */
void st7735r_backlight(
    st7735r_device_handle_t device,
    uint8_t level
);


/**
 * @brief Turns the display on
 *
 * Sends a DISPON (29h) command in a single SPI transaction,
 * followed by a 100ms delay for stabilization.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_dispon(st7735r_device_handle_t device);


/**
 * @brief Turns the display off
 *
 * Sends a DSPOFF (28h) command in a single SPI transaction,
 * followed by a 100ms delay for stabilization.
 *
 * @param device the ST7735r device handle
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_dispoff(st7735r_device_handle_t device);


/**
 * @brief Writes pixel data to device RAM
 *
 * This higher-level function sends the command and
 * data sequence necessary to write pixel data to a
 * defines space within the frame memory buffer.
 *
 * The data in the buffer needs to be arranged such
 * that the first two bytes read represent the color
 * of the lower-left most pixel in the space, with
 * subsequent reads moving right in the space before
 * wrapping to the left-most position of the next
 * line.
 *
 * NOTE 1: Beware that the ESP32 stores uint16_t
 * values most significant byte first. Thus, a
 * RGB/565 uint16_t value is arranged in memory
 * (and sent byte-by-byte to the TFT controller) as
 * G[2:0]BRG[5:3]. Obviously, this will not produce
 * the desired effect. Pre-inverting the bytes
 * when building color values is one of several
 * possible solutions to this problem.
 *
 * NOTE 2: If using DMA, the provided data must be in
 * RAM and not ROM since DMA cannot access ROM.
 *
 * NOTE 3: The number of bytes that can be sent is limited
 * by the `max_transfer_sz` value supplied via `bus_config_t`
 * when initializing the SPI bus. The default value of this
 * parameter is 4096 bytes.
 *
 * NOTE 4: This function uses the transaction's `tx_buffer`
 * rather than using the 64-byte `tx_data`. For this reason,
 * the provided data must remain accessible in memory until
 * the transaction is complete.
 *
 * @param device the ST7735r device handle
 * @param buffer the pixel data to write
 * @param num_pixes the number of pixels to write
 * @return ESP_OK or an error code
 */
esp_err_t st7735r_paint(
    st7735r_device_handle_t device,
    uint16_t * buffer,
    uint8_t x_min,
    uint8_t x_max,
    uint8_t y_min,
    uint8_t y_max
);


/**
 * @brief Builds RGB/565 color values
 *
 * This method create 2-byte (16-bit) color codes that
 * correct for the little endianness of the ESP's
 * memory layout.
 *
 * Since a buffer of pixel colors is handled upon SPI
 * transmission as an array of bytes (and not an array of
 * 2-byte integers), and because the ESP32 stores bytes
 * in memory in little endian order, the bytes of each
 * 16-bit color code are inverted when sent over the wire.
 *
 * The ESP32 knows that when reading a 2-byte integer
 * it needs to treat the first byte as least-signficant,
 * but the ESP32 does not know when reading an array of
 * bytes given by uint8_t* (or void *) that it needs to do
 * anything special with the byte order of any particular
 * grouping of bytes.
 *
 * This method corrects for that inversion by pre-inverting
 * the bytes so that when inverted in stored, they are
 * actually laid out in memory in the expect bit order,
 * and when read bit-by-bit or byte-by-byte, such as when
 * transmitting via SPI, they come out of memory in the
 * expected RGB/565 sequence.
 *
 * For example, B[4:0]G[5:0]R[4:0] is the expected sequence
 * of bits in an RGB/565 value. If the equivalent integer
 * value is assigned to an ESP32's uint16_t, the bitwise
 * layout in the ESP32's RAM (or ROM) will actually be
 * G[2:0]R[4:0]B[4:0]G[5:3], where G[2:0]R[4:0] is the least
 * signficant byte and B[4:0]G[5:3] is the most significant
 * byte. Further, G[2:0]R[4:0]B[4:0]G[5:3] is the order
 * in which the bytes will be transmitted to the ST7735R,
 * and the ST7735R treats this as a big-endian value
 * with no need for byte order inversion.
 *
 * NOTE: Values of 0x00 to 0xFF are legal for all three
 * color values, but the algorithm in use here will
 * truncate enough of the least significant bits of
 * each color code to fit the value into the available
 * bit space (5 bits for red and blue, 6 bits for
 * green). Thus values of 0x07 or less for red or blue
 * and 0x03 or less for green are equivalent to 0x00.
 *
 * @param red red pixel brightness
 * @param green green pixel brightness
 * @param blue blue pixel brightness
 * @return 16-bit RGB/565 color with the byte order
 *         inverted such that the value's bits are
 *         correcly order in ESP32 memory to be read
 *         out bit by bit or byte by byte (but not
 *         uint16_t by uint16_t!)
 */
uint16_t st7735r_rgb565(
    uint8_t red,
    uint8_t green,
    uint8_t blue
);


#endif // ST7735R_H
