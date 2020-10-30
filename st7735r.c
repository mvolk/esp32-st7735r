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
 * The above copyright notice this permission notice, and the disclaimer below
 * shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "st7735r.h"
#include <string.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>


static const char * ST7735R_TAG = "st7735r";


void st7735r_cmd_init(
    st7735r_device_handle_t device,
    spi_transaction_t *transaction,
    uint8_t cmd)
{
    memset(transaction, 0, sizeof(spi_transaction_t));
    transaction->user = &(device->dc_cmd);
    transaction->flags = SPI_TRANS_USE_TXDATA;
    transaction->tx_data[0] = cmd;
    transaction->length = 8;
}


void st7735r_data_init(
    st7735r_device_handle_t device,
    spi_transaction_t *transaction,
    const uint8_t *data,
    size_t len_bytes)
{
    memset(transaction, 0, sizeof(spi_transaction_t));
    transaction->user = &(device->dc_data);
    transaction->tx_buffer = data;
    transaction->length = len_bytes * 8;
}


esp_err_t st7735r_enqueue(
    st7735r_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions)
{
    uint8_t i;
    esp_err_t ret = ESP_OK;
    for (i = 0; i < num_transactions; i++) {
        ret = spi_device_queue_trans(
            device->spi_device,
            &transactions[i],
            portMAX_DELAY
        );
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7735R_TAG,
                "str7735r_enqueue(...) failed in spi_device_queue_trans(...): %s",
                esp_err_to_name(ret)
            );
            break;
        }
    }
    return ret;
}


esp_err_t st7735r_await(
    st7735r_device_handle_t device,
    size_t num_transactions,
    TickType_t ticks_to_wait)
{
    spi_transaction_t *transaction;
    esp_err_t ret = ESP_OK;
    esp_err_t inner_ret;
    for (size_t i = 0; i < num_transactions; i++) {
        inner_ret = spi_device_get_trans_result(
            device->spi_device, &transaction,
            ticks_to_wait
        );
        if (inner_ret != ESP_OK) {
            ESP_LOGE(
                ST7735R_TAG,
                "st7735r_await(...) failed in spi_device_get_trans_result(...): %s",
                esp_err_to_name(inner_ret)
            );
            if (ret == ESP_OK) ret = inner_ret;
        }
    }
    return ret;
}


esp_err_t st7735r_exec(
    st7735r_device_handle_t device,
    spi_transaction_t *transactions,
    size_t num_transactions)
{
    esp_err_t ret;
    ret = st7735r_enqueue(device, transactions, num_transactions);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7735R_TAG,
            "st7735r_exec(...) failed in st7735r_enqueue(...): %s",
            esp_err_to_name(ret)
        );
    } else {
        ret = st7735r_await(device, num_transactions, 10000.0 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7735R_TAG,
                "st7735r_exec(...) failed in st7735r_await(...): %s",
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


esp_err_t st7735r_send_cmd(
    st7735r_device_handle_t device,
    const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    st7735r_cmd_init(device, &t, cmd);
    ret = spi_device_polling_transmit(device->spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7735R_TAG,
            "st7735r_send_cmd(...) failed in spi_device_polling_transmit(...): %s",
            esp_err_to_name(ret)
        );
    }
    return ret;
}


esp_err_t st7735r_send_data(
    st7735r_device_handle_t device,
    const uint8_t *data,
    size_t len_bytes)
{
    esp_err_t ret = ESP_OK;
    spi_transaction_t t;
    if (len_bytes > 0)  {
        st7735r_data_init(device, &t, data, len_bytes);
        ret = spi_device_polling_transmit(device->spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7735R_TAG,
                "st7735r_send_data(...) failed in spi_device_polling_transmit(...): %s",
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


void st7735r_spi_pre_transfer_callback_isr(spi_transaction_t *t)
{
    st7735r_dc_setting_t * dc = (st7735r_dc_setting_t *) t->user;
    gpio_set_level(dc->gpio_dc, dc->setting);
}


st7735r_device_handle_t st7735r_init(
    const st7735r_params_t *params)
{
    st7735r_device_handle_t device;
    device = (st7735r_device_handle_t) malloc(sizeof(st7735r_device_t));
    if (!device) {
        ESP_LOGE(
            ST7735R_TAG,
            "ST7735R failed to allocate memory for device descriptor"
        );
        esp_restart();
    }
    st7735r_init_static(params, device);
    return device;
}


void st7735r_init_static(
    const st7735r_params_t *params,
    st7735r_device_handle_t device)
{
    if (params->host != HSPI_HOST && params->host != VSPI_HOST) {
        ESP_LOGE(ST7735R_TAG, "ST7735R requires either HSPI or VSPI host");
        esp_restart();
    }
    if (params->gpio_dc == GPIO_NUM_NC) {
        ESP_LOGE(ST7735R_TAG, "ST7735R requires a connected D/C pin");
        esp_restart();
    }

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .spics_io_num = params->gpio_cs,
        .queue_size = 6,
        .pre_cb = st7735r_spi_pre_transfer_callback_isr,
    };
    esp_err_t ret = spi_bus_add_device(
        params->host,
        &dev_cfg,
        &(device->spi_device)
    );
    ESP_ERROR_CHECK(ret);

    // Initialize non-SPI GPIOs
    gpio_set_direction(params->gpio_dc, GPIO_MODE_OUTPUT);
    device->dc_cmd.gpio_dc = params->gpio_dc;
    device->dc_cmd.setting = 0;
    device->dc_data.gpio_dc = params->gpio_dc;
    device->dc_data.setting = 1;

    if (params->gpio_rst != GPIO_NUM_NC)
        gpio_set_direction(params->gpio_rst, GPIO_MODE_OUTPUT);
    device->gpio_rst = params->gpio_rst;

    if (params->gpio_bckl != GPIO_NUM_NC)
        gpio_set_direction(params->gpio_bckl, GPIO_MODE_OUTPUT);
    device->gpio_bckl = params->gpio_bckl;
}


void st7735r_hwreset(st7735r_device_handle_t device)
{
    if (device->gpio_rst != GPIO_NUM_NC) {
        gpio_set_level(device->gpio_rst, 1);
        vTaskDelay(100 / portTICK_RATE_MS);
        gpio_set_level(device->gpio_rst, 0);
        vTaskDelay(100 / portTICK_RATE_MS);
        gpio_set_level(device->gpio_rst, 1);
        vTaskDelay(300 / portTICK_RATE_MS);
    }
}


static esp_err_t st7735r_with_data(
    st7735r_device_handle_t device,
    const char *fn_name,
    uint8_t cmd,
    void *data,
    size_t len_bytes)
{
    esp_err_t ret = st7735r_send_cmd(device, cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7735R_TAG,
            "%s(...) failed on st7735r_send_cmd(...): %s",
            fn_name,
            esp_err_to_name(ret)
        );
    } else if (len_bytes > 0) {
        ret = st7735r_send_data(device, data, len_bytes);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7735R_TAG,
                "%s(...) failed on st7735r_send_data(...): %s",
                fn_name,
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


static esp_err_t st7735r_set(
    st7735r_device_handle_t device,
    const char *fn_name,
    uint8_t cmd)
{
    return st7735r_with_data(
        device,
        fn_name,
        cmd,
        (void *) 0,
        0
    );
}


esp_err_t st7735r_swreset(st7735r_device_handle_t device)
{
    esp_err_t ret = st7735r_set(
        device,
        "st7735r_swreset",
        ST7735R_CMD_SWRESET
    );
    vTaskDelay(150.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7735r_slpin(st7735r_device_handle_t device)
{
    esp_err_t ret = st7735r_set(
        device,
        "st7735r_slpin",
        ST7735R_CMD_SLPIN
    );
    vTaskDelay(150.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7735r_slpout(st7735r_device_handle_t device) {
    esp_err_t ret = st7735r_set(
        device,
        "st7735r_slpout",
        ST7735R_CMD_SLPOUT
    );
    vTaskDelay(150.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7735r_frmctr1(
    st7735r_device_handle_t device,
    uint8_t rtna,
    uint8_t fp,
    uint8_t bp)
{
    uint8_t params[3] = { rtna, fp, bp };
    return st7735r_with_data(
        device,
        "st7735r_frmctr1",
        ST7735R_CMD_FRMCTR1,
        params,
        3
    );
}


esp_err_t st7735r_frmctr2(
    st7735r_device_handle_t device,
    uint8_t rtna,
    uint8_t fp,
    uint8_t bp)
{
    uint8_t params[3] = { rtna, fp, bp };
    return st7735r_with_data(
        device,
        "st7735r_frmctr2",
        ST7735R_CMD_FRMCTR2,
        params,
        3
    );
}


esp_err_t st7735r_frmctr3(
    st7735r_device_handle_t device,
    uint8_t rtna,
    uint8_t fp,
    uint8_t bp)
{
    uint8_t params[6] = { rtna, fp, bp, rtna, fp, bp };
    return st7735r_with_data(
        device,
        "st7735r_frmctr3",
        ST7735R_CMD_FRMCTR3,
        params,
        6
    );
}


esp_err_t st7735r_pwctr1(
    st7735r_device_handle_t device,
    uint8_t gvdd,
    uint8_t avdd,
    uint8_t gvcl,
    uint8_t mode)
{
    uint8_t params[3] = {
        (avdd << 5) | (gvdd & 0x1F),
        (gvcl & 0x1F),
        ((mode << 6) | 0x04)
    };
    return st7735r_with_data(
        device,
        "st7735r_pwctr1",
        ST7735R_CMD_PWCTR1,
        params,
        3
    );
}


esp_err_t st7735r_pwctr2(
    st7735r_device_handle_t device,
    uint8_t v25,
    uint8_t vgh,
    uint8_t vgl)
{
    uint8_t param = (v25 << 6) | ((vgl & 0x03) << 2) | (vgh & 0x03);
    return st7735r_with_data(
        device,
        "st7735r_pwctr2",
        ST7735R_CMD_PWCTR2,
        &param,
        1
    );
}


uint16_t st7735r_dca(
    uint8_t dca5,
    uint8_t dca4,
    uint8_t dca3,
    uint8_t dca2,
    uint8_t dca1)
{
    uint16_t res =
        ((dca5 & 0x03) << 8) |
        ((dca4 & 0x03) << 6) |
        ((dca3 & 0x03) << 4) |
        ((dca2 & 0x03) << 2) |
        (dca1 & 0x03);
    return res;
}


static esp_err_t st7735r_pwctrx_helper(
    st7735r_device_handle_t device,
    const char *fn_name,
    uint8_t cmd,
    uint8_t ap,
    uint8_t sap,
    uint16_t dca)
{
    uint8_t params[2] = {
        ((dca & 0x300) >> 2) | ((sap & 0x07) << 3) | (ap & 0x07),
        (dca & 0xFF)
    };
    return st7735r_with_data(
        device,
        fn_name,
        cmd,
        params,
        2
    );
}


esp_err_t st7735r_pwctr3(
    st7735r_device_handle_t device,
    uint8_t ap,
    uint8_t sap,
    uint16_t dca)
{
    return st7735r_pwctrx_helper(
        device,
        "st7735r_pwctr3",
        ST7735R_CMD_PWCTR3,
        ap,
        sap,
        dca
    );
}


esp_err_t st7735r_pwctr4(
    st7735r_device_handle_t device,
    uint8_t ap,
    uint8_t sap,
    uint16_t dca)
{
    return st7735r_pwctrx_helper(
        device,
        "st7735r_pwctr4",
        ST7735R_CMD_PWCTR4,
        ap,
        sap,
        dca
    );
}


esp_err_t st7735r_pwctr5(
    st7735r_device_handle_t device,
    uint8_t ap,
    uint8_t sap,
    uint16_t dca)
{
    return st7735r_pwctrx_helper(
        device,
        "st7735r_pwctr5",
        ST7735R_CMD_PWCTR5,
        ap,
        sap,
        dca
    );
}


esp_err_t st7735r_vmctr1(
    st7735r_device_handle_t device,
    int8_t vcoms)
{
    uint8_t param = vcoms & 0x3F;
    return st7735r_with_data(
        device,
        "st7735r_vmctr1",
        ST7735R_CMD_VMCTR1,
        &param,
        1
    );
}


esp_err_t st7735r_invon(st7735r_device_handle_t device)
{
    return st7735r_set(
        device,
        "st7735r_invon",
        ST7735R_CMD_INVON
    );
}


esp_err_t st7735r_invoff(st7735r_device_handle_t device)
{
    return st7735r_set(
        device,
        "st7735r_invoff",
        ST7735R_CMD_INVOFF
    );
}


esp_err_t st7735r_invctr(
    st7735r_device_handle_t device,
    uint8_t config
) {
    uint8_t param = config & 0x07;
    return st7735r_with_data(
        device,
        "st7735r_invctr",
        ST7735R_CMD_INVCTR,
        &param,
        1
    );
}


esp_err_t st7735r_madctl(
    st7735r_device_handle_t device,
    uint8_t config)
{
    return st7735r_with_data(
        device,
        "st7735r_madctl",
        ST7735R_CMD_MADCTL,
        &config,
        1
    );
}


esp_err_t st7735r_colmod(
    st7735r_device_handle_t device,
    uint8_t config)
{
    return st7735r_with_data(
        device,
        "st7735r_colmod",
        ST7735R_CMD_COLMOD,
        &config,
        1
    );
}


esp_err_t st7735r_caset(
    st7735r_device_handle_t device,
    uint8_t x_min,
    uint8_t x_max
) {
    uint8_t buffer[4] = { 0x00, x_min, 0x00, x_max };
    return st7735r_with_data(
        device,
        "st7735r_caset",
        ST7735R_CMD_CASET,
        buffer,
        4
    );
}


esp_err_t st7735r_raset(
    st7735r_device_handle_t device,
    uint8_t y_min,
    uint8_t y_max)
{
    uint8_t buffer[4] = { 0x00, y_min, 0x00, y_max };
    return st7735r_with_data(
        device,
        "st7735r_raset",
        ST7735R_CMD_RASET,
        buffer,
        4
    );
}


esp_err_t st7735r_ramwr(
    st7735r_device_handle_t device,
    uint16_t * buffer,
    size_t num_pixels)
{
    if (num_pixels == 0) return ESP_OK;
    esp_err_t ret = st7735r_send_cmd(device, ST7735R_CMD_RAMWR);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7735R_TAG,
            "st7735r_ramwr(...) failed on st7735r_send_cmd(...): %s",
            esp_err_to_name(ret)
        );
    } else if (num_pixels < 32) {
        // For small amounts of data, use polling
        ret = st7735r_send_data(device, (void *) buffer, num_pixels * 2);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7735R_TAG,
                "st7735r_ramwr(...) failed on st7735r_send_data(...): %s",
                esp_err_to_name(ret)
            );
        }
    } else {
        // For large amounts of data, use interrupts
        spi_transaction_t trans;
        st7735r_data_init(device, &trans, (void *) buffer, num_pixels * 2);
        esp_err_t ret = st7735r_exec(device, &trans, 1);
        if (ret != ESP_OK) {
            ESP_LOGE(
                ST7735R_TAG,
                "st7735r_ramwr(...) failed in st7735r_exec: %s",
                esp_err_to_name(ret)
            );
        }
    }
    return ret;
}


esp_err_t st7735r_gmctrp1(
    st7735r_device_handle_t device,
    uint8_t *polarity)
{
    return st7735r_with_data(
        device,
        "st7735r_gmctrp1",
        ST7735R_CMD_GMCTRP1,
        polarity,
        16
    );
}


esp_err_t st7735r_gmctrn1(
    st7735r_device_handle_t device,
    uint8_t *polarity)
{
    return st7735r_with_data(
        device,
        "st7735r_gmctrn1",
        ST7735R_CMD_GMCTRN1,
        polarity,
        16
    );
}


esp_err_t st7735r_noron(st7735r_device_handle_t device)
{
    esp_err_t ret = st7735r_set(
        device,
        "st7735r_noron",
        ST7735R_CMD_NORON
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7735r_ptlon(st7735r_device_handle_t device)
{
    esp_err_t ret = st7735r_set(
        device,
        "st7735r_ptlon",
        ST7735R_CMD_PTLON
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


void st7735r_backlight(
    st7735r_device_handle_t device,
    uint8_t level)
{
    if (device->gpio_bckl == GPIO_NUM_NC)
        return;
    if (level == 0 || level == 1)
        gpio_set_level(device->gpio_bckl, level);
}


esp_err_t st7735r_dispon(st7735r_device_handle_t device)
{
    esp_err_t ret = st7735r_set(
        device,
        "st7735r_dispon",
        ST7735R_CMD_DISPON
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7735r_dispoff(st7735r_device_handle_t device)
{
    esp_err_t ret = st7735r_set(
        device,
        "st7735r_dspoff",
        ST7735R_CMD_DISPOFF
    );
    vTaskDelay(10.0 / portTICK_RATE_MS);
    return ret;
}


esp_err_t st7735r_paint(
    st7735r_device_handle_t device,
    uint16_t * buffer,
    uint8_t x_min,
    uint8_t x_max,
    uint8_t y_min,
    uint8_t y_max)
{
    spi_transaction_t trans[6];

    st7735r_cmd_init(device, &trans[0], ST7735R_CMD_CASET);
    uint8_t col_range[4] = {
        0x00, x_min,
        0x00, x_max
    };
    st7735r_data_init(device, &trans[1], col_range, 4);

    st7735r_cmd_init(device, &trans[2], ST7735R_CMD_RASET);
    uint8_t row_range[4] = {
        0x00, y_min,
        0x00, y_max
    };
    st7735r_data_init(device, &trans[3], row_range, 4);

    st7735r_cmd_init(device, &trans[4], ST7735R_CMD_RAMWR);
    size_t num_bytes =
        (((x_max - x_min) + 1) * ((y_max - y_min) + 1)) * 2;
    st7735r_data_init(device, &trans[5], (uint8_t *) buffer, num_bytes);

    esp_err_t ret = st7735r_exec(device, trans, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(
            ST7735R_TAG,
            "st7735r_paint(...) failed in st7735r_exec: %s",
            esp_err_to_name(ret)
        );
    }
    return ret;
}
