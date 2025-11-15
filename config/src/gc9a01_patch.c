#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>

#define ST7789_NODE DT_NODELABEL(st7789v)

#if !DT_NODE_HAS_STATUS(ST7789_NODE, okay)
#warning "st7789v node not found or disabled in devicetree"
#endif

static const struct spi_dt_spec lcd_spi = SPI_DT_SPEC_GET(
    ST7789_NODE,
    SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    0
);

static const struct gpio_dt_spec lcd_dc =
    GPIO_DT_SPEC_GET(ST7789_NODE, cmd_data_gpios);

static int lcd_write_bytes(const uint8_t *buf, size_t len) {
    struct spi_buf spi_buf = {
        .buf = (void *)buf,
        .len = len,
    };
    struct spi_buf_set tx = {
        .buffers = &spi_buf,
        .count   = 1,
    };
    return spi_write_dt(&lcd_spi, &tx);
}

static int lcd_send_cmd(uint8_t cmd) {
    int ret = gpio_pin_set_dt(&lcd_dc, 0);
    if (ret) return ret;
    return lcd_write_bytes(&cmd, 1);
}

static int lcd_send_data8(uint8_t data) {
    int ret = gpio_pin_set_dt(&lcd_dc, 1);
    if (ret) return ret;
    return lcd_write_bytes(&data, 1);
}

static int lcd_send_data16(uint16_t data) {
    uint8_t buf[2] = { data & 0xFF, data >> 8 };
    int ret = gpio_pin_set_dt(&lcd_dc, 1);
    if (ret) return ret;
    return lcd_write_bytes(buf, 2);
}

static int gc9a01_fill_color(uint16_t color) {
    int ret = 0;
    const uint16_t WIDTH  = 240;
    const uint16_t HEIGHT = 240;

    ret |= lcd_send_cmd(0x2A);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(WIDTH - 1);

    ret |= lcd_send_cmd(0x2B);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(HEIGHT - 1);

    ret |= lcd_send_cmd(0x2C);

    if (ret) return ret;

    for (uint32_t i = 0; i < (uint32_t)WIDTH * HEIGHT; i++) {
        ret = lcd_send_data16(color);
        if (ret) return ret;
    }
    return 0;
}

static int gc9a01_patch_init(const struct device *dev) {
    ARG_UNUSED(dev);

    if (!device_is_ready(lcd_spi.bus)) return 0;
    if (!device_is_ready(lcd_dc.port)) return 0;

    gpio_pin_configure_dt(&lcd_dc, GPIO_OUTPUT_ACTIVE);

    k_msleep(500);

    lcd_send_cmd(0x3A);
    lcd_send_data8(0x05);
    lcd_send_cmd(0x36);
    lcd_send_data8(0x08);

    gc9a01_fill_color(0xFFFF);

    return 0;
}

SYS_INIT(gc9a01_patch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
