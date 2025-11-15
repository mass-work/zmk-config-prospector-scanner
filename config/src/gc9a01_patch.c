// config/src/gc9a01_patch.c

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

static int lcd_send_data(const uint8_t *buf, size_t len) {
    int ret = gpio_pin_set_dt(&lcd_dc, 1);
    if (ret) return ret;
    return lcd_write_bytes(buf, len);
}

static int gc9a01_fill_color(uint16_t color) {
    const uint16_t WIDTH  = 240;
    const uint16_t HEIGHT = 240;
    int ret = 0;

    ret |= lcd_send_cmd(0x2A);
    uint8_t col_param[4] = {
        0x00, 0x00,
        (uint8_t)((WIDTH - 1) >> 8),
        (uint8_t)((WIDTH - 1) & 0xFF),
    };
    ret |= lcd_send_data(col_param, sizeof(col_param));

    ret |= lcd_send_cmd(0x2B);
    uint8_t row_param[4] = {
        0x00, 0x00,
        (uint8_t)((HEIGHT - 1) >> 8),
        (uint8_t)((HEIGHT - 1) & 0xFF),
    };
    ret |= lcd_send_data(row_param, sizeof(row_param));

    ret |= lcd_send_cmd(0x2C);
    if (ret) return ret;

    static uint16_t line[240];
    uint8_t *line_bytes = (uint8_t *)line;

    for (int x = 0; x < WIDTH; x++) {
        line[x] = color;
    }

    for (int y = 0; y < HEIGHT; y++) {
        ret = lcd_send_data(line_bytes, WIDTH * 2);
        if (ret) return ret;
    }

    return 0;
}

static int gc9a01_patch_init(const struct device *dev) {
    ARG_UNUSED(dev);

    printk("GC9A01 patch: init start (line-buffer fill)\n");

    if (!device_is_ready(lcd_spi.bus)) {
        printk("GC9A01 patch: SPI bus not ready\n");
        return 0;
    }
    if (!device_is_ready(lcd_dc.port)) {
        printk("GC9A01 patch: DC GPIO not ready\n");
        return 0;
    }

    gpio_pin_configure_dt(&lcd_dc, GPIO_OUTPUT_ACTIVE);

    k_msleep(500);

    int ret = gc9a01_fill_color(0xF800);
    if (ret) {
        printk("GC9A01 patch: fill error %d\n", ret);
    } else {
        printk("GC9A01 patch: fill OK\n");
    }

    return 0;
}

SYS_INIT(gc9a01_patch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
