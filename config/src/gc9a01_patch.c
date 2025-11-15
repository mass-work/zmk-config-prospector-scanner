#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
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
static const struct gpio_dt_spec lcd_rst =
    GPIO_DT_SPEC_GET(ST7789_NODE, reset_gpios);

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
    int ret;

    ret = gpio_pin_set_dt(&lcd_dc, 0);
    if (ret) {
        return ret;
    }

    return lcd_write_bytes(&cmd, 1);
}

static int lcd_send_data8(uint8_t data) {
    int ret;

    ret = gpio_pin_set_dt(&lcd_dc, 1);
    if (ret) {
        return ret;
    }

    return lcd_write_bytes(&data, 1);
}

static int lcd_reset_gc9(void) {
    int ret;

    ret = gpio_pin_configure_dt(&lcd_rst, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&lcd_dc, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        return ret;
    }

    gpio_pin_set_dt(&lcd_rst, 1);
    k_msleep(100);
    gpio_pin_set_dt(&lcd_rst, 0);
    k_msleep(100);
    gpio_pin_set_dt(&lcd_rst, 1);
    k_msleep(100);

    return 0;
}

static int gc9a01_init_sequence(void) {
    int ret = 0;

    ret |= lcd_send_cmd(0xEF);
    ret |= lcd_send_cmd(0xEB);
    ret |= lcd_send_data8(0x14);

    ret |= lcd_send_cmd(0xFE);
    ret |= lcd_send_cmd(0xEF);

    ret |= lcd_send_cmd(0xEB);
    ret |= lcd_send_data8(0x14);

    ret |= lcd_send_cmd(0x84);
    ret |= lcd_send_data8(0x40);

    ret |= lcd_send_cmd(0x85);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x86);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x87);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x88);
    ret |= lcd_send_data8(0x0A);

    ret |= lcd_send_cmd(0x89);
    ret |= lcd_send_data8(0x21);

    ret |= lcd_send_cmd(0x8A);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0x8B);
    ret |= lcd_send_data8(0x80);

    ret |= lcd_send_cmd(0x8C);
    ret |= lcd_send_data8(0x01);

    ret |= lcd_send_cmd(0x8D);
    ret |= lcd_send_data8(0x01);

    ret |= lcd_send_cmd(0x8E);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x8F);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0xB6);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x20);

    ret |= lcd_send_cmd(0x36);
    ret |= lcd_send_data8(0x08);

    ret |= lcd_send_cmd(0x3A);
    ret |= lcd_send_data8(0x05);

    ret |= lcd_send_cmd(0x90);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);

    ret |= lcd_send_cmd(0xBD);
    ret |= lcd_send_data8(0x06);

    ret |= lcd_send_cmd(0xBC);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0xFF);
    ret |= lcd_send_data8(0x60);
    ret |= lcd_send_data8(0x01);
    ret |= lcd_send_data8(0x04);

    ret |= lcd_send_cmd(0xC3);
    ret |= lcd_send_data8(0x13);
    ret |= lcd_send_cmd(0xC4);
    ret |= lcd_send_data8(0x13);

    ret |= lcd_send_cmd(0xC9);
    ret |= lcd_send_data8(0x22);

    ret |= lcd_send_cmd(0xBE);
    ret |= lcd_send_data8(0x11);

    ret |= lcd_send_cmd(0xE1);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x0E);

    ret |= lcd_send_cmd(0xDF);
    ret |= lcd_send_data8(0x21);
    ret |= lcd_send_data8(0x0C);
    ret |= lcd_send_data8(0x02);

    ret |= lcd_send_cmd(0xF0);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x09);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x26);
    ret |= lcd_send_data8(0x2A);

    ret |= lcd_send_cmd(0xF1);
    ret |= lcd_send_data8(0x43);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x72);
    ret |= lcd_send_data8(0x36);
    ret |= lcd_send_data8(0x37);
    ret |= lcd_send_data8(0x6F);

    ret |= lcd_send_cmd(0xF2);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x09);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x26);
    ret |= lcd_send_data8(0x2A);

    ret |= lcd_send_cmd(0xF3);
    ret |= lcd_send_data8(0x43);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x72);
    ret |= lcd_send_data8(0x36);
    ret |= lcd_send_data8(0x37);
    ret |= lcd_send_data8(0x6F);

    ret |= lcd_send_cmd(0xED);
    ret |= lcd_send_data8(0x1B);
    ret |= lcd_send_data8(0x0B);

    ret |= lcd_send_cmd(0xAE);
    ret |= lcd_send_data8(0x77);

    ret |= lcd_send_cmd(0xCD);
    ret |= lcd_send_data8(0x63);

    ret |= lcd_send_cmd(0x70);
    ret |= lcd_send_data8(0x07);
    ret |= lcd_send_data8(0x07);
    ret |= lcd_send_data8(0x04);
    ret |= lcd_send_data8(0x0E);
    ret |= lcd_send_data8(0x0F);
    ret |= lcd_send_data8(0x09);
    ret |= lcd_send_data8(0x07);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x03);

    ret |= lcd_send_cmd(0xE8);
    ret |= lcd_send_data8(0x34);

    ret |= lcd_send_cmd(0x62);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x0D);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xED);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x0F);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xEF);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);

    ret |= lcd_send_cmd(0x63);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x11);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xF1);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x13);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xF3);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);

    ret |= lcd_send_cmd(0x64);
    ret |= lcd_send_data8(0x28);
    ret |= lcd_send_data8(0x29);
    ret |= lcd_send_data8(0xF1);
    ret |= lcd_send_data8(0x01);
    ret |= lcd_send_data8(0xF1);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x07);

    ret |= lcd_send_cmd(0x66);
    ret |= lcd_send_data8(0x3C);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0xCD);
    ret |= lcd_send_data8(0x67);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0x67);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x3C);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x01);
    ret |= lcd_send_data8(0x54);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x32);
    ret |= lcd_send_data8(0x98);

    ret |= lcd_send_cmd(0x74);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x85);
    ret |= lcd_send_data8(0x80);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x4E);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0x98);
    ret |= lcd_send_data8(0x3E);
    ret |= lcd_send_data8(0x07);

    ret |= lcd_send_cmd(0x35);
    ret |= lcd_send_cmd(0x21);

    ret |= lcd_send_cmd(0x11);
    k_msleep(120);
    ret |= lcd_send_cmd(0x29);
    k_msleep(20);

    return ret;
}


static int gc9a01_patch_init(const struct device *dev) {
    ARG_UNUSED(dev);

    if (!device_is_ready(lcd_spi.bus)) {
        printk("GC9A01 patch: SPI bus not ready\n");
        return 0;
    }
    if (!device_is_ready(lcd_dc.port) || !device_is_ready(lcd_rst.port)) {
        printk("GC9A01 patch: GPIO not ready\n");
        return 0;
    }

    int ret = lcd_reset_gc9();
    if (ret) {
        printk("GC9A01 patch: reset failed (%d)\n", ret);
        return 0;// config/src/gc9a01_patch.c


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
static const struct gpio_dt_spec lcd_rst =
    GPIO_DT_SPEC_GET(ST7789_NODE, reset_gpios);


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
    uint8_t buf[2] = { data >> 8, data & 0xFF };
    int ret = gpio_pin_set_dt(&lcd_dc, 1);
    if (ret) return ret;
    return lcd_write_bytes(buf, 2);
}

static int lcd_reset_gc9(void) {
    int ret;

    ret = gpio_pin_configure_dt(&lcd_rst, GPIO_OUTPUT_ACTIVE);
    if (ret) return ret;
    ret = gpio_pin_configure_dt(&lcd_dc, GPIO_OUTPUT_ACTIVE);
    if (ret) return ret;

    gpio_pin_set_dt(&lcd_rst, 1);
    k_msleep(100);
    gpio_pin_set_dt(&lcd_rst, 0);
    k_msleep(100);
    gpio_pin_set_dt(&lcd_rst, 1);
    k_msleep(100);

    return 0;
}


static int gc9a01_init_sequence(void) {
    int ret = 0;


    ret |= lcd_send_cmd(0xEF);
    ret |= lcd_send_cmd(0xEB);
    ret |= lcd_send_data8(0x14);

    ret |= lcd_send_cmd(0xFE);
    ret |= lcd_send_cmd(0xEF);

    ret |= lcd_send_cmd(0xEB);
    ret |= lcd_send_data8(0x14);

    ret |= lcd_send_cmd(0x84);
    ret |= lcd_send_data8(0x40);

    ret |= lcd_send_cmd(0x85);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x86);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x87);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x88);
    ret |= lcd_send_data8(0x0A);

    ret |= lcd_send_cmd(0x89);
    ret |= lcd_send_data8(0x21);

    ret |= lcd_send_cmd(0x8A);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0x8B);
    ret |= lcd_send_data8(0x80);

    ret |= lcd_send_cmd(0x8C);
    ret |= lcd_send_data8(0x01);

    ret |= lcd_send_cmd(0x8D);
    ret |= lcd_send_data8(0x01);

    ret |= lcd_send_cmd(0x8E);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0x8F);
    ret |= lcd_send_data8(0xFF);

    ret |= lcd_send_cmd(0xB6);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x20);

    ret |= lcd_send_cmd(0x36);
    ret |= lcd_send_data8(0x08); // 縦向き

    ret |= lcd_send_cmd(0x3A);
    ret |= lcd_send_data8(0x05);

    ret |= lcd_send_cmd(0x90);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);

    ret |= lcd_send_cmd(0xBD);
    ret |= lcd_send_data8(0x06);

    ret |= lcd_send_cmd(0xBC);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0xFF);
    ret |= lcd_send_data8(0x60);
    ret |= lcd_send_data8(0x01);
    ret |= lcd_send_data8(0x04);

    ret |= lcd_send_cmd(0xC3);
    ret |= lcd_send_data8(0x13);
    ret |= lcd_send_cmd(0xC4);
    ret |= lcd_send_data8(0x13);

    ret |= lcd_send_cmd(0xC9);
    ret |= lcd_send_data8(0x22);

    ret |= lcd_send_cmd(0xBE);
    ret |= lcd_send_data8(0x11);

    ret |= lcd_send_cmd(0xE1);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x0E);

    ret |= lcd_send_cmd(0xDF);
    ret |= lcd_send_data8(0x21);
    ret |= lcd_send_data8(0x0C);
    ret |= lcd_send_data8(0x02);

    ret |= lcd_send_cmd(0xF0);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x09);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x26);
    ret |= lcd_send_data8(0x2A);

    ret |= lcd_send_cmd(0xF1);
    ret |= lcd_send_data8(0x43);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x72);
    ret |= lcd_send_data8(0x36);
    ret |= lcd_send_data8(0x37);
    ret |= lcd_send_data8(0x6F);

    ret |= lcd_send_cmd(0xF2);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x09);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x26);
    ret |= lcd_send_data8(0x2A);

    ret |= lcd_send_cmd(0xF3);
    ret |= lcd_send_data8(0x43);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x72);
    ret |= lcd_send_data8(0x36);
    ret |= lcd_send_data8(0x37);
    ret |= lcd_send_data8(0x6F);

    ret |= lcd_send_cmd(0xED);
    ret |= lcd_send_data8(0x1B);
    ret |= lcd_send_data8(0x0B);

    ret |= lcd_send_cmd(0xAE);
    ret |= lcd_send_data8(0x77);

    ret |= lcd_send_cmd(0xCD);
    ret |= lcd_send_data8(0x63);

    ret |= lcd_send_cmd(0x70);
    ret |= lcd_send_data8(0x07);
    ret |= lcd_send_data8(0x07);
    ret |= lcd_send_data8(0x04);
    ret |= lcd_send_data8(0x0E);
    ret |= lcd_send_data8(0x0F);
    ret |= lcd_send_data8(0x09);
    ret |= lcd_send_data8(0x07);
    ret |= lcd_send_data8(0x08);
    ret |= lcd_send_data8(0x03);

    ret |= lcd_send_cmd(0xE8);
    ret |= lcd_send_data8(0x34);

    ret |= lcd_send_cmd(0x62);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x0D);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xED);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x0F);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xEF);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);

    ret |= lcd_send_cmd(0x63);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x11);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xF1);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x18);
    ret |= lcd_send_data8(0x13);
    ret |= lcd_send_data8(0x71);
    ret |= lcd_send_data8(0xF3);
    ret |= lcd_send_data8(0x70);
    ret |= lcd_send_data8(0x70);

    ret |= lcd_send_cmd(0x64);
    ret |= lcd_send_data8(0x28);
    ret |= lcd_send_data8(0x29);
    ret |= lcd_send_data8(0xF1);
    ret |= lcd_send_data8(0x01);
    ret |= lcd_send_data8(0xF1);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x07);

    ret |= lcd_send_cmd(0x66);
    ret |= lcd_send_data8(0x3C);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0xCD);
    ret |= lcd_send_data8(0x67);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x45);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0x67);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x3C);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x01);
    ret |= lcd_send_data8(0x54);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x32);
    ret |= lcd_send_data8(0x98);

    ret |= lcd_send_cmd(0x74);
    ret |= lcd_send_data8(0x10);
    ret |= lcd_send_data8(0x85);
    ret |= lcd_send_data8(0x80);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x00);
    ret |= lcd_send_data8(0x4E);
    ret |= lcd_send_data8(0x00);

    ret |= lcd_send_cmd(0x98);
    ret |= lcd_send_data8(0x3E);
    ret |= lcd_send_data8(0x07);

    ret |= lcd_send_cmd(0x35);
    ret |= lcd_send_cmd(0x21);

    ret |= lcd_send_cmd(0x11);
    k_msleep(120);
    ret |= lcd_send_cmd(0x29);
    k_msleep(20);

    return ret;
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

    printk("GC9A01 patch: init start\n");

    if (!device_is_ready(lcd_spi.bus)) {
        printk("GC9A01 patch: SPI bus not ready\n");
        return 0;
    }
    if (!device_is_ready(lcd_dc.port) || !device_is_ready(lcd_rst.port)) {
        printk("GC9A01 patch: GPIO not ready\n");
        return 0;
    }

    int ret = lcd_reset_gc9();
    if (ret) {
        printk("GC9A01 patch: reset failed (%d)\n", ret);
        return 0;
    }

    ret = gc9a01_init_sequence();
    if (ret) {
        printk("GC9A01 patch: init sequence error (%d)\n", ret);
        return 0;
    }

    ret = gc9a01_fill_color(0xF800);
    if (ret) {
        printk("GC9A01 patch: fill color error (%d)\n", ret);
    } else {
        printk("GC9A01 patch: init + fill OK\n");
    }

    return 0;
}

SYS_INIT(gc9a01_patch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

    }

    ret = gc9a01_init_sequence();
    if (ret) {
        printk("GC9A01 patch: init sequence error (%d)\n", ret);
    } else {
        printk("GC9A01 patch: init OK\n");
    }

    return 0;
}

SYS_INIT(gc9a01_patch_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
