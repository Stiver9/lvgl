#include <stdio.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#define GC9107_INVOFF 0x20
#define GC9107_INVON 0x21
#define GC9107_DISPOFF 0x28
#define GC9107_DISPON 0x29

#define GC9107_CASET 0x2A
#define GC9107_RASET 0x2B
#define GC9107_RAMWR 0x2C
#define GC9107_RAMRD 0x2E

#define GC9107_MADCTL 0x36
#define GC9107_COLMOD 0x3A

#define GC9107_MADCTL_MY 0x80
#define GC9107_MADCTL_MX 0x40
#define GC9107_MADCTL_MV 0x20
#define GC9107_MADCTL_ML 0x10
#define GC9107_MADCTL_BGR 0x08
#define GC9107_MADCTL_MH 0x04
#define GC9107_MADCTL_RGB 0x00

static const char *TAG = "lcd_panel.gc9107";

static esp_err_t panel_gc9107_del(esp_lcd_panel_t *panel);
static esp_err_t panel_gc9107_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_gc9107_init(esp_lcd_panel_t *panel);
static esp_err_t panel_gc9107_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_gc9107_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_gc9107_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_gc9107_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_gc9107_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_gc9107_disp_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_cal; // save surrent value of LCD_CMD_COLMOD register
} gc9107_panel_t;

esp_err_t esp_lcd_new_panel_gc9107(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    gc9107_panel_t *gc9107 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    gc9107 = calloc(1, sizeof(gc9107_panel_t));
    ESP_GOTO_ON_FALSE(gc9107, ESP_ERR_NO_MEM, err, TAG, "no mem for gc9107 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        gc9107->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        gc9107->madctl_val |= GC9107_MADCTL_BGR;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        gc9107->colmod_cal = 0x55;
        fb_bits_per_pixel = 16;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    gc9107->io = io;
    gc9107->fb_bits_per_pixel = fb_bits_per_pixel;
    gc9107->reset_gpio_num = panel_dev_config->reset_gpio_num;
    gc9107->reset_level = panel_dev_config->flags.reset_active_high;
    gc9107->base.del = panel_gc9107_del;
    gc9107->base.reset = panel_gc9107_reset;
    gc9107->base.init = panel_gc9107_init;
    gc9107->base.draw_bitmap = panel_gc9107_draw_bitmap;
    gc9107->base.invert_color = panel_gc9107_invert_color;
    gc9107->base.set_gap = panel_gc9107_set_gap;
    gc9107->base.mirror = panel_gc9107_mirror;
    gc9107->base.swap_xy = panel_gc9107_swap_xy;
    gc9107->base.disp_off = panel_gc9107_disp_off;
    *ret_panel = &(gc9107->base);
    ESP_LOGD(TAG, "new gc9107 panel @%p", gc9107);

    return ESP_OK;

err:
    if (gc9107) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(gc9107);
    }
    return ret;
}

static esp_err_t panel_gc9107_del(esp_lcd_panel_t *panel)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);

    if (gc9107->reset_gpio_num >= 0) {
        gpio_reset_pin(gc9107->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del gc9107 panel @%p", gc9107);
    free(gc9107);
    return ESP_OK;
}

static esp_err_t panel_gc9107_reset(esp_lcd_panel_t *panel)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9107->io;

    // perform hardware reset
    if (gc9107->reset_gpio_num >= 0) {
        gpio_set_level(gc9107->reset_gpio_num, gc9107->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100)); // 
        gpio_set_level(gc9107->reset_gpio_num, !gc9107->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100)); // 100 ms
    } else { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(100)); // spec, wait at least 5m before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_gc9107_init(esp_lcd_panel_t *panel)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9107->io;

    esp_lcd_panel_io_tx_param(io, 0xFE, NULL, 0);
    esp_lcd_panel_io_tx_param(io, 0xEF, NULL, 0);

    esp_lcd_panel_io_tx_param(io, 0xB0, (uint8_t[]) {
         0xC0,
     }, 1);
    esp_lcd_panel_io_tx_param(io, 0xB2, (uint8_t[]) {
         0x2F,
     }, 1);
    esp_lcd_panel_io_tx_param(io, 0xB3, (uint8_t[]) {
         0x03,
     }, 1);
    esp_lcd_panel_io_tx_param(io, 0xB6, (uint8_t[]) {
         0x19,
     }, 1);
    esp_lcd_panel_io_tx_param(io, 0xB7, (uint8_t[]) {
         0x01,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xAC, (uint8_t[]) {
         0xCB,
     }, 1);
    esp_lcd_panel_io_tx_param(io, 0xAB, (uint8_t[]) {
         0x0E,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xB4, (uint8_t[]) {
         0x04,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xA8, (uint8_t[]) {
         0x17,
     }, 1);    

    esp_lcd_panel_io_tx_param(io, 0x3A, (uint8_t[]) {
         0x05,
     }, 1);  

    esp_lcd_panel_io_tx_param(io, 0xB8, (uint8_t[]) {
         0x08,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xE8, (uint8_t[]) {
         0x24,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xE9, (uint8_t[]) {
         0x48,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xEA, (uint8_t[]) {
         0x22,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xC6, (uint8_t[]) {
         0x21,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xC7, (uint8_t[]) {
         0x12,
     }, 1);

    esp_lcd_panel_io_tx_param(io, 0xF0, (uint8_t[]) {
         0x1F,
         0x28,
         0x04,
         0x3E,
         0x2A,
         0x2E,
         0x20,
         0x00,
         0x0C,
         0x06,
         0x00,
         0x1C,
         0x1F,
         0x0f,
     }, 14);

     esp_lcd_panel_io_tx_param(io, 0xF1, (uint8_t[]) {
         0X00,
         0X2D,
         0X2F,
         0X3C,
         0X6F,
         0X1C,
         0X0B,
         0X00,
         0X00,
         0X00,
         0X07,
         0X0D,
         0X11,
         0X0f,
     }, 14);    

    esp_lcd_panel_io_tx_param(io, 0x21, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(io, 0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(io, 0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));   
   
    // Rotation
    esp_lcd_panel_io_tx_param(io, GC9107_MADCTL, (uint8_t[]) {
        (GC9107_MADCTL_BGR),
    }, 1);

    return ESP_OK;
}

static esp_err_t panel_gc9107_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = gc9107->io;

    x_start += gc9107->x_gap;
    x_end += gc9107->x_gap;
    y_start += gc9107->y_gap;
    y_end += gc9107->y_gap;

    esp_lcd_panel_io_tx_param(io, GC9107_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4);
    esp_lcd_panel_io_tx_param(io, GC9107_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4);
    size_t len = (x_end - x_start) * (y_end - y_start) * gc9107->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, GC9107_RAMWR, color_data, len);
    return ESP_OK;
}

static esp_err_t panel_gc9107_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9107->io;
    int command = 0;
    if (invert_color_data) {
        command = GC9107_INVON;
    } else {
        command = GC9107_INVOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_gc9107_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9107->io;
    if (mirror_x) {
        gc9107->madctl_val |= GC9107_MADCTL_MX;
    } else {
        gc9107->madctl_val &= ~GC9107_MADCTL_MX;
    }
    if (mirror_y) {
        gc9107->madctl_val |= GC9107_MADCTL_MY;
    } else {
        gc9107->madctl_val &= ~GC9107_MADCTL_MY;
    }
    esp_lcd_panel_io_tx_param(io, GC9107_MADCTL, (uint8_t[]) {
        gc9107->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_gc9107_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9107->io;
    if (swap_axes) {
        gc9107->madctl_val |= GC9107_MADCTL_MV;
    } else {
        gc9107->madctl_val &= ~GC9107_MADCTL_MV;
    }
    esp_lcd_panel_io_tx_param(io, GC9107_MADCTL, (uint8_t[]) {
        gc9107->madctl_val
    }, 1);
    return ESP_OK;
}

static esp_err_t panel_gc9107_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    gc9107->x_gap = x_gap;
    gc9107->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_gc9107_disp_off(esp_lcd_panel_t *panel, bool off)
{
    gc9107_panel_t *gc9107 = __containerof(panel, gc9107_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9107->io;
    int command = 0;
    if (off) {
        command = GC9107_INVOFF;
    } else {
        command = GC9107_INVON;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}
