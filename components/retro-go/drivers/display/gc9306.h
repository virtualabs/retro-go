#include <stdlib.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_dma_utils.h"
#include "esp_check.h"

//#include "rick.h"

#define gc9306_CMD_RAMCTRL               0xb0
#define gc9306_DATA_LITTLE_ENDIAN_BIT    (1 << 3)

///
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM
// PCLK frequency can't go too high as the limitation of PSRAM bandwidth
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (2 * 1000 * 1000)
#else
//#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (10 * 1000 * 1000)
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (10 * 1000 * 1000)
#endif // CONFIG_EXAMPLE_LCD_I80_COLOR_IN_PSRAM

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0          12
#define EXAMPLE_PIN_NUM_DATA1          13
#define EXAMPLE_PIN_NUM_DATA2          14
#define EXAMPLE_PIN_NUM_DATA3          15
#define EXAMPLE_PIN_NUM_DATA4          16
#define EXAMPLE_PIN_NUM_DATA5          21
#define EXAMPLE_PIN_NUM_DATA6          5
#define EXAMPLE_PIN_NUM_DATA7          4
#define EXAMPLE_PIN_NUM_DATA8          17
#define EXAMPLE_PIN_NUM_DATA9          18
#define EXAMPLE_PIN_NUM_DATA10         10
#define EXAMPLE_PIN_NUM_DATA11         11
#define EXAMPLE_PIN_NUM_DATA12         19
#define EXAMPLE_PIN_NUM_DATA13         20
#define EXAMPLE_PIN_NUM_DATA14         2
#define EXAMPLE_PIN_NUM_DATA15         1
#define EXAMPLE_PIN_NUM_PCLK           7
#define EXAMPLE_PIN_NUM_CS             9
#define EXAMPLE_PIN_NUM_DC             8
#define EXAMPLE_PIN_NUM_RST            6
#define EXAMPLE_PIN_NUM_BK_LIGHT       0

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              320
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#if CONFIG_EXAMPLE_LCD_TOUCH_ENABLED
#define EXAMPLE_I2C_NUM                 0   // I2C number
#define EXAMPLE_I2C_SCL                 39
#define EXAMPLE_I2C_SDA                 40
#endif

/* 320 rows / 4 rows = 80 "slices" */
#define LCD_FB_NB_ROWS              (4)
//#define LCD_FB_MAX                  (EXAMPLE_LCD_V_RES / LCD_FB_NB_ROWS)
#define LCD_FB_SIZE_BYTES           (240*LCD_FB_NB_ROWS*2)
#define LCD_FB_MAX 2 
#define LCD_RB_MAX (LCD_FB_MAX+1)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t cmd;
    uint8_t params[8];
    uint8_t nb_params;
} gc9306_init_cmd_t;


#ifdef __cplusplus
}
#endif

static esp_lcd_panel_io_handle_t io_handle = NULL;

static const char *TAG = "lcd_panel.gc9306";

static esp_err_t panel_gc9306_del(esp_lcd_panel_t *panel);
static esp_err_t panel_gc9306_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_gc9306_init(esp_lcd_panel_t *panel);
static esp_err_t panel_gc9306_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                          const void *color_data);
static esp_err_t panel_gc9306_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_gc9306_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_gc9306_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_gc9306_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_gc9306_disp_on_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t panel_gc9306_sleep(esp_lcd_panel_t *panel, bool sleep);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val;    // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val;    // save current value of LCD_CMD_COLMOD register
    uint8_t ramctl_val_1;
    uint8_t ramctl_val_2;
} gc9306_panel_t;


/* GC9306 Init commands. */
gc9306_init_cmd_t g_init_cmds[] = {
    {0xFE, {0}, 0},     /* Enable inner register 1. */
    {0xEF, {0}, 0},     /* Enable inner register 2. */
    {0x36, {0x48}, 1},  /* Set screen orientation (MV=1, MY=0, MX=0, ML=0, BGR=1) */
    {0x3A, {0x05}, 1},  /* 16-bits per pixels on MCU interface. */
    {0xA4, {0x44}, 1},  /* Set VCore voltage to 1.7v */
    {0xA6, {0x2A}, 1},  /* Set VREG1A OUT voltage */
    {0xA7, {0x2B}, 1},  /* Set VREG1B OUT voltage */
    {0xA8, {0x18}, 1},  /* Set VREG2A OUT voltage */
    {0xA9, {0x2a}, 1},  /* Set VREG2B OUT voltage */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},    /* Set CAS */
    {0x2B, {0x00, 0x00, 0x01, 0x3F}, 4},    /* Set RAS */
    {0x2C, {0}, 0},  /* Write memory */
    {0xF0, {0x02, 0x00, 0x00, 0x1B, 0x1F, 0x0B}, 6},    /* Set GAMMA1 */
    {0xF1, {0x01, 0x03, 0x00, 0x28, 0x2B, 0x0E}, 6},    /* Set GAMMA2 */
    {0xF2, {0x0B, 0x08, 0x3B, 0x04, 0x03, 0x4C}, 6},    /* Set GAMMA3 */
    {0xF3, {0x0E, 0x07, 0x46, 0x04, 0x05, 0x51}, 6},    /* Set GAMMA4 */
    {0xF4, {0x08, 0x15, 0x15, 0x1F, 0x22, 0x0F}, 6},    /* Set GAMMA5 */
    {0xF5, {0x0B, 0x13, 0x11, 0x1F, 0x21, 0x0F}, 6},    /* Set GAMMA6 */
    {0x00, {0}, 0}
};

typedef struct {
    /* Jitter capacity. */
    int capacity;
    
    /* Current size. */
    int size;

    /* Pending flush. */
    bool pending;

    /* Buffer. */
    uint16_t buffer[LCD_BUFFER_LENGTH*4];
} jitter_t;

typedef struct {
    /* Current window settings. */
    int x;
    int xend;
    int y;
    int yend;
    int width;
    int height;

    /* Jitter height. */
    int nrows;

    /* Pixel count. */
    int pixel_total;
    int pixel_count; 

    /* Jitters. */
    jitter_t jitter;
} rg_window_t;

static SemaphoreHandle_t g_lcd_sem = NULL;
static rg_window_t g_window;

/* Main framebuffer. */
static uint16_t g_fb[LCD_BUFFER_LENGTH];


/***
 * Jitter implementation
 **/

void jitter_init(jitter_t *jitter, int capacity)
{
    /* Reset jitter state. */
    jitter->size = 0;
    jitter->pending = false;

    /* Save its capacity. */
    jitter->capacity = capacity;
}


bool jitter_add(jitter_t *jitter, uint16_t *buffer, int size)
{
    //RG_LOGD("   adding %d data bytes to jitter ...", size);

    /* Do we have enough space to save our data ? */
    if ((jitter->size + size) > (jitter->capacity*2))
    {
        /* Failure, overflow. SHOULD NOT HAPPEN. */
        return false;
    }
    else
    {
        //RG_LOGD("  jitter size before adding data: %d bytes", jitter->size);

        /* Append data to jitter's buffer. */
        for (int j=0; j<size/2; j++)
        {
            jitter->buffer[jitter->size/2+j] = (buffer[j]>>8) | ((buffer[j]&0x00ff)<<8);
        }
        //memcpy(&jitter->buffer[jitter->size], buffer, size);

        /* Increase jitter size by 'size' bytes. */
        jitter->size += size;

        //RG_LOGD("  jitter size after data added: %d bytes", jitter->size);

        /* Success. */
        return true;
    }
}

bool jitter_full(jitter_t *jitter)
{
    return (jitter->size >= jitter->capacity);
}

bool jitter_flush(jitter_t *jitter)
{
    if (jitter_full(jitter))
    {
        /* Flush jitter. */
        if (jitter->capacity != jitter->size)
        {
            for (int i=0; i<(jitter->size - jitter->capacity)/2; i++)
            {
                jitter->buffer[i] = jitter->buffer[jitter->capacity/2 + i];
            }
        }

        /* Adjust jitter's size. */
        jitter->size -= jitter->capacity;

        /* Reset the pending state. */
        jitter->pending = false;

        /* Success. */
        return true;
    }

    /* Not full. */
    return false;
}


/***
 * GC9306 Panel driver
 **/

esp_err_t
esp_lcd_new_panel_gc9306(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                         esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    gc9306_panel_t *gc9306 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    gc9306 = calloc(1, sizeof(gc9306_panel_t));
    ESP_GOTO_ON_FALSE(gc9306, ESP_ERR_NO_MEM, err, TAG, "no mem for gc9306 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_endian) {
    case LCD_RGB_ENDIAN_RGB:
        gc9306->madctl_val = 0;
        break;
    case LCD_RGB_ENDIAN_BGR:
        gc9306->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        gc9306->colmod_val = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        gc9306->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    gc9306->ramctl_val_1 = 0x00;
    gc9306->ramctl_val_2 = 0xf0;    // Use big endian by default
    if ((panel_dev_config->data_endian) == LCD_RGB_DATA_ENDIAN_LITTLE) {
        // Use little endian
        gc9306->ramctl_val_2 |= gc9306_DATA_LITTLE_ENDIAN_BIT;
    }

    gc9306->io = io;
    gc9306->fb_bits_per_pixel = fb_bits_per_pixel;
    gc9306->reset_gpio_num = panel_dev_config->reset_gpio_num;
    gc9306->reset_level = panel_dev_config->flags.reset_active_high;
    gc9306->base.del = panel_gc9306_del;
    gc9306->base.reset = panel_gc9306_reset;
    gc9306->base.init = panel_gc9306_init;
    gc9306->base.draw_bitmap = panel_gc9306_draw_bitmap;
    gc9306->base.invert_color = panel_gc9306_invert_color;
    gc9306->base.set_gap = panel_gc9306_set_gap;
    gc9306->base.mirror = panel_gc9306_mirror;
    gc9306->base.swap_xy = panel_gc9306_swap_xy;
    gc9306->base.disp_on_off = panel_gc9306_disp_on_off;
    gc9306->base.disp_sleep = panel_gc9306_sleep;
    *ret_panel = &(gc9306->base);
    RG_LOGD("new gc9306 panel @%p", gc9306);

    return ESP_OK;

err:
    RG_LOGD("Error occurred while setting up gc9306 !");
    if (gc9306) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(gc9306);
    }
    return ret;
}

static esp_err_t panel_gc9306_del(esp_lcd_panel_t *panel)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);

    if (gc9306->reset_gpio_num >= 0) {
        gpio_reset_pin(gc9306->reset_gpio_num);
    }
    RG_LOGD("del gc9306 panel @%p", gc9306);
    free(gc9306);
    return ESP_OK;
}

static esp_err_t panel_gc9306_reset(esp_lcd_panel_t *panel)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;

    RG_LOGD("reset gc9306 panel");

    // perform hardware reset
    if (gc9306->reset_gpio_num >= 0) {
        gpio_set_level(gc9306->reset_gpio_num, gc9306->reset_level);
        //vTaskDelay(pdMS_TO_TICKS(10));
        rg_usleep(10 * 1000);
        gpio_set_level(gc9306->reset_gpio_num, !gc9306->reset_level);
        //vTaskDelay(pdMS_TO_TICKS(10));
        rg_usleep(10 * 1000);
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
                            "io tx param failed");
        //vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
        rg_usleep(20 * 1000);
    }

    return ESP_OK;
}

static esp_err_t panel_gc9306_init(esp_lcd_panel_t *panel)
{
    int i=0;
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;

    RG_LOGD("init gc9306 panel ...");

    /* GC9306 init commands */
    while (g_init_cmds[i].cmd != 0)
    {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, g_init_cmds[i].cmd, g_init_cmds[i].params, g_init_cmds[i].nb_params), TAG, "io tx param failed");
        //vTaskDelay(pdMS_TO_TICKS(10));
        rg_usleep(10 * 1000);
        
        i++;
    }

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG,
                        "io tx param failed");
    //vTaskDelay(pdMS_TO_TICKS(100));
    rg_usleep(100 * 1000);

    return ESP_OK;
}

static esp_err_t panel_gc9306_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                          const void *color_data)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = gc9306->io;

    x_start += gc9306->x_gap;
    x_end += gc9306->x_gap;
    y_start += gc9306->y_gap;
    y_end += gc9306->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4), TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4), TAG, "io tx param failed");
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * gc9306->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "io tx color failed");

    return ESP_OK;
}

static esp_err_t panel_gc9306_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_gc9306_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;
    if (mirror_x) {
        gc9306->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        gc9306->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        gc9306->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        gc9306->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        gc9306->madctl_val
    }, 1), TAG, "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_gc9306_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;
    if (swap_axes) {
        gc9306->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        gc9306->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        gc9306->madctl_val
    }, 1), TAG, "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_gc9306_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    gc9306->x_gap = x_gap;
    gc9306->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_gc9306_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;
    int command = 0;

    RG_LOGD("enable gc9306 display (%d)", on_off);
    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

static esp_err_t panel_gc9306_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;
    int command = 0;
    if (sleep) {
        command = LCD_CMD_SLPIN;
    } else {
        command = LCD_CMD_SLPOUT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    //vTaskDelay(pdMS_TO_TICKS(100));
    rg_usleep(100 * 1000);

    return ESP_OK;
}

/** I8080 interface with LVGL. **/

bool lcd_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    int fb;
    BaseType_t hptask_woken = pdFALSE;
      
    /* Release ownership of our LCD mutex. */
    xSemaphoreGiveFromISR(g_lcd_sem, &hptask_woken);

    return hptask_woken;
}

void example_init_i80_bus(esp_lcd_panel_io_handle_t *io_handle, void *user_ctx)
{
    RG_LOGD("Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
        .wr_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .bus_width = 16,
        .max_transfer_bytes = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t),
        .sram_trans_align = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = false, // Swap can be done in LvGL (default) or DMA
        },
        .on_color_trans_done = lcd_trans_done,
        .user_ctx = NULL,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, io_handle));
}

static void panel_fill(esp_lcd_panel_io_handle_t io, unsigned int x, unsigned int y, unsigned int w, unsigned int h, uint16_t color)
{
    int i;
    uint32_t pixels_count;
    unsigned int ymax = y+h-1;
    unsigned int xmax = x+w-1;
    unsigned int yend = -1;
    int nrows = (LCD_FB_SIZE_BYTES / (w*2));

    if (nrows > h)
    {
        nrows = h;
    }

    while (nrows > 0)
    {
        //RG_LOGD("panel_fill(): preparing to send pixels from row %d to %d", y, y+nrows);

        /* Compute last row index. */
        yend = y + nrows - 1;
        //RG_LOGD("yend=%d (y+nrows)", yend);

        /* Fill framebuffer with our color. */
        pixels_count = nrows*w;
        for (i=0; i<pixels_count; i++)
        {
            g_fb[i] = color;
        }

        /* Send to LCD. */
        //RG_LOGD("Send CASEL/PASEL to LCD driver: (%d,%d) - (%d,%d)", x,y, xmax,yend);

        /* Send to LCD. */
        if (xSemaphoreTake(g_lcd_sem, portMAX_DELAY) == pdTRUE)
        {
            esp_lcd_panel_io_tx_param(io, 0x2A, (uint8_t[]){(x>>8)&0xff, x&0xff, (xmax >> 8) & 0xff, xmax & 0xff}, 4);
            esp_lcd_panel_io_tx_param(io, 0x2B, (uint8_t[]){(y>>8)&0xff, y&0xff, (yend >> 8) & 0xff, yend & 0xff}, 4);

            /* Send colors. */
            //RG_LOGD("Sending pixels (%lu bytes) ...", pixels_count*2);
            esp_lcd_panel_io_tx_color(io, 0x2C, g_fb , pixels_count*2);
        }
        else
        {
            RG_LOGW("Could not take mutex ownership :(");
        }

        
        /* Process next slice. */
        //RG_LOGD("moving y from %d to %d", y, y+nrows);
        y += nrows;

        /* Are we done ? */
        if (y == (ymax+1))
        {
            break;
        }
        else if ((ymax - y) < nrows)
        {
            nrows = ymax-y+1;
        }
    }
}

void example_init_lcd_panel(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t *panel)
{
    esp_lcd_panel_handle_t panel_handle = NULL;

    /* Install LCD driver */
    RG_LOGD("Install LCD driver of GC9306");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9306(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    *panel = panel_handle;
}

/**
 * Display driver exposed functions
 **/



static void lcd_set_backlight(float percent)
{
    /* TODO */
}

static void lcd_set_window(int left, int top, int width, int height)
{
    //RG_LOGD("setting drawing window: x=%d, y=%d, w=%d, h=%d", left, top, width, height);
    
    /* Save window in our structure. */
    g_window.x = left;
    g_window.y = top;
    g_window.width = width;
    g_window.height = height;
    g_window.xend = left + width - 1;
    g_window.yend = top + height - 1;
    g_window.pixel_total = width*height;
    g_window.pixel_count = 0;

    /* Compute our jitter size and line numbers. */
    g_window.nrows = (LCD_BUFFER_LENGTH / width);  
    jitter_init(&g_window.jitter, g_window.nrows * width * 2);  
    //RG_LOGD("Jitter intialized with capacity=%d", g_window.jitter.capacity);
}

static inline uint16_t *lcd_get_buffer(size_t length)
{
    return g_fb;
}

static inline void lcd_send_buffer(uint16_t *buffer, size_t length)
{
    int jitter_left = 0;

    int x = g_window.x;
    int xmax = g_window.xend;
    int y = g_window.y;
    int yend;

    //RG_LOGD("sending %d pixels (%d bytes) to current window", length, length*2);

    /* Should we flush our jitter ? */
    if (g_window.jitter.pending)
    {
        //RG_LOGD("Jitter needs to be flushed, current size: %d", g_window.jitter.size);
        if (!jitter_flush(&g_window.jitter))
        {
            RG_LOGE("Could not flush jitter.");
        }
        //RG_LOGD("Jitter size after flush: %d", g_window.jitter.size);
    }

    /* Add pixel data to our jitter. */
    //RG_LOGD("adding %d bytes to jitter", length*2);
    if (jitter_add(&g_window.jitter, buffer, length*2))
    {
        /* Is our jitter full ? (ready to be sent) */
        if (jitter_full(&g_window.jitter))
        {
            //RG_LOGD("Jitter is full, sending data to screen.");

            /* Send jitter to screen. */
            if (xSemaphoreTake(g_lcd_sem, portMAX_DELAY) == pdTRUE)
            {
                yend = RG_MIN(g_window.y + g_window.nrows - 1, g_window.yend);

                esp_lcd_panel_io_tx_param(io_handle, 0x2A, (uint8_t[]){(x>>8)&0xff, x&0xff, (xmax >> 8) & 0xff, xmax & 0xff}, 4);
                esp_lcd_panel_io_tx_param(io_handle, 0x2B, (uint8_t[]){(y>>8)&0xff, y&0xff, (yend >> 8) & 0xff, yend & 0xff}, 4);

                /* Send colors. */
                //RG_LOGD("Sending pixels (%d bytes) ...", g_window.jitter.capacity);
                g_window.jitter.pending = true;
                esp_lcd_panel_io_tx_color(io_handle, 0x2C, g_window.jitter.buffer, g_window.jitter.capacity);

                /* Update the number of pixels already sent. */
                g_window.pixel_count += length;
                jitter_left = g_window.jitter.size - g_window.jitter.capacity;

                /* Update window y. */
                g_window.y += g_window.nrows;

                //RG_LOGD("Remaining bytes in jitter: %d", jitter_left);
            }
            else
            {
                RG_LOGW("Could not take mutex ownership :(");
            }
        }
        else
        {
            //RG_LOGD("jitter not full, current size: %d bytes", g_window.jitter.size);
            jitter_left = g_window.jitter.size;
        }

        /* Process data left in jitter. */
        //RG_LOGD("Check if last jitter to be sent: pixels=%d total=%d jitter=%d", g_window.pixel_count, g_window.pixel_total, jitter_left/2);
        if ((g_window.pixel_count + jitter_left/2) == g_window.pixel_total)
        {
            //RG_LOGD("Received %d pixels, %d pixels in jitter = %d pixels total (window full)",
            //        g_window.pixel_count, jitter_left/2, g_window.pixel_total);

            /* Flush jitter if needed. */
            if (g_window.jitter.pending)
            {
                //RG_LOGD("Jitter needs flushing: %d / %d bytes", g_window.jitter.size, g_window.jitter.capacity); 
                jitter_flush(&g_window.jitter);
                //RG_LOGD("Jitter has been flushed.");
            }

            //RG_LOGD("Window is full, sending remaining data to screen.");

            /* Send jitter to screen. */
            if (xSemaphoreTake(g_lcd_sem, portMAX_DELAY) == pdTRUE)
            {
                yend = g_window.yend;

                esp_lcd_panel_io_tx_param(io_handle, 0x2A, (uint8_t[]){(x>>8)&0xff, x&0xff, (xmax >> 8) & 0xff, xmax & 0xff}, 4);
                esp_lcd_panel_io_tx_param(io_handle, 0x2B, (uint8_t[]){(y>>8)&0xff, y&0xff, (yend >> 8) & 0xff, yend & 0xff}, 4);

                /* Send colors. */
                //RG_LOGD("Sending pixels (%d bytes) ...", jitter_left);
                esp_lcd_panel_io_tx_color(io_handle, 0x2C, g_window.jitter.buffer, jitter_left);

                /* Update the number of pixels already sent. */
                g_window.pixel_count += g_window.jitter.size/2;
            }
            else
            {
                RG_LOGW("Could not take mutex ownership :(");
            } 
        }
        else
        {
            //RG_LOGD("Received %d pixels so far, waiting more to reach %d pixels.", g_window.pixel_count, g_window.pixel_total);
        }

    } 
    else
    {
        RG_LOGE("Jitter has overflowed !");
    }
}

static void lcd_sync(void)
{
    // Unused for SPI LCD
}

/**
 * Initialize our GC9306 I80 LCD controller.
 **/

static void lcd_init()
{
    int fb;

    memset(g_fb, 0, LCD_FB_SIZE_BYTES);

    g_lcd_sem = xSemaphoreCreateBinary();
    if (g_lcd_sem == NULL)
    {
        RG_LOGE("Cannot create LCD binary semaphore !");
    }
    xSemaphoreGive(g_lcd_sem);

    RG_LOGD("initializing I8080 bus ...");
    example_init_i80_bus(&io_handle, NULL);

    RG_LOGD("initializing LCD GC9306 panel ...");
    esp_lcd_panel_handle_t panel_handle = NULL;
    example_init_lcd_panel(io_handle, &panel_handle);

    RG_LOGD("enabling lcd ...");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    rg_usleep(100*1000);

    /* Clean panel. */
    //RG_LOGD("filling with rick ...");
    //panel_fill(io_handle, 0, 0, 240, 320, 0x0000);
    //panel_blit(io_handle, 0, 0, 240, 314, rick, 75360);
    rg_display_clear(C_BLACK);
    //rg_usleep(3000 * 1000);
    RG_LOGD("init done");
}

static void lcd_deinit(void)
{
    // TODO
}

const rg_display_driver_t rg_display_driver_gc9306 = {
    .name = "gc9306",
};


