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

#define gc9306_CMD_RAMCTRL               0xb0
#define gc9306_DATA_LITTLE_ENDIAN_BIT    (1 << 3)

#define RG_SCREEN_CLOCK_HZ     (20 * 1000 * 1000)

// Bit number used to represent command and parameter
#define RG_SCREEN_CMD_BITS           8
#define RG_SCREEN_PARAM_BITS         8

/* 320 rows / 4 rows = 80 "slices" */
#define LCD_FB_NB_ROWS              (4)
#define LCD_FB_SIZE_BYTES           (RG_SCREEN_WIDTH*LCD_FB_NB_ROWS*2)


/****************************
 * GC9306 Panel control
 ***************************/

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

/* Exposed functions. */
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

/* Panel custom structure. */
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
    {0x36, {0xe8}, 1},  /* Set screen orientation (MV=1, MY=1, MX=1, ML=0, BGR=1) */
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


/****************************
 * LCD panel jitter.
 *
 * This jitter is used during serial/parallel
 * conversion due to the original display control
 * implemented in RetroGo (designed to drive an
 * SPI-based LCD screen instead of our I8080-based
 * screen).
 *
 * The jitter stores incoming pixel data and rebuild
 * the current subframes based on serial data sent
 * by RetroGo. This way, we can keep the drawing
 * window consistent and avoid sending raw data
 * without specifying the window, unlike SPI-based
 * screens. 
 ***************************/


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

    /* Last transaction flag. */
    bool b_last;
} rg_window_t;

/* Exposed functions. */
void jitter_init(jitter_t *jitter, int capacity);

/**
 * Globals
 **/

/* Debug tag. */
static const char *TAG = "lcd_panel.gc9306";

/* LCD panel handle. */
static esp_lcd_panel_io_handle_t io_handle = NULL;

/* SPI screen to I8080 adapter */
static SemaphoreHandle_t g_lcd_sem = NULL;
static SemaphoreHandle_t g_disp_sem = NULL;
static rg_window_t g_window;

/* Main framebuffer. */
static uint16_t g_fb[LCD_BUFFER_LENGTH];
static uint16_t g_lcd_trans[LCD_BUFFER_LENGTH];


/****************************
 * Jitter implementation.
 ***************************/

/**
 * Initialize a jitter with the given capacity.
 **/

void jitter_init(jitter_t *jitter, int capacity)
{
    /* Reset jitter state. */
    jitter->size = 0;
    jitter->pending = false;

    /* Save its capacity. */
    jitter->capacity = capacity;
}


/**
 * Append data to a given jitter.
 *
 * `buffer` points to the pixel data to add to the jitter,
 * `size` specifies the data size in bytes.
 **/

bool jitter_add(jitter_t *jitter, uint16_t *buffer, int size)
{
    /* Do we have enough space to save our data ? */
    if ((jitter->size + size) > (jitter->capacity*2))
    {
        /* Failure, overflow. SHOULD NOT HAPPEN. */
        return false;
    }
    else
    {
        /* Append data to jitter's buffer. */
        for (int j=0; j<size/2; j++)
        {
            jitter->buffer[jitter->size/2+j] = (buffer[j]>>8) | ((buffer[j]&0x00ff)<<8);
        }

        /* Increase jitter size by 'size' bytes. */
        jitter->size += size;

        /* Success. */
        return true;
    }
}


/**
 * Determine if a given jitter is full.
 *
 * A jitter is considered 'full' when the size of data
 * it contains is equal to or exceed its capacity. That
 * usually means it should be empty into the screen buffer.
 **/

bool jitter_full(jitter_t *jitter)
{
    return (jitter->size >= jitter->capacity);
}


/**
 * Flush a given jitter.
 *
 * Current data is removed from the jitter. If the current data size exceeds
 * the jitter capacity, only the current `capacity` bytes of data is flushed
 * and the remaining data is moved in the jitter's buffer.
 **/

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
        /* TODO: remove pending if not used anymore. */
        jitter->pending = false;

        /* Success. */
        return true;
    }
    else
    {
        /* Not full, no extra data to handle. */
        jitter->size = 0;
    }

    /* Not full. */
    return false;
}


/****************************
 * GC9306 Panel control impl.
 ***************************/

/**
 * Initialize a new GC9306 panel control structure.
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


/**
 * Deinit a previously allocated GC9306 panel control structure.
 **/

static esp_err_t panel_gc9306_del(esp_lcd_panel_t *panel)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);

    if (gc9306->reset_gpio_num >= 0) {
        gpio_reset_pin(gc9306->reset_gpio_num);
    }
    free(gc9306);
    return ESP_OK;
}


/**
 * Reset the GC9306 controller.
 **/

static esp_err_t panel_gc9306_reset(esp_lcd_panel_t *panel)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;

    // perform hardware reset
    if (gc9306->reset_gpio_num >= 0) {
        gpio_set_level(gc9306->reset_gpio_num, gc9306->reset_level);
        rg_usleep(10 * 1000);
        gpio_set_level(gc9306->reset_gpio_num, !gc9306->reset_level);
        rg_usleep(10 * 1000);
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
                            "io tx param failed");
        rg_usleep(20 * 1000);
    }

    return ESP_OK;
}


/**
 * Initialize the GC9306 controller.
 *
 * This function loops on `g_init_cmds` to configure the LCD panel
 * with the correct orientation and pixel format.
 **/

static esp_err_t panel_gc9306_init(esp_lcd_panel_t *panel)
{
    int i=0;
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;

    /* GC9306 init commands */
    while (g_init_cmds[i].cmd != 0)
    {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, g_init_cmds[i].cmd, g_init_cmds[i].params, g_init_cmds[i].nb_params), TAG, "io tx param failed");
        rg_usleep(10 * 1000);
        
        i++;
    }

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG,
                        "io tx param failed");
    rg_usleep(100 * 1000);

    return ESP_OK;
}


/**
 * Draw a bitmap on screen.
 **/

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

/**
 * Invert colors on LCD panel.
 **/

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

/**
 * Configure the LCD panel's MAD register.
 *
 * The MAD register controls the screen and window orientation. Should be used with care.
 **/

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


/**
 * Swap X/Y axis on the LCD screen.
 *
 * Be careful, drawing window is impacted by any axis swap.
 **/

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


/**
 * Set the GC9306 controller X and Y gaps.
 *
 * Gaps allow to skip a number of pixels on LCD sides.
 **/

static esp_err_t panel_gc9306_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    gc9306->x_gap = x_gap;
    gc9306->y_gap = y_gap;
    return ESP_OK;
}


/**
 * Enable/disable display.
 *
 * When enabled, the GC9306 controller is live and drives the TFT matrix. When
 * disabled, the screen is no more updated but pixels stay.
 * Backlight should be switched off to avoid artefacts.
 **/

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


/**
 * Put the GC9306 controller in sleep mode or wake it up.
 *
 * When in sleep mode, the GC9306 controller only reacts to
 * wake up command.
 **/

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
    rg_usleep(100 * 1000);

    return ESP_OK;
}

/****************************
 * RetroGo i8080 display impl.
 ***************************/

/**
 * Callback function to handle a succesfull i8080 transaction (pixel data).
 *
 * It gives back a semaphore used by the display task to synchronize and 
 * send the remaining pixel data.
 **/

bool IRAM_ATTR lcd_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t hptask_woken = pdFALSE;
      
    /* Release ownership of our LCD mutex. */
    xSemaphoreGiveFromISR(g_lcd_sem, &hptask_woken);

    return hptask_woken;
}


/**
 * Initialize the ESP32 LCD i8080 bus.
 *
 * This function configures the i8080 interface clock, pins, the screen
 * size, pixel data format and LCD commands and bits sizes.
 **/

void i80_bus_init(esp_lcd_panel_io_handle_t *io_handle, void *user_ctx)
{
    RG_LOGD("Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = RG_SCREEN_DC,
        .wr_gpio_num = RG_SCREEN_PCLK,
        .data_gpio_nums = {
            RG_SCREEN_DATA0,
            RG_SCREEN_DATA1,
            RG_SCREEN_DATA2,
            RG_SCREEN_DATA3,
            RG_SCREEN_DATA4,
            RG_SCREEN_DATA5,
            RG_SCREEN_DATA6,
            RG_SCREEN_DATA7,
            RG_SCREEN_DATA8,
            RG_SCREEN_DATA9,
            RG_SCREEN_DATA10,
            RG_SCREEN_DATA11,
            RG_SCREEN_DATA12,
            RG_SCREEN_DATA13,
            RG_SCREEN_DATA14,
            RG_SCREEN_DATA15,
        },
        .bus_width = 16,
        .max_transfer_bytes = RG_SCREEN_WIDTH * RG_SCREEN_HEIGHT * sizeof(uint16_t),
        .sram_trans_align = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = RG_SCREEN_CS,
        .pclk_hz = RG_SCREEN_CLOCK_HZ,
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
        .lcd_cmd_bits = RG_SCREEN_CMD_BITS,
        .lcd_param_bits = RG_SCREEN_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, io_handle));
}


/**
 * Initialize our LCD panel (GC9306 with i8080 interface).
 *
 * This function configures the ESP32 LCD driver, and initialize
 * the underlying GC9306 controller.
 **/

void lcd_panel_init(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t *panel)
{
    esp_lcd_panel_handle_t panel_handle = NULL;

    /* Install LCD driver */
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = RG_SCREEN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9306(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    *panel = panel_handle;
}


/**
 * RetroGo LCD display APIs.
 **/

static void lcd_send_jitter(void);

/**
 * Configure the LCD screen backlight.
 *
 * Backlight level is given as a percentage, and usually
 * we drive it through PWM. 
 *
 * TODO !
 **/

static void lcd_set_backlight(float percent)
{
    /* TODO */
}


/**
 * LCD vertical synchronization callback.
 *
 * This function is called when RetroGo is done refreshing the
 * screen content. We use this callback to make sure we sent
 * all the pixel data to the current window (if there is some
 * data to be sent).
 **/

static void lcd_sync(void)
{
    /* If some pixels left in our jitter, send to scren. */
    if (g_window.jitter.size > 0)
    {
        //RG_LOGI("Jitter is not empty, flush to screen.");
        lcd_send_jitter();
    }

    /* We are done with the current window, give back the display semaphore. */
    xSemaphoreGive(g_disp_sem);
}


/**
 * Set the LCD drawing window.
 *
 * This function is called whenever RetroGo is about to draw on screen.
 * The provided window (origin X and Y coordinates, window width and height)
 * can be anything and RetroGo usually sets it wide enough and won't send
 * as many pixels as expected based on the window size.
 *
 * More importantly, the i8080 interface requires the driver to reset the
 * drawing window before sending new data. This is why we are using a jitter
 * to store pending pixel data before sending it to screen once a stride
 * complete. The jitter is then flushed and ready to receive new data.
 **/

static void lcd_set_window(int left, int top, int width, int height)
{
    /* We take our display semaphore to avoid retro-go setting a new window 
     * until we are done drawing the previous one.
     */
    lcd_sync();

    if (xSemaphoreTake(g_disp_sem, portMAX_DELAY) == pdTRUE)
    {
        /* Save window in our structure. */
        g_window.x = left;
        g_window.y = top;
        g_window.width = width;
        g_window.height = height;
        g_window.xend = left + width - 1;
        g_window.yend = top + height - 1;
        g_window.pixel_total = width*height;
        g_window.pixel_count = 0;
        g_window.b_last = false;

        /* Compute our jitter size and line numbers. */
        g_window.nrows = (LCD_BUFFER_LENGTH / width);  
        jitter_init(&g_window.jitter, g_window.nrows * width * 2);  
        //RG_LOGD("Jitter initialized with capacity=%d", g_window.jitter.capacity);
    }
    else
    {
        RG_LOGE("Cannot take semaphore ownership for display");
    }
}


/**
 * Return the current drawing buffer.
 *
 * This function always returns the same buffer, as RetroGo calls it
 * before commiting the modified buffer by calling `lcd_send_buffer()`
 * with the updated buffer as parameter, that we retrieve and push into
 * our jitter. Once 'sent', the same buffer can be used by RetroGo
 * to prepare another screen region and send it later.
 **/

static inline uint16_t *lcd_get_buffer(size_t length)
{
    return g_fb;
}


/**
 * Send jitter to LCD and flush.
 *
 * Once fulled (or containing only remaining data to send), the jitter's buffer
 * is sent to the screen regarding the current drawing window and stride. Once
 * sent to the screen (pushed into the display task queue), the buffer is flushed
 * and remaining data is moved into the current jitter's buffer.
 *
 * The screen update operation is asynchronous, except for the configuration of
 * the current display window with `esp_lcd_panel_io_tx_param()` which is synchronous.
 * Once pixel data successfully sent to the LCD panel, our `lcd_trans_done()` callback
 * function is called and gives back our binary semaphore to allow further pixel data
 * to be sent to the LCD panel.
 **/

static void IRAM_ATTR lcd_send_jitter(void)
{
    int x = g_window.x;
    int xmax = g_window.xend;
    int y = g_window.y;
    int yend;
    int lcd_data_size = RG_MIN(g_window.jitter.size, g_window.jitter.capacity);

    /* Send jitter to screen. */
    if (xSemaphoreTake(g_lcd_sem, portMAX_DELAY) == pdTRUE)
    {
        /* Compute the end row for our update window. */
        yend = RG_MIN(g_window.y + g_window.nrows - 1, g_window.yend);

        /* Set drawing window (CASEL/PASEL). */
        esp_lcd_panel_io_tx_param(io_handle, 0x2A, (uint8_t[]){(x>>8)&0xff, x&0xff, (xmax >> 8) & 0xff, xmax & 0xff}, 4);
        esp_lcd_panel_io_tx_param(io_handle, 0x2B, (uint8_t[]){(y>>8)&0xff, y&0xff, (yend >> 8) & 0xff, yend & 0xff}, 4);

        /* Send pixel data. */
        memcpy(g_lcd_trans, g_window.jitter.buffer, lcd_data_size);
        esp_lcd_panel_io_tx_color(io_handle, 0x2C, g_lcd_trans, lcd_data_size);

        /* Update the number of pixels already sent. */
        g_window.pixel_count += lcd_data_size/2;

        /* Update window y. */
        g_window.y += lcd_data_size/(g_window.width*2);

        /* Flush our jitter. */
        jitter_flush(&g_window.jitter);
    }
    else
    {
        RG_LOGW("Could not take mutex ownership :(");
    }
}


/**
 * Send a prepared buffer to screen.
 *
 * RetroGo calls this function to send pixel data to the selected drawing window.
 * Pixel data may not be aligned with the current window width, which is not an
 * issue when an SPI-based screen controller is used (like the ILI9341), but is
 * really one in our case. We need to adapt the SPI LCD interface to a more
 * restrictive i8080 interface that does not accept incomplete data.
 *
 * This function uses a jitter to store the incoming pixel data in a way it
 * can keep the drawing window and the data aligned and send complete stride
 * to the LCD screen in order to avoid graphical glitches or issues.
 **/

static inline void lcd_send_buffer(uint16_t *buffer, size_t length)
{
    int jitter_left = 0;

    /* If length == 0, just exit as we don't have to process this buffer. */
    if (length == 0)
    {
        return;
    }

    /* Add pixel data to our jitter. */
    if (jitter_add(&g_window.jitter, buffer, length*2))
    {
        /* Is our jitter full ? (ready to be sent) */
        if (jitter_full(&g_window.jitter))
        {
            lcd_send_jitter();
        }
        else
        {
            jitter_left = g_window.jitter.size;
        }

        /* Process data left in jitter. */
        if ((g_window.pixel_count + jitter_left/2) == g_window.pixel_total)
        {
            /* Send remaining pixels to screen, if any. */
            if (jitter_left > 0) 
            {
                lcd_send_jitter();
            }

            /* We are done with the current window, give back the display semaphore. */
            xSemaphoreGive(g_disp_sem);
        }
    } 
    else
    {
        RG_LOGE("Jitter has overflowed !");
    }

}


/**
 * Initialize our GC9306 I80 LCD controller.
 *
 * RetroGo calls this function to initialize the LCD driver, its interface and GPIOs.
 **/

static void lcd_init()
{
    memset(g_fb, 0, LCD_FB_SIZE_BYTES);

    /* Create a binary semaphore to marshall I80 pixel write operations. */
    g_lcd_sem = xSemaphoreCreateBinary();
    if (g_lcd_sem == NULL)
    {
        RG_LOGE("Cannot create LCD binary semaphore !");
    }
    xSemaphoreGive(g_lcd_sem);

    /* Create a binary semaphore to marshall window draw operations. */
    g_disp_sem = xSemaphoreCreateBinary();
    if (g_disp_sem == NULL)
    {
        RG_LOGE("Cannot create Display binary semaphore !");
    }
    xSemaphoreGive(g_disp_sem);

    i80_bus_init(&io_handle, NULL);

    esp_lcd_panel_handle_t panel_handle = NULL;
    lcd_panel_init(io_handle, &panel_handle);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    rg_usleep(100*1000);

    /* Clean panel. */
    rg_display_clear(C_BLACK);
    RG_LOGI("init done");
}

/**
 * Deinitialize the LCD screen interface.
 **/

static void lcd_deinit(void)
{
}

/* RetroGo display driver info structure. */
const rg_display_driver_t rg_display_driver_gc9306 = {
    .name = "gc9306",
};


