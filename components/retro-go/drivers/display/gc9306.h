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
#include "rick.h"
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
#define LCD_FB_MAX 10
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
    uint16_t *p_buffer;
    bool b_used;
} framebuffer_t;

/* Array of framebuffers. */
static framebuffer_t framebuffers[LCD_FB_MAX];

/* FBs ringbuffer. */
static uint8_t pending_fbs[LCD_RB_MAX];
static int pfbs_head, pfbs_tail;

/* FB semaphores. */
static SemaphoreHandle_t g_fb_sem = NULL;
static SemaphoreHandle_t g_fb_mutex = NULL;
static SemaphoreHandle_t g_rb_mutex = NULL;

static uint16_t g_framebuffer[LCD_BUFFER_LENGTH];


/***
 * Framebuffer ring-buffer
 **/

bool fb_ringbuf_init(){
	/* Initialize our ringbuffer mutex. */
	g_rb_mutex = xSemaphoreCreateMutex();
	if (g_rb_mutex == NULL)
	{
		ESP_LOGE(TAG, "cannot create mutex for FB ringbuf");
		return false;
	}
	else
	{
		/* Initialize our ringbuf structure. */
		memset(pending_fbs, 0xff, LCD_FB_MAX);
		pfbs_head = 0;
		pfbs_tail = 0;
	}

	/* Success. */
	return true;
}

bool fb_ringbuf_insert(int fb)
{
	/* Do we have enough space ? */
	if (((pfbs_tail + 1) % LCD_RB_MAX) == pfbs_head)
	{
		ESP_LOGD(TAG, "ringbuffer is full !");
		return false;
	}
	else
	{
		if (xSemaphoreTake(g_rb_mutex, portMAX_DELAY) == pdTRUE)
		{
			pending_fbs[pfbs_tail] = fb;
			pfbs_tail = (pfbs_tail + 1)%LCD_RB_MAX;

			/* Release mutex. */
			xSemaphoreGive(g_rb_mutex);
		}
		else
		{
			ESP_LOGE(TAG, "cannot acquire FB ringbuf mutex !");
			return false;
		}
	}

	/* Success. */
	return true;
}

int fb_ringbuf_pop()
{
	int fb = -1;

	if (pfbs_head == pfbs_tail)
	{
		ESP_DRAM_LOGD(TAG, "ringbuffer is empty !");
	}
	else
	{
		fb = pending_fbs[pfbs_head];
		pfbs_head = (pfbs_head + 1)%LCD_RB_MAX;
	}

	return fb;
}

/***
 * Framebuffer marshall
 **/

/**
 * Find a unused framebuffer, wait for one if no resource available.
 *
 * :return: Framebuffer index, -1 if an error occured.
 * :rtype: int
 **/

int fb_get_free(void)
{
  int fb = -1;

	/* Enter critical section. */
  if (xSemaphoreTake(g_fb_mutex, portMAX_DELAY) == pdTRUE)
	{
		if (xSemaphoreTake(g_fb_sem, portMAX_DELAY) == pdTRUE)
		{
				/* Find the first unused framebuffer. */
				for (int i=0; i<LCD_FB_MAX; i++)
				{
					if (!framebuffers[i].b_used)
					{
						/*
							Mark this framebuffer used, return the corresponding
							index.
						*/
						framebuffers[i].b_used = true;
						fb = i;
						break;
					}
				}
		}
		else
		{
			ESP_LOGE(TAG, "fb_get_free(): cannot retrieve a free framebuffer (xSemaphoreTake() returned FALSE)\n");
		}

		/* Exit critical section. */
		xSemaphoreGive(g_fb_mutex);
	}
  else
  {
		ESP_LOGE(TAG, "fd_get_free(): cannot acquire framebuffer marshall mutex !\n");
	}

	return fb;
}

bool fb_recycle_from_isr(int fb_id, BaseType_t *xHigherPriorityTaskWoken)
{
	xHigherPriorityTaskWoken = pdFALSE;

	if ((fb_id >= 0) && (fb_id < LCD_FB_MAX))
	{	
			/* Mark the given framebuffer as unused. */
			framebuffers[fb_id].b_used = false;

			/* Give back this framebuffer. */
			if (xSemaphoreGiveFromISR(g_fb_sem, xHigherPriorityTaskWoken) == pdTRUE)
			{
				return true;
			}

			/* Success. */
			return false;
  }
	else
	{
		ESP_DRAM_LOGE(TAG, "fb_recycle_from_isr(): invalid framebuffer id (%d)", fb_id);

		/* Failure. */
		return false;
	}
}

bool fb_recycle(int fb_id)
{
	if ((fb_id >= 0) && (fb_id < LCD_FB_MAX))
	{	
			/* Mark the given framebuffer as unused. */
			framebuffers[fb_id].b_used = false;

			/* Give back this framebuffer. */
			if (xSemaphoreGive(g_fb_sem) == pdTRUE)
			{
				ESP_LOGI(TAG, "fb_recycle(): framebuffer %d has been recycled !", fb_id);
			}

			/* Success. */
			return true;
  }
	else
	{
		ESP_LOGE(TAG, "fb_recycle(): invalid framebuffer id (%d)", fb_id);

		/* Failure. */
		return false;
	}
}

int fb_find(uint16_t *p_buffer)
{
	if (p_buffer != NULL)
	{
      for (int i=0; i<LCD_FB_MAX; i++)
      {
          if (framebuffers[i].p_buffer == p_buffer)
              return i;
      }
  }

  /* Failure. */
  return -1;
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
    RG_LOGI("new gc9306 panel @%p", gc9306);

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
    RG_LOGI("del gc9306 panel @%p", gc9306);
    free(gc9306);
    return ESP_OK;
}

static esp_err_t panel_gc9306_reset(esp_lcd_panel_t *panel)
{
    gc9306_panel_t *gc9306 = __containerof(panel, gc9306_panel_t, base);
    esp_lcd_panel_io_handle_t io = gc9306->io;

    RG_LOGI("reset gc9306 panel");

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

    RG_LOGI("init gc9306 panel ...");

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
	
	//ESP_DRAM_LOGI(TAG, "lcd_trans_done(): framebuffer sent to screen, recycle current fb");

	/* Extract first framebuffer from ringbuf. */
	fb = fb_ringbuf_pop();
	if (fb < 0)
	{
		//ESP_DRAM_LOGD(TAG, "lcd_trans_done(): no data in ringbuffer !");
	}
	else
	{
		//ESP_DRAM_LOGI(TAG, "lcd_trans_done(): recycling framebuffer %d ...", fb);

		/* Recycle framebuffer. */
		fb_recycle_from_isr(fb, &hptask_woken);

	}

	return hptask_woken;
}

void example_init_i80_bus(esp_lcd_panel_io_handle_t *io_handle, void *user_ctx)
{
    RG_LOGI("Initialize Intel 8080 bus");
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

void panel_fill(esp_lcd_panel_io_handle_t io, unsigned int x, unsigned int y, unsigned int w, unsigned int h, uint16_t color)
{
	int i;
	uint32_t pixels_count;
	int fb = -1;
	uint16_t *p_pixels = NULL;
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
		//ESP_LOGI(TAG, "panel_fill(): preparing to send pixels from row %d to %d", y, y+nrows);

		/* Compute last row index. */
		yend = y + nrows - 1;
		//ESP_LOGD(TAG, "yend=%d (y+nrows)", yend);

		/* Allocate a framebuffer. */
		//ESP_LOGD(TAG, "allocating a framebuffer ...");
		fb = fb_get_free();
		if (fb >= 0)
		{
			//ESP_LOGD(TAG, "framebuffer allocated (%d, %p), fill with color (%d pixels)...", fb, framebuffers[fb].p_buffer, nrows*w);

			/* Fill framebuffer with our color. */
			p_pixels = framebuffers[fb].p_buffer;
			pixels_count = nrows*w;
			for (i=0; i<pixels_count; i++)
			{
				p_pixels[i] = color;
			}

			/* Send to LCD. */
			//ESP_LOGD(TAG, "Send CASEL/PASEL to LCD driver: (%d,%d) - (%d,%d)", x,y, xmax,yend);
			esp_lcd_panel_io_tx_param(io, 0x2A, (uint8_t[]){(x>>8)&0xff, x&0xff, (xmax >> 8) & 0xff, xmax & 0xff}, 4);
			esp_lcd_panel_io_tx_param(io, 0x2B, (uint8_t[]){(y>>8)&0xff, y&0xff, (yend >> 8) & 0xff, yend & 0xff}, 4);

			/* Send colors. */
			//ESP_LOGD(TAG, "Sending pixels (%lu bytes) ...", pixels_count*2);
			fb_ringbuf_insert(fb);
			esp_lcd_panel_io_tx_color(io, 0x2C, p_pixels , pixels_count*2);

			/* Process next slice. */
			//ESP_LOGD(TAG, "moving y from %d to %d", y, y+nrows);
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
		else
		{
			//ESP_LOGE(TAG, "panel_fill(): unexpected error while requesting a framebuffer.");
			break;
		}
	}
}

void panel_blit(esp_lcd_panel_io_handle_t io, unsigned int x, unsigned int y, unsigned int w, unsigned int h, uint16_t* p_buffer, unsigned int bufsize)
{
	int i;
	uint32_t pixels_count;
	int fb = -1;
	uint16_t *p_pixels = NULL;
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
		//ESP_LOGI(TAG, "panel_fill(): preparing to send pixels from row %d to %d", y, y+nrows);

		/* Compute last row index. */
		yend = y + nrows - 1;
		//ESP_LOGD(TAG, "yend=%d (y+nrows)", yend);

		/* Allocate a framebuffer. */
		//ESP_LOGD(TAG, "allocating a framebuffer ...");
		fb = fb_get_free();
		if (fb >= 0)
		{
			//ESP_LOGD(TAG, "framebuffer allocated (%d, %p), fill with color (%d pixels)...", fb, framebuffers[fb].p_buffer, nrows*w);

			/* Fill framebuffer with pixels from buffer. */
			p_pixels = framebuffers[fb].p_buffer;
			pixels_count = nrows*w;
			memcpy(framebuffers[fb].p_buffer, (void *)&p_buffer[y*w], pixels_count*2); 

			/* Send to LCD. */
			//ESP_LOGD(TAG, "Send CASEL/PASEL to LCD driver: (%d,%d) - (%d,%d)", x,y, xmax,yend);
			esp_lcd_panel_io_tx_param(io, 0x2A, (uint8_t[]){(x>>8)&0xff, x&0xff, (xmax >> 8) & 0xff, xmax & 0xff}, 4);
			esp_lcd_panel_io_tx_param(io, 0x2B, (uint8_t[]){(y>>8)&0xff, y&0xff, (yend >> 8) & 0xff, yend & 0xff}, 4);

			/* Send colors. */
			//ESP_LOGD(TAG, "Sending pixels (%lu bytes) ...", pixels_count*2);
			fb_ringbuf_insert(fb);
			esp_lcd_panel_io_tx_color(io, 0x2C, p_pixels , pixels_count*2);

			/* Process next slice. */
			//ESP_LOGD(TAG, "moving y from %d to %d", y, y+nrows);
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
		else
		{
			//ESP_LOGE(TAG, "panel_fill(): unexpected error while requesting a framebuffer.");
			break;
		}
	}
}


void example_init_lcd_panel(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t *panel)
{
    esp_lcd_panel_handle_t panel_handle = NULL;

    /* Initialize our framebuffers. */
		memset(framebuffers, 0, LCD_FB_MAX*sizeof(framebuffer_t));
    for (int fb = 0; fb < LCD_FB_MAX; fb++)
    {
        /* Allocate a framebuffer in PSRAM and save it into our structure. */ 
        framebuffers[fb].p_buffer = (uint16_t *)malloc(LCD_FB_SIZE_BYTES);
        if (framebuffers[fb].p_buffer == NULL)
        {
            ESP_LOGE(TAG, "Memory error: cannot allocate %d bytes for framebuffer %d", LCD_FB_SIZE_BYTES, fb);
            framebuffers[fb].b_used = true;
        }
        else
        {
            framebuffers[fb].b_used = false;
        }
    }

		/* Create our counting semaphore used to marshall LCB framebuffers. */
		g_fb_sem = xSemaphoreCreateCounting(LCD_FB_MAX, LCD_FB_MAX);
	  if (g_fb_sem == NULL)
		{
			ESP_LOGE(TAG, "Cannot create counting semaphore (LCD framebuffers marshall) !");
		}
		g_fb_mutex = xSemaphoreCreateMutex();
		if (g_fb_mutex == NULL)
		{
			ESP_LOGE(TAG, "Cannot create mutex (LCD framebuffers marshall) !");
		}

		/* Initialize our ringbuffer. */
		fb_ringbuf_init();

    /* Install LCD driver */
    ESP_LOGI(TAG, "Install LCD driver of GC9306");
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
    int right = left + width - 1;
    int bottom = top + height - 1;

    if (left < 0 || top < 0 || right >= display.screen.real_width || bottom >= display.screen.real_height)
        RG_LOGW("Bad lcd window (x0=%d, y0=%d, x1=%d, y1=%d)\n", left, top, right, bottom);

    /* Set CASET and RASET. */
    esp_lcd_panel_io_tx_param(io_handle, 0x2A, (uint8_t[]){(left>>8)&0xff, left&0xff, (right >> 8) & 0xff, right & 0xff}, 4);
    esp_lcd_panel_io_tx_param(io_handle, 0x2B, (uint8_t[]){(top>>8)&0xff, top&0xff, (bottom >> 8) & 0xff, bottom & 0xff}, 4);
}


static inline uint16_t *lcd_get_buffer(size_t length)
{
    int fb;

    /* Allocate a framebuffer. */
    fb = fb_get_free();
    if (fb < 0)
    {
        RG_LOGW("gc9306: framebuffer allocation failed !");
        return NULL;
    }
    
    return framebuffers[fb].p_buffer;
}


static inline void lcd_send_buffer(uint16_t *buffer, size_t length)
{
    int fb;

    /* Find the corresponding framebuffer id. */
    fb = fb_find(buffer);
    if (fb < 0) {
        RG_LOGE("Cannot identify the provided framebuffer: %p", buffer);
    }
    else
    {
        if (length > 0)
        {
            /* Save framebuffer id into our FB ring buffer. */
            fb_ringbuf_insert(fb);

            /* Send colors. */
            esp_lcd_panel_io_tx_color(io_handle, 0x2C, buffer , length);
        }
        else
        {
            fb_recycle(fb);
        }
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
    RG_LOGI("initializing I8080 bus ...");
    example_init_i80_bus(&io_handle, NULL);

    RG_LOGI("initializing LCD GC9306 panel ...");
    esp_lcd_panel_handle_t panel_handle = NULL;
    example_init_lcd_panel(io_handle, &panel_handle);

    RG_LOGI("enabling lcd ...");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    /* Clean panel. */
    RG_LOGI("filling with rick ...");
    //panel_fill(io_handle, 0, 0, 240, 320, 0x0000);
    //panel_blit(io_handle, 0, 0, 240, 314, rick, 75360);
    rg_display_clear(C_BLACK);
    rg_usleep(100 * 1000);
}

static void lcd_deinit(void)
{
    // TODO
}

const rg_display_driver_t rg_display_driver_gc9306 = {
    .name = "gc9306",
};


