// Target definition
#define RG_TARGET_NAME             "TSINGTAO"

// Storage
#define RG_STORAGE_ROOT             "/sd"
#define RG_STORAGE_SDSPI_HOST       SPI3_HOST
#define RG_STORAGE_SDSPI_SPEED      SDMMC_FREQ_DEFAULT
// #define RG_STORAGE_SDMMC_HOST       SDMMC_HOST_SLOT_1
// #define RG_STORAGE_SDMMC_SPEED      SDMMC_FREQ_DEFAULT
//#define RG_STORAGE_FLASH_PARTITION  "vfs"

// Audio
#define RG_AUDIO_USE_INT_DAC        0   // 0 = Disable, 1 = GPIO25, 2 = GPIO26, 3 = Both
#define RG_AUDIO_USE_EXT_DAC        1   // 0 = Disable, 1 = Enable

// Video
#define RG_SCREEN_DRIVER            1   // 1 = GC9306
#define RG_SCREEN_BACKLIGHT         1
#define RG_SCREEN_WIDTH             320
#define RG_SCREEN_HEIGHT            240
#define RG_SCREEN_ROTATE            0
#define RG_SCREEN_MARGIN_TOP        0
#define RG_SCREEN_MARGIN_BOTTOM     0
#define RG_SCREEN_MARGIN_LEFT       0
#define RG_SCREEN_MARGIN_RIGHT      0
#define RG_SCREEN_DATA0             GPIO_NUM_12
#define RG_SCREEN_DATA1             GPIO_NUM_13
#define RG_SCREEN_DATA2             GPIO_NUM_14
#define RG_SCREEN_DATA3             GPIO_NUM_15
#define RG_SCREEN_DATA4             GPIO_NUM_16
#define RG_SCREEN_DATA5             GPIO_NUM_21
#define RG_SCREEN_DATA6             GPIO_NUM_5
#define RG_SCREEN_DATA7             GPIO_NUM_4
#define RG_SCREEN_DATA8             GPIO_NUM_17
#define RG_SCREEN_DATA9             GPIO_NUM_18
#define RG_SCREEN_DATA10            GPIO_NUM_10
#define RG_SCREEN_DATA11            GPIO_NUM_11
#define RG_SCREEN_DATA12            GPIO_NUM_19
#define RG_SCREEN_DATA13            GPIO_NUM_20
#define RG_SCREEN_DATA14            GPIO_NUM_2
#define RG_SCREEN_DATA15            GPIO_NUM_1
#define RG_SCREEN_PCLK              GPIO_NUM_7
#define RG_SCREEN_CS                GPIO_NUM_9
#define RG_SCREEN_DC                GPIO_NUM_8
#define RG_SCREEN_RST               GPIO_NUM_6
#define RG_GPIO_LCD_BCKL            GPIO_NUM_40

/* Gamepad (MCP23017) */
#define RG_I2C_DRIVER               2
#define RG_GPIO_I2C_SCL             GPIO_NUM_38
#define RG_GPIO_I2C_SDA             GPIO_NUM_39
#define RG_GAMEPAD_I2C_MAP {\
    {RG_KEY_UP,     (1 << 0)},\
    {RG_KEY_RIGHT,  (1 << 1)},\
    {RG_KEY_DOWN,   (1 << 2)},\
    {RG_KEY_LEFT,   (1 << 3)},\
    {RG_KEY_A,      (1 << 4)},\
    {RG_KEY_B,      (1 << 5)},\
    {RG_KEY_SELECT, (1 << 6)},\
    {RG_KEY_START,  (1 << 8)},\
    {RG_KEY_MENU,   (1 << 9)},\
}

// Battery
#define RG_BATTERY_DRIVER           0

// Status LED
// #define RG_GPIO_LED                 GPIO_NUM_38

#define RG_GPIO_SDSPI_MISO          GPIO_NUM_35
#define RG_GPIO_SDSPI_MOSI          GPIO_NUM_36
#define RG_GPIO_SDSPI_CLK           GPIO_NUM_37
#define RG_GPIO_SDSPI_CS            GPIO_NUM_41

// External I2S DAC
#define RG_GPIO_SND_I2S_BCK         GPIO_NUM_47
#define RG_GPIO_SND_I2S_WS          GPIO_NUM_45 /* Actually, we should not use GPIO 45, PCB v2 uses 48. */
#define RG_GPIO_SND_I2S_DATA        GPIO_NUM_42
// #define RG_GPIO_SND_AMP_ENABLE      18
