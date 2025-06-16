/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_mipi_dsi.h"

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_ops.h"

#define BSP_LCD_BACKLIGHT 21  // default backlight GPIO; adjust if needed
#define BSP_LCD_RST 27

#include "driver/ledc.h"
#include "esp_ldo_regulator.h"

#include <stdint.h>
#include "soc/soc_caps.h"

#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_mipi_dsi.h"

#include "esp_lcd_types.h"
#include "esp_lcd_mipi_dsi.h"
#include "sdkconfig.h"




/* LCD color formats */
#define ESP_LCD_COLOR_FORMAT_RGB565    (1)
#define ESP_LCD_COLOR_FORMAT_RGB888    (2)

/* LCD display color format */
//#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
//#define BSP_LCD_COLOR_FORMAT        (ESP_LCD_COLOR_FORMAT_RGB888)
//#else
#define BSP_LCD_COLOR_FORMAT        (ESP_LCD_COLOR_FORMAT_RGB565)
//#endif
/* LCD display color bytes endianess */
#define BSP_LCD_BIGENDIAN           (0)
/* LCD display color bits */
//#define BSP_LCD_BITS_PER_PIXEL      (24)
#define BSP_LCD_BITS_PER_PIXEL      (16)

/* LCD display color space */
#define BSP_LCD_COLOR_SPACE         (ESP_LCD_COLOR_SPACE_BGR)


/* LCD display definition 1280x800 */
#define BSP_LCD_H_RES              (800)
#define BSP_LCD_V_RES              (20)

#define BSP_LCD_MIPI_DSI_LANE_NUM          (2)    // 2 data lanes
#define BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS (1000) // 1Gbps

#define BSP_MIPI_DSI_PHY_PWR_LDO_CHAN       (3)  // LDO_VO3 is connected to VDD_MIPI_DPHY
#define BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

/**
 * @brief BSP HDMI resolution types
 *
 */
typedef enum {

    BSP_HDMI_RES_1280x800,  /*!< 1280x800@60HZ  */
} bsp_hdmi_resolution_t;

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    bsp_hdmi_resolution_t hdmi_resolution;    /*!< HDMI resolution selection */
    struct {
        mipi_dsi_phy_clock_source_t phy_clk_src;  /*!< DSI bus config - clock source */
        uint32_t lane_bit_rate_mbps;              /*!< DSI bus config - lane bit rate */
    } dsi_bus;
} bsp_display_config_t;

/**
 * @brief BSP display return handles
 *
 */
typedef struct {
    esp_lcd_dsi_bus_handle_t    mipi_dsi_bus;  /*!< MIPI DSI bus handle */
    esp_lcd_panel_io_handle_t   io;            /*!< ESP LCD IO handle */
    esp_lcd_panel_handle_t      panel;         /*!< ESP LCD panel (color) handle */
    esp_lcd_panel_handle_t      control;       /*!< ESP LCD panel (control) handle */
} bsp_lcd_handles_t;

void bsp_display_delete(void);

typedef struct {
    int cmd;                /*<! The specific LCD command */
    const void *data;       /*<! Buffer that holds the command specific data */
    size_t data_bytes;      /*<! Size of `data` in memory, in bytes */
    unsigned int delay_ms;  /*<! Delay in milliseconds after this command */
} ili9881c_lcd_init_cmd_t;

typedef struct {
    const ili9881c_lcd_init_cmd_t *init_cmds;       /*!< Pointer to initialization commands array. Set to NULL if using default commands.
                                                     *   The array should be declared as `static const` and positioned outside the function.
                                                     *   Please refer to `vendor_specific_init_default` in source file.
                                                     */
    uint16_t init_cmds_size;                        /*<! Number of commands in above array */
    struct {
        esp_lcd_dsi_bus_handle_t dsi_bus;               /*!< MIPI-DSI bus configuration */
        const esp_lcd_dpi_panel_config_t *dpi_config;   /*!< MIPI-DPI panel configuration */
        uint8_t  lane_num;                              /*!< Number of MIPI-DSI lanes */
    } mipi_config;
} ili9881c_vendor_config_t;

esp_err_t esp_lcd_new_panel_ili9881c(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                     esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief MIPI-DSI bus configuration structure
 *
 */
#define ILI9881C_PANEL_BUS_DSI_2CH_CONFIG()              \
    {                                                    \
        .bus_id = 0,                                     \
        .num_data_lanes = 2,                             \
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,     \
        .lane_bit_rate_mbps = 1000,                      \
    }

/**
 * @brief MIPI-DBI panel IO configuration structure
 *
 */
#define ILI9881C_PANEL_IO_DBI_CONFIG()  \
    {                                   \
        .virtual_channel = 0,           \
        .lcd_cmd_bits = 8,              \
        .lcd_param_bits = 8,            \
    }



static const char *TAG = "example";

// Handle for MIPI DSI PHY power LDO
static esp_ldo_channel_handle_t disp_phy_pwr_chan = NULL;

void app_main(void)
{
    // Power on MIPI DSI PHY via on-chip LDO
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK( esp_ldo_acquire_channel(&ldo_cfg, &disp_phy_pwr_chan) );
    ESP_LOGI(TAG, "MIPI DSI PHY LDO powered (chan %d @ %dmV)", BSP_MIPI_DSI_PHY_PWR_LDO_CHAN, BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV);
    // Wait a bit for LDO output to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));

    // default full brightness
    uint32_t brightness_percent = 100;

    // configure the timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = 1,
        .freq_hz          = 5 * 1000,  // 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK( ledc_timer_config(&timer_cfg) );

    // configure the channel
    ledc_channel_config_t chan_cfg = {
        .gpio_num     = BSP_LCD_BACKLIGHT,
        .speed_mode   = LEDC_LOW_SPEED_MODE,
        .channel      = 1,
        .intr_type    = LEDC_INTR_DISABLE,
        .timer_sel    = 1,
        .duty         = 0,
        .hpoint       = 0
    };
    ESP_ERROR_CHECK( ledc_channel_config(&chan_cfg) );

    // then later, to set brightness:
    uint32_t duty = (1023 * brightness_percent) / 100;
    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, duty) );
    ESP_ERROR_CHECK( ledc_update_duty(LEDC_LOW_SPEED_MODE, 1) );

    ESP_LOGI("example", "Cycling solid-color screens");

    // Create display and retrieve all handles
    bsp_display_config_t cfg = {
        .hdmi_resolution = BSP_HDMI_RES_1280x800,
        .dsi_bus = {
            .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
            .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
        }
    };

    // Log panel native parameters from BSP macros
    ESP_LOGI(TAG, "Panel resolution: %ux%u", BSP_LCD_H_RES, BSP_LCD_V_RES);
    ESP_LOGI(TAG, "Bits per pixel: %d", BSP_LCD_BITS_PER_PIXEL);
    ESP_LOGI(TAG, "Color format: %s", (BSP_LCD_COLOR_FORMAT == ESP_LCD_COLOR_FORMAT_RGB888) ? "RGB888" : "RGB565");
    ESP_LOGI(TAG, "Color space: %s", (BSP_LCD_COLOR_SPACE == ESP_LCD_COLOR_SPACE_RGB) ? "RGB" : "BGR");
    ESP_LOGI(TAG, "MIPI DSI data lanes: %d", BSP_LCD_MIPI_DSI_LANE_NUM);
    ESP_LOGI(TAG, "MIPI DSI max lane bitrate: %u Mbps", BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS);

    // Log current DSI config before override
    ESP_LOGI(TAG, "DSI lane bit rate before override: %u Mbps", cfg.dsi_bus.lane_bit_rate_mbps);
    // Override the DSI lane bit rate
    uint32_t custom_bitrate = 600;
    cfg.dsi_bus.lane_bit_rate_mbps = custom_bitrate;
    ESP_LOGI(TAG, "DSI lane bit rate after override: %u Mbps", cfg.dsi_bus.lane_bit_rate_mbps);

//    bsp_display_backlight_on();

    // --- manual DSI + DPI panel init (in place of BSP helper) ---
    // 1) Create DSI bus
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = BSP_LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = cfg.dsi_bus.lane_bit_rate_mbps,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    // 2) Install DBI IO
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,
        .lcd_param_bits  = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io));

    // 3) Inline DPI config (expanded from ILI9881C_800_1280_PANEL_60HZ_DPI_CONFIG)
    esp_lcd_dpi_panel_config_t dpi_config =
    {                                                      \
            .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT, \
            .dpi_clock_freq_mhz = 60,                           \
            .virtual_channel    = 0,                            \
            .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565,                    \
            .num_fbs            = 1,                            \
            .video_timing = {                                   \
                .h_size             = BSP_LCD_H_RES,            \
                .v_size             = BSP_LCD_V_RES,            \
                .hsync_back_porch   = 172,                      \
                .hsync_pulse_width  = 16,                       \
                .hsync_front_porch  = 32,                       \
                .vsync_back_porch   = 40,                       \
                .vsync_pulse_width  = 4,                        \
                .vsync_front_porch  = 26,                       \
            },                                                  \
            .flags.use_dma2d      = true,                      \
        };


    // 4) Vendor and panel dev config
    ili9881c_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus    = mipi_dsi_bus,
            .dpi_config = &dpi_config,
            .lane_num   = BSP_LCD_MIPI_DSI_LANE_NUM,
        },
    };
    esp_lcd_panel_dev_config_t panel_dev_config = {
        .reset_gpio_num  = GPIO_NUM_NC, // No reset GPIO used
        .rgb_ele_order   = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel  = BSP_LCD_BITS_PER_PIXEL,
        .vendor_config   = &vendor_config,
    };

    // 5) Create, reset, init and turn on the ILI9881C panel
    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(io, &panel_dev_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    // --- end manual init ---

    // Now 'panel' holds your configured DPI panel handle.

    // Get single frame buffer
    void *fb0 = NULL;
    // Compute and log the actual frame buffer size
    size_t fb_size = BSP_LCD_H_RES * BSP_LCD_V_RES * (BSP_LCD_BITS_PER_PIXEL / 8);
    ESP_LOGI(TAG, "Allocating frame buffer: %u bytes", (unsigned)fb_size);
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &fb0));
    ESP_LOGI(TAG, "Frame buffer at %p, size: %u bytes", fb0, (unsigned)fb_size);
    uint16_t *pixels = (uint16_t*)fb0;
    const int width = BSP_LCD_H_RES;
    const int height = BSP_LCD_V_RES;

    // Define rotation colors: white, red, green, blue
    uint16_t colors[] = { 0xFFFF, 0xF800, 0x07E0, 0x001F };
    size_t color_count = sizeof(colors) / sizeof(colors[0]);
    size_t color_index = 0;

    while (1) {
        uint16_t color = colors[color_index];
        // Draw 50×50 checkerboard: black squares on color background
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (((x / 50) + (y / 50)) % 2) {
                    pixels[y * width + x] = 0x0000; // black square
                } else {
                    pixels[y * width + x] = color; // background color
                }
            }
        }
        // Push entire frame buffer to the display
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, width, height, fb0));
        // Wait before rotating to the next color
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Advance to next color
        color_index = (color_index + 1) % color_count;
    }
}
