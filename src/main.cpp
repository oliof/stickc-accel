#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include <esp_http_server.h>
#include "M5Unified.hpp"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "mdns.h"
#include "i2c-base.hpp"
#include "fft.h"

#define IMU_ADDR 0x68
#define AXP_ADDR 0x34
#define BTN_A GPIO_NUM_37
#define BTN_B GPIO_NUM_39
#define IMU_INT GPIO_NUM_35
esp_err_t get_handler(httpd_req_t *req) {
    const char resp[] = "/ Response";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void filterAccelData(float *accdata, uint32_t length) {
    // Define filter coefficients
    float Q = 10.0f;
    #define FILTER_ORDER 2
    #define FILTER_FC1 10.0 // Low cut-off frequency in Hz
    #define FILTER_FC2 150.0 // High cut-off frequency in Hz
    #define FILTER_FS 1000.0 // Sampling frequency in Hz

    float PI = std::atan(1.0)*4;
    // Initialize filter coefficients
    float wc1 = 2.0 * PI * FILTER_FC1 / FILTER_FS;
    float wc2 = 2.0 * PI * FILTER_FC2 / FILTER_FS;
    float alpha1 = sin(wc1) / (2.0 * Q);
    float alpha2 = sin(wc2) / (2.0 * Q);
    float b[FILTER_ORDER+1] = { alpha2, 0.0, -alpha2 };
    float a[FILTER_ORDER+1] = { 1.0f + alpha1 + alpha2, -2.0f*cos(wc1), 1.0f - alpha1 - alpha2 };
    float x[FILTER_ORDER+1] = { 0.0, 0.0, 0.0 };
    float y[FILTER_ORDER+1] = { 0.0, 0.0, 0.0 };

    // Apply the filter to the accelerometer data
    for (int i = 0; i < length; i++) {
        // Shift data into the filter input buffer
        x[0] = x[1];
        x[1] = x[2];
        x[2] = accdata[i];
    
        // Compute filter output
        y[0] = y[1];
        y[1] = y[2];
        y[2] = (b[0]*x[0] + b[1]*x[1] + b[2]*x[2]
                - a[1]*y[0] - a[2]*y[1]) / a[0];
    
        // Store filtered data back into the accelerometer data array
        accdata[i] = y[2];
    }
}

void dumpImuConfig() {
    uint8_t wm[2];
    uint16_t fifocount;
    auto err = i2c_read(IMU_ADDR, 0x72, (uint8_t *) &fifocount, 2);
    if (err != ESP_OK) {
        printf("err reading fifocount 0x%04x\n", err);
    }
    err = i2c_read(IMU_ADDR, 0x60, wm, 2);
    if (err != ESP_OK) {
        printf("err reading watermark 0x%04x\n", err);
    }
    vTaskDelay(1);
    printf("fc=%d\n", ntohs(fifocount));
    printf("wm=%d\n", wm[0] << 8 | wm[1]);
    uint8_t data;
    I2C_ERR_CHECK(i2c_read8(IMU_ADDR, 0x38, &data), "failed to read int_en: 0x%04x\n");
    printf("int_en=0x%02x\n", data);
    vTaskDelay(1);
    I2C_ERR_CHECK(i2c_read8(IMU_ADDR, 0x23, &data), "failed to read fifo_en: 0x%04x\n");
    printf("fifo_en=0x%02x\n", data);
    vTaskDelay(1);
    I2C_ERR_CHECK(i2c_read8(IMU_ADDR, 0x6A, &data), "failed to read user_ctrl: 0x%04x\n");
    printf("user_ctl=0x%02x\n", &data);
    vTaskDelay(1);
    uint8_t cfg[4];
    err = i2c_read(IMU_ADDR, 0x1A, cfg, 4);
    if (err != ESP_OK) {
        printf("err reading cfg 0x%04x\n", err);
    }
    printf("cfg=0x%02x\n", cfg[0]);
    printf("gyro cfg=0x%02x\n", cfg[1]);
    printf("accel cfg=0x%02x\n", cfg[2]);
    printf("accel cfg2=0x%02x\n", cfg[3]);
}

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_handler,
    .user_ctx = NULL
};

#define EXAMPLE_ESP_WIFI_SSID      "AN_SSID_HERE"
#define EXAMPLE_ESP_WIFI_PASS      "A_WIFI_PW_HERE"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

StaticSemaphore_t screen_toggle_buffer;
StaticSemaphore_t imu_fifo_sync_buffer;
SemaphoreHandle_t fifo_sync;
SemaphoreHandle_t screen_toggle;

// M5Unified uses their own i2c driver that conflicts with native i2c
StaticSemaphore_t i2c_exclusive_buffer;
SemaphoreHandle_t i2c_exclusive;

void add_mdns_services()
{
    //add our services
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    //NOTE: services must be added before their properties can be set
    //use custom instance for the web server
    mdns_service_instance_name_set("_http", "_tcp", "Accelerometer Data Web Server");

}
void start_mdns_service()
{
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    //set hostname
    mdns_hostname_set("stickc");
    //set default instance
    mdns_instance_name_set("M5 StickC Accelerometer");
    add_mdns_services();
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        start_mdns_service();
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
extern "C" {
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = { 'H', 'a', 'n', 'H', 'u', 'y', ' ', 'P', 'l', 'u', 's',  0},
            .password = { 'p', 'f', 'n', 'g', 'u', 'y', 'e', 'n', 0 },
            // .ssid = EXAMPLE_ESP_WIFI_SSID,
            // .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold = { .authmode = WIFI_AUTH_WPA2_PSK },

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
}
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

uint8_t getBatteryLevel() {
    std::uint8_t buf[4];
    auto err = i2c_read(AXP_ADDR, 0x78, buf, 4);
    if (err != ESP_OK) 
    {
        printf("err reading battery 0x%04x\n", err);
        return -1;
    }

    std::uint_fast16_t voltage = (buf[0] << 4) + buf[1];
    std::uint_fast16_t current = (buf[2] << 5) + buf[3];

    std::int_fast16_t res = (voltage > 3150) ? (( voltage - 3075 ) * 0.16f  )
                          : (voltage > 2690) ? (( voltage - 2690 ) * 0.027f )
                          : 0;
    if (current > 16) { res -= 16; }

    return (res < 100) ? res : 100;
}
bool imuInitialized = false;
float ax, ay, az;
uint8_t watermarkStatus;
uint8_t intStatus;
bool screenOn = true;
void screenTask(void *pvParams) {
    printf("Screen entry\n");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(1000 / 30);
    M5.Displays(0).setTextSize(1);
    M5Canvas canvas;
    auto depth = M5.Displays(0).getColorDepth();
    canvas.setColorDepth(depth);
    int32_t width = M5.Displays(0).width();
    int32_t height = M5.Displays(0).height();
    canvas.createSprite(width, height);
    auto palette = canvas.getPalette();
    // M5.Displays(0).sleep();
    // M5.Displays(0).setBrightness(0);
    for (;;) {
        static int on = 0;
        on = !on;

        if (screenOn) {
            canvas.setCursor(0, 0);
            canvas.fillRect(0, 0, 80, 80);
            canvas.printf("No.%d\n", on);
            if (imuInitialized) {
                canvas.printf("x = %f\n", ax);
                canvas.printf("y = %f\n", ay);
                canvas.printf("z = %f\n", az);
                canvas.printf("wm = 0x%02x\nint = 0x%02x\n", watermarkStatus, intStatus);
            } else {
                canvas.printf("No IMU\n");
            }
            canvas.fillRect(30, 140, 50, 20);
            canvas.setCursor(30, 140);
            // canvas.printf("%3d%%\n", M5.Power.getBatteryLevel());
            canvas.printf("%3d%%\n", getBatteryLevel());
            M5.Displays(0).pushImageDMA(0, 0, width, height, canvas.getBuffer(), depth, palette);
        }

        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

bool pwrButtonClicked() {

    while (!xSemaphoreTake(i2c_exclusive, pdMS_TO_TICKS(1000)));
    uint8_t val;
    I2C_ERR_CHECK(i2c_read8(AXP_ADDR, 0x46, &val), "failed to read pwr button: 0x%04x\n");
    xSemaphoreGive(i2c_exclusive);
    if (val & 0x03) { i2c_write8(AXP_ADDR, 0x46, val); }
    return val == 2;
}

void loopTask(void *pvParams) {
    printf("Loop entry\n");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(100);
    for (;;) {
        if (pwrButtonClicked()) {
            ESP_LOGI(TAG, "Restart");
            esp_restart();
        }
        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

bool need_fifo_sync = false;
uint8_t fifobuffer[1024];
uint16_t *fifodata = (uint16_t *)fifobuffer;
void IRAM_ATTR imuFifoIsr(void *args) {
    xSemaphoreGive(fifo_sync);
    need_fifo_sync = true;
    gpio_intr_disable(IMU_INT);
}

void imuInit() {
    static bool initializing = false;

    if (initializing) {
        return;
    }

    uint8_t imuWhoami;
    I2C_ERR_CHECK(i2c_read8(IMU_ADDR, 0x75, &imuWhoami), "failed to read imu whoami: 0x%04x\n");
    if (imuWhoami != 25) {
        printf("MPU6886 not detected\n");
        return;
    }
    vTaskDelay(1);
    initializing = true;

    // PWR_MGMT_1(0x6B)
    i2c_write8(IMU_ADDR, 0x6B, 0x00);
    vTaskDelay(10);

    // PWR_MGMT_1(0x6B)
    i2c_write8(IMU_ADDR, 0x6B, 0x80);
    vTaskDelay(10);

    // PWR_MGMT_1(0x6B)
    i2c_write8(IMU_ADDR, 0x6B, 0x01);
    vTaskDelay(10);

    static constexpr std::uint8_t init_cmd[] =
    { 0x1C, 0x10  // ACCEL_CONFIG(0x1C) : +-8G
    , 0x6C, 0x07  // PWR_MGMT_2, 0x07 disable gyro
    // , 0x6B, 0x01 | 0x04 // PWR_MGMT no temp
    , 0x1B, 0x18  // GYRO_CONFIG(0x1B) : +-2000dps
    , 0x23, 0x08  // FIFO_EN(0x23)
    , 0x1A, 0x41  // CONFIG(0x1A)
    , 0x19, 0x00  // SMPLRT_DIV(0x19)
    , 0x38, 0x00  // INT_ENABLE(0x38)
    , 0x1D, 0x00  // ACCEL_CONFIG 2(0x1D)
    , 0x37, 0x80 | 0x20 | 0x10 | 0x02  // INT_PIN_CFG(0x37)
    , 0x60, (uint8_t)(750 >> 8)  // WATERMARK HIGH
    , 0x61, (uint8_t)(750 & 0xff)  // WATERMARK HIGH
    , 0x1A, 0x41  // CONFIG(0x1A)
    , 0x38, 0x10  // INT_ENABLE(0x38)
    , 0x6A, 0x44  // USER_CTRL(0x6A)
    , 0x23, 0x08  // FIFO_EN(0x23)
    , 0xFF, 0xFF  // EOF
    };

    for (int idx = -1;;)
    {
      std::uint8_t reg = init_cmd[++idx];
      std::uint8_t val = init_cmd[++idx];
      if ((reg & val) == 0xFF) { break; }
      auto err = i2c_write8(IMU_ADDR, reg, val);
      if (err != ESP_OK) {
        printf("Failed to write register 0x%02x -- 0x%04x\n", reg, err);
        idx -= 2;
      }
      vTaskDelay(1);
    }
    printf("Configured MPU6886\n");
    imuInitialized = true;
    initializing = false;
    dumpImuConfig();
}

void imuSyncTask(void *pvParams) {
    printf("Sync entry\n");
    TickType_t xFrequency = pdMS_TO_TICKS(1000);
    static constexpr float aRes = 8.0f / 32768.0f;
    for (;;) {
        if (!xSemaphoreTake(i2c_exclusive, xFrequency)) {
            printf("i2c exclusive timed out???\n");
        }
        if (need_fifo_sync || xSemaphoreTake(fifo_sync, xFrequency)) {
            if (!imuInitialized) {
                imuInit();
                esp_task_wdt_reset();
                vTaskDelay(1000);
                goto give;
            }
            // read, should have 0x40
            I2C_ERR_CHECK(i2c_read8(IMU_ADDR, 0x39, &watermarkStatus), "failed to read watermark status: 0x%04x\n");
            // read 0x10 if fifo overflow
            I2C_ERR_CHECK(i2c_read8(IMU_ADDR, 0x3A, &intStatus), "failed to read int status: 0x%04x\n");
            if ((intStatus & 0x01) != 0) {
                static uint8_t adata[6];
                auto err = i2c_read(IMU_ADDR, 0x3B, adata, 6);
                if (err != ESP_OK) {
                    printf("err reading accel 0x%04x\n", err);
                }
                ax = (std::int16_t)((adata[0] << 8) + adata[1]) * aRes;
                ay = (std::int16_t)((adata[2] << 8) + adata[3]) * aRes;
                az = (std::int16_t)((adata[4] << 8) + adata[5]) * aRes;
                goto give;
            }
            if ((intStatus & 0x10) != 0) {
                printf("FIFO overflow!\n");
            }
            if ((watermarkStatus & 0x40) == 0) {
                goto give;
            }
            // printf("Interrupt: wm=0x%02x int=0x%02x\n", watermarkStatus, intStatus);
            uint8_t fc[2];
            auto err = i2c_read(IMU_ADDR, 0x72, fc, 2);
            if (err != ESP_OK) {
                printf("err reading fifocount 0x%04x\n", err);
            }
            uint16_t fifocount = (fc[0] << 8) | fc[1];
            if (fifocount && fifocount < 1025) {
                uint16_t fiforem = fifocount;
                uint16_t offset = 0;
                uint16_t readsize = 210;
                bool haserrors = false;
                while (fiforem > 0) {
                    uint16_t toread = std::min(readsize, fiforem);
                    err = i2c_read(IMU_ADDR, 0x74, fifobuffer + offset, toread);
                    if (err != ESP_OK) {
                        printf("Failed to read FIFO at %d 0x%04x\n", offset, err);
                        haserrors = true;
                    }
                    offset += toread;
                    fiforem -= toread;
                }
                if (haserrors) {
                    auto err = i2c_read(IMU_ADDR, 0x72, fc, 2);
                    if (err != ESP_OK) {
                        printf("err reading fifocount2 0x%04x\n", err);
                    }
                    fifocount = (fc[0] << 8) | fc[1];
                    printf("remaining after errors=%d\n", fifocount);
                }

                printf("cnt=%d, x=%f, y=%f, z=%f, t=%f\n", fifocount,
                    (int16_t)ntohs(fifodata[4]) * aRes,
                    (int16_t)ntohs(fifodata[5]) * aRes,
                    (int16_t)ntohs(fifodata[6]) * aRes,
                    25.0f + ntohs(fifodata[3]) / 326.8f);

                ax = (int16_t)ntohs(fifodata[4]) * aRes;
                ay = (int16_t)ntohs(fifodata[5]) * aRes;
                az = (int16_t)ntohs(fifodata[6]) * aRes;
            } else if (fifocount > 1024) {
                imuInitialized = false;
                imuInit();
                vTaskDelay(1000);
                goto give;
            }
            uint8_t level = gpio_get_level(IMU_INT);
            if (!level && imuInitialized) {
                printf("Interrupt: wm=0x%02x int=0x%02x\n", watermarkStatus, intStatus);
                printf("INT still active, reading again\n");
                esp_task_wdt_reset();
                goto give;
            }

            gpio_intr_enable(IMU_INT);
            ESP_ERROR_CHECK(gpio_set_intr_type(IMU_INT, GPIO_INTR_LOW_LEVEL));
            need_fifo_sync = false;
        } else {
            if (!imuInitialized) {
                // printf("Skipping, IMU not initialized\n");
                imuInit();
                vTaskDelay(1000);
                esp_task_wdt_reset();
                goto give;
            }
            printf("No IMU INT received\n");
            uint8_t level = gpio_get_level(IMU_INT);
            // printf("gpio35=%d\n", level);
            dumpImuConfig();
            if (!level) {
                printf("Manual SYNC!\n");
                xSemaphoreGive(fifo_sync);
            }
            gpio_intr_enable(IMU_INT);
            ESP_ERROR_CHECK(gpio_set_intr_type(IMU_INT, GPIO_INTR_LOW_LEVEL));
            if (level) {

                imuInitialized = false;
                vTaskDelay(1000);
                imuInit();
            }
        }
        give:
        xSemaphoreGive(i2c_exclusive);
    }
}

void imuTask(void *pvParams) {
    printf("IMU entry\n");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t xFrequency = pdMS_TO_TICKS(500);

    imuInit();
    taskYIELD();

    for (;;) {
        if (!imuInitialized) {
            imuInit();
        } else {
            uint8_t level = gpio_get_level(IMU_INT);
            printf("gpio35 = %d\n", level);
            if (!level) {
                printf("Manually invoking ISR\n");
                xSemaphoreGive(fifo_sync);
            }
            vTaskDelete(NULL);
            return;
        }
        taskYIELD();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void i2c_init() {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 400000 },
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void screenToggle(void *pvArgs) {
    static bool savedBrightness = false;
    static uint8_t brightness;
    for (;;) {
        if (xSemaphoreTake(screen_toggle, pdMS_TO_TICKS(5000))) {
            while (!xSemaphoreTake(i2c_exclusive, pdMS_TO_TICKS(1000)));
            screenOn = !screenOn;
            if (!savedBrightness) {
                savedBrightness = true;
                brightness = M5.Displays(0).getBrightness();
            }
            if (screenOn) {
                M5.Displays(0).setBrightness(brightness);
                M5.Displays(0).wakeup();
            } else {
                M5.Displays(0).setBrightness(0);
                M5.Displays(0).sleep();
            }
            xSemaphoreGive(i2c_exclusive);
        }
    }
}

void screenToggleIsr(void *args) {
    xSemaphoreGive(screen_toggle);
}

void gpio_init() {
    ESP_ERROR_CHECK(gpio_wakeup_disable(IMU_INT));
    ESP_ERROR_CHECK(gpio_set_direction(IMU_INT, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(IMU_INT, imuFifoIsr, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type(IMU_INT, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_pullup_en(IMU_INT));

    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_A, screenToggleIsr, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type(BTN_A, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_pullup_en(BTN_A));
}

void shutdown_handler() {
    M5.Displays(0).releaseBus();
    gpio_uninstall_isr_service();
    i2c_driver_delete(I2C_NUM_0);
}

TaskHandle_t tLoop;
TaskHandle_t tScreen;
TaskHandle_t tImu;
TaskHandle_t tImuSync;
TaskHandle_t tScreenToggle;
extern "C" {
void app_main() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    esp_register_shutdown_handler(shutdown_handler);

    auto cfg = M5.config();
    cfg.internal_imu = false;
    cfg.internal_rtc = false;
    cfg.internal_mic = false;
    cfg.internal_spk = false;
    M5.begin(cfg);

    i2c_init();
    uint8_t axp03;
    I2C_ERR_CHECK(i2c_read8(0x34, 0x03, &axp03), "failed to read axp 0x03: 0x%04x\n");
    printf("\naxp03 == 0x03: %d\n", axp03 == 0x03);

    fifo_sync = xSemaphoreCreateBinaryStatic(&imu_fifo_sync_buffer);
    screen_toggle = xSemaphoreCreateBinaryStatic(&screen_toggle_buffer);
    i2c_exclusive = xSemaphoreCreateMutexStatic(&i2c_exclusive_buffer);
    xSemaphoreGive(i2c_exclusive);
    gpio_init();

    xTaskCreatePinnedToCore(loopTask, "loop", 2*2048, NULL, 2, &tLoop, APP_CPU_NUM);
    xTaskCreatePinnedToCore(screenTask, "screen", 8 * 2048, NULL, 2, &tScreen, APP_CPU_NUM);
    xTaskCreatePinnedToCore(imuTask, "imu", 4 * 2048, NULL, 2, &tImu, APP_CPU_NUM);
    xTaskCreatePinnedToCore(imuSyncTask, "imusync", 4 * 2048, NULL, 2, &tImuSync, APP_CPU_NUM);
    xTaskCreatePinnedToCore(screenToggle, "screentoggle", 2048, NULL, 2, &tScreenToggle, APP_CPU_NUM);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
    }
    printf("App Started\n");
}
}
