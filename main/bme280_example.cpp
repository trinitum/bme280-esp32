/* bme280 example
*/

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "bme280.h"

static const char *TAG = "bme280-example";

#define _I2C_NUM(num) I2C_NUM_##num
#define I2C_NUM(num) _I2C_NUM(num)
#define I2C_PORT_NUM I2C_NUM(CONFIG_I2C_PORT_NUM)  /* I2C port number */
#define I2C_SCL_IO   CONFIG_I2C_SCL                /* gpio number for I2C clock */
#define I2C_SDA_IO   CONFIG_I2C_SDA                /* gpio number for I2C data  */
#define I2C_FREQ_HZ  CONFIG_I2C_FREQUENCY          /* I2C clock frequency */

#define BME280_ADDRESS CONFIG_BME280_ADDRESS

SemaphoreHandle_t print_mux = NULL;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_PORT_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    conf.clk_flags = 0;
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

static void bme280_task(void *arg)
{
    bme280 sensor (0, BME280_ADDRESS);
    sensor_values sv;
    esp_err_t err = sensor.init(0x01, 1, 1, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't initialize sensor: %d %s", err, esp_err_to_name(err));
    }

    while (err == ESP_OK) {
        ESP_LOGI(TAG, "checking BME280");
        err = sensor.force();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "couldn't force reading: %d %s", err, esp_err_to_name(err));
            continue;
        }
        vTaskDelay(10 / portTICK_RATE_MS + 1);
        err = sensor.read_values(&sv);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "couldn't read values: %d %s", err, esp_err_to_name(err));
            continue;
        }
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("****************************************\n");
        printf("* Temperature: %fC\n", sv.temperature);
        printf("* Pressure: %fhPa\n", sv.pressure/100);
        printf("* Humidity: %f%%\n", sv.humidity);
        xSemaphoreGive(print_mux);
        vTaskDelay(15000 / portTICK_RATE_MS);
    }

    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(bme280_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
