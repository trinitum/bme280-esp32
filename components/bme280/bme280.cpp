#include "driver/i2c.h"
#include "esp_log.h"

#include "bme280.h"

static const char *TAG = "bme280";

bme280::bme280(int port, uint8_t address) {
    m_port = port;
    m_address = address;
}

esp_err_t bme280::write(uint8_t *data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m_address<<1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, size, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(m_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t bme280::read(uint8_t *data, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (m_address << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(m_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t bme280::read_calibration() {
    uint8_t cal[42];
    uint8_t cal0 = 0x88;
    uint8_t cal26 = 0xe1;
    esp_err_t err = write(&cal0, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't request calibration data 0: %d %s", err, esp_err_to_name(err));
        return err;
    }
    err = read(cal, 26);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't read calibration data 0: %d %s", err, esp_err_to_name(err));
        return err;
    }
    err = write(&cal26, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't request calibration data 26: %d %s", err, esp_err_to_name(err));
        return err;
    }
    err = read(cal+26, 16);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't read calibration data 26: %d %s", err, esp_err_to_name(err));
        return err;
    }

    m_dig_T1 = cal[0] | (cal[1] << 8);
    m_dig_T2 = cal[2] | (cal[3] << 8);
    m_dig_T3 = cal[4] | (cal[5] << 8);
    ESP_LOGD(TAG, "T1, T2, T3: %d, %d, %d", m_dig_T1, m_dig_T2, m_dig_T3);
    m_dig_P1 = cal[6] | (cal[7] << 8);
    m_dig_P2 = cal[8] | (cal[9] << 8);
    m_dig_P3 = cal[10] | (cal[11] << 8);
    m_dig_P4 = cal[12] | (cal[13] << 8);
    m_dig_P5 = cal[14] | (cal[15] << 8);
    m_dig_P6 = cal[16] | (cal[17] << 8);
    m_dig_P7 = cal[18] | (cal[19] << 8);
    m_dig_P8 = cal[20] | (cal[21] << 8);
    m_dig_P9 = cal[22] | (cal[23] << 8);
    m_dig_H1 = cal[25];
    m_dig_H2 = cal[26] | (cal[27] << 8);
    m_dig_H3 = cal[28];
    m_dig_H4 = (cal[29] << 4) | (cal[30] & 0x0f);
    m_dig_H5 = (cal[30] >> 4) | (cal[31] << 4);
    m_dig_H6 = cal[32];
    return ESP_OK;
}

esp_err_t bme280::init(uint8_t mode, uint8_t ost, uint8_t osp, uint8_t osh) {
    // start with reading the chip ID
    uint8_t id_reg = 0xd0;
    esp_err_t err = write(&id_reg, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't request the chip id: %d %s", err, esp_err_to_name(err));
        return err;
    }
    uint8_t id;
    err = read(&id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't read the chip id: %d %s", err, esp_err_to_name(err));
        return err;
    }

    // the chip id must be 0x60 for BME280
    if (id != 0x60) {
        ESP_LOGE(TAG, "the chip id is 0x%x, not BME280", id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "found BME280, initializing");

    err = read_calibration();
    if (err != ESP_OK) {
        return err;
    }

    // now we can initialize the chip
    m_ctrl_hum = osh & 0x07;
    m_ctrl_meas = ((ost & 0x07) << 5) | ((osp & 0x07) << 2) | (mode & 0x03);
    uint8_t ctrl_reg[] = { 0xf2, m_ctrl_hum, 0xf4, m_ctrl_meas };
    err = write(ctrl_reg, sizeof(ctrl_reg));
    return err;
}

esp_err_t bme280::force() {
    uint8_t ctrl_meas[] = { 0xf4, (uint8_t)((m_ctrl_meas & 0xfc) | 0x1) };
    esp_err_t err = write(ctrl_meas, sizeof(ctrl_meas));
    return err;
}

esp_err_t bme280::read_values(sensor_values *res) {
    uint8_t press_reg = 0xf7;
    esp_err_t err = write(&press_reg, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't request the values: %d %s", err, esp_err_to_name(err));
        return err;
    }
    uint8_t adc[8];
    err = read(adc, sizeof(adc));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "couldn't read the values: %d %s", err, esp_err_to_name(err));
        return err;
    }
    ESP_LOGD(TAG, "values: %x %x %x %x %x %x %x %x", adc[0], adc[1], adc[2], adc[3], adc[4], adc[5], adc[6], adc[7]);
    int32_t adc_P = (((adc[0] << 8) | adc[1]) << 4) | (adc[2] >> 4);
    int32_t adc_T = (((adc[3] << 8) | adc[4]) << 4) | (adc[5] >> 4);
    int32_t adc_H = (adc[6] << 8) | adc[7];
    int32_t t_fine = compensate_t(adc_T);
    int32_t temp = (t_fine * 5 + 128) >> 8;
    uint32_t press = compensate_p(adc_P, t_fine);
    uint32_t hum = compensate_h(adc_H, t_fine);
    res->temperature = (float)temp / 100.0;
    res->pressure = (float)press / 256.0;
    res->humidity = (float)hum / 1024.0;
    return ESP_OK;
}

int32_t bme280::compensate_t(int32_t adc_T) {
    int32_t dig_T1 = (int32_t)m_dig_T1;
    int32_t var1  = (((adc_T>>3) - (dig_T1<<1)) * (int32_t)m_dig_T2) >> 11;
    int32_t var2 = (((((adc_T>>4) - dig_T1) * ((adc_T>>4) - (dig_T1))) >> 12) * ((int32_t)m_dig_T3)) >> 14;
    return var1 + var2;
}

uint32_t bme280::compensate_p(int32_t adc_P, int32_t t_fine) {
    int64_t var1 = t_fine - 128000;
    int64_t var2 = var1 * var1 * m_dig_P6;
    var2 = var2 + (((int64_t)var1*m_dig_P5)<<17);
    var2 = var2 + ((int64_t)m_dig_P4<<35);
    var1 = ((var1 * var1 * m_dig_P3)>>8) + ((var1 * m_dig_P2)<<12);
    var1 = (((int64_t)1<<47)+var1)*m_dig_P1>>33;
    if (var1 == 0) {
        return 0;
    }
    int64_t p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (m_dig_P9 * (p>>13) * (p>>13)) >> 25;
    var2 = (m_dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (m_dig_P7<<4);
    return (uint32_t)p;
}

uint32_t bme280::compensate_h(int32_t adc_H, int32_t t_fine) {
    int32_t res = t_fine - 76800;
    res = ((((adc_H << 14) - ((int32_t)m_dig_H4 << 20) - (m_dig_H5 * res))
                + 16384) >> 15) * (((((((res * m_dig_H6) >> 10) *
                (((res * m_dig_H3) >> 11) + 32768)) >> 10) + 2097152) * m_dig_H2 +
                8192) >> 14);
    res = res - (((((res >> 15) * (res >> 15)) >> 7) * m_dig_H1) >> 4);
    res = res < 0 ? 0 : res;
    res = res > 419430400 ? 419430400 : res;
    return (uint32_t)(res >> 12);
}
