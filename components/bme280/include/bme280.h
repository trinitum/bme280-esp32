#ifndef BME280_H_
#define BME280_H_

typedef struct {
    float temperature;
    float pressure;
    float humidity;
} sensor_values;

class bme280 {
    public:
        bme280(int port, uint8_t address);
        /**
         * @brief Initialize sensor with specified mode, and oversampling
         * values for temperature, pressure and humidity. The method checks
         * that the chip id is 0x60 and reads calibration data from the
         * chip before setting oversampling values and mode. The I2C has to be
         * initialized already.
         */
        esp_err_t init(uint8_t mode, uint8_t ost, uint8_t osp, uint8_t osh);
        /**
         * @brief Send a command to the sensor to take measurements one time
         * and then go into sleep mode. After measurments have been taken they
         * can be read at any time.
         */
        esp_err_t force();
        /**
         * @brief Read the values from the sensor. The result stored into the
         * structure pointed by res.
         */
        esp_err_t read_values(sensor_values *res);
    private:
        esp_err_t read(uint8_t *data, size_t size);
        esp_err_t write(uint8_t *data, size_t size);
        esp_err_t read_calibration();
        int32_t compensate_t(int32_t adc_T);
        uint32_t compensate_p(int32_t adc_P, int32_t t_fine);
        uint32_t compensate_h(int32_t adc_H, int32_t t_fine);
        // I2C port
        int m_port;
        // the address of the sensor
        uint8_t m_address;
        // control registers
        uint8_t m_ctrl_hum;
        uint8_t m_ctrl_meas;
        // callibration values
        uint16_t m_dig_T1;
        int16_t m_dig_T2;
        int16_t m_dig_T3;
        uint16_t m_dig_P1;
        int16_t m_dig_P2;
        int16_t m_dig_P3;
        int16_t m_dig_P4;
        int16_t m_dig_P5;
        int16_t m_dig_P6;
        int16_t m_dig_P7;
        int16_t m_dig_P8;
        int16_t m_dig_P9;
        uint8_t m_dig_H1;
        int16_t m_dig_H2;
        uint8_t m_dig_H3;
        int16_t m_dig_H4;
        int16_t m_dig_H5;
        int8_t m_dig_H6;
};

#endif // BME280_H_
