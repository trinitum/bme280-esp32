# BME280

This project implements BME280 I2C driver for ESP32. The driver is located in
components/bme280 directory, the project also works as a basic example of how
to use BME280 driver. To build and run the example execute:

```
idf.py build flash monitor
```

it should build the example, upload it to ESP32 and show you the output
updating every 10 second.
