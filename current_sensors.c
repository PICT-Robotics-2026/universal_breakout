
#include <ina219.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <rom/ets_sys.h> // for ets_delay_us (hardware delay)

#include "current_sensors.h"

#define I2C_MASTER_PORT         0
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_FREQ_HZ      100000

#define SENSOR_1_ADDR           0x41
#define SENSOR_2_ADDR           0x44
#define SENSOR_3_ADDR           0x45

#define SHUNT_RESISTOR_OHM      0.1f

ina219_t current_sensors[3];

static void recover_i2c_bus(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pin_bit_mask = (1ULL << I2C_MASTER_SCL_IO) | (1ULL << I2C_MASTER_SDA_IO),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    // toggle SCL 9 times to flush stuck slaves
    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        ets_delay_us(5); // 5us delay (hardware, not RTOS)
        gpio_set_level(I2C_MASTER_SCL_IO, 0);
        ets_delay_us(5);
    }

    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    
    gpio_reset_pin(I2C_MASTER_SDA_IO);
    gpio_reset_pin(I2C_MASTER_SCL_IO);
}

static esp_err_t init_single_device(ina219_t *dev, uint8_t addr)
{
    esp_err_t res = ina219_init_desc(dev, addr, I2C_MASTER_PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if (res != ESP_OK) return res;

    res = ina219_init(dev);
    if (res != ESP_OK) return res;

    res = ina219_configure(dev, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
                           INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, 
                           INA219_MODE_CONT_SHUNT_BUS);
    if (res != ESP_OK) return res;

    return ina219_calibrate(dev, SHUNT_RESISTOR_OHM);
}

int are_sensors_initialized = false;

void current_sensors_init(void)
{
    if (are_sensors_initialized)
	return;
    
    recover_i2c_bus();

    ESP_ERROR_CHECK(i2cdev_init()); 

    init_single_device(&current_sensors[0], SENSOR_1_ADDR);
    init_single_device(&current_sensors[1], SENSOR_2_ADDR);
    init_single_device(&current_sensors[2], SENSOR_3_ADDR);

    are_sensors_initialized = true;
}

// All currents are in Amperes

float sensor_get_current(current_sensor_t sensor)
{
    float current = 0;
    esp_err_t err = ina219_get_current(&current_sensors[sensor],
				       &current);

    if (err == ESP_OK)
	return current;
    
    return 0.0f;
}
