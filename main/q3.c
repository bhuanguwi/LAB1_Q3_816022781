/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"


static const char* TAG = "main";

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a ADS1115 sensor for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO0 is assigned as the data signal of i2c master port
 *    GPIO2 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO0/GPIO2
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

#define I2C_EXAMPLE_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           0               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define ADS1115_SENSOR_ADDR                 0x48             /*!< slave address for MPU6050 sensor */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

 /**
  * Define the ads1115 register address:
  */
#define CFG_REG         0x01
#define CONVERSION_REG  0x00

  /**
   * @brief i2c master initialization
   */
static esp_err_t i2c_example_master_init()
{
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
  conf.sda_pullup_en = 1;
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
  conf.scl_pullup_en = 1;
  conf.clk_stretch_tick = 500; // 500 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
  return ESP_OK;
}

/**
 * @brief test code to write ads1115
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */

 // Page 24 Section 9.5.3 First Paragraph
// ads1115 write function (master write to slave)
static esp_err_t i2c_example_master_ads1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t* data, size_t data_len)
{
  int ret;
  // creating a command link 
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // populate command link with the following bits : start,slave address,data,stop bit
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN); 
  i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

/**
 * @brief test code to read ads1115
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */

 // Page 24 Section 9.5.3 Second Paragraph
 // ads1115 read function ( master reads from slave ) 
static esp_err_t i2c_example_master_ads1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t* data, size_t data_len)
{
  // Writing slave address byte first 
  int ret;
  // create command link
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // start bit
  i2c_master_start(cmd);
  // writing the slave address byte 
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  // writing the address pointer byte which is the register being read from
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
  // stop bit
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }

  // Reading from slave 
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ADS1115_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

static esp_err_t i2c_example_master_ads1115_init(i2c_port_t i2c_num)
{
  // writing to the config register, refer to Table 8 , Section 9.6.3, Page 28
  uint8_t cfg_bits[2];
  /* msb of configuration register
   0b 1 100 001 1 (OS, MUX[2:0], PGA[2:0], MODE) for single shot conversion
   0b 0 100 001 0 for continuous conversion */
  cfg_bits[0] = 0x42;
  // lsb of configuration register
  // 0b 100 0 0 0 11 (DR[2:0], COMP_MODE, COMP_POL, COMP_LAT, COMP_QUEUE[1:0])
  cfg_bits[1] = 0x83;
  vTaskDelay(1000 / portTICK_RATE_MS);
  i2c_example_master_init();
  // writing configuration bits to the configuration register 
  ESP_ERROR_CHECK(i2c_example_master_ads1115_write(i2c_num, CFG_REG, &cfg_bits, 2));
  return ESP_OK;
}

static void i2c_ads1115_task(void* arg)
{
  uint8_t sensor_data[2];
  int16_t adc_reading;
  int ret;

  ret = i2c_example_master_ads1115_init(I2C_EXAMPLE_MASTER_NUM);
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "ADS1115 initialized!\n");
  }
  else
  {
    ESP_LOGI(TAG, "Error occured while trying to initialize the ADS1115\n");
    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
  }

  while (1)
  {
    memset(sensor_data, 0, 2);
    // extracting msb and storing in sensor_data[0] and lsb in sensor_data [1]
    ret = i2c_example_master_ads1115_read(I2C_EXAMPLE_MASTER_NUM, CONVERSION_REG, sensor_data, 2);

    if (ret == ESP_OK)
    {
      // converting the reading into adc value from 0 to 2^16 - 1
      adc_reading = (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
      ESP_LOGI(TAG, "Analog 0 Pin Value : %d \n", adc_reading);
    }
    else
    {
      ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
    }
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
  


  void app_main(void)
  {
    ESP_LOGI(TAG, "Starting up\n ");
    //start i2c task
    xTaskCreate (i2c_ads1115_task, "i2c_ads1115_task", 2048, NULL, 10, NULL);
  }
