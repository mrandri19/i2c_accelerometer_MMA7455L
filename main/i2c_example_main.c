#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdio.h>

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO                                                      \
  CONFIG_I2C_MASTER_SCL /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO                                                      \
  CONFIG_I2C_MASTER_SDA /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM                                                         \
  I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev   \
                                          */
#define I2C_MASTER_FREQ_HZ                                                     \
  CONFIG_I2C_MASTER_FREQUENCY       /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1 /*!< I2C master will check ack from slave*/
#define ACK_VAL 0x0      /*!< I2C ack value */
#define NACK_VAL 0x1     /*!< I2C nack value */

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_x_lsb,
                                        uint8_t *data_y_lsb,
                                        uint8_t *data_z_lsb,
                                        uint8_t *data_x_msb,
                                        uint8_t *data_y_msb,
                                        uint8_t *data_z_msb) {
  int ret;

  // Build a command
  // SINGLE BYTE READ
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  {
    i2c_master_start(cmd); // ST
    i2c_master_write_byte(cmd, (0x1d << 1) | 0,
                          ACK_CHECK_EN); // -> Device Address [6:0] + W -> AK
    i2c_master_write_byte(cmd, 0x00,
                          ACK_CHECK_EN); // -> Register address [7:0] -> AK

    i2c_master_start(cmd); // SR
    i2c_master_write_byte(cmd, (0x1d << 1) | 1,
                          ACK_CHECK_EN); // -> Device address [6:0] + R -> AK
    i2c_master_read_byte(cmd, data_x_lsb, ACK_VAL);  // -> AK
    i2c_master_read_byte(cmd, data_x_msb, ACK_VAL);  // -> AK
    i2c_master_read_byte(cmd, data_y_lsb, ACK_VAL);  // -> AK
    i2c_master_read_byte(cmd, data_y_msb, ACK_VAL);  // -> AK
    i2c_master_read_byte(cmd, data_z_lsb, ACK_VAL);  // -> AK
    i2c_master_read_byte(cmd, data_z_msb, NACK_VAL); // -> NAK
    i2c_master_stop(cmd);                            // -> SP

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  }
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);

  return ret;
}

static esp_err_t i2c_master_init(void) {
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER; // act as i2c master

  conf.sda_io_num =
      I2C_MASTER_SDA_IO; // set on which pin to transmit serial data
  conf.sda_pullup_en =
      GPIO_PULLUP_ENABLE; // enable the internal pull-up resistors

  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

  i2c_param_config(i2c_master_port, &conf);

  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_set_measurement_mode(i2c_port_t i2c_num) {
  int ret;

  // Build a command
  // SINGLE BYTE WRITe
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  {
    i2c_master_start(cmd); // ST
    i2c_master_write_byte(cmd, (0x1d << 1) | 0,
                          ACK_CHECK_EN); // -> Device Address [6:0] + W -> AK
    i2c_master_write_byte(cmd, 0x16,
                          ACK_CHECK_EN); // -> Register address [7:0] -> AK
    i2c_master_write_byte(cmd, 1,
                          ACK_CHECK_EN); // -> Data [7:0] -> AK
    i2c_master_stop(cmd);                // -> SP

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  }
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);

  return ret;
}

static void i2c_test_task(void *arg) {
  int ret;
  uint8_t data_x_lsb = 42;
  uint8_t data_y_lsb = 42;
  uint8_t data_z_lsb = 42;
  uint8_t data_x_msb = 42;
  uint8_t data_y_msb = 42;
  uint8_t data_z_msb = 42;

  while (1) {
    ret = i2c_master_sensor_test(I2C_MASTER_NUM, &data_x_lsb, &data_y_lsb,
                                 &data_z_lsb, &data_x_msb, &data_y_msb,
                                 &data_z_msb);
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "I2C Timeout");
    } else if (ret == ESP_OK) {
      // Take 8 bits of the low register and 2 bits of the high register to make
      // a 10-bit uint in 2's complement
      uint16_t utmp_x = ((uint16_t)data_x_msb << 8) + (uint16_t)data_x_lsb;
      uint16_t utmp_y = ((uint16_t)data_y_msb << 8) + (uint16_t)data_y_lsb;
      uint16_t utmp_z = ((uint16_t)data_z_msb << 8) + (uint16_t)data_z_lsb;
      // If the MSB of the 10-bit uint is 1 then make the other bits to the left
      // 1 as well to correctly extend 10-bit 2's complement to 16 bits
      int16_t tmp_x = (int16_t)((utmp_x > 0x0200) ? utmp_x + 0xfc00 : utmp_x);
      int16_t tmp_y = (int16_t)((utmp_y > 0x0200) ? utmp_y + 0xfc00 : utmp_y);
      int16_t tmp_z = (int16_t)((utmp_z > 0x0200) ? utmp_z + 0xfc00 : utmp_z);
      // [-512,511] -> [-8g, +8g]
      float x = 8.0f * 9.81f * ((float)tmp_x) / 512.0f;
      float y = 8.0f * 9.81f * ((float)tmp_y) / 512.0f;
      float z = 8.0f * 9.81f * ((float)tmp_z) / 512.0f;

      printf("(x, y, z): (%+06.2f, %+06.2f, %+06.2f) m/s^2\n", x, y, z);
    } else {
      ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...",
               esp_err_to_name(ret));
    }

    vTaskDelay(8 / portTICK_RATE_MS);
  }

  vTaskDelete(NULL);
}

void app_main(void) {

  ESP_ERROR_CHECK(i2c_master_init());

  ESP_ERROR_CHECK(i2c_set_measurement_mode(I2C_MASTER_NUM));

  // This task will delete print_mux semaphore and then delete itself
  xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
