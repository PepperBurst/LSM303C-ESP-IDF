/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "LSM303C.h"
#include "driver/i2c.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define PIN_LED GPIO_NUM_23
#define PIN_SDA GPIO_NUM_18
#define PIN_SCL GPIO_NUM_19
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

void vReadIMU(void *pvParameters);

LSM303C imu;

void app_main(void) {
  printf("Hello world!\n");

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  printf("silicon revision %d, ", chip_info.revision);

  printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");

  i2c_config_t i2c_conf = {.mode = I2C_MODE_MASTER,
                           .sda_io_num = PIN_SDA,
                           .sda_pullup_en = GPIO_PULLUP_ENABLE,
                           .scl_io_num = PIN_SCL,
                           .scl_pullup_en = GPIO_PULLUP_ENABLE,
                           .master.clk_speed = 100000};
  i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
  i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);

  uint8_t status = LSM303C_Init(&imu, I2C_MASTER_NUM);

  if (status != 0) {
    printf("Error in intializing LSM303C\n");
    for (int i = 10; i >= 0; i--) {
      printf("Restarting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
  }

  xTaskCreate(&vReadIMU, "IMU Task", 4096, NULL, 1, NULL);
}

void vReadIMU(void *pvParameters) {
  for (;;) {
    LSM303C_ReadAccelData(&imu);
    printf("X/Y/Z (g):\t%0.2f\t%0.2f\t%0.2f\n", imu.accelG[0], imu.accelG[1],
           imu.accelG[2]);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}