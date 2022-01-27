#include "LSM303C.h"
#include <stdio.h>

esp_err_t LSM303C_ReadRegisters_A(LSM303C *sensor, uint8_t regAddr,
                                  uint8_t *data, uint16_t len) {
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LSM303C_ADDR_A << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd, regAddr, 1);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(sensor->i2cBusNumber, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LSM303C_ADDR_A << 1) | I2C_MASTER_READ, 1);
  i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(sensor->i2cBusNumber, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return (ret);
}

esp_err_t LSM303C_WriteRegisters_A(LSM303C *sensor, uint8_t regAddr,
                                   uint8_t *data, uint16_t len) {
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LSM303C_ADDR_A << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write_byte(cmd, regAddr, 1);
  i2c_master_write(cmd, data, len, 1);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(sensor->i2cBusNumber, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t LSM303C_ReadRegister_A(LSM303C *sensor, uint8_t regAddr,
                                 uint8_t *data) {
  return (LSM303C_ReadRegisters_A(sensor, regAddr, data, 1));
}

esp_err_t LSM303C_WriteRegister_A(LSM303C *sensor, uint8_t regAddr,
                                  uint8_t data) {
  return (LSM303C_WriteRegisters_A(sensor, regAddr, &data, 1));
}

uint8_t LSM303C_Init(LSM303C *sensor, int i2cBusNumber) {
  sensor->i2cBusNumber = i2cBusNumber;

  uint8_t chipID;

  LSM303C_ReadRegister_A(sensor, LSM303C_WHO_AM_I_A, &chipID);

  if (chipID != LSM303C_CHIP_ID_A) {
    return (1);
  }

  uint8_t controlRegisterData = (LSM303C_HR_MASK_A & LSM303C_HR_NORMAL_A) |
                                (LSM303C_ODR_MASK_A & LSM303C_ODR_10_A) |
                                (LSM303C_BDU_MASK_A & LSM303C_BDU_CONT_A) |
                                (LSM303C_ZEN_MASK_A & LSM303C_ZEN_ON_A) |
                                (LSM303C_YEN_MASK_A & LSM303C_YEN_ON_A) |
                                (LSM303C_XEN_MASK_A & LSM303C_XEN_ON_A);

  printf("Register data to write:\t%02X\n", controlRegisterData);

  LSM303C_WriteRegister_A(sensor, LSM303C_CTRL_REG1_A, controlRegisterData);

  return (0);
}

uint8_t LSM303C_ReadAccelData(LSM303C *sensor) {
  esp_err_t ret;
  uint8_t data[6];

  /* Read accelerometer data */
  ret = LSM303C_ReadRegisters_A(sensor, LSM303C_OUT_X_L_A, data, 6);

  if (ret != ESP_OK) {
    return (1);
  }

  sensor->accelRaw[0] = data[0] | data[1] << 8;
  sensor->accelRaw[1] = data[2] | data[3] << 8;
  sensor->accelRaw[2] = data[4] | data[5] << 8;

  /* Process data */
  sensor->accelG[0] = (sensor->accelRaw[0] * 0.061f / 1000.0f) - 2.0f;
  sensor->accelG[1] = (sensor->accelRaw[1] * 0.061f / 1000.0f) - 2.0f;
  sensor->accelG[2] = (sensor->accelRaw[2] * 0.061f / 1000.0f) - 2.0f;

  return (0);
}