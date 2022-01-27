#ifndef LSM303C_H
#define LSM303C_H

#include <driver/i2c.h>

#define LSM303C_ADDR_A 0x1D
#define LSM303C_CHIP_ID_A 0x41
#define LSM303C_WHO_AM_I_A 0x0F
#define LSM303C_CTRL_REG1_A 0x20
#define LSM303C_OUT_X_L_A 0x28
#define LSM303C_OUT_X_H_A 0x29
#define LSM303C_OUT_Y_L_A 0x2A
#define LSM303C_OUT_Y_H_A 0x2B
#define LSM303C_OUT_Z_L_A 0x2C
#define LSM303C_OUT_Z_H_A 0x2D

#define LSM303C_HR_MASK_A 0x01 << 7
#define LSM303C_ODR_MASK_A 0x07 << 4
#define LSM303C_BDU_MASK_A 0x01 << 3
#define LSM303C_ZEN_MASK_A 0x01 << 2
#define LSM303C_YEN_MASK_A 0x01 << 1
#define LSM303C_XEN_MASK_A 0x01 << 0

#define LSM303C_HR_NORMAL_A 0x00
#define LSM303C_HR_HIGH_A 0x80
#define LSM303C_ODR_OFF_A 0x00
#define LSM303C_ODR_10_A 0x10
#define LSM303C_ODR_50_A 0x20
#define LSM303C_ODR_100_A 0x30
#define LSM303C_ODR_200_A 0x40
#define LSM303C_ODR_400_A 0x50
#define LSM303C_ODR_800_A 0x60
#define LSM303C_BDU_CONT_A 0x00
#define LSM303C_BDU_NU_A 0x08
#define LSM303C_ZEN_OFF_A 0x00
#define LSM303C_ZEN_ON_A 0x04
#define LSM303C_YEN_OFF_A 0x00
#define LSM303C_YEN_ON_A 0x02
#define LSM303C_XEN_OFF_A 0x00
#define LSM303C_XEN_ON_A 0x01

typedef struct {
  int i2cBusNumber;
  uint16_t accelRaw[3];
  float accelG[3];
  uint8_t status;
} LSM303C;

esp_err_t LSM303C_ReadRegisters_A(LSM303C *sensor, uint8_t regAddr,
                                  uint8_t *data, uint16_t len);
esp_err_t LSM303C_WriteRegisters_A(LSM303C *sensor, uint8_t regAddr,
                                   uint8_t *data, uint16_t len);

esp_err_t LSM303C_ReadRegister_A(LSM303C *sensor, uint8_t regAddr,
                                 uint8_t *data);
esp_err_t LSM303C_WriteRegister_A(LSM303C *sensor, uint8_t regAddr,
                                  uint8_t data);

uint8_t LSM303C_Init(LSM303C *sensor, int i2cBusNumber);
uint8_t LSM303C_ReadAccelData(LSM303C *sensor);

#endif //  LSM303C_H
