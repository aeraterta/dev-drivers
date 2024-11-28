/**
 ******************************************************************************
 * @file    LSM303.c
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    13-September-2024
 * @brief   Contains all the functionalities to control the LSM303DLHC
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 mcu-dev
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "lsm303.h"

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param lsm303_params - The structure that contains the device initial
 * 					   parameters.
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t lsm303_setup(struct lsm303_dev **device,
                     struct lsm303_init_param lsm303_params) {
  uint8_t ret;

  struct lsm303_dev *dev;

  dev->acc_power_mode = lsm303_params.acc_power_mode;
  dev->acc_odr = lsm303_params.acc_odr;
  dev->acc_axes = lsm303_params.acc_axes;
  dev->acc_scale = lsm303_params.acc_scale;

  ret |= lsm303_set_power_mode(dev, dev->acc_power_mode);
  ret |= lsm303_acc_enable_axes(dev, dev->acc_axes);
  ret |= lsm303_acc_set_odr(dev, dev->acc_odr);
  ret |= lsm303_acc_set_scale(dev, dev->acc_scale);

  *device = dev;

  return ret;
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param mode - The power mode.
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t lsm303_set_power_mode(struct lsm303_dev *device,
                              enum lsm303_acc_power_mode mode) {
  uint8_t val = 0x00;

  // TO DO: Get the current value of the register
  val = lsm303_reg_read(&device, ACC_I2C_ADDRESS, , &val);

  // TO DO: Update only the power mode
  val = val | mode << ACC_POWER_MODE_MASK;

  // TO DO: I2C communication
  return lsm303_reg_write();
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param axes - The axes enable/disable controls.
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t lsm303_acc_enable_axes(struct lsm303_dev *device,
                               enum lsm303_acc_axes_enable axes) {
  uint8_t val = 0x00;

  // TO DO: Get the current value of the register
  val = lsm303_reg_read();

  // TO DO: Update only the power mode
  val = val | axes << ACC_AXES_MASK;

  // TO DO: I2C communication
  return lsm303_reg_write();
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param odr - The ODR controls.
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t lsm303_acc_set_odr(struct lsm303_dev *device, enum lsm303_acc_odr odr) {
  uint8_t val = 0x00;

  // TO DO: Get the current value of the register
  val = lsm303_reg_read();

  // TO DO: Update only the power mode
  val = val | odr << ACC_ODR_MASK;

  // TO DO: I2C communication
  return lsm303_reg_write();
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param scale - The scale controls.
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t lsm303_acc_set_scale(struct lsm303_dev *device,
                             enum lsm303_acc_full_scale scale) {
  uint8_t val = 0x00;

  // TO DO: Get the current value of the register
  val = lsm303_reg_read();

  // TO DO: Update only the power mode
  val = val | scale << ACC_SCALE_MASK;

  // TO DO: I2C communication
  return lsm303_reg_write();
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 * 					   parameters.
 * @param status - The structure that will contains the ADC status
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t lsm303_reg_read() {
  uint8_t ret = 0;

  // TO DO: I2C communication
  return ret;
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 * 					   parameters.
 * @param status - The structure that will contains the ADC status
 * @return 0 in case of success, negative error code otherwise.
 */
uint8_t lsm303_reg_write() {
  uint8_t ret = 0;

  // TO DO: I2C communication
  return ret;
}
