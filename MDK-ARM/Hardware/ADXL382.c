#include <stdlib.h>
#include "ADXL382.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "debug.h"

static const uint8_t adxl38x_scale_mul[3] = {1, 2, 4};
static int64_t adxl38x_accel_conv(struct adxl38x_dev *dev, uint16_t raw_accel);
static int adxl38x_set_to_standby(struct adxl38x_dev *dev);


extern SPI_HandleTypeDef hspi5;

/**
 * @brief 通过 SPI5 从 ADXL38x 读取寄存器数据
 * @param dev            设备结构体指针（仅传递以兼容接口，可不使用）
 * @param base_address   起始寄存器地址
 * @param size           要读取的字节数（最大64）
 * @param read_data      接收数据缓冲区
 * @return HAL_StatusTypeDef
 */
int adxl38x_read_device_data(struct adxl38x_dev *dev,
                             uint8_t base_address,
                             uint16_t size,
                             uint8_t *read_data)
{
    uint8_t cmd = (base_address << 1) | ADXL38X_SPI_READ;
    uint8_t rx_buf[64] = {0};

    if (size > sizeof(rx_buf))
        return HAL_ERROR;

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi5, &cmd, 1, 1000) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
        return HAL_ERROR;
    }
    if (HAL_SPI_Receive(&hspi5, rx_buf, size, 1000) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);

    memcpy(read_data, rx_buf, size);
    return HAL_OK;
}

/**
 * @brief 通过 SPI5 向 ADXL38x 写寄存器数据
 * @param dev            设备结构体指针（仅传递以兼容接口，可不使用）
 * @param base_address   起始寄存器地址
 * @param size           要写入的字节数（最大63）
 * @param write_data     待写入数据缓冲区
 * @return HAL_StatusTypeDef
 */
int adxl38x_write_device_data(struct adxl38x_dev *dev,
                              uint8_t base_address,
                              uint16_t size,
                              uint8_t *write_data)
{
    uint8_t buf[1 + 64] = {0};

    if (size > 64)
        return HAL_ERROR;

    buf[0] = (base_address << 1) | ADXL38X_SPI_WRITE;
    memcpy(&buf[1], write_data, size);

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi5, buf, size + 1, 1000) != HAL_OK) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
    return HAL_OK;
}


/***************************************************************************//**
 * @brief Updates register with specified bits using given mask
 *
 * @param dev          	- The device structure.
 * @param reg_addr 		- Address of the register to update.
 * @param mask         	- Mask for the update field.
 * @param update_val   	- Value to be updated.
 *
 * @return ret         	- Result of the writing procedure.
*******************************************************************************/
int adxl38x_register_update_bits(struct adxl38x_dev *dev, uint8_t reg_addr,
				 uint8_t mask,
				 uint8_t update_val)
{
	int ret;
	uint8_t data;

	ret = adxl38x_read_device_data(dev, reg_addr, 1, &data);
	if (ret)
		return ret;

	if (reg_addr == ADXL38X_OP_MODE)
		adxl38x_set_to_standby(dev);

	data &= ~mask;
	data |= update_val;

	return adxl38x_write_device_data(dev, reg_addr, 1, &data);
}

/***************************************************************************//**
 * @brief Initializes the device and checks for valid peripheral communication
 *
 * @param device        - The device structure.
 * @param init_param 	- Structure containing initialization parameters
 *
 * @return ret         	- Result of the initialization
*******************************************************************************/
int adxl38x_init(struct adxl38x_dev **device,
		 struct adxl38x_init_param init_param)
{
	struct adxl38x_dev *dev;
	int ret;
	uint8_t reg_value;

	dev = (struct adxl38x_dev *)malloc(sizeof *dev);;
	if (!dev)
		return -1;

	switch (init_param.dev_type) {
	case ID_ADXL380:
	case ID_ADXL382:
		dev->dev_type = init_param.dev_type;
		break;
	default:
		return -1;
	}

	dev->comm_type = init_param.comm_type;

	ret = adxl38x_read_device_data(dev, ADXL38X_DEVID_AD,
				       1, &reg_value);
	if (ret || (reg_value != ADXL38X_RESET_DEVID_AD))
		goto error;

	ret = adxl38x_read_device_data(dev, ADXL38X_DEVID_MST,
				       1, &reg_value);
	if (ret || (reg_value != ADXL38X_RESET_DEVID_MST))
		goto error;

	ret = adxl38x_read_device_data(dev, ADXL38X_PART_ID,
				       1, &reg_value);
	if (ret || (reg_value != ADXL38X_RESET_PART_ID))
		goto error;

	/* Set device type 380/382 */
	ret = adxl38x_read_device_data(dev, ADXL38X_MISC0,
				       1, &reg_value);
	if (ret)
		goto error;
	if ((reg_value & BIT(7))) {
		dev->dev_type = ID_ADXL382;
	} else {
		dev->dev_type = ID_ADXL380;
	}

	/* Set default values to the device structure */
	dev->range = ADXL380_RANGE_4G;
	dev->op_mode = ADXL38X_MODE_STDBY;
	*device = dev;
	return 0;
error:
    free(dev);
    return -1;
}

/***************************************************************************//**
 * @brief Free the resources allocated by the init function.
 *
 * @param dev  - The device structure.
 *
 * @return ret - Result of the remove procedure.
*******************************************************************************/
int adxl38x_remove(struct adxl38x_dev *dev)
{
	int ret;
	return ret;
}

/***************************************************************************//**
 * @brief Performs a soft reset of the device.
 *
 * @param dev  - The device structure.
 *
 * @return ret - Result of the soft reset procedure.
*******************************************************************************/
int adxl38x_soft_reset(struct adxl38x_dev *dev)
{
	uint8_t reg_value;
	int ret;
	uint8_t data = ADXL38X_RESET_CODE;

	// Perform soft reset
	ret = adxl38x_write_device_data(dev, ADXL38X_REG_RESET, 1, &data);
	if (ret)
    {
        printf("reset error1\r\n");
		return ret;
    }
	// Delay is needed between soft reset and initialization (From SOFT_RESET
	// bit description in REG_RESET)
	HAL_Delay(500);
	ret = adxl38x_read_device_data(dev, ADXL38X_DEVID_AD, 1, &reg_value);
    printf("reg_value %#x\r\n", reg_value);
	if (reg_value != 0xAD)
    {
        printf("reset error2\r\n");
		return -1;
    }
	return 0;
}

/***************************************************************************//**
 * @brief Places the device into the given operation mode.
 *
 * @param dev     - The device structure.
 * @param op_mode - Operation mode mode.
 *
 * @return ret    - Result of the setting operation procedure.
*******************************************************************************/
int adxl38x_set_op_mode(struct adxl38x_dev *dev, enum adxl38x_op_mode op_mode)
{
	int ret;

	ret = adxl38x_register_update_bits(dev, ADXL38X_OP_MODE,
					   ADXL38X_MASK_OP_MODE,
					   FIELD_PREP(ADXL38X_MASK_OP_MODE, op_mode));
	if (!ret)
		dev->op_mode = op_mode;
	// 2ms wait for op_mode to settle (See Operating Modes section of datasheet)
	HAL_Delay(2);

	return ret;
}

/***************************************************************************//**
 * @brief Gets the current operation mode of the device
 *
 * @param dev     - The device structure.
 * @param op_mode - Read operation mode.
 *
 * @return ret    - Result of the reading operation procedure.
*******************************************************************************/
int adxl38x_get_op_mode(struct adxl38x_dev *dev, enum adxl38x_op_mode *op_mode)
{
	int ret;
	uint8_t op_mode_reg_val;

	ret = adxl38x_read_device_data(dev, ADXL38X_OP_MODE, 1, &op_mode_reg_val);
	if (ret)
		return ret;
	*op_mode = FIELD_GET(ADXL38X_MASK_OP_MODE, op_mode_reg_val);

	if (!ret)
		dev->op_mode = *op_mode;

	return 0;
}

/***************************************************************************//**
 * @brief Sets the measurement range register value.
 *
 * @param dev       - The device structure.
 * @param range_val - Selected range.
 *
 * @return ret      - Result of the writing procedure.
*******************************************************************************/
int adxl38x_set_range(struct adxl38x_dev *dev, enum adxl38x_range range_val)
{
	int ret;

	ret = adxl38x_register_update_bits(dev, ADXL38X_OP_MODE,
					   ADXL38X_MASK_RANGE,
					   FIELD_PREP(ADXL38X_MASK_RANGE, range_val));
	if (!ret)
		dev->range = range_val;

	return ret;
}

/***************************************************************************//**
 * @brief Gets the current range setting of the device
 *
 * @param dev       - The device structure.
 * @param range_val - Read range.
 *
 * @return ret    	- Result of the reading operation procedure.
*******************************************************************************/
int adxl38x_get_range(struct adxl38x_dev *dev, enum adxl38x_range *range_val)
{
	int ret;
	uint8_t range_reg_val;

	ret = adxl38x_read_device_data(dev, ADXL38X_OP_MODE, 1, &range_reg_val);
	if (ret)
		return ret;
	*range_val = FIELD_GET(ADXL38X_MASK_RANGE, range_reg_val);

	if (!ret)
		dev->range = *range_val;

	return 0;
}

/***************************************************************************//**
 * @brief Gets the current device type
 *
 * @param dev     - The device structure
 * @param devID   - Read Device ID
 *
 * @return ret    - Result of the reading operation procedure.
*******************************************************************************/
int adxl38x_get_deviceID(struct adxl38x_dev *dev,
			 enum adxl38x_id *devID)
{
	int ret;
	uint8_t misc0_reg;

	ret = adxl38x_read_device_data(dev, ADXL38X_MISC0, 1, &misc0_reg);
	if (misc0_reg & BIT(7))
		dev->dev_type = ID_ADXL382;
	else
		dev->dev_type = ID_ADXL380;
	*devID = dev->dev_type;

	return ret;
}

/***************************************************************************//**
 * NOTE: It is recommended that the ADXL380 self-test should be performed in mid
 * and high gee ranges and in high power mode.
 * @brief Sets the part to execute self-test routine
 *
 * @param dev  		- The device structure.
 * @param st_mode 	- Enable/disable bit for self test.
 * @param st_force 	- Enable/disable bit for forced self test.
 * @param st_dir 	- Direction bit for self test. 0 - Forward, 1 - Reverse.
 *
 * @return ret 		- Result of the writing procedure.
*******************************************************************************/
int adxl38x_set_self_test_registers(struct adxl38x_dev *dev, bool st_mode,
				    bool st_force, bool st_dir)
{
	int ret;
	uint8_t st_fields_value = 0;

	if (st_mode)
		st_fields_value |= BIT(7);
	if (st_force)
		st_fields_value |= BIT(6);
	if (st_dir)
		st_fields_value |= BIT(5);

	ret = adxl38x_register_update_bits(dev, ADXL38X_SNSR_AXIS_EN,
					   ADXL38X_SLF_TST_CTRL_MSK, st_fields_value);

	return ret;
}

/***************************************************************************//**
 * NOTE: It is recommended that the ADXL380 self-test should be performed in mid
 * and high gee ranges and in high power mode.
 * @brief Resets the self test registers for the part.
 *
 * @param dev  - The device structure.
 *
 * @return ret - Result of the writing procedure.
*******************************************************************************/
int adxl38x_clear_self_test_registers(struct adxl38x_dev *dev)
{
	int ret;

	ret = adxl38x_register_update_bits(dev, ADXL38X_SNSR_AXIS_EN,
					   ADXL38X_SLF_TST_CTRL_MSK, ADXL38X_RESET_ZERO);

	return ret;
}

/***************************************************************************//**
 * TODO: Update according to datasheet (Currently based on RevH)
 * @brief Reads the status registers
 *
 * @param dev          - The device structure.
 * @param status_flags - Combined Status value.
 *
 * @return ret         - 32-bit status value as a combination of status 0 to 3
 * 						 registers.
*******************************************************************************/
int adxl38x_get_sts_reg(struct adxl38x_dev *dev,
			union adxl38x_sts_reg_flags *status_flags)
{
	int ret;
	uint8_t status_value[4];

	ret = adxl38x_read_device_data(dev, ADXL38X_STATUS0, 4, status_value);
	if (ret)
		return ret;
	status_flags->value = get_be32(status_value);

	return 0;
}

/***************************************************************************//**
 * @brief Reads the raw output data using continuous read.
 *
 * @param dev   - The device structure.
 * @param raw_x - X-axis's raw output data.
 * @param raw_y - Y-axis's raw output data.
 * @param raw_z - Z-axis's raw output data.
 *
 * @return ret  - Result of the reading procedure.
*******************************************************************************/
int adxl38x_get_raw_xyz(struct adxl38x_dev *dev, uint16_t *raw_x,
			uint16_t *raw_y, uint16_t *raw_z)
{
	int ret;
	uint8_t array_raw_data[6] = {0};
	uint8_t op_mode;

	// Enable XYZ channels
	ret = adxl38x_register_update_bits(dev, ADXL38X_DIG_EN,
					   ADXL38X_MASK_CHEN_DIG_EN,
					   FIELD_PREP(ADXL38X_MASK_CHEN_DIG_EN, ADXL38X_CH_EN_XYZ));
	if (ret)
		return ret;

	// Put part in measurement mode if not placed in any measurement mode
	ret = adxl38x_get_op_mode(dev, &op_mode);
	if (ret)
		return ret;
	if (op_mode == 0) {
		adxl38x_set_op_mode(dev, ADXL38X_MODE_HP);
	}

	// Read accel data channels
	ret = adxl38x_read_device_data(dev, ADXL38X_XDATA_H, 6, array_raw_data);
	if (ret)
		return ret;
	*raw_x = get_be16(array_raw_data);
	*raw_y = get_be16(array_raw_data + 2);
	*raw_z = get_be16(array_raw_data + 4);

	return 0;
}

/***************************************************************************//**
 * @brief Reads the raw temperature data using burst read.
 *
 * @param dev   		- The device structure.
 * @param temp_degC 	- Temperature data in degree Celcius
 *
 * @return ret  	- Result of the reading procedure.
*******************************************************************************/
int adxl38x_get_temp(struct adxl38x_dev *dev,
		     struct adxl38x_fractional_val *temp_degC)
{
	int ret;
	uint8_t op_mode;
	uint8_t array_raw_data[2] = {0};

	// Enable T channel only
	ret = adxl38x_register_update_bits(dev, ADXL38X_DIG_EN,
					   ADXL38X_MASK_CHEN_DIG_EN,
					   FIELD_PREP(ADXL38X_MASK_CHEN_DIG_EN, ADXL38X_CH_EN_T));
	if (ret)
		return ret;

	// Put part in measurement mode if not placed in any measurement mode
	ret = adxl38x_get_op_mode(dev, &op_mode);
	if (ret)
		return ret;
	if (op_mode == 0) {
		adxl38x_set_op_mode(dev, ADXL38X_MODE_HP);
	}

	//Read accel temperature channel
	ret = adxl38x_read_device_data(dev, ADXL38X_TDATA_H, 2, array_raw_data);
	if (ret)
		return ret;
	temp_degC->integer = get_be16(array_raw_data) >> 4;

	//LSB to C conversion
	temp_degC->integer = temp_degC->integer - ADXL38X_TEMP_OFFSET;
	temp_degC->fractional = (temp_degC->integer * ADXL38X_TEMP_SCALE_DEN) %
				ADXL38X_TEMP_SCALE_NUM;
	temp_degC->integer = (temp_degC->integer * ADXL38X_TEMP_SCALE_DEN) /
			     ADXL38X_TEMP_SCALE_NUM;

	return ret;
}

/***************************************************************************//**
 * @brief Reads the raw output data using burst read.
 *
 * @param dev   	- The device structure.
 * @param channels 	- Channels to enable.
 * @param raw_x 	- X-axis's raw output data.
 * @param raw_y 	- Y-axis's raw output data.
 * @param raw_z 	- Z-axis's raw output data.
 * @param raw_temp 	- Raw temp output data.
 *
 * @return ret  	- Result of the reading procedure.
*******************************************************************************/
int adxl38x_get_raw_data(struct adxl38x_dev *dev,
			 enum adxl38x_ch_select channels,
			 uint16_t *raw_x, uint16_t *raw_y,
			 uint16_t *raw_z, uint16_t *raw_temp)
{
	uint8_t array_raw_data[8] = {0};
	uint8_t array_rearranged_data[8] = {0};
	int ret;
	int j = 0;
	uint8_t op_mode;
	uint8_t tzyx, start_addr;
	uint16_t num_bytes = 0;

	// Enable given channel(s)
	ret = adxl38x_register_update_bits(dev, ADXL38X_DIG_EN,
					   ADXL38X_MASK_CHEN_DIG_EN,
					   FIELD_PREP(ADXL38X_MASK_CHEN_DIG_EN, channels));
	if (ret)
		return ret;

	// Put part in measurement mode if not placed in any measurement mode
	ret = adxl38x_get_op_mode(dev, &op_mode);
	if (ret)
		return ret;
	if (op_mode == 0) {
		adxl38x_set_op_mode(dev, ADXL38X_MODE_HP);
	}

	//Find the channels and number of bytes to read
	tzyx = (uint8_t)channels;
	if (tzyx & 0x01) {
		start_addr = ADXL38X_XDATA_H;
	} else if (tzyx & 0x02) {
		start_addr = ADXL38X_YDATA_H;
	} else if (tzyx & 0x04) {
		start_addr = ADXL38X_ZDATA_H;
	} else {
		start_addr = ADXL38X_TDATA_H;
	}
	while (tzyx) {
		num_bytes += tzyx & 0x01;
		tzyx >>= 1;
	}
	num_bytes = num_bytes * 2;

	//Read accel data channels
	ret = adxl38x_read_device_data(dev, start_addr, num_bytes, array_raw_data);
	if (ret)
		return ret;
	//Re-assign raw data to corresponding channels
	tzyx = (uint8_t)channels;
	for (int i = 0; i <  8; i += 2) {
		if (tzyx & 0x01) {
			array_rearranged_data[i] = array_raw_data[j];
			array_rearranged_data[i + 1] = array_raw_data[j + 1];
			j += 2;
		} else {
			array_rearranged_data[i] = 0;
			array_rearranged_data[i + 1] = 0;
		}
		tzyx >>= 1;
	}
	if (raw_x)
		*raw_x = get_be16(array_rearranged_data);
	if (raw_y)
		*raw_y = get_be16(array_rearranged_data + 2);
	if (raw_z)
		*raw_z = get_be16(array_rearranged_data + 4);
	if (raw_temp)
		*raw_temp = get_be16(array_rearranged_data + 6) >> 4;

	return ret;
}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to g.
 *
 * @param dev  		- The device structure.
 * @param channels 	- Channels to enable.
 * @param x    		- X-axis's output data.
 * @param y    		- Y-axis's output data.
 * @param z    		- Z-axis's output data.
 *
 * @return ret 		- Result of the reading procedure.
*******************************************************************************/
int adxl38x_get_xyz_gees(struct adxl38x_dev *dev,
			 enum adxl38x_ch_select channels,
			 struct adxl38x_fractional_val *x, struct adxl38x_fractional_val *y,
			 struct adxl38x_fractional_val *z)
{
	int ret;
	uint16_t raw_accel_x;
	uint16_t raw_accel_y;
	uint16_t raw_accel_z;

	ret = adxl38x_get_raw_data(dev, channels, &raw_accel_x, &raw_accel_y,
				   &raw_accel_z, NULL);
	if (ret)
		return ret;

   /* 转换并计算整数部分和小数部分 */
    int64_t conv_x = adxl38x_accel_conv(dev, raw_accel_x);
    x->integer = conv_x / ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;
    x->fractional = conv_x % ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;

    int64_t conv_y = adxl38x_accel_conv(dev, raw_accel_y);
    y->integer = conv_y / ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;
    y->fractional = conv_y % ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;

    int64_t conv_z = adxl38x_accel_conv(dev, raw_accel_z);
    z->integer = conv_z / ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;
    z->fractional = conv_z % ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;

	return ret;
}

/***************************************************************************//**
 * @brief Converts raw acceleration value to m/s^2 value.
 *
 * @param dev       - The device structure.
 * @param raw_accel - Raw acceleration value.
 *
 * @return ret      - Converted data.
*******************************************************************************/
static int64_t adxl38x_accel_conv(struct adxl38x_dev *dev,
				  uint16_t raw_accel)
{
	int32_t accel_data;

	// Raw acceleration is in two's complement
	// Convert from two's complement to int
	if (raw_accel & BIT(15))
		accel_data = raw_accel | ADXL38X_NEG_ACC_MSK;
	else
		accel_data = raw_accel;

	// Apply scale factor based on the selected range
	switch (dev->dev_type) {
	case ID_ADXL380:
		return ((int64_t)(accel_data * ADXL380_ACC_SCALE_FACTOR_GEE_MUL *
				  adxl38x_scale_mul[dev->range]));
	case ID_ADXL382:
		return ((int64_t)(accel_data * ADXL382_ACC_SCALE_FACTOR_GEE_MUL *
				  adxl38x_scale_mul[dev->range]));
	default:
		return -1;
	}
}

/***************************************************************************//**
 * @brief Puts the part in a safe state before setting important register values.
 *
 * @param dev       - The device structure.
 *
 * @return ret      - Result of setting to stanby mode.
*******************************************************************************/
static int adxl38x_set_to_standby(struct adxl38x_dev *dev)
{
	int ret;
	uint8_t op_mode_reg_val;

	ret = adxl38x_read_device_data(dev, ADXL38X_OP_MODE, 1, &op_mode_reg_val);
	// Applying mask sets underlying op_mode to standby (0)
	op_mode_reg_val = (op_mode_reg_val & ~ADXL38X_MASK_OP_MODE);
	ret |= adxl38x_write_device_data(dev, ADXL38X_OP_MODE,
					 1, &op_mode_reg_val);
	if (!ret)
		dev->op_mode = FIELD_GET(ADXL38X_MASK_OP_MODE, op_mode_reg_val);

	return ret;
}

/***************************************************************************//**
 * @brief Executed Selftest on the sensing axes and returns the outcome of
 * the test.
 *
 * @param dev   	- The device structure.
 * @param op_mode  - Operation mode in which self-test is executed
 * @param st_x 		- Result of X-axis self test success/failure
 * @param st_y 		- Result of X-axis self test success/failure
 * @param st_z 		- Result of X-axis self test success/failure
 *
 * @return ret  	- Outcome of the selftest.
*******************************************************************************/
int adxl38x_selftest(struct adxl38x_dev *dev, enum adxl38x_op_mode op_mode,
		     bool *st_x, bool *st_y, bool *st_z)
{
	int ret;
	uint8_t array_raw_data[6] = {0};
	int32_t low_limit_xy, low_limit_z;
	int32_t up_limit_xy, up_limit_z;
	int32_t st_delta_data[3] = {0};
	int32_t st_positive_data[3] = {0};
	int32_t st_negative_data[3] = {0};
	int64_t x_sum = 0;
	int64_t y_sum = 0;
	int64_t z_sum = 0;
	*st_x = false;
	*st_y = false;
	*st_z = false;

	/* Enter standby mode */
	ret = adxl38x_set_op_mode(dev, ADXL38X_MODE_STDBY);

	/* Enable desired channels (except temperature) */
	if (op_mode == ADXL38X_MODE_HRT_SND) {
		// Only one axis
		ret = adxl38x_register_update_bits(dev, ADXL38X_DIG_EN,
						   ADXL38X_MASK_CHEN_DIG_EN,
						   FIELD_PREP(ADXL38X_MASK_CHEN_DIG_EN, ADXL38X_CH_EN_X));
	} else {
		ret = adxl38x_register_update_bits(dev, ADXL38X_DIG_EN,
						   ADXL38X_MASK_CHEN_DIG_EN,
						   FIELD_PREP(ADXL38X_MASK_CHEN_DIG_EN, ADXL38X_CH_EN_XYZ));
	}
	if (ret)
		return ret;

	/* Enable desired OP mode */
	ret = adxl38x_set_op_mode(dev, op_mode);
	if (ret)
		return ret;

	/* Enable ST mode, force and positive direction */
	ret = adxl38x_set_self_test_registers(dev, true, true, false);
	if (ret)
		return ret;

	/* Measure output on all three axes for at least 25 samples and compute average*/
	for (int k = 0; k < 25; k++) {
		HAL_Delay(10);
		ret = adxl38x_read_device_data(dev, ADXL38X_XDATA_H, 6, array_raw_data);

		x_sum += (int64_t)(get_be16(array_raw_data));
		y_sum += (int64_t)(get_be16(array_raw_data + 2));
		z_sum += (int64_t)(get_be16(array_raw_data + 4));
	}
	st_positive_data[0] = sign_extend32(x_sum / 25, 15);
	st_positive_data[1] = sign_extend32(y_sum / 25, 15);
	st_positive_data[2] = sign_extend32(z_sum / 25, 15);

	/* Enable ST mode, force and negative direction */
	ret = adxl38x_set_self_test_registers(dev, true, true, true);
	if (ret)
		return ret;

	/* Measure output on all three axes for at lest 25 samples and compute average*/
	x_sum = 0;
	y_sum = 0;
	z_sum = 0;
	for (int k = 0; k < 25; k++) {
		HAL_Delay(10);
		ret = adxl38x_read_device_data(dev, ADXL38X_XDATA_H, 6, array_raw_data);

		x_sum += (int64_t)(get_be16(array_raw_data));
		y_sum += (int64_t)(get_be16(array_raw_data + 2));
		z_sum += (int64_t)(get_be16(array_raw_data + 4));
	}
	st_negative_data[0] = sign_extend32(x_sum / 25, 15);
	st_negative_data[1] = sign_extend32(y_sum / 25, 15);
	st_negative_data[2] = sign_extend32(z_sum / 25, 15);

	/* Compute self-test delta */
	// Multiply self test denomitator for the values to be comparable
	for (int k = 0; k < 3; k++) {
		st_delta_data[k] = (int32_t) abs(st_positive_data[k] - st_negative_data[k]);
		st_delta_data[k] = st_delta_data[k] * ADXL38X_ST_LIMIT_DENOMINATOR;
	}

	/* Compare self-test delta magnitude with Datasheet values */
	// Apply scale factor based on the selected range
	switch (dev->dev_type) {
	case ID_ADXL380:
		low_limit_xy = (ADXL380_XY_ST_LIMIT_MIN * ADXL380_ACC_SENSITIVITY) >>
			       (dev->range);
		up_limit_xy = (ADXL380_XY_ST_LIMIT_MAX * ADXL380_ACC_SENSITIVITY) >>
			      (dev->range);
		low_limit_z = (ADXL380_Z_ST_LIMIT_MIN * ADXL380_ACC_SENSITIVITY) >>
			      (dev->range);
		up_limit_z = (ADXL380_Z_ST_LIMIT_MAX * ADXL380_ACC_SENSITIVITY) >> (dev->range);
		break;
	case ID_ADXL382:
		low_limit_xy = (ADXL382_XY_ST_LIMIT_MIN * ADXL382_ACC_SENSITIVITY) >>
			       (dev->range);
		up_limit_xy = (ADXL382_XY_ST_LIMIT_MAX * ADXL382_ACC_SENSITIVITY) >>
			      (dev->range);
		low_limit_z = (ADXL382_Z_ST_LIMIT_MIN * ADXL382_ACC_SENSITIVITY) >>
			      (dev->range);
		up_limit_z = (ADXL382_Z_ST_LIMIT_MAX * ADXL382_ACC_SENSITIVITY) >> (dev->range);
		break;
	default:
		return -1;
	}

	if (st_delta_data[0] >= low_limit_xy && st_delta_data[0] <= up_limit_xy)
		*st_x = true;
	if (st_delta_data[1] >= low_limit_xy && st_delta_data[1] <= up_limit_xy)
		*st_y = true;
	if (st_delta_data[2] >= low_limit_z && st_delta_data[2] <= up_limit_z)
		*st_z = true;

	/* Reset ST bits */
	ret = adxl38x_set_self_test_registers(dev, false, false, false);
	if (ret)
		return ret;

	return 0;
}

/***************************************************************************//**
 * @brief Function to set the paramenters for FIFO mode
 *
 * @param dev        		- The device structure.
 * @param num_samples 		- Number of FIFO entries that FIFI_WATERMARK should set.
 * @param external_trigger 	- Enable/disable external trigger in FIFO stream mode.
 * @param fifo_mode        	- FIFO mode setting.
 * @param ch_ID_enable      - Enable/disable channel ID.
 * @param read_reset       	- reset read/write point and read state machine.
 *
 * @return ret      		- Result of the procedure.
*******************************************************************************/
int adxl38x_accel_set_FIFO(struct adxl38x_dev *dev, uint16_t num_samples,
			   bool external_trigger, enum adxl38x_fifo_mode fifo_mode, bool ch_ID_enable,
			   bool read_reset)
{
	int ret;
	uint8_t write_data = 0;
	uint8_t fifo_samples_low;
	uint8_t fifo_samples_high;
	uint8_t set_channels;

	// Obtain the channels enabled in DIG_EN register
	ret = adxl38x_read_device_data(dev, ADXL38X_DIG_EN, 1, &set_channels);
	if (ret)
		return ret;
	set_channels = FIELD_GET(ADXL38X_MASK_CHEN_DIG_EN, set_channels);

	// Check if number of samples provided is allowed
	if (num_samples > 320)
		return -1;
	else if ((num_samples > 318) &&
		 ((!set_channels) || (set_channels == ADXL38X_CH_EN_XYZ) ||
		  (set_channels == ADXL38X_CH_EN_YZT)))
		return -1;

	// set FIFO_CFG1 register
	fifo_samples_low = (uint8_t) num_samples & 0xFF;
	ret = adxl38x_write_device_data(dev, ADXL38X_FIFO_CFG1, 1, &fifo_samples_low);
	if (ret)
		return ret;

	// building data for FIFO_CFG0 register
	fifo_samples_high = (uint8_t) num_samples >> 8;
	fifo_samples_high = fifo_samples_high & BIT(0);
	write_data = fifo_samples_high;

	fifo_mode = FIELD_PREP(ADXL38X_FIFOCFG_FIFOMODE_MSK, fifo_mode);
	write_data |= fifo_mode;

	if (read_reset)
		write_data |= BIT(7);

	if (ch_ID_enable)
		write_data |= BIT(6);

	if (external_trigger && fifo_mode == ADXL38X_FIFO_TRIGGER)
		write_data |= BIT(3);

	ret = adxl38x_write_device_data(dev, ADXL38X_FIFO_CFG0, 1, &write_data);

	return ret;
}

/***************************************************************************//**
 * @brief Function to convert accel data to gees
 *
 * @param dev        		- The device structure.
 * @param raw_accel_data 	- Raw data array of two bytes
 * @param data_frac        	- Fractional data in gees
 *
 * @return ret      		- Result of the procedure.
*******************************************************************************/
int adxl38x_data_raw_to_gees(struct adxl38x_dev *dev, uint8_t *raw_accel_data,
			     struct adxl38x_fractional_val *data_frac)
{
    uint16_t data = get_be16(raw_accel_data);
    /* 先做原始值到“刻度后”（scale）整数的转换 */
    int64_t conv = adxl38x_accel_conv(dev, data);
    /* 整数部分 = conv / 缩放因子 */
    data_frac->integer = conv / ADXL38X_ACC_SCALE_FACTOR_GEE_DIV;
    /* 小数部分 = conv % 缩放因子（对负数也要取绝对值） */
    data_frac->fractional = llabs(conv % ADXL38X_ACC_SCALE_FACTOR_GEE_DIV);
	return 0;
}

/***************************************************************************//**
 * @brief Assigns axis based on channel index
 *
 * @param chID         - Channel index
 *
 * @return ret         - Corresponding channel ID for channel index provided
*******************************************************************************/
char getaxis(uint8_t chID)
{
	if (chID)
		return chID > 1 ? 'z' : 'y';
	return 'x';
}

inline uint16_t get_be16(const uint8_t *buf) {
    return (uint16_t)buf[0] << 8 | buf[1];
}
inline uint32_t get_be32(const uint8_t *buf) {
    return (uint32_t)buf[0] << 24 | (uint32_t)buf[1] << 16 |
           (uint32_t)buf[2] << 8  | buf[3];
}
inline int32_t sign_extend32(int32_t val, int bits) {
    int32_t m = 1U << (bits - 1);
    return (val ^ m) - m;
}
