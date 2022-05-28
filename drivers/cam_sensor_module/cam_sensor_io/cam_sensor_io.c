// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */

#include "cam_sensor_io.h"
#include "cam_sensor_i2c.h"

#ifdef CONFIG_CAM_DISTINGUISH_SENSOR_VERSION

#define SPECIAL_SENSOR_ID 0x5041
#define SPECIAL_SENSOR_ADDR_ID 0x300B
#define SPECIAL_SENSOR_ADDR_VERSION 0x900E
#define SPECIAL_SENSOR_IIC_ADDR 0x10

static uint32_t s_r900e_val = 0;

struct cam_sensor_i2c_reg_array special_on_write_settings =  {0x0100, 0x01, 0x05, 0xFF};
struct cam_sensor_i2c_reg_array special_off_write_settings = {0x0100, 0x00, 0x00, 0xFF};

struct cam_sensor_i2c_reg_array special_streamon_write_settings[] =
{
	{0x7278, 0x00, 0x00, 0xFF},
	{0x727a, 0x01, 0x00, 0xFF},
	{0x7280, 0x07, 0x00, 0xFF},
	{0x0100, 0x01, 0x00, 0xFF}
};
#endif

int32_t camera_io_dev_poll(struct camera_io_master *io_master_info,
	uint32_t addr, uint16_t data, uint32_t data_mask,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type,
	uint32_t delay_ms)
{
	int16_t mask = data_mask & 0xFF;

	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_cci_i2c_poll(io_master_info->cci_client,
			addr, data, mask, data_type, addr_type, delay_ms);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_poll(io_master_info->client,
			addr, data, data_mask, addr_type, data_type,
			delay_ms);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
}

int32_t camera_io_dev_erase(struct camera_io_master *io_master_info,
	uint32_t addr, uint32_t size)
{
	int rc = 0;

	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (size == 0)
		return rc;

	if (io_master_info->master_type == SPI_MASTER) {
		CAM_DBG(CAM_SENSOR, "Calling SPI Erase");
		return cam_spi_erase(io_master_info, addr,
			CAMERA_SENSOR_I2C_TYPE_WORD, size);
	} else if (io_master_info->master_type == I2C_MASTER ||
		io_master_info->master_type == CCI_MASTER) {
		CAM_ERR(CAM_SENSOR, "Erase not supported on master :%d",
			io_master_info->master_type);
		rc = -EINVAL;
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		rc = -EINVAL;
	}
	return rc;
}

int32_t camera_io_dev_read(struct camera_io_master *io_master_info,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
#ifdef CONFIG_CAM_DISTINGUISH_SENSOR_VERSION
	int32_t retval = 0;
	uint32_t r900e = 0;
	struct cam_sensor_i2c_reg_setting write_setting = {0};
#endif

	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
#ifdef CONFIG_CAM_DISTINGUISH_SENSOR_VERSION
		retval = cam_cci_i2c_read(io_master_info->cci_client,
			addr, data, addr_type, data_type);
#else
		return cam_cci_i2c_read(io_master_info->cci_client,
			addr, data, addr_type, data_type);
#endif
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_read(io_master_info->client,
			addr, data, addr_type, data_type);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_read(io_master_info,
			addr, data, addr_type, data_type);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}

#ifdef CONFIG_CAM_DISTINGUISH_SENSOR_VERSION
	if (retval == 0 &&
		data != NULL &&
		*data == SPECIAL_SENSOR_ID &&
		addr == SPECIAL_SENSOR_ADDR_ID &&
		io_master_info->cci_client != NULL &&
		io_master_info->cci_client->cci_i2c_master == 0 &&
		io_master_info->cci_client->sid == SPECIAL_SENSOR_IIC_ADDR)
	{
		write_setting.size = 1;
		write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		write_setting.delay = 21;
		write_setting.read_buff = NULL;
		write_setting.read_buff_len = 0;
		write_setting.reg_setting = &special_on_write_settings;
		retval = cam_cci_i2c_write_table(io_master_info, &write_setting);

		retval = cam_cci_i2c_read(io_master_info->cci_client,
				SPECIAL_SENSOR_ADDR_VERSION,
				&r900e,
				CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_BYTE);

		CAM_INFO(CAM_SENSOR, "retval = %d, r900e = %d", retval, r900e);

		if(retval == 0)
			s_r900e_val = r900e;

		write_setting.size = 1;
		write_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		write_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		write_setting.delay = 0;
		write_setting.read_buff = NULL;
		write_setting.read_buff_len = 0;
		write_setting.reg_setting = &special_off_write_settings;
		retval = cam_cci_i2c_write_table(io_master_info, &write_setting);
	}

	return retval;
#else
	return 0;
#endif
}

int32_t camera_io_dev_read_seq(struct camera_io_master *io_master_info,
	uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type, int32_t num_bytes)
{
	if (io_master_info->master_type == CCI_MASTER) {
		return cam_camera_cci_i2c_read_seq(io_master_info->cci_client,
			addr, data, addr_type, data_type, num_bytes);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_read_seq(io_master_info->client,
			addr, data, addr_type, num_bytes);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_read_seq(io_master_info,
			addr, data, addr_type, num_bytes);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_write_seq(io_master_info,
			addr, data, addr_type, num_bytes);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
	return 0;
}

int32_t camera_io_dev_ois_read_seq(struct camera_io_master *io_master_info,
	uint32_t addr, uint8_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type, int32_t num_bytes)
{
	if (io_master_info->master_type == CCI_MASTER) {
		return cam_camera_cci_i2c_ois_read_seq(io_master_info->cci_client,
			addr, data, addr_type, data_type, num_bytes);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_read_seq(io_master_info->client,
			addr, data, addr_type, num_bytes);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_read_seq(io_master_info,
			addr, data, addr_type, num_bytes);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_write_seq(io_master_info,
			addr, data, addr_type, num_bytes);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
	return 0;
}

int32_t camera_io_dev_write(struct camera_io_master *io_master_info,
	struct cam_sensor_i2c_reg_setting *write_setting)
{
#ifdef CONFIG_CAM_DISTINGUISH_SENSOR_VERSION
	int32_t retval = 0;
#endif

	if (!write_setting || !io_master_info) {
		CAM_ERR(CAM_SENSOR,
			"Input parameters not valid ws: %pK ioinfo: %pK",
			write_setting, io_master_info);
		return -EINVAL;
	}

	if (!write_setting->reg_setting) {
		CAM_ERR(CAM_SENSOR, "Invalid Register Settings");
		return -EINVAL;
	}

#ifdef CONFIG_CAM_DISTINGUISH_SENSOR_VERSION
	if (io_master_info->cci_client != NULL &&
		io_master_info->cci_client->sid== SPECIAL_SENSOR_IIC_ADDR &&
		io_master_info->cci_client->cci_device == 1 &&
		io_master_info->cci_client->cci_i2c_master == 0 &&
		write_setting->reg_setting->reg_addr == 0x0100 &&
		write_setting->reg_setting->reg_data == 0x01)
	{
		CAM_INFO(CAM_SENSOR, "Detect stream on setting, r900e = %d", s_r900e_val);

		if (s_r900e_val == 0x02)
		{
			write_setting->reg_setting->reg_addr = special_streamon_write_settings[0].reg_addr;
			write_setting->reg_setting->reg_data = special_streamon_write_settings[0].reg_data;
			write_setting->reg_setting->delay = special_streamon_write_settings[0].delay;
			write_setting->reg_setting->data_mask = special_streamon_write_settings[0].data_mask;
			retval = cam_cci_i2c_write_table(io_master_info, write_setting);

			write_setting->reg_setting->reg_addr = special_streamon_write_settings[1].reg_addr;
			write_setting->reg_setting->reg_data = special_streamon_write_settings[1].reg_data;
			write_setting->reg_setting->delay = special_streamon_write_settings[1].delay;
			write_setting->reg_setting->data_mask = special_streamon_write_settings[1].data_mask;
			retval = cam_cci_i2c_write_table(io_master_info, write_setting);

			write_setting->reg_setting->reg_addr = special_streamon_write_settings[2].reg_addr;
			write_setting->reg_setting->reg_data = special_streamon_write_settings[2].reg_data;
			write_setting->reg_setting->delay = special_streamon_write_settings[2].delay;
			write_setting->reg_setting->data_mask = special_streamon_write_settings[2].data_mask;
			retval = cam_cci_i2c_write_table(io_master_info, write_setting);

			write_setting->reg_setting->reg_addr = special_streamon_write_settings[3].reg_addr;
			write_setting->reg_setting->reg_data = special_streamon_write_settings[3].reg_data;
			write_setting->reg_setting->delay = special_streamon_write_settings[3].delay;
			write_setting->reg_setting->data_mask = special_streamon_write_settings[3].data_mask;
		}
	}
#endif

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_cci_i2c_write_table(io_master_info,
			write_setting);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_write_table(io_master_info,
			write_setting);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_write_table(io_master_info,
			write_setting);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
}

int32_t camera_io_dev_write_continuous(struct camera_io_master *io_master_info,
	struct cam_sensor_i2c_reg_setting *write_setting,
	uint8_t cam_sensor_i2c_write_flag)
{
	if (!write_setting || !io_master_info) {
		CAM_ERR(CAM_SENSOR,
			"Input parameters not valid ws: %pK ioinfo: %pK",
			write_setting, io_master_info);
		return -EINVAL;
	}

	if (!write_setting->reg_setting) {
		CAM_ERR(CAM_SENSOR, "Invalid Register Settings");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_cci_i2c_write_continuous_table(io_master_info,
			write_setting, cam_sensor_i2c_write_flag);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_write_continuous_table(io_master_info,
			write_setting, cam_sensor_i2c_write_flag);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_write_table(io_master_info,
			write_setting);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
}

int32_t camera_io_init(struct camera_io_master *io_master_info)
{
	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		io_master_info->cci_client->cci_subdev =
		cam_cci_get_subdev(io_master_info->cci_client->cci_device);
		return cam_sensor_cci_i2c_util(io_master_info->cci_client,
			MSM_CCI_INIT);
	} else if ((io_master_info->master_type == I2C_MASTER) ||
			(io_master_info->master_type == SPI_MASTER)) {
		return 0;
	}

	return -EINVAL;
}

int32_t camera_io_release(struct camera_io_master *io_master_info)
{
	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_sensor_cci_i2c_util(io_master_info->cci_client,
			MSM_CCI_RELEASE);
	} else if ((io_master_info->master_type == I2C_MASTER) ||
			(io_master_info->master_type == SPI_MASTER)) {
		return 0;
	}

	return -EINVAL;
}
