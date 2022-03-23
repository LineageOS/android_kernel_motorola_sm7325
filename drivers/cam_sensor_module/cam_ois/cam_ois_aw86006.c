/*
 * File: cam_ois_aw86006.c
 *
 * Author: hushanping <hushanping@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <asm/arch_timer.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_ois_dev.h"
#include "cam_cci_dev.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "cam_ois_aw86006.h"

#define AW86006_DRIVER_VERSION		"v0.4.7"
#define AW86006_FW_NAME			"mot_aw86006.prog"

const char fw_check_str[] = { 'A', 'W', 'I', 'N', 'I', 'C', 0, 0 };
uint8_t g_cci_freq = I2C_FAST_PLUS_MODE;
struct cam_ois_ctrl_t *o_ctrl_g;
static struct class *ois_debug_class;
struct aw86006_info g_aw86006_info = {
	.checkinfo_fw = { 0 },
	.checkinfo_rd = { 0 },
	.fw = { 0 },
};

static int ois_block_read_addr8_data8(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t addr_type, uint8_t *data, uint32_t num_byte)
{
	enum i2c_freq_mode temp_freq;
	int ret = -1;


	if (o_ctrl == NULL || data == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args o_ctrl: %pK, data: %pK",
			o_ctrl, data);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_ERR(CAM_OIS, "Not in right state to start soc reads: %d",
			 o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;
	/* Modify i2c freq to 100K */
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_STANDARD_MODE;
	ret = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, data,
		addr_type, CAMERA_SENSOR_I2C_TYPE_BYTE, num_byte);
	if (ret < 0)
		CAM_ERR(CAM_OIS, "err! ret:%d", ret);
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;

	return ret;
}

static int ois_block_write_addr8_data8(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t addr_type, uint8_t *data, uint32_t num_byte)
{
	int ret = -1;
	int cnt;
	enum i2c_freq_mode temp_freq;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;


	if (o_ctrl == NULL || data == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args o_ctrl: %pK, data: %pK",
			o_ctrl, data);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_ERR(CAM_OIS, "Not in right state to start soc writes: %d",
			 o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = g_cci_freq;
	i2c_reg_setting.addr_type = addr_type;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = num_byte;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting =
	    (struct cam_sensor_i2c_reg_array *)
	    kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * num_byte,
		    GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL) {
		CAM_ERR(CAM_OIS, "kzalloc failed");
		return OIS_ERROR;
	}
	i2c_reg_setting.reg_setting[0].reg_addr = addr;
	i2c_reg_setting.reg_setting[0].reg_data = data[0];
	i2c_reg_setting.reg_setting[0].delay = 0;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	for (cnt = 1; cnt < num_byte; cnt++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = 0;
		i2c_reg_setting.reg_setting[cnt].reg_data = data[cnt];
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}
	ret = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
					   &i2c_reg_setting, 0);
	if (ret < 0)
		CAM_ERR(CAM_OIS, "err! ret:%d", ret);
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;
	kfree(i2c_reg_setting.reg_setting);


	return ret;
}

static int aw_reset(struct cam_ois_ctrl_t *o_ctrl)
{
	int ret = OIS_ERROR;
	uint16_t temp_addr = 0;
	uint8_t boot_cmd_1[] = { 0xFF, 0xF0, 0x20, 0x20, 0x02, 0x02, 0x19, 0x29,
								0x19, 0x29 };
	uint8_t boot_cmd_2[] = { 0xFF, 0xC4, 0xC4 };
	uint8_t boot_cmd_3[] = { 0xFF, 0xC4, 0x00 };
	uint8_t boot_cmd_4[] = { 0xFF, 0x10, 0xC3 };

	CAM_INFO(CAM_OIS, "enter");
	/* first: shutdown */
	temp_addr = o_ctrl->io_master_info.cci_client->sid;
	o_ctrl->io_master_info.cci_client->sid = AW_SHUTDOWN_I2C_ADDR;
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_1[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_1[1], sizeof(boot_cmd_1) - 1);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "boot_cmd_1 write error!");
		goto err_exit;
	}
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_2[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_2[1], sizeof(boot_cmd_2) - 1);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "boot_cmd_2 write error!");
		goto err_exit;
	}
	usleep_range(AW_SHUTDOWN_DELAY, AW_SHUTDOWN_DELAY + 50); /* 2 ms */
	/* second: wake up */
	o_ctrl->io_master_info.cci_client->sid = AW_WAKEUP_I2C_ADDR;
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_3[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_3[1], sizeof(boot_cmd_3) - 1);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "boot_cmd_3 write error!");
		goto err_exit;
	}
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_4[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_4[1], sizeof(boot_cmd_4) - 1);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "boot_cmd_4 write error!");
		goto err_exit;
	}

	o_ctrl->io_master_info.cci_client->sid = temp_addr;
	return OIS_SUCCESS;
 err_exit:
	o_ctrl->io_master_info.cci_client->sid = temp_addr;
	return OIS_ERROR;
}

static int aw_runtime_check(struct cam_ois_ctrl_t *o_ctrl)
{
	int ret = OIS_ERROR;
	uint32_t version = 0;
	uint8_t reg_val[4] = { 0 };
	uint8_t chip_id = 0;

	ret = ois_block_read_addr8_data8(o_ctrl, REG_CHIPID,
			CAMERA_SENSOR_I2C_TYPE_WORD, &chip_id, 1);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "read chipid error!");
		return ret;
	}

	ret = ois_block_read_addr8_data8(o_ctrl, REG_VERSION,
			CAMERA_SENSOR_I2C_TYPE_WORD, &reg_val[0], 4);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "read version error!");
		return ret;
	}
	version = (reg_val[3] << AW_DATA_SHIFT_24_BIT) |
					(reg_val[2] << AW_DATA_SHIFT_16_BIT) |
					(reg_val[1] << AW_DATA_SHIFT_8_BIT) |
					(reg_val[0] << AW_DATA_SHIFT_0_BIT);
	CAM_INFO(CAM_OIS, "Chip_ID: 0x%02X, Version: v%d.%d.%d.%d", chip_id,
				reg_val[3], reg_val[2], reg_val[1], reg_val[0]);

	if ((chip_id != g_aw86006_info.fw.app_id)
				|| (version != g_aw86006_info.fw.app_version)) {
		CAM_ERR(CAM_OIS, "Chip_ID or Version not match!");
		return OIS_ERROR;
	}
	CAM_INFO(CAM_OIS, "pass!");

	return OIS_SUCCESS;
}

static int aw_soc_buf_build(struct cam_ois_ctrl_t *o_ctrl, uint8_t *buf,
			    struct soc_protocol *soc_struct)
{
	int i = 0;
	uint8_t *p_head = (uint8_t *)soc_struct;
	uint8_t checksum = 0;
	uint8_t data_sum = 0;

	if ((buf == NULL) || ((soc_struct == NULL)))
		return OIS_ERROR;
	if (soc_struct->p_data == NULL)
		soc_struct->len[0] = 0;
	soc_struct->protocol_ver = SOC_VERSION;
	soc_struct->ack = SOC_ACK;
	soc_struct->addr =
		((soc_struct->ack_juge == SOC_CTL) ? SOC_ADDR : SOC_READ_ADDR);
	for (i = 0; i < soc_struct->len[0]; i++) {
		data_sum += soc_struct->p_data[i];
		buf[i + SOC_PROTOCAL_HEAD] = soc_struct->p_data[i];
	}
	soc_struct->sum = data_sum;
	for (i = 1; i < SOC_PROTOCAL_HEAD; i++) {
		checksum += p_head[i];
		buf[i] = p_head[i];
	}
	soc_struct->checksum = checksum;
	buf[0] = p_head[0];

	return OIS_SUCCESS;
}

static int aw_soc_connect_check(struct cam_ois_ctrl_t *o_ctrl)
{
	struct soc_protocol soc_struct = { 0 };
	int ret = OIS_ERROR;
	uint8_t w_buf[14] = { 0 };
	uint8_t r_buf[14] = { 0 };
	uint8_t cmp_buf[5] = {0x00, 0x01, 0x00, 0x00, 0x00};

	soc_struct.module = SOC_HANK;

	soc_struct.event = SOC_HANK_CONNECT;
	soc_struct.len[0] = 0;
	soc_struct.p_data = NULL;
	soc_struct.ack_juge = SOC_CTL;

	memset(w_buf, 0, sizeof(w_buf));
	aw_soc_buf_build(o_ctrl, w_buf, &soc_struct);
	ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE, &w_buf[1],
		SOC_CONNECT_WRITE_LEN - 1);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "write connect w_buf error!");
		return ret;
	}

	usleep_range(SOC_CONNECT_DELAY, SOC_CONNECT_DELAY + 50);

	memset(r_buf, 0, sizeof(r_buf));
	ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
		CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 14);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "read connect r_buf error!");
		return ret;
	}
	/* regroup w_buf */
	soc_struct.event = SOC_HANK_CONNECT_ACK;
	soc_struct.ack_juge = SOC_ACK;
	soc_struct.len[0] = 5;
	soc_struct.p_data = cmp_buf;

	memset(w_buf, 0, sizeof(w_buf));
	aw_soc_buf_build(o_ctrl, w_buf, &soc_struct);
	ret = memcmp(w_buf, r_buf, sizeof(r_buf));
	if (ret != 0) {
		CAM_ERR(CAM_OIS, "connect memcmp error!");
		return OIS_ERROR;
	}

	return OIS_SUCCESS;
}

static int aw_soc_flash_read_check(struct cam_ois_ctrl_t *o_ctrl,
		uint32_t addr, uint8_t *bin_buf, uint32_t len)
{
	struct soc_protocol soc_struct = { 0 };
	int i = 0;
	int ret = OIS_ERROR;
	int loop = 0;
	uint8_t temp_buf[SOC_READ_STRUCT_LEN] = { 0 };
	uint8_t w_buf[SOC_READ_WRITE_LEN] = { 0 };
	uint8_t r_buf[80] = { 0 };
	uint8_t checksum = 0;

	temp_buf[0] = len;
	temp_buf[1] = 0;
	for (i = 0; i < 4; i++)
		temp_buf[i + 2] = (uint8_t) (addr >> (i * 8));
	soc_struct.module = SOC_FLASH;

	do {
		soc_struct.event = SOC_FLASH_READ;
		soc_struct.len[0] = SOC_READ_STRUCT_LEN;
		soc_struct.p_data = temp_buf;

		memset(w_buf, 0, sizeof(w_buf));
		aw_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
			CAMERA_SENSOR_I2C_TYPE_BYTE, &w_buf[1], sizeof(w_buf) - 1);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "i2c write w_buf error, loop: %d", loop);
			continue;
		}
		usleep_range(SOC_READ_BLOCK_DELAY, SOC_READ_BLOCK_DELAY + 50);

		memset(r_buf, 0, sizeof(r_buf));
		ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
			CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 14 + len);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "i2c read r_buf error, loop: %d", loop);
			continue;
		}
		/* check error flag */
		if (r_buf[9] != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "error flag wrong: %d, loop: %d",
							r_buf[10], loop);
			continue;
		}
		checksum = 0;
		/* compute data checksum */
		for (i = 0; i < len + 5; i++)
			checksum += r_buf[9 + i];
		if (checksum != r_buf[8]) {
			CAM_ERR(CAM_OIS, "data checksum error:0x%02x != 0x%02x, loop:%d",
				checksum, r_buf[8], loop);
			continue;
		}
		checksum = 0;
		/* compute head checksum */
		for (i = 1; i < 9; i++)
			checksum += r_buf[i];
		if (checksum != r_buf[0]) {
			CAM_ERR(CAM_OIS, "head checksum error:0x%02x != 0x%02x, loop:%d",
				checksum, r_buf[0], loop);
			continue;
		}
		memcpy(bin_buf, (uint8_t *)&r_buf[14], len);
		break; /* Check pass */
	} while ((++loop) < AW_FLASH_READ_ERROR_LOOP);
	if (loop >= AW_FLASH_READ_ERROR_LOOP)
		return OIS_ERROR;

	return OIS_SUCCESS;
}

static int aw_soc_flash_write_check(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t block_num, uint8_t *bin_buf, uint32_t len)
{
	struct soc_protocol soc_struct = { 0 };
	int i = 0;
	int ret = OIS_ERROR;
	int loop = 0;

#ifdef AW_FLASH_WRITE_CONNECT
	int connect_loop = 0;
#endif

	uint8_t temp_buf[68] = { 0 };
	uint8_t w_buf[77] = { 0 };
	uint8_t r_buf[10] = { 0 };
	uint8_t cmp_buf[1] = { 0 };

	for (i = 0; i < 4; i++)
		temp_buf[i] = (uint8_t) ((addr +
				block_num * AW_FLASH_WRITE_LEN) >> (i * 8));
	for (i = 0; i < len; i++)
		temp_buf[i + 4] = (bin_buf + block_num * AW_FLASH_WRITE_LEN)[i];

	soc_struct.module = SOC_FLASH;

#ifdef AW_FLASH_WRITE_CONNECT
	do {
		ret = aw_soc_connect_check(o_ctrl);
		if (ret == OIS_SUCCESS)
			break;
		CAM_ERR(CAM_OIS, "connect_loop error ret: %d, connect_loop: %d",
							ret, connect_loop);
	} while ((++connect_loop) < AW_ERROR_LOOP);
	if (connect_loop >= AW_ERROR_LOOP)
		return OIS_ERROR;
#endif

	do {
		soc_struct.event = SOC_FLASH_WRITE;
		soc_struct.len[0] = (uint8_t) (4 + len);
		soc_struct.p_data = temp_buf;
		soc_struct.ack_juge = SOC_CTL;

		memset(w_buf, 0, sizeof(w_buf));
		aw_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
					CAMERA_SENSOR_I2C_TYPE_BYTE, &w_buf[1],
					SOC_WRITE_BLOCK_HEAD - 1 + len);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "i2c write w_buf error, loop: %d", loop);
			continue;
		}
		usleep_range(SOC_WRITE_BLOCK_DELAY, SOC_WRITE_BLOCK_DELAY + 50);

		memset(r_buf, 0, sizeof(r_buf));
		ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
				CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 10);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "i2c read r_buf error, loop: %d", loop);
			continue;
		}
		/* regroup w_buf */
		soc_struct.event = SOC_FLASH_WRITE_ACK;
		soc_struct.len[0] = 1;
		soc_struct.p_data = cmp_buf;
		soc_struct.ack_juge = SOC_ACK;

		memset(w_buf, 0, sizeof(w_buf));
		aw_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = memcmp(w_buf, r_buf, sizeof(r_buf));
		if (ret == 0)
			break;
		CAM_ERR(CAM_OIS, "memcmp error! ret=%d, loop: %d",ret , loop);
	} while ((++loop) < AW_FLASH_WRITE_ERROR_LOOP);
	if (loop >= AW_FLASH_WRITE_ERROR_LOOP)
		return OIS_ERROR;

	return OIS_SUCCESS;
}

static int aw_soc_flash_erase_check(struct cam_ois_ctrl_t *o_ctrl,
						uint32_t addr, uint32_t len)
{
	struct soc_protocol soc_struct = { 0 };
	uint32_t erase_block = 0;
	int i = 0;
	int ret = OIS_ERROR;
	int loop = 0;
	uint8_t temp_buf[6] = { 0 };
	uint8_t cmp_buf[1] = { 0 };
	uint8_t w_buf[15] = { 0 };
	uint8_t r_buf[10] = { 0 };

	erase_block = len / AW_FLASH_ERASE_LEN +
					((len % AW_FLASH_ERASE_LEN) ? 1 : 0);
	temp_buf[0] = (uint8_t) erase_block;
	temp_buf[1] = 0x00;
	for (i = 0; i < 4; i++)
		temp_buf[i + 2] = (uint8_t) (addr >> (i * 8));

	soc_struct.module = SOC_FLASH;

	do {
		soc_struct.event = SOC_FLASH_ERASE_BLOCK;
		soc_struct.len[0] = SOC_ERASE_STRUCT_LEN;
		soc_struct.p_data = temp_buf;
		soc_struct.ack_juge = SOC_CTL;

		memset(w_buf, 0, sizeof(w_buf));
		aw_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
					CAMERA_SENSOR_I2C_TYPE_BYTE, &w_buf[1],
					SOC_ERASE_WRITE_LEN - 1);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "i2c write w_buf error, loop: %d", loop);
			continue;
		}
		msleep(erase_block * SOC_ERASE_BLOCK_DELAY);

		memset(r_buf, 0, sizeof(r_buf));
		ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
				CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 10);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "i2c read r_buf error, loop: %d", loop);
			continue;
		}
		soc_struct.event = SOC_FLASH_ERASE_BLOCK_ACK;
		soc_struct.len[0] = 1;
		soc_struct.ack_juge = SOC_ACK;
		soc_struct.p_data = cmp_buf;

		memset(w_buf, 0, sizeof(w_buf));
		aw_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = memcmp(w_buf, r_buf, sizeof(r_buf));
		if (ret == 0)
			break;
		CAM_ERR(CAM_OIS, "memcmp error!, loop: %d", loop);
	} while ((++loop) < AW_FLASH_ERASE_ERROR_LOOP);
	if (loop >= AW_FLASH_ERASE_ERROR_LOOP)
		return OIS_ERROR;

	CAM_INFO(CAM_OIS, "flash erase success!, addr: 0x%08x, len: 0x%08x",
								addr, len);
	return OIS_SUCCESS;
}

static int aw_soc_flash_download_check(struct cam_ois_ctrl_t *o_ctrl,
				uint32_t addr, uint8_t *bin_buf, size_t len)
{
	uint32_t flash_block = 0;
	uint32_t flash_tail = 0;
	uint32_t flash_checkinfo = 0;
	int i = 0;
	int ret = OIS_ERROR;

	flash_block = len / AW_FLASH_WRITE_LEN;
	flash_tail = len % AW_FLASH_WRITE_LEN;

	if (addr == AW_FLASH_BASE_ADDR) {
		flash_checkinfo = AW_FLASH_MOVE_LENGTH / AW_FLASH_WRITE_LEN;
		/* first erase app+info data */
		ret = aw_soc_flash_erase_check(o_ctrl, AW_FLASH_APP_ADDR,
						len - AW_FLASH_MOVE_LENGTH);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "Failed to erase app+checkinfo!");
			return ret;
		}
		/* then erase move data */
		ret = aw_soc_flash_erase_check(o_ctrl, AW_FLASH_BASE_ADDR,
			AW_FLASH_MOVE_LENGTH);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "Failed to erase move!");
			return ret;
		}
		/* write move data */
		for (i = 0; i < flash_checkinfo; i++) {
			ret = aw_soc_flash_write_check(o_ctrl, addr, i, bin_buf,
							AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "Failed to write flash block: %d", i);
				return ret;
			}
		}
		/* write app block data */
		for (i = flash_checkinfo + 1; i < flash_block; i++) {
			ret = aw_soc_flash_write_check(o_ctrl, addr, i, bin_buf,
				AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "Failed to write flash block: %d", i);
				return ret;
			}
		}
		/* write app tail data */
		if ((flash_tail != 0) && (len > AW_FLASH_MOVE_LENGTH)) {
			ret = aw_soc_flash_write_check(o_ctrl, addr, i, bin_buf,
				flash_tail);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "Failed to write flash app tail");
				return ret;
			}
		}
	} else if (addr == AW_FLASH_APP_ADDR) {
		flash_checkinfo = 0;
		/* erase app+info data */
		ret = aw_soc_flash_erase_check(o_ctrl, AW_FLASH_APP_ADDR, len);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "Failed to erase app+checkinfo!");
			return ret;
		}
		/* write app block data */
		for (i = 1; i < flash_block; i++) {
			ret = aw_soc_flash_write_check(o_ctrl, addr, i, bin_buf,
				AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "Failed to write flash block: %d", i);
				return ret;
			}
		}
		/* write app tail data */
		if (flash_tail != 0) {
			ret = aw_soc_flash_write_check(o_ctrl, addr, i, bin_buf,
								flash_tail);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "Failed to write flash tail");
				return ret;
			}
		}
	} else {
		CAM_ERR(CAM_OIS, "wrong addr!");
		return OIS_ERROR;
	}

	/* Write checkinfo data */
	ret = aw_soc_flash_write_check(o_ctrl, AW_FLASH_APP_ADDR, 0,
						g_aw86006_info.checkinfo_fw,
						AW_FLASH_WRITE_LEN);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "Failed to write checkinfo!");
		return ret;
	}

	return OIS_SUCCESS;
}

static int aw_jump_move_check(struct cam_ois_ctrl_t *o_ctrl)
{
	ktime_t kstart;
	ktime_t kend;
	uint32_t move_version = 0;
	uint32_t ms_count_reset = 0;
	uint32_t ms_count_stop = 0;
	uint32_t ms_first_f0 = 0;
	int ret = OIS_ERROR;
	int i = 0;
	int jump_loop = 0;
	uint8_t move_cmd[] = {0xF0, 0xF0, 0xF0};
	uint8_t version_cmd[] = {0x00, 0x55};
	uint8_t version_ack[5] = {0x00};

	do {
		ret = aw_reset(o_ctrl);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "aw_reset error, ret: %d, loop: %d",
								ret, jump_loop);
			continue;
		}
		/* delay: 9ms */
		kstart = ktime_get();
		for (i = 0; i < AW_JUMP_MOVE_DELAY_MAX; i++) {
			mdelay(1);
			kend = ktime_get();
			ms_count_reset = ktime_to_ms(ktime_sub(kend, kstart));
			if (ms_count_reset >= AW_JUMP_MOVE_DELAY)
				break;
		}
		if (ms_count_reset >= AW_JUMP_MOVE_DELAY_MAX) {
			CAM_ERR(CAM_OIS, "ms_count_reset timeout: %d, loop: %d",
						ms_count_reset, jump_loop);
			continue;
		}
		/* send jump cmd */
		for (i = 0; i < AW_STAY_ON_MOVE_LOOP; i++) { /* loop 20 */
			ois_block_write_addr8_data8(o_ctrl, move_cmd[0],
				CAMERA_SENSOR_I2C_TYPE_BYTE, &move_cmd[1],
				sizeof(move_cmd) - 1);
			if (i == 0) {
				kend = ktime_get();
				ms_first_f0 =
					ktime_to_ms(ktime_sub(kend, kstart));
			}
		}

		kend = ktime_get();
		ms_count_stop = ktime_to_ms(ktime_sub(kend, kstart));
		CAM_INFO(CAM_OIS, "ms_count_reset: %u, ms_count_stop: %u",
						ms_count_reset, ms_count_stop);

		if (ms_first_f0 > AW_JUMP_MOVE_DELAY_MAX) {
			CAM_INFO(CAM_OIS, "first 0xF0: %d ms", ms_first_f0);
			continue;
		}

		/* send read move version cmd */
		ret = ois_block_write_addr8_data8(o_ctrl, version_cmd[0],
			CAMERA_SENSOR_I2C_TYPE_BYTE, &version_cmd[1],
			sizeof(version_cmd) - 1);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "write version_cmd, loop: %d", jump_loop);
			continue;
		}
		usleep_range(ISP_READ_VERSION_DELAY,
						ISP_READ_VERSION_DELAY + 50);
		ret = ois_block_read_addr8_data8(o_ctrl, ISP_VERS_CONNECT_ACK,
					CAMERA_SENSOR_I2C_TYPE_BYTE,
					&version_ack[0], ISP_VERSION_ACK_LEN);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "read version error, loop:%d", jump_loop);
			continue;
		}

		if (version_ack[0] != ISP_EVENT_OK) {
			CAM_ERR(CAM_OIS, "wrong version_ack: %d, loop: %d",
						version_ack[0], jump_loop);
			continue;
		}
		move_version = (version_ack[4] << AW_DATA_SHIFT_24_BIT) |
		    (version_ack[3] << AW_DATA_SHIFT_16_BIT) |
		    (version_ack[2] << AW_DATA_SHIFT_8_BIT) |
		    (version_ack[1] << AW_DATA_SHIFT_0_BIT);

		if (move_version == g_aw86006_info.fw.move_version)
			break;

		CAM_ERR(CAM_OIS, "move_version not match: 0x%08X != 0x%08X, loop:%d",
				move_version, g_aw86006_info.fw.move_version,
				jump_loop);
	} while ((++jump_loop) < AW_JUMP_MOVE_LOOP);
	if (jump_loop >= AW_JUMP_MOVE_LOOP)
		return OIS_ERROR;

	CAM_INFO(CAM_OIS, "Success! move_version: 0x%08X", move_version);

	return OIS_SUCCESS;
}

static int aw_jump_boot_check(struct cam_ois_ctrl_t *o_ctrl)
{
	ktime_t kend;
	ktime_t kstart;
	uint32_t ms_count_reset = 0;
	uint32_t ms_count_stop = 0;
	uint32_t ms_first_ac = 0;
	int ret = OIS_ERROR;
	int i = 0;
	int jump_loop = 0;
	uint8_t boot_cmd[] = {0xAC, 0xAC, 0xAC};

	do {
		ret = aw_reset(o_ctrl);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "aw_reset error, ret: %d, loop: %d",
							ret, jump_loop);
			continue;
		}
		/* delay: 2ms */
		kstart = ktime_get();
		for (i = 0; i < AW_JUMP_BOOT_DELAY_MAX; i++) {
			mdelay(1);
			kend = ktime_get();
			ms_count_reset = ktime_to_ms(ktime_sub(kend, kstart));
			if (ms_count_reset >= AW_JUMP_BOOT_DELAY)
				break;
		}
		if (ms_count_reset >= AW_JUMP_BOOT_DELAY_MAX) {
			CAM_ERR(CAM_OIS, "ms_count_reset timeout: %d, loop: %d",
				ms_count_reset, jump_loop);
			continue;
		}
		/* send jump boot cmd */
		for (i = 0; i < AW_STAY_ON_BOOT_LOOP; i++) {
			ois_block_write_addr8_data8(o_ctrl, boot_cmd[0],
				CAMERA_SENSOR_I2C_TYPE_BYTE,
				&boot_cmd[1], sizeof(boot_cmd) - 1);
			if (i == 0) {
				kend = ktime_get();
				ms_first_ac =
					ktime_to_ms(ktime_sub(kend, kstart));
			}
		}
		/* get the end time of send boot cmd */
		kend = ktime_get();
		ms_count_stop = ktime_to_ms(ktime_sub(kend, kstart));
		CAM_INFO(CAM_OIS, "ms_count_reset: %u, ms_count_stop: %u",
						 ms_count_reset, ms_count_stop);
		if (ms_first_ac > AW_JUMP_BOOT_DELAY_MAX) {
			CAM_INFO(CAM_OIS, "first 0xAC: %d ms", ms_first_ac);
			continue;
		}

		ret = aw_soc_connect_check(o_ctrl);
		if (ret == OIS_SUCCESS)
			break;

		CAM_ERR(CAM_OIS, "soc connect failed! loop:%d", jump_loop);
	} while ((++jump_loop) < AW_JUMP_BOOT_LOOP);
	if (jump_loop >= AW_JUMP_BOOT_LOOP)
		return OIS_ERROR;

	CAM_INFO(CAM_OIS, "success");

	return OIS_SUCCESS;
}

static int aw_soc_flash_update(struct cam_ois_ctrl_t *o_ctrl, uint32_t addr,
			       uint8_t *data_p, size_t fw_size)
{
	int ret = OIS_ERROR;
	int loop = 0;

	if (!data_p) {
		CAM_ERR(CAM_OIS, "data_p is NULL");
		return OIS_ERROR;
	}
	/* enter boot mode */
	ret = aw_jump_boot_check(o_ctrl);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "jump boot failed!");
		return OIS_ERROR;
	}
	/* update flash */
	do {
		ret = aw_soc_flash_download_check(o_ctrl, addr, data_p, fw_size);
		if (ret == OIS_SUCCESS)
			break;
		CAM_ERR(CAM_OIS, "aw_soc_flash_download_check failed! loop:%d", loop);
	} while ((++loop) < AW_ERROR_LOOP);
	if (loop >= AW_ERROR_LOOP)
		return OIS_ERROR;

	ret = aw_reset(o_ctrl);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "aw_reset failed!");
		return ret;
	}
	msleep(AW_RESET_DELAY);

	CAM_INFO(CAM_OIS, "success!");

	return OIS_SUCCESS;
}

static int aw_firmware_check(struct cam_ois_ctrl_t *o_ctrl,
				const struct firmware *fw)
{
	uint32_t temp = 0;
	uint32_t *check_ptr = NULL;
	int i = 0;
	int ret = 0;
	char *identify = (char *)fw->data + AW_FW_SHIFT_IDENTIFY;

	if (fw == NULL) {
		CAM_ERR(CAM_OIS, "FW Empty!!!");
		return OIS_ERROR;
	}

	ret = memcmp(fw_check_str, identify, 8); /* AWINIC00 8bytes */
	if (ret != 0) {
		CAM_ERR(CAM_OIS, "loaded wrong firmware!");
		return OIS_ERROR;
	}

	/* get fw info */
	g_aw86006_info.fw.size = fw->size;
	g_aw86006_info.fw.checksum =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_CHECKSUM]);
	g_aw86006_info.fw.app_checksum =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_APP_CHECKSUM]);
	/* app_length: app + check_info */
	g_aw86006_info.fw.app_length =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_APP_LENGTH]) +
							AW_FW_INFO_LENGTH;
	g_aw86006_info.fw.app_version =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_APP_VERSION]);
	g_aw86006_info.fw.app_id =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_APP_ID]);
	g_aw86006_info.fw.move_checksum =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_MOVE_CHECKSUM]);
	g_aw86006_info.fw.move_version =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_MOVE_VERSION]);
	g_aw86006_info.fw.move_length =
		*(uint32_t *) (&fw->data[AW_FW_SHIFT_MOVE_LENGTH]);

	CAM_INFO(CAM_OIS, "fw->size:0x%04X, app_length:0x%04X, app_version:0x%08X, app_id:0x%02X, move_version:0x%08X, move_length:0x%04X",
		g_aw86006_info.fw.size, g_aw86006_info.fw.app_length,
		g_aw86006_info.fw.app_version, g_aw86006_info.fw.app_id,
		g_aw86006_info.fw.move_version, g_aw86006_info.fw.move_length);

	/* length check */
	temp = g_aw86006_info.fw.move_length + g_aw86006_info.fw.app_length;
	if (g_aw86006_info.fw.size != temp) {
		CAM_ERR(CAM_OIS, "fw->size error: 0x%X != 0x%X",
						g_aw86006_info.fw.size, temp);
		return OIS_ERROR;
	}

	/* move checksum check */
	check_ptr = (uint32_t *) &fw->data[0];
	temp = 0;
	for (i = 0; i < (g_aw86006_info.fw.move_length / sizeof(uint32_t)); i++)
		temp += check_ptr[i];
	if (temp != g_aw86006_info.fw.move_checksum) {
		CAM_ERR(CAM_OIS, "move checksum error:0x%08X != 0x%08X",
					g_aw86006_info.fw.move_checksum, temp);
		return OIS_ERROR;
	}

	/* info checksum check */
	check_ptr = (uint32_t *) &fw->data[AW_FW_SHIFT_CHECKSUM_ADDR];
	temp = 0;
	/* identify length:8bytes; info_checksum length:4byes */
	for (i = 0; i < ((AW_FW_INFO_LENGTH - 8 - 4) / sizeof(uint32_t)); i++)
		temp += check_ptr[i];
	if (temp != g_aw86006_info.fw.checksum) {
		CAM_ERR(CAM_OIS, "check_info checksum error:0x%08X != 0x%08X",
			g_aw86006_info.fw.checksum, temp);
		return OIS_ERROR;
	}

	/* app checksum check */
	check_ptr = (uint32_t *) &fw->data[AW_FLASH_MOVE_LENGTH +
							AW_FW_INFO_LENGTH];
	temp = 0;
	for (i = 0; i < ((g_aw86006_info.fw.app_length - AW_FW_INFO_LENGTH) /
							sizeof(uint32_t)); i++)
		temp += check_ptr[i];

	if (temp != g_aw86006_info.fw.app_checksum) {
		CAM_ERR(CAM_OIS, "app checksum error:0x%08X != 0x%08X",
			g_aw86006_info.fw.app_checksum, temp);
		return OIS_ERROR;
	}
	/* Save firmware checkinfo */
	memcpy(&g_aw86006_info.checkinfo_fw[0],
		fw->data + AW_FLASH_MOVE_LENGTH, AW_FW_INFO_LENGTH);

	CAM_INFO(CAM_OIS, "pass!");

	return OIS_SUCCESS;
}

static int aw_checkinfo_analyse(struct cam_ois_ctrl_t *o_ctrl,
				uint8_t *checkinfo_rd, struct aw86006_fw *info)
{
	uint32_t temp = 0;
	uint32_t *check_ptr = NULL;
	int i = 0;
	int ret = 0;

	if ((checkinfo_rd == NULL) || (info == NULL)) {
		CAM_ERR(CAM_OIS, "checkinfo empty!");
		return OIS_ERROR;
	}
	/* compare identify: AWINIC00 */
	ret = memcmp(fw_check_str, (char *)checkinfo_rd, 8);
	if (ret != 0) {
		CAM_ERR(CAM_OIS, "checkinfo not match!");
		return OIS_ERROR;
	}

	info->checksum = *(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_CHECKSUM -
							AW_FLASH_MOVE_LENGTH]);
	info->app_checksum =
		*(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_APP_CHECKSUM -
							AW_FLASH_MOVE_LENGTH]);
	info->app_length = *(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_APP_LENGTH -
				AW_FLASH_MOVE_LENGTH]) + AW_FW_INFO_LENGTH;
	info->app_version =
		*(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_APP_VERSION -
							AW_FLASH_MOVE_LENGTH]);
	info->app_id = *(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_APP_ID -
							AW_FLASH_MOVE_LENGTH]);
	info->move_checksum =
		*(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_MOVE_CHECKSUM -
							AW_FLASH_MOVE_LENGTH]);
	info->move_version =
		*(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_MOVE_VERSION -
							AW_FLASH_MOVE_LENGTH]);
	info->move_length =
		*(uint32_t *) (&checkinfo_rd[AW_FW_SHIFT_MOVE_LENGTH -
							AW_FLASH_MOVE_LENGTH]);

	CAM_INFO(CAM_OIS, "checkinfo app_length:0x%04X, app_version:0x%08X, app_id:0x%02X, move_version:0x%08X, move_length:0x%04X",
		info->app_length, info->app_version, info->app_id,
		info->move_version, info->move_length);

	/* info checksum check */
	check_ptr = (uint32_t *) &checkinfo_rd[AW_FW_SHIFT_CHECKSUM_ADDR -
							AW_FLASH_MOVE_LENGTH];
	temp = 0;
	for (i = 0; i < ((AW_FW_INFO_LENGTH - 8 - 4) / sizeof(uint32_t)); i++)
		temp += check_ptr[i];
	if (temp != info->checksum) {
		CAM_ERR(CAM_OIS, "checkinfo_rd checksum error:0x%08X != 0x%08X",
			info->checksum, temp);
		return OIS_ERROR;
	}
	CAM_INFO(CAM_OIS, "pass!");
	return OIS_SUCCESS;
}

static int aw_get_standby_flag(struct cam_ois_ctrl_t *o_ctrl, uint8_t *pflag)
{
	int ret = 0;
	uint8_t flag = 0;
	uint8_t temp_addr = 0;

	temp_addr = o_ctrl->io_master_info.cci_client->sid;
	o_ctrl->io_master_info.cci_client->sid = AW_SHUTDOWN_I2C_ADDR;
	ret = ois_block_read_addr8_data8(o_ctrl, 0xFF11,
			CAMERA_SENSOR_I2C_TYPE_WORD, &flag, 1);
	o_ctrl->io_master_info.cci_client->sid = temp_addr;
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "read standby flag error, ret: %d", ret);
		return ret;
	}

	*pflag = flag;
	CAM_INFO(CAM_OIS, "standby flag: %d", flag);

	return OIS_SUCCESS;
}

static int aw86006_mem_download(struct cam_ois_ctrl_t *o_ctrl,
				const struct firmware *fw)
{
	struct aw86006_fw info_rd;
	size_t all_buf_size = g_aw86006_info.fw.app_length +
							AW_FLASH_MOVE_LENGTH;
	size_t app_buf_size = g_aw86006_info.fw.app_length;
	int i = 0;
	int ret = OIS_ERROR;
	uint8_t *all_buf_ptr = (uint8_t *) fw->data;
	uint8_t *app_buf_ptr = (uint8_t *) fw->data + AW_FLASH_MOVE_LENGTH;

	if (fw == NULL) {
		CAM_ERR(CAM_OIS, "FW Empty!!!");
		return OIS_ERROR;
	}

	/* enter boot mode */
	ret = aw_jump_boot_check(o_ctrl);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "jump boot failed!");
		return OIS_ERROR;
	}
#ifdef AW_SOC_FLASH_READ_FLAG
	/* get check_info data */
	ret = aw_soc_flash_read_check(o_ctrl, AW_FLASH_APP_ADDR,
		&g_aw86006_info.checkinfo_rd[0], AW_FLASH_READ_LEN);
	if (ret != OIS_SUCCESS)
		CAM_ERR(CAM_OIS, "checkinfo read error!");
#endif
	g_aw86006_info.fw.update_flag =
		g_aw86006_info.checkinfo_rd[AW_ARRAY_SHIFT_UPDATE_FLAG];

	if (g_aw86006_info.fw.update_flag != 0x01) {
		CAM_INFO(CAM_OIS, "update_flag not match, update all!");
		ret = aw_soc_flash_update(o_ctrl, AW_FLASH_BASE_ADDR,
					  all_buf_ptr, all_buf_size);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "update all failed!");
			return OIS_ERROR;
		}
	} else {
		CAM_INFO(CAM_OIS, "update_flag match!");
		ret = memcmp(g_aw86006_info.checkinfo_rd,
			     g_aw86006_info.checkinfo_fw, AW_FW_INFO_LENGTH);
		if (ret != 0) {
			ret = aw_checkinfo_analyse(o_ctrl,
					g_aw86006_info.checkinfo_rd, &info_rd);
			if ((ret != OIS_SUCCESS) || (info_rd.move_version !=
					g_aw86006_info.fw.move_version)) {
				CAM_ERR(CAM_OIS,
					"checkinfo or move not match, update all!");
				ret = aw_soc_flash_update(o_ctrl,
						AW_FLASH_BASE_ADDR,
						all_buf_ptr, all_buf_size);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "update all failed!");
					return OIS_ERROR;
				}
			} else if ((info_rd.app_version !=
					g_aw86006_info.fw.app_version) ||
					(info_rd.app_id !=
						g_aw86006_info.fw.app_id)) {
				CAM_ERR(CAM_OIS, "app not match, update app!");
				ret = aw_soc_flash_update(o_ctrl,
						AW_FLASH_APP_ADDR,
						app_buf_ptr, app_buf_size);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "update app failed!");
					return OIS_ERROR;
				}
			} else {
				CAM_ERR(CAM_OIS, "other error, update all!");
				ret = aw_soc_flash_update(o_ctrl,
						AW_FLASH_BASE_ADDR,
						all_buf_ptr, all_buf_size);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "update all failed!");
					return OIS_ERROR;
				}
			}
		} else {
			ret = aw_jump_move_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS,
					"aw_jump_move_check fail, update all!");
				ret = aw_soc_flash_update(o_ctrl,
					AW_FLASH_BASE_ADDR, all_buf_ptr,
					all_buf_size);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "update all failed!");
					return OIS_ERROR;
				}
			} else {
				ret = aw_reset(o_ctrl);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "aw_reset failed!");
					return OIS_ERROR;
				}
				msleep(AW_RESET_DELAY);
			}
		}
	}

	for (i = 0; i <= AW_ERROR_LOOP; i++) {
		ret = aw_runtime_check(o_ctrl);
		if (ret == OIS_SUCCESS) {
			CAM_INFO(CAM_OIS,
				 "runtime_check pass, no need to update fw!");
			break;
		}
		CAM_ERR(CAM_OIS,
			"runtime_check failed, update app! loop:%d", i);
		if (i == AW_ERROR_LOOP)
			break;

		ret = aw_soc_flash_update(o_ctrl, AW_FLASH_APP_ADDR, app_buf_ptr,
					  app_buf_size);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "update app failed!");
			break;
		}
	}

	return ret;
}

int aw_firmware_update(struct cam_ois_ctrl_t *o_ctrl, const struct firmware *fw)
{
//	const struct firmware *fw;
//	int loop = 0;
	int ret = OIS_ERROR;
	uint8_t standby_flag = 0;

	CAM_INFO(CAM_OIS, "start!");
	/* load firmware */
	/*
	do {
		ret = request_firmware(&fw, AW86006_FW_NAME, &o_ctrl->pdev->dev);
		if (ret == 0)
			break;
	} while ((++loop) < AW_ERROR_LOOP);
	if (loop >= AW_ERROR_LOOP) {
		CAM_ERR(CAM_OIS, "request_firmware [%s] failed!", AW86006_FW_NAME);
		return OIS_ERROR;
	}
	*/
	/* fw check */
	CAM_INFO(CAM_OIS, "Load:%s size:%zu", AW86006_FW_NAME, fw->size);
	ret = aw_firmware_check(o_ctrl, fw);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "fw check failed!");
		goto err_fw_check;
	}
	/* reset */
	ret = aw_reset(o_ctrl);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "reset failed");
		goto err_reset;
	}
	mdelay(AW_RESET_DELAY); /* run app after reset */
	/* Get standby flag */
	ret = aw_get_standby_flag(o_ctrl, &standby_flag);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "get standby flag failed");
		goto err_get_standby_flg;
	}
	if (standby_flag == AW_IC_STANDBY) {
		ret = aw_runtime_check(o_ctrl);
		if (ret == OIS_SUCCESS)
			CAM_INFO(CAM_OIS, "runtime_check pass, no need to update fw!");
	}
	/* update flash */
	if ((standby_flag != AW_IC_STANDBY) || (ret != OIS_SUCCESS)) {
		ret = aw86006_mem_download(o_ctrl, fw);
		if (ret == OIS_SUCCESS)
			CAM_INFO(CAM_OIS, "fw update success!");
		else
			CAM_ERR(CAM_OIS, "fw update failed, ret: %d", ret);
	}

err_get_standby_flg:
err_reset:
err_fw_check:
	release_firmware(fw);
	return ret;
}

void aw_firmware_update_work_routine(struct work_struct *work)
{
        struct cam_ois_ctrl_t *o_ctrl =
                container_of(work, struct cam_ois_ctrl_t, fw_update_work);
        const struct firmware *fw = NULL;
        int loop = 0;
        int ret = 0;

        CAM_INFO(CAM_OIS, "enter");

        // load firmware
        do {
                ret = request_firmware(&fw, AW86006_FW_NAME, &o_ctrl->pdev->dev);
                if (ret == 0)
                        break;
        } while ((++loop) < AW_ERROR_LOOP);
        if (loop >= AW_ERROR_LOOP) {
                CAM_ERR(CAM_OIS, "request_firmware [%s] failed! stop update firmware!", AW86006_FW_NAME);
        }
        else{
            mutex_lock(&o_ctrl->ois_mutex_aw);
            aw_firmware_update(o_ctrl,fw);
            mutex_unlock(&o_ctrl->ois_mutex_aw);
        }
}

/*******************************************************************************
 * adb debug interface
 ******************************************************************************/
static ssize_t reg_show(struct class *class,
			struct class_attribute *attr, char *buf)
{
	uint8_t reg_val = 0;
	ssize_t len = 0;
	uint32_t i = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	mutex_lock(&o_ctrl->ois_mutex_aw);
	for (i = 0; i < 20; i++) {
		ois_block_read_addr8_data8(o_ctrl, i, CAMERA_SENSOR_I2C_TYPE_WORD,
			&reg_val, 1);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%04X=0x%02X\n", i, reg_val);
	}
	mutex_unlock(&o_ctrl->ois_mutex_aw);
	return len;
}

static ssize_t reg_store(struct class *class,
			 struct class_attribute *attr,
			 const char *buf, size_t count)
{
	uint32_t databuf[2] = {0};
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		mutex_lock(&o_ctrl->ois_mutex_aw);
		ois_block_write_addr8_data8(o_ctrl, databuf[0],
			CAMERA_SENSOR_I2C_TYPE_WORD, (uint8_t *) &databuf[1],
			CAMERA_SENSOR_I2C_TYPE_BYTE);
		mutex_unlock(&o_ctrl->ois_mutex_aw);
	}

	return count;
}

static ssize_t awrw_show(struct class *class,
			 struct class_attribute *attr, char *buf)
{
	int i = 0;
	ssize_t len = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	if (o_ctrl->awrw_flag != AW_SEQ_READ) {
		CAM_ERR(CAM_OIS, "not read mode");
		return -ERANGE;
	}
	if (o_ctrl->reg_data == NULL) {
		CAM_ERR(CAM_OIS, "reg_data empty!");
		return -ERANGE;
	}
	for (i = 0; i < o_ctrl->reg_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"0x%02x,", o_ctrl->reg_data[i]);
	}
	len += snprintf(buf + len - 1, PAGE_SIZE - len, "\n");
	return len;
}

static ssize_t awrw_store(struct class *class, struct class_attribute *attr,
			  const char *buf, size_t count)
{
	uint8_t value = 0;
	char data_buf[5] = {0};
	int i = 0;
	int rc = 0;
	uint32_t flag = 0;
	uint32_t reg_num = 0;
	uint32_t reg_addr = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	if (sscanf(buf, "%x %x %x", &flag, &reg_num, &reg_addr) == 3) {
		if (!reg_num) {
			CAM_ERR(CAM_OIS, "reg_num param error");
			return -ERANGE;
		}
		mutex_lock(&o_ctrl->ois_mutex_aw);
		o_ctrl->awrw_flag = flag;
		o_ctrl->reg_num = reg_num;
		if (o_ctrl->reg_data != NULL)
			kfree(o_ctrl->reg_data);
		o_ctrl->reg_data = kmalloc(reg_num, GFP_KERNEL);
		if (flag == AW_SEQ_WRITE) {
			for (i = 0; i < reg_num; i++) {
				if ((i * 5 + 5) > (strlen(buf) - 2 * (4 + 1) - (6 + 1))) {
					CAM_ERR(CAM_OIS, "buf length error");
					mutex_unlock(&o_ctrl->ois_mutex_aw);
					return -ERANGE;
				}
				memcpy(data_buf, &buf[17 + i * 5], 4);
				data_buf[4] = '\0';
				rc = kstrtou8(data_buf, 0, &value);
				if (rc < 0) {
					CAM_ERR(CAM_OIS, "input buf error");
					mutex_unlock(&o_ctrl->ois_mutex_aw);
					return -ERANGE;
				}
				o_ctrl->reg_data[i] = value;
			}
			ois_block_write_addr8_data8(o_ctrl, reg_addr,
				CAMERA_SENSOR_I2C_TYPE_WORD, o_ctrl->reg_data, reg_num);
		} else if (flag == AW_SEQ_READ) {
			ois_block_read_addr8_data8(o_ctrl, reg_addr,
				CAMERA_SENSOR_I2C_TYPE_WORD, o_ctrl->reg_data, reg_num);
		}
		mutex_unlock(&o_ctrl->ois_mutex_aw);
	} else {
		CAM_ERR(CAM_OIS, "param error");
	}
	return count;
}

static ssize_t update_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"update ois: echo 1 > update\n");

	return len;
}

static ssize_t update_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	uint32_t databuf = 0;
	int ret = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	ret = kstrtouint(buf, 0, &databuf);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "kstrtouint fail");
		return ret;
	}

	/* fw update */
	if (databuf != 1) {
		CAM_INFO(CAM_OIS, "flag error: %d", databuf);
		return -EPERM;
	}

	schedule_work(&o_ctrl->fw_update_work);
	return count;
}

static ssize_t erase_show(struct class *class,
			  struct class_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"erase app flash: echo app > erase\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
			"erase all flash: echo all > erase\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
			"ois reset: echo reset > erase\n");

	return len;
}

static ssize_t erase_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	char databuf[10] = {0};
	char readbuf[64] = {0};
	int ret = OIS_ERROR;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	if (sscanf(buf, "%s", databuf) == 1) {
		mutex_lock(&o_ctrl->ois_mutex_aw);
		if (strcmp(databuf, "app") == 0) {
			CAM_INFO(CAM_OIS, "aw86006 will erase app!");
			ret = aw_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw_jump_boot_check error!");
				goto err_exit;
			}
			ret = aw_soc_flash_erase_check(o_ctrl, AW_FLASH_APP_ADDR,
						 AW_FLASH_FULL_SIZE -
						 AW_FLASH_MOVE_LENGTH);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw_soc_flash_erase_check error!");
				goto err_exit;
			}
		} else if (strcmp(databuf, "all") == 0) {
			CAM_INFO(CAM_OIS, "aw86006 will erase all!");
			ret = aw_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw_jump_boot_check error!");
				goto err_exit;
			}
			ret = aw_soc_flash_erase_check(o_ctrl, AW_FLASH_BASE_ADDR,
						 AW_FLASH_FULL_SIZE);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw_soc_flash_erase_check error!");
				goto err_exit;
			}
		} else if (strcmp(databuf, "read") == 0) {
			ret = aw_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw_jump_boot_check error!");
				goto err_exit;
			}
			ret = aw_soc_flash_read_check(o_ctrl, AW_FLASH_APP_ADDR,
				readbuf, AW_FLASH_READ_LEN);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw_soc_flash_erase_check error!");
				goto err_exit;
			}
		} else if (strcmp(databuf, "reset") == 0) {
			CAM_INFO(CAM_OIS, "aw86006 reset");
			ret = aw_reset(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw_reset failed!");
				goto err_exit;
			}
		} else {
			CAM_ERR(CAM_OIS, "Erase: %s is not support!", databuf);
		}

		mutex_unlock(&o_ctrl->ois_mutex_aw);
	}

	return count;

err_exit:
	mutex_unlock(&o_ctrl->ois_mutex_aw);
	return ret;
}

static ssize_t mode_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
					"OIS_ON: echo 1 > mode\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
					"OIS_OFF: echo 0 > mode\n");

	return len;
}

static ssize_t mode_store(struct class *class,
						struct class_attribute *attr,
						const char *buf, size_t count)
{
	uint8_t temp = 0;
	int ret = OIS_SUCCESS;
	int loop = 0;
	uint32_t ois_status = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	ret = kstrtou32(buf, 0, &ois_status);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "Number of parameters error");
		return ret;
	}
	if ((ois_status != OIS_DISABLE) && (ois_status != OIS_ENABLE)) {
		CAM_ERR(CAM_OIS, "ois_status error: %d", ois_status);
		return -EPERM;
	}
	/* enable OIS mode */
	mutex_lock(&o_ctrl->ois_mutex_aw);
	do {
		temp = (uint8_t)ois_status;
		CAM_INFO(CAM_OIS, "set ois status: %d", temp);

		ret = ois_block_write_addr8_data8(o_ctrl, REG_OIS_ENABLE,
					CAMERA_SENSOR_I2C_TYPE_WORD, &temp, 1);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Send OIS mode failed!");
			mutex_unlock(&o_ctrl->ois_mutex_aw);
			return ret;
		}
		/* 1000 us */
		usleep_range(1000, 1500);

		/* check OIS enable status */
		temp = 0;
		ret = ois_block_read_addr8_data8(o_ctrl, REG_OIS_ENABLE,
					CAMERA_SENSOR_I2C_TYPE_WORD, &temp, 1);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Read OIS mode error");
			mutex_unlock(&o_ctrl->ois_mutex_aw);
			return ret;
		}

		if (temp == ois_status) {
			CAM_INFO(CAM_OIS, "Set OIS mode success!");
			break;
		}
		CAM_ERR(CAM_OIS, "OIS mode check error! value: 0x%02x", temp);
	} while ((++loop) < AW_ERROR_LOOP);
	mutex_unlock(&o_ctrl->ois_mutex_aw);
	if (loop >= AW_ERROR_LOOP)
		return OIS_ERROR;

	return count;
}

static ssize_t chipid_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	uint8_t chipid = 0;
	ssize_t len = 0;
	int ret = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	mutex_lock(&o_ctrl->ois_mutex_aw);
	ret = ois_block_read_addr8_data8(o_ctrl, REG_CHIPID,
				CAMERA_SENSOR_I2C_TYPE_WORD, &chipid, 1);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "read chipid error");
		mutex_unlock(&o_ctrl->ois_mutex_aw);
		return ret;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "Chipid: 0x%02X\n", chipid);
	mutex_unlock(&o_ctrl->ois_mutex_aw);

	return len;
}

static ssize_t chipid_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	return count;
}

static ssize_t version_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	uint8_t temp[4] = { 0 };
	uint32_t version = 0;
	int ret = 0;
	ssize_t len = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	mutex_lock(&o_ctrl->ois_mutex_aw);
	ret = ois_block_read_addr8_data8(o_ctrl, REG_VERSION,
				CAMERA_SENSOR_I2C_TYPE_WORD, temp, 4);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "read version error");
		mutex_unlock(&o_ctrl->ois_mutex_aw);
		return ret;
	}

	version = (temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0];

	len += snprintf(buf + len, PAGE_SIZE - len, "version: v%d.%d.%d.%d\n",
					temp[3], temp[2], temp[1], temp[0]);
	mutex_unlock(&o_ctrl->ois_mutex_aw);

	return len;
}

static ssize_t version_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	return count;
}

static ssize_t checkinfo_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int i;

	for (i = 0; i < 8; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"checkinfo_rd_0x%x 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			i, g_aw86006_info.checkinfo_rd[i * 8 + 0],
			g_aw86006_info.checkinfo_rd[i * 8 + 1],
			g_aw86006_info.checkinfo_rd[i * 8 + 2],
			g_aw86006_info.checkinfo_rd[i * 8 + 3],
			g_aw86006_info.checkinfo_rd[i * 8 + 4],
			g_aw86006_info.checkinfo_rd[i * 8 + 5],
			g_aw86006_info.checkinfo_rd[i * 8 + 6],
			g_aw86006_info.checkinfo_rd[i * 8 + 7]);
		len += snprintf(buf + len, PAGE_SIZE - len,
			"checkinfo_fw_0x%x 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			i, g_aw86006_info.checkinfo_fw[i * 8 + 0],
			g_aw86006_info.checkinfo_fw[i * 8 + 1],
			g_aw86006_info.checkinfo_fw[i * 8 + 2],
			g_aw86006_info.checkinfo_fw[i * 8 + 3],
			g_aw86006_info.checkinfo_fw[i * 8 + 4],
			g_aw86006_info.checkinfo_fw[i * 8 + 5],
			g_aw86006_info.checkinfo_fw[i * 8 + 6],
			g_aw86006_info.checkinfo_fw[i * 8 + 7]);
	}

	return len;
}

static ssize_t checkinfo_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	uint32_t data[9] = {0};
	int i;

	if (sscanf(buf, "%x %x %x %x %x %x %x %x %x", &data[0], &data[1],
		   &data[2], &data[3], &data[4], &data[5], &data[6], &data[7],
		   &data[8]) == 9) {
		if (data[0] > 7) {
			CAM_ERR(CAM_OIS, "wrong data[0]:%d", data[0]);
			return count;
		}
		for (i = 0; i < 8; i++) {
			g_aw86006_info.checkinfo_rd[data[0] * 8 + i] =
			    (uint8_t) data[i + 1];
		}
	}
	return count;
}

static ssize_t freq_show(struct class *class,
			 struct class_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"g_cci_freq=%d\n", g_cci_freq);

	return len;
}

static ssize_t freq_store(struct class *class,
			  struct class_attribute *attr,
			  const char *buf, size_t count)
{
	uint32_t databuf[1];

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		if (databuf[0] < I2C_MAX_MODES)
			g_cci_freq = databuf[0];
	}

	return count;
}

static ssize_t jump_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"jump boot: echo boot > jump\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
			"jump move: echo move > jump\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
			"reset: echo reset > jump\n");

	return len;
}

static ssize_t jump_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	char databuf[10] = {0};
	int ret = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;

	if (sscanf(buf, "%s", databuf) == 1) {
		mutex_lock(&o_ctrl->ois_mutex_aw);
		if (strcmp(databuf, "boot") == 0) {
			CAM_INFO(CAM_OIS, "aw86006 will jump boot!");
			ret = aw_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "jump boot failed!");
				goto err_exit;
			}
		} else if (strcmp(databuf, "move") == 0) {
			CAM_INFO(CAM_OIS, "aw86006 will jump move!");
			ret = aw_jump_move_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "jump move failed!");
				goto err_exit;
			}
		} else if (strcmp(databuf, "reset") == 0) {
			CAM_INFO(CAM_OIS, "aw86006 will reset!");
			ret = aw_reset(o_ctrl);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "reset failed!");
				goto err_exit;
			}
		} else {
			CAM_ERR(CAM_OIS, "jump [%s] is not support!", databuf);
		}

		mutex_unlock(&o_ctrl->ois_mutex_aw);
	}

	return count;

err_exit:
	mutex_unlock(&o_ctrl->ois_mutex_aw);
	return ret;
}

static ssize_t gyro_offset_cali_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;
	uint8_t val[6] = { 0 };
	int16_t offset_val;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"gyro_offset: echo 1 > gyro_offset_cali\n");

	/* Gyro offset data */
	ois_block_read_addr8_data8(o_ctrl, 0xf84c,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[0], 4);
	offset_val = val[1]<<8 |val[0];
	len += snprintf(buf + len, PAGE_SIZE - len,
			"gyro x offset = %d\n", offset_val);

	offset_val = val[3]<<8 |val[2];
	len += snprintf(buf + len, PAGE_SIZE - len,
			"gyro y offset = %d\n", offset_val);

	return len;
}

static ssize_t gyro_offset_cali_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = o_ctrl_g;
	int i = 0;
	uint8_t data_buf[5] = { 0 };
	uint8_t val[15] = { 0 };
	uint8_t ois_mode = 0;
	int16_t offset_val = 0;

	if (sscanf(buf, "%d", &data_buf[0]) != 1) {
		CAM_ERR(CAM_OIS, "input para error!");
		return OIS_ERROR;
	}
	CAM_INFO(CAM_OIS, "aw86006 gyro offset cali start!");

	/* off OIS */
	ois_mode = OIS_DISABLE;
	ois_block_write_addr8_data8(o_ctrl, REG_OIS_ENABLE,
				CAMERA_SENSOR_I2C_TYPE_WORD, &ois_mode, 1);
	msleep(20);

	i = 0;
	do {
		val[0] = 0xa4;
		ois_block_write_addr8_data8(o_ctrl, 0xf20f,
					CAMERA_SENSOR_I2C_TYPE_WORD, &val[0], 1);
		msleep(20);
		ois_block_read_addr8_data8(o_ctrl, 0xf20f,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[0], 1);
		i++;
	} while ((val[0] != 1) && (i < (50 * AW_ERROR_LOOP)));
	if (val[0] != 1) {
		CAM_ERR(CAM_OIS, "read reg[0xf20f] != 0x01, i=%d,val[0]=%d",i,val[0] );
		return OIS_ERROR;
	}

	i = 0;
	do {
		val[1] = 0xac;
		ois_block_write_addr8_data8(o_ctrl, 0xf8ff,
					CAMERA_SENSOR_I2C_TYPE_WORD, &val[1], 1);
		msleep(20);
		ois_block_read_addr8_data8(o_ctrl, 0xf8ff,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[1], 1);
		i++;
	} while ((val[1] != 1) && (i <  (50 * AW_ERROR_LOOP)));
	if (val[1] != 1) {
		CAM_ERR(CAM_OIS, "read reg[0xf8ff] != 0x01, i=%d,val=%d",i,val[1] );
		return OIS_ERROR;
	}

	val[2] = 0x01;
	ois_block_write_addr8_data8(o_ctrl, 0xf8e4,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[2], 1);
	msleep(5); /* delay 5ms at least */

	val[3] = 0x02;
	ois_block_write_addr8_data8(o_ctrl, 0xf200,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[3], 1);
	val[4] = 0x01;
	ois_block_write_addr8_data8(o_ctrl, 0xf203,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[4], 1);
	val[5] = 0x01;
	ois_block_write_addr8_data8(o_ctrl, 0xf202,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[5], 1);
	msleep(2000); /* delay 2s at least */

	i = 0;
	do {
		ois_block_read_addr8_data8(o_ctrl, 0xf201,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[5], 1);
		msleep(20);
		i++;
	} while ((val[5] != 0x21) && (i < (50 *  AW_ERROR_LOOP)));
	if (val[5] != 0x21) {
		CAM_ERR(CAM_OIS, "read reg[0xf201] != 0x21, i=%d,val=%d",i,val[5] );
		return OIS_ERROR;
	}

	val[6] = 0x00;
	ois_block_write_addr8_data8(o_ctrl, 0xf203,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[6], 1);
	val[7] = 0x02;
	ois_block_write_addr8_data8(o_ctrl, 0xf8e4,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[7], 1);
	val[8] = 0xdc;
	ois_block_write_addr8_data8(o_ctrl, 0xf8ff,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[8], 1);
	val[9] = 0xd4;
	ois_block_write_addr8_data8(o_ctrl, 0xf20f,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[9], 1);

	/* Gyro offset data */
	ois_block_read_addr8_data8(o_ctrl, 0xf84c,
				CAMERA_SENSOR_I2C_TYPE_WORD, &val[10], 4);
	offset_val = (val[11]<<8) |val[10];
	CAM_INFO(CAM_OIS, "aw86006 gyro x offset = %d!", offset_val);

	offset_val = (val[13]<<8) |val[12];
	CAM_INFO(CAM_OIS, "aw86006 gyro y offset = %d!", offset_val);

	CAM_INFO(CAM_OIS, "aw86006 gyro offset cali success!");
	return count;
}

/*******************************************************************************
 * Debug node
 * Path: /sys/class/aw86006_ois
 * ****************************************************************************/
static CLASS_ATTR_RW(reg);
static CLASS_ATTR_RW(awrw);
static CLASS_ATTR_RW(update);
static CLASS_ATTR_RW(erase);
static CLASS_ATTR_RW(mode);
static CLASS_ATTR_RW(chipid);
static CLASS_ATTR_RW(version);
static CLASS_ATTR_RW(checkinfo);
static CLASS_ATTR_RW(freq);
static CLASS_ATTR_RW(jump);
static CLASS_ATTR_RW(gyro_offset_cali);

int aw_create_sysfs(void)
{
	int ret = 0;

	if (!ois_debug_class) {
		CAM_INFO(CAM_OIS, "create aw86006_result_class!");
		ois_debug_class = class_create(THIS_MODULE, "ois");

		ret = class_create_file(ois_debug_class, &class_attr_reg);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file reg failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_awrw);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file awrw failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_update);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file update failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_erase);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file erase failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_mode);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file mode failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_chipid);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file chipid failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_version);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file version failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_checkinfo);
		if (ret < 0) {
			CAM_ERR(CAM_OIS,
				"Create class file checkinfo failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_freq);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file freq failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_jump);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "Create class file jump failed, ret:%d",
				ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_gyro_offset_cali);
		if (ret < 0) {
			 CAM_ERR(CAM_OIS, "Create class file gyro offset failed, ret:%d", ret);
			 return ret;
		}
	}

	CAM_INFO(CAM_OIS, "Creat sysfs debug success.");
	return 0;
}

void aw_destroy_sysfs(void)
{
	if (ois_debug_class) {
		class_remove_file(ois_debug_class, &class_attr_reg);
		class_remove_file(ois_debug_class, &class_attr_update);
		class_remove_file(ois_debug_class, &class_attr_erase);
		class_remove_file(ois_debug_class, &class_attr_awrw);
		class_remove_file(ois_debug_class, &class_attr_checkinfo);
		class_remove_file(ois_debug_class, &class_attr_freq);
		class_remove_file(ois_debug_class, &class_attr_jump);
		class_remove_file(ois_debug_class, &class_attr_chipid);
		class_remove_file(ois_debug_class, &class_attr_version);
		class_remove_file(ois_debug_class, &class_attr_mode);
		class_remove_file(ois_debug_class, &class_attr_gyro_offset_cali);
		class_destroy(ois_debug_class);
		ois_debug_class = NULL;
		CAM_INFO(CAM_OIS, "delete ois_debug_class done!");
	}
}


/*******************************************************************************
 * aw86006 ois init
 ******************************************************************************/
int aw_ois_init(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;

	CAM_INFO(CAM_OIS, "enter, %s", AW86006_DRIVER_VERSION);
	o_ctrl_g = o_ctrl;
	INIT_WORK(&o_ctrl->fw_update_work, aw_firmware_update_work_routine);
	mutex_init(&o_ctrl->ois_mutex_aw);
	aw_create_sysfs();
	CAM_INFO(CAM_OIS, "exit");

	return rc;
}

int aw_ois_exit(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;

	CAM_INFO(CAM_OIS, "enter");
	aw_destroy_sysfs();
	return rc;
}

MODULE_DESCRIPTION("AWINIC OIS Driver");
MODULE_LICENSE("GPL v2");
