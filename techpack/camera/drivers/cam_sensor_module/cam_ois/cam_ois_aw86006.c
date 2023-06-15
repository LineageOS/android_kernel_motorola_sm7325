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

#define AW86006_DRIVER_VERSION		"v0.5.0.9"
#define AW86006_FW_NAME			"mot_aw86006.prog"

const char fw_check_str[] = { 'A', 'W', 'I', 'N', 'I', 'C', 0, 0 };
struct cam_ois_ctrl_t *g_o_ctrl;
static struct class *ois_debug_class;
struct aw86006_info g_aw86006_info = {
	.already_update = 0,
	.checkinfo_fw = { 0 },
	.checkinfo_rd = { 0 },
	.fw = { 0 },
};

static int ois_block_read_addr8_data8(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t addr_type, uint8_t *data, uint32_t num_byte)
{
	enum i2c_freq_mode temp_freq;
	int ret = OIS_ERROR;

	if (o_ctrl == NULL || data == NULL) {
		AW_LOGE("Invalid Args o_ctrl: %pK, data: %pK", o_ctrl, data);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		AW_LOGE("Not in right state to start soc reads: %d",
							o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;
	/* Modify i2c freq to 100K */
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_STANDARD_MODE;
	ret = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, data,
		addr_type, CAMERA_SENSOR_I2C_TYPE_BYTE, num_byte);
	if (ret < 0)
		AW_LOGE("err! ret:%d", ret);
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;

	return ret;
}

static int ois_block_write_addr8_data8(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t addr_type, uint8_t *data, uint32_t num_byte)
{
	int ret = OIS_ERROR;
	int cnt;
	enum i2c_freq_mode temp_freq;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;


	if (o_ctrl == NULL || data == NULL) {
		AW_LOGE("Invalid Args o_ctrl: %pK, data: %pK", o_ctrl, data);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		AW_LOGE("Not in right state to start soc writes: %d",
							o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	i2c_reg_setting.addr_type = addr_type;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = num_byte;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
			kzalloc(sizeof(struct cam_sensor_i2c_reg_array) *
							num_byte, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL) {
		AW_LOGE("kzalloc failed");
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
		AW_LOGE("err! ret:%d", ret);
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;
	kfree(i2c_reg_setting.reg_setting);

	return ret;
}

static int aw86006_reset(struct cam_ois_ctrl_t *o_ctrl)
{
	int ret = OIS_ERROR;
	uint16_t temp_addr;
	uint8_t boot_cmd_1[] = { 0xFF, 0xF0, 0x20, 0x20, 0x02, 0x02, 0x19, 0x29,
								0x19, 0x29 };
	uint8_t boot_cmd_2[] = { 0xFF, 0xC4, 0xC4 };
	uint8_t boot_cmd_3[] = { 0xFF, 0xC4, 0x00 };
	uint8_t boot_cmd_4[] = { 0xFF, 0x10, 0xC3 };

	AW_LOGI("enter");

	/* first: shutdown */
	temp_addr = o_ctrl->io_master_info.cci_client->sid;
	o_ctrl->io_master_info.cci_client->sid = AW_SHUTDOWN_I2C_ADDR;
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_1[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_1[1], sizeof(boot_cmd_1) - 1);
	if (ret < 0) {
		AW_LOGE("boot_cmd_1 write error!");
		goto err_exit;
	}
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_2[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_2[1], sizeof(boot_cmd_2) - 1);
	if (ret < 0) {
		AW_LOGE("boot_cmd_2 write error!");
		goto err_exit;
	}
	usleep_range(AW_SHUTDOWN_DELAY, AW_SHUTDOWN_DELAY + 50); /* 2 ms */
	/* second: wake up */
	o_ctrl->io_master_info.cci_client->sid = AW_WAKEUP_I2C_ADDR;
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_3[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_3[1], sizeof(boot_cmd_3) - 1);
	if (ret < 0) {
		AW_LOGE("boot_cmd_3 write error!");
		goto err_exit;
	}
	ret = ois_block_write_addr8_data8(o_ctrl, boot_cmd_4[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE,
		&boot_cmd_4[1], sizeof(boot_cmd_4) - 1);
	if (ret < 0) {
		AW_LOGE("boot_cmd_4 write error!");
		goto err_exit;
	}

	o_ctrl->io_master_info.cci_client->sid = temp_addr;

	AW_LOGD("end");

	return OIS_SUCCESS;

 err_exit:
	o_ctrl->io_master_info.cci_client->sid = temp_addr;
	return OIS_ERROR;
}

static int aw86006_runtime_check(struct cam_ois_ctrl_t *o_ctrl)
{
	int ret = OIS_ERROR;
	uint32_t version = 0;
	uint8_t reg_val[4] = { 0 };
	uint8_t chip_id = 0;

	AW_LOGI("enter");

	ret = ois_block_read_addr8_data8(o_ctrl, REG_CHIPID,
			CAMERA_SENSOR_I2C_TYPE_WORD, &chip_id, 1);
	if (ret < 0) {
		AW_LOGE("read chipid error!");
		return ret;
	}

	ret = ois_block_read_addr8_data8(o_ctrl, REG_VERSION,
			CAMERA_SENSOR_I2C_TYPE_WORD, &reg_val[0], 4);
	if (ret < 0) {
		AW_LOGE("read version error!");
		return ret;
	}
	version = (reg_val[3] << AW_DATA_SHIFT_24_BIT) |
					(reg_val[2] << AW_DATA_SHIFT_16_BIT) |
					(reg_val[1] << AW_DATA_SHIFT_8_BIT) |
					(reg_val[0] << AW_DATA_SHIFT_0_BIT);

	AW_LOGI("Chip_ID: 0x%02X, Version: v%d.%d.%d.%d", chip_id, reg_val[3],
					reg_val[2], reg_val[1], reg_val[0]);

	if ((chip_id != g_aw86006_info.fw.app_id) ||
				(version != g_aw86006_info.fw.app_version)) {
		AW_LOGE("Chip_ID or Version not match!");
		return OIS_ERROR;
	}
	AW_LOGI("pass!");

	return OIS_SUCCESS;
}

static int aw86006_soc_buf_build(struct cam_ois_ctrl_t *o_ctrl, uint8_t *buf,
						struct soc_protocol *soc_struct)
{
	int i = 0;
	uint8_t *p_head = (uint8_t *) soc_struct;
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

static int aw86006_soc_connect_check(struct cam_ois_ctrl_t *o_ctrl)
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
	aw86006_soc_buf_build(o_ctrl, w_buf, &soc_struct);
	ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
		CAMERA_SENSOR_I2C_TYPE_BYTE, &w_buf[1],
		SOC_CONNECT_WRITE_LEN - 1);
	if (ret < 0) {
		AW_LOGE("write connect w_buf error!");
		return ret;
	}

	usleep_range(SOC_CONNECT_DELAY, SOC_CONNECT_DELAY + 50);

	memset(r_buf, 0, sizeof(r_buf));
	ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
		CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 14);
	if (ret < 0) {
		AW_LOGE("read connect r_buf error!");
		return ret;
	}
	/* regroup w_buf */
	soc_struct.event = SOC_HANK_CONNECT_ACK;
	soc_struct.ack_juge = SOC_ACK;
	soc_struct.len[0] = 5;
	soc_struct.p_data = cmp_buf;

	memset(w_buf, 0, sizeof(w_buf));
	aw86006_soc_buf_build(o_ctrl, w_buf, &soc_struct);

	ret = memcmp(w_buf, r_buf, sizeof(r_buf));
	if (ret != 0) {
		AW_LOGE("connect memcmp error!");
		return OIS_ERROR;
	}

	return OIS_SUCCESS;
}

static int aw86006_soc_flash_read_check(struct cam_ois_ctrl_t *o_ctrl,
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
		soc_struct.ack_juge = SOC_CTL;

		memset(w_buf, 0, sizeof(w_buf));
		aw86006_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
						CAMERA_SENSOR_I2C_TYPE_BYTE,
						&w_buf[1], sizeof(w_buf) - 1);
		if (ret < 0) {
			AW_LOGE("i2c write w_buf error, loop: %d", loop);
			continue;
		}
		usleep_range(SOC_READ_BLOCK_DELAY, SOC_READ_BLOCK_DELAY + 50);

		memset(r_buf, 0, sizeof(r_buf));
		ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
			CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 14 + len);
		if (ret < 0) {
			AW_LOGE("i2c read r_buf error, loop: %d", loop);
			continue;
		}
		/* check error flag */
		if (r_buf[9] != OIS_SUCCESS) {
			AW_LOGE("flag error: %d, loop: %d", r_buf[10], loop);
			continue;
		}
		checksum = 0;
		/* compute data checksum */
		for (i = 0; i < len + 5; i++)
			checksum += r_buf[9 + i];
		if (checksum != r_buf[8]) {
			AW_LOGE("data checksum error: 0x%02x != 0x%02x",
							checksum, r_buf[8]);
			continue;
		}
		checksum = 0;
		/* compute head checksum */
		for (i = 1; i < 9; i++)
			checksum += r_buf[i];
		if (checksum != r_buf[0]) {
			AW_LOGE("head checksum error: 0x%02x != 0x%02x",
							checksum, r_buf[0]);
			continue;
		}
		memcpy(bin_buf, (uint8_t *)&r_buf[14], len);
		break; /* Check pass */
	} while ((++loop) < AW_FLASH_READ_ERROR_LOOP);
	if (loop >= AW_FLASH_READ_ERROR_LOOP)
		return OIS_ERROR;

	return OIS_SUCCESS;
}

static int aw86006_soc_flash_write_check(struct cam_ois_ctrl_t *o_ctrl,
					uint32_t addr, uint32_t block_num,
					uint8_t *bin_buf, uint32_t len)
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
		ret = aw86006_soc_connect_check(o_ctrl);
		if (ret == OIS_SUCCESS)
			break;
		AW_LOGE("connect_loop error ret: %d, connect_loop: %d", ret,
								connect_loop);
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
		aw86006_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
					CAMERA_SENSOR_I2C_TYPE_BYTE, &w_buf[1],
					SOC_WRITE_BLOCK_HEAD - 1 + len);
		if (ret < 0) {
			AW_LOGE("i2c write w_buf error, loop: %d", loop);
			continue;
		}
		usleep_range(SOC_WRITE_BLOCK_DELAY, SOC_WRITE_BLOCK_DELAY + 50);

		memset(r_buf, 0, sizeof(r_buf));
		ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
				CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 10);
		if (ret < 0) {
			AW_LOGE("i2c read r_buf error, loop: %d", loop);
			continue;
		}
		/* regroup w_buf */
		soc_struct.event = SOC_FLASH_WRITE_ACK;
		soc_struct.len[0] = 1;
		soc_struct.p_data = cmp_buf;
		soc_struct.ack_juge = SOC_ACK;

		memset(w_buf, 0, sizeof(w_buf));
		aw86006_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = memcmp(w_buf, r_buf, sizeof(r_buf));
		if (ret == 0)
			break;

		AW_LOGI("memcmp error, try again, loop: %d", loop);
	} while ((++loop) < AW_FLASH_WRITE_ERROR_LOOP);
	if (loop >= AW_FLASH_WRITE_ERROR_LOOP)
		return OIS_ERROR;

	return OIS_SUCCESS;
}

static int aw86006_soc_flash_erase_check(struct cam_ois_ctrl_t *o_ctrl,
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
		aw86006_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = ois_block_write_addr8_data8(o_ctrl, w_buf[0],
					CAMERA_SENSOR_I2C_TYPE_BYTE, &w_buf[1],
					SOC_ERASE_WRITE_LEN - 1);
		if (ret < 0) {
			AW_LOGE("i2c write w_buf error, loop: %d", loop);
			continue;
		}
		msleep(erase_block * SOC_ERASE_BLOCK_DELAY);

		memset(r_buf, 0, sizeof(r_buf));
		ret = ois_block_read_addr8_data8(o_ctrl, AW_I2C_WRITE_BYTE_ZERO,
				CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 10);
		if (ret < 0) {
			AW_LOGE("i2c read r_buf error, loop: %d", loop);
			continue;
		}
		soc_struct.event = SOC_FLASH_ERASE_BLOCK_ACK;
		soc_struct.len[0] = 1;
		soc_struct.ack_juge = SOC_ACK;
		soc_struct.p_data = cmp_buf;

		memset(w_buf, 0, sizeof(w_buf));
		aw86006_soc_buf_build(o_ctrl, w_buf, &soc_struct);
		ret = memcmp(w_buf, r_buf, sizeof(r_buf));
		if (ret == 0)
			break;

		AW_LOGI("memcmp error, try again, loop: %d", loop);
	} while ((++loop) < AW_FLASH_ERASE_ERROR_LOOP);
	if (loop >= AW_FLASH_ERASE_ERROR_LOOP)
		return OIS_ERROR;

	AW_LOGI("flash erase success!, addr: 0x%08x, len: 0x%08x", addr, len);

	return OIS_SUCCESS;
}

static int aw86006_soc_flash_download_check(struct cam_ois_ctrl_t *o_ctrl,
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
		ret = aw86006_soc_flash_erase_check(o_ctrl, AW_FLASH_APP_ADDR,
						len - AW_FLASH_MOVE_LENGTH);
		if (ret != OIS_SUCCESS) {
			AW_LOGE("Failed to erase app+checkinfo");
			return OIS_ERROR;
		}
		/* then erase move data */
		ret = aw86006_soc_flash_erase_check(o_ctrl, AW_FLASH_BASE_ADDR,
							AW_FLASH_MOVE_LENGTH);
		if (ret != OIS_SUCCESS) {
			AW_LOGE("Failed to erase move");
			return OIS_ERROR;
		}
		/* write move data */
		for (i = 0; i < flash_checkinfo; i++) {
			ret = aw86006_soc_flash_write_check(o_ctrl, addr, i,
						bin_buf, AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Failed to write flash block: %d", i);
				return OIS_ERROR;
			}
		}
		/* write app block data */
		for (i = flash_checkinfo + 1; i < flash_block; i++) {
			ret = aw86006_soc_flash_write_check(o_ctrl, addr, i,
						bin_buf, AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Failed to write flash block: %d", i);
				return OIS_ERROR;
			}
		}
		/* write app tail data */
		if ((flash_tail != 0) && (len > AW_FLASH_MOVE_LENGTH)) {
			ret = aw86006_soc_flash_write_check(o_ctrl, addr, i,
							bin_buf, flash_tail);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Failed to write flash app tail");
				return OIS_ERROR;
			}
		}
	} else if (addr == AW_FLASH_APP_ADDR) {
		flash_checkinfo = 0;
		/* erase app+info data */
		ret = aw86006_soc_flash_erase_check(o_ctrl, AW_FLASH_APP_ADDR,
									len);
		if (ret != OIS_SUCCESS) {
			AW_LOGE("Failed to erase app+checkinfo!");
			return OIS_ERROR;
		}
		/* write app block data */
		for (i = 1; i < flash_block; i++) {
			ret = aw86006_soc_flash_write_check(o_ctrl, addr, i,
						bin_buf, AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Failed to write flash block: %d", i);
				return OIS_ERROR;
			}
		}
		/* write app tail data */
		if (flash_tail != 0) {
			ret = aw86006_soc_flash_write_check(o_ctrl, addr, i,
							bin_buf, flash_tail);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Failed to write flash tail");
				return OIS_ERROR;
			}
		}
	} else {
		AW_LOGE("wrong addr!");
		return OIS_ERROR;
	}

	/* Write checkinfo data */
	ret = aw86006_soc_flash_write_check(o_ctrl, AW_FLASH_APP_ADDR, 0,
						g_aw86006_info.checkinfo_fw,
						AW_FLASH_WRITE_LEN);
	if (ret != OIS_SUCCESS) {
		AW_LOGE("Failed to write checkinfo!");
		return OIS_ERROR;
	}

	return OIS_SUCCESS;
}

static int aw86006_jump_move_check(struct cam_ois_ctrl_t *o_ctrl)
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
		ret = aw86006_reset(o_ctrl);
		if (ret != OIS_SUCCESS) {
			AW_LOGE("aw86006_reset error, ret: %d, loop: %d", ret,
								jump_loop);
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
			AW_LOGE("ms_count_reset timeout: %d, loop: %d",
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
		AW_LOGD("ms_count_reset: %u, ms_count_stop: %u", ms_count_reset,
								ms_count_stop);

		if (ms_first_f0 > AW_JUMP_MOVE_DELAY_MAX) {
			AW_LOGI("first 0xF0: %d ms", ms_first_f0);
			continue;
		}

		/* send read move version cmd */
		ret = ois_block_write_addr8_data8(o_ctrl, version_cmd[0],
			CAMERA_SENSOR_I2C_TYPE_BYTE, &version_cmd[1],
			sizeof(version_cmd) - 1);
		if (ret < 0) {
			AW_LOGE("write version_cmd failed! loop:%d", jump_loop);
			continue;
		}
		usleep_range(ISP_READ_VERSION_DELAY,
						ISP_READ_VERSION_DELAY + 50);
		ret = ois_block_read_addr8_data8(o_ctrl, ISP_VERS_CONNECT_ACK,
					CAMERA_SENSOR_I2C_TYPE_BYTE,
					&version_ack[0], ISP_VERSION_ACK_LEN);
		if (ret < 0) {
			AW_LOGE("read version error, loop: %d", jump_loop);
			continue;
		}

		if (version_ack[0] != ISP_EVENT_OK) {
			AW_LOGE("wrong version_ack: %d, loop: %d",
						version_ack[0], jump_loop);
			continue;
		}
		move_version = (version_ack[4] << AW_DATA_SHIFT_24_BIT) |
				(version_ack[3] << AW_DATA_SHIFT_16_BIT) |
				(version_ack[2] << AW_DATA_SHIFT_8_BIT) |
				(version_ack[1] << AW_DATA_SHIFT_0_BIT);

		if (move_version == g_aw86006_info.fw.move_version)
			break;

		AW_LOGE("move_version not match: 0x%08X != 0x%08X, loop: %d",
				move_version, g_aw86006_info.fw.move_version,
				jump_loop);
	} while ((++jump_loop) < AW_JUMP_MOVE_LOOP);
	if (jump_loop >= AW_JUMP_MOVE_LOOP)
		return OIS_ERROR;

	AW_LOGI("Jump move success! move_version: 0x%08X", move_version);
	return OIS_SUCCESS;
}

static int aw86006_jump_boot_check(struct cam_ois_ctrl_t *o_ctrl)
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
		ret = aw86006_reset(o_ctrl);
		if (ret != OIS_SUCCESS) {
			AW_LOGE("aw86006_reset error, ret: %d, loop: %d", ret,
								jump_loop);
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
			AW_LOGE("ms_count_reset timeout: %d, loop: %d",
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
		AW_LOGD("ms_count_reset: %u, ms_count_stop: %u",
						ms_count_reset, ms_count_stop);
		if (ms_first_ac > AW_JUMP_BOOT_DELAY_MAX) {
			AW_LOGI("first 0xAC: %d ms", ms_first_ac);
			continue;
		}

		ret = aw86006_soc_connect_check(o_ctrl);
		if (ret == OIS_SUCCESS)
			break;

		AW_LOGE("soc connect failed, ret:%d, loop: %d", ret, jump_loop);
	} while ((++jump_loop) < AW_JUMP_BOOT_LOOP);
	if (jump_loop >= AW_JUMP_BOOT_LOOP)
		return OIS_ERROR;

	AW_LOGI("success");

	return OIS_SUCCESS;
}

static int aw86006_soc_flash_update(struct cam_ois_ctrl_t *o_ctrl,
				uint32_t addr, uint8_t *data_p, size_t fw_size)
{
	int ret = OIS_ERROR;
	int loop = 0;

	if (!data_p) {
		AW_LOGE("data_p is NULL");
		return OIS_ERROR;
	}
	/* enter boot mode */
	ret = aw86006_jump_boot_check(o_ctrl);
	if (ret != OIS_SUCCESS) {
		AW_LOGE("jump boot failed!");
		return OIS_ERROR;
	}
	/* update flash */
	do {
		ret = aw86006_soc_flash_download_check(o_ctrl, addr, data_p,
								fw_size);
		if (ret == OIS_SUCCESS)
			break;
		AW_LOGE("soc_flash_download_check failed! loop: %d", loop);
	} while ((++loop) < AW_ERROR_LOOP);
	if (loop >= AW_ERROR_LOOP)
		return OIS_ERROR;

	ret = aw86006_reset(o_ctrl);
	if (ret != OIS_SUCCESS) {
		AW_LOGE("aw86006_reset failed!");
		return OIS_ERROR;
	}
	msleep(AW_RESET_DELAY);

	AW_LOGI("success!");

	return OIS_SUCCESS;
}

static int aw86006_firmware_check(struct cam_ois_ctrl_t *o_ctrl,
						const struct firmware *fw)
{
	uint32_t temp = 0;
	uint32_t *check_ptr = NULL;
	int i = 0;
	int ret = OIS_ERROR;
	char *identify = (char *)fw->data + AW_FW_SHIFT_IDENTIFY;

	if (fw == NULL) {
		AW_LOGE("FW Empty!!!");
		return OIS_ERROR;
	}

	ret = memcmp(fw_check_str, identify, 8); /* AWINIC00 8bytes */
	if (ret != 0) {
		AW_LOGE("loaded wrong firmware!");
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

	AW_LOGI("fw->size:0x%04X, app_length:0x%04X, app_version:0x%08X",
						g_aw86006_info.fw.size,
						g_aw86006_info.fw.app_length,
						g_aw86006_info.fw.app_version);
	AW_LOGI("app_id:0x%02X, move_version:0x%08X, move_length:0x%04X",
						g_aw86006_info.fw.app_id,
						g_aw86006_info.fw.move_version,
						g_aw86006_info.fw.move_length);

	/* length check */
	temp = g_aw86006_info.fw.move_length + g_aw86006_info.fw.app_length;
	if (g_aw86006_info.fw.size != temp) {
		AW_LOGE("fw->size error: 0x%X != 0x%X", g_aw86006_info.fw.size,
									temp);
		return OIS_ERROR;
	}

	/* move checksum check */
	check_ptr = (uint32_t *) &fw->data[0];
	temp = 0;
	for (i = 0; i < (g_aw86006_info.fw.move_length / sizeof(uint32_t)); i++)
		temp += check_ptr[i];
	if (temp != g_aw86006_info.fw.move_checksum) {
		AW_LOGE("move checksum error:0x%08X != 0x%08X",
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
		AW_LOGE("check_info checksum error:0x%08X != 0x%08X",
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
		AW_LOGE("app checksum error:0x%08X != 0x%08X",
					g_aw86006_info.fw.app_checksum, temp);
		return OIS_ERROR;
	}
	/* Save firmware checkinfo */
	memcpy(&g_aw86006_info.checkinfo_fw[0],
		fw->data + AW_FLASH_MOVE_LENGTH, AW_FW_INFO_LENGTH);

	AW_LOGI("Pass!");

	return OIS_SUCCESS;
}

static int aw86006_checkinfo_analyse(struct cam_ois_ctrl_t *o_ctrl,
				uint8_t *checkinfo_rd, struct aw86006_fw *info)
{
	uint32_t temp = 0;
	uint32_t *check_ptr = NULL;
	int i = 0;
	int ret = OIS_ERROR;

	if ((checkinfo_rd == NULL) || (info == NULL)) {
		AW_LOGE("checkinfo empty!");
		return OIS_ERROR;
	}
	/* compare identify: AWINIC00 */
	ret = memcmp(fw_check_str, (char *)checkinfo_rd, 8);
	if (ret != 0) {
		AW_LOGE("checkinfo not match!");
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

	AW_LOGI("app_length: 0x%04X, app_version: 0x%08X, app_id: 0x%02X",
							info->app_length,
							info->app_version,
							info->app_id);
	AW_LOGI("move_version:0x%08X, move_length:0x%04X", info->move_version,
							info->move_length);

	/* info checksum check */
	check_ptr = (uint32_t *) &checkinfo_rd[AW_FW_SHIFT_CHECKSUM_ADDR -
							AW_FLASH_MOVE_LENGTH];
	temp = 0;
	for (i = 0; i < ((AW_FW_INFO_LENGTH - 8 - 4) / sizeof(uint32_t)); i++)
		temp += check_ptr[i];
	if (temp != info->checksum) {
		AW_LOGE("checkinfo_rd checksum error:0x%08X != 0x%08X",
			info->checksum, temp);
		return OIS_ERROR;
	}
	AW_LOGI("pass!");
	return OIS_SUCCESS;
}

static int aw86006_get_standby_flag(struct cam_ois_ctrl_t *o_ctrl,
								uint8_t *pflag)
{
	int ret = OIS_ERROR;
	uint8_t flag = 0;
	uint8_t temp_addr = 0;

	AW_LOGI("enter");

	temp_addr = o_ctrl->io_master_info.cci_client->sid;
	o_ctrl->io_master_info.cci_client->sid = AW_SHUTDOWN_I2C_ADDR;
	ret = ois_block_read_addr8_data8(o_ctrl, 0xFF11,
			CAMERA_SENSOR_I2C_TYPE_WORD, &flag, 1);
	o_ctrl->io_master_info.cci_client->sid = temp_addr;
	if (ret < 0) {
		AW_LOGE("read standby flag error, ret: %d", ret);
		return ret;
	}

	*pflag = flag;

	AW_LOGI("standby flag: %d", flag);

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

	AW_LOGI("enter");

	if (fw == NULL) {
		AW_LOGE("error, FW Empty!!!");
		return OIS_ERROR;
	}

	/* enter boot mode */
	ret = aw86006_jump_boot_check(o_ctrl);
	if (ret != OIS_SUCCESS) {
		AW_LOGE("jump boot failed!");
		return OIS_ERROR;
	}

#ifdef AW_SOC_FLASH_READ_FLAG
	/* get check_info data */
	ret = aw86006_soc_flash_read_check(o_ctrl, AW_FLASH_APP_ADDR,
			&g_aw86006_info.checkinfo_rd[0], AW_FLASH_READ_LEN);
	if (ret != OIS_SUCCESS)
		AW_LOGE("read checkinfo error!");
#endif

	g_aw86006_info.fw.update_flag =
			g_aw86006_info.checkinfo_rd[AW_ARRAY_SHIFT_UPDATE_FLAG];

	if (g_aw86006_info.fw.update_flag != 0x01) {
		AW_LOGI("update_flag not match, update all!");
		ret = aw86006_soc_flash_update(o_ctrl, AW_FLASH_BASE_ADDR,
						all_buf_ptr, all_buf_size);
		if (ret != OIS_SUCCESS) {
			AW_LOGE("update all failed!");
			return OIS_ERROR;
		}
	} else {
		AW_LOGI("update_flag match!");
		ret = memcmp(g_aw86006_info.checkinfo_rd,
				g_aw86006_info.checkinfo_fw, AW_FW_INFO_LENGTH);
		if (ret != 0) {
			ret = aw86006_checkinfo_analyse(o_ctrl,
					g_aw86006_info.checkinfo_rd, &info_rd);
			if ((ret != OIS_SUCCESS) || (info_rd.move_version !=
					g_aw86006_info.fw.move_version)) {
				AW_LOGE("checkinfo/move not match, update all");
				ret = aw86006_soc_flash_update(o_ctrl,
						AW_FLASH_BASE_ADDR, all_buf_ptr,
						all_buf_size);
				if (ret != OIS_SUCCESS) {
					AW_LOGE("update all failed!");
					return OIS_ERROR;
				}
			} else if ((info_rd.app_version !=
					g_aw86006_info.fw.app_version) ||
					(info_rd.app_id !=
						g_aw86006_info.fw.app_id)) {
				AW_LOGE("app not match, update app!");
				ret = aw86006_soc_flash_update(o_ctrl,
						AW_FLASH_APP_ADDR, app_buf_ptr,
						app_buf_size);
				if (ret != OIS_SUCCESS) {
					AW_LOGE("update app failed!");
					return OIS_ERROR;
				}
			} else {
				AW_LOGE("other error, update all!");
				ret = aw86006_soc_flash_update(o_ctrl,
						AW_FLASH_BASE_ADDR, all_buf_ptr,
						all_buf_size);
				if (ret != OIS_SUCCESS) {
					AW_LOGE("update all failed!");
					return OIS_ERROR;
				}
			}
		} else {
			ret = aw86006_jump_move_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("jump_move_check fail, update all!");
				ret = aw86006_soc_flash_update(o_ctrl,
						AW_FLASH_BASE_ADDR, all_buf_ptr,
						all_buf_size);
				if (ret != OIS_SUCCESS) {
					AW_LOGE("update all failed!");
					return OIS_ERROR;
				}
			} else {
				ret = aw86006_reset(o_ctrl);
				if (ret != OIS_SUCCESS) {
					AW_LOGE("reset failed!");
					return OIS_ERROR;
				}
				msleep(AW_RESET_DELAY);
			}
		}
	}

	for (i = 0; i <= AW_ERROR_LOOP; i++) {
		ret = aw86006_runtime_check(o_ctrl);
		if (ret == OIS_SUCCESS) {
			AW_LOGI("runtime_check pass, no need to update fw!");
			break;
		}
		AW_LOGE("runtime_check failed, update app! loop: %d", i);
		if (i == AW_ERROR_LOOP)
			break;

		ret = aw86006_soc_flash_update(o_ctrl, AW_FLASH_APP_ADDR,
						app_buf_ptr, app_buf_size);
		if (ret != OIS_SUCCESS) {
			AW_LOGE("update app failed!");
			break;
		}
	}

	return ret;
}

int aw86006_firmware_update(struct cam_ois_ctrl_t *o_ctrl,
						const struct firmware *fw)
{
	int ret = OIS_ERROR;
	int i = 0;
	uint8_t standby_flag = 0;

	AW_LOGI("enter");

	/* fw check */
	AW_LOGI("Load:%s size:%zu", AW86006_FW_NAME, fw->size);
	ret = aw86006_firmware_check(o_ctrl, fw);
	if (ret != OIS_SUCCESS) {
		AW_LOGE("fw check failed!");
		goto err_fw_check;
	}
	/* reset */
	ret = aw86006_reset(o_ctrl);
	if (ret != OIS_SUCCESS) {
		AW_LOGE("reset failed");
		goto err_reset;
	}
	mdelay(AW_RESET_DELAY); /* run app after reset */
	/* Get standby flag */
	for (i = 0; i < AW_ERROR_LOOP; i++) {
		aw86006_get_standby_flag(o_ctrl, &standby_flag);
		if (standby_flag == AW_IC_STANDBY)
			break;
		mdelay(AW_RESET_DELAY);
	}

	if (standby_flag == AW_IC_STANDBY) {
		ret = aw86006_runtime_check(o_ctrl);
		if (ret == OIS_SUCCESS)
			AW_LOGI("runtime_check pass, no need to update fw!");
	}
	/* update flash */
	if ((standby_flag != AW_IC_STANDBY) || (ret != OIS_SUCCESS)) {
		/* update flash */
		ret = aw86006_mem_download(o_ctrl, fw);
		if (ret == OIS_SUCCESS)
			AW_LOGI("fw update success!");
		else
			AW_LOGE("fw update failed, ret: %d", ret);
	}

err_reset:
err_fw_check:
	release_firmware(fw);
	return ret;
}

static void aw86006_firmware_update_work_routine(struct work_struct *work)
{
	struct cam_ois_ctrl_t *o_ctrl = container_of(work,
				struct cam_ois_ctrl_t, aw_fw_update_work);
	const struct firmware *fw = NULL;
	int loop = 0;
	int ret = OIS_ERROR;

	AW_LOGI("enter");

	/* load firmware */
	do {
		ret = request_firmware(&fw, AW86006_FW_NAME,
							&o_ctrl->pdev->dev);
		if (ret == 0)
			break;
	} while ((++loop) < AW_ERROR_LOOP);
	if (loop >= AW_ERROR_LOOP) {
		AW_LOGE("request_firmware [%s] failed!", AW86006_FW_NAME);
		return;
	}
	mutex_lock(&o_ctrl->aw_ois_mutex);
	aw86006_firmware_update(o_ctrl, fw);
	mutex_unlock(&o_ctrl->aw_ois_mutex);

	AW_LOGI("End");
}

/*******************************************************************************
 * adb debug interface
 ******************************************************************************/
static ssize_t reg_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	ssize_t len = 0;
	int i = 0;
	uint8_t reg_val[20] = { 0 };

	AW_LOGI("Start");

	mutex_lock(&o_ctrl->aw_ois_mutex);
	for (i = 0; i < 20; i++) {
		ois_block_read_addr8_data8(o_ctrl, i, CAMERA_SENSOR_I2C_TYPE_WORD,
			&reg_val[i], 1);
		len += snprintf(buf + len, PAGE_SIZE - len,
					"reg[0x%04X]:0x%02X\n", i, reg_val[i]);
		AW_LOGD("reg[0x%04X]:0x%02X", i, reg_val[i]);
	}
	mutex_unlock(&o_ctrl->aw_ois_mutex);

	AW_LOGI("End");

	return len;
}

static ssize_t reg_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	uint32_t databuf[2] = { 0 };

	AW_LOGI("Start");

	mutex_lock(&o_ctrl->aw_ois_mutex);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		AW_LOGD("reg[0x%04X]:0x%02X", databuf[0], databuf[1]);
		ois_block_write_addr8_data8(o_ctrl, databuf[0],
			CAMERA_SENSOR_I2C_TYPE_WORD, (uint8_t *) &databuf[1],
			CAMERA_SENSOR_I2C_TYPE_BYTE);
	} else {
		AW_LOGE("Number of parameters");
		mutex_unlock(&o_ctrl->aw_ois_mutex);
		return -EPERM;
	}
	mutex_unlock(&o_ctrl->aw_ois_mutex);

	AW_LOGI("End");

	return count;
}

static int aw86006_parse_reg_data(struct awrw_ctrl *awrw_ctrl, const char *buf)
{
	int ret = OIS_ERROR;
	int i = 0;
	char data_buf[5] = { 0 };
	uint8_t temp = 0;

	AW_LOGI("Start");

	for (i = 0; i < awrw_ctrl->reg_num; i++) {
		if ((i * 5 + 5) > (strlen(buf) - 2 * (4 + 1) - (6 + 1))) {
			AW_LOGE("buf length error");
			return -ERANGE;
		}
		memcpy(data_buf, &buf[17 + i * 5], 4);
		data_buf[4] = '\0';
		/* char convert to unsigned char */
		ret = kstrtou8(data_buf, 0, &temp);
		if (ret < 0) {
			AW_LOGE("Parse reg_data[%d] error", i);
			return ret;
		}
		awrw_ctrl->reg_data[i] = temp;

		AW_LOGD("reg_data[%d]: 0x%02x", i, awrw_ctrl->reg_data[i]);
	}

	return 0;
}

static ssize_t awrw_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	struct awrw_ctrl *awrw_ctrl = g_o_ctrl->awrw_ctrl;
	int i = 0;
	ssize_t len = 0;
	uint16_t reg_num = awrw_ctrl->reg_num;
	uint8_t flag = awrw_ctrl->flag;
	uint8_t *reg_data = awrw_ctrl->reg_data;

	AW_LOGI("Start");

	if (!reg_num) {
		AW_LOGE("Reg_nums parameter error");
		return -ERANGE;
	}

	if (reg_data == NULL) {
		AW_LOGE("reg_data empty!");
		return -ERANGE;
	}

	if (flag == AW_SEQ_READ) {
		for (i = 0; i < reg_num; i++) {
			AW_LOGD("reg_data[%d]: 0x%02x", i, reg_data[i]);
			len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,",
								reg_data[i]);
		}
		len += snprintf(buf + len - 1, PAGE_SIZE - len, "\n");
	} else {
		AW_LOGE("Need to read awrw before show");
		return -EPERM;
	}

	AW_LOGI("End");

	return len;
}

static ssize_t awrw_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	struct awrw_ctrl *awrw_ctrl = g_o_ctrl->awrw_ctrl;
	int ret = OIS_ERROR;
	uint32_t flag = 0;
	uint32_t reg_num = 0;
	uint32_t reg_addr = 0;

	AW_LOGI("Start");

	if (sscanf(buf, "%x %x %x", &flag, &reg_num, &reg_addr) == 3) {
		if (!reg_num) {
			AW_LOGE("awrw parameter error");
			return -ERANGE;
		}
		mutex_lock(&o_ctrl->aw_ois_mutex);
		awrw_ctrl->flag = (uint8_t)flag;
		awrw_ctrl->reg_num = (uint16_t)reg_num;
		/* clear old reg_data */
		if (awrw_ctrl->reg_data != NULL)
			kfree(awrw_ctrl->reg_data);
		/* Request memory space for reg_data; Unit: byte */
		awrw_ctrl->reg_data = kzalloc(reg_num, GFP_KERNEL);

		if (awrw_ctrl->flag == AW_SEQ_WRITE) {
			/* parse reg data */
			ret = aw86006_parse_reg_data(awrw_ctrl, buf);
			if (ret < 0) {
				AW_LOGE("Parse reg_data error");
				mutex_unlock(&o_ctrl->aw_ois_mutex);
				return ret;
			}
			AW_LOGD("Parse reg_data OK");
			/* writes reg_data */
			ois_block_write_addr8_data8(o_ctrl, reg_addr,
						CAMERA_SENSOR_I2C_TYPE_WORD,
						awrw_ctrl->reg_data,
						awrw_ctrl->reg_num);
		} else if (awrw_ctrl->flag == AW_SEQ_READ) /* read data */
			ois_block_read_addr8_data8(o_ctrl, reg_addr,
						CAMERA_SENSOR_I2C_TYPE_WORD,
						awrw_ctrl->reg_data,
						awrw_ctrl->reg_num);

		mutex_unlock(&o_ctrl->aw_ois_mutex);
	} else {
		AW_LOGE("Number of parameters");
	}

	AW_LOGI("End");

	return count;
}

static ssize_t update_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
					"update ois: echo 1 > update\n");

	return len;
}

static ssize_t update_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	uint32_t databuf = 0;
	int ret = OIS_ERROR;

	AW_LOGI("Start");

	ret = kstrtouint(buf, 0, &databuf);
	if (ret < 0) {
		AW_LOGE("Number of parameters error");
		return ret;
	}

	/* fw update */
	if (databuf != 1) {
		AW_LOGE("flag error: %d", databuf);
		return -EPERM;
	}

	schedule_work(&o_ctrl->aw_fw_update_work);

	AW_LOGI("End");

	return count;
}

static ssize_t erase_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
					"erase app flash: echo app > erase\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
					"erase all flash: echo all > erase\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
					"read checkinfo: echo read > erase\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
					"ois reset: echo reset > erase\n");

	return len;
}

static ssize_t erase_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	int ret = OIS_ERROR;
	char databuf[10] = {0};
	char readbuf[64] = {0};

	if (sscanf(buf, "%s", databuf) == 1) {
		mutex_lock(&o_ctrl->aw_ois_mutex);
		if (strcmp(databuf, "app") == 0) {
			AW_LOGI("aw86006 will erase app!");
			ret = aw86006_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Jump boot failed");
				goto err_exit;
			}
			ret = aw86006_soc_flash_erase_check(o_ctrl,
					AW_FLASH_APP_ADDR, AW_FLASH_FULL_SIZE -
					AW_FLASH_MOVE_LENGTH);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Erase app failed");
				goto err_exit;
			}
		} else if (strcmp(databuf, "all") == 0) {
			AW_LOGI("aw86006 will erase all!");
			ret = aw86006_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Jump boot failed");
				goto err_exit;
			}
			ret = aw86006_soc_flash_erase_check(o_ctrl,
							AW_FLASH_BASE_ADDR,
							AW_FLASH_FULL_SIZE);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Erase all failed");
				goto err_exit;
			}
		} else if (strcmp(databuf, "read") == 0) {
			ret = aw86006_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Jump boot failed");
				goto err_exit;
			}
			ret = aw86006_soc_flash_read_check(o_ctrl,
						AW_FLASH_APP_ADDR, readbuf,
						AW_FLASH_READ_LEN);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Read checkinfo failed");
				goto err_exit;
			}
		} else if (strcmp(databuf, "reset") == 0) {
			AW_LOGI("aw86006 reset");
			ret = aw86006_reset(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Reset failed");
				goto err_exit;
			}
		} else {
			AW_LOGE("Erase: %s is not support!", databuf);
		}

		mutex_unlock(&o_ctrl->aw_ois_mutex);
	}

	return count;

err_exit:
	mutex_unlock(&o_ctrl->aw_ois_mutex);
	return ret;
}

static ssize_t mode_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
					"OIS_ON: echo 1 > mode\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
					"OIS_OFF: echo 0 > mode\n");

	return len;
}

static ssize_t mode_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	int ret = OIS_ERROR;
	int loop = 0;
	uint32_t ois_status = 0;
	uint8_t temp = 0;

	AW_LOGI("enter");

	ret = kstrtou32(buf, 0, &ois_status);
	if (ret < 0) {
		AW_LOGE("Number of parameters error");
		return ret;
	}
	if ((ois_status != OIS_DISABLE) && (ois_status != OIS_ENABLE)) {
		AW_LOGE("ois_status error: %d", ois_status);
		return -EPERM;
	}
	/* enable OIS mode */
	mutex_lock(&o_ctrl->aw_ois_mutex);
	do {
		temp = (uint8_t)ois_status;
		AW_LOGD("set ois status: %d", temp);

		ret = ois_block_write_addr8_data8(o_ctrl, REG_OIS_ENABLE,
					CAMERA_SENSOR_I2C_TYPE_WORD, &temp, 1);
		if (ret < 0) {
			AW_LOGE("Send OIS mode error");
			mutex_unlock(&o_ctrl->aw_ois_mutex);
			return ret;
		}
		/* 1000 us */
		usleep_range(1000, 1500);

		/* check OIS enable status */
		temp = 0;
		ret = ois_block_read_addr8_data8(o_ctrl, REG_OIS_ENABLE,
					CAMERA_SENSOR_I2C_TYPE_WORD, &temp, 1);
		if (ret < 0) {
			AW_LOGE("Read OIS mode error");
			mutex_unlock(&o_ctrl->aw_ois_mutex);
			return ret;
		}
		AW_LOGD("OIS enable value: 0x%02x", temp);

		if (temp == ois_status) {
			AW_LOGI("Set OIS mode success!");
			break;
		}
		AW_LOGE("OIS mode check error! value: 0x%02x", temp);
	} while ((++loop) < AW_ERROR_LOOP);
	mutex_unlock(&o_ctrl->aw_ois_mutex);
	if (loop >= AW_ERROR_LOOP)
		return OIS_ERROR;

	return count;
}

static ssize_t chipid_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	ssize_t len = 0;
	int ret = OIS_ERROR;
	uint8_t chipid = 0;

	mutex_lock(&o_ctrl->aw_ois_mutex);
	ret = ois_block_read_addr8_data8(o_ctrl, REG_CHIPID,
				CAMERA_SENSOR_I2C_TYPE_WORD, &chipid, 1);
	if (ret < 0) {
		AW_LOGE("read chipid error");
		mutex_unlock(&o_ctrl->aw_ois_mutex);
		return ret;
	}

	AW_LOGD("Chipid: 0x%02X", chipid);

	len += snprintf(buf + len, PAGE_SIZE - len, "Chipid: 0x%02X\n", chipid);
	mutex_unlock(&o_ctrl->aw_ois_mutex);

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
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	ssize_t len = 0;
	uint32_t version = 0;
	int ret = OIS_ERROR;
	uint8_t temp[4] = { 0 };

	mutex_lock(&o_ctrl->aw_ois_mutex);
	ret = ois_block_read_addr8_data8(o_ctrl, REG_VERSION,
				CAMERA_SENSOR_I2C_TYPE_WORD, temp, 4);
	if (ret < 0) {
		AW_LOGE("read version error");
		mutex_unlock(&o_ctrl->aw_ois_mutex);
		return ret;
	}

	version = (temp[3] << 24) | (temp[2] << 16) | (temp[1] << 8) | temp[0];
	AW_LOGD("version:v%d.%d.%d.%d", temp[3], temp[2], temp[1], temp[0]);

	len += snprintf(buf + len, PAGE_SIZE - len, "version: v%d.%d.%d.%d\n",
					temp[3], temp[2], temp[1], temp[0]);
	mutex_unlock(&o_ctrl->aw_ois_mutex);

	return len;
}

static ssize_t version_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	return count;
}

static ssize_t checkinfo_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	ssize_t len = 0;
	int i = 0;

	for (i = 0; i < 8; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"checkinfo_rd_0x%x 0x%02X 0x%02X 0x%02X ",
				i, g_aw86006_info.checkinfo_rd[i * 8 + 0],
				g_aw86006_info.checkinfo_rd[i * 8 + 1],
				g_aw86006_info.checkinfo_rd[i * 8 + 2]);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
				g_aw86006_info.checkinfo_rd[i * 8 + 3],
				g_aw86006_info.checkinfo_rd[i * 8 + 4],
				g_aw86006_info.checkinfo_rd[i * 8 + 5],
				g_aw86006_info.checkinfo_rd[i * 8 + 6],
				g_aw86006_info.checkinfo_rd[i * 8 + 7]);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"checkinfo_fw_0x%x 0x%02X 0x%02X 0x%02X ",
				i, g_aw86006_info.checkinfo_fw[i * 8 + 0],
				g_aw86006_info.checkinfo_fw[i * 8 + 1],
				g_aw86006_info.checkinfo_fw[i * 8 + 2]);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
				g_aw86006_info.checkinfo_fw[i * 8 + 3],
				g_aw86006_info.checkinfo_fw[i * 8 + 4],
				g_aw86006_info.checkinfo_fw[i * 8 + 5],
				g_aw86006_info.checkinfo_fw[i * 8 + 6],
				g_aw86006_info.checkinfo_fw[i * 8 + 7]);
	}

	return len;
}

static ssize_t checkinfo_store(struct class *class,
						struct class_attribute *attr,
						const char *buf, size_t count)
{
	return count;
}

static ssize_t standby_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	ssize_t len = 0;
	int ret = OIS_ERROR;
	uint8_t standby_flag = 0;

	mutex_lock(&o_ctrl->aw_ois_mutex);
	ret = aw86006_get_standby_flag(o_ctrl, &standby_flag);
	if (ret != OIS_SUCCESS)
		len += snprintf(buf + len, PAGE_SIZE - len,
					"aw_read_standby error!\n");
	else
		len += snprintf(buf + len, PAGE_SIZE - len,
					"standby_flag: %d\n", standby_flag);
	mutex_unlock(&o_ctrl->aw_ois_mutex);

	return len;
}

static ssize_t standby_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	/* struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl; */
	uint32_t databuf = 0;
	int ret = OIS_ERROR;

	ret = kstrtouint(buf, 0, &databuf);
	if (ret < 0) {
		AW_LOGE("kstrtouint fail");
		return ret;
	}

	return count;
}

static ssize_t jump_show(struct class *class, struct class_attribute *attr,
								char *buf)
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

static ssize_t jump_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	char databuf[10] = {0};
	int ret = OIS_ERROR;

	if (sscanf(buf, "%s", databuf) == 1) {
		mutex_lock(&o_ctrl->aw_ois_mutex);
		if (strcmp(databuf, "boot") == 0) {
			AW_LOGI("aw86006 will jump boot!");
			ret = aw86006_jump_boot_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Jump boot failed");
				goto err_exit;
			}
		} else if (strcmp(databuf, "move") == 0) {
			AW_LOGI("aw86006 will jump move!");
			ret = aw86006_jump_move_check(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Jump boot failed");
				goto err_exit;
			}
		} else if (strcmp(databuf, "reset") == 0) {
			AW_LOGI("aw86006 will reset!");
			ret = aw86006_reset(o_ctrl);
			if (ret != OIS_SUCCESS) {
				AW_LOGE("Jump boot failed");
				goto err_exit;
			}
		} else {
			AW_LOGE("jump [%s] is not support!", databuf);
		}

		mutex_unlock(&o_ctrl->aw_ois_mutex);
	}

	return count;

err_exit:
	mutex_unlock(&o_ctrl->aw_ois_mutex);
	return ret;
}

static int aw86006_raw_data_check(uint8_t *pdata, uint8_t size)
{
	uint8_t cnt = 0;

	while (cnt < size) {
		if (pdata != NULL && *pdata != 0xff)
			break;

		pdata++;
		cnt++;
	}

	if (cnt == size)
		return OIS_ERROR;

	return OIS_SUCCESS;
}

static int aw86006_accelgyro_dift_raw_data_check(struct cam_ois_ctrl_t *o_ctrl)
{
	uint8_t dift_val[12] = { 0 };
	int ret = 0;

	ois_block_read_addr8_data8(o_ctrl, 0xf84c, CAMERA_SENSOR_I2C_TYPE_WORD,
								dift_val, 12);
	ret = aw86006_raw_data_check(dift_val,
					sizeof(dift_val)/sizeof(dift_val[0]));
	if (ret < 0) {
		AW_LOGE("accelgyro_dift raw data is all 0xff\n");
		return OIS_ERROR;
	}

	return OIS_SUCCESS;
}

static int aw86006_get_accelgyro_dift(struct cam_ois_ctrl_t *o_ctrl,
						struct accelgyro_dift *ag_dift)
{
	uint8_t dift_val[12] = { 0 };

	ois_block_read_addr8_data8(o_ctrl, 0xf84c, CAMERA_SENSOR_I2C_TYPE_WORD,
								dift_val, 12);

	ag_dift->gyro_dift[AXIS_X] = (int16_t)((dift_val[1] << 8) | dift_val[0]);
	ag_dift->gyro_dift[AXIS_Y] = (int16_t)((dift_val[3] << 8) | dift_val[2]);
	ag_dift->gyro_dift[AXIS_Z] = (int16_t)((dift_val[5] << 8) | dift_val[4]);

	ag_dift->accel_dift[AXIS_X] = (int16_t)((dift_val[7] << 8) | dift_val[6]);
	ag_dift->accel_dift[AXIS_Y] = (int16_t)((dift_val[9] << 8) | dift_val[8]);
	ag_dift->accel_dift[AXIS_Z] = (int16_t)((dift_val[11] << 8) | dift_val[10]);

	AW_LOGI("gyro_dift: 0x%hx(%d) 0x%hx(%d) 0x%hx(%d)",
			ag_dift->gyro_dift[AXIS_X], ag_dift->gyro_dift[AXIS_X],
			ag_dift->gyro_dift[AXIS_Y], ag_dift->gyro_dift[AXIS_Y],
			ag_dift->gyro_dift[AXIS_Z], ag_dift->gyro_dift[AXIS_Z]);

	AW_LOGI("accel_dift: 0x%hx(%d) 0x%hx(%d) 0x%hx(%d)",
			ag_dift->accel_dift[AXIS_X], ag_dift->accel_dift[AXIS_X],
			ag_dift->accel_dift[AXIS_Y], ag_dift->accel_dift[AXIS_Y],
			ag_dift->accel_dift[AXIS_Z], ag_dift->accel_dift[AXIS_Z]);

	/* check gyro x/y dift */
	if(CHECK_DIFF(ag_dift->gyro_dift[AXIS_X], AW_GYROACCEL_DIFT_LIMIT) > 0) {
		AW_LOGE("gyro_dift_x error!!");
		return OIS_ERROR;
	}
	if(CHECK_DIFF(ag_dift->gyro_dift[AXIS_Y], AW_GYROACCEL_DIFT_LIMIT) > 0) {
		AW_LOGE("gyro_dift_y error!!");
		return OIS_ERROR;
	}
	/* check accel x/y dift */
	if(CHECK_DIFF(ag_dift->accel_dift[AXIS_X], AW_GYROACCEL_DIFT_LIMIT) > 0) {
		AW_LOGE("accel_dift_x error!!");
		return OIS_ERROR;
	}
	if(CHECK_DIFF(ag_dift->accel_dift[AXIS_Y], AW_GYROACCEL_DIFT_LIMIT) > 0) {
		AW_LOGE("accel_dift_y error!!");
		return OIS_ERROR;
	}

	return OIS_SUCCESS;
}

static int aw86006_set_normal_mode(struct cam_ois_ctrl_t *o_ctrl)
{
	int i = 0;
	uint8_t send_cmd[2] = { 0xae, 0x02 };
	uint8_t check_val = 0;

	/*unlock */
	do {
		ois_block_write_addr8_data8(o_ctrl, 0xff1f,
				CAMERA_SENSOR_I2C_TYPE_WORD, &send_cmd[0], 1);
		ois_block_read_addr8_data8(o_ctrl, 0xff1f,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0x01) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x01) {
		AW_LOGE("unlock error:, check_val: 0x%02x", check_val);
		return OIS_ERROR;
	}

	/* set normal mode */
	do {
		ois_block_write_addr8_data8(o_ctrl, 0xff11,
				CAMERA_SENSOR_I2C_TYPE_WORD, &send_cmd[1], 1);
		msleep(2);
		ois_block_read_addr8_data8(o_ctrl, 0xff10,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0) {
		AW_LOGE("enter normal error:, check_val: 0x%02x", check_val);
		return OIS_ERROR;
	}

	return OIS_SUCCESS;
}

static ssize_t gyro_offset_cali_show(struct class *class,
					struct class_attribute *attr, char *buf)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	struct accelgyro_dift ag_dift = {0};
	ssize_t len = 0;

	aw86006_get_accelgyro_dift(o_ctrl, &ag_dift);

	len += snprintf(buf + len, PAGE_SIZE - len,
					"gyro_dift: 0x%hx(%d) 0x%hx(%d) 0x%hx(%d)\n",
						ag_dift.gyro_dift[AXIS_X], ag_dift.gyro_dift[AXIS_X],
						ag_dift.gyro_dift[AXIS_Y], ag_dift.gyro_dift[AXIS_Y],
						ag_dift.gyro_dift[AXIS_Z], ag_dift.gyro_dift[AXIS_Z]);

	len += snprintf(buf + len, PAGE_SIZE - len,
					"accel_dift: 0x%hx(%d) 0x%hx(%d) 0x%hx(%d)\n",
						ag_dift.accel_dift[AXIS_X], ag_dift.accel_dift[AXIS_X],
						ag_dift.accel_dift[AXIS_Y], ag_dift.accel_dift[AXIS_Y],
						ag_dift.accel_dift[AXIS_Z], ag_dift.accel_dift[AXIS_Z]);

	return len;
}

static ssize_t gyro_offset_cali_store(struct class *class,
						struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	struct accelgyro_dift ag_dift;
	int i = 0;
	int ret = 0;
	uint8_t data_buf[5] = { 0 };
	uint8_t ois_mode = 0;
	uint8_t unlock_val[2] = { 0xa4, 0xac };
	uint8_t lock_val[2] = { 0xdc, 0xd4 };
	uint8_t trigger_cmd[4] = { 0x01, 0x02, 0x01, 0x01 };
	uint8_t exit_cmd[4] = { 0x00, 0x00, 0x00, 0x02 };
	uint8_t check_val = 0;

	if (sscanf(buf, "%d", &data_buf[0]) != 1) {
		AW_LOGE("input para error!");
		return OIS_ERROR;
	}

	AW_LOGI("aw86006 gyro offset cali start!");

	/* check accelgyro_diff */
	ret = aw86006_accelgyro_dift_raw_data_check(o_ctrl);
	if (ret < 0) {
		AW_LOGE("accelgyro_dift raw data error");
		return OIS_ERROR;
	}

	/* OIS off */
	ois_mode = OIS_DISABLE;
	do {
		ois_block_write_addr8_data8(o_ctrl, REG_OIS_ENABLE,
				CAMERA_SENSOR_I2C_TYPE_WORD, &ois_mode, 1);
		ois_block_read_addr8_data8(o_ctrl, REG_OIS_ENABLE,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0x00) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x00) {
		AW_LOGE("reg[0001]:0x%x != 0x00, i: %d", check_val, i);
		return OIS_ERROR;
	}

	/* enter normal mode */
	ret = aw86006_set_normal_mode(o_ctrl);
	if (ret < 0)
		return OIS_ERROR;

	/* unlock register */
	i = 0;
	do {
		ois_block_write_addr8_data8(o_ctrl, 0xf20f,
				CAMERA_SENSOR_I2C_TYPE_WORD, &unlock_val[0], 1);
		ois_block_read_addr8_data8(o_ctrl, 0xf20f,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0x01) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x01) {
		AW_LOGE("reg[f20f]:0x%x != 0x01, i: %d", check_val, i);
		return OIS_ERROR;
	}

	/* unlock register */
	i = 0;
	do {
		ois_block_write_addr8_data8(o_ctrl, 0xf8ff,
				CAMERA_SENSOR_I2C_TYPE_WORD, &unlock_val[1], 1);
		ois_block_read_addr8_data8(o_ctrl, 0xf8ff,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0x01) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x01) {
		AW_LOGE("reg[f8ff]:0x%x != 0x01, i: %d", check_val, i);
		return OIS_ERROR;
	}

	/* Read the data in the flash into the register, trigger_cmd[0]: 0x01 */
	ois_block_write_addr8_data8(o_ctrl, 0xf8e4, CAMERA_SENSOR_I2C_TYPE_WORD,
							&trigger_cmd[0], 1);
	msleep(5); /* delay 5ms at least */

	/* Select the gyroscope zero drift calibration function, trigger_cmd[1]: 0x02 */
	ois_block_write_addr8_data8(o_ctrl, 0xf200, CAMERA_SENSOR_I2C_TYPE_WORD,
							&trigger_cmd[1], 1);

	/* Enter mass production test mode, trigger_cmd[2]: 0x01 */
	i = 0;
	do {
		ois_block_write_addr8_data8(o_ctrl, 0xf203,
				CAMERA_SENSOR_I2C_TYPE_WORD, &trigger_cmd[2], 1);
		ois_block_read_addr8_data8(o_ctrl, 0xf203,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0x01) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x01) {
		AW_LOGE("reg[f203]:0x%x != 0x01, i: %d", check_val, i);
		return OIS_ERROR;
	}

	/* Trigger zero drift calibration of gyroscope, trigger_cmd[3]:0x01 */
	i = 0;
	do {
		ois_block_write_addr8_data8(o_ctrl, 0xf202,
				CAMERA_SENSOR_I2C_TYPE_WORD, &trigger_cmd[3], 1);
		ois_block_read_addr8_data8(o_ctrl, 0xf201,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0x11) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x11) {
		AW_LOGE("reg[f201]:0x%x != 0x11, i: %d", check_val, i);
		return OIS_ERROR;
	}

	msleep(1000); /* delay 1s at least */

	/* Check if the calibration is complete */
	i = 0;
	do {
		ois_block_read_addr8_data8(o_ctrl, 0xf201,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
		msleep(20);
	} while ((check_val != 0x21) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x21) {
		AW_LOGE("reg[f201]:0x%x != 0x21, i: %d", check_val, i);
		return OIS_ERROR;
	}

	/* Exit calibration mode */
	ois_block_write_addr8_data8(o_ctrl, 0xf200, CAMERA_SENSOR_I2C_TYPE_WORD,
							&exit_cmd[0], 1);

	i = 0;
	do {
		/* exit_cmd[2]:0x00 */
		ois_block_write_addr8_data8(o_ctrl, 0xf203,
				CAMERA_SENSOR_I2C_TYPE_WORD, &exit_cmd[2], 1);
		ois_block_read_addr8_data8(o_ctrl, 0xf203,
				CAMERA_SENSOR_I2C_TYPE_WORD, &check_val, 1);
	} while ((check_val != 0x00) && (i++ < AW_ERROR_LOOP));

	if (check_val != 0x00) {
		AW_LOGE("reg[f203]:0x%x != 0, i: %d", check_val, i);
		return OIS_ERROR;
	}

	/* Data update to flash, exit_cmd[3]: 0x02 */
	ois_block_write_addr8_data8(o_ctrl, 0xf8e4, CAMERA_SENSOR_I2C_TYPE_WORD,
							&exit_cmd[3], 1);

	msleep(20 * 2); /* 2: two sector */

	/* Lock register */
	ois_block_write_addr8_data8(o_ctrl, 0xf8ff, CAMERA_SENSOR_I2C_TYPE_WORD,
							&lock_val[0], 1);
	ois_block_write_addr8_data8(o_ctrl, 0xf20f, CAMERA_SENSOR_I2C_TYPE_WORD,
							&lock_val[1], 1);

	/* Read the zero drift value of the three-axis angular velocity and
	 * acceleration of the gyroscope */
	ret = aw86006_get_accelgyro_dift(o_ctrl, &ag_dift);
	if (ret < 0)
		AW_LOGE("aw86006 gyro offset cali error!");
	else
		AW_LOGI("aw86006 gyro offset cali ok!");

	return count;
}

static ssize_t slave_var_show(struct class *class, struct class_attribute *attr,
								char *buf)
{
	ssize_t len = 0;
	return len;
}

static ssize_t slave_var_store(struct class *class,
						struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
	uint32_t databuf[5] = { 0 };
	uint16_t temp_addr = 0;
	uint32_t addr_type = 1;

	AW_LOGI("enter");

	temp_addr = o_ctrl->io_master_info.cci_client->sid;

	if (sscanf(buf, "%x %x %x %x %x", &databuf[0], &databuf[1], &databuf[2],
					&databuf[3], &databuf[4]) == 5) {
		AW_LOGI("1.slave: 0x%02x, flag: 0x%02x, addr_type: 0x%02x",
					databuf[0], databuf[1], databuf[2]);
		AW_LOGI("reg: 0x%04x, val: 0x%02x", databuf[3], databuf[4]);

		o_ctrl->io_master_info.cci_client->sid = databuf[0];
		if (databuf[2] == 0x01)
			addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		else if (databuf[2] == 0x02)
			addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;

		if (databuf[1] == AW_SEQ_WRITE) {
			ois_block_write_addr8_data8(o_ctrl, databuf[3],
					addr_type, (uint8_t *) &databuf[4],
					CAMERA_SENSOR_I2C_TYPE_BYTE);
			/* callback read */
			databuf[4] = 0;
			ois_block_read_addr8_data8(o_ctrl, databuf[3],
					addr_type, (uint8_t *) &databuf[4],
					CAMERA_SENSOR_I2C_TYPE_BYTE);
		} else if (databuf[1] == AW_SEQ_READ) {
			databuf[4] = 0;
			ois_block_read_addr8_data8(o_ctrl, databuf[3],
					addr_type, (uint8_t *) &databuf[4],
					CAMERA_SENSOR_I2C_TYPE_BYTE);
		}
		AW_LOGI("2.slave: 0x%02x, flag: 0x%02x, addr_type: 0x%02x",
					databuf[0], databuf[1], databuf[2]);
		AW_LOGI("reg: 0x%04x, val: 0x%02x", databuf[3], databuf[4]);
	}

	o_ctrl->io_master_info.cci_client->sid = temp_addr;

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
static CLASS_ATTR_RW(standby);
static CLASS_ATTR_RW(jump);
static CLASS_ATTR_RW(gyro_offset_cali);
static CLASS_ATTR_RW(slave_var);

static int aw86006_create_sysfs(void)
{
	int ret = OIS_ERROR;

	AW_LOGI("Start");

	if (!ois_debug_class) {
		AW_LOGI("create aw86006_result_class!");
		ois_debug_class = class_create(THIS_MODULE, "aw86006_ois");

		ret = class_create_file(ois_debug_class, &class_attr_reg);
		if (ret < 0) {
			AW_LOGE("Create reg failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_awrw);
		if (ret < 0) {
			AW_LOGE("Create awrw failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_update);
		if (ret < 0) {
			AW_LOGE("Create update failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_erase);
		if (ret < 0) {
			AW_LOGE("Create erase failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_mode);
		if (ret < 0) {
			AW_LOGE("Create mode failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_chipid);
		if (ret < 0) {
			AW_LOGE("Create chipid failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_version);
		if (ret < 0) {
			AW_LOGE("Create version failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_checkinfo);
		if (ret < 0) {
			AW_LOGE("Create checkinfo failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_standby);
		if (ret < 0) {
			AW_LOGE("Create standby failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_jump);
		if (ret < 0) {
			AW_LOGE("Create jump failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class,
						&class_attr_gyro_offset_cali);
		if (ret < 0) {
			AW_LOGE("Create gyro_offset_cali failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(ois_debug_class, &class_attr_slave_var);
		if (ret < 0) {
			AW_LOGE("Create slave_var failed, ret: %d", ret);
			return ret;
		}
	}

	AW_LOGI("Creat sysfs debug success.");

	return 0;
}

static void aw86006_destroy_sysfs(void)
{
	AW_LOGI("Start");

	if (ois_debug_class) {
		class_remove_file(ois_debug_class, &class_attr_reg);
		class_remove_file(ois_debug_class, &class_attr_awrw);
		class_remove_file(ois_debug_class, &class_attr_update);
		class_remove_file(ois_debug_class, &class_attr_erase);
		class_remove_file(ois_debug_class, &class_attr_mode);
		class_remove_file(ois_debug_class, &class_attr_chipid);
		class_remove_file(ois_debug_class, &class_attr_version);
		class_remove_file(ois_debug_class, &class_attr_checkinfo);
		class_remove_file(ois_debug_class, &class_attr_standby);
		class_remove_file(ois_debug_class, &class_attr_jump);
		class_remove_file(ois_debug_class,
						&class_attr_gyro_offset_cali);
		class_remove_file(ois_debug_class, &class_attr_slave_var);
		class_destroy(ois_debug_class);
		ois_debug_class = NULL;
		AW_LOGD("destroy ois_debug_class done!");
	}

	AW_LOGI("End");
}

/*******************************************************************************
 * aw86006 ois init
 * Callback: Path: function
 ******************************************************************************/
int aw86006_ois_init(struct cam_ois_ctrl_t *o_ctrl)
{
	int loop = 0;

	AW_LOGI("enter, %s", AW86006_DRIVER_VERSION);

	/* awrw_ctrl: debug node awrw */
	do {
		o_ctrl->awrw_ctrl = kzalloc(sizeof(struct awrw_ctrl),
								GFP_KERNEL);
		if (!o_ctrl->awrw_ctrl) {
			AW_LOGE("Request memory for awrw_ctrl error");
			continue;
		}
		break;
	} while ((++loop) < AW_ERROR_LOOP);
	if (loop >= AW_ERROR_LOOP)
		return -ENOMEM;

	INIT_WORK(&o_ctrl->aw_fw_update_work,
					aw86006_firmware_update_work_routine);
	mutex_init(&o_ctrl->aw_ois_mutex);
	aw86006_create_sysfs(); /* adb debug node */

	g_o_ctrl = o_ctrl;

	AW_LOGI("exit");

	return OIS_SUCCESS;
}

int aw86006_ois_exit(struct cam_ois_ctrl_t *o_ctrl)
{
	AW_LOGI("enter");

	aw86006_destroy_sysfs();

	kfree(o_ctrl->awrw_ctrl);

	return OIS_SUCCESS;
}

MODULE_AUTHOR("<hushanping@awinic.com>");
MODULE_DESCRIPTION("AWINIC OIS Driver");
MODULE_LICENSE("GPL v2");
