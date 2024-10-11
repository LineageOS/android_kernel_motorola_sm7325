/*
 * Copyright (C) 2021 Motorola Mobility LLC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"

#define DW9784_CHIP_ID_ADDRESS 0x7000
#define DW9784_CHECKSUM_ADDRESS 0x700C
#define DW9784_CHIP_ID 0x9784
#define FW_VER_CURR_ADDR 0x7001

#define EOK             0

#define FW_VERSION_OFFSET 10493
#define FW_CHECKSUM_OFFSET 10495

typedef struct
{
	unsigned int driverIc;
	unsigned int size;
	const uint16_t *fwContentPtr;
	uint16_t version;
	uint16_t checksum;
} FirmwareContex;

static FirmwareContex g_dw9784FirmwareContext;

static int32_t dw9784_cci_write(struct camera_io_master * io_master_info, uint16_t reg, uint16_t val)
{
	int32_t rc = 0;
	struct cam_sensor_i2c_reg_array reg_setting;
	struct cam_sensor_i2c_reg_setting wr_setting;

	reg_setting.reg_addr = reg;
	reg_setting.reg_data = val;
	reg_setting.delay = 0;
	reg_setting.data_mask = 0;
	wr_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	wr_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	wr_setting.reg_setting = &reg_setting;
	wr_setting.size = 1;
	wr_setting.delay = 0;
	rc = camera_io_dev_write(io_master_info, &wr_setting);
	return rc;
}

static int32_t dw9784_cci_read(struct camera_io_master * io_master_info, uint16_t reg, uint16_t *val)
{
	int32_t rc = 0;
	uint32_t regVal = 0;
	rc = camera_io_dev_read(io_master_info, reg, &regVal, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (!rc) {
		*val = (uint16_t)regVal;
	}
	return rc;
}

static void dw9784_delay_ms(uint32_t ms)
{
	usleep_range(ms*1000, ms*1000+10);
	return;
}

void dw9784_ois_reset(struct camera_io_master * io_master_info)
{
	CAM_INFO(CAM_OIS, "[dw9784_ois_reset] ois reset");
	dw9784_cci_write(io_master_info, 0xD002, 0x0001); /* DW9784_DBGc reset */
	dw9784_delay_ms(4);
	dw9784_cci_write(io_master_info, 0xD001, 0x0001); /* Active mode (DSP ON) */
	dw9784_delay_ms(25);                              /* ST gyro - over wait 25ms, default Servo On */
	dw9784_cci_write(io_master_info, 0xEBF1, 0x56FA); /* User protection release */
}

void dw9784_code_pt_off(struct camera_io_master * io_master_info)
{
	CAM_INFO(CAM_OIS, "[dw9784_code_pt_off] start");
	/* release all protection */
	dw9784_cci_write(io_master_info, 0xFD00, 0x5252);
	dw9784_delay_ms(1);
	CAM_INFO(CAM_OIS, "[dw9784_code_pt_off] finish");
}

void dw9784_pid_erase(struct camera_io_master * io_master_info)
{
	CAM_INFO(CAM_OIS, "[dw9784_pid_erase] start pid flash(IF) erase");
	dw9784_cci_write(io_master_info, 0xde03, 0x0000);			// page 0
	dw9784_delay_ms(1);
	dw9784_cci_write(io_master_info, 0xde04, 0x0008);			// page erase
	dw9784_delay_ms(10);											// need to delay after erase
	CAM_INFO(CAM_OIS, "[dw9784_pid_erase] finish");
}

void dw9784_fw_eflash_erase(struct camera_io_master * io_master_info)
{
	CAM_INFO(CAM_OIS, "[dw9784_fw_eflash_erase] start fw flash erase");
	dw9784_cci_write(io_master_info, 0xde03, 0x0000);			// 4k Sector_0
	dw9784_delay_ms(1);
	dw9784_cci_write(io_master_info, 0xde04, 0x0002);			// 4k Sector Erase
	dw9784_delay_ms(10);						// need to delay after erase

	dw9784_cci_write(io_master_info, 0xde03, 0x0008);			// 4k Sector_1
	dw9784_delay_ms(1);
	dw9784_cci_write(io_master_info, 0xde04, 0x0002);			// 4k Sector Erase
	dw9784_delay_ms(10);						// need to delay after erase

	dw9784_cci_write(io_master_info, 0xde03, 0x0010);			// 4k Sector_2
	dw9784_delay_ms(1);
	dw9784_cci_write(io_master_info, 0xde04, 0x0002);			// 4k Sector Erase
	dw9784_delay_ms(10);						// need to delay after erase

	dw9784_cci_write(io_master_info, 0xde03, 0x0018);			// 4k Sector_3
	dw9784_delay_ms(1);
	dw9784_cci_write(io_master_info, 0xde04, 0x0002);			// 4k Sector Erase
	dw9784_delay_ms(10);						// need to delay after erase

	dw9784_cci_write(io_master_info, 0xde03, 0x0020);			// 4k Sector_4
	dw9784_delay_ms(1);
	dw9784_cci_write(io_master_info, 0xde04, 0x0002);			// 4k Sector Erase
	dw9784_delay_ms(10);						// need to delay after erase
	CAM_INFO(CAM_OIS, "[dw9784_fw_eflash_erase] finish");
}

void dw9784_flash_acess(struct camera_io_master * io_master_info)
{
	CAM_INFO(CAM_OIS, "[dw9784_flash_acess] execution");
	/* release all protection */
	dw9784_cci_write(io_master_info, 0xFAFA, 0x98AC);
	dw9784_delay_ms(1);
	dw9784_cci_write(io_master_info, 0xF053, 0x70BD);
	dw9784_delay_ms(1);
}

int dw9784_whoami_chk(struct camera_io_master * io_master_info)
{
	uint16_t sec_chip_id;
	dw9784_cci_write(io_master_info, 0xD000, 0x0001); /* chip enable */
	dw9784_delay_ms(4);
	dw9784_cci_write(io_master_info, 0xD001, 0x0000); /* dsp off mode */
	dw9784_delay_ms(1);

	dw9784_flash_acess(io_master_info); /* All protection */

	dw9784_cci_read(io_master_info, 0xD060, &sec_chip_id); /* 2nd chip id */
	CAM_INFO(CAM_OIS, "[dw9784_ois_ready_check] sec_chip_id : 0x%04x", sec_chip_id);
	if(sec_chip_id != 0x0020)
	{
		CAM_ERR(CAM_OIS, "[dw9784] second_chip_id check fail : 0x%04X", sec_chip_id);
		CAM_ERR(CAM_OIS, "[dw9784] second_enter shutdown mode");
		dw9784_cci_write(io_master_info, 0xD000, 0x0000); /* ic */
		return -1;
	}

	dw9784_ois_reset(io_master_info); /* ois reset */
	return 0;
}

int dw9784_checksum_fw_chk(struct camera_io_master * io_master_info)
{
	/*
	Bit [0]: FW checksum error
	Bit [1]: Module cal. checksum error
	Bit [2]: Set cal. checksum error
	*/
	uint16_t reg_fw_checksum;			// 0x700C
	uint16_t reg_checksum_status;		// 0x700D

	dw9784_cci_read(io_master_info, 0x700C, &reg_fw_checksum);
	dw9784_cci_read(io_master_info, 0x700D, &reg_checksum_status);

	CAM_INFO(CAM_OIS, "[dw9784_checksum_fw_chk] reg_checksum_status : 0x%04X", reg_checksum_status);
	CAM_INFO(CAM_OIS, "[dw9784_checksum_fw_chk] reg_fw_checksum : 0x%04X", reg_fw_checksum);

	if( (reg_checksum_status & 0x0001) == 0)
	{
		CAM_INFO(CAM_OIS, "[dw9784_checksum_fw_chk] fw checksum pass");
		return 0;
	}else
	{
		CAM_ERR(CAM_OIS, "[dw9784_checksum_fw_chk] fw checksum error reg_fw_checksum : 0x%04X", reg_fw_checksum);
		return -1;
	}
}

static int dw9784_erase_mtp_rewritefw(struct camera_io_master * io_master_info)
{
	uint16_t FMC;
	CAM_INFO(CAM_OIS, "dw9784 erase for rewritefw starting..");
	dw9784_cci_write(io_master_info, 0xd001, 0x0000);
	dw9784_delay_ms(1);
	dw9784_flash_acess(io_master_info);

	dw9784_cci_write(io_master_info, 0xDE01, 0x0000); // FMC block FW select
	dw9784_delay_ms(1);
	dw9784_cci_read(io_master_info, 0xDE01, &FMC);
	if (FMC != 0)
	{
		CAM_INFO(CAM_OIS, "[dw9784_download_fw] FMC register value 1st warning : %04x", FMC);
		dw9784_cci_write(io_master_info, 0xDE01, 0x0000);
		dw9784_delay_ms(1);
		FMC = 0; // initialize FMC value

		dw9784_cci_read(io_master_info, 0xDE01, &FMC);
		if (FMC != 0)
		{
			CAM_ERR(CAM_OIS, "[dw9784_download_fw] 2nd FMC register value 2nd warning : %04x", FMC);
			CAM_ERR(CAM_OIS, "[dw9784_download_fw] stop f/w download");
			return -1;
		}
	}

	 /* code protection off */
	dw9784_code_pt_off(io_master_info);

	/* 512 byte page */
	dw9784_cci_write(io_master_info, 0xde03, 0x0027);
	/* page erase */
	dw9784_cci_write(io_master_info, 0xde04, 0x0008);
	dw9784_delay_ms(10);

	dw9784_cci_write(io_master_info, 0xd000, 0x0000); /* Shut download mode */

	return 0;
}

static int dw9784_prepare_fw_download(struct camera_io_master * io_master_info)
{
	/* step 1: MTP Erase and DSP Disable for firmware 0x8000 write */
	/* step 2: MTP setup                                           */
	/* step 3. FMC register check                                  */
	/* step 4. code protection off                                 */
	/* step 5. erase flash fw data                                 */
	uint16_t FMC;
	dw9784_cci_write(io_master_info, 0xd001, 0x0000);
	dw9784_flash_acess(io_master_info);

	dw9784_cci_write(io_master_info, 0xDE01, 0x0000); // FMC block FW select
	dw9784_delay_ms(1);
	dw9784_cci_read(io_master_info, 0xDE01, &FMC);
	if (FMC != 0)
	{
		CAM_INFO(CAM_OIS, "[dw9784_download_fw] FMC register value 1st warning : %04x", FMC);
		dw9784_cci_write(io_master_info, 0xDE01, 0x0000);
		dw9784_delay_ms(1);
		FMC = 0; // initialize FMC value

		dw9784_cci_read(io_master_info, 0xDE01, &FMC);
		if (FMC != 0)
		{
			CAM_ERR(CAM_OIS, "[dw9784_download_fw] 2nd FMC register value 2nd warning : %04x", FMC);
			CAM_ERR(CAM_OIS, "[dw9784_download_fw] stop f/w download");
			return -1;
		}
	}

	dw9784_code_pt_off(io_master_info);
	dw9784_fw_eflash_erase(io_master_info);
	CAM_INFO(CAM_OIS, "[dw9784_download_fw] start firmware download");
	return 0;
}

int dw9784_check_fw_download(struct camera_io_master * io_master_info, const uint8_t *fwData, uint32_t fwSize)
{
	uint8_t ret;
	uint8_t needDownload = 0;
	uint16_t fwchecksum = 0;
	uint16_t first_chip_id = 0;
	uint16_t chip_checksum = 0;
	uint16_t fw_version_current = 0;
	uint16_t fw_version_latest = 0;

	if (io_master_info == NULL) {
		CAM_ERR(CAM_OIS, "FATAL: OIS CCI context error!!!");
		return -1;
	}

	if (fwData == NULL || fwSize < FW_VERSION_OFFSET*sizeof(uint16_t)) {
		CAM_ERR(CAM_OIS, "FATAL: firmware buffer(%p) is NULL or size(%d) abnormal!!!", fwData, fwSize);
		return -1;
	}

	g_dw9784FirmwareContext.driverIc = 0x9784;
	g_dw9784FirmwareContext.fwContentPtr = (const uint16_t *)fwData;
	g_dw9784FirmwareContext.size = fwSize;
	g_dw9784FirmwareContext.version = *(g_dw9784FirmwareContext.fwContentPtr+FW_VERSION_OFFSET);
	g_dw9784FirmwareContext.version = ((g_dw9784FirmwareContext.version << 8) & 0xff00) |
	                                  ((g_dw9784FirmwareContext.version >> 8) & 0xff);
	g_dw9784FirmwareContext.checksum = *(g_dw9784FirmwareContext.fwContentPtr+FW_CHECKSUM_OFFSET);
	g_dw9784FirmwareContext.checksum = ((g_dw9784FirmwareContext.checksum << 8) & 0xff00) |
	                                   ((g_dw9784FirmwareContext.checksum >> 8) & 0xff);
	if (dw9784_whoami_chk(io_master_info) != 0) {
		CAM_ERR(CAM_OIS, "[dw9784] second chip id check fail");
		return -1;
	}

	dw9784_cci_read(io_master_info, DW9784_CHIP_ID_ADDRESS, &first_chip_id);
	dw9784_cci_read(io_master_info, 0x7001, &chip_checksum);
	CAM_INFO(CAM_OIS, "[dw9784] FW_VER_PHONE_MAKER : 0x%x", chip_checksum);
	dw9784_cci_read(io_master_info, 0x7002, &chip_checksum);
	CAM_INFO(CAM_OIS, "[dw9784] FW_DATE_PHONE_MAKER : 0x%x", chip_checksum);

	CAM_INFO(CAM_OIS, "[dw9784] first_chip_id : 0x%x", first_chip_id);
	if (first_chip_id != DW9784_CHIP_ID) { /* first_chip_id verification failed */
		CAM_INFO(CAM_OIS, "[dw9784] start flash download:: size:%d, version:0x%x",
			g_dw9784FirmwareContext.size, g_dw9784FirmwareContext.version);
		needDownload = 1;
		ret = dw9784_prepare_fw_download(io_master_info); /* Need to forced update OIS firmware again. */
	} else {
		fwchecksum = dw9784_checksum_fw_chk(io_master_info);
		if(fwchecksum != 0)
		{
			needDownload = 1;
			CAM_INFO(CAM_OIS, "[dw9784] firmware checksum error");
		}

		dw9784_cci_read(io_master_info, FW_VER_CURR_ADDR, &fw_version_current);
		fw_version_latest = g_dw9784FirmwareContext.version; /*Firmware version read from file content.*/

		CAM_INFO(CAM_OIS, "[dw9784] fw_version_current = 0x%x, fw_version_latest = 0x%x",
			              fw_version_current, fw_version_latest);

		/* download firmware, check if need update, download firmware to flash */
		if (needDownload || ((fw_version_current & 0xFFFF) != (fw_version_latest & 0xFFFF))) {
			needDownload = 1;

			CAM_INFO(CAM_OIS, "[dw9784] start flash download:: size:%d, version:0x%x needDownload %d",
			                 g_dw9784FirmwareContext.size, g_dw9784FirmwareContext.version, needDownload);

			ret = dw9784_prepare_fw_download(io_master_info);
			CAM_INFO(CAM_OIS, "[dw9784] flash download::vendor_dw9784");
			if (ret != EOK) {
				dw9784_erase_mtp_rewritefw(io_master_info);
				CAM_ERR(CAM_OIS, "[dw9784] firmware download error, ret = 0x%x", ret);
				CAM_ERR(CAM_OIS, "[dw9784] change dw9784 state to shutdown mode");
				needDownload = 1;
			}
		} else {
			CAM_INFO(CAM_OIS, "[dw9784] ois firmware version is updated, skip download");
		}
	}
	return needDownload;
}
EXPORT_SYMBOL(dw9784_check_fw_download);

int dw9784_check_if_download(struct camera_io_master * io_master_info)
{
	uint16_t FMC;

	/* step 1. Writes 512Byte FW(PID) data to IF flash.	(FMC register check) */
	dw9784_cci_write(io_master_info, 0xDE01, 0x1000);
	dw9784_delay_ms(1);
	dw9784_cci_read(io_master_info,0xDE01, &FMC);
	dw9784_delay_ms(1);
	if (FMC != 0x1000)
	{
		CAM_INFO(CAM_OIS, "[dw9784_download_fw] IF FMC register value 1st warning : %04x", FMC);
		dw9784_cci_write(io_master_info, 0xDE01, 0x1000);
		dw9784_delay_ms(1);
		FMC = 0; // initialize FMC value

		dw9784_cci_read(io_master_info,0xDE01, &FMC);
		if (FMC != 0x1000)
		{
			CAM_ERR(CAM_OIS, "[dw9784_download_fw] 2nd IF FMC register value 2nd fail : %04x", FMC);
			CAM_ERR(CAM_OIS, "[dw9784_download_fw] stop firmware download");
			return -1;
		}
	}

	/* step 2. erease IF(FW/PID) eFLASH  */
	dw9784_pid_erase(io_master_info);

	CAM_INFO(CAM_OIS, "[dw9784_download_fw] start firmware/pid download");
	return 0;
}
EXPORT_SYMBOL(dw9784_check_if_download);

void dw9784_post_firmware_download(struct camera_io_master * io_master_info)
{
	uint16_t fwchecksum = 0;

	dw9784_ois_reset(io_master_info);
	fwchecksum = dw9784_checksum_fw_chk(io_master_info);
	if(fwchecksum != 0)
	{
		CAM_ERR(CAM_OIS, "[dw9784] firmware checksum error");
		dw9784_erase_mtp_rewritefw(io_master_info);
		CAM_ERR(CAM_OIS, "[dw9784] change dw9784 state to shutdown mode");
	}
	return;
}

EXPORT_SYMBOL(dw9784_post_firmware_download);
