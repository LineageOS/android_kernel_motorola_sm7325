// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

extern int dw9781c_check_fw_download(struct camera_io_master * io_master_info, const uint8_t *fwData, uint32_t fwSize);
extern void dw9781_post_firmware_download(struct camera_io_master * io_master_info, const uint8_t *fwData, uint32_t fwSize);
extern int aw86006_firmware_update(struct cam_ois_ctrl_t *o_ctrl, const struct firmware *fw);

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;
	bridge_params.dev_id = CAM_OIS;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	if (ois_acq_dev.device_handle <= 0) {
		CAM_ERR(CAM_OIS, "Can not create device handle");
		return -EFAULT;
	}
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;
	o_ctrl->prev_timestamp = 0;
	o_ctrl->curr_timestamp = 0;
	o_ctrl->is_first_vsync = 1;
	o_ctrl->is_video_mode  = false;
	o_ctrl->is_need_eis_data  = false;
	o_ctrl->q_timer_cnt    = QTIMER_SAMPLE_TIME*10*2;
	o_ctrl->mono_timestamp = 0;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	CAM_INFO(CAM_OIS, "OIS Power up successfully");

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;
cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info))
		CAM_ERR(CAM_OIS, "Power Down failed");

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	CAM_INFO(CAM_OIS, "OIS power down successed");

	camera_io_release(&o_ctrl->io_master_info);

	return rc;
}

static int cam_ois_update_time(struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t size = 0;
	uint32_t i = 0;
	uint64_t qtime_ns = 0;

	if (i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = cam_sensor_util_get_current_qtimer_ns(&qtime_ns);
	if (rc < 0) {
		CAM_ERR(CAM_OIS,
			"Failed to get current qtimer value: %d",
			rc);
		return rc;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_SEQ) {
			size = i2c_list->i2c_settings.size;
			/* qtimer is 8 bytes so validate here*/
			if (size < 8) {
				CAM_ERR(CAM_OIS, "Invalid write time settings");
				return -EINVAL;
			}
			for (i = 0; i < size; i++) {
				CAM_DBG(CAM_OIS, "time: reg_data[%d]: 0x%x",
					i, (qtime_ns & 0xFF));
				i2c_list->i2c_settings.reg_setting[i].reg_data =
					(qtime_ns & 0xFF);
				qtime_ns >>= 8;
			}
		}
	}

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
			if (o_ctrl->ic_name != NULL && strstr(o_ctrl->ic_name, "aw86006")) {
				if (i2c_list->i2c_settings.reg_setting[0].reg_addr == AW86006_PACKET_ENABLE &&
					i2c_list->i2c_settings.reg_setting[0].reg_data == 0x01)
					o_ctrl->is_video_mode = true;
				else
					o_ctrl->is_video_mode = false;
			}
			if (o_ctrl->ic_name != NULL && strstr(o_ctrl->ic_name, "dw9781c")) {
				if (i2c_list->i2c_settings.reg_setting[0].reg_addr == 0x7014 &&
					i2c_list->i2c_settings.reg_setting[0].reg_data == 0x00)
					o_ctrl->is_video_mode = true;
				else
					o_ctrl->is_video_mode = false;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(
				&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings),
				0);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed to seq write I2C settings: %d",
					rc);
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc == 1) {
					CAM_ERR(CAM_OIS,
						"i2c poll fails addr:data %x:%x",
						i2c_list->i2c_settings.reg_setting[i].reg_addr,
						i2c_list->i2c_settings.reg_setting[i].reg_data);
					return rc;
				}
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->ois_preprog_flag = ois_info->ois_preprog_flag;
		o_ctrl->ois_precoeff_flag = ois_info->ois_precoeff_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		o_ctrl->ois_postcalib_flag = ois_info->ois_postcalib_flag;
		o_ctrl->ois_fw_txn_data_sz = ois_info->ois_fw_txn_data_sz;
		o_ctrl->ois_fw_inc_addr = ois_info->ois_fw_inc_addr;
		o_ctrl->ois_fw_addr_type = ois_info->ois_fw_addr_type;
		o_ctrl->ois_fw_data_type = ois_info->ois_fw_data_type;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_fw_prog_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, total_idx, packet_idx;
	uint32_t                           txn_data_size, txn_regsetting_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	char                               name_prog[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	void                              *vaddr = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	if (strstr(o_ctrl->ois_name, "dw9781")) {
		if (!dw9781c_check_fw_download(&(o_ctrl->io_master_info), fw->data, fw->size)) {
			CAM_INFO(CAM_OIS, "Skip firmware download.");
			release_firmware(fw);
			return 0;
		}
		CAM_INFO(CAM_OIS, "Firmware download started.");
	}
	else if (strstr(o_ctrl->ois_name, "aw86006")) {
		mutex_lock(&o_ctrl->aw_ois_mutex);
		rc = aw86006_firmware_update(o_ctrl, fw);
		mutex_unlock(&o_ctrl->aw_ois_mutex);
		return rc;
	}

	total_bytes = fw->size;
	if(o_ctrl->ois_fw_txn_data_sz == 0)
		txn_data_size = total_bytes;
	else
		txn_data_size = o_ctrl->ois_fw_txn_data_sz;

	i2c_reg_setting.addr_type = o_ctrl->ois_fw_addr_type;
	i2c_reg_setting.data_type = o_ctrl->ois_fw_data_type;
	i2c_reg_setting.delay = 0;
	txn_regsetting_size = sizeof(struct cam_sensor_i2c_reg_array) * txn_data_size;
	vaddr = vmalloc(txn_regsetting_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	CAM_DBG(CAM_OIS, "fw len: %d, addr_type: %d, data_type: %d, chunck: %d, ois_fw_data_type:%d", total_bytes,
	                 i2c_reg_setting.addr_type,
	                 i2c_reg_setting.data_type,
	                 txn_data_size,
	                 o_ctrl->ois_fw_data_type);

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (vaddr);

	for (total_idx = 0, ptr = (uint8_t *)fw->data; total_idx < total_bytes;) {
		for (packet_idx = 0;
			(packet_idx < (txn_data_size/o_ctrl->ois_fw_data_type)) && (total_idx + (packet_idx*o_ctrl->ois_fw_data_type) < total_bytes);
			packet_idx ++, ptr += o_ctrl->ois_fw_data_type)
		{
			int regAddrOffset = 0;
			if(o_ctrl->ois_fw_inc_addr == 1)
				regAddrOffset = total_idx/o_ctrl->ois_fw_data_type + packet_idx;

			i2c_reg_setting.reg_setting[packet_idx].reg_addr =
				o_ctrl->opcode.prog + regAddrOffset;
			if (o_ctrl->ois_fw_data_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
				i2c_reg_setting.reg_setting[packet_idx].reg_data = (uint32_t)(*ptr << 8) | *(ptr+1);
			} else {
				i2c_reg_setting.reg_setting[packet_idx].reg_data = *ptr;
			}
			i2c_reg_setting.reg_setting[packet_idx].delay = 0;
			i2c_reg_setting.reg_setting[packet_idx].data_mask = 0;
			CAM_DBG(CAM_OIS, "OIS_FW Reg:[0x%04x]: 0x%04x P:0x%x",
			    i2c_reg_setting.reg_setting[packet_idx].reg_addr,
			    i2c_reg_setting.reg_setting[packet_idx].reg_data,
			    (ptr-(uint8_t *)fw->data));
		}
		i2c_reg_setting.size = packet_idx;
		if (o_ctrl->ois_fw_inc_addr == 1) {
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&i2c_reg_setting, 0);
		} else {
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&i2c_reg_setting, 1);
		}
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
			goto release_firmware;
		}
		total_idx += packet_idx*o_ctrl->ois_fw_data_type;
		CAM_DBG(CAM_OIS, "packet_idx: %d, total_idx: %d", packet_idx, total_idx);
	}

	if (strstr(o_ctrl->ois_name, "dw9781")) {
		dw9781_post_firmware_download(&(o_ctrl->io_master_info), fw->data, fw->size);
	}

release_firmware:
	vfree(vaddr);
	vaddr = NULL;
	txn_regsetting_size = 0;
	release_firmware(fw);

	return rc;
}

static int cam_ois_fw_coeff_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, total_idx, packet_idx;
	uint32_t                           txn_data_size, txn_regsetting_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	void                              *vaddr = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (strstr(o_ctrl->ois_name, "dw9781") || strstr(o_ctrl->ois_name, "aw86006")) {
		CAM_DBG(CAM_OIS, "not need download coeff fw for %s.", o_ctrl->ois_name);
		return 0;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);
	fw_name_coeff = name_coeff;

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	if(o_ctrl->ois_fw_txn_data_sz == 0)
		txn_data_size = total_bytes;
	else
		txn_data_size = o_ctrl->ois_fw_txn_data_sz;

	i2c_reg_setting.addr_type = o_ctrl->ois_fw_addr_type;
	i2c_reg_setting.data_type = o_ctrl->ois_fw_data_type;
	i2c_reg_setting.delay = 0;
	txn_regsetting_size = sizeof(struct cam_sensor_i2c_reg_array) * txn_data_size;
	vaddr = vmalloc(txn_regsetting_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (vaddr);

	for (total_idx = 0, ptr = (uint8_t *)fw->data; total_idx < total_bytes;) {
		for(packet_idx = 0;
			(packet_idx < txn_data_size) && (total_idx + packet_idx < total_bytes);
			packet_idx++, ptr++)
		{
			int regAddrOffset = 0;
			if(o_ctrl->ois_fw_inc_addr == 1)
				regAddrOffset = total_idx + packet_idx;

			i2c_reg_setting.reg_setting[packet_idx].reg_addr =
				o_ctrl->opcode.coeff + regAddrOffset;
			i2c_reg_setting.reg_setting[packet_idx].reg_data = *ptr;
			i2c_reg_setting.reg_setting[packet_idx].delay = 0;
			i2c_reg_setting.reg_setting[packet_idx].data_mask = 0;
		}
		i2c_reg_setting.size = packet_idx;
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
			&i2c_reg_setting, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
			goto release_firmware;
		}
		total_idx += packet_idx;
	}

release_firmware:
	vfree(vaddr);
	vaddr = NULL;
	txn_regsetting_size = 0;
	release_firmware(fw);

	return rc;
}

static int cam_ois_write_q_timer(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_sensor_i2c_reg_setting i2c_reg_setting = {NULL,1,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD,0};
	struct cam_sensor_i2c_reg_array i2c_write_settings = {QTIMER_ADDR,0x0000,0,0};
	int32_t rc        = 0;
	uint32_t data     = 0;
	uint64_t mono_time_ns = 0;
	struct timespec64 ts;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (o_ctrl->q_timer_cnt > 60000)
		o_ctrl->q_timer_cnt = QTIMER_SAMPLE_TIME*10*2;

	data = ++o_ctrl->q_timer_cnt;

	i2c_write_settings.reg_data = data;
	i2c_reg_setting.reg_setting = &(i2c_write_settings);

	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &(i2c_reg_setting));
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "Failed in Applying Q-timer settings");
	} else {
		ktime_get_boottime_ts64(&ts);
		mono_time_ns = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);
		o_ctrl->mono_timestamp = mono_time_ns;
	}

	CAM_DBG(CAM_OIS,"Write Q-timer %d, mono timestamp %lld", data, mono_time_ns);
	return rc;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		return -EINVAL;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_OIS, "Invalid packet params");
		return -EINVAL;
	}


	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				return -EINVAL;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				return -EINVAL;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					return rc;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					return rc;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					return rc;
				}
			} else if (((o_ctrl->ois_preprog_flag) != 0) &&
				o_ctrl->i2c_preprog_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS, "Received PreProg Settings");
				i2c_reg_settings = &(o_ctrl->i2c_preprog_data);
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"preprog parsing failed: %d", rc);
					return rc;
				}
			} else if (((o_ctrl->ois_precoeff_flag) != 0) &&
				o_ctrl->i2c_precoeff_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS, "Received PreCoeff Settings");
				i2c_reg_settings = &(o_ctrl->i2c_precoeff_data);
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"precoeff parsing failed: %d", rc);
					return rc;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					return rc;
				}
			} else if (((o_ctrl->ois_postcalib_flag) != 0) &&
				o_ctrl->i2c_postcalib_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS, "Received PostCalib Settings");
				i2c_reg_settings = &(o_ctrl->i2c_postcalib_data);
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"postcalib parsing failed: %d", rc);
					return rc;
				}
			}
			break;
			}
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				return rc;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->ois_preprog_flag) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_preprog_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply preprog settings");
				goto pwr_dwn;
			}
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_prog_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}

		if (o_ctrl->ois_precoeff_flag) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_precoeff_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply precoeff settings");
				goto pwr_dwn;
			}
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_coeff_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS COEFF FW Download");
				goto pwr_dwn;
			}
		}

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if ((rc == -EAGAIN) &&
			(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
			CAM_WARN(CAM_OIS,
				"CCI HW is restting: Reapplying INIT settings");
			usleep_range(1000, 1010);
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_init_data);
		}
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Cannot apply Init settings: rc = %d",
				rc);
			goto pwr_dwn;
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}

		if (o_ctrl->ois_postcalib_flag) {
			CAM_DBG(CAM_OIS, "starting post calib data");
			rc = cam_ois_apply_settings(o_ctrl,
			&o_ctrl->i2c_postcalib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply post calib data");
				goto pwr_dwn;
			}
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_preprog_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting PreProg data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_precoeff_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting PreCoeff data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_postcalib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting PostCalib data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}

		o_ctrl->is_need_eis_data  = false;

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			return rc;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_GYRO_OFFSET:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_gyro_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply gyro offset settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting gyro offset data: rc: %d", rc);
			return rc;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_READ: {
		struct cam_buf_io_cfg *io_cfg;
		struct i2c_settings_array i2c_read_settings;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_OIS, "No I/O configs to process");
			rc = -EINVAL;
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		/* validate read data io config */
		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, &io_cfg[0]);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_sensor_i2c_read_data(
			&i2c_read_settings,
			&o_ctrl->io_master_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
			delete_request(&i2c_read_settings);
			return rc;
		}

		if (csl_packet->num_io_configs > 1) {
			rc = cam_sensor_util_write_qtimer_to_io_buffer(
				&io_cfg[1]);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"write qtimer failed rc: %d", rc);
				delete_request(&i2c_read_settings);
				return rc;
			}
		}

		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed in deleting the read settings");
			return rc;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_WRITE_TIME: {
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_ERR(CAM_OIS,
				"Not in right state to write time to OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_time_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_ois_update_time(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot update time");
			return rc;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			return rc;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_ACTIVE_OIS_ROHM: {
		struct cam_buf_io_cfg *io_cfg;
		struct cam_buf_io_cfg *timestamp_io_cfg;
		struct i2c_settings_array i2c_read_settings;
		struct timespec64 ts;
		uintptr_t buf_addr = 0x0;
		size_t buf_size = 0;
		uint64_t timestamp;
		uint8_t *timestampBuf;
		uint32_t timestampBufLen;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs < 2) {
			CAM_ERR(CAM_OIS, "Not enough I/O Configs");
			rc = -EINVAL;
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		timestamp_io_cfg = &io_cfg[1];

		if (timestamp_io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "timestamp I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		if (cmd_desc == NULL) {
			CAM_ERR(CAM_OIS, "cmd desc is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_sensor_i2c_read_data(
			&i2c_read_settings,
			&o_ctrl->io_master_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
			delete_request(&i2c_read_settings);
			return rc;
		}

		ktime_get_boottime_ts64(&ts);
		timestamp =
			(uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);
		rc = cam_mem_get_cpu_buf(timestamp_io_cfg->mem_handle[0],
				&buf_addr, &buf_size);
		timestampBuf = (uint8_t *)buf_addr + timestamp_io_cfg->offsets[0];
		timestampBufLen =  buf_size - timestamp_io_cfg->offsets[0];
		if(timestampBufLen < sizeof(uint64_t)) {
			CAM_ERR(CAM_OIS, "Buffer not large enough for timestamp");
			return -EINVAL;
		}

		memcpy((void *)timestampBuf, (void *)&timestamp, sizeof(uint64_t));
		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed in deleting the read settings");
			return rc;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_ACTIVE_OIS_DONGWOON: {
		struct cam_buf_io_cfg *io_cfg;
		struct cam_buf_io_cfg *timestamp_io_cfg;
		struct i2c_settings_array i2c_read_settings;
		uintptr_t buf_addr = 0x0;
		size_t buf_size = 0;
		uint8_t *timestampBuf;
		uint32_t timestampBufLen;
		struct i2c_settings_list *i2c_list;
		uint8_t *read_buff = NULL;
		uint32_t buff_length = 0;
		uint32_t read_length = 0;
		uint32_t size;
		unsigned long rem_jiffies = 0;

		o_ctrl->is_need_eis_data  = true;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs < 2) {
			CAM_ERR(CAM_OIS, "Not enough I/O Configs");
			rc = -EINVAL;
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		timestamp_io_cfg = &io_cfg[1];

		if (timestamp_io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "timestamp I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		if (cmd_desc == NULL) {
			CAM_ERR(CAM_OIS, "cmd desc is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			return rc;
		}

		list_for_each_entry(i2c_list,
			&(i2c_read_settings.list_head), list) {
			if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
				rc = camera_io_dev_write(&(o_ctrl->io_master_info),
					&(i2c_list->i2c_settings));
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Failed in Applying i2c wrt settings");
					delete_request(&i2c_read_settings);
					return rc;
				}
			} else if (i2c_list->op_code == CAM_SENSOR_I2C_READ_SEQ) {
				read_buff     = i2c_list->i2c_settings.read_buff;
				buff_length   = i2c_list->i2c_settings.read_buff_len;
				read_length   = i2c_list->i2c_settings.size;

				CAM_DBG(CAM_OIS, "buff_length = %d, read_length = %d", buff_length, read_length);

				if (read_length > buff_length || buff_length < PACKET_BYTE*MAX_PACKET) {
					CAM_ERR(CAM_SENSOR,
					"Invalid buffer size, readLen: %d, bufLen: %d",
					read_length, buff_length);
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				rc = cam_mem_get_cpu_buf(timestamp_io_cfg->mem_handle[0],
						&buf_addr, &buf_size);
				timestampBuf    = (uint8_t *)buf_addr + timestamp_io_cfg->offsets[0];
				timestampBufLen = buf_size - timestamp_io_cfg->offsets[0];

				if(timestampBufLen < sizeof(uint64_t)) {
					CAM_ERR(CAM_OIS, "Buffer not large enough for timestamp");
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				rem_jiffies = wait_for_completion_timeout(&o_ctrl->ois_data_complete,
										msecs_to_jiffies(120));
				if (rem_jiffies == 0) {
					CAM_ERR(CAM_OIS, "Wait ois data completion timeout 120 ms");
					delete_request(&i2c_read_settings);
					return -ETIMEDOUT;
				}

				mutex_lock(&(o_ctrl->vsync_mutex));
				memcpy((void *)timestampBuf, (void *)&o_ctrl->prev_timestamp, sizeof(uint64_t));

				if (o_ctrl->ois_data_size <= buff_length)
					memcpy((void *)read_buff, (void *)o_ctrl->ois_data, o_ctrl->ois_data_size);
				else
					memcpy((void *)read_buff, (void *)o_ctrl->ois_data, buff_length);

				mutex_unlock(&(o_ctrl->vsync_mutex));
			} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
				size = i2c_list->i2c_settings.size;
				for (i = 0; i < size; i++) {
					rc = camera_io_dev_poll(
					&(o_ctrl->io_master_info),
					i2c_list->i2c_settings.reg_setting[i].reg_addr,
					i2c_list->i2c_settings.reg_setting[i].reg_data,
					0xFFFE,
					i2c_list->i2c_settings.addr_type,
					i2c_list->i2c_settings.data_type,
					i2c_list->i2c_settings.reg_setting[i].delay);
				}
				if ((rc == 1) || (rc < 0)) {
					CAM_ERR(CAM_OIS,
						"i2c poll fails rc: %d",rc);
					rc = -EINVAL;
					break;
				}
			}
		}
		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
			"Failed in deleting the read settings");
			return rc;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_ACTIVE_OIS_AW86006: {
		struct cam_buf_io_cfg *io_cfg;
		struct cam_buf_io_cfg *timestamp_io_cfg;
		struct i2c_settings_array i2c_read_settings;
		uintptr_t buf_addr = 0x0;
		size_t buf_size = 0;
		uint8_t *timestampBuf;
		uint32_t timestampBufLen;
		struct i2c_settings_list *i2c_list;
		uint8_t *read_buff = NULL;
		uint32_t buff_length = 0;
		uint32_t read_length = 0;
		unsigned long rem_jiffies = 0;

		o_ctrl->is_need_eis_data  = true;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs < 2) {
			CAM_ERR(CAM_OIS, "Not enough I/O Configs");
			rc = -EINVAL;
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		timestamp_io_cfg = &io_cfg[1];

		if (timestamp_io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "timestamp I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		if (cmd_desc == NULL) {
			CAM_ERR(CAM_OIS, "cmd desc is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			return rc;
		}

		list_for_each_entry(i2c_list,
			&(i2c_read_settings.list_head), list) {
			if (i2c_list->op_code == CAM_SENSOR_I2C_READ_SEQ) {
				read_buff     = i2c_list->i2c_settings.read_buff;
				buff_length   = i2c_list->i2c_settings.read_buff_len;
				read_length   = i2c_list->i2c_settings.size;

				CAM_DBG(CAM_OIS, "buff_length = %d, read_length = %d", buff_length, read_length);

				if (read_length > buff_length || buff_length < RING_BUFFER_LEN) {
					CAM_ERR(CAM_SENSOR,
					"Invalid buffer size, readLen: %d, bufLen: %d",
					read_length, buff_length);
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				rc = cam_mem_get_cpu_buf(timestamp_io_cfg->mem_handle[0],
						&buf_addr, &buf_size);
				timestampBuf    = (uint8_t *)buf_addr + timestamp_io_cfg->offsets[0];
				timestampBufLen = buf_size - timestamp_io_cfg->offsets[0];

				if(timestampBufLen < sizeof(uint64_t)) {
					CAM_ERR(CAM_OIS, "Buffer not large enough for timestamp");
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				rem_jiffies = wait_for_completion_timeout(&o_ctrl->ois_data_complete,
										msecs_to_jiffies(120));
				if (rem_jiffies == 0) {
					CAM_ERR(CAM_OIS, "Wait ois data completion timeout 120 ms");
					delete_request(&i2c_read_settings);
					return -ETIMEDOUT;
				}

				mutex_lock(&(o_ctrl->vsync_mutex));
				memcpy((void *)timestampBuf, (void *)&o_ctrl->prev_timestamp, sizeof(uint64_t));

				if (o_ctrl->ring_buff_size <= buff_length)
					memcpy((void *)read_buff, (void *)o_ctrl->ring_buff, o_ctrl->ring_buff_size);
				else
					memcpy((void *)read_buff, (void *)o_ctrl->ring_buff, buff_length);

				mutex_unlock(&(o_ctrl->vsync_mutex));
			}
		}
		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
			"Failed in deleting the read settings");
			return rc;
		}
		break;
	}
	case CAM_OIS_PACKET_OPCODE_ACTIVE_OIS_DONGWOON_QTIMER: {
		struct cam_buf_io_cfg *io_cfg;
		struct cam_buf_io_cfg *timestamp_io_cfg;
		struct i2c_settings_array i2c_read_settings;
		uintptr_t buf_addr = 0x0;
		size_t buf_size = 0;
		uint8_t *timestampBuf;
		uint32_t timestampBufLen;
		struct i2c_settings_list *i2c_list;
		uint8_t *read_buff = NULL;
		uint32_t buff_length = 0, read_length = 0;
		uint16_t sample_cnt = 0, flag = 0, latest_data_ts = 0;
		uint64_t delta_timestamp_ns = 0;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs < 2) {
			CAM_ERR(CAM_OIS, "Not enough I/O Configs");
			rc = -EINVAL;
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		timestamp_io_cfg = &io_cfg[1];

		if (timestamp_io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "timestamp I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		if (cmd_desc == NULL) {
			CAM_ERR(CAM_OIS, "cmd desc is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			return rc;
		}

		list_for_each_entry(i2c_list,
			&(i2c_read_settings.list_head), list) {
			if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
				rc = camera_io_dev_write(&(o_ctrl->io_master_info), &(i2c_list->i2c_settings));
				if (rc < 0) {
					CAM_ERR(CAM_OIS, "Failed in Applying i2c wrt settings");
					delete_request(&i2c_read_settings);
					return rc;
				}
			} else if (i2c_list->op_code == CAM_SENSOR_I2C_READ_SEQ) {
				read_buff     = i2c_list->i2c_settings.read_buff;
				buff_length   = i2c_list->i2c_settings.read_buff_len;
				read_length   = i2c_list->i2c_settings.size;

				CAM_DBG(CAM_OIS, "buff_length = %d, read_length = %d", buff_length, read_length);

				if (buff_length < read_length) {
					CAM_ERR(CAM_SENSOR, "Invalid buffer size, readLen: %d, bufLen: %d", read_length, buff_length);
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				rc = cam_mem_get_cpu_buf(timestamp_io_cfg->mem_handle[0],
						&buf_addr, &buf_size);
				timestampBuf    = (uint8_t *)buf_addr + timestamp_io_cfg->offsets[0];
				timestampBufLen = buf_size - timestamp_io_cfg->offsets[0];

				if(timestampBufLen < sizeof(uint64_t)) {
					CAM_ERR(CAM_OIS, "Buffer not large enough for timestamp");
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				rc = cam_ois_write_q_timer(o_ctrl);
				udelay(300);

				if (rc < 0) {
					CAM_ERR(CAM_OIS,"Write Q-timer failed rc: %d", rc);
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				// SM7325 support burst read
				rc = camera_io_dev_read_seq(
					&o_ctrl->io_master_info,
					i2c_list->i2c_settings.reg_setting[0].reg_addr,
					read_buff,
					i2c_list->i2c_settings.addr_type,
					i2c_list->i2c_settings.data_type,
					i2c_list->i2c_settings.size);
				if (rc < 0) {
					CAM_ERR(CAM_OIS, "Failed: seq read I2C settings: %d", rc);
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				flag           = (read_buff[0] >> 4) & 0xF;
				sample_cnt     = ((read_buff[0] & 0xF) << 8) | read_buff[1];
				latest_data_ts = (read_buff[2] << 8) | read_buff[3];

				CAM_DBG(CAM_OIS, "flag = %d, sample_cnt = %d, latest_data_ts = %d",
						flag, sample_cnt, latest_data_ts);

				if (flag != 0x2 && flag != 0x3) {
					CAM_ERR(CAM_OIS, "flag %d is not in correct status", flag);
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				if (sample_cnt == 0 || sample_cnt > QTIMER_MAX_SAMPLE) {
					CAM_ERR(CAM_OIS, "sample_cnt %d is out of range", sample_cnt);
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				if (latest_data_ts == 0 ||
					latest_data_ts > o_ctrl->q_timer_cnt ||
					((QTIMER_SAMPLE_TIME*10+3) + latest_data_ts) <= o_ctrl->q_timer_cnt) {
						CAM_ERR(CAM_OIS, "latest_data_ts %d is out of range, q_timer_cnt %d",
								latest_data_ts, o_ctrl->q_timer_cnt);
						delete_request(&i2c_read_settings);
						return -EINVAL;
				}

				delta_timestamp_ns = (o_ctrl->q_timer_cnt - latest_data_ts)*100000;
				o_ctrl->mono_timestamp -= delta_timestamp_ns; // latest ois data timestamp

				if (o_ctrl->mono_timestamp == 0) {
					CAM_ERR(CAM_OIS, "mono_timestamp is zero.");
					delete_request(&i2c_read_settings);
					return -EINVAL;
				}

				CAM_DBG(CAM_OIS,"latest ois data mono timestamp %lld", o_ctrl->mono_timestamp);

				memcpy((void *)timestampBuf, (void *)&o_ctrl->mono_timestamp, sizeof(uint64_t));
			}
		}
		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed in deleting the read settings");
			return rc;
		}
		break;
	}
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		return -EINVAL;
	}

	if (!rc)
		return rc;
pwr_dwn:
	//cam_ois_power_down(o_ctrl); /* ois will pown down in CAM_RELEASE_DEV when closed camera */
	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_gyro_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_gyro_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_gyro_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_gyro_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
