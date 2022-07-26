// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include "cam_ois_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_ois_soc.h"
#include "cam_ois_core.h"
#include "cam_debug_util.h"
#include "camera_main.h"

extern int aw86006_ois_init(struct cam_ois_ctrl_t *o_ctrl); /* awinic add */
extern int aw86006_ois_exit(struct cam_ois_ctrl_t *o_ctrl); /* awinic add */

static int cam_ois_clear_data_ready(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting = {NULL,1,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD,0};
	struct cam_sensor_i2c_reg_array i2c_write_settings = {0x70DA,0x0000,0,0};
	int32_t rc = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	i2c_reg_setting.reg_setting = &(i2c_write_settings);

	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &(i2c_reg_setting));
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "Failed in Applying i2c wrt settings");
	}

	CAM_DBG(CAM_OIS,"Clear data-ready success");
	return rc;
}

static int cam_ois_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_ois_ctrl_t *o_ctrl =
		v4l2_get_subdevdata(sd);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "o_ctrl ptr is NULL");
			return -EINVAL;
	}

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));

	return 0;
}

static int cam_ois_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open(CAM_OIS);

	if (crm_active) {
		CAM_DBG(CAM_OIS, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}

	return cam_ois_subdev_close_internal(sd, fh);
}

static long cam_ois_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int                       rc     = 0;
	struct cam_ois_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ois_driver_cmd(o_ctrl, arg);
		if (rc)
			CAM_ERR(CAM_OIS,
				"Failed with driver cmd: %d", rc);
		break;
	case CAM_SD_SHUTDOWN:
		if (!cam_req_mgr_is_shutdown()) {
			CAM_ERR(CAM_CORE, "SD shouldn't come from user space");
			return 0;
		}
		rc = cam_ois_subdev_close_internal(sd, NULL);
		break;
	default:
		CAM_ERR(CAM_OIS, "Wrong IOCTL cmd: %u", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static int32_t cam_ois_update_i2c_info(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_ois_i2c_info_t *i2c_info)
{
	struct cam_sensor_cci_client        *cci_client = NULL;

	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = o_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_OIS, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = o_ctrl->cci_i2c_master;
		cci_client->sid = (i2c_info->slave_addr) >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long cam_ois_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_OIS,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ois_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_OIS,
				"Failed in ois suddev handling rc %d",
				rc);
			return rc;
		}
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid compat ioctl: %d", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_OIS,
				"Failed to copy from user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static const struct v4l2_subdev_internal_ops cam_ois_internal_ops = {
	.close = cam_ois_subdev_close,
};

static struct v4l2_subdev_core_ops cam_ois_subdev_core_ops = {
	.ioctl = cam_ois_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_ois_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_ois_subdev_ops = {
	.core = &cam_ois_subdev_core_ops,
};

static int cam_ois_init_subdev_param(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;

	o_ctrl->v4l2_dev_str.internal_ops = &cam_ois_internal_ops;
	o_ctrl->v4l2_dev_str.ops = &cam_ois_subdev_ops;
	strlcpy(o_ctrl->device_name, CAM_OIS_NAME,
		sizeof(o_ctrl->device_name));
	o_ctrl->v4l2_dev_str.name = o_ctrl->device_name;
	o_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	o_ctrl->v4l2_dev_str.ent_function = CAM_OIS_DEVICE_TYPE;
	o_ctrl->v4l2_dev_str.token = o_ctrl;
	 o_ctrl->v4l2_dev_str.close_seq_prior = CAM_SD_CLOSE_MEDIUM_PRIORITY;

	rc = cam_register_subdev(&(o_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_OIS, "fail to create subdev");

	return rc;
}

static int cam_ois_i2c_driver_probe(struct i2c_client *client,
	 const struct i2c_device_id *id)
{
	int                          rc = 0;
	struct cam_ois_ctrl_t       *o_ctrl = NULL;
	struct cam_ois_soc_private  *soc_private = NULL;

	if (client == NULL || id == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args client: %pK id: %pK",
			client, id);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_OIS, "i2c_check_functionality failed");
		goto probe_failure;
	}

	o_ctrl = kzalloc(sizeof(*o_ctrl), GFP_KERNEL);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "kzalloc failed");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, o_ctrl);

	o_ctrl->soc_info.dev = &client->dev;
	o_ctrl->soc_info.dev_name = client->name;
	o_ctrl->ois_device_type = MSM_CAMERA_I2C_DEVICE;
	o_ctrl->io_master_info.master_type = I2C_MASTER;
	o_ctrl->io_master_info.client = client;

	soc_private = kzalloc(sizeof(struct cam_ois_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto octrl_free;
	}

	o_ctrl->soc_info.soc_private = soc_private;
	rc = cam_ois_driver_soc_init(o_ctrl);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: cam_sensor_parse_dt rc %d", rc);
		goto soc_free;
	}

	rc = cam_ois_init_subdev_param(o_ctrl);
	if (rc)
		goto soc_free;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;

	return rc;

soc_free:
	kfree(soc_private);
octrl_free:
	kfree(o_ctrl);
probe_failure:
	return rc;
}

static int cam_ois_i2c_driver_remove(struct i2c_client *client)
{
	int                             i;
	struct cam_ois_ctrl_t          *o_ctrl = i2c_get_clientdata(client);
	struct cam_hw_soc_info         *soc_info;
	struct cam_ois_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_INFO(CAM_OIS, "i2c driver remove invoked");
	soc_info = &o_ctrl->soc_info;

	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_ois_soc_private *)soc_info->soc_private;
	power_info = &soc_private->power_info;

	kfree(o_ctrl->soc_info.soc_private);
	v4l2_set_subdevdata(&o_ctrl->v4l2_dev_str.sd, NULL);
	kfree(o_ctrl);

	return 0;
}

static irqreturn_t cam_aw86006_ois_vsync_irq_thread(int irq, void *data)
{
	struct cam_ois_ctrl_t *o_ctrl = data;
	int rc = -EINVAL, handled = IRQ_NONE, sample_cnt = 0, delay_time = 0;
	uint64_t mono_time_ns;
	struct timespec64 ts;
	uint8_t *read_buff;
	uint32_t k, read_len;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return IRQ_NONE;
	}

	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_WARN(CAM_OIS, "Not in right state to read Eis data: %d", o_ctrl->cam_ois_state);
		return IRQ_NONE;
	}

	if (o_ctrl->is_video_mode == false ||
		o_ctrl->is_need_eis_data == false) {
		CAM_DBG(CAM_OIS, "No need to read Eis data: %d %d", o_ctrl->is_video_mode, o_ctrl->is_need_eis_data);
		return IRQ_NONE;
	}

	if (!mutex_trylock(&o_ctrl->vsync_mutex)) {
		CAM_WARN(CAM_OIS, "try to get mutex fail, skip this irq");
		return IRQ_NONE;
	}

	ktime_get_boottime_ts64(&ts);
	mono_time_ns = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);

	CAM_DBG(CAM_OIS, "aw86006 ois vsync sof mono timestamp is %lld", mono_time_ns);

	o_ctrl->prev_timestamp = o_ctrl->curr_timestamp;
	o_ctrl->curr_timestamp = mono_time_ns;

	// when the first vsync arrived, return
	if (o_ctrl->is_first_vsync) {
		o_ctrl->is_first_vsync = 0;
		rc = -EINVAL;
		goto release_mutex;
	}

	memset(o_ctrl->ring_buff, 0, o_ctrl->ring_buff_size);
	read_buff = o_ctrl->ring_buff;

	for (k = 0; k < RING_BUFFER_LEN/READ_BYTE + 1; k++) {
		if (k == RING_BUFFER_LEN/READ_BYTE)
			read_len = RING_BUFFER_LEN%READ_BYTE;
		else
			read_len = READ_BYTE;

		if (read_len == 0) {
			CAM_WARN(CAM_OIS, "read length is 0, break loop read");
			break;
		}

		rc = camera_io_dev_ois_read_seq(
			&o_ctrl->io_master_info,
			AW86006_PACKET_ADDR,
			read_buff + k*READ_BYTE,
			CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			read_len);

		if (rc < 0) {
			CAM_ERR(CAM_OIS, "failed to read ois data: %d", rc);
			goto release_mutex;
		}
	}

	sample_cnt = read_buff[0];
	delay_time = read_buff[1];

	CAM_DBG(CAM_OIS,"sample_cnt = %d, delay_time = %d (100 us)", sample_cnt, delay_time);

	if (sample_cnt == 0 || sample_cnt > AW86006_MAX_SAMPLE) {
		CAM_WARN(CAM_OIS,"sample_cnt %d is invalid, skip the data", sample_cnt);
		rc = -EINVAL;
		goto release_mutex;
	}

release_mutex:
	if (rc < 0) {
		memset(o_ctrl->ring_buff, 0, o_ctrl->ring_buff_size);
		handled = IRQ_NONE;
	} else
		handled = IRQ_HANDLED;

	mutex_unlock(&(o_ctrl->vsync_mutex));

	if (rc >= 0)
		complete(&o_ctrl->ois_data_complete);

	return handled;
}

static irqreturn_t cam_ois_vsync_irq_thread(int irq, void *data)
{
	struct cam_ois_ctrl_t *o_ctrl = data;
	int rc = 0, handled = IRQ_NONE, packet_cnt = 0, sample_cnt = 0;
	uint64_t mono_time_ns;
	struct timespec64 ts;
	uint8_t *read_buff;
	uint32_t k, read_len;
	uint32_t data_ready = 0xFFFF;
	int i = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return IRQ_NONE;
	}

	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_WARN(CAM_OIS, "Not in right state to read OIS: %d", o_ctrl->cam_ois_state);
		goto release_mutex;
	}

	if(o_ctrl->is_video_mode == false ||
		o_ctrl->is_need_eis_data == false) {
		CAM_WARN(CAM_OIS, "No need to read Eis data %d %d", o_ctrl->is_video_mode, o_ctrl->is_need_eis_data);
		return IRQ_NONE;
	}

	if (!mutex_trylock(&o_ctrl->vsync_mutex)) {
		CAM_ERR(CAM_OIS, "try lock fail, skip this irq");
		return IRQ_NONE;
	}

	ktime_get_boottime_ts64(&ts);
	mono_time_ns = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);

	CAM_DBG(CAM_OIS, "vsync sof mono timestamp is %lld", mono_time_ns);

	o_ctrl->prev_timestamp = o_ctrl->curr_timestamp;
	o_ctrl->curr_timestamp = mono_time_ns;

	// when the first vsync arrived, clear data-ready, and return.
	if (o_ctrl->is_first_vsync) {
		o_ctrl->is_first_vsync = 0;
		rc = cam_ois_clear_data_ready(o_ctrl);
		udelay(1000);
		goto release_mutex;
	}

	memset(o_ctrl->ois_data, 0, o_ctrl->ois_data_size);
	read_buff = o_ctrl->ois_data;

	for (i = 0; i < READ_COUNT; i++) {
		rc = camera_io_dev_read(
			&o_ctrl->io_master_info,
			DATA_READY_ADDR,
			&data_ready,
			CAMERA_SENSOR_I2C_TYPE_WORD,
			CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "failed to read OIS 0x70da reg rc: %d", rc);
			goto release_mutex;
		}
		if (data_ready == DATA_READY) {
			CAM_DBG(CAM_OIS, "data_ready == 0x0001 i = %d", i);
			break;
		} else if (data_ready != DATA_READY && i < READ_COUNT - 1) {
			CAM_ERR(CAM_OIS, "data_ready != 0x0001 i = %d", i);
			udelay(1000);
		} else {
			CAM_ERR(CAM_OIS, "data_ready check fail i = %d", i);
			goto release_mutex;
		}
	}
	do {
		if (packet_cnt > 0 && packet_cnt < MAX_PACKET)
			read_buff += PACKET_BYTE;

		// SM6375: CCI_VERSION_1_2_9 can only support read 0xE bytes data one time.
		for (k = 0; k < PACKET_BYTE/READ_BYTE + 1; k++) {
			if (k == PACKET_BYTE/READ_BYTE)
				read_len = PACKET_BYTE%READ_BYTE;
			else
				read_len = READ_BYTE;

			if (read_len == 0) {
				CAM_WARN(CAM_OIS, "Read length is zero, break loop read");
				break;
			}

			rc = camera_io_dev_ois_read_seq(
				&o_ctrl->io_master_info,
				PACKET_ADDR + k*READ_BYTE/2,
				read_buff + k*READ_BYTE,
				CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_WORD,
				read_len);

			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed: seq read I2C settings: %d", rc);
				goto release_mutex;
			}

			if (k == 0 && packet_cnt == 0) {
				sample_cnt = read_buff[1];
				if (sample_cnt != 0) {
					packet_cnt = (sample_cnt + 9)/10;
					CAM_DBG(CAM_OIS,"sample data = 0x%x, packet_cnt = %d", sample_cnt, packet_cnt);

					// we only need max 30 sample.
					if (packet_cnt > MAX_PACKET || sample_cnt > MAX_SAMPLE) {
						CAM_WARN(CAM_OIS,"Too many packet, skip this read");
						rc = -EINVAL;
						goto release_mutex;
					}
				} else {
					CAM_WARN(CAM_OIS,"No-fatal: sample data is zero, break the loop read");
					goto release_mutex;
				}
			}
		}

		if (sample_cnt > 10 && packet_cnt > 1) {
			CAM_WARN(CAM_OIS,"more than 1 packet, clear data-ready and read next packet");
			rc = cam_ois_clear_data_ready(o_ctrl);
			udelay(1000);

			if (rc < 0) {
				CAM_ERR(CAM_OIS,"Write failed rc: %d", rc);
				goto release_mutex;
			}
		}

		packet_cnt--;
	} while(packet_cnt > 0);

release_mutex:
	if (rc < 0) {
		memset(o_ctrl->ois_data, 0, o_ctrl->ois_data_size);
		handled = IRQ_NONE;
	} else
		handled = IRQ_HANDLED;

	mutex_unlock(&(o_ctrl->vsync_mutex));

	complete(&o_ctrl->ois_data_complete);

	return handled;
}

static int cam_ois_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t                         rc = 0;
	struct cam_ois_ctrl_t          *o_ctrl = NULL;
	struct cam_ois_soc_private     *soc_private = NULL;
	struct platform_device *pdev = to_platform_device(dev);

	o_ctrl = kzalloc(sizeof(struct cam_ois_ctrl_t), GFP_KERNEL);
	if (!o_ctrl)
		return -ENOMEM;

	o_ctrl->soc_info.pdev = pdev;
	o_ctrl->pdev = pdev;
	o_ctrl->soc_info.dev = &pdev->dev;
	o_ctrl->soc_info.dev_name = pdev->name;

	o_ctrl->ois_device_type = MSM_CAMERA_PLATFORM_DEVICE;

	o_ctrl->ring_buff_size = RING_BUFFER_LEN;
	o_ctrl->ring_buff = kzalloc(o_ctrl->ring_buff_size, GFP_KERNEL);
	if (!o_ctrl->ring_buff)
		goto free_o_ctrl;

	o_ctrl->ois_data_size = PACKET_BYTE*MAX_PACKET;
	o_ctrl->ois_data = kzalloc(o_ctrl->ois_data_size, GFP_KERNEL);
	if (!o_ctrl->ois_data)
		goto free_ring_buff;

	o_ctrl->io_master_info.master_type = CCI_MASTER;
	o_ctrl->io_master_info.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!o_ctrl->io_master_info.cci_client)
		goto free_ois_data;

	soc_private = kzalloc(sizeof(struct cam_ois_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_cci_client;
	}
	o_ctrl->soc_info.soc_private = soc_private;
	soc_private->power_info.dev  = &pdev->dev;

	INIT_LIST_HEAD(&(o_ctrl->i2c_init_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_preprog_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_precoeff_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_calib_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_postcalib_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_mode_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_gyro_data.list_head));
	mutex_init(&(o_ctrl->ois_mutex));
	rc = cam_ois_driver_soc_init(o_ctrl);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: soc init rc %d", rc);
		goto free_soc;
	}

	rc = cam_ois_init_subdev_param(o_ctrl);
	if (rc)
		goto free_soc;

	rc = cam_ois_update_i2c_info(o_ctrl, &soc_private->i2c_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: to update i2c info rc %d", rc);
		goto unreg_subdev;
	}
	o_ctrl->bridge_intf.device_hdl = -1;

	platform_set_drvdata(pdev, o_ctrl);
	o_ctrl->cam_ois_state = CAM_OIS_INIT;

	mutex_init(&(o_ctrl->vsync_mutex));
	init_completion(&o_ctrl->ois_data_complete);

	if (o_ctrl->ic_name != NULL && strstr(o_ctrl->ic_name, "aw86006"))
		aw86006_ois_init(o_ctrl);

	if (o_ctrl->is_ois_vsync_irq_supported) {
		o_ctrl->vsync_irq = platform_get_irq_optional(pdev, 0);

		if (o_ctrl->vsync_irq > 0) {
			CAM_DBG(CAM_OIS, "get ois-vsync irq: %d", o_ctrl->vsync_irq);

			if (o_ctrl->ic_name != NULL && strstr(o_ctrl->ic_name, "aw86006")) {
				rc = devm_request_threaded_irq(dev,
								o_ctrl->vsync_irq,
								NULL,
								cam_aw86006_ois_vsync_irq_thread,
								(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
								"aw86006-vsync-irq",
								o_ctrl);
			} else {
				rc = devm_request_threaded_irq(dev,
								o_ctrl->vsync_irq,
								NULL,
								cam_ois_vsync_irq_thread,
								(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
								"ois-vsync-irq",
								o_ctrl);
			}

			if (rc != 0)
				CAM_ERR(CAM_OIS, "failed: to request ois-vsync IRQ %d, rc %d", o_ctrl->vsync_irq, rc);
			else
				CAM_DBG(CAM_OIS, "request ois-vsync IRQ success");
		} else
			CAM_ERR(CAM_OIS, "failed: to get ois-vsync IRQ");
	}

	CAM_DBG(CAM_OIS, "Component bound successfully");
	return rc;
unreg_subdev:
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));
free_soc:
	kfree(soc_private);
free_cci_client:
	kfree(o_ctrl->io_master_info.cci_client);
free_ois_data:
	kfree(o_ctrl->ois_data);
free_ring_buff:
	kfree(o_ctrl->ring_buff);
free_o_ctrl:
	kfree(o_ctrl);
	return rc;
}

static void cam_ois_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int                             i;
	struct cam_ois_ctrl_t          *o_ctrl;
	struct cam_ois_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info         *soc_info;
	struct platform_device *pdev = to_platform_device(dev);

	o_ctrl = platform_get_drvdata(pdev);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return;
	}

	CAM_INFO(CAM_OIS, "platform driver remove invoked");
	if (o_ctrl->ic_name != NULL && strstr(o_ctrl->ic_name, "aw86006"))
		aw86006_ois_exit(o_ctrl);

	soc_info = &o_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	kfree(o_ctrl->soc_info.soc_private);
	kfree(o_ctrl->io_master_info.cci_client);
	kfree(o_ctrl->ois_data);
	kfree(o_ctrl->ring_buff);
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&o_ctrl->v4l2_dev_str.sd, NULL);
	kfree(o_ctrl);
}

const static struct component_ops cam_ois_component_ops = {
	.bind = cam_ois_component_bind,
	.unbind = cam_ois_component_unbind,
};

static int32_t cam_ois_platform_driver_probe(
	struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_OIS, "Adding OIS Sensor component");
	rc = component_add(&pdev->dev, &cam_ois_component_ops);
	if (rc)
		CAM_ERR(CAM_OIS, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_ois_platform_driver_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_ois_component_ops);
	return 0;
}

static const struct of_device_id cam_ois_dt_match[] = {
	{ .compatible = "qcom,ois" },
	{ }
};


MODULE_DEVICE_TABLE(of, cam_ois_dt_match);

struct platform_driver cam_ois_platform_driver = {
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = cam_ois_dt_match,
	},
	.probe = cam_ois_platform_driver_probe,
	.remove = cam_ois_platform_driver_remove,
};
static const struct i2c_device_id cam_ois_i2c_id[] = {
	{ "msm_ois", (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver cam_ois_i2c_driver = {
	.id_table = cam_ois_i2c_id,
	.probe  = cam_ois_i2c_driver_probe,
	.remove = cam_ois_i2c_driver_remove,
	.driver = {
		.name = "msm_ois",
	},
};

static struct cam_ois_registered_driver_t registered_driver = {
	0, 0};

int cam_ois_driver_init(void)
{
	int rc = 0;

	rc = platform_driver_register(&cam_ois_platform_driver);
	if (rc) {
		CAM_ERR(CAM_OIS, "platform_driver_register failed rc = %d",
			rc);
		return rc;
	}

	registered_driver.platform_driver = 1;

	rc = i2c_add_driver(&cam_ois_i2c_driver);
	if (rc) {
		CAM_ERR(CAM_OIS, "i2c_add_driver failed rc = %d", rc);
		return rc;
	}

	registered_driver.i2c_driver = 1;
	return rc;
}

void cam_ois_driver_exit(void)
{
	if (registered_driver.platform_driver)
		platform_driver_unregister(&cam_ois_platform_driver);

	if (registered_driver.i2c_driver)
		i2c_del_driver(&cam_ois_i2c_driver);
}

MODULE_DESCRIPTION("CAM OIS driver");
MODULE_LICENSE("GPL v2");
