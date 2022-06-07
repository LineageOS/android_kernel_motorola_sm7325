/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */
#ifndef _CAM_OIS_DEV_H_
#define _CAM_OIS_DEV_H_

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_sensor.h>
#include <cam_sensor_i2c.h>
#include <cam_sensor_spi.h>
#include <cam_sensor_io.h>
#include <cam_cci_dev.h>
#include <cam_req_mgr_util.h>
#include <cam_req_mgr_interface.h>
#include <cam_mem_mgr.h>
#include <cam_subdev.h>
#include "cam_soc_util.h"
#include "cam_context.h"

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define MODE_ADDR 0x7014
#define ENABLE_ADDR 0x7015
#define PACKET_ADDR 0x70B0
#define DATA_READY_ADDR 0x70DA
#define DATA_READY 0x0001
#define READ_COUNT 3
#define PACKET_BYTE 62
#define MAX_PACKET 5
#define MAX_SAMPLE 30
#define READ_BYTE 0xC

#define QTIMER_ADDR 0x70DB
#define QTIMER_SAMPLE_TIME 2
#define QTIMER_MAX_SAMPLE 20

/* AW86006 EIS */
#define RING_BUFFER_LEN 42
#define AW86006_PACKET_ENABLE 0x0003
#define AW86006_PACKET_ADDR 0x0006
#define AW86006_MAX_SAMPLE 10


enum cam_ois_state {
	CAM_OIS_INIT,
	CAM_OIS_ACQUIRE,
	CAM_OIS_CONFIG,
	CAM_OIS_START,
};

/**
 * struct cam_ois_registered_driver_t - registered driver info
 * @platform_driver      :   flag indicates if platform driver is registered
 * @i2c_driver           :   flag indicates if i2c driver is registered
 *
 */
struct cam_ois_registered_driver_t {
	bool platform_driver;
	bool i2c_driver;
};

/**
 * struct cam_ois_i2c_info_t - I2C info
 * @slave_addr      :   slave address
 * @i2c_freq_mode   :   i2c frequency mode
 *
 */
struct cam_ois_i2c_info_t {
	uint16_t slave_addr;
	uint8_t i2c_freq_mode;
};

/**
 * struct cam_ois_soc_private - ois soc private data structure
 * @ois_name        :   ois name
 * @i2c_info        :   i2c info structure
 * @power_info      :   ois power info
 *
 */
struct cam_ois_soc_private {
	const char *ois_name;
	struct cam_ois_i2c_info_t i2c_info;
	struct cam_sensor_power_ctrl_t power_info;
};

/**
 * struct cam_ois_intf_params - bridge interface params
 * @device_hdl   : Device Handle
 * @session_hdl  : Session Handle
 * @ops          : KMD operations
 * @crm_cb       : Callback API pointers
 */
struct cam_ois_intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

struct awrw_ctrl {
	uint32_t addr[4];
	uint16_t reg_num;
	uint8_t flag;
	uint8_t *reg_data;
};

/**
 * struct cam_ois_ctrl_t - OIS ctrl private data
 * @device_name     :   ois device_name
 * @pdev            :   platform device
 * @ois_mutex       :   ois mutex
 * @soc_info        :   ois soc related info
 * @io_master_info  :   Information about the communication master
 * @cci_i2c_master  :   I2C structure
 * @v4l2_dev_str    :   V4L2 device structure
 * @bridge_intf     :   bridge interface params
 * @i2c_init_data   :   ois i2c init settings
 * @i2c_mode_data   :   ois i2c mode settings
 * @i2c_time_data   :   ois i2c time write settings
 * @i2c_preprog_data    :   ois i2c preprog settings
 * @i2c_precoeff_data   :   ois i2c precoeff settings
 * @i2c_postcalib_data  :   ois i2c postcalib settings
 * @i2c_calib_data  :   ois i2c calib settings
 * @ois_device_type :   ois device type
 * @cam_ois_state   :   ois_device_state
 * @ois_fw_flag     :   flag for firmware download
 * @ois_preprog_flag    :   flag for preprog reg settings
 * @ois_precoeff_flag   :   flag for precoeff reg settings
 * @is_ois_calib    :   flag for Calibration data
 * @ois_postcalib_flag  :   flag for postcalib reg settings
 * @opcode          :   ois opcode
 * @ois_fw_inc_addr     :   flag to increment address when sending fw
 * @ois_fw_addr_type    :   address type of fw
 * @ois_fw_txn_data_sz  :   num data bytes per i2c txn when sending fw
 * @device_name     :   Device name
 *
 */
struct cam_ois_ctrl_t {
	char device_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct platform_device *pdev;
	struct mutex ois_mutex;
	struct cam_hw_soc_info soc_info;
	struct camera_io_master io_master_info;
	enum cci_i2c_master_t cci_i2c_master;
	enum cci_device_num cci_num;
	struct cam_subdev v4l2_dev_str;
	struct cam_ois_intf_params bridge_intf;
	struct i2c_settings_array i2c_init_data;
	struct i2c_settings_array i2c_preprog_data;
	struct i2c_settings_array i2c_precoeff_data;
	struct i2c_settings_array i2c_calib_data;
	struct i2c_settings_array i2c_postcalib_data;
	struct i2c_settings_array i2c_mode_data;
	struct i2c_settings_array i2c_gyro_data;
	struct i2c_settings_array i2c_time_data;
	enum msm_camera_device_type_t ois_device_type;
	enum cam_ois_state cam_ois_state;
	char ois_name[32];
	uint8_t ois_fw_flag;
	uint8_t ois_preprog_flag;
	uint8_t ois_precoeff_flag;
	uint8_t is_ois_calib;
	uint8_t ois_postcalib_flag;
	uint8_t ois_fw_txn_data_sz;
	uint8_t ois_fw_inc_addr;
	uint8_t ois_fw_addr_type;
	uint8_t ois_fw_data_type;
	uint64_t prev_timestamp;
	uint64_t curr_timestamp;
	struct cam_ois_opcode opcode;
	bool is_ois_vsync_irq_supported;
	int vsync_irq;
	struct mutex vsync_mutex;
	struct completion ois_data_complete;
	bool is_first_vsync;
	uint8_t *ois_data;
	int ois_data_size;
	uint16_t q_timer_cnt;
	uint64_t mono_timestamp;
	/* awinic_add */
	const char *ic_name;
	struct work_struct aw_fw_update_work;
	struct mutex aw_ois_mutex;
	struct awrw_ctrl *awrw_ctrl;
	uint8_t *ring_buff;
	int ring_buff_size;
	bool is_video_mode;
	bool is_need_eis_data;
};

/**
 * @brief : API to register OIS hw to platform framework.
 * @return struct platform_device pointer on on success, or ERR_PTR() on error.
 */
int cam_ois_driver_init(void);

/**
 * @brief : API to remove OIS Hw from platform framework.
 */
void cam_ois_driver_exit(void);
#endif /*_CAM_OIS_DEV_H_ */
