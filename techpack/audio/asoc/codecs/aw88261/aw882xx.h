#ifndef _AW882XX_H_
#define _AW882XX_H_
#include <linux/version.h>
#include <linux/kernel.h>

#ifndef AW_SPIN_ENABLE
#define AW_SPIN_ENABLE	1
#endif

#include "aw_device.h"

/*
 * i2c transaction on Linux limited to 64k
 * (See Linux kernel documentation: Documentation/i2c/writing-clients)
*/
#define AW882XX_CHIP_ID_REG (0x00)
#define MAX_I2C_BUFFER_SIZE					65536
#define AW882XX_I2C_READ_MSG_NUM		2
#define AW882XX_DC_DELAY_TIME	(1000)
#define AW882XX_LOAD_FW_DELAY_TIME	(0)
#define AW_START_RETRIES	(5)
#define AW882XX_RUNIN_TEST
#define AW_ALGO_SKT_BIN		"aw_skt.bin"
#define AW_DEFAULT_PRO_ID	(0xFFFFFFFF)
#define CONFIG_AW882XX_ALGO_BIN_PARAMS

#define AW_I2C_RETRIES			5	/* 5 times */
#define AW_I2C_RETRY_DELAY		5	/* 5 ms */


#define AW882XX_RATES SNDRV_PCM_RATE_8000_48000
#define AW882XX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
						SNDRV_PCM_FMTBIT_S24_LE | \
						SNDRV_PCM_FMTBIT_S32_LE)

enum {
	AW882XX_STREAM_CLOSE = 0,
	AW882XX_STREAM_OPEN,
};

enum aw882xx_chipid {
	PID_1852_ID = 0x1852,
	PID_2013_ID = 0x2013,
	PID_2032_ID = 0x2032,
	PID_2055_ID = 0x2055,
	PID_2071_ID = 0x2071,
	PID_2113_ID = 0x2113,
};

#define AW882XX_SOFT_RESET_VALUE	(0x55aa)

enum aw882xx_int_type {
	INT_TYPE_NONE = 0,
	INT_TYPE_UVLO = 0x1,
	INT_TYPE_BSTOC = 0x2,
	INT_TYPE_OCDI = 0x4,
	INT_TYPE_OTHI = 0x8,
};

#ifdef CONFIG_AW882XX_ALGO_BIN_PARAMS
enum {
	AW_ALGO_PROFILE_ID_0 = 0,
	AW_ALGO_PROFILE_ID_1,
	AW_ALGO_PROFILE_ID_2,
	AW_ALGO_PROFILE_ID_3,
	AW_ALGO_PROFILE_ID_4,
	AW_ALGO_PROFILE_ID_5,
	AW_ALGO_PROFILE_ID_6,
	AW_ALGO_PROFILE_ID_MAX,
};

struct aw882xx_scene_info {
	int id;
	char name[20];
	int priority;
	int skt_profile_id;
	int is_active;
	int active_cnt;
};

enum {
	AW_PARAMS_PATH_MODE = 0,
	AW_PARAMS_DATA_MODE,
};


#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 1)
#define AW_KERNEL_VER_OVER_4_19_1
#endif


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
#define AW_KERNEL_VER_OVER_5_4_0
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif


#ifdef AW_KERNEL_VER_OVER_4_19_1
typedef struct snd_soc_component aw_snd_soc_codec_t;
typedef struct snd_soc_component_driver aw_snd_soc_codec_driver_t;
#else
typedef struct snd_soc_codec aw_snd_soc_codec_t;
typedef struct snd_soc_codec_driver aw_snd_soc_codec_driver_t;
#endif

struct aw_componet_codec_ops {
	aw_snd_soc_codec_t *(*kcontrol_codec)(struct snd_kcontrol *kcontrol);
	void *(*codec_get_drvdata)(aw_snd_soc_codec_t *codec);
	int (*add_codec_controls)(aw_snd_soc_codec_t *codec,
		const struct snd_kcontrol_new *controls, unsigned int num_controls);
	void (*unregister_codec)(struct device *dev);
	int (*register_codec)(struct device *dev,
			const aw_snd_soc_codec_driver_t *codec_drv,
			struct snd_soc_dai_driver *dai_drv,
			int num_dai);
};

enum {
	AWRW_I2C_ST_NONE = 0,
	AWRW_I2C_ST_READ,
	AWRW_I2C_ST_WRITE,
};

#define AWRW_ADDR_BYTES (1)
#define AWRW_DATA_BYTES (2)
#define AWRW_HDR_LEN (24)

enum {
	AWRW_FLAG_WRITE = 0,
	AWRW_FLAG_READ,
};

enum {
	AW_BSTCFG_DISABLE = 0,
	AW_BSTCFG_ENABLE,
};

enum {
	AW_FRCSET_DISABLE = 0,
	AW_FRCSET_ENABLE,
};

enum {
	AW_BOP_DISABLE = 0,
	AW_BOP_ENABLE,
};

enum {
	AW882XX_RAMP_OFF = 0,
	AW882XX_RAMP_ON,
};

#ifdef CONFIG_AW882XX_ALGO_BIN_PARAMS
enum {
	AW_ALGO_CALI_OFF = 0,
	AW_ALGO_CALI_ON,
};

enum {
	AW_ALGO_BYPASS_OFF = 0,
	AW_ALGO_BYPASS_ON,
};

enum {
	AW_ALGO_HANDSET_OFF = 0,
	AW_ALGO_HANDSET_ON,
};

enum {
	AW_ALGO_VOICE_OFF = 0,
	AW_ALGO_VOICE_ON,
};

enum {
	AW_ALGO_VOIP_OFF = 0,
	AW_ALGO_VOIP_ON,
};

enum {
	AW_ALGO_MUSIC_DEEPBUFFER_OFF = 0,
	AW_ALGO_MUSIC_DEEPBUFFER_ON,
};

enum {
	AW_ALGO_MUSIC_FASTTRACK_OFF = 0,
	AW_ALGO_MUSIC_FASTTRACK_ON,
};
#endif


enum {
	AWRW_HDR_WR_FLAG = 0,
	AWRW_HDR_ADDR_BYTES,
	AWRW_HDR_DATA_BYTES,
	AWRW_HDR_REG_NUM,
	AWRW_HDR_REG_ADDR,
	AWRW_HDR_MAX,
};

struct aw882xx_i2c_packet{
	char status;
	unsigned int reg_num;
	unsigned int reg_addr;
	char *reg_data;
};

/********************************************
 * struct aw882xx
 *******************************************/
struct aw882xx {
	int sysclk;
	int rate;
	int pstream;
	int cstream;
	int aw882xx_ramp_status;	/* ramp status */
#ifdef CONFIG_AW882XX_ALGO_BIN_PARAMS
	int aw882xx_algo_cali;
	int aw882xx_algo_bypass;
	int aw882xx_algo_handset;
	int aw882xx_algo_voice;
	int aw882xx_algo_voip;
	int aw882xx_algo_music_deepbuffer;
	int aw882xx_algo_music_fasttrack;
	int aw882xx_algo_prof_id;
	int cur_algo_prof_id;
#endif
	unsigned int skt_prof_mode;

	unsigned int fade_flag;
	unsigned char index;
	unsigned char phase_sync;	/* phase sync */
	unsigned char dc_flag;
	unsigned char dbg_en_prof;	/* debug enable/disable profile function */
	unsigned char allow_pw;		/* allow power */
	int reset_gpio;
	int irq_gpio;
	unsigned int chip_id;
	unsigned char fw_status;
	unsigned char fw_retry_cnt;
	unsigned char rw_reg_addr;	/* rw attr node store read addr */

	aw_snd_soc_codec_t *codec;
	struct aw_componet_codec_ops *codec_ops;

	struct i2c_client *i2c;
	struct device *dev;
	struct aw882xx_i2c_packet i2c_packet;
	struct aw_device *aw_pa;

	struct workqueue_struct *work_queue;
	struct delayed_work start_work;
	struct delayed_work interrupt_work;
	struct delayed_work dc_work;
	struct delayed_work fw_work;
#ifdef AW882XX_RUNIN_TEST
	struct delayed_work adsp_status;
#endif
	struct mutex lock;
};

void aw882xx_kcontorl_set(struct aw882xx *aw882xx);
int aw882xx_get_version(char *buf, int size);
int aw882xx_get_dev_num(void);
int aw882xx_i2c_write(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int reg_data);
int aw882xx_i2c_read(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int *reg_data);
int aw882xx_i2c_write_bits(struct aw882xx *aw882xx,
	unsigned char reg_addr, unsigned int mask, unsigned int reg_data);
int aw882xx_init(struct aw882xx *aw882xx, int index);
int aw882xx_hw_reset(struct aw882xx *aw882xx);


#endif

