#ifndef __MOT_ACTUATOR_H__
#define __MOT_ACTUATOR_H__

#include <linux/ioctl.h>
#include <linux/videodev2.h>
#define MOT_ACTUATOR_NAME "mot_actuator"

#define MOT_ACTUATOR_READ \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 50, int)
#define MOT_ACTUATOR_READ32 \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 50, int)

#define MOT_ACTUATOR_WRITE \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 51, int)
#define MOT_ACTUATOR_WRITE32 \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 51, int)

#define MOT_ACTUATOR_INIT \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 52, int)
#define MOT_ACTUATOR_INIT32 \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 52, int)

#define MOT_ACTUATOR_RELEASE \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 53, int)
#define MOT_ACTUATOR_RELEASE32 \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 53, int)

#define MOT_ACTUATOR_EXIT \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 54, int)
#define MOT_ACTUATOR_EXIT32 \
	_IOWR('X', BASE_VIDIOC_PRIVATE + 54, int)

int mot_actuator_driver_init(void);
void mot_actuator_driver_exit(void);
#endif /*__MOT_ACTUATOR_H__*/
