/*
 * utility.h
 *
 *  Created on: Apr 10, 2018
 *      Author: muniu
 */

#ifndef ARDUCOPTER_UTILITY_H_
#define ARDUCOPTER_UTILITY_H_

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

class Utility
{
public:
	// radians
	static float my_roll;
	static float my_pitch;
	static float my_yaw;
	// Latitude * 10**7   Longitude * 10**7
	static int32_t my_latitude;
	static int32_t my_longitude;
	// Altitude in centimeters (meters * 100)
	static int32_t my_inv_alt;
	// baro alt
	static int32_t my_baro_alt;
	// sona alt
	static int32_t my_sona_alt;
	// lidar alt
	static int32_t my_lidar_alt;
	// vel cm/s
	static float my_vel_x;
	static float my_vel_y;
	static float my_vel_z;
	// avoid info
	static int32_t my_avoid_flag;
	static float my_current_velocity;
	static float my_desired_velocity;
	static int32_t my_avoid_count;
	static float my_prx_dis;
	// beixing
	static int32_t my_beixing;

	static int my_fd;
	static char* my_fn;

	static void init_my_log()
	{
		char path[64];
		for (int i = 1; ;i++)
		{
			sprintf(path, "%s/%s_%d", HAL_BOARD_LOG_DIRECTORY, my_fn, i);
			if (access(path, 0) == -1)
			{
				break;
			}
		}
		my_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, S_IRUSR | S_IWUSR);
	}

	static ssize_t write_my_log_str(const char *fmt, ...)
	{
		if (my_fd == -1)
		{
			init_my_log();
		}
		if (my_fd != -1)
		{
			va_list ap;
			va_start(ap, fmt);
			char buff[512];
			int count = vsprintf(buff, fmt, ap);
			va_end(ap);
			return write(my_fd, buff, count);
		}
		return 0;
	}

	static ssize_t write_my_log_byte(const void *data, int len)
	{
		if (my_fd == -1)
		{
			init_my_log();
		}
		if (my_fd != -1)
		{
			return write(my_fd, data, len);
		}
		return 0;
	}

};


#endif /* ARDUCOPTER_UTILITY_H_ */
