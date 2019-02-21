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

	static const char* my_fd_name;

	static int my_fd;
//	static int my_avoid_fd;
//	static int my_st_fd;
//	static int my_beixing_fd;
//	static int my_pro_fd;

//	static void init_my_log()
//	{
//		char path[32];
//		for (int i = 1; ;i++)
//		{
//			sprintf(path, "%s%s%d", HAL_BOARD_LOG_DIRECTORY, "/mylog", i);
//			if (access(path, 0) == -1)
//			{
//				break;
//			}
//		}
//		my_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, S_IRUSR | S_IWUSR);
//	}

//	static ssize_t write_my_log(const char *fmt, ...)
//	{
//		if (my_fd == -1)
//		{
//			init_my_log();
//		}
//		if (my_fd != -1)
//		{
//			va_list ap;
//			va_start(ap, fmt);
//			char buff[128];
//			int count = vsprintf(buff, fmt, ap);
//			va_end(ap);
//			return write(my_fd, buff, count);
//		}
//		return 0;
//	}

	static void init_my_log()
	{
		char path[32];
		for (int i = 1; ;i++)
		{
			sprintf(path, "%s%s%d", HAL_BOARD_LOG_DIRECTORY, my_fd_name, i);
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
			char buff[128];
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

//	static void init_my_st_log()
//	{
//		char path[32];
//		for (int i = 1; ;i++)
//		{
//			sprintf(path, "%s%s%d", HAL_BOARD_LOG_DIRECTORY, "/st_adc_", i);
//			if (access(path, 0) == -1)
//			{
//				break;
//			}
//		}
//		my_st_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, S_IRUSR | S_IWUSR);
//	}

//	static ssize_t write_my_st_log(const void *data, int len)
//	{
//		if (my_st_fd == -1)
//		{
//			init_my_st_log();
//		}
//		if (my_st_fd != -1)
//		{
//			return write(my_st_fd, data, len);
//		}
//		return 0;
//	}

//	static void init_my_avoid_log()
//	{
//		char path[32];
//		for (int i = 1; ;i++)
//		{
//			sprintf(path, "%s%s%d", HAL_BOARD_LOG_DIRECTORY, "/avoid_", i);
//			if (access(path, 0) == -1)
//			{
//				break;
//			}
//		}
//		my_avoid_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, S_IRUSR | S_IWUSR);
//	}

//	static ssize_t write_my_avoid_log(const char *fmt, ...)
//	{
//		if (my_avoid_fd == -1)
//		{
//			init_my_avoid_log();
//		}
//		if (my_avoid_fd != -1)
//		{
//			va_list ap;
//			va_start(ap, fmt);
//			char buff[128];
//			int count = vsprintf(buff, fmt, ap);
//			va_end(ap);
//			return write(my_avoid_fd, buff, count);
//		}
//		return 0;
//	}

//	static void init_my_beixing_log()
//	{
//		char path[32];
//		for (int i = 1; ;i++)
//		{
//			sprintf(path, "%s%s%d", HAL_BOARD_LOG_DIRECTORY, "/beixing_", i);
//			if (access(path, 0) == -1)
//			{
//				break;
//			}
//		}
//		my_beixing_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, S_IRUSR | S_IWUSR);
//	}

//	static ssize_t write_my_beixing_log(const char *fmt, ...)
//	{
//		if (my_beixing_fd == -1)
//		{
//			init_my_beixing_log();
//		}
//		if (my_beixing_fd != -1)
//		{
//			va_list ap;
//			va_start(ap, fmt);
//			char buff[128];
//			int count = vsprintf(buff, fmt, ap);
//			va_end(ap);
//			return write(my_beixing_fd, buff, count);
//		}
//		return 0;
//	}

//	static void init_my_pro_log()
//	{
//		char path[32];
//		for (int i = 1; ;i++)
//		{
//			sprintf(path, "%s%s%d", HAL_BOARD_LOG_DIRECTORY, "/pro_", i);
//			if (access(path, 0) == -1)
//			{
//				break;
//			}
//		}
//		my_pro_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, S_IRUSR | S_IWUSR);
//	}

//	static ssize_t write_my_pro_log(const char *fmt, ...)
//	{
//		if (my_pro_fd == -1)
//		{
//			init_my_pro_log();
//		}
//		if (my_pro_fd != -1)
//		{
//			va_list ap;
//			va_start(ap, fmt);
//			char buff[128];
//			int count = vsprintf(buff, fmt, ap);
//			va_end(ap);
//			return write(my_pro_fd, buff, count);
//		}
//		return 0;
//	}
};


#endif /* ARDUCOPTER_UTILITY_H_ */
