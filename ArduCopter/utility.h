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

	static int my_fd;

	static void init_my_log()
	{
		char path[32];
		for (int i = 1; ;i++)
		{
			sprintf(path, "%s%s%d", HAL_BOARD_LOG_DIRECTORY, "/mylog", i);
			if (access(path, 0) == -1)
			{
				break;
			}
		}
		my_fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, S_IRUSR | S_IWUSR);
	}

	static ssize_t write_my_log(const char *fmt, ...)
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
};


#endif /* ARDUCOPTER_UTILITY_H_ */
