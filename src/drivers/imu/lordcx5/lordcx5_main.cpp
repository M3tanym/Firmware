/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file lordcx5_main.cpp
 */

#include "LORDCX5.h"

/**
 * Driver 'main' command.
 */
extern "C" int lordcx5_main(int argc, char *argv[])
{
    PX4_INFO("LORD starting");
    
    int device_port = 3;
	int baudrate_main = 115200;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *arg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &arg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(arg, baudrate_main) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;

		case 'd':
			device_port = atoi(arg);
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return 1;
	}
	LORDCX5 l(device_port, baudrate_main);
    // uint8_t ping[8] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6};
    // l.testWrite(ping, 8);
/*
	GPS *gps;
	if (instance == Instance::Main) {
		gps = new GPS(device_name, mode, interface, fake_gps, enable_sat_info, instance, baudrate_main);

		if (gps && device_name_secondary) {
			task_spawn(argc, argv, Instance::Secondary);
			// wait until running
			int i = 0;

			do {
				// wait up to 1s
				px4_usleep(2500);

			} while (!_secondary_instance && ++i < 400);

			if (i == 400) {
				PX4_ERR("Timed out while waiting for thread to start");
			}
		}
	} else { // secondary instance
		gps = new GPS(device_name_secondary, mode, interface, fake_gps, enable_sat_info, instance, baudrate_secondary);
	}
*/
    return PX4_OK;
}
