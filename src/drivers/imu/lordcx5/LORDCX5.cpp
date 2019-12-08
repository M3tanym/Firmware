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
 * @file LORDCX5.cpp
 */

#include "LORDCX5.h"

mip_interface device_interface;

LORDCX5::LORDCX5(int device_port, int baud) : ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(0)),
    baud_rate(baud),
    port_num(device_port) {
    
    ///
    //Initialize the interface to the device
    ///
    
    if (mip_interface_init(device_port, baud_rate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
        PX4_ERR("Failed to initialize LORD IMU!");
    
     u8  enable = 1;
 
    /// 
    //Enable the FILTER datastream
    ///

    mip_3dm_cmd_continuous_data_stream(&device_interface,       
    MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable);

    ///
    //Enable the AHRS datastream
    ///
    
    mip_3dm_cmd_continuous_data_stream(&device_interface, 
    MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable);
    
    ///
    //Enable the GPS datastream
    ///
    
    mip_3dm_cmd_continuous_data_stream(&device_interface,  
    MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_GPS_DATASTREAM, &enable);
}

void LORDCX5::updateMIPInterface() {
   mip_interface_update(&device_interface); 
}

//void LORDCX5::testRead() {
//    int err = 0;
//    int bytes_available = 0;
//    PX4_INFO("Reading...");
//    err = ioctl(serial_fd, FIONREAD, (unsigned long) &bytes_available);
//    PX4_INFO("Bytes available: %d, error: %d", bytes_available, err);
//    if (bytes_available > 0) {
//        uint8_t buf[bytes_available];
//        int ret = read(serial_fd, buf, sizeof(buf));
//
//        PX4_INFO("Read %d bytes: |%s|", ret, buf);
//    }
//}
//
//bool LORDCX5::testWrite(uint8_t *data, size_t len) {
//    PX4_INFO("Writing...");
//    size_t written = write(serial_fd, data, len);
//    fsync(serial_fd);
//
//    bool success = written == len;
//    PX4_INFO("Wrote %d bytes. Success: %d", written, success);
//    return success;
//}

void LORDCX5::start()
{
	// Make sure we are stopped first.
	stop();

	// Start polling at the specified interval
	ScheduleOnInterval((1_s / _sample_rate), 10000);
}

void LORDCX5::stop()
{
	ScheduleClear();
}

void LORDCX5::measure() {
    updateMIPInterface();
     // TODO set read values
}

void LORDCX5::Run()
{
	// Make another measurement.
	measure();
}
