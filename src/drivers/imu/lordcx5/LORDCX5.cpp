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

LORDCX5::LORDCX5(const char *device, int baud) :
    baud_rate(baud) {
    dev = new char[20];
    strncpy(dev, device, 19);
    dev[19] = 0;
}

LORDCX5::~LORDCX5() {
    delete[] dev;
}

int LORDCX5::configSerial() {
    PX4_INFO("Opening serial port...");
    struct termios uart_config;
    int termios_state;

    serial_fd = open(dev, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        PX4_ERR("Failed to open serial port: %s", dev);
        return -1;
    }

    PX4_INFO("Configuring serial port settings...");
    tcgetattr(serial_fd, &uart_config);

    // properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios)
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    uart_config.c_oflag = 0;
    // No line processing
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    // no parity, one stop bit, disable flow control
    uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    // Set speed
    if ((termios_state = cfsetispeed(&uart_config, baud_rate)) < 0) {
        PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
        return -1;
    }
    if ((termios_state = cfsetospeed(&uart_config, baud_rate)) < 0) {
        PX4_INFO("ERR: %d (cfsetospeed)", termios_state);
        return -1;
    }
    if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)", termios_state);
        return -1;
    }
    PX4_INFO("Opened serial port %s at speed %d", dev, baud_rate);
    return 0;
}

void LORDCX5::testRead() {
    int err = 0;
    int bytes_available = 0;
    PX4_INFO("Reading...");
    err = ioctl(serial_fd, FIONREAD, (unsigned long) &bytes_available);
    PX4_INFO("Bytes available: %d, error: %d", bytes_available, err);
    if (bytes_available > 0) {
        uint8_t buf[bytes_available];
        int ret = read(serial_fd, buf, sizeof(buf));
        PX4_INFO("Read %d bytes: |%s|", ret, buf);
    }
}

bool LORDCX5::testWrite(uint8_t *data, size_t len) {
    PX4_INFO("Writing...");
    size_t written = write(serial_fd, data, len);
    fsync(serial_fd);

    bool success = written == len;
    PX4_INFO("Wrote %d bytes. Success: %d", written, success);
    return success;
}
