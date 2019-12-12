/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_user_functions.c 
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Target-Specific Functions Required by the MIP SDK
//
// External dependencies:
//
//  mip.h
// 
//!@copyright 2014 Lord Microstrain Sensing Systems. 
//
//!@section CHANGES
//! 
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR 
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION 
//! WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//Include Files
//
////////////////////////////////////////////////////////////////////////////////


#include "mip_sdk_user_functions.h"

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_open(void *port_handle, int port_num, int baudrate)
//
//! @section DESCRIPTION
//! Target-Specific communications port open. 
//
//! @section DETAILS
//!
//! @param [out] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in] int port_num       - port number (as recognized by the operating system.)
//! @param [in] int baudrate       - baudrate of the com port.
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem opening the port.\n
//! @retval MIP_USER_FUNCTION_OK     The open was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_open(void **port_handle, int port_num, int baud_rate)
{
    char port_name[100] = {0};
    char port_index[10] = {0};
    static int serial_fd = {-1};
    struct termios uart_config;
    int termios_state;

    strcat(port_name, "/dev/ttyS");
    //construct port filename address string
    sprintf(&port_index[0], "%u", port_num);
    strcat(port_name, port_index);
    // open serial port
    serial_fd = open(port_name, O_RDWR | O_NOCTTY);
    
    if (serial_fd < 0) {
        return MIP_USER_FUNCTION_ERROR;
    }
    tcgetattr(serial_fd, &uart_config);
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    uart_config.c_oflag = 0;
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

    if ((termios_state = cfsetispeed(&uart_config, baud_rate)) < 0) {
        // PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
        return MIP_USER_FUNCTION_ERROR;
    }

    if ((termios_state = cfsetospeed(&uart_config, baud_rate)) < 0) {
        // PX4_INFO("ERR: %d (cfsetospeed)", termios_state);
        return MIP_USER_FUNCTION_ERROR;
    }

    if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
        // PX4_INFO("ERR: %d (tcsetattr)", termios_state);
        return MIP_USER_FUNCTION_ERROR;
    }

    // assign external pointer to port handle pointer
    *port_handle = &serial_fd;
    
    return MIP_USER_FUNCTION_OK; 
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_close(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific port close function 
//
//! @section DETAILS
//!
//! @param [in] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem closing the port.\n
//! @retval MIP_USER_FUNCTION_OK     The close was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_close(void *port_handle)
{
    int serial_fd = *reinterpret_cast<int*>(port_handle);
    close(serial_fd);
    return MIP_USER_FUNCTION_OK;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function. 
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_written - the number of bytes actually written to the port
//! @param [in]  u32 timeout_ms     - the write timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The write was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
{
    int serial_fd = *reinterpret_cast<int*>(port_handle);
    u32 written = write(serial_fd, buffer, num_bytes);
    fsync(serial_fd);
    *bytes_written = written;
    bool success = written == num_bytes;
    if (success)
        return MIP_USER_FUNCTION_OK;
    return MIP_USER_FUNCTION_ERROR;
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function. 
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_read    - the number of bytes actually read from the device
//! @param [in]  u32 timeout_ms     - the read timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The read was successful.\n
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
{
    int serial_fd = *reinterpret_cast<int*>(port_handle);
    u32 ret = read(serial_fd, buffer, num_bytes);
    *bytes_read = ret;
    if (ret != num_bytes)
        return MIP_USER_FUNCTION_ERROR;
    return MIP_USER_FUNCTION_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_port_read_count(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific Function to Get the Number of Bytes Waiting on the Port.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Number of bytes waiting on the port,\n
//!           0, if there is an error.
//
//! @section NOTES
//! 
//! The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//! edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_port_read_count(void *port_handle)
{
    int serial_fd = *reinterpret_cast<int*>(port_handle);
    int bytes_available = 0;
    ioctl(serial_fd, FIONREAD, (unsigned long) &bytes_available);
    uint8_t buf[bytes_available];
    u32 bytes_read = 0;
    return mip_sdk_port_read(port_handle, buf, bytes_available, &bytes_read, 0);
}


/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_get_time_ms()
//
//! @section DESCRIPTION
//! Target-Specific Call to get the current time in milliseconds.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Current time in milliseconds.
//
//! @section NOTES
//! 
//!   1) This value should no roll-over in short periods of time (e.g. minutes)\n
//!   2) Most systems have a millisecond counter that rolls-over every 32 bits\n
//!      (e.g. 49.71 days roll-over period, with 1 millisecond LSB)\n
//!   3) An absolute reference is not required since this function is\n
//!      used for relative time-outs.\n
//!   4) The user should copy the \c mip_sdk_user_functions.c file to their project directory and\n
//!      edit it as needed to support their target operating system.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_get_time_ms()
{
 //User must replace this code
 return 0; 
}