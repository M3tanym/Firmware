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
#include <windows.h>


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
    strcat(port_name, "/dev/ttyS");
    //construct port filename address string
    sprintf(&port_index[0], "%u", port_num);
    strcat(port_name, port_index);
    // open serial port
    serial_fd = open(port_name, O_RDWR | O_NOCTTY);
    
    if (serial_fd < 0) {
        return MIP_USER_FUNCTION_ERROR;
    }
    
    // configure serial port settings    
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
        return MIP_USER_FUNCTION_ERROR;
    }
    if ((termios_state = cfsetospeed(&uart_config, baud_rate)) < 0) {
        return MIP_USER_FUNCTION_ERROR;
    }
    if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
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
 //User must replace this code
 return MIP_USER_FUNCTION_ERROR; 
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
 //User must replace this code
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
 //User must replace this code
 return MIP_USER_FUNCTION_ERROR;
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
 //User must replace this code
 return 0;
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
