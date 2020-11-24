/*
I2C master features:
    - Clock generation
    - Start and Stop generation
I2C slave features:
    - Programmable I2C Address detection
    = Stop bit detection
Supports different communication speeds
    - Standard up to 100khz
    - Advanced up to 400khz
Status flags
    - Transmitter/ receiver mode flag
    - End-of-byte transmission flag
    - I2C busy flag
Error flags
    - Arbitration lost condition for master mode
    - Acknowledgement failure after address/ data transmission
    - Detection of misplaced start or stop condition
    - Overrun/ underrun if clock stretching is disable
3 types of interrups:
    - 1 communication interrupt
    - 1 error condition interrupt
    - 1 wakeup from Halt interrupt
Wake up capability
    - MCU wakes up from Low power mode on address detection in slave mode.
*/
/* DS1307 overview
    The DS1307 can be run in either 12-hour or 24-hour mode. Bit 6 of the hours register is defined as the  12-
    or 24-hour mode select bit. When high, the 12-hour mode is selected
    In the 12-hour mode, bit 5 is the AM/PM bit with logic high being PM
    In the 24-hour mode, bit 5 is the second 10 hour bit (20 - 23 hours)
    The DS1307 operates in the regular mode (100kHz) only
*/
/* Status 
    - Bus not busy: Both data and clock lines remain HIGH.
    - Start data transfer: A change in the state of the data line, from LOW to HIGH, while the clock is HIGH, define the STOP conditon.
    - Stop data transfer: A change in the state of the data line, from LOW to HIGH, while the clock line is HIGH, define the STOP condition.
    - Data valid: After a START condition. The data line is stable for the duration of the HIGH period of the clock signal.
    The data on the line must be changed during the LOW period of the clock signal.There is one clock pulse per bit of data
*/