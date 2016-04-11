/* 
 * File:   ETM_MODBUS_SLAVE.h
 * Author: hwanetick
 *
 * Created on April 11, 2016, 10:23 AM
 */

#ifndef ETM_MODBUS_SLAVE_H
#define	ETM_MODBUS_SLAVE_H


#include <uart.h>
#include "ETM_BUFFER_BYTE_64.h"
/* 
   The following must be global defines included in the project configuration
   ETM_FCY_CLK - defined as the processor FCY
   ETM_MODBUS_UART - defined as the UART for the modbus module to use
*/


/*
  When a command is called, the modbus module clears the "done" status for that response
  When the command completes (normally or with an error) the modbus module sets the "done" status.

  The ETM Modbus module must be able to buffer a number of modbus requests.
  The internal buffer should be 16 modbus "commands" deep.
  They should be executed in the order they were received.
  If the Modbus commond buffer is full, the response should be marked as "done" with a unique exception code

  **** The user code is responsible for polling the done status and executing accordingly
  **** The user code is responsible for insuring that data in the response structure is not overwritten

*/
extern void ETMmodbus_init(void);

extern void ETMmodbus_task(void);

extern unsigned int ETMmodbus_timer_10ms;  // for timeout response

#define UART1_BAUDRATE             19200        // U1 Baud Rate

#define MODBUS_U1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_2STOPBITS)
#define MODBUS_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define MODBUS_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)

#define UART1TX_ON_TRIS		(TRISDbits.TRISD7)
#define UART1TX_ON_IO		(PORTDbits.RD7)


// PLC slave address
#define MODBUS_SLAVE_ADDR       0x07

// Modbus exception codes
#define ILLEGAL_FUNCTION           0x01
#define ILLEGAL_ADDRESS            0x02
#define ILLEGAL_VALUE              0x03
#define DEVICE_FAILURE             0x04


// Modbus functions
#define FUNCTION_READ_BITS              0x01
#define FUNCTION_READ_REGISTERS         0x03
#define FUNCTION_READ_INPUT_REGISTERS   0x04
#define FUNCTION_WRITE_BIT              0x05
#define FUNCTION_WRITE_REGISTER         0x06

#define EXCEPTION_FLAGGED               0x09
 
#define ETMMODBUS_CMD_QUEUE_SIZE   16

typedef struct {
  unsigned char function_code;
  unsigned char received_function_code;
  unsigned char data_length_bytes;
  unsigned char exception_code;
  unsigned char done;
  unsigned int  output_address;
  unsigned int  data_address;
  unsigned int  qty_bits;
  unsigned int  qty_reg;
  unsigned int  write_value;
  unsigned int  data[125];
  unsigned char bit_data[125];
} MODBUS_MESSAGE;

#define SLAVE_BIT_ARRAY_SIZE          64
#define SLAVE_HOLD_REG_ARRAY_SIZE     64
#define SLAVE_INPUT_REG_ARRAY_SIZE    64

unsigned int ModbusSlaveHoldingRegister[SLAVE_HOLD_REG_ARRAY_SIZE];
unsigned int ModbusSlaveInputRegister[SLAVE_INPUT_REG_ARRAY_SIZE];
unsigned char ModbusSlaveBit[SLAVE_BIT_ARRAY_SIZE];

#define modbus_slave_hold_reg_0x00  ModbusSlaveHoldingRegister[0]
#define modbus_slave_hold_reg_0x01  ModbusSlaveHoldingRegister[1]
#define modbus_slave_hold_reg_0x02  ModbusSlaveHoldingRegister[2]
#define modbus_slave_hold_reg_0x03  ModbusSlaveHoldingRegister[3]
#define modbus_slave_hold_reg_0x04  ModbusSlaveHoldingRegister[4]
#define modbus_slave_hold_reg_0x05  ModbusSlaveHoldingRegister[5]
#define modbus_slave_hold_reg_0x06  ModbusSlaveHoldingRegister[6]
#define modbus_slave_hold_reg_0x07  ModbusSlaveHoldingRegister[7]
#define modbus_slave_hold_reg_0x08  ModbusSlaveHoldingRegister[8]
#define modbus_slave_hold_reg_0x09  ModbusSlaveHoldingRegister[9]
#define modbus_slave_hold_reg_0x0A  ModbusSlaveHoldingRegister[10]
#define modbus_slave_hold_reg_0x0B  ModbusSlaveHoldingRegister[11]
#define modbus_slave_hold_reg_0x0C  ModbusSlaveHoldingRegister[12]
#define modbus_slave_hold_reg_0x0D  ModbusSlaveHoldingRegister[13]
#define modbus_slave_hold_reg_0x0E  ModbusSlaveHoldingRegister[14]
#define modbus_slave_hold_reg_0x0F  ModbusSlaveHoldingRegister[15]
#define modbus_slave_hold_reg_0x10  ModbusSlaveHoldingRegister[16]
#define modbus_slave_hold_reg_0x11  ModbusSlaveHoldingRegister[17]
#define modbus_slave_hold_reg_0x12  ModbusSlaveHoldingRegister[18]
#define modbus_slave_hold_reg_0x13  ModbusSlaveHoldingRegister[19]
#define modbus_slave_hold_reg_0x14  ModbusSlaveHoldingRegister[20]
#define modbus_slave_hold_reg_0x15  ModbusSlaveHoldingRegister[21]
#define modbus_slave_hold_reg_0x16  ModbusSlaveHoldingRegister[22]
#define modbus_slave_hold_reg_0x17  ModbusSlaveHoldingRegister[23]
#define modbus_slave_hold_reg_0x18  ModbusSlaveHoldingRegister[24]
#define modbus_slave_hold_reg_0x19  ModbusSlaveHoldingRegister[25]
#define modbus_slave_hold_reg_0x1A  ModbusSlaveHoldingRegister[26]
#define modbus_slave_hold_reg_0x1B  ModbusSlaveHoldingRegister[27]
#define modbus_slave_hold_reg_0x1C  ModbusSlaveHoldingRegister[28]
#define modbus_slave_hold_reg_0x1D  ModbusSlaveHoldingRegister[29]
#define modbus_slave_hold_reg_0x1E  ModbusSlaveHoldingRegister[30]
#define modbus_slave_hold_reg_0x1F  ModbusSlaveHoldingRegister[31]
#define modbus_slave_hold_reg_0x20  ModbusSlaveHoldingRegister[32]
#define modbus_slave_hold_reg_0x21  ModbusSlaveHoldingRegister[33]
#define modbus_slave_hold_reg_0x22  ModbusSlaveHoldingRegister[34]
#define modbus_slave_hold_reg_0x23  ModbusSlaveHoldingRegister[35]
#define modbus_slave_hold_reg_0x24  ModbusSlaveHoldingRegister[36]
#define modbus_slave_hold_reg_0x25  ModbusSlaveHoldingRegister[37]
#define modbus_slave_hold_reg_0x26  ModbusSlaveHoldingRegister[38]
#define modbus_slave_hold_reg_0x27  ModbusSlaveHoldingRegister[39]
#define modbus_slave_hold_reg_0x28  ModbusSlaveHoldingRegister[40]
#define modbus_slave_hold_reg_0x29  ModbusSlaveHoldingRegister[41]
#define modbus_slave_hold_reg_0x2A  ModbusSlaveHoldingRegister[42]
#define modbus_slave_hold_reg_0x2B  ModbusSlaveHoldingRegister[43]
#define modbus_slave_hold_reg_0x2C  ModbusSlaveHoldingRegister[44]
#define modbus_slave_hold_reg_0x2D  ModbusSlaveHoldingRegister[45]
#define modbus_slave_hold_reg_0x2E  ModbusSlaveHoldingRegister[46]
#define modbus_slave_hold_reg_0x2F  ModbusSlaveHoldingRegister[47]
#define modbus_slave_hold_reg_0x30  ModbusSlaveHoldingRegister[48]
#define modbus_slave_hold_reg_0x31  ModbusSlaveHoldingRegister[49]
#define modbus_slave_hold_reg_0x32  ModbusSlaveHoldingRegister[50]
#define modbus_slave_hold_reg_0x33  ModbusSlaveHoldingRegister[51]
#define modbus_slave_hold_reg_0x34  ModbusSlaveHoldingRegister[52]
#define modbus_slave_hold_reg_0x35  ModbusSlaveHoldingRegister[53]
#define modbus_slave_hold_reg_0x36  ModbusSlaveHoldingRegister[54]
#define modbus_slave_hold_reg_0x37  ModbusSlaveHoldingRegister[55]
#define modbus_slave_hold_reg_0x38  ModbusSlaveHoldingRegister[56]
#define modbus_slave_hold_reg_0x39  ModbusSlaveHoldingRegister[57]
#define modbus_slave_hold_reg_0x3A  ModbusSlaveHoldingRegister[58]
#define modbus_slave_hold_reg_0x3B  ModbusSlaveHoldingRegister[59]
#define modbus_slave_hold_reg_0x3C  ModbusSlaveHoldingRegister[60]
#define modbus_slave_hold_reg_0x3D  ModbusSlaveHoldingRegister[61]
#define modbus_slave_hold_reg_0x3E  ModbusSlaveHoldingRegister[62]
#define modbus_slave_hold_reg_0x3F  ModbusSlaveHoldingRegister[63]


#define modbus_slave_input_reg_0x00  ModbusSlaveInputRegister[0]
#define modbus_slave_input_reg_0x01  ModbusSlaveInputRegister[1]
#define modbus_slave_input_reg_0x02  ModbusSlaveInputRegister[2]
#define modbus_slave_input_reg_0x03  ModbusSlaveInputRegister[3]
#define modbus_slave_input_reg_0x04  ModbusSlaveInputRegister[4]
#define modbus_slave_input_reg_0x05  ModbusSlaveInputRegister[5]
#define modbus_slave_input_reg_0x06  ModbusSlaveInputRegister[6]
#define modbus_slave_input_reg_0x07  ModbusSlaveInputRegister[7]
#define modbus_slave_input_reg_0x08  ModbusSlaveInputRegister[8]
#define modbus_slave_input_reg_0x09  ModbusSlaveInputRegister[9]
#define modbus_slave_input_reg_0x0A  ModbusSlaveInputRegister[10]
#define modbus_slave_input_reg_0x0B  ModbusSlaveInputRegister[11]
#define modbus_slave_input_reg_0x0C  ModbusSlaveInputRegister[12]
#define modbus_slave_input_reg_0x0D  ModbusSlaveInputRegister[13]
#define modbus_slave_input_reg_0x0E  ModbusSlaveInputRegister[14]
#define modbus_slave_input_reg_0x0F  ModbusSlaveInputRegister[15]
#define modbus_slave_input_reg_0x10  ModbusSlaveInputRegister[16]
#define modbus_slave_input_reg_0x11  ModbusSlaveInputRegister[17]
#define modbus_slave_input_reg_0x12  ModbusSlaveInputRegister[18]
#define modbus_slave_input_reg_0x13  ModbusSlaveInputRegister[19]
#define modbus_slave_input_reg_0x14  ModbusSlaveInputRegister[20]
#define modbus_slave_input_reg_0x15  ModbusSlaveInputRegister[21]
#define modbus_slave_input_reg_0x16  ModbusSlaveInputRegister[22]
#define modbus_slave_input_reg_0x17  ModbusSlaveInputRegister[23]
#define modbus_slave_input_reg_0x18  ModbusSlaveInputRegister[24]
#define modbus_slave_input_reg_0x19  ModbusSlaveInputRegister[25]
#define modbus_slave_input_reg_0x1A  ModbusSlaveInputRegister[26]
#define modbus_slave_input_reg_0x1B  ModbusSlaveInputRegister[27]
#define modbus_slave_input_reg_0x1C  ModbusSlaveInputRegister[28]
#define modbus_slave_input_reg_0x1D  ModbusSlaveInputRegister[29]
#define modbus_slave_input_reg_0x1E  ModbusSlaveInputRegister[30]
#define modbus_slave_input_reg_0x1F  ModbusSlaveInputRegister[31]
#define modbus_slave_input_reg_0x20  ModbusSlaveInputRegister[32]
#define modbus_slave_input_reg_0x21  ModbusSlaveInputRegister[33]
#define modbus_slave_input_reg_0x22  ModbusSlaveInputRegister[34]
#define modbus_slave_input_reg_0x23  ModbusSlaveInputRegister[35]
#define modbus_slave_input_reg_0x24  ModbusSlaveInputRegister[36]
#define modbus_slave_input_reg_0x25  ModbusSlaveInputRegister[37]
#define modbus_slave_input_reg_0x26  ModbusSlaveInputRegister[38]
#define modbus_slave_input_reg_0x27  ModbusSlaveInputRegister[39]
#define modbus_slave_input_reg_0x28  ModbusSlaveInputRegister[40]
#define modbus_slave_input_reg_0x29  ModbusSlaveInputRegister[41]
#define modbus_slave_input_reg_0x2A  ModbusSlaveInputRegister[42]
#define modbus_slave_input_reg_0x2B  ModbusSlaveInputRegister[43]
#define modbus_slave_input_reg_0x2C  ModbusSlaveInputRegister[44]
#define modbus_slave_input_reg_0x2D  ModbusSlaveInputRegister[45]
#define modbus_slave_input_reg_0x2E  ModbusSlaveInputRegister[46]
#define modbus_slave_input_reg_0x2F  ModbusSlaveInputRegister[47]
#define modbus_slave_input_reg_0x30  ModbusSlaveInputRegister[48]
#define modbus_slave_input_reg_0x31  ModbusSlaveInputRegister[49]
#define modbus_slave_input_reg_0x32  ModbusSlaveInputRegister[50]
#define modbus_slave_input_reg_0x33  ModbusSlaveInputRegister[51]
#define modbus_slave_input_reg_0x34  ModbusSlaveInputRegister[52]
#define modbus_slave_input_reg_0x35  ModbusSlaveInputRegister[53]
#define modbus_slave_input_reg_0x36  ModbusSlaveInputRegister[54]
#define modbus_slave_input_reg_0x37  ModbusSlaveInputRegister[55]
#define modbus_slave_input_reg_0x38  ModbusSlaveInputRegister[56]
#define modbus_slave_input_reg_0x39  ModbusSlaveInputRegister[57]
#define modbus_slave_input_reg_0x3A  ModbusSlaveInputRegister[58]
#define modbus_slave_input_reg_0x3B  ModbusSlaveInputRegister[59]
#define modbus_slave_input_reg_0x3C  ModbusSlaveInputRegister[60]
#define modbus_slave_input_reg_0x3D  ModbusSlaveInputRegister[61]
#define modbus_slave_input_reg_0x3E  ModbusSlaveInputRegister[62]
#define modbus_slave_input_reg_0x3F  ModbusSlaveInputRegister[63]




typedef struct {
  unsigned char data[16];
  unsigned char write_location;
  unsigned char read_location;
} BUFFERnBYTE;

#define BuffernMask 0b00001111	/* must equal (data size - 1) */


void CheckForCommand(void) {
    
  if (BuffernBytesInBuffer(&uart1_input_buffer) >= ETMMODBUS_COMMAND_SIZE_MIN) {
      
  	if (uart1_input_buffer.data[0] != PLC_SLAVE_ADDR) {
      current_response_ptr->done = ETMMODBUS_ERROR_SLAVE_ADDR; 
    } else {
      
    }
      
      
      
      
      
      
      
      
      
  }    
}


  
void BuffernWriteByte(BUFFERnBYTE* ptr, unsigned char value);
/*
  Writes a byte to the buffer
  If the buffer is full the oldest byte will be overwritten
*/

unsigned char BuffernReadByte(BUFFERnBYTE* ptr);
/*
  Reads a single byte from the buffer.
  If the buffer is empty zero will be returned and the write/read location will not be changed
  Before calling BuffernReadByte the buffer should be checked with BuffernBytesInBuffer or BuffernIsNotEmpty
*/

unsigned char BuffernBytesInBuffer(BUFFERnBYTE* ptr);
/*
  Returns the number of bytes stored in the buffer
*/

unsigned char BuffernIsNotEmpty(BUFFERnBYTE* ptr);
/*
  Returns zero if the buffer is Empty
  Returns one if the buffer is not empty
*/

#define PERIOD_3_5 437496  // 3.5 character timer
#define PERIOD_1_5 187496  // 1.5 character timer

#define MSG_LEN 512

//#define TIMER_3_5_INT() CpuTimer1Regs.TCR.bit.TIF  //macros to clear up DSP functions
//#define CHAR_RDY()      (ScibRegs.SCIFFRX.bit.RXFFST > 0)
//#define READ_TIMER()    CpuTimer1Regs.TIM.all

#define NOERROR 0
#define PRTYERR 1
#define FRAMERR 2

#define EMISSION_REQUIRED 1

#define SLAVE_ADDRESS 0x07  //Slave address

#define CRC_POLY 0xA001				// Reverse CR16 polynomial.

// MODBUS functions

#define MODBUS_READ_COILS               1
#define MODBUS_WRITE_SINGLE_COIL        5

#define MODBUS_READ_INPUT_REGISTERS     4
#define MODBUS_READ_HOLDING_REGISTERS   3
#define MODBUS_WRITE_HOLDING_REGISTER   6



#define MSG_OK                0
#define ILLEGAL_FUNCTION      1
#define ILLEGAL_DATA_ADDRESS  2
#define ILLEGAL_DATA_VALUE    3
#define SLAVE_DEVICE_FAILURE  4
#define SLAVE_ACK             5
#define SLAVE_BUSY            6

// MODBUS exception codes
#define ILLEGAL_FUNCTION      0x01
#define ILLEGAL_DATA_ADDRESS  0x02
#define ILLEGAL_DATA_VALUE    0x03
#define SLAVE_DEVICE_FAILURE  0x04

// MODBUS write holding registers
#define WREG_HV_ENABLE        0x0102
#define WREG_BEAM_ENABLE      0x0103
#define WREG_RESET_FAULTS     0x0201


#define WREG_HTR_SET_VOLTAGE         0x0301
#define WREG_TOP_SET_VOLTAGE         0x0302
#define WREG_HV_SET_VOLTAGE          0x0303


// MODBUS read input registers
#define RREG_HTR_VOLATGE             0x0401
#define RREG_HTR_CURRENT             0x0402
#define RREG_TOP_VOLTAGE             0x0403
#define RREG_CATH_VOLTAGE            0x0404
#define RREG_GD_TEMPERATURE          0x0405
#define RREG_BIAS_VOLTAGE            0x0406  
#define RREG_PK_BEAM_CURRENT         0x0407
#define RREG_HTR_WARMUP_REMAINING    0x0408
#define RREG_HTR_V_REF               0x0501  
#define RREG_TOP_V_REF               0x0502
#define RREG_CATH_V_REF   	         0x0503
#define RREG_CONTROL_STATE           0x0601
#define RREG_FAULTS                  0x0602
#define RREG_WARNINGS                0x0603



#endif	/* ETM_MODBUS_SLAVE_H */

