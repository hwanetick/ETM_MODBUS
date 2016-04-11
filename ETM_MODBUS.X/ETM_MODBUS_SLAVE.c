#include <ETM_MODBUS_SLAVE.h>


#if defined(__dsPIC30F__)
#include <p30fxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24Hxxxx.h>
#endif


//#if defined(__dsPIC33F__) || defined(__PIC24H__) || defined(__dsPIC33E__) || defined (__PIC24E__) || defined(__dsPIC30F1010__) || defined(__dsPIC30F2020__) || defined(__dsPIC30F2023__)
//#define __MODBUS_V2
//
//#elif defined(__dsPIC30F__)
//#define __MODBUS_V1
//
//#endif

/*
 Functions needed:
 * parse message
 * do modbus / modbus task
 * init modbus
 */

#include <p30F6014a.h>
#include "A36507.h"
#include "Buffern.h"

#define FCY_CLK  20000000


#define BuffernMask 0b00001111	/* must equal (data size - 1) */

unsigned int ETMmodbus_timer_10ms;

static unsigned char  ETMmodbus_put_index;
static unsigned char  ETMmodbus_get_index;

static MODBUS_RESP_SMALL*  ETMmodbus_resp_ptr[ETMMODBUS_CMD_QUEUE_SIZE];

BUFFERBYTE64 uart1_input_buffer;   
BUFFERBYTE64 uart1_output_buffer; 

static unsigned char normal_reply_length;
static MODBUS_MESSAGE * current_command_ptr;

void ETMmodbus_init(void);
void ETMModbusSlaveDoModbus(void);
void ReceiveCommand(void);
void SendResponse(MODBUS_MESSAGE * ptr);
void ProcessCommand (MODBUS_MESSAGE * ptr);
unsigned int checkCRC(unsigned char * ptr, unsigned int size);


void SendResponse(MODBUS_MESSAGE * ptr) 
{
  unsigned int crc;
  unsigned int data_length_words;

  unsigned int index;
  
  // clear input/output buffer first
  uart1_input_buffer.write_location = 0;  
  uart1_input_buffer.read_location = 0;
  uart1_output_buffer.write_location = 0;
  uart1_output_buffer.read_location = 0;
  
  switch (ptr->function_code)
  {
  case FUNCTION_READ_BITS:
	BufferByte64WriteByte(&uart1_output_buffer, MODBUS_SLAVE_ADDR);
	BufferByte64WriteByte(&uart1_output_buffer, ptr->function_code);
    BufferByte64WriteByte(&uart1_output_buffer, ptr->data_length_bytes);	// number of bytes to follow
    data_length_words = ptr->data_length_bytes;
    index = 0;
    while (data_length_words) {
      BufferByte64WriteByte(&uart1_output_buffer, ptr->bit_data[index] & 0xff);
      index++;
      data_length_words--;
    }
    break;
      
  case FUNCTION_READ_REGISTERS: 
  case FUNCTION_READ_INPUT_REGISTERS:
	BufferByte64WriteByte(&uart1_output_buffer, MODBUS_SLAVE_ADDR);
	BufferByte64WriteByte(&uart1_output_buffer, ptr->function_code);    
    data_length_words = ptr->qty_reg;
    ptr->data_length_bytes = (data_length_words * 2) & 0xff;
	BufferByte64WriteByte(&uart1_output_buffer, ptr->data_length_bytes);	// number of bytes to follow
    index = 0;
    while (data_length_words) {
      BufferByte64WriteByte(&uart1_output_buffer, (ptr->data[index] >> 8) & 0xff);	// data Hi
      BufferByte64WriteByte(&uart1_output_buffer, ptr->data[index] & 0xff);	// data Lo
      index++;
      data_length_words--;
    }
         
    crc = checkCRC(uart1_output_buffer.data, 3 + ptr->data_length_bytes);
	BufferByte64WriteByte(&uart1_output_buffer, crc & 0xff);
	BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8) & 0xff);
      
    break;
      
  case FUNCTION_WRITE_BIT:
  case FUNCTION_WRITE_REGISTER:
	BufferByte64WriteByte(&uart1_output_buffer, MODBUS_SLAVE_ADDR);
	BufferByte64WriteByte(&uart1_output_buffer, ptr->function_code);    
	BufferByte64WriteByte(&uart1_output_buffer, (ptr->data_address >> 8) & 0xff);	// addr Hi
	BufferByte64WriteByte(&uart1_output_buffer, ptr->data_address & 0xff);	// addr Lo
	BufferByte64WriteByte(&uart1_output_buffer, (ptr->write_value >> 8) & 0xff);	// data Hi
	BufferByte64WriteByte(&uart1_output_buffer, ptr->write_value & 0xff);	// data Lo
        
    crc = checkCRC(uart1_output_buffer.data, 6);
	BufferByte64WriteByte(&uart1_output_buffer, crc & 0xff);
	BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8) & 0xff);
      
    break;
      
  case EXCEPTION_FLAGGED:
	BufferByte64WriteByte(&uart1_output_buffer, MODBUS_SLAVE_ADDR);
	BufferByte64WriteByte(&uart1_output_buffer, ptr->received_function_code); 
	BufferByte64WriteByte(&uart1_output_buffer, ptr->exception_code);
    
    crc = checkCRC(uart1_output_buffer.data, 3);
	BufferByte64WriteByte(&uart1_output_buffer, crc & 0xff);
	BufferByte64WriteByte(&uart1_output_buffer, (crc >> 8) & 0xff);
      
    break;
      
  default:
  	break;
  }
  
  while (U1STAbits.UTXBF);	// wait for output buffer empty, shouldn't be long
  
  // setup timer and flag
  ptr->done = 0;
  ETMmodbus_timer_10ms = 0;
  
  
  if ((!U1STAbits.UTXBF) && (BufferByte64IsNotEmpty(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the outputbuffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
      All subsequent bytes will be moved from the output buffer to the transmit buffer by the U1 TX Interrupt
    */
    U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
  }
}

/****************************************************************************
  Function:
    ETMmodbus_init()

  Input:
    
  Description:
  Remarks:
    None
***************************************************************************/
void ETMmodbus_init(void)
{
	  // Initialize application specific hardware
	  UART1TX_ON_TRIS = 0;
	  UART1TX_ON_IO = 1;    // always enable TX1


	    // Configure UART Interrupts
	  _U1RXIE = 0;
	  _U1RXIP = 3;
	  
	  _U1TXIE = 0;
	  _U1TXIP = 3;

	  // ----------------- UART #1 Setup and Data Buffer -------------------------//
	  // Setup the UART input and output buffers
	  uart1_input_buffer.write_location = 0;  
	  uart1_input_buffer.read_location = 0;
	  uart1_output_buffer.write_location = 0;
	  uart1_output_buffer.read_location = 0;
            
      ETMmodbus_put_index = 0;
	  ETMmodbus_get_index = 0;
	  
      U1MODE = MODBUS_U1MODE_VALUE;
	  U1BRG = MODBUS_U1BRG_VALUE;
	  U1STA = MODBUS_U1STA_VALUE;
	  
	  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
	  _U1TXIE = 1;	// Enable Transmit Interrupts
	  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
	  _U1RXIE = 1;	// Enable Recieve Interrupts
	  
	  U1MODEbits.UARTEN = 1;	// And turn the peripheral on
      
      PIN_ENABLE_RS485 = 1;

}



void ProcessCommand (MODBUS_MESSAGE * ptr) {
  unsigned int coil_index;
  unsigned char bit_index;
  unsigned int byte_index;
  unsigned char byte_count;
  unsigned char last_bits;
  unsigned char data_index;
  unsigned int data_length_words;
  
  switch (ptr->function_code) {
      
    case FUNCTION_READ_BITS:

      if ((ptr->data_address + ptr->qty_bits) > SLAVE_BIT_ARRAY_SIZE) {
        ptr->received_function_code = ptr->function_code;
        ptr->function_code = EXCEPTION_FLAGGED;
        ptr->exception_code = ILLEGAL_ADDRESS;
        break;  
      }

      if (ptr->qty_bits <= 8) {
        ptr->data_length_bytes = 1;
      } else if (ptr->qty_bits & 0x0007) {
        ptr->data_length_bytes = ((ptr->qty_bits /8) + 1) & 0xff;
      } else {
        ptr->data_length_bytes = (ptr->qty_bits /8) & 0xff;
      }
        
      byte_count = ptr->qty_bits / 8;
      last_bits = ptr->qty_bits & 0x07;
      
      if (ptr->qty_bits == 1) {
        if (ModbusSlaveBit[ptr->data_address]) {
          ptr->bit_data[0] = 0x01;
        } else {
          ptr->bit_data[0] = 0x00;
        }
      } else {
        byte_index = 0;
        while (byte_index < byte_count) { 
          coil_index = ptr->data_address;
          bit_index = 0;
          ptr->bit_data[byte_index] = 0;
          while (bit_index < 8) {
            if (ModbusSlaveBit[coil_index]) {
              ptr->bit_data[byte_index] |= (0x01 << bit_index);
            }
            bit_index++;
            coil_index++
          } 
          byte_index++;
        }
        if (last_bits) {
          bit_index = 0;
          ptr->bit_data[byte_index] = 0;
          while (bit_index < 8) {
            if (bit_index < last_bits) {
              if (ModbusSlaveBit[coil_index] != 0) {
                ptr->bit_data[byte_index] |= (0x01 << bit_index);
                coil_index++;
              }  
            }
            bit_index++;            
          }      
          
        }
      }
      break;
      
    case FUNCTION_READ_REGISTERS:         
        
      if ((ptr->data_address + ptr->qty_reg) >= SLAVE_HOLD_REG_ARRAY_SIZE) {
        ptr->received_function_code = ptr->function_code;
        ptr->function_code = EXCEPTION_FLAGGED;
        ptr->exception_code = ILLEGAL_ADDRESS;
        break;  
      }
      data_length_words = ptr->qty_reg;
      byte_index = 0;
      data_index = 0;
      while (data_length_words) {
        ptr->data[data_index] =  ModbusSlaveHoldingRegister[ptr->data_address + byte_index];
        byte_index++;
        data_index++;
        data_length_words--;
      } 
      break;
      
    case FUNCTION_READ_INPUT_REGISTERS:
        
      if ((ptr->data_address + ptr->qty_reg) >= SLAVE_INPUT_REG_ARRAY_SIZE) {
        ptr->received_function_code = ptr->function_code;
        ptr->function_code = EXCEPTION_FLAGGED;
        ptr->exception_code = ILLEGAL_ADDRESS;
        break;  
      }
      data_length_words = ptr->qty_reg;
      byte_index = 0;
      data_index = 0;
      while (data_length_words) {
        ptr->data[data_index] =  ModbusSlaveInputRegister[ptr->data_address + byte_index];
        byte_index++;
        data_index++;
        data_length_words--;
      }
      break;
      
    case FUNCTION_WRITE_BIT:
      if (ptr->data_address >= SLAVE_BIT_ARRAY_SIZE) {
        ptr->received_function_code = ptr->function_code;
        ptr->function_code = EXCEPTION_FLAGGED;
        ptr->exception_code = ILLEGAL_ADDRESS;
        break;  
      }
      coil_index = ptr->data_address;
      if ((ptr->write_value == 0x0000) || (ptr->write_value == 0xFF00)) {
        ModbusSlaveBit[coil_index] = ptr->write_value;
      } else {
        ptr->received_function_code = ptr->function_code;
        ptr->function_code = EXCEPTION_FLAGGED;
        ptr->exception_code = ILLEGAL_VALUE;
      }     
      break;
      
    case FUNCTION_WRITE_REGISTER:
      if (ptr->data_address >= SLAVE_HOLD_REG_ARRAY_SIZE) {
        ptr->received_function_code = ptr->function_code;
        ptr->function_code = EXCEPTION_FLAGGED;
        ptr->exception_code = ILLEGAL_ADDRESS;
        break;  
      }
      byte_index = ptr->data_address;
      ModbusSlaveHoldingRegister[byte_index] = ptr->write_value;
      break;
      
    default:
  	  break;
  }
}    

void CheckValidData(MODBUS_MESSAGE * ptr) {
  if ((modbus_slave_invalid_data != 0) && (ptr->function_code == FUNCTION_WRITE_REGISTER)) {     
      ptr->received_function_code = ptr->function_code;
      ptr->function_code = EXCEPTION_FLAGGED;
      ptr->exception_code = ILLEGAL_VALUE;
  }     
}
    
if (current_command_ptr->done == ETMMODBUS_COMMAND_OK) {   
  ProcessCommand ();
} else {
  SendException ();
}


//use timer 1 for timers 1.5 and 3.5 char 
//
//timers 4 and 5 for CAN and timers 2 and 3 for GunDriver software

receiving_flag = 0;    
//void ETMModbusSlaveDoModbus(void) {
//  switch (ETM_modbus_state) {
//      
//    case STATE_IDLE: 
//      if (transmission needed) {
//        ETM_modbus_state = STATE_TRANSMITTING;
//        if (!U1STAbits.UTXBF) {
//          U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
//          transmittingflag = 0;
//        }
//      } else if (receiving_flag) {
//        receiving_flag = 0;
//        char_1_5_timer++;
//        ETM_modbus_state = STATE_RECEIVING;
//      }
//      break;
//  
//    case STATE_RECEIVING:
//      char_1_5_timer++;  
//      if (char_1_5_timer > 1_5_CHAR_TIME) {
//        if (BufferByte64BytesInBuffer(&uart1_output_buffer) == COMMAND_SIZE) {
//          ETM_modbus_state = STATE_PROCESSING;
//        } else {
//          incomplete frame error;
//          clear buffer;
//          ETM_modbus_state = STATE_IDLE;        
//        }
//      }
//      break;
//    
//    case STATE_PROCESSING:
//      if (3.5ch timer > 3_5_CHAR_TIME) {
//        
//      }
//      ETM_modbus_state = STATE_IDLE;
//    
//    case STATE_TRANSMITTING:
//      if (transmittingflag) {
//        1.5 ch timer start;
//        3.5 ch timer start;
//        transmittingflag = 0;
//        if (BufferByte64BytesInBuffer(&uart1_output_buffer)) {
//          if (1.5ch timer not expired) {
//            U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
//            1.5 ch timer start;
//          } else {              
//            transmission error = 0;
//            ETM_modbus_state = STATE_IDLE;
//          }  
//        } else {
//          transmission_needed = 0;
//          ETM_modbus_state = STATE_IDLE;
//        }
//      } else {
//        1.5 ch timer++;
//        3.5 ch timer++ ;
//      }
//      break;   
//        
//  }
//}  
    
    
void ETMModbusSlaveDoModbus(void) {
  switch (ETM_modbus_state) {
      
    case STATE_IDLE: 
      if (transmission needed) {
        ETM_modbus_state = STATE_TRANSMITTING;
        if (!U1STAbits.UTXBF) {
          U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
          transmittingflag = 0;
        }
      } else if (receiving_flag) {
        receiving_flag = 0;
        ETM_modbus_state = STATE_RECEIVING;
      }
      break;
  
    case STATE_RECEIVING:  
      if (BufferByte64BytesInBuffer(&uart1_output_buffer) == COMMAND_SIZE) {
        ETM_modbus_state = STATE_PROCESSING;
      } else {
        incomplete frame error;
        clear buffer;
        ETM_modbus_state = STATE_IDLE;        
      }   
      break;
    
    case STATE_PROCESSING:
      if (3.5ch timer > 3_5_CHAR_TIME) {
      }
      ETM_modbus_state = STATE_IDLE;
    
    case STATE_TRANSMITTING:
      if (transmittingflag) {
        3.5 ch timer start;
        transmittingflag = 0;
        if (BufferByte64BytesInBuffer(&uart1_output_buffer)) {
          U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
        } else {
          transmission_needed = 0;
          ETM_modbus_state = STATE_IDLE;
        }
      } else {
        3.5 ch timer++;
      }
      break;         
  }
}    
    
    
    
    
  switch (ETMmodbus_state) {
    case STATE_IDLE:
      if (ETM_queue_is_empty() == 0) {
        current_command_ptr = GetNextResponsePointer();             
        if (current_command_ptr != 0) {	// send command out
          ReceiveCommand(current_command_ptr);
          ETMmodbus_state = STATE_WAIT_RESPONSE;
        }
      }
      break;
    case STATE_WAIT_RESPONSE:
      LookForResponse();
      if (current_command_ptr->done) {
        uart1_input_buffer.write_location = 0;  
        uart1_input_buffer.read_location = 0;
        ETMmodbus_state = STATE_IDLE;        
      } else if (ETMmodbus_timer_10ms > ETMMODBUS_TIMEOUT_SET) {
        current_command_ptr->done = ETMMODBUS_ERROR_TIMEOUT;
        ETMmodbus_state = STATE_IDLE;
      }
      break;
    default:
      break;  
  }
}

//this is the function for parsing and processing 
void ReceiveCommand(void) {
  unsigned char reply_length;
  unsigned int crc, crc_in;
  
  if (BufferByte64BytesInBuffer(&uart1_input_buffer) >= ETMMODBUS_COMMAND_SIZE_MIN) {   
    crc_in = (uart1_input_buffer.data[7] << 8) + uart1_input_buffer.data[6];
    crc = checkCRC(uart1_input_buffer.data, 6); 
    if (crc_in != crc) {
      current_command_ptr->done = ETMMODBUS_ERROR_CRC;    
      return;
    } else {
      if (uart1_input_buffer.data[0] != MODBUS_SLAVE_ADDR) {
        current_command_ptr->done = ETMMODBUS_ERROR_SLAVE_ADDR; 
        return;
      } else { 
      	if (uart1_input_buffer.data[1] & 0x80) {
          current_command_ptr->done = ETMMODBUS_ERROR_FUNCTION;
          ptr->received_function_code = ptr->function_code;
          ptr->function_code = EXCEPTION_FLAGGED;
          ptr->exception_code = ILLEGAL_FUNCTION;
        } else {
          start_200_ms_response_timer;
          current_command_ptr->done = ETMMODBUS_COMMAND_OK;
          current_command_ptr->function_code = uart1_input_buffer.data[1] & 0x7F;
          current_command_ptr->data_address = (uart1_input_buffer.data[2] << 8) + uart1_input_buffer.data[3];
          switch (current_command_ptr->function_code) {
            case FUNCTION_READ_BITS:
              current_command_ptr->qty_bits = (uart1_input_buffer.data[4] << 8) + uart1_input_buffer.data[5];
              break;
            
            case FUNCTION_READ_REGISTERS:  
            case FUNCTION_READ_INPUT_REGISTERS:  
              current_command_ptr->qty_reg = (uart1_input_buffer.data[4] << 8) + uart1_input_buffer.data[5];                
              break;
    
            case FUNCTION_WRITE_BIT:
            case FUNCTION_WRITE_REGISTER:
              current_command_ptr->write_value = (uart1_input_buffer.data[4] << 8) + uart1_input_buffer.data[5];
              break;
    
            default:
              current_command_ptr->done = ETMMODBUS_ERROR_FUNCTION;
              ptr->received_function_code = ptr->function_code;
              ptr->function_code = EXCEPTION_FLAGGED;
              ptr->exception_code = ILLEGAL_FUNCTION;
              break;
          }
                      
        }

      }
    }
  }  
}

//-----------------------------------------------------------------------------
// CRC_Check
//-----------------------------------------------------------------------------
//
// Return Value : accum -- the end result of the CRC value.
// Parameter    : *ptr -- pointer to the array for CRC calculation.
//				: size -- size of the array
//
// This function calculates the 16-bit CRC value for the input array.
//
//-----------------------------------------------------------------------------
#define CRC_POLY  0xA001
unsigned int checkCRC(unsigned char * ptr, unsigned int size)
{
    unsigned int i, j;
    unsigned int accum, element;

    accum = 0xffff;

    for (j = 0; j < size; j++)
    {
        element = ptr[j];

        for (i = 8; i > 0; i--)		
        {
            if (((element ^ accum) & 0x0001) > 0)
                accum = (unsigned int)((accum >> 1) ^ ((unsigned int)CRC_POLY));
            else
                accum >>= 1;

            element >>= 1;
        }
    }

    return (accum);
}






int16 parsemsg()
{
	unsigned int CRC;
	unsigned int* address;
	unsigned int quantity;
		
	outbufmbptr = outbufmb;
	exceptionCode = MSG_OK;
	
	//first grab the CRC from the input stream.  CRC is 16-bit and use ptr arithmetic
	CRC = (*(inbufmbptr-1) << 8) + *(inbufmbptr - 2);
	
	//Check CRC of buffer, passing input buffer and length
	if (checkCRC(inbufmb, (Uint16)(inbufmbptr - inbufmb - 2)) == CRC)
		//check for proper slave address
		if (inbufmb[0] == SLAVE_ADDRESS) {
			//check the Modbus functions
			//Only a few supported for now
			switch (inbufmb[1]) {
				case MODBUS_READ_INPUT_REGISTERS:
					//read the address
					address = (Uint16*)((inbufmb[2] << 8) + inbufmb[3]);
					//read the quantity of registers
					quantity = ((inbufmb[4] << 8) + inbufmb[5]);
					//form the output stream
					*outbufmbptr++ = inbufmb[0];
					*outbufmbptr++ = inbufmb[1];					
					*outbufmbptr++ = quantity*2;
					exceptionCode = readInputs(address, quantity);
                    if (exceptionCode) break;  // to return error response
					//generate a CRC for the output
					CRC = checkCRC(outbufmb, outbufmbptr-outbufmb);
					*outbufmbptr++ = (CRC & 0x00FF);
					*outbufmbptr++ = ((CRC & 0xFF00) >> 8);
					//transmit on serial bus B
					scib_tx(outbufmb, outbufmbptr-outbufmb);
				break;
					
				case MODBUS_WRITE_HOLDING_REGISTER:
					address = (Uint16*)((inbufmb[2] << 8) + inbufmb[3]);
			        quantity = 1; // ((inbufmb[4] << 8) + inbufmb[5]);
					
					exceptionCode = writeInputs(address, quantity, &inbufmb[4]);
                    if (exceptionCode) break;  // to return error response

					*outbufmbptr++ = inbufmb[0];
					*outbufmbptr++ = inbufmb[1];
					*outbufmbptr++ = inbufmb[2];
					*outbufmbptr++ = inbufmb[3];
					*outbufmbptr++ = inbufmb[4];
					*outbufmbptr++ = inbufmb[5];
					CRC = checkCRC(outbufmb, outbufmbptr-outbufmb);
					*outbufmbptr++ = (CRC & 0x00FF);
					*outbufmbptr++ = ((CRC & 0xFF00) >> 8);
					scib_tx(outbufmb, outbufmbptr-outbufmb);
				break;
                
                case MODBUS_READ_HOLDING_REGISTERS:
//					address = (Uint16*)((inbufmb[2] << 8) + inbufmb[3]);
//			        quantity = 1; // ((inbufmb[4] << 8) + inbufmb[5]);
//					
//					exceptionCode = writeInputs(address, quantity, &inbufmb[4]);
//                    if (exceptionCode) break;  // to return error response
//
//					*outbufmbptr++ = inbufmb[0];
//					*outbufmbptr++ = inbufmb[1];
//					*outbufmbptr++ = inbufmb[2];
//					*outbufmbptr++ = inbufmb[3];
//					*outbufmbptr++ = inbufmb[4];
//					*outbufmbptr++ = inbufmb[5];
//					CRC = checkCRC(outbufmb, outbufmbptr-outbufmb);
//					*outbufmbptr++ = (CRC & 0x00FF);
//					*outbufmbptr++ = ((CRC & 0xFF00) >> 8);
//					scib_tx(outbufmb, outbufmbptr-outbufmb);
				break;
                
                case MODBUS_READ_COILS:
//					address = (Uint16*)((inbufmb[2] << 8) + inbufmb[3]);
//			        quantity = 1; // ((inbufmb[4] << 8) + inbufmb[5]);
//					
//					exceptionCode = writeInputs(address, quantity, &inbufmb[4]);
//                    if (exceptionCode) break;  // to return error response
//
//					*outbufmbptr++ = inbufmb[0];
//					*outbufmbptr++ = inbufmb[1];
//					*outbufmbptr++ = inbufmb[2];
//					*outbufmbptr++ = inbufmb[3];
//					*outbufmbptr++ = inbufmb[4];
//					*outbufmbptr++ = inbufmb[5];
//					CRC = checkCRC(outbufmb, outbufmbptr-outbufmb);
//					*outbufmbptr++ = (CRC & 0x00FF);
//					*outbufmbptr++ = ((CRC & 0xFF00) >> 8);
//					scib_tx(outbufmb, outbufmbptr-outbufmb);
				break;
                
                case MODBUS_WRITE_SINGLE_COIL:
//					address = (Uint16*)((inbufmb[2] << 8) + inbufmb[3]);
//			        quantity = 1; // ((inbufmb[4] << 8) + inbufmb[5]);
//					
//					exceptionCode = writeInputs(address, quantity, &inbufmb[4]);
//                    if (exceptionCode) break;  // to return error response
//
//					*outbufmbptr++ = inbufmb[0];
//					*outbufmbptr++ = inbufmb[1];
//					*outbufmbptr++ = inbufmb[2];
//					*outbufmbptr++ = inbufmb[3];
//					*outbufmbptr++ = inbufmb[4];
//					*outbufmbptr++ = inbufmb[5];
//					CRC = checkCRC(outbufmb, outbufmbptr-outbufmb);
//					*outbufmbptr++ = (CRC & 0x00FF);
//					*outbufmbptr++ = ((CRC & 0xFF00) >> 8);
//					scib_tx(outbufmb, outbufmbptr-outbufmb);
				break;
				
				default:
					exceptionCode = ILLEGAL_FUNCTION;
				break;
				
			}
            if (exceptionCode) 
            { // return error response
				outbufmbptr = outbufmb;
				*outbufmbptr++ = inbufmb[0];
				*outbufmbptr++ = inbufmb[1] | 0x80;
				*outbufmbptr++ = exceptionCode;
				CRC = checkCRC(outbufmb, outbufmbptr-outbufmb);
				*outbufmbptr++ = (CRC & 0x00FF);
				*outbufmbptr++ = ((CRC & 0xFF00) >> 8);
				scib_tx(outbufmb, outbufmbptr-outbufmb);
               
            }
			return exceptionCode;
		}
	else
		return exceptionCode;  //CRC Error
	
	
	return exceptionCode;
}
ADDRESS_FOR_HV_MON
ADDRESS_FOR_HV_MOD_SET
ADDRESS_FOR_HV_ENABLE


//-----------------------------------------------------------------------------
//   UART Interrupts
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
        
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
  _U1RXIF = 0;
  receivingingflag = 1;
//  global_char_ready = 1;
  char_1_5_timer = 0;
//  start 3.5ch timer
//  while (U1STAbits.URXDA) {
  BufferByte64WriteByte(&uart1_input_buffer, U1RXREG);
//  }
}



void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
  _U1TXIF = 0;
  transmittingflag = 1;
  while ((!U1STAbits.UTXBF) && (BufferByte64BytesInBuffer(&uart1_output_buffer))) {
      
    /*
      There is at least one byte available for writing in the output buffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
    */
    U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
}
}

