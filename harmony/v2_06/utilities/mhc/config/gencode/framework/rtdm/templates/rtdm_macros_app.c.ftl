<#-- rtdm_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_rtdm_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_rtdm_app_c_global_data>

//*+++++++++++++++++++++++++++++++ RTDM Variables ++++++++++++++++++++++++++++++++++++++++*/
/* Received data is stored in array RTDMRxBuffer  */
unsigned char RTDMRxBuffer[RTDM_RXBUFFERSIZE];
unsigned char * RTDMRxBufferLoLimit = RTDMRxBuffer;
unsigned char * RTDMRxBufferHiLimit = RTDMRxBuffer + RTDM_RXBUFFERSIZE - 1;
unsigned char * RTDMRxBufferIndex = RTDMRxBuffer;
unsigned char * RTDMRxBufferStartMsgPointer;
unsigned char * RTDMRxBufferEndMsgPointer;
/* Data to be transmitted using UART communication module */
const unsigned char RTDMTxdata[] = {'R','T','D','M','\0'};
const unsigned char RTDMSanityCheckOK[] = {'+','$','R','T','D','M','#',0x1B,0x86,'\0'}; 
const unsigned char RTDMWriteMemoryOK[] = {'+','$','O','K','#',0x4C,0x08,'\0'};
const unsigned char RTDMErrorIllegalFunction[] = {'-','$','E',0x01,'#',0xD3,0x6A,'\0'};
unsigned char RTDMErrorFrame[] = {'-','$','E',0,'#',0,0,'\0'};

/* Temp variables used to calculate the CRC16*/
unsigned int RTDMcrcTemp,RTDMcrcTempH,RTDMcrcTempL;

    DMCIFLAGS DMCIFlags;
	RTDMFLAGS	RTDMFlags;
	// Buffer to store the data samples for the DMCI data viewer 
	// Graph1, Graph2, Graph3 and Graph4
    int RecorderBuffer1[DATA_BUFFER_SIZE] __attribute__ ((aligned));
    int RecorderBuffer2[DATA_BUFFER_SIZE] __attribute__ ((aligned));
    int RecorderBuffer3[DATA_BUFFER_SIZE] __attribute__ ((aligned));
    int RecorderBuffer4[DATA_BUFFER_SIZE] __attribute__ ((aligned));
    
    int * PtrRecBuffer1 = &RecorderBuffer1[0];	//Tail pointer for the DMCI Graph1
    int * PtrRecBuffer2 = &RecorderBuffer2[0];	//Tail pointer for the DMCI Graph2
    int * PtrRecBuffer3 = &RecorderBuffer3[0];	//Tail pointer for the DMCI Graph3
    int * PtrRecBuffer4 = &RecorderBuffer4[0];	//Tail pointer for the DMCI Graph4
    int * RecBuffUpperLimit = RecorderBuffer4 + DATA_BUFFER_SIZE -1;	//Buffer Recorder Upper Limit
    int	SnapCount = 0;
    int SnapShotDelayCnt = 0;
    signed short int SnapShotDelay = SNAPDELAY;
	int SpeedReference = 32767;


DRV_HANDLE RTDM_UART_HANDLE;

</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_rtdm_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_rtdm_app_c_local_functions>


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: ${APP_NAME?upper_case}_RecBufferUpdate                      */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: This function records the debug variables into                */
/* recorder buffer															  */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void ${APP_NAME?upper_case}_RecBufferUpdate(void)
{
  
               


    	if(DMCIFlags.Recorder)
    	{
    		if(SnapShotDelayCnt++ >= SnapShotDelay)
    		{
    			SnapShotDelayCnt = 0;

                *PtrRecBuffer1++ 	= DEBUG_VARIABLE1;
        		*PtrRecBuffer2++	= DEBUG_VARIABLE2;
    			*PtrRecBuffer3++	= DEBUG_VARIABLE3;
    			*PtrRecBuffer4++	= DEBUG_VARIABLE4;
    			
    			if(PtrRecBuffer4 > RecBuffUpperLimit)
    			{
    				
                    PtrRecBuffer1 = RecorderBuffer1;
    				PtrRecBuffer2 = RecorderBuffer2;
    		        PtrRecBuffer3 = RecorderBuffer3;
    		        PtrRecBuffer4 = RecorderBuffer4;
    		        DMCIFlags.Recorder = 0;
    		    }   
    		}
    	}


}

/******************************************************************************
* Function:    	 ${APP_NAME?upper_case}_RTDM_Start()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code initializes the UART to be used to
*			exchange data with the host PC
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify which UART module is going to be used by RTDM	
*******************************************************************************/
int ${APP_NAME?upper_case}_RTDM_Start()
{
    RTDM_UART_HANDLE = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
/************* RTDM Flags Configuration & Initial Values *****************/
	RTDMFlags.MessageReceived = 0;
	RTDMFlags.MessageReceived = 0;
	RTDMRxBufferIndex = RTDMRxBufferLoLimit;
	RTDMRxBufferStartMsgPointer = 	RTDMRxBufferLoLimit;		
	RTDMRxBufferEndMsgPointer = RTDMRxBufferLoLimit;
    return 0;
}



/******************************************************************************
* Function:     	${APP_NAME?upper_case}_RTDM_ProcessMsgs()
*
* Output:		return 0 if no errors
*
* Overview:	Here is where the RTDM code process the message received and then 
*			executes the required task. These tasks are reading an specified memory
*			location, writing an specified memory location, receive a communication
*			link sanity check command, or being asked for the size of the bufffers.
* 
*******************************************************************************/
int ${APP_NAME?upper_case}_RTDM_ProcessMsgs()
{
	
	
    //Local pointer management variables
	unsigned long int * RTDMpu32AddressTemp;
    unsigned long int RTDMpu32AddressTemp1;
    unsigned long int RTDMpu32AddressTemp2;
    unsigned long int RTDMpu32AddressTemp3;
    unsigned long int RTDMpu32AddressTemp4;
    char     * ByteSizePointer;
	unsigned char     * RTDMpucWrData;
    unsigned char     * RTDMpucRdData;
    unsigned char     * RTDMpucWrAddr;	
    unsigned short      RTDMNumBytes;
    unsigned char       RTDMPacketBuf[16];
   
	unsigned int        RTDMProcessMsgsTemp1, RTDMProcessMsgsTemp2;
	unsigned int        N;
  
    if (!RTDMFlags.MessageReceived)
	{
		return -1;
	}
  
   // Terminate function if EndMsgPointer  < startMSg Pointer
    if(RTDMRxBufferEndMsgPointer < RTDMRxBufferStartMsgPointer)
    {
        return -1;
    }
    
   
	RTDMcrcTemp = 
		${APP_NAME?upper_case}_RTDM_CumulativeCrc16
		(RTDMRxBufferStartMsgPointer, 
		(unsigned int)(RTDMRxBufferEndMsgPointer-RTDMRxBufferStartMsgPointer)+1, 
		0xFFFF);
		
	RTDMcrcTempH = (RTDMcrcTemp & 0xFF00)>>8;
	RTDMcrcTempL = RTDMcrcTemp & 0x00FF;
	RTDMProcessMsgsTemp1 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+2);
	RTDMProcessMsgsTemp2 = (unsigned int)*((RTDMRxBufferEndMsgPointer)+1);

	RTDMRxBufferStartMsgPointer +=2;
	if((RTDMProcessMsgsTemp1 == (unsigned)RTDMcrcTempH) && (RTDMProcessMsgsTemp2 == RTDMcrcTempL))
	  {

		switch(*((RTDMRxBufferLoLimit)+1))
		  {
		  case 'm':
			/*************** Extract Address **************/
			//Capture address as 32 bit quantity to match protocol definition. 
		    RTDMpu32AddressTemp1 = *(RTDMRxBufferStartMsgPointer);
			RTDMpu32AddressTemp2 = *(RTDMRxBufferStartMsgPointer+1);
            RTDMpu32AddressTemp2 = RTDMpu32AddressTemp2 << 8;
            RTDMpu32AddressTemp3 = *(RTDMRxBufferStartMsgPointer+2);
            RTDMpu32AddressTemp3 = RTDMpu32AddressTemp3 << 16;
            RTDMpu32AddressTemp4 = *(RTDMRxBufferStartMsgPointer+3);
            RTDMpu32AddressTemp4 = RTDMpu32AddressTemp4 << 24;
            RTDMpu32AddressTemp4 = RTDMpu32AddressTemp4 | 0x80000000; // Translating from Physical to Virtual address
            RTDMpu32AddressTemp = (unsigned long *)(RTDMpu32AddressTemp1 | RTDMpu32AddressTemp2 | RTDMpu32AddressTemp3 | RTDMpu32AddressTemp4);
           
            //Increment receive buffer pointer to length field.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
         
           //Init a byte oriented data pointer  
            RTDMpucRdData = (unsigned char *) (( unsigned int) RTDMpu32AddressTemp);
			
			
    
			/********* Extract Number of Bytes ***********/			
			//Capture address as 16 bit quantity to match protocol definition. 
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
            //Init the CRC seed for the cumulative checksum calculation. 					
			RTDMcrcTemp = 0xffff;
			
			//Add packet header prefix
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Add null terminator for putsUARTx function...
			RTDMPacketBuf[2] = 0; 
			
			//Calc header prefix checksum piece
			RTDMcrcTemp = ${APP_NAME?upper_case}_RTDM_CumulativeCrc16(RTDMPacketBuf, 2, RTDMcrcTemp);	
            
			//Calc data payload checksum
			RTDMcrcTemp = ${APP_NAME?upper_case}_RTDM_CumulativeCrc16(RTDMpucRdData, RTDMNumBytes, RTDMcrcTemp);			
			
            ByteSizePointer = (char *)(RTDMPacketBuf);
            while(*ByteSizePointer != '\0')
            {
                while (DRV_USART_TransmitBufferIsFull(RTDM_UART_HANDLE)); /* wait if the buffer is full */
                DRV_USART_WriteByte(RTDM_UART_HANDLE, *ByteSizePointer++); /* transfer data word to TX reg */  
            }
			/* Wait for  transmission to complete */
            while(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY & DRV_USART_TransferStatus(RTDM_UART_HANDLE)));
			
            
			//Send data portion of message... 
			while(RTDMNumBytes--)
			 {
				DRV_USART_WriteByte(RTDM_UART_HANDLE, *RTDMpucRdData++);
                /* Wait for  transmission to complete */
                while(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY & DRV_USART_TransferStatus(RTDM_UART_HANDLE)));
			 }
			
			//Add packet trailer   
		    RTDMPacketBuf[0] = '#';
            
		    RTDMcrcTemp = ${APP_NAME?upper_case}_RTDM_CumulativeCrc16(RTDMPacketBuf, 1, RTDMcrcTemp);			
		    
		    //Add checksum bytes to packet
			RTDMPacketBuf[1] = RTDMcrcTemp & 0x00FF;
			RTDMPacketBuf[2] = (RTDMcrcTemp & 0xFF00) >> 8;
			
			//Send packet trailer and checksum. 			
			for (N=0; N < 3; N++)
			{
				DRV_USART_WriteByte(RTDM_UART_HANDLE,RTDMPacketBuf[N]);
                /* Wait for  transmission to complete */
                while(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY & DRV_USART_TransferStatus(RTDM_UART_HANDLE)));
            }	
		  break;
		  
		  case 'M':
			{
			/*************** Extract Address **************/						
		    //Capture address as 32 bit quantity to match protocol definition. 
		  
            RTDMpu32AddressTemp1 = *(RTDMRxBufferStartMsgPointer);
			RTDMpu32AddressTemp2 = *(RTDMRxBufferStartMsgPointer+1);
            RTDMpu32AddressTemp2 = RTDMpu32AddressTemp2 << 8;
            RTDMpu32AddressTemp3 = *(RTDMRxBufferStartMsgPointer+2);
            RTDMpu32AddressTemp3 = RTDMpu32AddressTemp3 << 16;
            RTDMpu32AddressTemp4 = *(RTDMRxBufferStartMsgPointer+3);
            RTDMpu32AddressTemp4 = RTDMpu32AddressTemp4 << 24;
            RTDMpu32AddressTemp4 = RTDMpu32AddressTemp4 | 0x80000000; // Translating from Physical to Virtual address
            RTDMpu32AddressTemp = (unsigned long *)(RTDMpu32AddressTemp1 | RTDMpu32AddressTemp2 | RTDMpu32AddressTemp3 | RTDMpu32AddressTemp4);
           
          
           
            //Increment receive buffer pointer to length field.
		    RTDMRxBufferStartMsgPointer += sizeof(unsigned long);
			
		    //Init a byte oriented address pointer for use in incrementing 
			//through the address range properly as we write each byte of data
			//in the range (length) of this write request.   
			RTDMpucWrAddr = (unsigned char *) (( unsigned int) RTDMpu32AddressTemp);
            
                
         
			/********* Extract Number of Bytes ************/
			//Capture length as 16 bit quantity to match protocol definition.
			RTDMNumBytes = *((unsigned short *) RTDMRxBufferStartMsgPointer);
			
			//Increment receive buffer pointer to start of data payload.
			RTDMRxBufferStartMsgPointer += sizeof(unsigned short);
			
			/********** Extract Data ************/			
			//Init a byte oriented data pointer so that we can increment a byte at at 
			//time for as many bytes as are in the range for this write. 
			RTDMpucWrData = RTDMRxBufferStartMsgPointer;
			
			//*** Write Data in specified RAM location *****			
			//Important to increment through address range using byte oriented address and data
			//pointers. Otherwise, single byte or odd byte ranges do not get written correctly. 
			while(RTDMNumBytes--)
			  {
    			*RTDMpucWrAddr++ = *RTDMpucWrData++;
    		  }

			//Transmit OK message
			            
            ByteSizePointer = (char *)(RTDMWriteMemoryOK);
            while(*ByteSizePointer != '\0')
            {
                while (DRV_USART_TransmitBufferIsFull(RTDM_UART_HANDLE)); /* wait if the buffer is full */
                DRV_USART_WriteByte(RTDM_UART_HANDLE, *ByteSizePointer++); /* transfer data word to TX reg */  
            }
			/* Wait for  transmission to complete */
			while(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY & DRV_USART_TransferStatus(RTDM_UART_HANDLE)));
			break;
			}
		  case 's':
			{
			/* Load transmit buffer and transmit the same till null character is encountered */
			//Transmit OK message
			ByteSizePointer = (char *)(RTDMSanityCheckOK);
            while(*ByteSizePointer != '\0')
            {
                while (DRV_USART_TransmitBufferIsFull(RTDM_UART_HANDLE)); /* wait if the buffer is full */
                DRV_USART_WriteByte(RTDM_UART_HANDLE, *ByteSizePointer++); /* transfer data word to TX reg */  
            }
			/* Wait for  transmission to complete */
			while(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY & DRV_USART_TransferStatus(RTDM_UART_HANDLE)));
		    break;
			}
		  
		  case 'L':
		    RTDMcrcTemp = 0xffff; //Init the CRC seed.
			
			RTDMPacketBuf[0] = '+';
			RTDMPacketBuf[1] = '$';
			//Size of the RTDM Receive buffer.
			RTDMPacketBuf[2] = (sizeof(RTDMRxBuffer) & 0x00FF);
			RTDMPacketBuf[3] = (sizeof(RTDMRxBuffer) & 0xFF00) >> 8;
			//Note: We dod not utilize a transmit buffer since any data memory source is 
			//essentially already buffered. So the transmit limit is now just a way to 
			//limit the total message length that a client make with any single read request.
			RTDMPacketBuf[4] = (RTDM_MAX_XMIT_LEN & 0x00FF);
			RTDMPacketBuf[5] = (RTDM_MAX_XMIT_LEN & 0xFF00) >> 8;
			RTDMPacketBuf[6] = '#';		
           
			RTDMcrcTemp = ${APP_NAME?upper_case}_RTDM_CumulativeCrc16(RTDMPacketBuf, 7, RTDMcrcTemp);			
			RTDMPacketBuf[7] = (RTDMcrcTemp & 0x00FF);
			RTDMPacketBuf[8] = (RTDMcrcTemp & 0xFF00) >> 8;

			//Send completed message which is 9 bytes in length.
			for (N=0; N < 9; N++)
			{
				DRV_USART_WriteByte(RTDM_UART_HANDLE,RTDMPacketBuf[N]);
				while(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY & DRV_USART_TransferStatus(RTDM_UART_HANDLE)));
			}	
			break;

		  default:
			// ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
			//Transmit ERROR message 1
             ByteSizePointer = (char *)(RTDMErrorIllegalFunction);
            while(*ByteSizePointer != '\0')
            {
                while (DRV_USART_TransmitBufferIsFull(RTDM_UART_HANDLE)); /* wait if the buffer is full */
                DRV_USART_WriteByte(RTDM_UART_HANDLE, *ByteSizePointer++); /* transfer data word to TX reg */  
            }
			
			/* Wait for  transmission to complete */
			while(!(DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY & DRV_USART_TransferStatus(RTDM_UART_HANDLE)));
			break;
		  }
				   
	  }

	  memset(&RTDMRxBuffer, 0, sizeof(RTDMRxBuffer));
	  
	  RTDMFlags.MessageReceived = 0;
	  RTDMRxBufferIndex             = RTDMRxBufferLoLimit;
	  RTDMRxBufferStartMsgPointer   = RTDMRxBufferLoLimit;		
	  RTDMRxBufferEndMsgPointer     = RTDMRxBufferLoLimit;
	  
	  return 0;
}	



 
 
//calculation with pre-calculated polyniomial values to speed-up 
//checksum calculation time. 

 unsigned int crc_16_tab[] = {
  0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
  0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
  0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
  0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
  0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
  0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
  0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
  0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
  0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
  0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
  0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
  0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
  0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
  0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
  0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
  0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
  0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
  0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
  0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
  0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
  0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
  0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
  0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
  0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
  0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
  0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
  0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
  0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
  0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
  0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
  0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
  0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

/******************************************************************************
* Function:     	${APP_NAME?upper_case}_RTDM_CumulativeCrc16 
				(unsigned char *buf, 
				unsigned int u16Length, 
				unsigned int u16CRC)
*
* Output:		return CRC16
*
* Overview:	This routine calculates the polynomial for the checksum byte using
*			a coefficients table. 
*			This approach has a faster performance but consumes a higher amount of 
*			program memory
*
* Note:		Some processors may have more UART modules, that is why it is required to
*			specify which UART module is going to be used by RTDM	
*******************************************************************************/
unsigned int  ${APP_NAME?upper_case}_RTDM_CumulativeCrc16(unsigned char *buf, unsigned int bsize, unsigned int crcSeed)
{
   unsigned char * pData = buf;

   while (bsize--)
    {
   		crcSeed = (unsigned int)(crcSeed >> 8) ^ crc_16_tab[(crcSeed ^ *pData++) & 0xff];
    }		
   
  return crcSeed;

}

void  ${APP_NAME?upper_case}_RTDMRXInterruptTasks()
{
   while(DRV_USART_TRANSFER_STATUS_RECEIVER_DATA_PRESENT & DRV_USART_TransferStatus(RTDM_UART_HANDLE))
    {
      *(RTDMRxBufferIndex++) = DRV_USART_ReadByte(RTDM_UART_HANDLE);		
      
    }
   
 
	RTDMRxBufferEndMsgPointer = RTDMRxBufferIndex-3;
	if(RTDMRxBufferIndex > (RTDMRxBufferHiLimit-1))
	  {
	  RTDMRxBufferIndex = RTDMRxBufferLoLimit;
    
    
	  RTDMRxBufferEndMsgPointer = RTDMRxBufferHiLimit-1;
  
	  }

  
        if(*(RTDMRxBufferStartMsgPointer) == '$')
        {
          if(*(RTDMRxBufferEndMsgPointer) == '#')
          {
            RTDMFlags.MessageReceived = 1;
           
          }
        }  
        else
        {
            RTDMRxBufferIndex = RTDMRxBufferLoLimit;
           
        }
}


</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->
<#macro macro_rtdm_app_c_initialize>
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_rtdm_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            
			
			bool appInitialized = true;
-->   
<#macro macro_rtdm_app_c_tasks_state_init>
			${APP_NAME?upper_case}_RTDM_Start();	//RTDM start function
			DMCIFlags.Recorder =1; 					// Enable Recording into Recorder Buffer
</#macro>  

<#--        
            if (appInitialized)
            {
-->
<#macro macro_rtdm_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_PROCESS_MSGS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_PROCESS_MSGS:
        {
-->
<#macro macro_rtdm_app_c_tasks_state_service_tasks>
			${APP_NAME?upper_case}_RTDM_ProcessMsgs();	//RTDM process incoming and outgoing messages
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

<#macro macro_rtdm_app_c_tasks_states>
</#macro>
