/*******************************************************************************
  MIIM Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_miim_local.h

  Summary:
    MIIM driver local declarations and definitions.

  Description:
    This file contains the MIIM driver's local declarations and definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_MIIM_MAPPING_H
#define _DRV_MIIM_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************
   static const short _MIIMClockDivisorTable[]=
    {
        4, 6, 8, 10, 14, 20, 28, 40, 
#if defined (__PIC32MZ__)
        48, 50,
#endif  // defined (__PIC32MZ__)
    };  // divider values for the Host clock
   
    static  __inline__ DRV_MIIM_RESULT __attribute__((always_inline))_DRV_MIIM_ETH_ENABLE(uintptr_t ethphyId)
    {
        DRV_MIIM_RESULT res = DRV_MIIM_RES_OK;
        if(!PLIB_ETH_IsEnabled(ethphyId)) 
        { 
            PLIB_ETH_Enable(ethphyId);  
            res = DRV_MIIM_RES_INIT_WARNING;
        } 
        return res;
    }
    

    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_MII_RELEASE_RESET(uintptr_t ethphyId)
    {
        PLIB_ETH_MIIResetDisable(ethphyId);
    }
    
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_SETUP_PERAMBLE(uintptr_t ethphyId, const DRV_MIIM_SETUP* pSetUp)
    {
        if((pSetUp->setupFlags & DRV_MIIM_SETUP_FLAG_PREAMBLE_SUPPRESSED) != 0)
        {
            PLIB_ETH_MIIMNoPreEnable(ethphyId);
        }
        else
        {
            PLIB_ETH_MIIMNoPreDisable(ethphyId);
        }
    }
    
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_SCAN_INCREMENT(uintptr_t ethphyId, const DRV_MIIM_SETUP* pSetUp)
    {
        if((pSetUp->setupFlags & DRV_MIIM_SETUP_FLAG_SCAN_ADDRESS_INCREMENT) != 0)
        {
            PLIB_ETH_MIIMScanIncrEnable(ethphyId);
        }
        else
        {
            PLIB_ETH_MIIMScanIncrDisable(ethphyId);
        }
    }

    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_MNGMNT_PORT_ENABLE(uintptr_t ethphyId)
    {
       
    }
    
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_MNGMNT_PORT_DISABLE(uintptr_t ethphyId)
    {
       
    }
    
    static  __inline__ bool __attribute__((always_inline))_DRV_MIIM_IS_BUSY(uintptr_t ethphyId)
    {
        return PLIB_ETH_MIIMIsBusy(ethphyId);
    }
    
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_PHYADDR_SET(uintptr_t ethphyId,DRV_MIIM_OP_DCPT* pOpDcpt)
    {
       PLIB_ETH_PHYAddressSet(ethphyId, pOpDcpt->phyAdd);
       PLIB_ETH_RegisterAddressSet(ethphyId,pOpDcpt->regIx);
    }
                                    
    static  __inline__ void __attribute__((always_inline))PLIB_ETH_ClearDataValid(uintptr_t ethphyId)
    {
        volatile eth_registers_t * eth = ((eth_registers_t *)(ethphyId));	
        eth->EMACxMINDSET = _EMACxMIND_NOTVALID_MASK;
    }
    
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_CLEAR_DATA_VALID(uintptr_t ethphyId)
    {
        PLIB_ETH_ClearDataValid(ethphyId);
    }
   
    static  __inline__ DRV_MIIM_TXFER_STAT __attribute__((always_inline))_DRV_MIIM_OP_SCAN_ENABLE(uintptr_t ethphyId)
    {        
        PLIB_ETH_ClearDataValid(ethphyId);
        PLIB_ETH_MIIMScanModeEnable(ethphyId);
        return DRV_MIIM_TXFER_SCAN_STALE;
    }
    
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_OP_WRITE_DATA(uintptr_t ethphyId,DRV_MIIM_OP_DCPT* pOpDcpt)
    {
        PLIB_ETH_MIIMWriteDataSet(ethphyId, pOpDcpt->opData);
    }
    

    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_WRITE_START(uintptr_t ethphyId)
    {
        PLIB_ETH_MIIMWriteStart(ethphyId);
    }
    
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_OP_READ_START(uintptr_t ethphyId,DRV_MIIM_OP_DCPT* pOpDcpt)
    {
        PLIB_ETH_MIIMReadStart(ethphyId);
    }

    static  __inline__ uint16_t __attribute__((always_inline))_DRV_MIIM_OP_READ_DATA_GET(uintptr_t ethphyId)
    {
        return PLIB_ETH_MIIMReadDataGet(ethphyId);
    }
    
    static  __inline__ bool __attribute__((always_inline))_DRV_MIIM_IS_DATA_VALID(uintptr_t ethphyId)
    {
        return !PLIB_ETH_DataNotValid(ethphyId);
    }
      
    static  __inline__ void __attribute__((always_inline))_DRV_MIIM_SCAN_DIABLE(uintptr_t ethphyId)
    {    
        PLIB_ETH_MIIMScanModeDisable(ethphyId);
    }
    
    static  __inline__ void __attribute__((always_inline)) _DRV_MIIM_SMI_CLOCK_SET(uintptr_t ethphyId, uint32_t hostClock, uint32_t maxMIIMClock )
    {
        int  ix;

        PLIB_ETH_MIIMResetEnable(ethphyId); // Issue MIIM reset
        PLIB_ETH_MIIMResetDisable(ethphyId); // Clear MIIM reset

        for(ix = 0; ix < sizeof(_MIIMClockDivisorTable) / sizeof(*_MIIMClockDivisorTable); ix++)
        {
            if(hostClock / _MIIMClockDivisorTable[ix] <= maxMIIMClock)
            {   // found it
                break;
            }
        }

        if(ix == sizeof(_MIIMClockDivisorTable) / sizeof(*_MIIMClockDivisorTable))
        {
            ix--;   // max divider; best we can do
        }

        PLIB_ETH_MIIMClockSet(ethphyId, ix + 1);  // program the clock*/
    }
#endif //#ifndef _DRV_MIIM_MAPPING_H

/*******************************************************************************
 End of File
*/

